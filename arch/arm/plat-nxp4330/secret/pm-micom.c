/*!
 * @date     2014/03/15
 * @id       " "
 * @version  v1.0.0
 * @brief    
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pm.h>
#include <mach/platform.h>
#include <mach/soc.h>
#include <mach/pm.h>
#include <linux/suspend.h>
#include <linux/mfd/nxe2000.h>

#define DEVICE_NAME	"pm-micom"
#define DEVICE_ADDR	0x30
#define DEVICE_BUS 	3

#define MAX_RETRY_I2C_XFER 				(100)

#define MICOM_REG_NEXE_PWR_STATE		0x00
#define MICOM_REG_CURRENT_PWR_STATE		0x01
#define MICOM_REG_INT_SOURCE			0x02
#define MICOM_REG_IR_VALUE				0x03
#define MICOM_REG_MCU_FW_VER			0x04
#define MICOM_REG_MCU_TIMER_CTL			0x05
#define MICOM_REG_BT_CTL				0x06

#define MICOM_CMD_PWR_OFF				0x00
#define MICOM_CMD_PWR_SUSPEND_FLASH		0x10
#define MICOM_CMD_PWR_SUSPEND_RAM		0x11
#define MICOM_CMD_PWR_RUN				0x80

#define MICOM_PWRSTATE_OFF				0x00
#define MICOM_PWRSTATE_SUSPEND_FLASH	0x10
#define MICOM_PWRSTATE_SUSPEND_RAM		0x11
#define MICOM_PWRSTATE_RUN				0x0F

#define MICOM_INTSRC_SENSOR_INT			0x01
#define MICOM_INTSRC_LOWBAT_DET			0x02
#define MICOM_INTSRC_PMIC_INT			0x04
#define MICOM_INTSRC_TIMER_INT			0x08
#define MICOM_INTSRC_VUSB_DET			0x10
#define MICOM_INTSRC_BT_HOST_WAKE		0x40
//#define MICOM_INTSRC_WL_HOST_WAKE		0x80

#define MICOM_BT_REG_ON					(1<<3)
#define MICOM_BT_DEV_WAKEUP				(1<<5)

/*
 * Bluetooth BCM struct
 */
enum bt_ctl_type {
	BT_TYPE_POWER		= 1,
	BT_TYPE_WAKE_DEVICE,
	BT_TYPE_WAKE_HOST,
};

struct micom_data {
	struct i2c_client *client;
};

static struct drvPXMICOM {
	struct class* class;
	struct device* device;
	dev_t version;
	struct i2c_client* client;
	struct semaphore sem;
	struct cdev cdev;
} *drvPXMICOM;

static struct i2c_client *mi_client = NULL;

extern void (*pm_power_off)(void);
static void (*nxe2000_pm_power_off)(void);

int hib_enable;
void micom_pm_hibernation_off(void)
{
	int ret;
	uint8_t reg_val = 0;

	printk("%s enter, hib %d\n", __func__, hib_enable);

#if defined(CONFIG_BATTERY_NXE2000)
	reg_val = g_soc;
	reg_val &= 0x7f;

	ret = nxe2000_pm_write(NXE2000_PSWR, reg_val);
	if (ret < 0)
		pr_err("Error in writing PSWR_REG\n");

	if (g_fg_on_mode == 0) {
		/* Clear NXE2000_FG_CTRL 0x01 bit */
		ret = nxe2000_pm_read(NXE2000_FG_CTRL, &reg_val);
		if (reg_val & 0x01) {
			reg_val &= ~0x01;
			ret = nxe2000_pm_write(NXE2000_FG_CTRL, reg_val);
		}
		if (ret < 0)
			pr_err("Error in writing FG_CTRL\n");
	}
#endif

	/* set rapid timer 300 min */
	ret = nxe2000_pm_read(TIMSET_REG, &reg_val);
	if (ret < 0)
		ret = nxe2000_pm_read(TIMSET_REG, &reg_val);

	reg_val |= 0x03;

	ret = nxe2000_pm_write(TIMSET_REG, reg_val);
	if (ret < 0)
		ret = nxe2000_pm_write(TIMSET_REG, reg_val);
	if (ret < 0)
		pr_err("Error in writing the TIMSET_Reg\n");

	/* Disable all Interrupt */
	ret = nxe2000_pm_write(NXE2000_INTC_INTEN, 0);
	if (ret < 0)
		ret = nxe2000_pm_write(NXE2000_INTC_INTEN, 0);

	/* Not repeat power ON after power off(Power Off/N_OE) */
	ret = nxe2000_pm_write(NXE2000_PWR_REP_CNT, 0x0);
	if (ret < 0)
		ret = nxe2000_pm_write(NXE2000_PWR_REP_CNT, 0x0);

	if(hib_enable) {
		/* [START] set Suspend to FLASH mode */
		ret = nxe2000_pm_read(NXE2000_PWR_DC4_SLOT, &reg_val);
		if (ret < 0)
			ret = nxe2000_pm_read(NXE2000_PWR_DC4_SLOT, &reg_val);
	
		reg_val &= 0xf0;
	
		ret = nxe2000_pm_write(NXE2000_PWR_DC4_SLOT, reg_val);
		/* [END] set Suspend to FLASH mode */

		nxp_cpu_goto_stop();
	}
	else {
		/* Power OFF */
		ret = nxe2000_pm_write(NXE2000_PWR_SLP_CNT, 0x1);
		if (ret < 0)
			ret = nxe2000_pm_write(NXE2000_PWR_SLP_CNT, 0x1);
	}

    nxp_cpu_shutdown();

}

static int micom_smbus_read_byte(struct i2c_client *client, unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	int i;

	for(i=0; i<3; i++)
	{
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0)
		{
			mdelay(10);
		}
		else
		{
			*data = dummy & 0x000000ff;
			return 0;
		}
	}

	dev_err(&client->dev, "%s: fail!!! \n", __func__);
	PM_DBGOUT("%s: fail!!! \n", __func__);
	return -1;
}

static int micom_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char data)
{
	s32 dummy;
	int i;

	for(i=0; i<3; i++)
	{
		dummy = i2c_smbus_write_byte_data(client, reg_addr, data);
		if (dummy < 0)
		{
			mdelay(10);
		}
		else
		{
			return 0;
		}
	}

	dev_err(&client->dev, "%s: fail!!! \n", __func__);
	PM_DBGOUT("%s: fail!!! \n", __func__);
	return -1;
}

#if 0
static int micom_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
	{

		dev_err(&client->dev, "%s: fail!!! \n", __func__);
		PM_DBGOUT("%s: fail!!! \n", __func__);
		return -1;
	}
	return 0;
}

static int bma_i2c_burst_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u16 len)
{
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < MAX_RETRY_I2C_XFER ; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(1);
	}

	if (MAX_RETRY_I2C_XFER  <= retry) {
		dev_err(&client->dev, "%s: fail!!! \n", __func__);
		PM_DBGOUT("%s: fail!!! \n", __func__);
		return -EIO;
	}

	return 0;
}
#endif

static ssize_t micom_suspend_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data=0;
	//struct i2c_client *client = to_i2c_client(dev);

	return sprintf(buf, "%d\n", data);
}

static ssize_t micom_suspend_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	//struct i2c_client *client = to_i2c_client(dev);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	return count;
}
static DEVICE_ATTR(suspend, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH,
		micom_suspend_show, micom_suspend_store);

static struct attribute *micom_attributes[] = {
	&dev_attr_suspend.attr,
	NULL
};

static struct attribute_group micom_attribute_group = {
	.attrs = micom_attributes
};

int micom_bt_cmd(int sub, int on)
{
	int ret = 0;
	unsigned char value=0;
	struct micom_data *data = i2c_get_clientdata(mi_client);

	micom_smbus_read_byte(data->client, MICOM_REG_BT_CTL, &value);
	switch (sub)
	{
		case BT_TYPE_POWER:	
			if(on)
				value |= MICOM_BT_REG_ON;
			else
				value &= ~MICOM_BT_REG_ON;
			break;

		case BT_TYPE_WAKE_DEVICE:
			if(on)
				value |= MICOM_BT_DEV_WAKEUP;
			else
				value &= ~MICOM_BT_DEV_WAKEUP;
			break;

		default:
			printk(KERN_ERR "micom_bt_cmd: unkonwn bt control type ...\n");
			return -1;
	}		
	ret = micom_smbus_write_byte(data->client, MICOM_REG_BT_CTL, value);
	return ret;
}
EXPORT_SYMBOL(micom_bt_cmd);


static int micom_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
//	int value_gpio = 0;
	unsigned char value=0;
	struct micom_data *data;

	data = kzalloc(sizeof(struct micom_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	i2c_set_clientdata(client, data);
	data->client = client;
	mi_client = client;
//	value_gpio = gpio_get_value(CFG_MSP_READ);
//	printk(KERN_ERR "CFG_MSP_READ=[%d]\n", value_gpio);

	err=micom_smbus_read_byte(client, MICOM_REG_NEXE_PWR_STATE, &value);
	printk(KERN_INFO "%s() : 0x00(PWR ) : 0x%02x \n", __func__, value);
	err=micom_smbus_read_byte(client, MICOM_REG_CURRENT_PWR_STATE, &value);
	printk(KERN_INFO "%s() : 0x01(PWR ) : 0x%02x \n", __func__, value);
	err=micom_smbus_read_byte(client, MICOM_REG_INT_SOURCE, &value);
	printk(KERN_INFO "%s() : 0x02(INT ) : 0x%02x \n", __func__, value);
	err=micom_smbus_read_byte(client, MICOM_REG_IR_VALUE, &value);
	printk(KERN_INFO "%s() : 0x03(IR  ) : 0x%02x \n", __func__, value);
	err=micom_smbus_read_byte(client, MICOM_REG_MCU_FW_VER, &value);
	printk(KERN_INFO "%s() : 0x04(FW  ) : 0x%02x \n", __func__, value);
	err=micom_smbus_read_byte(client, MICOM_REG_MCU_TIMER_CTL, &value);
	printk(KERN_INFO "%s() : 0x05(TIME) : 0x%02x \n", __func__, value);
	err=micom_smbus_read_byte(client, MICOM_REG_BT_CTL, &value);
	printk(KERN_INFO "%s() : 0x06(BT  ) : 0x%02x \n", __func__, value);

	err = sysfs_create_group(&client->dev.kobj, &micom_attribute_group);
	if (err < 0)
		goto error_sysfs;

	return 0;

error_sysfs:
	kfree(data);
exit:
	return err;
}

int _pm_check_wakeup_dev(char *dev, int io) 
{
	printk("chekc wakeup dev:%s io[%d]\n", dev, io);
	return 1;
}

static int __devexit micom_remove(struct i2c_client *client)
{
	struct micom_data *data = i2c_get_clientdata(client);

    PM_DBGOUT("+%s\n", __func__);

	sysfs_remove_group(&client->dev.kobj, &micom_attribute_group);
	kfree(data);

    PM_DBGOUT("-%s\n", __func__);
	return 0;
}

static void micom_shutdown(void)
{
	struct micom_data *data = i2c_get_clientdata(mi_client);

    PM_DBGOUT("+%s\n", __func__);

	micom_smbus_write_byte(data->client, MICOM_REG_NEXE_PWR_STATE, MICOM_CMD_PWR_OFF);

    PM_DBGOUT("-%s\n", __func__);
	return;
}

#if 0//def CONFIG_PM

static int micom_suspend(struct i2c_client *client, pm_message_t mesg)
{
	
	struct micom_data *data = i2c_get_clientdata(client);
	int ret = 0;
    PM_DBGOUT("+%s\n", __func__);
	
	ret=micom_smbus_write_byte(data->client, MICOM_REG_NEXE_PWR_STATE, MICOM_CMD_PWR_SUSPEND_RAM);
	
    PM_DBGOUT("-%s, ret:%d\n", __func__, ret);
	return 0;
}

static int micom_resume(struct i2c_client *client)
{
//	int err = 0;
//	unsigned char value=0;

//	struct micom_data *data = i2c_get_clientdata(client);
    PM_DBGOUT("+%s\n", __func__);

	//micom_smbus_write_byte(data->client, MICOM_REG_NEXE_PWR_STATE, MICOM_CMD_PWR_RUN);

    PM_DBGOUT("-%s\n", __func__);
	return 0;
}

#else

#define micom_suspend		NULL
#define micom_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id micom_id[] = {
	{ DEVICE_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, micom_id);

static struct i2c_driver micom_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DEVICE_NAME,
	},
	.suspend	= micom_suspend,
	.resume		= micom_resume,
	.id_table	= micom_id,
	.probe		= micom_probe,
	.remove		= __devexit_p(micom_remove)
};	

static struct i2c_board_info info =
{
    I2C_BOARD_INFO(DEVICE_NAME, DEVICE_ADDR),
};

static ssize_t drvPXMICOM_read(struct file* filp, char* buff, size_t length, loff_t* offset)
{
    buff[0] = 1;
    return 1;
}

static ssize_t drvPXMICOM_write(struct file* filp, const char* buff, size_t len, loff_t* off)
{
	struct micom_data *data = i2c_get_clientdata(mi_client);
	micom_smbus_write_byte(data->client, buff[0], buff[1]);
	return 1;
}

static struct file_operations fops =
{
    .read = drvPXMICOM_read,
    .write = drvPXMICOM_write
};

static void micom_pm_poweroff(void)
{
	int ret = 0;
	struct micom_data *data = i2c_get_clientdata(mi_client);
	ret=micom_smbus_write_byte(data->client, MICOM_REG_NEXE_PWR_STATE, MICOM_CMD_PWR_SUSPEND_RAM);
	return;
}

static struct board_suspend_ops micom_pm_ops = {
	.poweroff	= micom_pm_poweroff
};

static int __init micom_init(void)
{
	int reval = -ENOMEM;
	int result;
	struct i2c_adapter* adapter;
	struct i2c_client* client;

	//printk(KERN_ALERT"+%s\n", __func__);
	pm_power_off_prepare = micom_shutdown;
	nxp_check_pm_wakeup_dev = _pm_check_wakeup_dev;
	
    drvPXMICOM = kmalloc(sizeof *drvPXMICOM, GFP_KERNEL);
	if(!drvPXMICOM)
	{
		printk(KERN_ALERT"pm-micom.c: cannot allocate memory for drvPXMICOM driver\n");
	}
    adapter = i2c_get_adapter(DEVICE_BUS);
	if(!adapter)
	{
		printk(KERN_ALERT"pm-micom.c: cannot get adapter\n");
	}
    client = i2c_new_device(adapter, &info);
	if(!client)
	{
		printk(KERN_ALERT"(%s-%s():%4d): Cannot create new device \n", __FILE__, __func__, __LINE__);
	}else{
		printk(KERN_ALERT"(%s-%s():%4d): Create new device \n", __FILE__, __func__, __LINE__);
	}

    drvPXMICOM->version = MKDEV(0,0);
    reval = alloc_chrdev_region(&drvPXMICOM->version, 0, 1, DEVICE_NAME);
	if(reval < 0)
	{
		printk(KERN_ALERT"pm-micom.c: error getting major number %d\r\n", reval);
	}

    drvPXMICOM->class = class_create(THIS_MODULE, DEVICE_NAME);
	if(!drvPXMICOM->class)
	{
		printk(KERN_ALERT"pm-micom.c: error creating class\n");
	}

    drvPXMICOM->device = device_create(drvPXMICOM->class, NULL, drvPXMICOM->version, NULL, DEVICE_NAME);
	if(!drvPXMICOM->device)
	{
		printk(KERN_ALERT"pm-micom.c: error creating device\n");
	}

    cdev_init(&drvPXMICOM->cdev, &fops);
    drvPXMICOM->cdev.owner = THIS_MODULE;
    drvPXMICOM->cdev.ops = &fops;
    reval = cdev_add(&drvPXMICOM->cdev, drvPXMICOM->version, 1);

	if(reval)
	{
		printk(KERN_ALERT"pm-micom.c: fail to add cdev\n");
	}
//	return i2c_add_driver(&micom_driver);

	result = i2c_add_driver(&micom_driver);
	printk(KERN_ALERT"pm-micom.c: i2c_add_driver return : %d\r\n", result);	

	nxp_board_suspend_register(&micom_pm_ops);

	nxe2000_pm_power_off = pm_power_off;
	pm_power_off = micom_pm_hibernation_off;
	hib_enable = 0;
	
	return result;
}

static void __exit micom_exit(void)
{
    device_destroy(drvPXMICOM->class, drvPXMICOM->version);
    class_destroy(drvPXMICOM->class);
    unregister_chrdev_region(drvPXMICOM->version, 1);
    i2c_unregister_device(drvPXMICOM->client);
    kfree(drvPXMICOM);
	i2c_del_driver(&micom_driver);
    PM_DBGOUT("-%s\n", __func__);
}

module_init(micom_init);
module_exit(micom_exit);

MODULE_AUTHOR(" < @nexell.co.kr>");
MODULE_DESCRIPTION("micom driver");
MODULE_LICENSE("GPL");
