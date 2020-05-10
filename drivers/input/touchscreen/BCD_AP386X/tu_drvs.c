/* capacivite multi-touch device driver.*/


//#define DEBUG

//#define DEBUG_PRINT
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/slab.h>

#include <linux/delay.h>
#include <mach/soc.h>
// #include "tu_drvs.h"
#include <plat/tu_drvs.h>

static int fwVersion[4] = {0}; /* used to store fw version x.x.x.x */

static int revert_x = 0;
static int revert_y = 0;
static int report_pressure = 40; // 40 = default pressure to report
static char *fw_version = "0x0";

module_param(revert_x, bool, 0644);
MODULE_PARM_DESC(revert_x, "Revert x-axis. (0=disable, 1=enable)");
module_param(revert_y, bool, 0644);
MODULE_PARM_DESC(revert_y, "Revert y-axis. (0=disable, 1=enable)");
module_param(report_pressure, short, 0644);
MODULE_PARM_DESC(report_pressure, "Pressure value to report to input-event system. Range: 0-255, 0=raw, default=40");
static int report_rate = 60;
static __u64 next_touch_ns = 0;
module_param(report_rate, short, 0644);
MODULE_PARM_DESC(report_rate, "Throttle touch report rate (default:60)");
module_param(fw_version, charp, 0444);
MODULE_PARM_DESC(fw_version, "Displays firmware version");

//SZSZ struct undefine bypass for now
#undef CONFIG_PM


#define TU_DRIVER_NAME 		TU_I2C_NAME
#define MAX_TRACKING_ID 	POINT_STRUCT_SIZE

#define COORD_INTERPRET(MSB_BYTE, LSB_BYTE) \
		(MSB_BYTE << 8 | LSB_BYTE)

struct tu_data{
	__u16 	x, y, w, id;
	struct i2c_client *client;
	struct input_dev *dev;
	struct mutex lock;
	int irq;
	struct work_struct work;
#ifdef CONFIG_PM
	struct early_suspend early_suspend;
#endif
};

static struct i2c_client *g_i2c_client;

#ifdef CONFIG_PM
static void tu_early_suspend(struct early_suspend *h);
static void tu_late_resume(struct early_suspend *h);
#endif


static float x_scale = (AA_X_SIZE/480.0);
static float y_scale = (AA_Y_SIZE/272.0);

static void tu_touch_enable(struct tu_data *tu) {
	u_int8_t ret = 0;
	u_int8_t Cmdbuf[16]; //Command Buffer to write

	Cmdbuf[0] = 0x0E;
	Cmdbuf[1] = 0x01;
	Cmdbuf[2] = 0x00;
	Cmdbuf[3] = 0x00;

	ret = i2c_smbus_write_i2c_block_data(tu->client, 0, 4, Cmdbuf);

	return;
}

static void tu_touch_disable(struct tu_data *tu) {
	u_int8_t ret = 0;
	u_int8_t Cmdbuf[16]; //Command Buffer to write

	Cmdbuf[0] = 0x0E;
	Cmdbuf[1] = 0x03;
	Cmdbuf[2] = 0x00;
	Cmdbuf[3] = 0x00;

	ret = i2c_smbus_write_i2c_block_data(tu->client, 0, 4, Cmdbuf);

	return;
}

/* tu_read_version()
 * 	stores fw version into global fwVersion array
 */
static void tu_read_version(struct tu_data *tu) {
	u_int8_t ret = 0;
	u_int8_t Cmdbuf[16]; //Command Buffer to write
	u_int8_t Recbuf [16];//Receive Buffer to Read
	u_int8_t Verbuf [16];

	Cmdbuf[0] = 0x0C;
	Cmdbuf[1] = 0x19;
	Cmdbuf[2] = 0x00;
	Cmdbuf[3] = 0x00;

	ret = i2c_smbus_write_i2c_block_data(tu->client, 0, 4, Cmdbuf);

	mdelay(5);

	//Read Data from Touch IC
	ret = i2c_smbus_read_i2c_block_data(tu->client, 0, 8, Recbuf);

	Verbuf[0] = Recbuf[4]; //Project Byte 0
	Verbuf[1] = Recbuf[5]; //Project Byte 1
	Verbuf[2] = Recbuf[6]; //Version Byte 0
	Verbuf[3] = Recbuf[7]; //Version Byte 1

	//Print Version
	printk("Version : %4x %4x %4x %4x\n", Verbuf[0], Verbuf[1], Verbuf[2], Verbuf[3]);

	int i;
	for (i=0; i<4; i++)
		fwVersion[i] = Verbuf[i];

	mdelay(5);

	tu_touch_enable(tu);

	return;
}


/*SZ Unused but compile okay
static int __tu_reg_write(struct tu_data *tu, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(tu->client, reg, value);
}


static int reg_write(struct tu_data *tu, u_int8_t reg, u_int8_t val)
{
	int ret;

	mutex_lock(&tu->lock);
	ret = __tu_reg_write(tu, reg, val);
	mutex_unlock(&tu->lock);

	return ret;
}
static int __tu_reg_read(struct tu_data *tu, u8 reg)
{
	return i2c_smbus_read_byte_data(tu->client, reg);
}

static u_int8_t reg_read(struct tu_data *tu, u_int8_t reg)
{
	int32_t ret;

	mutex_lock(&tu->lock);
	ret = __tu_reg_read(tu, reg);
	mutex_unlock(&tu->lock);

	return ret & 0xff;
}

static int reg_set_bit_mask(struct tu_data *tu,
			    u_int8_t reg, u_int8_t mask, u_int8_t val)
{
	int ret;
	u_int8_t tmp;

	val &= mask;

	mutex_lock(&tu->lock);

	tmp = __tu_reg_read(tu, reg);
	tmp &= ~mask;
	tmp |= val;
	ret = __tu_reg_write(tu, reg, tmp);

	mutex_unlock(&tu->lock);

	return ret;
}
*/
static struct i2c_driver tu_driver;

static irqreturn_t tu_irq(int irq, void *_tu)
{
	struct tu_data *tu = _tu;
	
	// MUTEX LOCK HERE FOR IRQ?

	schedule_work(&tu->work);
	return IRQ_HANDLED;
}

static inline void tu_report(struct tu_data *tu, u_int8_t touch_num)
{
	if (touch_num == 0) {
		struct timespec ts;
		__u64 curr_ns;
		ktime_get_ts(&ts);
		curr_ns = timespec_to_ns(&ts);
		if(curr_ns < next_touch_ns)
			return;
		if(report_rate < 1)
			report_rate = 1;
		if(report_rate > 100)
			report_rate = 100;
		next_touch_ns = curr_ns + NSEC_PER_SEC / report_rate;

		input_report_key(tu->dev, BTN_TOUCH, 1);
	    //SZSZ
		input_report_abs(tu->dev, ABS_X, tu->x/x_scale);
		input_report_abs(tu->dev, ABS_Y, tu->y/y_scale);
		input_report_abs(tu->dev, ABS_PRESSURE, report_pressure);
	}

	input_report_abs(tu->dev, ABS_MT_TOUCH_MAJOR, tu->w);
	input_report_abs(tu->dev, ABS_MT_POSITION_X, tu->x/x_scale);
	input_report_abs(tu->dev, ABS_MT_POSITION_Y, tu->y/y_scale);
	input_report_abs(tu->dev, ABS_MT_TRACKING_ID, tu->id);

	input_mt_sync(tu->dev);
}

static void tu_i2c_work(struct work_struct *work)
{
	u_int8_t i;

	u_int8_t idx_x_low;
	u_int8_t idx_x_hi;
	u_int8_t idx_y_low;
	u_int8_t idx_y_hi;
	u_int8_t idx_id_st;

	u_int8_t touchcnt;

	//SZSZ conflict static u_int8_t prev_key = 0;


	struct tu_data *tu =
			container_of(work, struct tu_data, work);
	u_int8_t ret = 0;
	//u_int8_t read_buf[REPORT_BUF_SIZE+27];
	u_int8_t read_buf[REPORT_BUF_SIZE];
	static unsigned int prev_key = 0;

	//I2C Read Data
	ret = i2c_smbus_read_i2c_block_data(tu->client,
							0x00, REPORT_BUF_SIZE, read_buf);

#ifdef DEBUG_PRINT
	for( i=0; i<REPORT_BUF_SIZE; i++ )
	{
		printk( "%2x ", read_buf[i] );
	}
//	printk( "\n" );
#endif

	if(read_buf[TU_RMOD] == 0xb2)
	{
		switch (read_buf[TU_KEY_CODE]) 
		{
		/* SZSZ undefined - now defined, but not needed
			case TOUCH_KEY_HOME:
				input_event(tu->dev, EV_KEY, KEY_HOME, !!read_buf[TU_KEY_CODE]);
				prev_key = KEY_HOME;
				break;
			case TOUCH_KEY_BACK:
				input_event(tu->dev, EV_KEY, KEY_BACK, !!read_buf[TU_KEY_CODE]);
				prev_key = KEY_BACK;
				break;
			case TOUCH_KEY_MENU:
				input_event(tu->dev, EV_KEY, KEY_MENU, !!read_buf[TU_KEY_CODE]);
				prev_key = KEY_MENU;
				break;
			case TOUCH_KEY_REL:
				input_event(tu->dev, EV_KEY, prev_key, !!read_buf[TU_KEY_CODE]);
				break;
		*/
			default:
				//SZSZ error printk(tu->dev, "Unknown Key ID %02x", read_buf[TU_KEY_CODE]);
				break;
		}		
	}
	else if (read_buf[TU_RMOD] == 0xb1) 
	{
		prev_key = 0;
		touchcnt = read_buf[TU_POINTS];
		if( touchcnt==0 ) 
		{
			struct timespec ts;
			ktime_get_ts(&ts);
			if(report_rate < 1)
				report_rate = 1;
			if(report_rate > 100)
				report_rate = 100;
			next_touch_ns = timespec_to_ns(&ts) + NSEC_PER_SEC / report_rate;

			input_report_key(tu->dev, BTN_TOUCH, 0);
			input_report_abs(tu->dev, ABS_MT_TOUCH_MAJOR, 0);
            //RDRD
			input_report_abs(tu->dev, ABS_PRESSURE, 0);

#ifdef DEBUG_PRINT
			printk("TCnt=0 P=0\n");
#endif
		}
		else
		{
			idx_x_low = TU_1_POS_X_LOW;
			idx_x_hi  = TU_1_POS_X_HI;
			idx_y_low = TU_1_POS_Y_LOW;
			idx_y_hi  = TU_1_POS_Y_HI;
			idx_id_st = TU_1_ID_STATUS;
			for( i=0; i<touchcnt; i++ )
			{
				tu->x = COORD_INTERPRET(read_buf[idx_x_hi], read_buf[idx_x_low]);
//				tu->y = (TU_Y_AXIS - COORD_INTERPRET(read_buf[idx_y_hi],read_buf[idx_y_low]));
				tu->y = (COORD_INTERPRET(read_buf[idx_y_hi],read_buf[idx_y_low]));
				tu->w = (read_buf[idx_id_st]&0x0f);
				tu->id = (read_buf[idx_id_st]>>4)&0x0f;
				
				//RARA - revert x/y
				if (revert_x)
					tu->x = TU_X_AXIS - tu->x;
				if (revert_y)
					tu->y = TU_Y_AXIS - tu->y;
				
				
				
				tu_report(tu, i);		
				idx_x_low  += POINT_STRUCT_SIZE;
				idx_x_hi   += POINT_STRUCT_SIZE;
				idx_y_low  += POINT_STRUCT_SIZE;
				idx_y_hi   += POINT_STRUCT_SIZE;
				idx_id_st  += POINT_STRUCT_SIZE;
				//SZSZ touchcnt==0 seems never happen. Need to check tu->w (reported as touchMajor); if 0, pen up, else touch.
				//SZSZ simplify the work and handle one touch
				/*if (!tu->w) {
					input_report_abs(tu->dev, ABS_PRESSURE, 0);
#ifdef DEBUG_PRINT
				    printk("TCnt=%d P=0\n", touchcnt );
				} else {
				    printk("TCnt=%d\n", touchcnt );
#endif
				}*/
			}
		}
		//SZSZ report touch only for valid ID
		//input_sync(tu->dev);
	}
#ifdef DEBUG_PRINT
	else {
			printk("\n");
	}
#endif
	// Now that all mt/st events have been reported, issue a 'sync'
	 input_sync(tu->dev);
}


static void reset_panel(struct tu_platform_data *pdata)
{
	nxp_soc_gpio_set_out_value(pdata->gpio_reset,  (pdata->reset_cfg)); // reset
	// msleep (160); //quadruple x 10
//msleep (16); //quadruple
	mdelay (16); //quadruple
	nxp_soc_gpio_set_out_value(pdata->gpio_reset, !(pdata->reset_cfg)); // unreset/ena
	mdelay (200); // waiting for PixArt's spec
	return;
}

static int
tu_probe(struct i2c_client *client, const struct i2c_device_id *ids)
{
	struct tu_data *tu;
	struct input_dev *input_dev;
	int err = 0;
	struct tu_platform_data *pdata = client->dev.platform_data;

	/* allocate tu data */
	tu = kzalloc(sizeof(struct tu_data), GFP_KERNEL);
	if (!tu)
		return -ENOMEM;

	tu->client = client;
	dev_info(&tu->client->dev, "device probing\n");
	i2c_set_clientdata(client, tu);

	g_i2c_client=client;

	mutex_init(&tu->lock);

	/* allocate input device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tu->client->dev, "failed to allocate input device \n");
		goto exit_kfree;
	}
	tu->dev = input_dev;

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) |
					BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_VOLUMEUP, input_dev->keybit);
	set_bit(KEY_VOLUMEDOWN, input_dev->keybit);
	set_bit(KEY_SEND, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 8, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 8, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, MAX_TRACKING_ID, 0, 0);

	//SZSZ input_set_abs_params(input_dev, ABS_PRESSURE, 0, 256, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 100, 0, 0);

	//*X, Y Resolution
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
						0, TU_X_AXIS, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
						0, TU_Y_AXIS, 0, 0);

	input_set_abs_params(input_dev, ABS_X,
						0, TU_X_AXIS, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
						0, TU_Y_AXIS, 0, 0);

	// input_dev->name = client->name;
	//SZSZ need this name
	input_dev->name = "touchscreen interface";

	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	err = input_register_device(input_dev);
	if (err)
		goto exit_input;

	//*Interrupt Work for Scheduling
	INIT_WORK(&tu->work, tu_i2c_work);

	/* request IRQ resouce */
	if (client->irq < 0) {
		dev_err(&tu->client->dev,
			"No irq allocated in client resources!\n");
		goto exit_input;
	}

	tu->irq = client->irq;
	err = request_irq(tu->irq, tu_irq,
			IRQF_TRIGGER_FALLING, TU_DRIVER_NAME, tu);

    //SZSZ
    reset_panel(pdata);

    /* read fw version into global fwVersion array */
    tu_read_version(tu);
    /* store fw version into module param as string */
    sprintf(fw_version, "%x.%x.%x.%x", \
    		fwVersion[0], fwVersion[1], fwVersion[2], fwVersion[3]);

#ifdef CONFIG_PM
	//Set Suspend or Resumt
	tu->early_suspend.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tu->early_suspend.suspend	= tu_early_suspend;
	tu->early_suspend.resume	= tu_late_resume;
	register_early_suspend(&tu->early_suspend);
#endif
	return 0;

exit_input:
	input_unregister_device(tu->dev);
exit_kfree:
	kfree(tu);
	return err;
}

static int __devexit tu_remove(struct i2c_client *client)
{
	struct tu_data *tu = i2c_get_clientdata(client);

	free_irq(tu->irq, tu);
	input_unregister_device(tu->dev);
	kfree(tu);
	return 0;
}



#ifdef CONFIG_PM

static int tu_suspend(struct i2c_client *client, pm_message_t state)
{
	int ret;
	struct tu_data *tu = i2c_get_clientdata(client);
	printk("suspend!\n");
	
    ret = cancel_work_sync(&tu->work);
    ret = i2c_smbus_write_i2c_block_data(client, 0, NORM_CMD_LENG, command_list[0]);

	if (ret < 0)
		printk(KERN_ERR "i2c_smbus_write_i2c_block_data failed\n");
	
	return 0;
}



static int tu_resume(struct i2c_client *client)
{
   struct tu_data *tu = i2c_get_clientdata(client);
   int ret;
   printk("resume\n");
   ret = i2c_smbus_write_i2c_block_data(client, 0, NORM_CMD_LENG, command_list[1]);

	if (ret < 0)
		printk(KERN_ERR "i2c_smbus_write_i2c_block_data failed\n");

	return 0;
}

static void tu_early_suspend(struct early_suspend *h)
{
    struct tu_data *tu;
    
	tu_suspend(g_i2c_client, PMSG_SUSPEND);

}

static void tu_late_resume(struct early_suspend *h)
{
	tu_resume(g_i2c_client);
}

#else
#define tu_suspend NULL
#define tu_resume NULL
#endif




static struct i2c_device_id tu_id_table[] = {
    /* the slave address is passed by i2c_boardinfo */
	    {TU_DRIVER_NAME, },
//	    {"touchscreen interface", },
    {/* end of list */}
};

static struct i2c_driver tu_driver = {
	.driver = {
		.name	 = TU_DRIVER_NAME,
		.suspend = tu_suspend,
		.resume	 = tu_resume,
	},
	.id_table 	= tu_id_table,
	.probe 		= tu_probe,
	.remove 	= tu_remove,

#ifdef CONFIG_PM
	.suspend    = tu_early_suspend,
#endif
};

static int __init tu_init(void)
{
	return i2c_add_driver(&tu_driver);
}

static void tu_exit(void)
{
	i2c_del_driver(&tu_driver);
}
module_init(tu_init);
module_exit(tu_exit);

MODULE_LICENSE("GPL v2");

