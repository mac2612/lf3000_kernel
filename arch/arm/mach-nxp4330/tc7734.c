/*
 * Toshiba's TC7734 PMIC Driver
 *
 * Copyright:   (C) 2014 Toshiba
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/lf3000/gpio.h>

#include <mach/platform_id.h>
#include <mach/tc7734.h>

/* LF3000 USB sometimes misses VBUS disconnect, remind that driver */
void dwc_otg_pcd_update_disconnected_state_tc7734(void);

#define ST_VBATN_COUNT 120

static struct tc7734_chip *local_chip = NULL;

/*
 * local TC7734 write reg routine, no range checking
 */
static int tc7734_write_reg_raw(unsigned int reg, unsigned int value)
{
	struct i2c_adapter* i2c;
	struct i2c_msg i2c_messages[2];
	u8 buf[2];

	/* write new register value */
//	if (is_board_lucy())
//		i2c = i2c_get_adapter(TC7734_I2C_ADAPTER_1);
//	else
		i2c = i2c_get_adapter(TC7734_I2C_ADAPTER_0);

	if (!i2c) {
		printk(KERN_ERR "%s.%d return -EIO\n",
			__FUNCTION__, __LINE__);
		return -EIO;
	}

	buf[0] = 0xff & reg;
	buf[1] = 0xFF & value;

	/* write portion */
	i2c_messages[0].addr = TC7734_ADDR;
	i2c_messages[0].buf = buf;
	i2c_messages[0].len = 2;
	i2c_messages[0].flags = 0;	/* write */

	if (i2c_transfer(i2c, i2c_messages, 1) < 0) {
		i2c_put_adapter(i2c);
		printk(KERN_ERR "%s.%d return -EIO\n",
			__FUNCTION__, __LINE__);
		return -EIO;
	}

#if DEBUG_I2C
	printk(KERN_INFO "%s: %02x: %02x\n", __FUNCTION__, buf[0], buf[1]);
#endif

	i2c_put_adapter(i2c);
	return 0;
}

/*
 * local i2c read reg routine
 */
static int tc7734_read_reg_raw(unsigned int reg)
{
	struct i2c_adapter* i2c;
	struct i2c_msg i2c_messages[2];
	u8 buf[2];
	int ret;

//	if (is_board_lucy()) {
		i2c = i2c_get_adapter(TC7734_I2C_ADAPTER_0);
//	}
//	else {
//		i2c = i2c_get_adapter(0);
//	}
	if (!i2c) {
		printk(KERN_ERR "%s.%d return -EIO\n",
				__FUNCTION__, __LINE__);

		return -1;
	}

	buf[0] = 0xFF & reg;           /* read this register */
	buf[1] = 0;

	/* write portion */
	i2c_messages[0].addr = TC7734_ADDR;
	i2c_messages[0].buf = buf;
	i2c_messages[0].len = 1;
	i2c_messages[0].flags = 0;      /* write */

	/* read portion */
	i2c_messages[1].addr = TC7734_ADDR;
	i2c_messages[1].buf = buf;
	i2c_messages[1].len = 1; //2;
	i2c_messages[1].flags = I2C_M_RD;

	ret = i2c_transfer(i2c, i2c_messages, 2);
	i2c_put_adapter(i2c);
	if (ret < 0) {
		printk(KERN_ERR "%s.%d reg=%d, i2c_transfer=%d\n",
			__FUNCTION__, __LINE__, reg, ret);
		return -EIO;
	}

#if DEBUG_I2C
	printk(KERN_INFO "%s: %02x: %02x\n", __FUNCTION__, reg, buf[0]);
#endif

	return 0xFF & buf[0]; //[1];
}

static int tc7734_available(void)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&local_chip->lock, flags);
	ret = !local_chip->busy;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	return ret;
}

/*
 * local TC7734 write reg routine
 * Handle password protected registers, lock access as needed
 */

int tc7734_write_reg(unsigned int reg, unsigned int value)
{
	u8 *cache = local_chip->reg_cache;
	u8 *properties = local_chip->reg_properties;
	unsigned long flags;
	int ret=0;
	
	if ((!local_chip->have_tc7734) || (reg < TC7734_FIRSTREG) ||
		(reg > local_chip->maximum_register))
		return -EIO;

	/* serialize access to TC7734 and cache */
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(tc7734_available())))
			return -ERESTARTSYS;
		spin_lock_irqsave(&local_chip->lock, flags);
	}
	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	/*
	 * Only perform an I2C operation if reg is
	 * volatile or the new value is different
	 * Update cache and solve password protection
	 */

	if ((properties[reg] & REG_IS_VOLATILE) || (cache[reg] != value)) {
		if (properties[reg] & REG_HAS_PASSWORD) {
			/* unlock reg if needed */
			ret = tc7734_write_reg_raw(TC7734_PASSWD, PASSWD_VAL);
			if (ret < 0)
				goto exit;
		}
		ret = tc7734_write_reg_raw(reg, value);

		if (0 <= ret) /* wrote value to hardware, update the cache */
			cache[reg] = value;
	}

	/* release TC7734 */
exit:
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);
	return ret;
}
EXPORT_SYMBOL(tc7734_write_reg);

/*
 * local i2c read reg routine, use buffer if possible
 * Handle password protected registers, lock access as needed
 */
int tc7734_read_reg(unsigned int reg)
{
	u8 *cache = local_chip->reg_cache;
	u8 *properties = local_chip->reg_properties;
	unsigned long flags;
	int ret;

	if ((!local_chip->have_tc7734) || (reg < TC7734_FIRSTREG ||
		reg > local_chip->maximum_register))
		return -EIO;

	/* serialize access to TC7734 and cache */
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(tc7734_available())))
			return -ERESTARTSYS;
		spin_lock_irqsave(&local_chip->lock, flags);
	}
	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	/* return cached value if register is not volatile */
	if ((properties[reg] & REG_IS_VOLATILE) != REG_IS_VOLATILE) {
		ret = cache[reg];
	} else {
		/* register is volatile, read it */
		ret = tc7734_read_reg_raw(reg);
		if (0 <= ret ) {
			/* update the cache */
			cache[reg] = ret;
		} else {
			printk(KERN_ERR "%s.%d ret=%d\n", __func__,
					__LINE__, ret);
		}
	}

	/* release TC7734 */
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);

	return ret;
}

EXPORT_SYMBOL(tc7734_read_reg);

static int tc7734_write_bits(unsigned int reg, unsigned int mask, int value)
{
	int temp_val, ret;
	
	temp_val = tc7734_read_reg(reg);
	
	if (temp_val < 0)
		return temp_val;

	temp_val &= ~mask;
	temp_val |= value;

	ret = tc7734_write_reg(reg, temp_val);

	return ret;
}
static int tc7734_write_bits_raw(unsigned int reg, unsigned int mask, int value)
{
	int temp_val, ret;

	temp_val = tc7734_read_reg_raw(reg);

	if (temp_val < 0)
		return temp_val;

	temp_val &= ~mask;
	temp_val |= value;

	ret = tc7734_write_reg_raw(reg, temp_val);

	return ret;
}

static ssize_t show_vbus_draw(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	/* XXX: We do not have such setting. Confirm? */
	return sprintf(buf, "UNKNOWN\n");
}

static ssize_t set_vbus_draw(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	/* XXX: We do not have such setting. Confirm? */
	return count;
}

static DEVICE_ATTR(vbus_draw, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
	show_vbus_draw, set_vbus_draw);

static ssize_t show_charge_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 tmp;
	ssize_t ret = 0;

	tmp = tc7734_read_reg(TC7734_CHGCNF5) & 0xF;
	switch (tmp) {
		case TC7734_CHGCFG5_USBILIM_DEFAULT:
			/* TODO: What value to show here? */
			ret = sprintf(buf, "DEFAULT\n");
			break;
		case TC7734_CHGCFG5_USBILIM_100MA:
			ret = sprintf(buf, "100\n");
			break;
		case TC7734_CHGCFG5_USBILIM_300MA:
			ret = sprintf(buf, "300\n");
			break;
		case TC7734_CHGCFG5_USBILIM_400MA:
			ret = sprintf(buf, "400\n");
			break;
		case TC7734_CHGCFG5_USBILIM_500MA:
			ret = sprintf(buf, "500\n");
			break;
		case TC7734_CHGCFG5_USBILIM_700MA:
			ret = sprintf(buf, "700\n");
			break;
		case TC7734_CHGCFG5_USBILIM_1000MA:
			ret = sprintf(buf, "1000\n");
			break;
		case TC7734_CHGCFG5_USBILIM_1200MA:
			ret = sprintf(buf, "1200\n");
			break;
		case TC7734_CHGCFG5_USBILIM_1400MA:
			ret = sprintf(buf, "1400\n");
			break;
		case TC7734_CHGCFG5_USBILIM_1500MA:
			ret = sprintf(buf, "1500\n");
			break;
		default:
			ret = sprintf(buf, "UNKNOWN\n");
	}
	return ret;
}

static ssize_t set_charge_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	if (tc7734_set_charger_current(value))
		return -EINVAL;
	
	return count;
}

static DEVICE_ATTR(charge_current, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|
		   S_IWOTH,show_charge_current, set_charge_current);

static ssize_t show_backlight(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u32 tmp;

	tmp = tc7734_read_reg(TC7734_DDIM);
	tmp = tmp & TC7734_BACKLIGHT_CONFIG_MASK;
	return sprintf(buf, "BACKLIGHT = %d\n", tmp);
}

void tc7734_backlight_off(void)
{
	tc7734_write_bits(TC7734_POWER_EN,TC7734_PWREN_LEDD_EN, 0x00);		
}
EXPORT_SYMBOL(tc7734_backlight_off);

static ssize_t set_backlight(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%x", &value) != 1)
		return -EINVAL;

	if (32 < value)
		return -EINVAL;

	tc7734_write_bits(TC7734_DDIM, TC7734_BACKLIGHT_CONFIG_MASK,
				value);
	return count;
}

static DEVICE_ATTR(backlight, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
	show_backlight, set_backlight);

static ssize_t show_charge_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int tmp;
	ssize_t ret = 0;

	tmp = tc7734_read_reg(TC7734_STATE2) & TC7734_STATE2_CHG_EN;
	
	if (tmp)
		ret = sprintf(buf, "Enabled\n");
	else
		ret = sprintf(buf, "Disabled\n");
	
	return ret;
}

static ssize_t set_charge_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;
	
	if(value) {
		tc7734_write_bits(TC7734_STATE2, TC7734_STATE2_CHG_EN,
				TC7734_STATE2_CHG_EN);
	} else {
		tc7734_write_bits(TC7734_STATE2, TC7734_STATE2_CHG_EN, 0);
	}

	return count;
}

static DEVICE_ATTR(charge_enable, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|
	S_IWOTH, show_charge_enable, set_charge_enable);

static ssize_t show_power_source(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
        int charge_source;
	int reg;
	
	reg = tc7734_read_reg(TC7734_STAT1);
	if (reg & TC7734_STAT1_USBAC) {
                /* Charger detected */
                charge_source = (reg & TC7734_STAT1_STYP_MASK) >> 1;

		switch (charge_source) {
			case 0:
				ret = sprintf(buf, "Charger-Undefined\n");
				break;
			case 1:
				ret = sprintf(buf, "SDP USB\n");
				break;
			case 2:
				ret = sprintf(buf, "CDP USB\n");
				break;
			case 3:
				ret = sprintf(buf, "DCP/AC USB\n");
				break;
		}
	} else {
			ret = sprintf(buf, "Battery\n"); //battery is the power source if there is no USB connected
	}
	return ret;
}

static DEVICE_ATTR(power_source, S_IRUSR|S_IRGRP|S_IROTH, show_power_source, 
		NULL);

static ssize_t show_pmic_revision(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	ret = sprintf(buf,"1P0 \n");
	
	return ret;
}

static DEVICE_ATTR(pmic_revision, S_IRUSR|S_IRGRP|S_IROTH,
		show_pmic_revision, NULL);

static struct attribute *tc7734_attributes[] = {
	&dev_attr_backlight.attr,
	&dev_attr_vbus_draw.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_charge_enable.attr,
	&dev_attr_power_source.attr,
	&dev_attr_pmic_revision.attr,
	NULL
};

static struct attribute_group tc7734_attr_group = {
	.attrs = tc7734_attributes
};	


static void tc7734_monitor_task(struct work_struct *work)
{
	u8 *cache = local_chip->reg_cache;
	int pb_changed = 0;
	unsigned long flags;
	u8 int_reg=0;
	u8 reg;
	u8 charger_timeout_status,stat1=0,stat3=0;
	/* clears GPIO interrupt condition (unsafe in IRQ handler) */
	//disable_irq(gpio_to_irq(TC7734_INT));

	/* serialize access to chip*/
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(tc7734_available())))
			return;
		spin_lock_irqsave(&local_chip->lock, flags);
	}

	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	int_reg = tc7734_read_reg_raw(TC7734_INT_STAT);
	if (0 <= int_reg)
		cache[TC7734_INT_STAT] = int_reg;
	printk(KERN_INFO "INT: 0x%x\n",int_reg);
	if (int_reg & TC7734_INT_PB)   /* If PB state changed*/
				pb_changed = 1;
	else{
		msleep(75);   //75 passed, 50 failed.
		stat1 = tc7734_read_reg_raw(TC7734_STAT1);
		if (reg < 0)
			return;
		printk(KERN_INFO "STAT1 register: 0x%x\n", stat1);
		if (int_reg & TC7734_INT_USBAC) {  /* if USB status changed*/
			printk(KERN_NOTICE "\nUSBAC\n");
			/* charger connected */
			reg = tc7734_read_reg_raw(TC7734_STATE2);
			if (reg < 0)
				return;
			if (reg & TC7734_STATE2_CHG) {
				local_chip->charger_status = true;
				local_chip->charging_complete = false;
			}
		}
		if ((int_reg & TC7734_INT_CHGER)&&(stat1&TC7734_STAT1_USBAC)) {
			/*dont act on charger error if usb is not connected*/

			printk(KERN_INFO "CHGER\n");
			stat3 = tc7734_read_reg_raw(TC7734_STAT3);
			if (reg < 0)
				return;
			printk(KERN_INFO "STAT3 register: 0x%x\n", stat3);

			charger_timeout_status=stat3 & TC7734_STAT3_TOUT_MASK;
			if(!(stat3 & TC7734_STAT3_CGED1)) { //if charge is not complete
				if(charger_timeout_status==TC7734_STAT3_CHG_WAITING){  //charging did not start

					 //disable charger interrupt to avoid further interrupts
					tc7734_write_reg_raw(TC7734_INTMASK, ~(TC7734_INT_CHGCMP |
									TC7734_INT_PB |
									TC7734_INT_USBAC));   //enable charger interrupt in case it was disabled before
					tc7734_write_bits_raw(TC7734_STATE2,TC7734_STATE2_CHG_EN,0); //disable charger
					//gpio_set_value(CHG_FLT,1);  /*turn Red LED ON*/
					gpio_set_value(CHG_GRN_DISABLE,1);/*turn green LED off*/
				}
				else if((charger_timeout_status==TC7734_STAT3_PRECHG_TOUT)   //precharge timeout
					||(charger_timeout_status==TC7734_STAT3_CHG_TOUT)){      //charging timeout

					if((stat3&TC7734_STAT3_CGMD_MASK)==0){  //charging mode=no charge
							//disable charger interrupt to avoid further interrupts
						tc7734_write_reg_raw(TC7734_INTMASK, ~(TC7734_INT_CHGCMP |
										TC7734_INT_PB |
										TC7734_INT_USBAC));   //enable charger interrupt in case it was disabled before
						tc7734_write_bits_raw(TC7734_STATE2,TC7734_STATE2_CHG_EN,0); //disable charger
						gpio_set_value(CHG_FLT,1);  /*turn Red LED ON*/
						gpio_set_value(CHG_GRN_DISABLE,1);/*turn green LED off*/
					}
				}
			}
		}

		if(!(stat1&TC7734_STAT1_USBAC)){ //if USB is disconnected
			printk(KERN_INFO "USB DISCONNECT \n");
			gpio_set_value(CHG_FLT,0);  		/* let PMIC control charge status red LED */
			gpio_set_value(CHG_GRN_DISABLE,0);  /* let PMIC control charge status Green LED */
			tc7734_write_reg_raw(TC7734_INTMASK, ~(TC7734_INT_CHGER |
				TC7734_INT_CHGCMP |
				TC7734_INT_PB |
				TC7734_INT_USBAC));   //enable charger interrupt in case it was disabled before
			tc7734_write_bits_raw(TC7734_STATE2,TC7734_STATE2_CHG_EN,TC7734_STATE2_CHG_EN); //enable charger
			dwc_otg_pcd_update_disconnected_state_tc7734();
		}
	}
	/* Service register changes that have been delayed */
	if (local_chip->have_new_vbus_draw) {
		local_chip->have_new_vbus_draw = false;
		/* TODO - if applicable*/
	}
	/* release TC7734 */
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);
	if (pb_changed && local_chip->power_button_callback)
		local_chip->power_button_callback();
	return;
}


static irqreturn_t tc7734_chip_irq(int irq, void *priv)
{
	struct tc7734_chip *chip = (struct tc7734_chip *)priv;

	/*
	 * FIXME:
	 * 	Interrupt only stays for 5ms
	 * 	Do we need to check for pin level?
	 */
//	if (gpio_get_value(TC7734_INT) == 0) {
		/* schedule task, defer clearing interrupt condition */
		queue_work(chip->tc7734_tasks, &chip->tc7734_work);
		return IRQ_HANDLED;
//	}
//	return IRQ_NONE;
}


int tc7734_is_usb_present(void)
{
	int ret = 0;

	ret = ((tc7734_read_reg(TC7734_STAT1)& TC7734_STAT1_USBAC) ==
				TC7734_STAT1_USBAC)? 1 : 0;
	return ret;		
}
EXPORT_SYMBOL(tc7734_is_usb_present);


/* This routine is designed to return 1 if the device is taking no power from battery
 * routine will return 0 if the device is running completely or partially running on battery
 * Since Bogota consumes more than 500mA in active mode, it drains battery even when connected to SDP
 * Bogota runs completely on USB power only when connected to CDP or DCP
 */
#define USB_NONSTANDARD		0
#define USB_SDP				1
#define USB_CDP				2
#define USB_DCP				3

int tc7734_is_usb_powered()
{
	int ret = 0;
	int charge_source;
	u8 tc7734_stat1_reg ;

	tc7734_stat1_reg = tc7734_read_reg(TC7734_STAT1);
	if (tc7734_stat1_reg & TC7734_STAT1_USBAC) {
		charge_source = (tc7734_stat1_reg & TC7734_STAT1_STYP_MASK) >> 1;
		if((charge_source==USB_CDP)||(charge_source==USB_DCP))
			return 1;  //device is not taking any power from battery
		else
			return 0;  //device is getting partially powered by battery
	}
	else
		return 0;  //no USB source so this should be battery
}

int tc7734_is_charging_active(void)
{
	int ret = 0;
	
	ret = ((tc7734_read_reg(TC7734_STATE2) & TC7734_STATE2_CHG_MASK) ==
		TC7734_STATE2_CHG) ? 1 : 0;

	return ret;
}
EXPORT_SYMBOL(tc7734_is_charging_active);

void tc7734_set_power_button_callback(void (*callback)(void))
{
        local_chip->power_button_callback = callback;
}
EXPORT_SYMBOL(tc7734_set_power_button_callback);

int tc7734_get_power_button(void)
{
        int ret = 0;

	if (tc7734_read_reg(TC7734_STAT1) & TC7734_STATUS1_PB) {
		ret = 1;
		printk(KERN_INFO "%s.%d power button status %d\n", __FUNCTION__,
				__LINE__, ret);
	}

	return ret;
}
EXPORT_SYMBOL(tc7734_get_power_button);

void tc7734_set_power_standby(void)
{
	/* go to standby mode */
	tc7734_write_reg(TC7734_STATE1, TC7734_STATE_SW_STANDBY);
}
//EXPORT_SYMBOL(tc7734_set_power_standby);
// not exporting so that it's not used by mistake. Always go through tc7734_set_power_off routine

/*FWBOG-299-PMIC should be OFF if USB NOT connected*/
void tc7734_set_power_off(void)
{
	if(tc7734_is_usb_present()){
		printk(KERN_ALERT "PMIC Standby.\n");
		/*PMIC in standby mode to continue charging */
		tc7734_set_power_standby();
	}else{
		printk(KERN_ALERT "PMIC OFF\n");
		/*PMIC in OFF mode to reduce sleep current and increase battery life on shelf */
		tc7734_write_reg(TC7734_STATE1, TC7734_STATE_OFF);
	}

}
EXPORT_SYMBOL(tc7734_set_power_off);


int tc7734_set_vbus_draw(unsigned int mA)
{
	/*
	 * TODO: Supported by our chip?
	 * 	We only support setting charging current
	 */
	if (local_chip) {
		return -ENOTSUPP;
	}
	else
	{
		return -EIO;
	}

	/* XXX: Not needed if not supported */
#if 0
	local_chip->have_new_vbus_draw = true;
	/* schedule task, defer clearing interrupt condition */
	queue_work(local_chip->tc7734_tasks, &local_chip->tc7734_work);
	printk(KERN_ALERT "%s vbus draw is %d mA\n", __FUNCTION__, mA);
#endif
	return 0;
}

EXPORT_SYMBOL(tc7734_set_vbus_draw);

int tc7734_set_charger_current(unsigned int mA)
{
	/*
	 * FIXME: Do we need to change the DCP & CDP limit?
	 * 	I believe it should change automatically. Please confirm
	 */

	if (local_chip) {
		if      (mA <= 100)
			local_chip->ichrg_value = TC7734_CHGCFG5_USBILIM_100MA;
		else if (mA <= 300)
			local_chip->ichrg_value = TC7734_CHGCFG5_USBILIM_300MA;
		else if (mA <= 400)
			local_chip->ichrg_value = TC7734_CHGCFG5_USBILIM_400MA;
		else if (mA <= 500)
			local_chip->ichrg_value = TC7734_CHGCFG5_USBILIM_500MA;
		else if (mA <= 700)
			local_chip->ichrg_value = TC7734_CHGCFG5_USBILIM_700MA;
		else if (mA <= 1000)
			local_chip->ichrg_value = TC7734_CHGCFG5_USBILIM_1000MA;
		else if (mA <= 1200)
			local_chip->ichrg_value = TC7734_CHGCFG5_USBILIM_1200MA;
		else if (mA <= 1400)
			local_chip->ichrg_value = TC7734_CHGCFG5_USBILIM_1400MA;
		else if (mA <= 1500)
			local_chip->ichrg_value = TC7734_CHGCFG5_USBILIM_1500MA;
		else return -ENOTSUPP;
	}
	else
	{
		return -EIO;
	}

	//local_chip->have_new_ichrg_value = true;
	/* schedule task, defer clearing interrupt condition */
	//queue_work(local_chip->tc7734_tasks, &local_chip->tc7734_work);

	tc7734_write_bits(TC7734_CHGCNF5,
			TC7734_CHGCFG5_USBILIM_MASK,
			local_chip->ichrg_value);
	printk(KERN_ALERT "%s charger current is %d mA\n", __FUNCTION__, mA);

	return 0;
}

EXPORT_SYMBOL(tc7734_set_charger_current);

enum charging_currents tc7734_get_default_charger_current(void)
{
	enum charging_currents value;
    int charge_source;
	int reg;
	
	if(!(tc7734_read_reg(TC7734_STATE2) & TC7734_STATE2_CHG_EN)) //Charging is disabled
	{
		value = NO_CHARGING_CURRENT;
		return value;
	}
	
	reg = tc7734_read_reg(TC7734_STAT1);
	if (reg & TC7734_STAT1_USBAC) {
		/* Charger detected */
		charge_source = (reg & TC7734_STAT1_STYP_MASK) >> 1;
		switch (charge_source) {
			case 1:
				value = SDP_CHARGING_CURRENT; //SDP
				break;
			case 2:
				value = CDP_CHARGING_CURRENT; //CDP
				break;
			case 3:
				value = DCP_CHARGING_CURRENT; //DCP
				break;
			default: 
				value = 0;
				break;
		}
	} 
	else {
		/* charger not detected */
		value = BATTERY_NO_CURRENT; // On battery
	}
	
	return value;
}

EXPORT_SYMBOL(tc7734_get_default_charger_current); 

int tc7734_have_tc7734(void)
{
	if (local_chip && local_chip->have_tc7734)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(tc7734_have_tc7734);

int tc7734_get_num_registers(void)
{
	return(local_chip->maximum_register - TC7734_FIRSTREG + 1);
}
EXPORT_SYMBOL(tc7734_get_num_registers);

u8 tc7734_get_pmic_rev(void)
{
	if(local_chip && local_chip->tc7734_rev)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(tc7734_get_pmic_rev);

static int tc7734_chip_probe(struct platform_device *pdev)
{
	int i;
	int ret = 0;
	struct tc7734_chip *priv;

	printk(KERN_INFO "%s: init %s\n", __FUNCTION__, TC7734_NAME);

	priv = kzalloc(sizeof(struct tc7734_chip), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto fail_alloc;
	}

	platform_set_drvdata(pdev, priv);
	local_chip = priv;

	/* mark registers that are password protected or volatile */
	priv->reg_properties[TC7734_POWER_EN] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_STATE1] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_STATE2] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_DEFLDO12] = REG_HAS_PASSWORD;
	priv->reg_properties[TC7734_DEFDCDC12] = REG_HAS_PASSWORD;
	priv->reg_properties[TC7734_DEFDCDC34] = REG_HAS_PASSWORD;
	priv->reg_properties[TC7734_SEQDLY1] = REG_HAS_PASSWORD;
	priv->reg_properties[TC7734_SEQDLY2] = REG_HAS_PASSWORD;
	priv->reg_properties[TC7734_CHGCNF5] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_STATE_CONFIG] = REG_HAS_PASSWORD;
	priv->reg_properties[TC7734_INTMASK] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_PG_MASK] = REG_HAS_PASSWORD;
	priv->reg_properties[TC7734_PASSWD] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_INT_STAT] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_STAT1] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_STAT2] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_STAT3] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_STAT4] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_STAT5] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_STAT6] = REG_IS_VOLATILE;
	priv->reg_properties[TC7734_PG_MON] = REG_IS_VOLATILE;

	init_waitqueue_head(&priv->wait);
	spin_lock_init(&priv->lock);

	/* get TC7734 revision */
	ret = tc7734_read_reg_raw(TC7734_PRODUCT_ID);
	if (ret < 0) {
		printk(KERN_ERR "%s.%d No  TC7734\n",
				__FUNCTION__, __LINE__);
		ret = -ENXIO;
		goto no_tc7734;
	}

	priv->reg_cache[TC7734_PRODUCT_ID] = ret;
	if((priv->reg_cache[TC7734_PRODUCT_ID] & TC7734_CHIPID_CHIP_MASK) 
					== TC7734_CHIPID_CHIP_TC7734){
		printk(KERN_INFO "%s.%d Chip ID = %u\n", __FUNCTION__,__LINE__,
				(priv->reg_cache[TC7734_PRODUCT_ID] 
					& TC7734_CHIPID_CHIP_MASK));
		priv->maximum_register = TC7734_LASTREG;
	}

	/* initialize TC7734 cache by reading registers */
	for (i = 0; i <= priv->maximum_register; i++) {
		/* No register between 0x16-0x19 */
		if (i == 0x16)
			i = 0x20;

		ret = tc7734_read_reg_raw(i);
		if (ret < 0) {
			printk(KERN_ERR "%s.%d No TC7734\n",
					__FUNCTION__, __LINE__);
			ret = -ENXIO;
			goto no_tc7734;
		}
		priv->reg_cache[i] = ret;
	}

	priv->have_tc7734 = 1;		/* read registers */
	priv->charger_status = 0;	/* Before charging is enabled */
	priv->charging_complete = 0;	/* assume battery is not fully
					   charged till the 1st pass */
	ret = sysfs_create_group(&pdev->dev.kobj, &tc7734_attr_group);
	if (ret)
		goto sysfs_fail;


	/* request GPIO input for TC7734 IRQ */
	ret = gpio_request_one(TC7734_INT, GPIOF_IN, "TC7734 INT");
	//ret = gpio_request_one(TC7734 INT, GPIOF_IN, "LFP100 INT");
	if (ret) {
		printk(KERN_ERR "%s.%d: failed to get TC7734 GPIO %d\n",
				__FUNCTION__, __LINE__, TC7734_INT);
		goto sysfs_fail;
	}
	ret = request_irq(gpio_to_irq(TC7734_INT),
			tc7734_chip_irq,
			IRQF_TRIGGER_FALLING | IRQF_DISABLED,
			"TC7734 IRQ", priv);
	if (ret) {
		printk(KERN_ERR "%s.%d: failed to get TC7734 IRQ\n",
				__FUNCTION__, __LINE__);
		goto sysfs_fail;
	}
	disable_irq(gpio_to_irq(TC7734_INT));

	/* initialize worker thread which processes irq */
	priv->tc7734_tasks = create_singlethread_workqueue("tc7734 tasks");
	INIT_WORK(&priv->tc7734_work, tc7734_monitor_task);

	/* XXX: USB Limit currently set as default for CDP, SDP & DCP */

	/* turn on TC7734 backlight */
	tc7734_write_bits(TC7734_POWER_EN, TC7734_PWREN_LEDD_EN,
			TC7734_PWREN_LEDD_EN);

	/*
	 * Interrupt enable:
	 * 1. charge status
	 * 2. Charge completion
	 * 3. Push Button
	 * 4. USB Adapter Detect/Remove
	 * XXX: Enable more? see reg 0x10
	 */
	tc7734_write_reg(TC7734_INTMASK, ~(TC7734_INT_CHGER |
			TC7734_INT_CHGCMP |
			TC7734_INT_PB |
			TC7734_INT_USBAC));

	/* Enable Charger */
	tc7734_write_reg(TC7734_STATE2, TC7734_STATE2_DCP_EN |
			TC7734_STATE2_CDP_EN |
			TC7734_STATE2_SDP_EN |
			TC7734_STATE2_CHG_EN);

	/* clear any pending interrupts */
	tc7734_read_reg(TC7734_INT_STAT);

	/* Register dump */		
#if DEBUG_I2C
	for (i = 0; i <= priv->maximum_register; i++) {
		ret = tc7734_read_reg_raw(i);
		if (0 <= ret) {
			printk(KERN_INFO "%s: reg 0x%02X: %02X\n", __FUNCTION__, i, ret);
		} else {
			printk(KERN_INFO "%s: reg 0x%02X: error ret=%d\n", __FUNCTION__, i, ret);
		}
	}
#endif

	/* setup IRQ */
	enable_irq(gpio_to_irq(TC7734_INT));
	printk(KERN_INFO "%s: success %s\n", __FUNCTION__, TC7734_NAME);
	local_chip->dcin_cnt = 0;
	ret = 0;
no_tc7734:
	return ret;

sysfs_fail:
	gpio_free(TC7734_INT);
	sysfs_remove_group(&pdev->dev.kobj, &tc7734_attr_group);
	kfree(priv);
fail_alloc:
	local_chip = NULL;
	return ret;
}


static int tc7734_chip_remove(struct platform_device *pdev)
{
	disable_irq(gpio_to_irq(TC7734_INT));
	free_irq(gpio_to_irq(TC7734_INT), local_chip);
	gpio_free(TC7734_INT);

	destroy_workqueue(local_chip->tc7734_tasks);

	sysfs_remove_group(&pdev->dev.kobj, &tc7734_attr_group);
	kfree(local_chip);
	local_chip = NULL;
	return 0;
}

static struct platform_driver tc7734_chip_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tc7734-chip",
	},
	.probe		= tc7734_chip_probe,
	.remove		= tc7734_chip_remove,
};

static int __init tc7734_chip_init(void)
{
	printk(KERN_INFO "%s: \n", __FUNCTION__);
	return platform_driver_register(&tc7734_chip_driver);
}

static void tc7734_chip_exit(void)
{
	printk(KERN_INFO "%s: \n", __FUNCTION__);
	platform_driver_unregister(&tc7734_chip_driver);
}

MODULE_DESCRIPTION("TC7734 PMIC Driver");
MODULE_LICENSE("GPL");

//module_init(tc7734_chip_init);
late_initcall(tc7734_chip_init);
module_exit(tc7734_chip_exit);
