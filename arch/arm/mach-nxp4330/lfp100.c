/*
 * LeapFrog LFP100 / LFP200 / LFP300 device driver
 *
 * Written by Scott Esters <sesters@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The LFP100 chip haves codec, LCD backlight, and power functions in a
 * single package.  Chip support is here with addtional functionality
 * in the respective hwmon, sound/soc, and video driver directories.
 *
 * Place shared code here for all I2C access.  Cache the I2C registers, like
 * in the sound driver codecs.  Provide access via private getter/setter calls
 * and sysfs hooks.  Monitor register changes via IRQ, read_reg and write_reg
 * routines.
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
#include <mach/lfp100.h>

#define DEBUG_I2C	0	// 0 = none, 1 = tracers, 2 = no reg caching
#define POLL_I2C	0	// 1 = polling, 0 = interrupt

struct lfp100_chip {
	wait_queue_head_t wait;
	spinlock_t lock;
	struct work_struct lfp100_work;		/* task			*/
	struct workqueue_struct *lfp100_tasks;	/* workqueue		*/
	bool	have_lfp100;			/* 1 = found chip	*/
	bool	busy;				/* 1 = chip is busy	*/
	bool	have_new_vbus_draw;		/* update vbus draw */
	bool	have_new_ichrg_value;		/* update ichrg_value */
	bool    charger_status;			/* charging started or stopped */
	bool    charging_complete;		/* charging has completed */
	u8	reg_cache[LFP100_NUMREGS];	/* saved reg values	*/
	u8	reg_properties[LFP100_NUMREGS];	/* register properties	*/
	void	(*power_button_callback)(void);
	u8	iusb_value;			/* new vbus draw value  */
	u8	ichrg_value;			/* new battery charge value */
	int	maximum_register;		/* maximum register value */
	u8 lfp100_rev;              /* pmic version 1P0 or 1P1 */
	unsigned int usb_powered;   /* track USB plugged in w/ or w/o push button */
};

#define REG_HAS_PASSWORD 0x01	/* register has password */
#define REG_IS_VOLATILE	 0x02	/* register value is volatile  */

static struct lfp100_chip *local_chip = NULL;

/*
 * local LFP100 write reg routine, no range checking
 */
static int lfp100_write_reg_raw(unsigned int reg, unsigned int value)
{
	struct i2c_adapter* i2c;
	struct i2c_msg i2c_messages[2];
	u8 buf[2];

	/* write new register value */
	if (is_board_lucy())
		i2c = i2c_get_adapter(LFP100_I2C_ADAPTER_1);
	else
		i2c = i2c_get_adapter(LFP100_I2C_ADAPTER_0);

	if (!i2c) {
		printk(KERN_ERR "%s.%d return -EIO\n",
			__FUNCTION__, __LINE__);
		return -EIO;
	}

	buf[0] = 0xff & reg;
	buf[1] = 0xFF & value;

	/* write portion */
	i2c_messages[0].addr = LFP100_ADDR;
	i2c_messages[0].buf = buf;
	i2c_messages[0].len = 2;
	i2c_messages[0].flags = 0;      /* write */

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
static int lfp100_read_reg_raw(unsigned int reg)
{
	struct i2c_adapter* i2c;
	struct i2c_msg i2c_messages[2];
	u8 buf[2];
	int ret;

	if (is_board_lucy())
		i2c = i2c_get_adapter(LFP100_I2C_ADAPTER_0);
	else
		i2c = i2c_get_adapter(0);
	if (!i2c)
		return -1;

	buf[0] = 0xFF & reg;           /* read this register */
	buf[1] = 0;

	/* write portion */
	i2c_messages[0].addr = LFP100_ADDR;
	i2c_messages[0].buf = buf;
	i2c_messages[0].len = 1;
	i2c_messages[0].flags = 0;      /* write */

	/* read portion */
	i2c_messages[1].addr = LFP100_ADDR;
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

static int lfp100_available(void)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&local_chip->lock, flags);
	ret = !local_chip->busy;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	return ret;
}

static int lfp100_is_dac_ready(void)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&local_chip->lock, flags);
	ret = (local_chip->reg_cache[LFP100_STATUS2] & LFP100_STATUS2_ABUSY) != LFP100_STATUS2_ABUSY;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	return ret;
}

/*
 * local LFP100 write reg routine
 * Handle password protected registers, lock access as needed
 */

int lfp100_write_reg(unsigned int reg, unsigned int value)
{
	u8 *cache = local_chip->reg_cache;
	u8 *properties = local_chip->reg_properties;
	unsigned long flags;
	int ret=0;
	int i;
	
	if ((!local_chip->have_lfp100) || (reg < LFP100_FIRSTREG) ||
		(reg > local_chip->maximum_register))
		return -EIO;

#if !(POLL_I2C)
	(void)i;

	/* wait for DAC ready if writing HP_EN, SPK_EN, DMUTE, AMUTE */
	if (true || reg == LFP100_A_CONTROL || reg == LFP100_VOLUME 
					    || reg == LFP100_MGAIN) {
		ret = lfp100_read_reg(LFP100_STATUS2);
		if (ret & LFP100_STATUS2_ABUSY)
			wait_event_interruptible(local_chip->wait, lfp100_is_dac_ready());
	}
#endif

	/* serialize access to LFP100 and cache */
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(lfp100_available())))
			return -ERESTARTSYS;
		spin_lock_irqsave(&local_chip->lock, flags);
	}
	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

#if POLL_I2C
	/* wait for busy bit to clear */
	for (i=0; i < 20; i++) {
		if(lfp100_read_reg_raw(LFP100_STATUS2) &
			LFP100_STATUS2_ABUSY) {
			msleep(20);
		} else {
			break;
		}
	}
#endif

	/*
	 * Only perform an I2C operation if reg is
	 * volatile or the new value is different
	 */
#if (DEBUG_I2C < 2)
	if (((properties[reg - LFP100_FIRSTREG] & REG_IS_VOLATILE) == REG_IS_VOLATILE) ||
	    (cache[reg - LFP100_FIRSTREG] != value))
#endif
	{
		if ((properties[reg - LFP100_FIRSTREG] & REG_HAS_PASSWORD) == REG_HAS_PASSWORD) {
			/* unlock reg if needed */
			ret =
			   lfp100_write_reg_raw(LFP100_PASSWORD,
				LFP100_UNLOCK(reg));
			if (ret < 0)
				goto exit;
		}
		ret = lfp100_write_reg_raw(reg, value);

		if (0 <= ret) /* wrote value to hardware, update the cache */
			cache[reg - LFP100_FIRSTREG] = value;
	}

	/* release LFP100 */
exit:
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);
	return ret;
}

EXPORT_SYMBOL(lfp100_write_reg);

/*
 * local i2c read reg routine, use buffer if possible
 * Handle password protected registers, lock access as needed
 */

int lfp100_read_reg(unsigned int reg)
{
	u8 *cache = local_chip->reg_cache;
	u8 *properties = local_chip->reg_properties;
	unsigned long flags;
	int ret;

	if ((!local_chip->have_lfp100) || (reg < LFP100_FIRSTREG ||
		reg > local_chip->maximum_register))
		return -EIO;

	/* serialize access to LFP100 and cache */
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(lfp100_available())))
			return -ERESTARTSYS;
		spin_lock_irqsave(&local_chip->lock, flags);
	}
	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

#if (DEBUG_I2C < 2)
	/* return cached value if register is not volatile */
	if ((properties[reg - LFP100_FIRSTREG] & REG_IS_VOLATILE) != REG_IS_VOLATILE) {
		ret = cache[reg - LFP100_FIRSTREG];
	} else
#endif
	{ /* register is volatile, read it */
		ret = lfp100_read_reg_raw(reg);
		if (0 <= ret )	/* update the cache */
			cache[reg - LFP100_FIRSTREG] = ret;
		else
			printk(KERN_ERR "%s.%d ret=%d\n",
				__FUNCTION__, __LINE__, ret);
	}

	/* release LFP100 */
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);

	return ret;
}

EXPORT_SYMBOL(lfp100_read_reg);

/*
 * sysfs interface
 */

static ssize_t show_volume(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u32 tmp;

	tmp = lfp100_read_reg(LFP100_VOLUME);
	return sprintf(buf, "VOLUME = %d\n", tmp);
}

static ssize_t set_volume(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%x", &value) != 1)
		return -EINVAL;

	if (63 < value)
		return -EINVAL;

	lfp100_write_reg(LFP100_VOLUME, (u8)value);
	return count;
}

static DEVICE_ATTR(volume, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
	show_volume, set_volume);

static ssize_t show_backlight(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u32 tmp;

	tmp = lfp100_read_reg(LFP100_WLED);
	return sprintf(buf, "BACKLIGHT = %d\n", tmp);
}

static ssize_t set_backlight(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%x", &value) != 1)
		return -EINVAL;

	if (31 < value)
		return -EINVAL;

	lfp100_write_reg(LFP100_WLED, (u8)value);
	return count;
}

static DEVICE_ATTR(backlight, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
	show_backlight, set_backlight);
	
static ssize_t show_vbus_draw(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u32 tmp;
	ssize_t ret = 0;

	tmp = lfp100_read_reg(LFP100_PPATH) & LFP100_PPATH_IUSB_MASK;
	switch(tmp)
	{
		case LFP100_PPATH_IUSB_100MA:
			ret = sprintf(buf, "100\n");
			break;
		case LFP100_PPATH_IUSB_500MA:
			ret = sprintf(buf, "500\n");
			break;
		case LFP100_PPATH_IUSB_1300MA:
			ret = sprintf(buf, "1300\n");
			break;
		case LFP100_PPATH_IUSB_1800MA:
			ret = sprintf(buf, "1800\n");
			break;
		default:
			ret = sprintf(buf, "UNKNOWN\n");
	}
	return ret;
}

static ssize_t set_vbus_draw(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	if( lfp100_set_vbus_draw(value) )
		return -EINVAL;
	
	return count;
}

static DEVICE_ATTR(vbus_draw, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
	show_vbus_draw, set_vbus_draw);

static ssize_t show_charge_current(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u32 tmp;
	ssize_t ret = 0;

	tmp = lfp100_read_reg(LFP100_CHGCONFIG3) & LFP100_CHGCONFIG3_ICHRG_MASK;
	switch(tmp)
	{
		case LFP100_CHGCONFIG3_ICHRG_300MA:
			ret = sprintf(buf, "300\n");
			break;
		case LFP100_CHGCONFIG3_ICHRG_400MA:
			ret = sprintf(buf, "400\n");
			break;
		case LFP100_CHGCONFIG3_ICHRG_500MA:
			ret = sprintf(buf, "500\n");
			break;
		case LFP100_CHGCONFIG3_ICHRG_700MA:
			ret = sprintf(buf, "700\n");
			break;
		default:
			ret = sprintf(buf, "UNKNOWN\n");
	}
	return ret;
}

static ssize_t set_charge_current(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	if( lfp100_set_charger_current(value) )
		return -EINVAL;
	
	return count;
}

static DEVICE_ATTR(charge_current, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
	show_charge_current, set_charge_current);
	

static ssize_t show_charge_enable(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int tmp, reg;
	ssize_t ret = 0;

	tmp = lfp100_read_reg(LFP100_CHGCONFIG1) & LFP100_CHGCONFIG1_CHG_EN;
	reg = lfp100_read_reg(LFP100_CHGCONFIG0);
	
	if(tmp)
	{
		if(reg & LFP100_CHGCONFIG0_TERMI)
		{
			ret = sprintf(buf, "Disabled\n");
		}
		else
		{
			ret = sprintf(buf, "Enabled\n");
		}
	}
	else
		ret = sprintf(buf, "Disabled\n");
	
	return ret;
}

static ssize_t set_charge_enable(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;
	
	if(value)
		lfp100_write_reg(LFP100_CHGCONFIG1, lfp100_read_reg(LFP100_CHGCONFIG1) | LFP100_CHGCONFIG1_CHG_EN);
	else
		lfp100_write_reg(LFP100_CHGCONFIG1, lfp100_read_reg(LFP100_CHGCONFIG1) & (~(LFP100_CHGCONFIG1_CHG_EN)));
		
	return count;
}

static DEVICE_ATTR(charge_enable, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
	show_charge_enable, set_charge_enable);
	
static ssize_t show_power_source(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u8 lfp100_status1_reg;
	u8 lfp100_int2_reg;
	ssize_t ret = 0;
	
	lfp100_int2_reg = lfp100_read_reg(LFP100_INT2);
	
	/* show power source */
	lfp100_status1_reg = lfp100_read_reg(LFP100_STATUS1);
	
	switch(lfp100_status1_reg & LFP100_STATUS1_SOURCE_MASK) {
	case LFP100_STATUS1_SOURCE_AC:
		ret = sprintf(buf, "AC\n");
		break;
	case LFP100_STATUS1_SOURCE_USB:
		ret = sprintf(buf, "USB\n");
		break;
	case LFP100_STATUS1_SOURCE_BAT:
		ret = sprintf(buf, "Battery\n");
		break;
	case LFP100_STATUS1_SOURCE_UNDEF:
		ret = sprintf(buf, "Undefined\n");
		break;
	}
	
	return ret;
}

static DEVICE_ATTR(power_source, S_IRUSR|S_IRGRP|S_IROTH, show_power_source, NULL);

static ssize_t show_pmic_revision(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u8 rev;
	ssize_t ret = 0;
	
	/* 1P0 (old) or 1P1 (new) */
	rev = lfp100_get_pmic_rev();
	
	if(rev)
	{
		ret = sprintf(buf,"1P1 \n");
	}
	else
	{
		ret = sprintf(buf,"1P0 \n");
	}
	
	return ret;
}

static DEVICE_ATTR(pmic_revision, S_IRUSR|S_IRGRP|S_IROTH, show_pmic_revision, NULL);
	
static struct attribute *lfp100_attributes[] = {
	&dev_attr_volume.attr,
	&dev_attr_backlight.attr,
	&dev_attr_vbus_draw.attr,
	&dev_attr_charge_current.attr,
	&dev_attr_charge_enable.attr,
	&dev_attr_power_source.attr,
	&dev_attr_pmic_revision.attr,
	NULL
};

static struct attribute_group lfp100_attr_group = {
	.attrs = lfp100_attributes
};


static void lfp100_monitor_task(struct work_struct *work)
{
	u8 *cache = local_chip->reg_cache;
	unsigned long flags;
	int reg;

	/* clears GPIO interrupt condition (unsafe in IRQ handler) */
	//disable_irq(gpio_to_irq(LFP100_INT));

	/* serialize access to LFP100 and cache */
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(lfp100_available())))
			return;
		spin_lock_irqsave(&local_chip->lock, flags);
	}
	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	/* read LFP100 registers */
	reg = lfp100_read_reg_raw(LFP100_CONTROL);
	if (0 <= reg)
		cache[LFP100_CONTROL] = reg;

	reg = lfp100_read_reg_raw(LFP100_STATUS1);
	if (0 <= reg)
		cache[LFP100_STATUS1] = reg;

	reg = lfp100_read_reg_raw(LFP100_STATUS2);
	if (0 <= reg)
		cache[LFP100_STATUS2] = reg;

	reg = lfp100_read_reg_raw(LFP100_INT3);
	if (0 <= reg)
		cache[LFP100_INT3] = reg;
		
	/* Check if interrupt is for charging status change and 
	   if it's because the battery was charged to its capacity. */
	if(reg & LFP100_MASK3_CHARGERM)
		local_chip->charger_status = true;
	else
		local_chip->charger_status = false;
	
	if(local_chip->charger_status)
	{
		reg = lfp100_read_reg_raw(LFP100_CHGCONFIG0);
		if(reg & LFP100_CHGCONFIG0_TERMI)
		{
			local_chip->charging_complete = true;
			printk(KERN_ALERT "%s: Battery Charging has completed \n", 
			__func__);
		}
		else
		{
			local_chip->charging_complete = false;
		}
	}
	
	if (0 <= reg)
		cache[LFP100_CHGCONFIG0] = reg;
		
	reg = lfp100_read_reg_raw(LFP100_INT2);
	if (0 <= reg)
		cache[LFP100_INT2] = reg;
		
	reg = lfp100_read_reg_raw(LFP100_INT1);
	if (0 <= reg)
		cache[LFP100_INT1] = reg;

	/* Service register changes that have been delayed */
	if (local_chip->have_new_vbus_draw) {
		local_chip->have_new_vbus_draw = false;
		reg = lfp100_read_reg_raw(LFP100_PPATH);
		reg &= ~(LFP100_PPATH_IUSB_MASK);
		reg |= local_chip->iusb_value;
		lfp100_write_reg_raw(LFP100_PPATH, reg);
		cache[LFP100_PPATH] = reg;
	}
	
	if (local_chip->have_new_ichrg_value)
	{
		local_chip->have_new_ichrg_value = false;
		reg = lfp100_read_reg_raw(LFP100_CHGCONFIG3);
		reg &= ~(LFP100_CHGCONFIG3_ICHRG_MASK);
		reg |= local_chip->ichrg_value;
		lfp100_write_reg_raw(LFP100_CHGCONFIG3, reg);
		cache[LFP100_CHGCONFIG3] = reg;
	}

	/* release LFP100 */
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);

	//enable_irq(gpio_to_irq(LFP100_INT));

	// Enable for debugging LFP300 interrupt sources
	// Don't print while holding onto LFP100 spinlock
#if 0
	if ((cache[LFP100_CHIPID] & LFP100_CHIPID_CHIP_MASK)
		== LFP100_CHIPID_CHIP_LFP300) {
		printk(KERN_INFO "%s: LFP100 CONTROL:0x%2.2X  STATUS1:0x%2.2X  STATUS2:0x%2.2X\n",
			__func__,
			cache[LFP100_CONTROL], cache[LFP100_STATUS1], cache[LFP100_STATUS2]);
		printk(KERN_INFO "%s: LFP100    INT1:0x%2.2X     INT2:0x%2.2X     INT3:0x%2.2X\n",
			__func__,
			cache[LFP100_INT1], cache[LFP100_INT2], cache[LFP100_INT3]);
		printk(KERN_INFO "%s: LFP100 CHGCONFIG0:0x%2.2X\n",
			__func__,
			cache[LFP100_CHGCONFIG0]);
	}
	else {
		printk(KERN_INFO "%s: LFP100 CONTROL:0x%2.2X  STATUS1:0x%2.2X  STATUS2:0x%2.2X\n",
			__func__,
			cache[LFP100_CONTROL], cache[LFP100_STATUS1], cache[LFP100_STATUS2]);
		printk(KERN_INFO "%s: LFP100    INT1:0x%2.2X     INT2:0x%2.2X     INT3:0x%2.2X\n",
			__func__,
			cache[LFP100_INT1], cache[LFP100_INT2], cache[LFP100_INT3]);
	}
#endif

	if (local_chip->power_button_callback)		// call power handler
	{
		local_chip->power_button_callback();
	}
	return;
}

/*
 * unmute HP or SPK, depending on HP switch setting
 */
void lfp100_unmute_hp_sp(void)
{
	int	reg;
	int	ret;
	int	hp;
	int	i;
	int	status2;

	/* use synchronized read_reg, write_reg */

	/* read HP status switch before change */
	status2 = lfp100_read_reg(LFP100_STATUS2);
	if (status2 < 0) {
		printk(KERN_ERR "%s.%d error reading LFP100_STATUS2 (%d)\n",
			__FUNCTION__, __LINE__, status2);
		goto read_err;
	}
	hp = (status2 & LFP100_STATUS2_HP) ? 1 : 0;

	reg = lfp100_read_reg(LFP100_A_CONTROL);
	if (reg < 0) {
		printk(KERN_ERR "%s.%d error reading LFP100_A_CONTROL (%d)\n",
			__FUNCTION__, __LINE__, reg);
		goto read_err;
	}

	/* remove existing HP and SPK enable bits */
	reg &= ~(LFP100_A_CONTROL_HP_EN | LFP100_A_CONTROL_SPK_EN);

	if (hp)
		reg |= LFP100_A_CONTROL_HP_EN;
	else
		reg |= LFP100_A_CONTROL_SPK_EN;

	ret = lfp100_write_reg(LFP100_A_CONTROL, reg);

#if POLL_I2C
	/* wait for busy bit to clear */
	for (i=0; i < 200; i++) { // FIXME
		if (lfp100_read_reg(LFP100_STATUS2) &
				LFP100_STATUS2_ABUSY) {
			msleep(20);
		} else {
			break;
		}
	}
#else
	(void)i;

	/* wait for DAC to settle before returning */
	status2 = lfp100_read_reg(LFP100_STATUS2);
	wait_event_interruptible(local_chip->wait, lfp100_is_dac_ready());
#endif

read_err:
	/* release LFP100 */
	wake_up_interruptible(&local_chip->wait);
}

EXPORT_SYMBOL(lfp100_unmute_hp_sp);

/*
 * got an interrupt.  Let background task process it.
 */
static irqreturn_t lfp100_chip_irq(int irq,	void *priv)
{
	struct lfp100_chip *chip = (struct lfp100_chip *)priv;

	if (gpio_get_value(LFP100_INT) == 0) {
		/* schedule task, defer clearing interrupt condition */
		queue_work(chip->lfp100_tasks, &chip->lfp100_work);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

int lfp100_is_battery(void)
{
	int ret = 0;

	ret = ((lfp100_read_reg(LFP100_STATUS1) & LFP100_STATUS1_SOURCE_MASK) ==
		 LFP100_STATUS1_SOURCE_BAT) ? 1 : 0;
	return ret;
}
EXPORT_SYMBOL(lfp100_is_battery);

int lfp100_is_usb_present(void)
{
        int ret = 0;

        ret = ((lfp100_read_reg(LFP100_STATUS1) & LFP100_STATUS1_USB_MASK) ==
                 LFP100_STATUS1_USB) ? 1 : 0;

        return ret;
}
EXPORT_SYMBOL(lfp100_is_usb_present);

int lfp100_is_charging_active(void)
{
	int ret = 0;
	
	if(lfp100_read_reg(LFP100_CHGCONFIG0) & LFP100_CHGCONFIG0_ACTIVE)
		ret = 1;
		
	return ret;
}
EXPORT_SYMBOL(lfp100_is_charging_active);

void lfp100_set_power_button_callback(void (*callback)(void))
{
	local_chip->power_button_callback = callback;
}
EXPORT_SYMBOL(lfp100_set_power_button_callback);

int lfp100_get_power_button(void)
{
	int ret = 0;

	if (lfp100_read_reg(LFP100_STATUS1) &
		LFP100_STATUS1_PB) {
		ret = 1;
		printk(KERN_INFO "%s.%d power button status %d\n", __FUNCTION__,
			__LINE__, ret);
	}
	return ret;
}
EXPORT_SYMBOL(lfp100_get_power_button);

void lfp100_set_power_standby(void)
{
	/* go to standby mode */
	lfp100_write_reg(LFP100_CONTROL, LFP100_CONTROL_STANDBY);
}
EXPORT_SYMBOL(lfp100_set_power_standby);

void lfp100_set_power_off(void)
{
	/*go to off mode */
	lfp100_write_reg(LFP100_CONTROL, LFP100_CONTROL_OFF);
}
EXPORT_SYMBOL(lfp100_set_power_off);

int lfp100_set_vbus_draw(unsigned int mA)
{
	if (local_chip) {
		if      (mA <=  100)
			local_chip->iusb_value = LFP100_PPATH_IUSB_100MA;
		else if (mA <=  500)
			local_chip->iusb_value = LFP100_PPATH_IUSB_500MA;
		else if (mA <= 1300)
			local_chip->iusb_value = LFP100_PPATH_IUSB_1300MA;
		else if (mA <= 1800)
			local_chip->iusb_value = LFP100_PPATH_IUSB_1800MA;
		else return -ENOTSUPP;
	}
	else
	{
		return -EIO;
	}

	local_chip->have_new_vbus_draw = true;
	/* schedule task, defer clearing interrupt condition */
	queue_work(local_chip->lfp100_tasks, &local_chip->lfp100_work);
	printk(KERN_ALERT "%s vbus draw is %d mA\n", __FUNCTION__, mA);
	return 0;
}

EXPORT_SYMBOL(lfp100_set_vbus_draw);

int lfp100_set_charger_current(unsigned int mA)
{
	if (local_chip) {
		if      (mA <=  300)
			local_chip->ichrg_value = LFP100_CHGCONFIG3_ICHRG_300MA;
		else if (mA <=  400)
			local_chip->ichrg_value = LFP100_CHGCONFIG3_ICHRG_400MA;
		else if (mA <= 500)
			local_chip->ichrg_value = LFP100_CHGCONFIG3_ICHRG_500MA;
		else if (mA <= 700)
			local_chip->ichrg_value = LFP100_CHGCONFIG3_ICHRG_700MA;
		else return -ENOTSUPP;
	}
	else
	{
		return -EIO;
	}

	local_chip->have_new_ichrg_value = true;
	/* schedule task, defer clearing interrupt condition */
	queue_work(local_chip->lfp100_tasks, &local_chip->lfp100_work);
	printk(KERN_ALERT "%s charger current is %d mA\n", __FUNCTION__, mA);
	return 0;
}

EXPORT_SYMBOL(lfp100_set_charger_current);

int lfp100_have_lfp100(void)
{
	if (local_chip && local_chip->have_lfp100)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(lfp100_have_lfp100);

int lfp100_get_num_registers(void)
{
	return(local_chip->maximum_register - LFP100_FIRSTREG + 1);
}
EXPORT_SYMBOL(lfp100_get_num_registers);

u8 lfp100_get_pmic_rev(void)
{
	if(local_chip && local_chip->lfp100_rev)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(lfp100_get_pmic_rev);

void lfp100_backlight_off()
{
	lfp100_write_reg(LFP100_P_ENABLE, lfp100_read_reg(LFP100_P_ENABLE) & ~(LFP100_P_ENABLE_WLED_EN));		
}
EXPORT_SYMBOL(lfp100_backlight_off);

static int lfp100_chip_probe(struct platform_device *pdev)
{
	int i;
	int ret = 0;
	struct lfp100_chip *priv;
	u8 lfp100_ppath_reg;
	u8 lfp100_chrg_config_reg;
	u8 lfp100_int_mask3_reg = 0;

	printk(KERN_INFO "%s: init %s\n", __FUNCTION__, LFP100_NAME);

	priv = kzalloc(sizeof(struct lfp100_chip), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto fail_alloc;
	}

	platform_set_drvdata(pdev, priv);
	local_chip = priv;

	/* mark registers that are password protected or volatile */
	//		     LFP100_CHIPID        can cache, LFP100 Read-Only
	priv->reg_properties[LFP100_CONTROL]	= REG_IS_VOLATILE;
	priv->reg_properties[LFP100_STATUS1]	= REG_IS_VOLATILE;
	priv->reg_properties[LFP100_STATUS2]	= REG_IS_VOLATILE;
	priv->reg_properties[LFP100_INT1]	= REG_IS_VOLATILE;
	priv->reg_properties[LFP100_INT2]	= REG_IS_VOLATILE;
	priv->reg_properties[LFP100_INT3]	= REG_IS_VOLATILE;
	//		     LFP100_MASK1         can cache, LFP100 Read-Only
	//		     LFP100_MASK2         can cache, LFP100 Read-Only
	//		     LFP100_MASK3         can cache, LFP100 Read-Only
	//		     LFP100_WLED	  can cache, LFP100 Read-Only
	priv->reg_properties[LFP100_PPATH]	= REG_IS_VOLATILE;
	priv->reg_properties[LFP100_IO]		= REG_IS_VOLATILE;
	priv->reg_properties[LFP100_PASSWORD]	= REG_IS_VOLATILE;
	//		     LFP100_P_ENABLE	  can cache, LFP100 Read-Only
	priv->reg_properties[LFP100_DCDC1_PW]   = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_DCDC2_PW]   = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SLEW_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_LDO1_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_LDO2_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_LDO3_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_PG_PW]      = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_UVLO_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ1_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ2_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ3_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ4_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ5_PW]    = REG_IS_VOLATILE | \
						  REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_FORMAT_PW]  = REG_HAS_PASSWORD;
	//           LFP100_FILTER	  can cache, LFP100 Read-Only
	priv->reg_properties[LFP100_A_APOP_PW]  = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_A_CONTROL]	= REG_IS_VOLATILE;
	//		     LFP100_GAINSL	  can cache, LFP100 Read-Only
	priv->reg_properties[LFP100_GAINADJ_PW] = REG_HAS_PASSWORD;
	//		     LFP100_MGAIN	  can cache, LFP100 Read-Only
	//		     LFP100_VOLUME	  can cache, LFP100 Read-Only
	priv->reg_properties[LFP100_VLIMIT_PW]  = REG_HAS_PASSWORD;
	//		     LFP100_MICGAIN	  can cache, LFP100 Read-Only
	/* SP 093013 - Don't exist for P220 */
	//priv->reg_properties[LFP100_CHGCONFIG0] =  REG_IS_VOLATILE;
	//		     LFP100_CHGCONFIG1	  can cache, LFP100 Read-Only
	//		     LFP100_CHGCONFIG2	  can cache, LFP100 Read-Only
	//		     LFP100_CHGCONFIG3	  can cache, LFP100 Read-Only

	init_waitqueue_head(&priv->wait);
	spin_lock_init(&priv->lock);

	/* get LFP100 revision */
	ret = lfp100_read_reg_raw(LFP100_CHIPID);
	if (ret < 0) {
		printk(KERN_ERR "%s.%d No LFP100\n",
			__FUNCTION__, __LINE__);
		ret = -ENXIO;
		goto no_lfp100;
	}
	priv->reg_cache[LFP100_CHIPID] = ret;
	switch (priv->reg_cache[LFP100_CHIPID] & LFP100_CHIPID_CHIP_MASK) {
	case LFP100_CHIPID_CHIP_LFP100:
	case LFP100_CHIPID_CHIP_LFP200:
	case LFP100_CHIPID_CHIP_LFP210:
	case LFP100_CHIPID_CHIP_LFP220:
		printk(KERN_INFO "%s.%d Chip ID = %u\n", __FUNCTION__,__LINE__,
			(priv->reg_cache[LFP100_CHIPID] & LFP100_CHIPID_CHIP_MASK));
		priv->maximum_register = LFP100_MICGAIN;
		break;
	case LFP100_CHIPID_CHIP_LFP300:
		priv->maximum_register = LFP100_CHGCONFIG3;
		break;

	default:
		printk(KERN_ERR "%s.%d No LFP100\n",
			__FUNCTION__, __LINE__);
		ret = -ENXIO;
		goto no_lfp100;
	}

	/* initialize LFP100 cache by reading registers */
	for (i = 0; i <= priv->maximum_register; i++) {
		ret = lfp100_read_reg_raw(i);
		if (ret < 0) {
			printk(KERN_ERR "%s.%d No LFP100\n",
				__FUNCTION__, __LINE__);
			ret = -ENXIO;
			goto no_lfp100;
		}
		priv->reg_cache[i] = ret;
	}
	
	priv->have_lfp100 = 1;		/* read registers */
	priv->charger_status = 0; /* Before charging is enabled */
	priv->charging_complete = 0; /* assume battery is not fully charged till the 1st pass */
	ret = sysfs_create_group(&pdev->dev.kobj, &lfp100_attr_group);
	if (ret)
		goto sysfs_fail;

	/* request GPIO input for LFP100 IRQ */
	ret = gpio_request_one(LFP100_INT, GPIOF_IN, "LFP100 INT");
	if (ret) {
		printk(KERN_ERR "%s.%d: failed to get LFP100 GPIO %d\n",
			__FUNCTION__, __LINE__, LFP100_INT);
		goto sysfs_fail;
	}
	ret = request_irq(gpio_to_irq(LFP100_INT),
		lfp100_chip_irq, IRQF_TRIGGER_FALLING | IRQF_DISABLED, "LFP100 IRQ", priv);
	if (ret) {
		printk(KERN_ERR "%s.%d: failed to get LFP100 IRQ\n",
			__FUNCTION__, __LINE__);
		goto sysfs_fail;
	}
	disable_irq(gpio_to_irq(LFP100_INT));

	/* initialize worker thread which processes irq */
	priv->lfp100_tasks = create_singlethread_workqueue("lfp100 tasks");
	INIT_WORK(&priv->lfp100_work, lfp100_monitor_task);

	//FIXME: don't limit current on LFP300 during development
	if ((priv->reg_cache[LFP100_CHIPID] & LFP100_CHIPID_CHIP_MASK) == LFP100_CHIPID_CHIP_LFP300) {
		printk(KERN_INFO "USB not limited to 100MA at boot during LFP300 development\n");
		/* Only applicable to LFP300 Initially limit charging to 300 mA only if PMIC rev is 1P0 */
		if ((priv->reg_cache[LFP100_CHIPID] & LFP100_CHIPID_REV_MASK) == LFP100_CHIPID_CHIP_1P0)
		{
			priv->lfp100_rev = 0;
			printk(KERN_INFO "%s.%d: PMIC rev is 0x22 \n",__FUNCTION__, __LINE__);
			lfp100_chrg_config_reg = lfp100_read_reg(LFP100_CHGCONFIG3);
			lfp100_chrg_config_reg &= ~(LFP100_CHGCONFIG3_ICHRG_MASK);
			lfp100_chrg_config_reg |= LFP100_CHGCONFIG3_ICHRG_300MA;
			lfp100_write_reg(LFP100_CHGCONFIG3, lfp100_chrg_config_reg);
		}
		else
		{
			priv->lfp100_rev = 1;
			printk(KERN_INFO "%s.%d: PMIC rev is 0x42 \n",__FUNCTION__, __LINE__);
		}
	} 
	else {
		/* limit USB power source to 100ma before negotiations
		 * and turning on backlight.
		 */
		lfp100_ppath_reg = lfp100_read_reg(LFP100_PPATH);
		lfp100_ppath_reg &= ~(LFP100_PPATH_IUSB_MASK);
		lfp100_ppath_reg |= LFP100_PPATH_IUSB_100MA;
		lfp100_write_reg(LFP100_PPATH, lfp100_ppath_reg);
	}

	/* turn on LFP100 backlight */
	lfp100_write_reg(LFP100_P_ENABLE,
		lfp100_read_reg(LFP100_P_ENABLE) | LFP100_P_ENABLE_WLED_EN);
	lfp100_write_reg(LFP100_VOLUME, 0x00); // init volume 0db
	/* setup LFP100 IRQ for USB, AC, power button, charger or headphone changes */
	lfp100_write_reg(LFP100_MASK1, 0xFF);
	lfp100_write_reg(LFP100_MASK2,
		~(LFP100_MASK2_USBM | LFP100_MASK2_ACM | LFP100_MASK2_PBM));
		
	if ((priv->reg_cache[LFP100_CHIPID] & LFP100_CHIPID_CHIP_MASK) == LFP100_CHIPID_CHIP_LFP300){
		lfp100_int_mask3_reg = ~((LFP100_MASK3_HP | LFP100_MASK3_ABUSY) | LFP100_MASK3_CHARGERM);
	}
	else {
		lfp100_int_mask3_reg = ~((LFP100_MASK3_HP | LFP100_MASK3_ABUSY));
	}	
	lfp100_write_reg(LFP100_MASK3, lfp100_int_mask3_reg);

	/* If LFP300 then enable charger */
	if ((lfp100_read_reg(LFP100_CHIPID) & LFP100_CHIPID_CHIP_MASK)
		== LFP100_CHIPID_CHIP_LFP300) {
		lfp100_write_reg(LFP100_CHGCONFIG1,
			lfp100_read_reg(LFP100_CHGCONFIG1) | LFP100_CHGCONFIG1_CHG_EN);
		printk(KERN_INFO "%s.%d:%s  Enable Charger\n",
			__FILE__, __LINE__, __FUNCTION__);
	}

	/* clear any pending interrupts */
	lfp100_read_reg(LFP100_INT1);
	lfp100_read_reg(LFP100_INT2);
	lfp100_read_reg(LFP100_INT3);

	/* Register dump -- COMMENTED OUT 4/21/14, save time on boot.
	for (i = 0; i <= priv->maximum_register; i++) {
		ret = lfp100_read_reg_raw(i);
		if (ret > 0) {
			printk(KERN_INFO "%s: reg %02X: %02X\n", __FUNCTION__, i, ret);
		}
	}
	*/
	
	/* setup IRQ */
	printk(KERN_INFO "%s: gpio_to_irq(%d) = %d\n", __FUNCTION__, LFP100_INT, gpio_to_irq(LFP100_INT));
	enable_irq(gpio_to_irq(LFP100_INT));
	printk(KERN_INFO "%s: success %s\n", __FUNCTION__, LFP100_NAME);

	return 0;

no_lfp100:
	priv->have_lfp100 = 0;
	return ret;

sysfs_fail:
	gpio_free(LFP100_INT);
	sysfs_remove_group(&pdev->dev.kobj, &lfp100_attr_group);
	kfree(priv);
fail_alloc:
	local_chip = NULL;
	return ret;
}

static int lfp100_chip_remove(struct platform_device *pdev)
{
	disable_irq(gpio_to_irq(LFP100_INT));
	free_irq(gpio_to_irq(LFP100_INT), local_chip);
	gpio_free(LFP100_INT);

	destroy_workqueue(local_chip->lfp100_tasks);

	sysfs_remove_group(&pdev->dev.kobj, &lfp100_attr_group);
	kfree(local_chip);
	local_chip = NULL;
	return 0;
}

static struct platform_driver lfp100_chip_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "lfp100-chip",
	},
	.probe		= lfp100_chip_probe,
	.remove		= lfp100_chip_remove,
};

/*
 * module stuff
 */

static int __init lfp100_chip_init(void)
{
	printk(KERN_INFO "%s: \n", __FUNCTION__);
	return platform_driver_register(&lfp100_chip_driver);
}

static void lfp100_chip_exit(void)
{
	printk(KERN_INFO "%s: \n", __FUNCTION__);
	platform_driver_unregister(&lfp100_chip_driver);
}

MODULE_AUTHOR("Scott Esters");
MODULE_DESCRIPTION("LFP100 support");
MODULE_LICENSE("GPL");

module_init(lfp100_chip_init);
module_exit(lfp100_chip_exit);
