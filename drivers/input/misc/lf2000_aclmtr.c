/*
 * drivers/input/misc/lf1000_aclmtr.c
 *
 * Accelerometer driver for the LF2000 platform.
 * Generically named to fit input driver framework.
 * Supports Bosch BMA150, BMA220 devices via I2C.
 *
 * Copyright 2010 LeapFrog Enterprises Inc.
 *
 * Dave Milici <dmilici@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <mach/platform.h>
#include <mach/platform_id.h>
#include <mach/gpio.h>
#include <mach/devices.h>


/*
 * device
 */

#define INPUT_SAMPLING_HZ		10
#define INPUT_SAMPLING_JIFFIES	(HZ / INPUT_SAMPLING_HZ)

#define BMA150_ADDR			(0x70 >> 1)
#define BMA220_ADDR			(0x16 >> 1)
#define BMA222_ADDR			(0x10 >> 1)
#define BMA222E_STD_ADDR	(0x30 >> 1)
#define BMA222E_ALT_ADDR	(0x32 >> 1)
#define BMA222E_ADDR		BMA222E_STD_ADDR

#define MIN_XYZ				-(0x001F+1)
#define MAX_XYZ				0x001F

#define MIN_PHI				0x00
#define MAX_PHI				0x07
#define FLAT_MASK			0x08
#define NUM_CAL				10
#define MIN_CAL				0x00010000
#define MAX_CAL				0x00400000

/* BMA222/BMA222E register/bit definitions */
#define NEW_DATA			0x01  	//1: new data available, 0: new data not available
#define BMA222_ACCD_X_LSB	0x02	//bit 0 = NEW_DATA
#define BMA222_ACCD_X_MSB	0x03
#define BMA222_ACCD_Y_LSB	0x04	//bit 0 = NEW_DATA
#define BMA222_ACCD_Y_MSB	0x05
#define BMA222_ACCD_Z_LSB	0x06	//bit 0 = NEW_DATA
#define BMA222_ACCD_Z_MSB	0x07


struct lf1000_aclmtr {
	struct input_dev *input;
	struct timer_list input_timer;

	struct	workqueue_struct *input_tasks;
	struct	work_struct       input_work;

	struct i2c_client  *control_data;
	struct i2c_adapter *i2c_dev;
	unsigned int		i2c_bus;
	unsigned int		i2c_addr;

	unsigned int do_enable;
	unsigned int do_orient;
	unsigned int do_tick;
	unsigned int rate;
	unsigned int rate_jiffies;
	unsigned int average;
	int biasx, biasy, biasz;
	int	x, y, z, phi;
	int scalex, scaley, scalez;
	int do_raw_only;
	int calibration[NUM_CAL];
	int isflat;
};

static int bma_write_reg(struct lf1000_aclmtr *dev, unsigned int reg,
		unsigned int value)
{
	struct i2c_adapter *adapter = i2c_get_adapter(dev->i2c_bus);
	struct i2c_msg msg;
	char buf[2];
	int ret;

	/* BMA220 left-justified index */
	if (BMA220_ADDR == dev->i2c_addr)
		reg <<= 1;

	buf[0] = reg & 0xFF;
	buf[1] = value & 0xFF;

	msg.addr = dev->i2c_addr;
	msg.buf = buf;
	msg.len = 2;
	msg.flags = 0; /* write */

	ret = i2c_transfer(adapter, &msg, 1);

	i2c_put_adapter(adapter);

	if (ret < 0)
		return -EIO;

	return 0;
}

static unsigned int bma_read_reg(struct lf1000_aclmtr *dev,
		unsigned int reg)
{
	struct i2c_adapter *adapter = i2c_get_adapter(dev->i2c_bus);
	struct i2c_msg msg[2];
	char buf[2];
	int ret;

	/* BMA220 left-justified index */
	if (BMA220_ADDR == dev->i2c_addr)
		reg <<= 1;

	buf[0] = reg & 0xFF;
	buf[1] = 0;

	msg[0].addr = dev->i2c_addr;
	msg[0].buf = buf;
	msg[0].len = 1;
	msg[0].flags = 0; /* write */

	msg[1].addr = dev->i2c_addr;
	msg[1].buf = &buf[1];
	msg[1].len = 1; //2;
	msg[1].flags = I2C_M_RD;

	ret = i2c_transfer(adapter, msg, 2);

	i2c_put_adapter(adapter);

	return (ret < 0) ? ret : buf[1];
}

static int bma_detect(struct lf1000_aclmtr* dev)
{
	int id[2];
	int n = 0;
	int bus = 0;

	/* Madrid, Emerald CIP, L2K, Cabo, (Lima, Glasgow) */
	switch (get_leapfrog_platform()) {
	case BOGOTA:
		bus = 1;
		break;
	case CABO:
		bus = 1;
		break;
	case GLASGOW:
		/* to be added when known*/
		break;
	case LIMA:
		/* to be added when known */
		break;
	case LUCY:
		bus = 0;
		break;
	case RIO:
		bus = 1;
		break;
	case VALENCIA:
		bus = 1;
		break;
	case XANADU:
		bus = 1;
		break;
	case UNKNOWN:
		break;

	}

	for (n = bus; n < 2; n++) {
		dev->i2c_bus = n;
		dev->i2c_addr = BMA222E_ADDR;
		id[0] = bma_read_reg(dev, 0x00);
		if (0xF8 == id[0]) {
			printk(KERN_INFO "%s: BMA222E device found\n", __FUNCTION__);
			bma_write_reg(dev, 0x14, 0xB6);	/* soft reset */
			msleep(2);  /*give 2ms for reset, accl specs suggest max 1.8mSec. Make sure this does not cause any trouble!*/
			bma_write_reg(dev, 0x16, 0xC7);	/* flat and orient enable */
			bma_write_reg(dev, 0x17, 0x10);	/* data enable */
			bma_write_reg(dev, 0x13, 0x00);	/* filtered */
			//bma_write_reg(dev, 0x10, 0x08);	/* bandwidth = 7.5Hz */
			bma_write_reg(dev, 0x10, 0x0C);	/* bandwidth = 125Hz */
			dev->scalex = dev->scaley = dev->scalez = 4;
			if (get_leapfrog_platform() == LUCY)
				dev->scalex = dev->scalez *= -1;
			if (get_leapfrog_platform() == CABO ||
				get_leapfrog_platform() == BOGOTA ||
				get_leapfrog_platform() == XANADU)
                                dev->scalez *= -1;
			return 1;
		}

		dev->i2c_bus = n;
		dev->i2c_addr = BMA220_ADDR;
		id[0] = bma_read_reg(dev, 0x00);
		id[1] = bma_read_reg(dev, 0x01);

		if (0xDD == id[0] && 0x00 == id[1]) {
			printk(KERN_INFO "%s: BMA220 device found\n", __FUNCTION__);
			bma_write_reg(dev, 0x0D, 0xC0);	/* data mode & orient enable */
			bma_write_reg(dev, 0x0F, 0x07);	/* x,y,z axis enable */
			if (get_leapfrog_platform() == LUCY)
				dev->scalex = dev->scalez *= -1;
			return 1;
		}

		dev->i2c_bus = n;
		dev->i2c_addr = BMA222_ADDR;
		id[0] = bma_read_reg(dev, 0x00);

		if (0x03 == id[0]) {
			printk(KERN_INFO "%s: BMA222 device found\n", __FUNCTION__);
			bma_write_reg(dev, 0x16, 0x47);	/* orient enable */
			bma_write_reg(dev, 0x17, 0x10);	/* data enable */
			bma_write_reg(dev, 0x13, 0x00);	/* filtered */
			bma_write_reg(dev, 0x10, 0x08);	/* bandwidth */
			dev->scalex = dev->scaley = dev->scalez = 4;
			if (get_leapfrog_platform() == LUCY)
				dev->scalex = dev->scalez *= -1;
			/* The accelerometer on Cabo is mounted upside down 
			compared to Rio and so scalez needs to have the 
        		signed reversed for compatibility so the UI 
		        behaves as expected */

			if (get_leapfrog_platform() == CABO ||
				get_leapfrog_platform() == BOGOTA ||
				get_leapfrog_platform() == XANADU)
                                dev->scalez *= -1;
			return 1;
		}

	}

#if 0
	/* Acorn */
	if (gpio_have_gpio_acorn()) {
		dev->i2c_bus = 0;
		dev->i2c_addr = BMA150_ADDR;
		id[0] = bma_read_reg(dev, 0x00);
		id[1] = bma_read_reg(dev, 0x01);

		if (0x02 == id[0] && 0x11 == id[1]) {
			printk(KERN_INFO "%s: BMA150 device found\n", __FUNCTION__);
			return 1;
		}
	}
#endif

	printk(KERN_INFO "%s: device not found\n", __FUNCTION__);
	return 0;
}

static void bma150_get_xyz(struct lf1000_aclmtr* dev, int* x, int* y, int* z)
{
	*x = (bma_read_reg(dev, 0x02) >> 6) | (bma_read_reg(dev, 0x03) << 2);
	*y = (bma_read_reg(dev, 0x04) >> 6) | (bma_read_reg(dev, 0x05) << 2);
	*z = (bma_read_reg(dev, 0x06) >> 6) | (bma_read_reg(dev, 0x07) << 2);
	if (*x > 0x01FF)
		*x -= 0x3FF+1;
	if (*y > 0x01FF)
		*y -= 0x3FF+1;
	if (*z > 0x01FF)
		*z -= 0x3FF+1;
	dev->x = *x;
	dev->y = *y;
	dev->z = *z;
}

static void bma220_get_xyz(struct lf1000_aclmtr* dev, int* x, int* y, int* z)
{
	*x = bma_read_reg(dev, 0x02) >> 2;
	*y = bma_read_reg(dev, 0x03) >> 2;
	*z = bma_read_reg(dev, 0x04) >> 2;
	if (*x > 0x001F)
		*x -= 0x03F+1;
	if (*y > 0x001F)
		*y -= 0x03F+1;
	if (*z > 0x001F)
		*z -= 0x03F+1;
	dev->x = *x;
	dev->y = *y;
	dev->z = *z;
}

static void bma222_get_xyz(struct lf1000_aclmtr* dev, int* x, int* y, int* z)
{
/*FWBOG-944, read NEW_DATA flag to make sure correct data is available.
 * The data might be read as 0xFF if we read data when this flag is not set.
 * do not update the device data if NEW_DAT flag does not get set after 10 reads,
 * so that wrong data does not get posted */
	int count=10;
	while(count){
		if(bma_read_reg(dev, BMA222_ACCD_X_LSB)& NEW_DATA){
			*x = bma_read_reg(dev, BMA222_ACCD_X_MSB);
			if (*x > 0x007F)
					*x -= 0x100;
			dev->x = *x;
			break;
		}
		else{
			count--;
			if(count==0)
				printk(KERN_ERR "%s: New data not available\n", __FUNCTION__);
		}
	}
	count=10;
	while(count){
		if(bma_read_reg(dev, BMA222_ACCD_Y_LSB)& NEW_DATA){
			*y = bma_read_reg(dev, BMA222_ACCD_Y_MSB);
			if (*y > 0x007F)
				*y -= 0x100;
			dev->y = *y;
			break;
		}
		else{
			count--;
			if(count==0)
				printk(KERN_ERR "%s: New data not available\n", __FUNCTION__);
		}
	}
	count=10;
	while(count){
		if(bma_read_reg(dev, BMA222_ACCD_Z_LSB)& NEW_DATA) {
			*z = bma_read_reg(dev, BMA222_ACCD_Z_MSB);
			if (*z > 0x007F)
					*z -= 0x100;
			dev->z = *z;
			break;
		}
		else{
			count--;
			if(count==0)
				printk(KERN_ERR "%s: New data not available\n", __FUNCTION__);
		}
	}
}

static void get_orient(struct lf1000_aclmtr* dev, int* orient)
{
	switch (dev->i2c_addr) {
	case BMA220_ADDR:
		*orient = bma_read_reg(dev, 0x0B) >> 4;
		*orient &= MAX_PHI;
		dev->phi = *orient;
		return;
	case BMA222_ADDR:
	case BMA222E_ADDR:
		*orient = bma_read_reg(dev, 0x0C) >> 4;
		dev->isflat = *orient & FLAT_MASK;	
		*orient &= MAX_PHI;
		dev->phi = *orient;
		return;
	}
	dev->phi = *orient = 0;
}

static void get_xyz(struct lf1000_aclmtr* dev, int* x, int* y, int* z)
{
	switch (dev->i2c_addr) {
	case BMA150_ADDR:
		return bma150_get_xyz(dev, x, y, z);
	case BMA220_ADDR:
		return bma220_get_xyz(dev, x, y, z);
	case BMA222_ADDR:
	case BMA222E_ADDR:
		return bma222_get_xyz(dev, x, y, z);
	}
	*x = *y = *z = 0;
}

inline int ez_scale(int x, int k)
{
	switch (k) {
	case  1:	return x;
	case -1:	return -x;
	case  4:	return x >> 2;
	case -4:	return -x >> 2;
	case  0:	return x;
	default:	return x / k;	// k != 0
	}
}

static int calibrate(struct lf1000_aclmtr* dev)
{
	/* VALENCIA: {+1G, 0, 0} , {0 -1G, 0} , {0, 0, -1G} , 64K */
	/* LUCY    : {-1G, 0, 0} , {0 -1G, 0} , {0, 0, +1G} , 64K */
	int *x = &dev->calibration[0];
	if (x[9] != MIN_CAL)
		return 0;
	/* 1G in scale range? ~15 for BMA220, ~63 for BMA222 */
	if (ez_scale(x[0] >> 16, dev->scalex) < +7)
		return 0;
	if (ez_scale(x[4] >> 16, dev->scaley) > -7)
		return 0;
	if (ez_scale(x[8] >> 16, dev->scalez) > -7)
		return 0;
	/* calculate average bias from 2 calibration samples */
	dev->biasx = ez_scale((x[6] + x[3]) >> 17, dev->scalex);
	dev->biasy = ez_scale((x[7] + x[1]) >> 17, dev->scaley);
	dev->biasz = ez_scale((x[5] + x[2]) >> 17, dev->scalez);
	return 1;
}

static struct lf1000_aclmtr* g_dev = NULL;	/* not cached in work_struct */
static void input_work_task(struct	work_struct *work)
{
	struct lf1000_aclmtr *i_dev = g_dev;
	int x, y, z, t;
	int orient = 0;
	static int tick=0;
	static int acount=0;
	static int sx=0, sy=0, sz=0;

	/* get x,y,z from device */
	get_xyz(i_dev, &x, &y, &z);

	/* The accelerometer on Cabo is mounted upside down and turned 90 degrees 
	compared to Rio. The x and y values needs to be swapped so it is compatible 
	to Rio and the UI behaves as expected */  

	if (get_leapfrog_platform() == CABO ||
		get_leapfrog_platform() == BOGOTA ||
		get_leapfrog_platform() == XANADU) {
		t = x;
		x = y;
		y = t;	
	}

	if (i_dev->scalex)
		x = ez_scale(x, i_dev->scalex);
	if (i_dev->scaley)
		y = ez_scale(y, i_dev->scaley);
	if (i_dev->scalez)
		z = ez_scale(z, i_dev->scalez);

	/* get orientation from device */
	if (i_dev->do_orient) {
		get_orient(i_dev, &orient);

		if(i_dev->isflat)
			orient = 1;
		if (get_leapfrog_platform() == CABO ||
			get_leapfrog_platform() == BOGOTA ||
			get_leapfrog_platform() == XANADU) {
		 /* swizzle orientation to rectify effect of incorrect mounting*/
                                switch (orient) {
                               	case 0: orient = 7; break;
	                        case 1: orient = 6; break;
        	                case 4: orient = 3; break;
                	        case 5: orient = 2; break;
                                case 2: orient = 5; break;
                               	case 3: orient = 4; break;
	                        case 6: orient = 1; break;
        	                case 7: orient = 0; break;
                                }       
		} else {

			/* swizzle orientation per x,y sign */
			if (i_dev->scalex < 0) {
				switch (orient) {
				case 0: orient = 1; break;
				case 1: orient = 0; break;
				case 4: orient = 5; break;
				case 5: orient = 4; break;
				}
			}	
			if (i_dev->scaley < 0) {
				switch (orient) {
				case 2: orient = 3; break;
				case 3: orient = 2; break;
				case 6: orient = 7; break;
				case 7: orient = 6; break;
				}
			}
		}
	}

	if (i_dev->average > 1)
	{
		sx += x;
		sy += y;
		sz += z;
		if (++acount >= i_dev->average)
		{
			int a=i_dev->average; /* Make signed */
			x = (sx-i_dev->biasx)/a;
			y = (sy-i_dev->biasy)/a;
			z = (sz-i_dev->biasz)/a;
			/* Clear accumulators */
			acount = 0;
			sx = sy = sz = 0;
		}
		else
			return; /* No report */
	}
	else
	{
		x -= i_dev->biasx;
		y -= i_dev->biasy;
		z -= i_dev->biasz;
	}
		
	/* report input data */
	input_report_abs(i_dev->input, ABS_X, x);
	input_report_abs(i_dev->input, ABS_Y, y);
	input_report_abs(i_dev->input, ABS_Z, z);
	/* orientation is optional */
	if (i_dev->do_orient)
		input_report_abs(i_dev->input, ABS_MISC, orient);
	/* Force at least one changing value to get a new event every sample */
	if (i_dev->do_tick)
		input_report_abs(i_dev->input, ABS_WHEEL, tick++);
	input_sync(i_dev->input);
}

static void input_monitor_task(unsigned long data)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)data;

	/* defer input sampling to work queue */
	if (i_dev->do_enable)
		queue_work(i_dev->input_tasks, &i_dev->input_work);

	/* reset task timer */
	i_dev->input_timer.expires += i_dev->rate_jiffies;
	i_dev->input_timer.function = input_monitor_task;
	i_dev->input_timer.data = data;
	add_timer(&i_dev->input_timer);
}

/*
 * sysfs Interface
 */

static ssize_t show_tick(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->do_tick);
}
static ssize_t set_tick(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	i_dev->do_tick = temp;
	return(count);
}

static DEVICE_ATTR(tick, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_tick, set_tick);

static ssize_t show_rate(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->rate);
}
static ssize_t set_rate(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp <= 0 || temp > HZ)
		return -EINVAL;
	i_dev->rate = temp;
	i_dev->rate_jiffies = HZ / temp;
	return(count);
}

static DEVICE_ATTR(rate, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_rate, set_rate);

static ssize_t show_average(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->average);
}
static ssize_t set_average(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	i_dev->average = temp;
	return(count);
}

static DEVICE_ATTR(average, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_average, set_average);

static ssize_t show_bias(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d %d %d\n", 
		       i_dev->biasx, i_dev->biasy, i_dev->biasz);
}
static ssize_t set_bias(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int tx, ty, tz;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%d %d %d", &tx, &ty, &tz) != 3)
		return -EINVAL;
	i_dev->biasx = tx;
	i_dev->biasy = ty;
	i_dev->biasz = tz;
	return(count);
}

static DEVICE_ATTR(bias, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_bias, set_bias);

static ssize_t show_scale(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d %d %d\n",
		       i_dev->scalex, i_dev->scaley, i_dev->scalez);
}
static ssize_t set_scale(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int sx, sy, sz;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%d %d %d", &sx, &sy, &sz) != 3)
		return -EINVAL;
	i_dev->scalex = sx;
	i_dev->scaley = sy;
	i_dev->scalez = sz;
	return(count);
}

static DEVICE_ATTR(scale, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_scale, set_scale);

static ssize_t show_enable(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->do_enable);
}
static ssize_t set_enable(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	i_dev->do_enable = temp;
	return(count);
}

static DEVICE_ATTR(enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_enable, set_enable);

static ssize_t show_orient(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->do_orient);
}
static ssize_t set_orient(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	i_dev->do_orient = temp;
	return(count);
}

static DEVICE_ATTR(orient, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_orient, set_orient);

static ssize_t show_raw_xyz(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d %d %d\n", i_dev->x, i_dev->y, i_dev->z);
}

static DEVICE_ATTR(raw_xyz, S_IRUSR|S_IRGRP|S_IROTH, show_raw_xyz, NULL);

static ssize_t show_raw_phi(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->phi);
}

static DEVICE_ATTR(raw_phi, S_IRUSR|S_IRGRP|S_IROTH, show_raw_phi, NULL);

static ssize_t show_raw_only(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->do_raw_only);
}
static ssize_t set_raw_only(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	i_dev->do_raw_only = temp;
	if (i_dev->do_raw_only) {
		i_dev->biasx  = i_dev->biasy  = i_dev->biasz  = 0;
		i_dev->scalex = i_dev->scaley = i_dev->scalez = 0;
	}
	else if (  i_dev->i2c_addr == BMA222_ADDR
			|| i_dev->i2c_addr == BMA222E_ADDR ) {
		i_dev->biasx  = i_dev->biasy  = i_dev->biasz  = 0;
		i_dev->scalex = i_dev->scaley = i_dev->scalez = 4;
		if (get_leapfrog_platform() == LUCY)
			i_dev->scalex = i_dev->scalez *= -1;
		if (get_leapfrog_platform() == CABO ||
			get_leapfrog_platform() == BOGOTA ||
			get_leapfrog_platform() == XANADU)
                        i_dev->scalez *= -1;
		calibrate(i_dev);
	}
	else {
		i_dev->biasx  = i_dev->biasy  = i_dev->biasz  = 0;
		i_dev->scalex = i_dev->scaley = i_dev->scalez = 1;
		if (get_leapfrog_platform() == LUCY)
			i_dev->scalex = i_dev->scalez *= -1;
		if (get_leapfrog_platform() == CABO ||
			get_leapfrog_platform() == BOGOTA ||
			get_leapfrog_platform() == XANADU)
                        i_dev->scalez *= -1;
		calibrate(i_dev);
	}
	return(count);
}

static DEVICE_ATTR(raw_only, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_raw_only, set_raw_only);

static ssize_t show_calibration(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	int *x = &i_dev->calibration[0];
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d\n", x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9]);
}
static ssize_t set_calibration(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	int x[NUM_CAL];
	if (sscanf(buf, "%d %d %d %d %d %d %d %d %d %d", &x[0], &x[1], &x[2], &x[3], &x[4], &x[5], &x[6], &x[7], &x[8], &x[9]) != NUM_CAL)
		return -EINVAL;
	if (x[NUM_CAL-1] != MIN_CAL)
		return -EINVAL;
	memcpy(&i_dev->calibration[0], &x[0], sizeof(x));
	calibrate(i_dev);
	return(count);
}

static DEVICE_ATTR(calibration, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_calibration, set_calibration);

static struct attribute *aclmtr_attributes[] = {
	&dev_attr_tick.attr,
	&dev_attr_rate.attr,
	&dev_attr_average.attr,
	&dev_attr_bias.attr,
	&dev_attr_scale.attr,
	&dev_attr_enable.attr,
	&dev_attr_orient.attr,
	&dev_attr_raw_xyz.attr,
	&dev_attr_raw_phi.attr,
	&dev_attr_raw_only.attr,
	&dev_attr_calibration.attr,
	NULL
};

static struct attribute_group aclmtr_attr_group = {
	.attrs = aclmtr_attributes
};


/*
 * platform device
 */

static int lf1000_aclmtr_probe(struct platform_device *pdev)
{
	struct lf1000_aclmtr *lf1000_aclmtr_dev;
	struct input_dev *input_dev;
	int ret;

	lf1000_aclmtr_dev = kzalloc(sizeof(struct lf1000_aclmtr), GFP_KERNEL);
	if (!lf1000_aclmtr_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, lf1000_aclmtr_dev);

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		goto fail_input;
	}

	input_dev->name = "LF2000 Accelerometer";
	input_dev->phys = "lf2000/aclmtr";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;
	lf1000_aclmtr_dev->input = input_dev;

	lf1000_aclmtr_dev->do_enable = 0;
	lf1000_aclmtr_dev->do_orient = 0;
	lf1000_aclmtr_dev->do_tick = 0;
	lf1000_aclmtr_dev->rate = INPUT_SAMPLING_HZ;
	lf1000_aclmtr_dev->rate_jiffies = INPUT_SAMPLING_JIFFIES;
	lf1000_aclmtr_dev->average = 1;
	lf1000_aclmtr_dev->biasx = 0;
	lf1000_aclmtr_dev->biasy = 0;
	lf1000_aclmtr_dev->biasz = 0;
	lf1000_aclmtr_dev->scalex = 1;
	lf1000_aclmtr_dev->scaley = 1;
	lf1000_aclmtr_dev->scalez = 1;
	
	/* event types that we support */
	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	input_dev->absbit[0] = BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) |
		BIT_MASK(ABS_Z) | BIT_MASK(ABS_WHEEL) | BIT_MASK(ABS_MISC);
	input_set_abs_params(input_dev, ABS_X, MIN_XYZ, MAX_XYZ, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, MIN_XYZ, MAX_XYZ, 0, 0);
	input_set_abs_params(input_dev, ABS_Z, MIN_XYZ, MAX_XYZ, 0, 0);
	input_set_abs_params(input_dev, ABS_WHEEL, 0, 0x7FFFFFFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MISC, MIN_PHI, MAX_PHI, 0, 0);

	ret = input_register_device(lf1000_aclmtr_dev->input);
	if (ret)
		goto fail_register;

	/* query for accelerometer device */
	ret = bma_detect(lf1000_aclmtr_dev);
	if (!ret) {
		ret = -ENODEV;
		goto fail_detect;
	}

	/* create work queue to defer input sampling */
	lf1000_aclmtr_dev->input_tasks = create_singlethread_workqueue("accelerometer-tasks");
	INIT_WORK(&lf1000_aclmtr_dev->input_work, input_work_task);
	g_dev = lf1000_aclmtr_dev;

	/* create periodic input task */
	setup_timer(&lf1000_aclmtr_dev->input_timer, input_monitor_task, (unsigned long)lf1000_aclmtr_dev);
	lf1000_aclmtr_dev->input_timer.expires = get_jiffies_64() + 
		lf1000_aclmtr_dev->rate_jiffies;
	lf1000_aclmtr_dev->input_timer.function = input_monitor_task;
	lf1000_aclmtr_dev->input_timer.data = (unsigned long)lf1000_aclmtr_dev;
	add_timer(&lf1000_aclmtr_dev->input_timer);

	/* create sysfs entries */
	ret = sysfs_create_group(&pdev->dev.kobj, &aclmtr_attr_group);

	return 0;

fail_detect:
	input_unregister_device(input_dev);
fail_register:
	input_free_device(input_dev);
fail_input:
	kfree(lf1000_aclmtr_dev);
	return ret;
}

static int lf1000_aclmtr_remove(struct platform_device *pdev)
{
	struct lf1000_aclmtr *lf1000_aclmtr_dev = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &aclmtr_attr_group);
	del_timer_sync(&lf1000_aclmtr_dev->input_timer);
	destroy_workqueue(lf1000_aclmtr_dev->input_tasks);
	input_unregister_device(lf1000_aclmtr_dev->input);
	kfree(lf1000_aclmtr_dev);

	return 0;
}

static struct platform_driver lf1000_aclmtr_driver = {
	.probe		= lf1000_aclmtr_probe,
	.remove		= lf1000_aclmtr_remove,
	.driver		= {
		.name		= DEV_NAME_LF_ACLMTR,
	},
};

/*
 * module stuff
 */

static int __devinit lf1000_aclmtr_init(void)
{
	return platform_driver_register(&lf1000_aclmtr_driver);
}

static void __exit lf1000_aclmtr_exit(void)
{
	platform_driver_unregister(&lf1000_aclmtr_driver);
}

module_init(lf1000_aclmtr_init);
module_exit(lf1000_aclmtr_exit);

MODULE_AUTHOR("Dave Milici <dmilici@leapfrog.com>");
MODULE_DESCRIPTION("LF2000 Accelerometer driver");
MODULE_LICENSE("GPL");
