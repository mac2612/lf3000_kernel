/* hwmon-lf1000.c
 *
 * Power and Battery monitoring.  This driver provides battery and external
 * power information, as well as an input device for the Power button.
 * 
 * Scott Esters <sesters@leapfrog.com>
 *
 * Copyright 2008 LeapFrog Enterprises Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/lf3000/gpio.h>
#include <linux/interrupt.h>

#include <mach/platform.h>
#include <plat/power.h>
#include <mach/lfp100.h>
#include <mach/tc7734.h>
#include <mach/adc.h>
#include <mach/soc.h>
#include <mach/bq24250-charger.h>

#ifdef CONFIG_ARCH_LF1000
#include <mach/adc.h>
#include <mach/gpio.h>
#include <mach/gpio_hal.h>
#include <mach/common.h>
#include <mach/clkpwr.h>
#endif

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system_info.h>

/* FIXME */
#include "mach/board_revisions.h"

#include "nx_alive.h"
#include "alive.h"

/*
 * power hardware
 */

enum glasgow_power_chip {
	UNDEFINED = 0,
	LF1000 = 1,	/* uses LF1000 power toggle input pin */
	LFP100 = 2,	/* using LFP100 power management chip */
	TC7734 = 3,
};

/*
 * configuration
 */

#define POWER_BUTTON_SAMPLING_J	(HZ/4)

#define SHUTDOWN_SECS	3
/* power down system after SHUTDOWN_SECS */
#define POWER_BUTTON_COUNT  ((HZ * SHUTDOWN_SECS) / POWER_BUTTON_SAMPLING_J)

/* platform device data */
struct glasgow_hwmon {
	enum lf1000_power_status status;	/* board status	*/
	
	unsigned shutdown : 1;			/* shutdown requested */

	struct platform_device *pdev;

	struct work_struct battery_work;	/* monitor power */
	struct workqueue_struct *battery_tasks;
	struct timer_list battery_timer;

	struct work_struct power_button_work;	/* monitor power button*/
	struct workqueue_struct *power_button_tasks;
	struct timer_list power_button_timer;

	int	power_button_count;		/* count power button calls */

	enum	glasgow_power_chip chip;		/* support LF1000 / LFP100 / TC7734 */

	/* input device interface */
	unsigned char buttons[7];		/* one slot for each state */
	struct input_dev *input;
};

static struct glasgow_hwmon *hwmon_dev = NULL;

/*
 * sysfs Interface
 */

/* report whether shutdown was requested */
static ssize_t show_shutdown(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct glasgow_hwmon *priv = (struct glasgow_hwmon *)dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", priv->shutdown);
}
/* set shutdown to requested value */
static ssize_t store_shutdown(struct device *pdev, struct device_attribute *attr,
		                      const char *buf, size_t count)
{
	int temp;
	struct glasgow_hwmon *priv = (struct glasgow_hwmon *)dev_get_drvdata(pdev);

	if (sscanf(buf, "%d", &temp) != 1)
		return -EINVAL;

	if (temp != 0 && temp != 1)
		return -EINVAL;

	priv->shutdown = temp;
	if(hwmon_dev->shutdown)
		dev_info(&hwmon_dev->pdev->dev, "shutdown requested \n");

	return(count);
}
static DEVICE_ATTR(shutdown, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH,
		show_shutdown, store_shutdown);

static struct attribute *power_attributes[] = {
	&dev_attr_shutdown.attr,
	NULL
};

static struct attribute_group power_attr_group = {
	.attrs = power_attributes
};

static void power_button_monitoring_task(unsigned long data)
{
	struct glasgow_hwmon *priv = (struct glasgow_hwmon *)data;
	queue_work(priv->power_button_tasks, &priv->power_button_work);
	
	/* reset task timer */
	priv->power_button_timer.expires += POWER_BUTTON_SAMPLING_J;
	priv->power_button_timer.function = power_button_monitoring_task;
	priv->power_button_timer.data = data;
	add_timer(&priv->power_button_timer);
}

/*
 * read the system power button status
 */
static int get_power_button(int chip)
{
	int ret = 0;
	
	if(NX_ALIVE_GetVDDPWRTOGGLE())
	{
		ret = 1;
	}
	return ret;
}


/*
 * handle power button, shutdown if pressed for SHUTDOWN_SECS
 */

static void glasgow_power_button(struct work_struct *work)
{
	//printk(KERN_INFO "%s: %d\n", __FUNCTION__, __LINE__);

	int power_button = get_power_button(hwmon_dev->chip);

	/* report event if power button just pressed */
	if (!hwmon_dev->power_button_count && power_button) {	
		printk(KERN_INFO "%s: %d \n", __FUNCTION__, __LINE__);
		input_report_key(hwmon_dev->input, KEY_POWER, 1);
		input_sync(hwmon_dev->input);
		dev_info(&hwmon_dev->pdev->dev, "Power button pressed\n");
		hwmon_dev->shutdown = 1;
		if(hwmon_dev->shutdown)
			dev_info(&hwmon_dev->pdev->dev, "shutdown requested \n");
	}

	/* report if power button is released */
	if (hwmon_dev->power_button_count < POWER_BUTTON_COUNT &&
		!power_button) {

		/* report event if power button just released */
		if (hwmon_dev->power_button_count) {
			input_report_key(hwmon_dev->input, KEY_POWER, 0);
			input_sync(hwmon_dev->input);
			dev_info(&hwmon_dev->pdev->dev,
				"Power button released\n");
		}

		hwmon_dev->power_button_count = 0; /* reset count */ 
		return;
	}

	/* button still pressed, run task up to POWER_BUTTON_COUNT times */
	if (hwmon_dev->power_button_count++ <= POWER_BUTTON_COUNT) {
		printk(KERN_INFO "%s: %d power button count = %d \n", __FUNCTION__, __LINE__, hwmon_dev->power_button_count);
		hwmon_dev->power_button_timer.expires +=
			 POWER_BUTTON_SAMPLING_J;
		hwmon_dev->power_button_timer.function =
			power_button_monitoring_task;
		hwmon_dev->power_button_timer.data = (unsigned long)hwmon_dev;

		if (!timer_pending(&hwmon_dev->power_button_timer))
			add_timer(&hwmon_dev->power_button_timer);

		/*
		 * Use second to last time through to show some status,
		 * as pm_power_off() is too late.
		 */
		if (hwmon_dev->power_button_count == POWER_BUTTON_COUNT)
		{
			printk(KERN_INFO "%s: %d power button count = %d \n", __FUNCTION__, __LINE__, hwmon_dev->power_button_count);
			/* ran timer long enough, hw reset */
			alive_set_request(SCRATCH_REQUEST_PLAY);
			hwmon_dev->shutdown = 0;
			if(!hwmon_dev->shutdown)
				dev_info(&hwmon_dev->pdev->dev, "Power button held, HW Reset requested \n");
		}
	} 
	else {
		if (pm_power_off)
		{
			dev_info(&hwmon_dev->pdev->dev, "power off requested \n");
			pm_power_off();
		}
		else {
			dev_alert(&hwmon_dev->pdev->dev, "%s.%d: no pm_power_off()\n", __FUNCTION__, __LINE__);
		}
	}
}

/*
 * Start background process
 */

static void glasgow_start_power_button_monitor(void)
{
	if (!hwmon_dev->power_button_count) {/* run background task */
		printk(KERN_INFO "%s: %d\n", __FUNCTION__, __LINE__);
		hwmon_dev->power_button_timer.expires = get_jiffies_64();
		hwmon_dev->power_button_timer.function =
			power_button_monitoring_task;
		hwmon_dev->power_button_timer.data = (unsigned long)hwmon_dev;
		if (!timer_pending(&hwmon_dev->power_button_timer))
			add_timer(&hwmon_dev->power_button_timer);
	}
}

/*
 * set up input device for power button and critical battery
 */
static int setup_power_button(struct platform_device *pdev)
{
	struct glasgow_hwmon *data = platform_get_drvdata(pdev);
	struct input_dev *input_dev;
	int ret;

	input_dev = input_allocate_device();
	if(!input_dev)
		return -ENOMEM;

	input_dev->name = "Power Button";
	input_dev->phys = "glasgow/power_button";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;
	
	input_dev->evbit[0] = BIT(EV_KEY);
	input_dev->keycode = data->buttons;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = 1;

	data->input = input_dev;
	data->buttons[0] = KEY_POWER;	/* we only support power button */
	set_bit(data->buttons[0], input_dev->keybit);

	ret = input_register_device(data->input);
	if(ret)
		goto fail_register;

fail_register:
	input_free_device(input_dev);
	return ret;
}

static int glasgow_power_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct glasgow_hwmon *priv;
	struct device dev = pdev->dev;

	priv = kzalloc(sizeof(struct glasgow_hwmon), GFP_KERNEL);
	if(!priv) {
		ret = -ENOMEM;
		goto fail_alloc;
	}

	priv->pdev = pdev;

	platform_set_drvdata(pdev, priv);

	hwmon_dev = priv;
	
	/* set up work queue to handle the power button */
	priv->power_button_tasks =
		create_singlethread_workqueue("power button tasks");
	INIT_WORK(&priv->power_button_work, glasgow_power_button);

	/* init button power timer */
	setup_timer(&priv->power_button_timer, power_button_monitoring_task,
		       (unsigned long)priv);
	priv->power_button_timer.expires = get_jiffies_64() + 
		POWER_BUTTON_SAMPLING_J;
	priv->power_button_timer.function = power_button_monitoring_task;
	priv->power_button_timer.data = (unsigned long)hwmon_dev;
	add_timer(&priv->power_button_timer);

	/* setup power button monitoring after initializing timer structure */
	ret = setup_power_button(pdev);
	if(ret)
		goto fail_button;
		
	/* initialize power button count */
	hwmon_dev->power_button_count = 0;

	ret = sysfs_create_group(&pdev->dev.kobj, &power_attr_group);
	if(pdev)
		printk(KERN_INFO "power button sysfs %s\n", pdev->name);

	return 0;

fail_button:
fail_alloc:
	kfree(priv);
	return ret;
}

static int glasgow_power_remove(struct platform_device *pdev)
{
	struct glasgow_hwmon *priv = platform_get_drvdata(pdev);

	destroy_workqueue(priv->power_button_tasks);

	sysfs_remove_group(&pdev->dev.kobj, &power_attr_group);

	kfree(priv);

	return 0;
}

static struct platform_driver glasgow_power_driver = {
	.probe      = glasgow_power_probe,
	.remove     = glasgow_power_remove,
	.driver     = {
		.name	= "glasgow-power",
		.owner	= THIS_MODULE,
	},
};

/*
 * module stuff
 */
 
static int __init init_glasgow_power(void)
{
	return platform_driver_register(&glasgow_power_driver);
}

static void cleanup_glasgow_power(void)
{
	platform_driver_unregister(&glasgow_power_driver);
}

late_initcall(init_glasgow_power);
module_exit(cleanup_glasgow_power);

MODULE_AUTHOR("Sukhada Palav");
MODULE_DESCRIPTION("Glasgow hardware monitoring");
MODULE_LICENSE("GPL");
