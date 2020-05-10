/* drivers/lf1000/lfcart.c - LeapFrog Cartridge "bus"
 *
 * Copyright (c) 2010 Leapfrog Enterprises Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/lf3000/gpio.h>

#include <mach/platform.h>
#include <mach/devices.h>

#define NAND_CART_DETECT_LEVEL 0

enum lfcart_state {
	LFCART_IDLE,
	LFCART_INSERTING,
	LFCART_REMOVING,
};

struct lfcart_priv {
#if 1	/* 29oct13 */
	u32			inserted;
	u32			run;
#else
	bool			inserted;
	bool			run;
#endif	/* 29oct13 */
	struct device		cartridge;
	enum lfcart_state	state;
	unsigned		debounce, debounce_level;
	struct task_struct 	*detect_thread;
	struct semaphore	detect_thread_done;

	struct dentry		*debug;
};

static struct bus_type lfcart_bus_type;

static int lfcart_match_device(struct device *dev, struct device_driver *drv)
{
	return 1;
}

static struct device_attribute lfcart_dev_attrs[] = {
	__ATTR_NULL,
};

static struct bus_type lfcart_bus_type = {
	.name		= "lfcart",
	.dev_attrs	= lfcart_dev_attrs,
	.match		= lfcart_match_device,
};

/*
 * Cartridge detection
 */

static void lfcart_report_insert(struct lfcart_priv *priv)
{
#if 1	/* 29oct13 */
	int status;

printk(KERN_INFO "lfcart_report_insert()\n");
#endif	/* 29oct13 */
	device_initialize(&priv->cartridge);
	dev_set_name(&priv->cartridge, "cartridge");
	priv->cartridge.bus = &lfcart_bus_type;
#if 1	/* 29oct13 */
	status = device_add(&priv->cartridge);
	if (0 == status) {
		printk(KERN_INFO "Added cart device\n");
		dev_dbg(&priv->cartridge, "inserted\n");
	}
	else {
		printk(KERN_INFO "Error (%d) while adding cart device\n", 
			status);
	}
#else
	device_add(&priv->cartridge);

	dev_dbg(&priv->cartridge, "inserted\n");
#endif
}

static void lfcart_report_remove(struct lfcart_priv *priv)
{
printk(KERN_INFO "lfcart_report_remove()\n");
	dev_dbg(&priv->cartridge, "removed\n");
	device_del(&priv->cartridge);
	memset(&priv->cartridge, 0, sizeof(priv->cartridge));
}

#if 0	/* 6nov13 */
int is_cart_inserted(void);
#endif	/* 6nov13 */

static int lfcart_detect(void *data)
{
	struct lfcart_priv *priv = (struct lfcart_priv *)data;
	int cur;

#if 1	/* 14nov13  debug */
printk(KERN_INFO "Entering lfcart_detect() [14nov13 11:30]\n");
#endif	/* 8nov13 */

	while (1) {
		if (!priv->run) {
			up(&priv->detect_thread_done);
printk(KERN_INFO "lfcart_detect() exits (run == 0)\n");
			do_exit(0);
		}
		cur = !gpio_get_value(CARTRIDGE_DETECT);

		switch (priv->state) {
			case LFCART_IDLE:
				if (cur != priv->inserted)
					priv->state = cur ?
						LFCART_INSERTING :
						LFCART_REMOVING;
				break;
			case LFCART_INSERTING:
printk(KERN_INFO "LFCART_INSERTING; cur %d\n", cur);
				if (cur)
					priv->debounce++;
				else {
					priv->debounce = 0;
					priv->state = LFCART_IDLE;
				}
				break;
			case LFCART_REMOVING:
printk(KERN_INFO "LFCART_REMOVING; cur %d\n", cur);
				if (!cur)
					priv->debounce++;
				else {
					priv->debounce = 0;
					priv->state = LFCART_IDLE;
				}
				break;
		}

		if (priv->state > LFCART_IDLE &&
				priv->debounce >= priv->debounce_level) {

			if (cur)
				lfcart_report_insert(priv);
			else
				lfcart_report_remove(priv);

			priv->state = LFCART_IDLE;
			priv->debounce = 0;
			priv->inserted = cur;
		}

		msleep(60);
	}

	return 0;
}

/*
 * Module
 */

static struct lfcart_priv *lfcart = NULL;

int __init lfcart_init(void)
{
	int ret;

	lfcart = kzalloc(sizeof(struct lfcart_priv), GFP_KERNEL);
	if (!lfcart)
		return -ENOMEM;
	lfcart->debounce_level = 5;

	ret = gpio_request(CARTRIDGE_DETECT,"Cartridge Detect");
	if (ret) {
		printk(KERN_INFO "lfcart_init's gpio_request() failed: %d\n",
			ret);
		return ret;
	}
	else {
		printk(KERN_INFO "lfcart_init's gpio_request() succeeded\n");
	}
	gpio_set_function(CARTRIDGE_DETECT, 1);
	gpio_direction_input(CARTRIDGE_DETECT);

	ret = bus_register(&lfcart_bus_type);
	if (ret) {
		printk(KERN_INFO "lfcart_init's bus_register() failed: %d\n",
			ret);
		return ret;
	}

	sema_init(&lfcart->detect_thread_done, 0);
	lfcart->run = 1;
	lfcart->detect_thread = kthread_run(lfcart_detect, (void *)lfcart,
			"lfcart-detect");

	lfcart->debug = debugfs_create_dir("lfcart", NULL);
	if (IS_ERR(lfcart->debug))
		lfcart->debug = NULL;

	if (lfcart->debug) {
		debugfs_create_u32("debounce", S_IRWXUGO, lfcart->debug,
				&lfcart->debounce_level);
		debugfs_create_u32("cur_debounce", S_IRWXUGO, lfcart->debug,
				&lfcart->debounce);
		debugfs_create_u32("cur_run", S_IRWXUGO, lfcart->debug,
				&lfcart->run);
		debugfs_create_u32("cur_state", S_IRWXUGO, lfcart->debug,
				&lfcart->state);
	}
	return 0;
}

void __exit lfcart_exit(void)
{
	lfcart->run = 0;
	down(&lfcart->detect_thread_done);
	if (lfcart->inserted)
		lfcart_report_remove(lfcart);
	bus_unregister(&lfcart_bus_type);
	if (lfcart->debug)
		debugfs_remove(lfcart->debug);
	kfree(lfcart);
}

module_init(lfcart_init);
module_exit(lfcart_exit);
MODULE_AUTHOR("LeapFrog");
MODULE_LICENSE("GPL");

