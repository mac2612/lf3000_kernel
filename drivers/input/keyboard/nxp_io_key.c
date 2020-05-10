/*
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
/* nexell soc headers */
#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>

#if (0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "key: " msg); }
#define	ERROUT(msg...)		{ printk(KERN_ERR  "key: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#define	ERROUT(msg...)		do {} while (0)
#endif

#define	KEY_STAT_PRESS		(0)
#define	KEY_STAT_RELEASE	(1)
#define	DELAY_WORK_JIFFIES 	(1)

struct key_code {
	struct delayed_work key_event_work;	/* struct work_struct */
	struct workqueue_struct *key_workqueue;
	struct key_info *info;
	unsigned int io;
	unsigned int keycode;
	unsigned int val;
	unsigned int keystat;		/* current detect mode */
	unsigned int detect_high;	/* detect edge */
};

struct key_info {
	struct input_dev *input;
	int	keys;
	struct key_code *code;
};

#define	POWER_KEY_MASK	(0x3FC)

struct input_dev *key_input = NULL;
EXPORT_SYMBOL_GPL(key_input);

void nxp_key_power_event(void)
{
	if (key_input) {
		input_report_key(key_input, KEY_POWER, 1);
    	input_sync(key_input);
    	input_report_key(key_input, KEY_POWER, 0);
    	input_sync(key_input);
    }
}
EXPORT_SYMBOL(nxp_key_power_event);

static void nxp_key_setup(struct key_info *key)
{
	struct key_code *code = key->code;
	int num = key->keys ;
	int i = 0;

	for (; num > i; i++, code++)
		code->keystat = KEY_STAT_RELEASE;

	key_input = key->input;
}

static void nxp_key_event(struct work_struct *work)
{
	struct key_code *code = (struct key_code *)work;
	struct key_info *key = code->info;
	unsigned int keycode = code->keycode;
	int press = 0;
	u_long flags;

	local_irq_save(flags);

	press = gpio_get_value_cansleep(code->io);
	if (code->detect_high)
		press = !press;

	local_irq_restore(flags);

	if(press != code->keystat) {
		code->keystat = press;
		if (KEY_STAT_PRESS == press) {
			DBGOUT("%s io:%d, code:%4d DN\n", __func__, code->io, keycode);
			input_report_key(key->input, keycode, 1);
			input_sync(key->input);
		} else {
			DBGOUT("%s io:%d, code:%4d UP\n", __func__, code->io, keycode);
			input_report_key(key->input, keycode, 0);
			input_sync(key->input);
		}
	}
}

static irqreturn_t nxp_key_interrupt(int irqno, void *dev_id)
{
	struct key_code *code = dev_id;
	queue_delayed_work(code->key_workqueue,
				&code->key_event_work, DELAY_WORK_JIFFIES);
	return IRQ_HANDLED;
}

static int nxp_key_probe(struct platform_device *pdev)
{
	struct nxp_key_plat_data * plat = pdev->dev.platform_data;
	struct key_info *key = NULL;
	struct key_code *code = NULL;
	struct input_dev *input = NULL;
	int i, keys;
	int ret = 0;

	DBGOUT("%s (device name:%s, id:%d)\n", __func__, pdev->name, pdev->id);

	/*	allocate key_info data */
	key = kzalloc(sizeof(struct key_info), GFP_KERNEL);
	if (! key) {
		printk(KERN_ERR "fail, %s allocate driver info ...\n", pdev->name);
		return -ENOMEM;
	}

	keys = plat->bt_count;
	code = kzalloc(sizeof(struct key_code)*keys, GFP_KERNEL);
	if (!code) {
		printk(KERN_ERR "fail, %s key code ...\n", pdev->name);
		ret = -ENOMEM;
		goto err_mem;
	}

	for (i = 0; keys > i; i++) {
		code[i].io = plat->bt_io[i];
		code[i].keycode = plat->bt_code[i];
		code[i].detect_high = plat->bt_detect_high ? plat->bt_detect_high[i]: 0;
		code[i].val = i;
		code[i].info = key;
		DBGOUT("%d key [io=%3d, key=%4d]\n", i, code[i].io, code[i].keycode);
	}

	input = input_allocate_device();
	if (!input) {
		printk(KERN_ERR "fail, %s allocate input device\n", pdev->name);
		ret = -ENOMEM;
		goto err_mem;
	}

	/* set io info */
	key->input = input;
	key->keys = keys;
	key->code = code;

	/* set input device info */
	input->name	= "Nexell Keypad";
	input->phys = "nexell/input0";
	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0002;
	input->id.version = 0x0100;
	input->dev.parent = &pdev->dev;
	input->keycode = plat->bt_code;
	input->keycodesize = sizeof(plat->bt_code[0]);
	input->keycodemax = plat->bt_count * 2;	// for long key
	input->evbit[0] = BIT_MASK(EV_KEY);
	if (plat->bt_repeat)
		 input->evbit[0] |= BIT_MASK(EV_REP);

	input_set_capability(input, EV_MSC, MSC_SCAN);
	input_set_drvdata(input, key);
	for (i = 0; keys > i; i++)
		__set_bit(code[i].keycode, input->keybit);

	ret = input_register_device(input);
	if (ret) {
		printk(KERN_ERR "fail, %s register for input device ...\n", pdev->name);
		goto err_mem;
	}
	nxp_key_setup(key);

	for (i=0; keys > i; i++) {
		INIT_DELAYED_WORK(&code[i].key_event_work, nxp_key_event);
	    code[i].key_workqueue = create_singlethread_workqueue(pdev->name);
		if (!code[i].key_workqueue) {
    	   ret = -ESRCH;
    	   goto err_irq;
	    }

		ret = request_irq(gpio_to_irq(code[i].io), nxp_key_interrupt,
				IRQF_SHARED | IRQ_TYPE_EDGE_BOTH, pdev->name, &code[i]);
		if (ret) {
			printk(KERN_ERR "fail, gpio[%d] %s request irq...\n",
				code[i].io, pdev->name);
			goto err_irq;
		}
	}

	platform_set_drvdata(pdev, key);
	return ret;

err_irq:
	for (--i; i >= 0; i--) {
		cancel_work_sync(&code[i].key_event_work.work);
	    destroy_workqueue(code[i].key_workqueue);
		free_irq(gpio_to_irq(code[i].io), &code[i]);
	}
	input_free_device(input);

err_mem:
	if (code) kfree(code);
	if (key) kfree(key);

	return ret;
}

static int nxp_key_remove(struct platform_device *pdev)
{
	struct key_info *key = platform_get_drvdata(pdev);
	struct key_code *code = key->code;
	int i = 0, irq;
	DBGOUT("%s\n", __func__);

	input_free_device(key->input);

	for (i = 0; i < key->keys; i++) {
		cancel_work_sync(&code[i].key_event_work.work);
	    destroy_workqueue(code[i].key_workqueue);
		irq = gpio_to_irq(code[i].io);
		free_irq(irq, &code[i]);
	}

	if (code) kfree(code);
	if (key) kfree(key);

	return 0;
}

static int nxp_key_suspend(struct platform_device *pdev, pm_message_t state)
{
	PM_DBGOUT("%s\n", __func__);
	return 0;
}

static int nxp_key_resume(struct platform_device *pdev)
{
	struct key_info *key = platform_get_drvdata(pdev);
	struct key_code * code = key->code;
	int i = 0;
	PM_DBGOUT("%s\n", __func__);

	for (i = 0; key->keys > i; i++) {
		unsigned int keycode = code[i].keycode;
		int io = code[i].io;
		if (keycode == KEY_POWER) {
			if (nxp_check_pm_wakeup_dev("power key", io)) {
				input_report_key(key->input, KEY_POWER, 1);
		    	input_sync(key->input);
		    	input_report_key(key->input, KEY_POWER, 0);
		    	input_sync(key->input);
				PM_DBGOUT("%s POWER UP\n", __func__);
			}
		}
	}
	return 0;
}

static struct platform_driver key_plat_driver = {
	.probe		= nxp_key_probe,
	.remove		= nxp_key_remove,
	.suspend	= nxp_key_suspend,
	.resume		= nxp_key_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DEV_NAME_KEYPAD,
	},
};

static int __init nxp_key_init(void)
{
	DBGOUT("%s\n", __func__);
	return platform_driver_register(&key_plat_driver);
}

static void __exit nxp_key_exit(void)
{
	DBGOUT("%s\n", __func__);
	platform_driver_unregister(&key_plat_driver);
}

module_init(nxp_key_init);
module_exit(nxp_key_exit);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Keypad driver for the Nexell board");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:io keypad");

