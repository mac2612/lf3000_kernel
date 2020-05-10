/* linux/drivers/watchdog/nxp4330_wdt.c
 *
 * Copyright (c) 2014 Nexell Co
 *	hyun seok jung <hsjung@nexell.co.kr>
 *
 * NXP4330 Watchdog Timer Support
 *
 * Based on, softdog.c by Alan Cox,
 *     (c) Copyright 1996 Alan Cox <alan@lxorguk.ukuu.org.uk>
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/miscdevice.h> /* for MODULE_ALIAS_MISCDEV */
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>

#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>
#include <mach/nxp-dfs-bclk.h>

#define CONFIG_NXP4330_WATCHDOG_DEFAULT_TIME	(10)
#define CONFIG_NXP4330_WATCHDOG_MAX_TIME		(10)

#if !defined(CFG_WATCHDOG_MAGICCLOSE_OFF)
#define OPTIONS (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE)
#else
#define OPTIONS (WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING)
#endif

#define NXP_WDOGREG(x) ((x) + IO_ADDRESS(PHY_BASEADDR_WDT))

#define NXP4330_WTCON      NXP_WDOGREG(0x00)
#define NXP4330_WTDAT      NXP_WDOGREG(0x04)
#define NXP4330_WTCNT      NXP_WDOGREG(0x08)
#define NXP4330_WTCLRINT   NXP_WDOGREG(0x0c)

/* the watchdog can either generate a reset pulse, or an
 * interrupt.
 */

#define NXP4330_WTCON_RSTEN   (0x01)
#define NXP4330_WTCON_INTEN   (1<<2)
#define NXP4330_WTCON_ENABLE  (1<<5)

#define NXP4330_WTCON_DIV16   (0<<3)
#define NXP4330_WTCON_DIV32   (1<<3)
#define NXP4330_WTCON_DIV64   (2<<3)
#define NXP4330_WTCON_DIV128  (3<<3)

#define NXP4330_WTCON_PRESCALE(x) ((x) << 8)
#define NXP4330_WTCON_PRESCALE_MASK (0xff00)

static bool nowayout	= WATCHDOG_NOWAYOUT;
static int tmr_margin	= CONFIG_NXP4330_WATCHDOG_DEFAULT_TIME;
static int soft_noboot;
static int debug;
static unsigned long wdt_freq;

module_param(tmr_margin,  int, 0);
module_param(nowayout,   bool, 0);
module_param(soft_noboot, int, 0);
module_param(debug,	  int, 0);

MODULE_PARM_DESC(tmr_margin, "Watchdog tmr_margin in seconds. (default="
		__MODULE_STRING(CONFIG_NXP4330_WATCHDOG_DEFAULT_TIME) ")");
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
MODULE_PARM_DESC(soft_noboot, "Watchdog action, set to 1 to ignore reboots, "
			"0 to reboot (default 0)");
MODULE_PARM_DESC(debug, "Watchdog debug, set to >1 for debug (default 0)");

static struct device    *wdt_dev;	/* platform device attached to */
static struct resource	*wdt_mem;
static struct resource	*wdt_irq;
static struct clk	*wdt_clock;
static unsigned int	 wdt_count;
static DEFINE_SPINLOCK(wdt_lock);

/* watchdog control routines */
#define DBG(fmt, ...)					\
do {							\
	if (debug)					\
		pr_info(fmt, ##__VA_ARGS__);		\
} while (0)

/* functions */

static int nxp4330wdt_keepalive(struct watchdog_device *wdd)
{
	spin_lock(&wdt_lock);
        writel(0, NXP4330_WTCLRINT);
	writel(wdt_count, NXP4330_WTCNT);
	spin_unlock(&wdt_lock);

	return 0;
}

static void __nxp4330wdt_stop(void)
{
	unsigned long wtcon;

	wtcon = readl(NXP4330_WTCON);
	wtcon &= ~(NXP4330_WTCON_ENABLE | NXP4330_WTCON_RSTEN);
	writel(wtcon, NXP4330_WTCON);
        writel(0, NXP4330_WTCLRINT);
}

static int nxp4330wdt_stop(struct watchdog_device *wdd)
{
	spin_lock(&wdt_lock);
	__nxp4330wdt_stop();
	spin_unlock(&wdt_lock);

	return 0;
}

static int nxp4330wdt_start(struct watchdog_device *wdd)
{
	unsigned long wtcon;

	spin_lock(&wdt_lock);

	__nxp4330wdt_stop();

	wtcon = readl(NXP4330_WTCON);
	wtcon |= NXP4330_WTCON_ENABLE | NXP4330_WTCON_DIV128;

	if (soft_noboot) {
		wtcon |= NXP4330_WTCON_INTEN;
		wtcon &= ~NXP4330_WTCON_RSTEN;
	} else {
		wtcon |= NXP4330_WTCON_INTEN;
		wtcon |= NXP4330_WTCON_RSTEN;
	}

	DBG("%s: wdt_count=0x%08x, wtcon=%08lx\n",
	    __func__, wdt_count, wtcon);

	writel(wdt_count, NXP4330_WTDAT);
	writel(wdt_count, NXP4330_WTCNT);
	writel(wtcon, NXP4330_WTCON);
	spin_unlock(&wdt_lock);

	return 0;
}

static inline int nxp4330wdt_is_running(void)
{
	return readl(NXP4330_WTCON) & NXP4330_WTCON_ENABLE;
}

static int nxp4330wdt_set_heartbeat(struct watchdog_device *wdd, unsigned timeout)
{
	unsigned long freq = wdt_freq;
	unsigned int count;
	unsigned int divisor = 1;
	unsigned long wtcon;


	if ((timeout < 1) || (timeout > CONFIG_NXP4330_WATCHDOG_MAX_TIME)){
		dev_err(wdt_dev, "timeout %d invalid value\n", timeout);
		return -EINVAL;
	}

	freq /= 128;
	count = timeout * freq;

	DBG("%s: count=%d, timeout=%d, freq=%lu\n",
	    __func__, count, timeout, freq);

	/* if the count is bigger than the watchdog register,
	   then work out what we need to do (and if) we can
	   actually make this value
	*/

	if (count >= 0x10000) {
		for (divisor = 1; divisor <= 0x100; divisor++) {
			if ((count / divisor) < 0x10000)
				break;
		}

		if ((count / divisor) >= 0x10000) {
			dev_err(wdt_dev, "timeout %d too big\n", timeout);
			return -EINVAL;
		}
	}

	DBG("%s: timeout=%d, divisor=%d, count=%d (%08x)\n",
	    __func__, timeout, divisor, count, count/divisor);

	count /= divisor;
	wdt_count = count;

	/* update the pre-scaler */
	wtcon = readl(NXP4330_WTCON);
	wtcon &= ~NXP4330_WTCON_PRESCALE_MASK;
	wtcon |= NXP4330_WTCON_PRESCALE(divisor-1);

	writel(count, NXP4330_WTDAT);
	writel(wtcon, NXP4330_WTCON);

	wdd->timeout = timeout;

	return 0;
}

static const struct watchdog_info nxp4330_wdt_ident = {
	.options          =     OPTIONS,
	.firmware_version =	0,
	.identity         =	"NXP4330 Watchdog",
};

static struct watchdog_ops nxp4330wdt_ops = {
	.owner = THIS_MODULE,
	.start = nxp4330wdt_start,
	.stop = nxp4330wdt_stop,
	.ping = nxp4330wdt_keepalive,
	.set_timeout = nxp4330wdt_set_heartbeat,
};

static struct watchdog_device nxp4330_wdd = {
	.info = &nxp4330_wdt_ident,
	.ops = &nxp4330wdt_ops,
};

/* interrupt handler code */

static irqreturn_t nxp4330wdt_irq(int irqno, void *param)
{
	if(soft_noboot)
		nxp4330wdt_keepalive(&nxp4330_wdd);
	return IRQ_HANDLED;
}

#ifdef CONFIG_NEXELL_DFS_BCLK

static int nxp4330wdt_bclk_dfs_transition(struct notifier_block *nb,
					  unsigned long val, void *data)
{
	int ret;

	DBG("%s val=%lu,data=%u WDT_STA=%d\n", __func__, val, *(uint32_t *)data, (bool)nxp4330wdt_is_running());

	wdt_freq = (*(uint32_t *)data)/2;

	if (!nxp4330wdt_is_running())
		goto done;

	if (val == BCLK_CHANGED) {
		nxp4330wdt_stop(&nxp4330_wdd);

		ret = nxp4330wdt_set_heartbeat(&nxp4330_wdd, nxp4330_wdd.timeout);

		if (ret >= 0)
			nxp4330wdt_start(&nxp4330_wdd);
		else
			goto err;
	}
	else {
		/* To ensure that over the change we don't cause the
		 * watchdog to trigger, we perform an keep-alive if
		 * the watchdog is running.
		 */

		nxp4330wdt_keepalive(&nxp4330_wdd);
	}
done:
	return 0;

 err:
	dev_err(wdt_dev, "cannot set new value for timeout %d. and restart(default timeout value) \n",
				nxp4330_wdd.timeout);
	nxp4330wdt_set_heartbeat(&nxp4330_wdd, CONFIG_NXP4330_WATCHDOG_DEFAULT_TIME);
	nxp4330wdt_start(&nxp4330_wdd);

	return ret;
}

static struct notifier_block nxp4330wdt_bclk_dfs_transition_nb = {
	.notifier_call	= nxp4330wdt_bclk_dfs_transition,
};

static inline void nxp4330wdt_bclk_dfs_register(void)
{
	bclk_dfs_register_notify(&nxp4330wdt_bclk_dfs_transition_nb);
}

static inline void nxp4330wdt_bclk_dfs_deregister(void)
{
	bclk_dfs_unregister_notify(&nxp4330wdt_bclk_dfs_transition_nb);
}

#else
static inline int nxp4330wdt_bclk_dfs_register(void)
{
	return 0;
}

static inline void nxp4330wdt_bclk_dfs_deregister(void)
{
}
#endif

static int __devinit nxp4330wdt_probe(struct platform_device *pdev)
{
	struct device *dev;
	unsigned int wtcon;
	int started = 0;
	int ret;
	int size;

	DBG("%s: probe=%p\n", __func__, pdev);

	dev = &pdev->dev;
	wdt_dev = &pdev->dev;

	wdt_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (wdt_mem == NULL) {
		dev_err(dev, "no memory resource specified\n");
		return -ENOENT;
	}

	wdt_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (wdt_irq == NULL) {
		dev_err(dev, "no irq resource specified\n");
		ret = -ENOENT;
		goto err;
	}

	/* get the memory region for the watchdog timer */

	size = resource_size(wdt_mem);
	if (!request_mem_region(wdt_mem->start, size, pdev->name)) {
		dev_err(dev, "failed to get memory region\n");
		ret = -EBUSY;
		goto err;
	}

	wdt_clock = clk_get(NULL, "pclk");
	if (IS_ERR(wdt_clock)) {
		dev_err(dev, "failed to find watchdog clock source\n");
		ret = PTR_ERR(wdt_clock);
		goto err_map;
	}

	clk_enable(wdt_clock);
	wdt_freq = clk_get_rate(wdt_clock);

	nxp4330wdt_bclk_dfs_register();

	nxp_soc_rsc_reset(RESET_ID_WDT);
	nxp_soc_rsc_reset(RESET_ID_WDT_POR);

	/* if we're not enabling the watchdog, then ensure it is
	 * disabled if it has been left running from the bootloader
	 * or other source */
	nxp4330wdt_stop(&nxp4330_wdd);

	/* see if we can actually set the requested timer margin, and if
	 * not, try the default value */

	if (nxp4330wdt_set_heartbeat(&nxp4330_wdd, tmr_margin)) {
		started = nxp4330wdt_set_heartbeat(&nxp4330_wdd,
					CONFIG_NXP4330_WATCHDOG_DEFAULT_TIME);

		if (started == 0)
			dev_info(dev,
			   "tmr_margin value out of range, default %d used\n",
			       CONFIG_NXP4330_WATCHDOG_DEFAULT_TIME);
		else
			dev_info(dev, "default timer value is out of range, "
							"cannot start\n");
	}

	ret = request_irq(wdt_irq->start, nxp4330wdt_irq, 0, pdev->name, pdev);
	if (ret != 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_clk;
	}

	watchdog_set_nowayout(&nxp4330_wdd, nowayout);

	ret = watchdog_register_device(&nxp4330_wdd);
	if (ret) {
		dev_err(dev, "cannot register watchdog (%d)\n", ret);
		goto err_irq;
	}


	/* print out a statement of readiness */

	wtcon = readl(NXP4330_WTCON);

	dev_info(dev, "watchdog %sactive, reset %sabled, irq %sabled\n",
		 (wtcon & NXP4330_WTCON_ENABLE) ?  "" : "in",
		 (wtcon & NXP4330_WTCON_RSTEN) ? "en" : "dis",
		 (wtcon & NXP4330_WTCON_INTEN) ? "en" : "dis");

	return 0;

 err_irq:
	free_irq(wdt_irq->start, pdev);

 err_clk:
	clk_disable(wdt_clock);
	clk_put(wdt_clock);
	wdt_clock = NULL;

 err_map:
	release_mem_region(wdt_mem->start, size);

 err:
	wdt_irq = NULL;
	wdt_mem = NULL;
	return ret;
}

static int __devexit nxp4330wdt_remove(struct platform_device *dev)
{
	watchdog_unregister_device(&nxp4330_wdd);

	free_irq(wdt_irq->start, dev);

	nxp4330wdt_bclk_dfs_deregister();

	clk_disable(wdt_clock);
	clk_put(wdt_clock);
	wdt_clock = NULL;

	release_mem_region(wdt_mem->start, resource_size(wdt_mem));
	wdt_irq = NULL;
	wdt_mem = NULL;
	return 0;
}

static void nxp4330wdt_shutdown(struct platform_device *dev)
{
	nxp4330wdt_stop(&nxp4330_wdd);
}

#ifdef CONFIG_PM

static unsigned long wtcon_save;
static unsigned long wtdat_save;

static int nxp4330wdt_suspend(struct platform_device *dev, pm_message_t state)
{
	/* Save watchdog state, and turn it off. */
	wtcon_save = readl(NXP4330_WTCON);

	/* Note that WTCNT doesn't need to be saved. */
	nxp4330wdt_stop(&nxp4330_wdd);

	return 0;
}

static int nxp4330wdt_resume(struct platform_device *dev)
{
	/* Restore watchdog state. */
	if (!nxp_soc_rsc_status(RESET_ID_WDT))
		nxp_soc_rsc_reset(RESET_ID_WDT);
	if (!nxp_soc_rsc_status(RESET_ID_WDT_POR))
		nxp_soc_rsc_reset(RESET_ID_WDT_POR);

	/* Note that WTCNT doesn't need to be saved. */
	nxp4330wdt_stop(&nxp4330_wdd);

	wdt_freq = clk_get_rate(wdt_clock);
	nxp4330wdt_set_heartbeat(&nxp4330_wdd, nxp4330_wdd.timeout);

	wtdat_save = readl(NXP4330_WTDAT);
	writel(wtdat_save, NXP4330_WTCNT); /* Reset count */

	wtcon_save &= 0xff;
	wtcon_save |= readl(NXP4330_WTCON)&NXP4330_WTCON_PRESCALE_MASK;
	writel(wtcon_save, NXP4330_WTCON);

	pr_info("watchdog %sabled\n",
		(wtcon_save & NXP4330_WTCON_ENABLE) ? "en" : "dis");

	return 0;
}

#else
#define nxp4330wdt_suspend NULL
#define nxp4330wdt_resume  NULL
#endif /* CONFIG_PM */

#ifdef CONFIG_OF
static const struct of_device_id nxp4330_wdt_match[] = {
	{ .compatible = "nexell,nxp-wdt" },
	{},
};
MODULE_DEVICE_TABLE(of, nxp4330_wdt_match);
#else
#define nxp4330_wdt_match NULL
#endif

static struct platform_driver nxp4330wdt_driver = {
	.probe		= nxp4330wdt_probe,
	.remove		= __devexit_p(nxp4330wdt_remove),
	.shutdown	= nxp4330wdt_shutdown,
	.suspend	= nxp4330wdt_suspend,
	.resume		= nxp4330wdt_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DEV_NAME_WDT,
		.of_match_table	= nxp4330_wdt_match,
	},
};


static int __init watchdog_init(void)
{
	pr_info("NXP4330 Watchdog Timer, (c) 2014 Nexell\n");

	return platform_driver_register(&nxp4330wdt_driver);
}

static void __exit watchdog_exit(void)
{
	platform_driver_unregister(&nxp4330wdt_driver);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("hyunseok jung <hsjung@nexell.co.kr>");
MODULE_DESCRIPTION("NXP4330 Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:nxp-wdt");
