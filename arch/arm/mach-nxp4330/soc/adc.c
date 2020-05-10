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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/hardirq.h>
#include <linux/platform_device.h>
#include <linux/device.h>

/* nexell soc headers */
#include <mach/devices.h>
#include <mach/platform.h>
#include <mach/soc.h>

#if (0)
#define DBGOUT(msg...)		{ printk("adc: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define IRQ_LOCK(flag) 	do { local_irq_save(flag); } while (0)
#define IRQ_UNLOCK(flag)  do { local_irq_restore(flag); } while (0)

/*-----------------------------------------------------------------------------*/
#define ADC_SPS		50000	
//#define ADC_PRESCALER   66    //(PCLK/(1+ADC_PRESCALER) = (166500000/(1+ 66) = 2.48 MHz
//#define ADC_PRESCALER	128	//(PCLK/(1+ADC_PRESCALER) = (166500000/(1+ 128) = 1.29 MHz

#define ADC_PRESCALER	254	//  (PCLK/(1+ADC_PRESCALER) = (166500000/(1+254) = 652 KHz

static int 			attach_count = 0;
static int 			div = ADC_PRESCALER;
#if (0)
static spinlock_t 		lock;
#define	ADC_LOCK_INIT()	{ spin_lock_init(&lock); }
#define	ADC_LOCK()		{ spin_lock(&lock); }
#define	ADC_UNLOCK()	{ spin_unlock(&lock); }
#else
static DEFINE_MUTEX(mutex);
#define	ADC_LOCK_INIT()	{ ; }
#define	ADC_LOCK(id)	{ if (! preempt_count() && ! in_interrupt()) mutex_lock(&mutex); }
#define	ADC_UNLOCK(id)	{ if (mutex_is_locked(&mutex)) mutex_unlock(&mutex); }
#endif

/*
 * sysfs Interface
 */

#define	ADC_TIMEOUT_IN_US	1000	/* max time to wait for ADC reading */

static ssize_t show_channel0(struct device* dev, struct device_attribute* attr,
		char *buf)
{
	return sprintf(buf, "%d\n", soc_adc_read(0, ADC_TIMEOUT_IN_US));
}
static DEVICE_ATTR(channel0, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_channel0, NULL);

static ssize_t show_channel1(struct device* dev, struct device_attribute* attr,
		char *buf)
{
	return sprintf(buf, "%d\n", soc_adc_read(1, ADC_TIMEOUT_IN_US));
}
static DEVICE_ATTR(channel1, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_channel1, NULL);

static ssize_t show_channel2(struct device* dev, struct device_attribute* attr,
		char *buf)
{
	return sprintf(buf, "%d\n", soc_adc_read(2, ADC_TIMEOUT_IN_US));
}
static DEVICE_ATTR(channel2, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_channel2, NULL);

static ssize_t show_channel3(struct device* dev, struct device_attribute* attr,
		char *buf)
{
	return sprintf(buf, "%d\n", soc_adc_read(3, ADC_TIMEOUT_IN_US));
}
static DEVICE_ATTR(channel3, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_channel3, NULL);

static ssize_t show_channel4(struct device* dev, struct device_attribute* attr,
		char *buf)
{
	return sprintf(buf, "%d\n", soc_adc_read(4, ADC_TIMEOUT_IN_US));
}
static DEVICE_ATTR(channel4, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_channel4, NULL);

static ssize_t show_channel5(struct device* dev, struct device_attribute* attr,
		char *buf)
{
	return sprintf(buf, "%d\n", soc_adc_read(5, ADC_TIMEOUT_IN_US));
}
static DEVICE_ATTR(channel5, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_channel5, NULL);

static ssize_t show_channel6(struct device* dev, struct device_attribute* attr,
		char *buf)
{
	return sprintf(buf, "%d\n", soc_adc_read(6, ADC_TIMEOUT_IN_US));
}
static DEVICE_ATTR(channel6, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_channel6, NULL);

static ssize_t show_channel7(struct device* dev, struct device_attribute* attr,
		char *buf)
{
	return sprintf(buf, "%d\n", soc_adc_read(7, ADC_TIMEOUT_IN_US));
}
static DEVICE_ATTR(channel7, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_channel7, NULL);

static ssize_t show_divisor(struct device* dev, struct device_attribute* attr,
                char *buf)
{
        return sprintf(buf, "%d\n", div);
}

static ssize_t store_divisor(struct device* dev, struct device_attribute* attr,
                const char *buf, size_t count )
{
	sscanf(buf, "%d", &div);
        return count;
}

static DEVICE_ATTR(divisor, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_divisor, store_divisor);

static struct attribute *adc_attributes[] = {
	&dev_attr_channel0.attr,
	&dev_attr_channel1.attr,
	&dev_attr_channel2.attr,
	&dev_attr_channel3.attr,
	&dev_attr_channel4.attr,
	&dev_attr_channel5.attr,
	&dev_attr_channel6.attr,
	&dev_attr_channel7.attr,
	&dev_attr_divisor.attr,
	NULL
};

static struct attribute_group adc_attr_group = {
	.attrs = adc_attributes,
};

static struct bus_type adc_subsys = {
	.name = "adc",
	.dev_name = "adc",
};

/*-----------------------------------------------------------------------------*/
void soc_adc_init(void)
{
	U32 pclk_freq = nxp_cpu_clock_hz (CORECLK_ID_PCLK);
	DBGOUT("%s\n", __func__);

	NX_ADC_SetBaseAddress(0, (U32)IO_ADDRESS(NX_ADC_GetPhysicalAddress(0)));
 	NX_ADC_OpenModule(0);

	nxp_soc_rsc_reset(RESETINDEX_OF_ADC_MODULE_nRST);
	NX_ADC_SetInterruptEnableAll(0, CFALSE);

	printk(KERN_INFO "soc_adc_init: pclk_freq=%u \n", pclk_freq);
	// FIXME NX_ADC_SetPrescalerValue(0,(pclk_freq + (ADC_SPS>>1))/ADC_SPS);
	NX_ADC_SetPrescalerValue(0, div);
	NX_ADC_SetPrescalerEnable(0, CFALSE);
	NX_ADC_SetStandbyMode(0, CTRUE);	/* FALSE=Power On, TRUE=Power Off */

	ADC_LOCK_INIT();
}

/*------------------------------------------------------------------------------
 * 	Description	: enable clock and prescaler of ADC
 *	Return 		: None
 */
void soc_adc_attach(void)
{
	ADC_LOCK();

	DBGOUT("%s(%d)\n", __func__, attach_count);

	if (0 == attach_count) {
		// FIXME NX_ADC_SetPrescalerValue(0, (pclk_freq + (ADC_SPS>>1))/ADC_SPS);
		NX_ADC_SetPrescalerValue(0, div);
		NX_ADC_SetPrescalerEnable(0, CTRUE);
		NX_ADC_SetStandbyMode(0, CFALSE);	/* FALSE=Power On, TRUE=Power Off */
	}

	attach_count++;

	ADC_UNLOCK();
}
EXPORT_SYMBOL(soc_adc_attach);

/*------------------------------------------------------------------------------
 * 	Description	: disable clock and prescaler of ADC
 *	Return 		: None
 */
void soc_adc_detach(void)
{
	ADC_LOCK();
	DBGOUT("%s(%d)\n", __func__, attach_count);
	attach_count--;

	if (0 == attach_count) {
		NX_ADC_SetPrescalerEnable(0, CFALSE);
		NX_ADC_SetStandbyMode(0, CTRUE);	/* FALSE=Power On, TRUE=Power Off */
	}

	/* clear attach count */
	if (0 > attach_count)
		attach_count = 0;

	DBGOUT("%s(%d)\n", __func__, attach_count);

	ADC_UNLOCK();
}
EXPORT_SYMBOL(soc_adc_detach);

/*------------------------------------------------------------------------------
 * 	Description	: get conversioned data of ADC
 *	In[ch]		: value of adc channel ( 0 ~ 7 )
 *	In[timeout]	: wait timer out (us)
 *	Return 		: -1 failure or 10bit data of ADC
 */
unsigned short soc_adc_read(int ch, uint timeout)
{
	ushort value = -1;
	uint   tout  = timeout;
	ulong flags;

	ADC_LOCK();

	NX_ADC_SetPrescalerEnable(0, CFALSE);
	NX_ADC_SetPrescalerValue(0, div);
	NX_ADC_SetPrescalerEnable(0, CTRUE);

	DBGOUT("%s (ch=%d)\n", __func__, ch);

	if (0 == attach_count || ch > 7) 
	{
		printk(KERN_ERR "fail, not attached or not support ADC:%d ...\n", ch);
		ADC_UNLOCK();
		return -1;
	}

	while (NX_ADC_IsBusy(0) && tout--)
		udelay(1);

	if (0 >= tout)
		goto err;

	NX_ADC_SetInputChannel(0, ch);
	
	IRQ_LOCK(flags);
	NX_ADC_Start(0);

	/* wait ADC convert done */
	tout  = timeout;
	while (NX_ADC_IsBusy(0) && tout--);
		//udelay(1);  
		//Note : Per Nexell, ADC data should be latched within 6 ADC clock cycles so taking off the delay

	if (0 >= tout)
		goto err;

	value = (unsigned short)NX_ADC_GetConvertedData(0);
	IRQ_UNLOCK(flags);

	//Note: Per Nexell, after End Of Cycle, SOC should start after 6 ADC clock cycles. Looks like in lf3k, 1us ~ 1.3us, so adding a delay of 8 us
	udelay(8);
err:
	ADC_UNLOCK();

	return value;
}
EXPORT_SYMBOL(soc_adc_read);

/*------------------------------------------------------------------------------
 * 	Description	: set suspend mode,
 *	Return 		: none.
 */
void soc_adc_suspend(void)
{
	PM_DBGOUT("%s\n", __func__);
}
EXPORT_SYMBOL_GPL(soc_adc_suspend);

/*------------------------------------------------------------------------------
 * 	Description	: resume mode
 *	Return 		: none.
 */
void soc_adc_resume(void)
{
	PM_DBGOUT("%s\n", __func__);

	if (! attach_count)
		return;

	// FIXME $NX_ADC_SetPrescalerValue(0, (pclk_freq + (ADC_SPS>>1))/ADC_SPS);
	NX_ADC_SetPrescalerValue(0, div);	
	NX_ADC_SetPrescalerEnable(0, CTRUE);
	NX_ADC_SetStandbyMode(0, CFALSE);	/* FALSE=Power On, TRUE=Power Off */
}
EXPORT_SYMBOL_GPL(soc_adc_resume);

static int __init lf3000_adc_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	struct resource *mem_adc = NULL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk(KERN_ERR "ADC: failed to get resource\n");
		return -ENXIO;
	}

	mem_adc = request_mem_region(res->start, (res->end - res->start) + 1,
			DEV_NAME_ADC);

	if (mem_adc == NULL) {
		printk(KERN_ERR "ADC: failed to map ADC region\n");
		return -EBUSY;
	}

	ret = subsys_system_register(&adc_subsys, NULL);

	ret = sysfs_create_group(&adc_subsys.dev_root->kobj, &adc_attr_group);
	if (ret) {
		sysfs_remove_group(&adc_subsys.dev_root->kobj, &adc_attr_group);
	}
	soc_adc_init();
	soc_adc_attach();
	return ret;
}

static int __exit lf3000_adc_remove(struct platform_device *pdev)
{
	struct resource *res;

	sysfs_remove_group(&adc_subsys.dev_root->kobj, &adc_attr_group);

	/* free memory */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, (res->end - res->start) + 1);

	platform_set_drvdata(pdev, NULL);
	return 0;
};


static struct platform_driver lf3000_adc_driver = {
	.remove = __exit_p(lf3000_adc_remove),
	.driver		= {
		.name	= DEV_NAME_ADC,
		.owner	= THIS_MODULE,
	},
};

static int __init lf3000_adc_init(void)
{
	return platform_driver_probe(&lf3000_adc_driver, lf3000_adc_probe);
}

static void __exit lf3000_adc_exit(void)
{
	platform_driver_unregister(&lf3000_adc_driver);
}

module_init(lf3000_adc_init);
module_exit(lf3000_adc_exit)

MODULE_AUTHOR("leapfrog.com");
MODULE_DESCRIPTION("LF3000 ADC driver");
MODULE_LICENSE("GPL");
