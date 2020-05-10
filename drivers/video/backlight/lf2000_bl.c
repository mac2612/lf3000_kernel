/*
 * drivers/video/backlight/lf2000_bl.c
 *
 * PWM backlight support for the LF2000 LeapFrog boards.
 *
 * Copyright 2012 LeapFrog Enterprises Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include <mach/platform.h>
#include <mach/pwm.h>
#include <mach/gpio.h>
#include <mach/lfp100.h>
#include <mach/devices.h>
#include <mach/tc7734.h>

#define LF2000_NO_BRIGHTNESS		 -1	// brightness not set
#define LF2000_INITIAL_BRIGHTNESS	318	// nominal second brightest
#define LF2000_MAX_BRIGHTNESS		511

struct lf2000_bl {
	struct platform_device	*pdev;
	struct backlight_device *bl;
	u32			pwmds;
	u32			pwm_channel;
};

/* convert 9 bit intensity range to 5 bit WLED range */
#define LFP100_WLED_ENTRIES	32
static int intensity_to_lfp100_wled[LFP100_WLED_ENTRIES] = {
	 13,  25,  38,  50,  63,  76,  88, 101,
	114, 126, 139, 151, 165, 182, 199, 217,
	234, 245, 255, 266, 276, 287, 297, 308,
	319, 345, 373, 401, 428, 456, 484, 512 };

/* convert 9 bit intensity range to 5 bit WLED range */
/* FIXME these values need to be calibrated for TC7734. */
/* FIXME backlight driver should have everything in lumens, individual drivers should convert it to register settings. */
#define TC7734_WLED_ENTRIES	32
static int intensity_to_tc7734_wled[TC7734_WLED_ENTRIES] = {
	 13,  25,  38,  50,  63,  76,  88, 101,
	114, 126, 139, 151, 165, 182, 199, 217,
	234, 245, 255, 266, 276, 287, 297, 308,
	319, 345, 373, 401, 428, 456, 484, 512 };

static int lf2000_intensity_to_wled(int intensity)
{
	int i;

#ifdef CONFIG_SOC_LFP100
	if (lfp100_have_lfp100()) {
		for (i = 0; i < LFP100_WLED_ENTRIES; i++) {
			if (intensity < intensity_to_lfp100_wled[i])
				break;
		}
		if (i < LFP100_WLED_ENTRIES)
			return i;
		/* entry not found, at max */
		return LFP100_WLED_ENTRIES - 1;
	}
#endif

#ifdef CONFIG_TC7734_PMIC
	if (tc7734_have_tc7734()) {
		for (i = 0; i < TC7734_WLED_ENTRIES; i++) {
				if (intensity < intensity_to_tc7734_wled[i])
					break;
			}
			if (i < TC7734_WLED_ENTRIES)
				return i;
			/* entry not found, at max */
			return TC7734_WLED_ENTRIES - 1;
	}
#endif
}


static int lf2000_bl_get_brightness(struct backlight_device *bd)
{
	struct lf2000_bl *priv = bl_get_data(bd);

	return priv->pwmds;
}

static int lf2000_bl_set_brightness(struct backlight_device *bd)
{
	struct lf2000_bl *priv = bl_get_data(bd);
	int intensity;

#if 0
	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
#endif

	if (bd->props.brightness == LF2000_NO_BRIGHTNESS) {

#ifdef CONFIG_SOC_LFP100
	    if (lfp100_have_lfp100()) {
	        priv->bl->props.brightness = 
	            intensity_to_lfp100_wled[lfp100_read_reg(LFP100_WLED)] - 1; 
	    }
#endif

#ifdef CONFIG_TC7734_PMIC
	    if (tc7734_have_tc7734()) {
			priv->bl->props.brightness =
				intensity_to_tc7734_wled[(tc7734_read_reg(TC7734_DDIM)&TC7734_BACKLIGHT_CONFIG_MASK)] - 1; //only read bits 4:0, bit 7 is phase select for TC7734.
	    } else {
	        priv->bl->props.brightness = LF2000_INITIAL_BRIGHTNESS;
	    }
#endif
	}

	intensity = bd->props.brightness;

#ifdef CONFIG_SOC_LFP100
	if (lfp100_have_lfp100()) {
		lfp100_write_reg(LFP100_WLED, 
			lf2000_intensity_to_wled(intensity));
	}
#endif

#ifdef CONFIG_TC7734_PMIC
	if (tc7734_have_tc7734()) {
		tc7734_write_reg(TC7734_DDIM,
				(lf2000_intensity_to_wled(intensity)&TC7734_BACKLIGHT_CONFIG_MASK|TC7734_LEDD_PS));
	}
#endif

#ifdef CONFIG_ARCH_LF1000
	else {
		if (pwm_set_duty_cycle(priv->pwm_channel, intensity))
			return -EINVAL;

		priv->pwmds = intensity;
	}
#endif

	return 0;
}

static struct backlight_ops lf2000_bl_ops = {
	.get_brightness	= lf2000_bl_get_brightness,
	.update_status	= lf2000_bl_set_brightness,
};

static int lf2000_bl_probe(struct platform_device *pdev)
{
	int ret;
	u8 polarity;
	struct lf2000_bl *priv;
	struct backlight_properties props;

	dev_info(&pdev->dev, "%s\n", __FUNCTION__);

	priv = kzalloc(sizeof(struct lf2000_bl), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "can't allocate priv data\n");
		return -ENOMEM;
	}
	priv->pdev = pdev;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_FIRMWARE;
	props.max_brightness = LF2000_MAX_BRIGHTNESS;

	priv->bl = backlight_device_register("lf2000-pwm-bl",
			&pdev->dev, priv, &lf2000_bl_ops, &props);
	if (IS_ERR(priv->bl)) {
		ret = PTR_ERR(priv->bl);
		dev_err(&pdev->dev, "failed to register backlight: %d\n", ret);
		kfree(priv);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	priv->bl->props.power = FB_BLANK_UNBLANK;

	/* idealy we would have read the LFP100 brightness
	 * set in U-BOOT, but  the LFP100 driver might not
	 * be loaded yet; so delay setting the backlight value
	 */
 
	priv->bl->props.brightness = LF2000_NO_BRIGHTNESS;

#ifdef CONFIG_ARCH_LF1000
	if (gpio_have_gpio_madrid()) {
		priv->pwm_channel = PWM_CHAN2;
		polarity = POL_INV;	/* inverted PWM polarity */
	} else {
		priv->pwm_channel = PWM_CHAN1;
		//On Explorer, pin A30 (PWM0) is LED_ENA and should be on
		//The bootloader does this, but can't hurt to do it here to be safe.
		polarity = POL_BYP;	/* normal PWM polarity */
		gpio_set_out_en(lf1000_l2p_port(LED_ENA),
			lf1000_l2p_pin(LED_ENA), 1);
		gpio_set_cur(lf1000_l2p_port(LED_ENA),
			lf1000_l2p_pin(LED_ENA), GPIO_CURRENT_8MA);
	}
	dev_info(&pdev->dev, "Using PWM Channel %d for backlight\n", priv->pwm_channel);
	
	ret = pwm_get_clock_rate();
	if (ret < 1) {
		dev_err(&pdev->dev, "can't get PWM rate\n");
		priv->pwmds = 0;
	} else {
		dev_info(&pdev->dev, "PWM rate is %d\n", ret);
		pwm_configure_pin(priv->pwm_channel);
		pwm_set_prescale(priv->pwm_channel, 1);
		pwm_set_period(priv->pwm_channel, 511);
		pwm_set_polarity(priv->pwm_channel, polarity);
	}
#endif

	/* removed -- leave brightness at U-Boot setting at startup */
#if 0
	lf2000_bl_set_brightness(priv->bl);
	backlight_update_status(priv->bl);
#endif
	return 0;
}

static int __exit lf2000_bl_remove(struct platform_device *pdev)
{
	struct lf2000_bl *priv = platform_get_drvdata(pdev);

	backlight_device_unregister(priv->bl);
	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return 0;
}

static struct platform_driver lf2000_bl_driver = {
	.probe	= lf2000_bl_probe,
	.remove	= __exit_p(lf2000_bl_remove),
	.driver = {
		.name	= DEV_NAME_LF_BL,
		.owner	= THIS_MODULE,
	},
};

static int __init lf2000_bl_init(void)
{
	return platform_driver_register(&lf2000_bl_driver);
}

static void __exit lf2000_bl_exit(void)
{
	platform_driver_unregister(&lf2000_bl_driver);
}

module_init(lf2000_bl_init);
module_exit(lf2000_bl_exit);

MODULE_AUTHOR("Daniel Lazzari");
MODULE_DESCRIPTION("LF2000 backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lf2000-bl");
