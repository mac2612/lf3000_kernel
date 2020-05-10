/*
 * linux/sound/soc/lf1000/lfp100-lfp100.c
 *
 * ALSA Machine driver for the LeapFrog LF1000 style game console.
 * Supports the audio codec part of the LeapFrog LFP100 power/codec chip,
 * connected to the LF1000 SoC's I2S controller.
 *
 * Author: Scott Esters <sesters@leapfrog.com>
 * Hacker: Dave Milici <dmilici@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation. 
 */

/* #define DEBUG */		/* for KERN_DEBUG tracers */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>

#include <mach/gpio.h>
#include <mach/platform_id.h>

#include "nxp-pcm.h"
#include "nxp-i2s.h"
#include "nxp-lfp100.h"
#include "../codecs/lfp100.h"

#define LFP100_DEFAULT_RATE	32000

/*
 * Board ops
 */

static int lfp100_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;

	printk(KERN_DEBUG "%s: \n", __FUNCTION__);

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret) {
		printk(KERN_ERR "%s: snd_soc_dai_set_fmt (codec) ret=%d\n", __FUNCTION__, ret);
		return ret;
	}

	return 0;
}

static int lfp100_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;

	printk(KERN_DEBUG "%s: \n", __FUNCTION__);

	/* set codec DAI configuration (slave clock) */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret) {
		printk(KERN_ERR "%s: snd_soc_dai_set_fmt ret=%d\n", __FUNCTION__, ret);
		return ret;
	}

	return 0;
}

static void lfp100_shutdown(struct snd_pcm_substream *substream)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	return;
}

static int lfp100_probe(struct snd_soc_card *card)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	return 0;
}

static int lfp100_resume_pre(struct snd_soc_card *card)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	return 0;
}

static int lfp100_init(struct snd_soc_pcm_runtime *rtd)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);

	{
		struct snd_soc_codec *codec;
		int i;

		/* program codec defaults */
		codec = rtd->codec;
		
		for (i = 0; i < ARRAY_SIZE(lfp100_common_settings); i++) {
			codec->driver->write(codec,
				lfp100_common_settings[i][0],
				lfp100_common_settings[i][1]);
		}		
		
		switch (get_leapfrog_platform()) {
			case RIO:
				for (i = 0; i < ARRAY_SIZE(lfp100_rio_settings); i++) {
					codec->driver->write(codec,
						lfp100_rio_settings[i][0],
						lfp100_rio_settings[i][1]);
				}
				break;
				
			case CABO:
			case XANADU:
				for (i = 0; i < ARRAY_SIZE(lfp100_cabo_settings); i++) {
					codec->driver->write(codec,
						lfp100_cabo_settings[i][0],
						lfp100_cabo_settings[i][1]);
				}
				break;
		}
	}

	return 0;
}

static struct snd_soc_ops lfp100_ops = {
	.startup	= lfp100_startup,
	.shutdown	= lfp100_shutdown,
	.hw_params	= lfp100_hw_params,
};

/* Platform-specific Nexell I2S driver name */
static char dev_name_i2s[16] = DEV_NAME_I2S ".0";

/* lfp100 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link lfp100_dai_lfp100_i2c_0[] = {
	{
		.name = "ASOC-LFP100",
		.stream_name = "LFP100 HiFi",
		.cpu_dai_name 	= dev_name_i2s,				/* nx_snd_i2s_driver name */
		.platform_name  = DEV_NAME_PCM,				/* nx_snd_pcm_driver name */
		.codec_dai_name = "lfp100-hifi",			/* lfp100_dai's name */
		.codec_name 	= "lfp100-codec.0-0066",	/* lfp100_i2c_driver name + i2c bus-address */
		.init			= lfp100_init,
		.ops = &lfp100_ops,
	},
};

/* lfp100 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link lfp100_dai_lfp100_i2c_1[] = {
	{
		.name = "ASOC-LFP100",
		.stream_name = "LFP100 HiFi",
		.cpu_dai_name 	= dev_name_i2s,				/* nx_snd_i2s_driver name */
		.platform_name  = DEV_NAME_PCM,				/* nx_snd_pcm_driver name */
		.codec_dai_name = "lfp100-hifi",			/* lfp100_dai's name */
		.codec_name 	= "lfp100-codec.1-0066",	/* lfp100_i2c_driver name + i2c bus-address */
		.init			= lfp100_init,
		.ops = &lfp100_ops,
	},
};

/* lfp100 audio machine driver */
static struct snd_soc_card snd_soc_lfp100_i2c_0 = {
	.name = "soc-audio-lfp100",
	.dai_link = &lfp100_dai_lfp100_i2c_0[0],
	.num_links = ARRAY_SIZE(lfp100_dai_lfp100_i2c_0),
	.probe = &lfp100_probe,
	.resume_pre = &lfp100_resume_pre,
};

static struct snd_soc_card snd_soc_lfp100_i2c_1 = {
	.name = "soc-audio-lfp100",
	.dai_link = &lfp100_dai_lfp100_i2c_1[0],
	.num_links = ARRAY_SIZE(lfp100_dai_lfp100_i2c_1),
	.probe = &lfp100_probe,
	.resume_pre = &lfp100_resume_pre,
};

/*
 * sysfs interface
 */

static ssize_t show_force_audio(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct lfp100_private *lfp100;
	if (is_board_lucy())
		lfp100 = i2c_get_clientdata(snd_soc_lfp100_i2c_1.rtd->codec->control_data);
	else
		lfp100 = i2c_get_clientdata(snd_soc_lfp100_i2c_0.rtd->codec->control_data);

        return(sprintf(buf,"%u\n", lfp100->force_audio));
}

static ssize_t set_force_audio(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        unsigned int value;
	struct lfp100_private *lfp100;

	if (is_board_lucy())
		lfp100 = i2c_get_clientdata(snd_soc_lfp100_i2c_1.rtd->codec->control_data);
	else
		lfp100 = i2c_get_clientdata(snd_soc_lfp100_i2c_0.rtd->codec->control_data);

        if (sscanf(buf, "%x", &value) != 1)
                return -EINVAL;

        lfp100->force_audio = value;
        return count;
}

static DEVICE_ATTR(force_audio, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
        show_force_audio, set_force_audio);

static struct attribute *lfp100_attributes[] = {
        &dev_attr_force_audio.attr,
        NULL
};

static struct attribute_group lfp100_attr_group = {
	.attrs = lfp100_attributes
};

static int lfp100_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &snd_soc_lfp100_i2c_0;
	struct snd_soc_dai *cpu_dai = NULL;
	struct snd_soc_dai_driver *driver = NULL;

	printk(KERN_DEBUG "%s: \n", __FUNCTION__);

	if (is_board_lucy())
		card = &snd_soc_lfp100_i2c_1;
	else
		card = &snd_soc_lfp100_i2c_0;

	/* register card */
	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);
		return ret;
	}

	if (card->rtd)
		cpu_dai = card->rtd->cpu_dai;

	if (cpu_dai)
		driver = cpu_dai->driver;

	dev_info(&pdev->dev, "register card %s -> %s\n",
		card->dai_link->codec_dai_name, card->dai_link->cpu_dai_name);

	/* Reset i2s support sample rates and formats */
	if (driver) {
		driver->playback.rates = SNDRV_PCM_RATE_8000_48000; //SND_SOC_I2S_RATES;
		driver->capture.rates  = SNDRV_PCM_RATE_8000_48000; //SND_SOC_I2S_RATES;
		driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE; //SND_SOC_I2S_FORMATS;
		driver->capture.formats  = SNDRV_PCM_FMTBIT_S16_LE; //SND_SOC_I2S_FORMATS;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &lfp100_attr_group);
	return 0;
}

static int __devexit lfp100_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	sysfs_remove_group(&pdev->dev.kobj, &lfp100_attr_group);

	return 0;
}

static struct platform_driver lfp100_audio_driver = {
	.probe	= lfp100_audio_probe,
	.remove	= lfp100_audio_remove,
	.driver	= {
		.name	= "lfp100-asoc",
		.owner	= THIS_MODULE,
	},
};

static int __init lfp100_audio_init(void)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	return platform_driver_register(&lfp100_audio_driver);
}
module_init(lfp100_audio_init);

static void __exit lfp100_audio_exit(void)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	platform_driver_unregister(&lfp100_audio_driver);
}
module_exit(lfp100_audio_exit);

MODULE_AUTHOR("Scott Esters <sesters@leapfrog.com>");
MODULE_DESCRIPTION("ALSA SoC LFP100");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:soc-audio");
