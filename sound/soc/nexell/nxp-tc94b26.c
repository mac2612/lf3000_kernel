/*
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

#include "nxp-i2s.h"
#include "../codecs/tc94b26.h"


/*
 * Board ops
 */

static int nxp_tc94b26_startup(struct snd_pcm_substream *substream)
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

static int nxp_tc94b26_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;

	printk(KERN_DEBUG "%s: \n", __FUNCTION__);

	return 0;
}

static void nxp_tc94b26_shutdown(struct snd_pcm_substream *substream)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	return;
}

static int nxp_tc94b26_probe(struct snd_soc_card *card)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	return 0;
}

static int nxp_tc94b26_resume_pre(struct snd_soc_card *card)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	return 0;
}

static int nxp_tc94b26_init(struct snd_soc_pcm_runtime *rtd)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	//TODO we are doing TC94B26 initialization in codec probe.
	//Any SoC related initialization could be added here.

	return 0;
}

static struct snd_soc_ops nxp_tc94b26_ops = {
	.startup	= nxp_tc94b26_startup,
	.shutdown	= nxp_tc94b26_shutdown,
	.hw_params	= nxp_tc94b26_hw_params,
};

/* Platform-specific Nexell I2S driver name */
static char dev_name_i2s[16] = DEV_NAME_I2S ".0";

/* nxp-tc94b26 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link nxp_tc94b26_dai_i2c_0[] = {
	{
		.name = "ASOC-NXP-TC94B26",
		.stream_name = "TC94B26 HiFi",
		.cpu_dai_name 	= dev_name_i2s,				/* nx_snd_i2s_driver name */
		.platform_name  = DEV_NAME_PCM,				/* nx_snd_pcm_driver name */
		.codec_dai_name = "tc94b26-hifi",			/* nxp_tc94b26_dai's name */
		.codec_name 	= "tc94b26-codec.0-001a",	/* nxp_tc94b26_i2c_driver name + i2c bus-address */
		.init			= nxp_tc94b26_init,
		.ops = &nxp_tc94b26_ops,
	},
};

/*  nxp-tc94b26 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link nxp_tc94b26_dai_i2c_1[] = {
	{
		.name = "ASOC-NXP-TC94B26",
		.stream_name = "TC94B26 HiFi",
		.cpu_dai_name 	= dev_name_i2s,				/* nx_snd_i2s_driver name */
		.platform_name  = DEV_NAME_PCM,				/* nx_snd_pcm_driver name */
		.codec_dai_name = "tc94b26-hifi",			/* nxp_tc94b26_dai's name */
		.codec_name 	= "tc94b26-codec.1-001a",	/* nxp_tc94b26_i2c_driver name + i2c bus-address */
		.init			= nxp_tc94b26_init,
		.ops = &nxp_tc94b26_ops,
	},
};

/* tc94b26-nxp  audio machine driver */
static struct snd_soc_card snd_soc_nxp_tc94b26_i2c_0 = {
	.name = "soc-audio-tc94b26",
	.dai_link = &nxp_tc94b26_dai_i2c_0[0],
	.num_links = ARRAY_SIZE(nxp_tc94b26_dai_i2c_0),
	.probe = &nxp_tc94b26_probe,
	.resume_pre = &nxp_tc94b26_resume_pre,
};

static struct snd_soc_card snd_soc_nxp_tc94b26_i2c_1 = {
	.name = "soc-audio-tc94b26",
	.dai_link = &nxp_tc94b26_dai_i2c_1[0],
	.num_links = ARRAY_SIZE(nxp_tc94b26_dai_i2c_1),
	.probe = &nxp_tc94b26_probe,
	.resume_pre = &nxp_tc94b26_resume_pre,
};

/*
 * sysfs interface
 */

/*TODO This sys interface will set mute variable, 
which is used in codec file to mute/unmute the audio
struct tc94b26_private is defined in codec header file.
Assuming that is_board_lucy() is declared and defined in customer's platform.
*/
static ssize_t show_force_audio(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tc94b26_private *nxp_tc94b26;

	if (is_board_lucy())
		nxp_tc94b26 = i2c_get_clientdata(snd_soc_nxp_tc94b26_i2c_1.rtd->codec->control_data);
	else
		nxp_tc94b26 = i2c_get_clientdata(snd_soc_nxp_tc94b26_i2c_0.rtd->codec->control_data);

	return(sprintf(buf,"%u\n", nxp_tc94b26->mute));

}

static ssize_t set_force_audio(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	struct tc94b26_private *nxp_tc94b26;

	if (is_board_lucy())
		nxp_tc94b26 = i2c_get_clientdata(snd_soc_nxp_tc94b26_i2c_1.rtd->codec->control_data);
	else
		nxp_tc94b26 = i2c_get_clientdata(snd_soc_nxp_tc94b26_i2c_0.rtd->codec->control_data);

	if (sscanf(buf, "%x", &value) != 1)
		return -EINVAL;

	nxp_tc94b26->mute = value;
	return count;

}

static DEVICE_ATTR(force_audio, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
        show_force_audio, set_force_audio);

static struct attribute *nxp_tc94b26_attributes[] = {
        &dev_attr_force_audio.attr,
        NULL
};

static struct attribute_group nxp_tc94b26_attr_group = {
	.attrs = nxp_tc94b26_attributes
};

static int nxp_tc94b26_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &snd_soc_nxp_tc94b26_i2c_0;
	struct snd_soc_dai *cpu_dai = NULL;
	struct snd_soc_dai_driver *driver = NULL;

	printk(KERN_DEBUG "%s: \n", __FUNCTION__);

	/*TODO Need to check what is the board name?*/
	if (is_board_lucy())
		card = &snd_soc_nxp_tc94b26_i2c_1;
	else
		card = &snd_soc_nxp_tc94b26_i2c_0;

	/* register card */
	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);
		return ret;
	}

        dev_info(&pdev->dev, "register card %s -> %s\n",
			                card->dai_link->codec_dai_name, card->dai_link->cpu_dai_name);

	ret = sysfs_create_group(&pdev->dev.kobj, &nxp_tc94b26_attr_group);
	return 0;
}

static int __devexit nxp_tc94b26_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	snd_soc_unregister_card(card);
	sysfs_remove_group(&pdev->dev.kobj, &nxp_tc94b26_attr_group);

	return 0;
}

static struct platform_driver nxp_tc94b26_audio_driver = {
	.probe	= nxp_tc94b26_audio_probe,
	.remove	= nxp_tc94b26_audio_remove,
	.driver	= {
		.name	= "nxp-tc94b26-asoc",
		.owner	= THIS_MODULE,
	},
};

static int __init nxp_tc94b26_audio_init(void)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	return platform_driver_register(&nxp_tc94b26_audio_driver);
}
module_init(nxp_tc94b26_audio_init);

static void __exit nxp_tc94b26_audio_exit(void)
{
	printk(KERN_DEBUG "%s: \n", __FUNCTION__);
	platform_driver_unregister(&nxp_tc94b26_audio_driver);
}
module_exit(nxp_tc94b26_audio_exit);

MODULE_AUTHOR("TAEC");
MODULE_DESCRIPTION("ALSA SoC Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:soc-audio");
