/*
 * Toshiba's TC94B26 ASoC codec driver
 *
 * Author: TAEC 
 *
 * Copyright:   (C) 2014 Toshiba
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* #define DEBUG */		/* for KERN_DEBUG tracers */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <sound/pcm_params.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/lf3000/gpio.h>
#include <sound/tc94b26.h>
#include "tc94b26.h"


//#define TIMEOUT		msecs_to_jiffies(50)  //50 creates 5 jiffies and that is not enough
#define TIMEOUT		msecs_to_jiffies(100000)  /*FIXME this number might be too big, fine tune later*/

#define CODEC_NAME "tc94b26-codec"

static struct tc94b26_private *local_tc94b26_chip = NULL;


/*
 * local i2c read reg routine
 */
static int tc94b26_read_reg_raw(unsigned int reg)
{
	struct i2c_adapter* i2c;
	struct i2c_msg i2c_messages[2];
	u8 buf[2];
	int ret;

	i2c = i2c_get_adapter(TC94B26_I2C_ADAPTER_0);

	if (!i2c) {
		printk(KERN_ERR "%s.%d return -EIO\n",
				__FUNCTION__, __LINE__);

		return -1;
	}

	buf[0] = 0xFF & reg;           /* read this register */
	buf[1] = 0;

	/* write portion */
	i2c_messages[0].addr = TC94B26_ADDR;
	i2c_messages[0].buf = buf;
	i2c_messages[0].len = 1;
	i2c_messages[0].flags = 0;      /* write */

	/* read portion */
	i2c_messages[1].addr = TC94B26_ADDR;
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
	printk(KERN_INFO "%s: reg=0x%02x: data=0x%02x\n", __FUNCTION__, reg, buf[0]);
#endif

	return 0xFF & buf[0]; //[1];
}

/*
 * local TC94B26 write reg routine, no range checking
 */
static int tc94b26_write_reg_raw(unsigned int reg, unsigned int value)
{
	struct i2c_adapter* i2c;
	struct i2c_msg i2c_messages[2];
	u8 buf[2];

	/* write new register value */
	i2c = i2c_get_adapter(TC94B26_I2C_ADAPTER_0);

	if (!i2c) {
		printk(KERN_ERR "%s.%d return -EIO\n",
			__FUNCTION__, __LINE__);
		return -EIO;
	}

	buf[0] = 0xff & reg;
	buf[1] = 0xFF & value;

	/* write portion */
	i2c_messages[0].addr = TC94B26_ADDR;
	i2c_messages[0].buf = buf;
	i2c_messages[0].len = 2;
	i2c_messages[0].flags = 0;	/* write */

	if (i2c_transfer(i2c, i2c_messages, 1) < 0) {
		i2c_put_adapter(i2c);
		printk(KERN_ERR "%s.%d return -EIO\n",
			__FUNCTION__, __LINE__);
		return -EIO;
	}

#if DEBUG_I2C
	printk(KERN_INFO "%s: 0x%02x: 0x%02x\n", __FUNCTION__, buf[0], buf[1]);
#endif

	i2c_put_adapter(i2c);
	return 0;
}
#define ZERO_DB	0x41  //register value for zero gain

/* On Bogota, digital gain for speaker and headphone are not the same.
 * digital gain for speaker is higher than the headphone.
 * The difference depends on locale. There is also a fixed
 * 3dB component that was introduced to avoid the distortion
 * on headphone. The difference in gain is stored in
 * local_tc94b26_chip->hp_gain_offset_dB. Current gain level
 * received from lfd is stored in local_tc94b26_chip->spkr_gain.
 * This routine calculates the gain value based on spkr gain and offset.
 * @output : new value for the register DVOLC*/
u8 convert_gain_level_for_hp(void)
{
	int db,new_db;
	u8 new_val;
	if(local_tc94b26_chip->spkr_gain>=ZERO_DB)
		db=(local_tc94b26_chip->spkr_gain-ZERO_DB)/2; //1 count=1/2 dB for gains>0dB
	else
		db=(ZERO_DB-local_tc94b26_chip->spkr_gain)*(-1); //1 count=1 dB for gains< 0dB

	/*if headphones are plugged in,we are switching to headphone so we need to reduce gain  */
	new_db=db-local_tc94b26_chip->hp_gain_offset_dB;
	if(new_db>=0)
		new_val=(new_db*2)+ZERO_DB;
	else
		new_val=new_db+ZERO_DB;
	return new_val;
}
//FWBOG-941
#define US_CA_OFFSET 		3 //headphone gain offset for US and canada
#define NON_US_CA_OFFSET 	9 //headphone gain offset for all other locales

int tc94b26_set_locale(const char *locale)
{
	strncpy(local_tc94b26_chip->locale, locale, sizeof(local_tc94b26_chip->locale));
	if ((!strncasecmp(local_tc94b26_chip->locale, "en-us",5))
	 || (!strncasecmp(local_tc94b26_chip->locale, "en-ca",5))) {

		local_tc94b26_chip->hp_gain_offset_dB = US_CA_OFFSET;
	}
	else
		local_tc94b26_chip->hp_gain_offset_dB = NON_US_CA_OFFSET;

//	printk(KERN_INFO "%s.%d locale=%s db offset=%d\n",__FUNCTION__, __LINE__,local_tc94b26_chip->locale,local_tc94b26_chip->hp_gain_offset_dB );
	return 1;
}
EXPORT_SYMBOL(tc94b26_set_locale);

char * tc94b26_get_locale(void)
{
//	printk(KERN_INFO "%s.%d \n",__FUNCTION__, __LINE__);
	return local_tc94b26_chip->locale;

}
EXPORT_SYMBOL(tc94b26_get_locale);

static int tc94b36_available(void)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&local_tc94b26_chip->lock, flags);
	ret = !local_tc94b26_chip->busy;
	spin_unlock_irqrestore(&local_tc94b26_chip->lock, flags);

	return ret;
}


static int tc94b26_wait_spk(struct snd_soc_codec *codec)
{
	unsigned char status;
	unsigned long timeout;

	timeout = jiffies + TIMEOUT;
	//printk(KERN_INFO "%s %d jiffies=%d timeout=%d ",__FUNCTION__, __LINE__,jiffies,timeout);

	do {
		status = snd_soc_read(codec, TC94B26_STATUS);
		if (status < 0)
			return status;

		if (time_after(jiffies, timeout)) {
			printk(KERN_DEBUG "%s %d status read timed out",__FUNCTION__, __LINE__);
			return -ETIMEDOUT;
		}
		cpu_relax();
	} while (status & (TC94B26_STATUS_BUSY_ST0));

	return 0;
}

static int tc94b26_wait_mic(struct snd_soc_codec *codec)
{
        unsigned char status;
        unsigned long timeout;

        timeout = jiffies + TIMEOUT;

        do {
                status = snd_soc_read(codec, TC94B26_STATUS);
                if (status < 0)
                        return status;

                if (time_after(jiffies, timeout)) {
                	printk(KERN_DEBUG "%s %d status read timed out",__FUNCTION__, __LINE__);
                        return -ETIMEDOUT;
                }
                cpu_relax();
        } while (status & (TC94B26_STATUS_BUSY_ST1));

        return 0;
}
static int tc94b26_protected_write(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	int ret;

	/* Unlock register for write operation */
	ret = snd_soc_write(codec, TC94B26_PWD, TC94B26_PW);
	ret |= snd_soc_write(codec, reg, value);

	return ret;
}

static int tc94b26_protected_update_bits(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int mask, unsigned int value)
{
	bool change;
	unsigned int old, new;
	int ret;

	ret = snd_soc_read(codec, reg);
	if (ret < 0)
		return ret;

	old = ret;
	new = (old & ~mask) | (value & mask);
	change = old != new;

	if (change)
		ret = tc94b26_protected_write(codec, reg, new);

	if (ret < 0)
		return ret;

	return change;
}

int tc94b26_mute(struct snd_soc_codec *codec)
{
	struct tc94b26_private *tc94b26  = snd_soc_codec_get_drvdata(codec);
	int status;	

	if (tc94b26_wait_spk(codec))
		return -ETIMEDOUT;

	/* Mute HP */
	snd_soc_update_bits(codec, TC94B26_HP_SET0, TC94B26_HP_MUTE,
			TC94B26_HP_MUTE);
	if (tc94b26_wait_spk(codec))
		return -ETIMEDOUT;

	/* Mute SPK */
	snd_soc_update_bits(codec, TC94B26_SPK_SET, TC94B26_SPK_MUTE,
			TC94B26_SPK_MUTE);

	return 0;
}

int  tc94b26_unmute(struct snd_soc_codec *codec)
{
	struct tc94b26_private *tc94b26  = snd_soc_codec_get_drvdata(codec);
	int status;	

	if (tc94b26_wait_spk(codec))
		return -ETIMEDOUT;

	/* Unmute HP */
	snd_soc_update_bits(codec, TC94B26_HP_SET0, TC94B26_HP_MUTE, 0);
	if (tc94b26_wait_spk(codec))
		return -ETIMEDOUT;

	/* Unmute SPK */
	snd_soc_update_bits(codec, TC94B26_SPK_SET, TC94B26_SPK_MUTE, 0);

	return 0;
}

static int tc94b26_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG0,
			       TC94B26_DBIT_MASK, TC94B26_DBIT_16);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG0,
			       TC94B26_DBIT_MASK, TC94B26_DBIT_24);
		break;
	default:
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 32000:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG1,
			       TC94B26_SRATE_MASK, TC94B26_SRATE_32K);
		break;
	case 44100:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG1,
			       TC94B26_SRATE_MASK, TC94B26_SRATE_44P1K);
		break;
	case 48000:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG1,
			       TC94B26_SRATE_MASK, TC94B26_SRATE_48K);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tc94b26_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	if ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)
		return -EINVAL;

	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG0,
				TC94B26_MODE_MASK, TC94B26_MODE_I2S);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG0,
				TC94B26_MODE_MASK, TC94B26_MODE_LEFT_J);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG0,
				TC94B26_MODE_MASK, TC94B26_MODE_RIGHT_J);
		break;
	default:
		return -EINVAL;
	}

	switch (format & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG1,
				TC94B26_WCLK_POL, 0);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		tc94b26_protected_update_bits(codec, TC94B26_I2S_CFG1,
				TC94B26_WCLK_POL, TC94B26_WCLK_POL);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tc94b26_dai_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tc94b26_private *tc94b26 = snd_soc_codec_get_drvdata(codec);

	printk(KERN_DEBUG "%s: %s, mute=%d, playback=%d, capture=%d\n",
			__func__, dai->name,
			mute, dai->playback_active, dai->capture_active);

	/* set bits as needed */
	if (mute || tc94b26->mute)
		tc94b26_mute(codec);
	else
		tc94b26_unmute(codec);

	return 0;
}

static int tc94b26_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {

		if (tc94b26_wait_mic(codec))
			return -ETIMEDOUT;
		snd_soc_update_bits(codec, TC94B26_MC_SET0,
				TC94B26_MC_EN, TC94B26_MC_EN);
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		if (tc94b26_wait_spk(codec))
			return -ETIMEDOUT;

		snd_soc_update_bits(codec, TC94B26_HP_SET0,
				TC94B26_HP_EN, TC94B26_HP_EN);
		if (tc94b26_wait_spk(codec))
			return -ETIMEDOUT;

		snd_soc_update_bits(codec, TC94B26_SPK_SET,
				TC94B26_SPK_EN, TC94B26_SPK_EN);
	}
	return 0;
}

static int tc94b26_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {

		if (tc94b26_wait_mic(codec))
			return -ETIMEDOUT;
		snd_soc_update_bits(codec, TC94B26_MC_SET0,
				TC94B26_MC_EN, 0);
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		if (tc94b26_wait_spk(codec))
			return -ETIMEDOUT;
		snd_soc_update_bits(codec, TC94B26_HP_SET0,
				TC94B26_HP_EN, 0);
		if (tc94b26_wait_spk(codec))
			return -ETIMEDOUT;

		snd_soc_update_bits(codec, TC94B26_SPK_SET,
				TC94B26_SPK_EN, 0);

	}

	return 0;
}

static int tc94b26_volsw_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{

		struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
		struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
		unsigned int reg = mc->reg;
		unsigned short val, mask;
		mask=0x7F;  //bit 7 of DVOLC register is read only
		val = ucontrol->value.integer.value[0];
		local_tc94b26_chip->spkr_gain=val;  //store the gain to calculate headphone gain
		if(local_tc94b26_chip->hp_connected_status){
			val=convert_gain_level_for_hp();
		}
		return snd_soc_update_bits(codec, reg, mask, val);
}

static int tc94b26_gain_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	//struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	unsigned short val;
	int change;

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = mask - val;
	mask <<= shift;
	val <<= shift;

	change = snd_soc_test_bits(codec, val, mask, reg);
	if (change)
		tc94b26_protected_update_bits(codec, reg, mask, val);

	return change;
}

static int tc94b26_put_mute(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	//struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tc94b26_private *tc94b26  = snd_soc_codec_get_drvdata(codec);

	tc94b26->mute = !ucontrol->value.integer.value[0];

	if (tc94b26->mute)
		tc94b26_mute(codec);
	else
		tc94b26_unmute(codec);
	return 0;
}

static const char *mic_bias[] = {
	"2.0V", "1.8V"
};
/*
 * master Volume and capture volume are mapped as:
 * -65dB (-infinity) to 0dB - 0 to 0x41 : 1dB step
 * 0 to 32dB (0 to 16dB actual) - 0x41 to 0x61 :1dB step (0.5dB actual)
 */
static const DECLARE_TLV_DB_MINMAX(mastervol_tlv, -6500, 3200);
static const DECLARE_TLV_DB_MINMAX(capturevol_tlv, -6500, 3200);
static const DECLARE_TLV_DB_MINMAX(micgain_tlv, 2700, 3600); // +27db .. +36db
static const DECLARE_TLV_DB_MINMAX(hpgain_tlv, -100, 800); // -1db .. +8db
static const DECLARE_TLV_DB_MINMAX(spkgain_tlv, 300, 2400); // +3db .. +24db

static const struct snd_kcontrol_new tc94b26_snd_controls[] = {
		SOC_SINGLE_EXT_TLV("Master Playback Volume", TC94B26_DVOLC, 0, 0x61, 0,
				snd_soc_get_volsw, tc94b26_volsw_put,mastervol_tlv),
		SOC_SINGLE_EXT_TLV("Master Volume Limiter", TC94B26_DV_LIM, 0, 0x61, 0,
				snd_soc_get_volsw, tc94b26_gain_put, mastervol_tlv),
		SOC_SINGLE_TLV("Capture Volume", TC94B26_MC_SET1, 0, 0x61, 0,
				capturevol_tlv),

		SOC_SINGLE_EXT("Master Playback Switch", TC94B26_SPK_SET, 6, 1, 1,
				snd_soc_get_volsw, tc94b26_put_mute),


	//FIXME: sesters - this enum looks out of place
	//SOC_ENUM_SINGLE(TC94B26_MC_SET0, 5, 2, mic_bias),
	
	SOC_SINGLE_EXT_TLV("Microphone Gain", TC94B26_GAIN_LVL,	5, 0x3, 1,
		snd_soc_get_volsw, tc94b26_gain_put, micgain_tlv),
	SOC_SINGLE_EXT_TLV("Headphone Gain", TC94B26_GAIN_LVL, 3, 0x3, 1,
		snd_soc_get_volsw, tc94b26_gain_put, hpgain_tlv),
	SOC_SINGLE_EXT_TLV("Speaker Gain", TC94B26_GAIN_LVL, 0, 0x7, 1,
		snd_soc_get_volsw, tc94b26_gain_put, spkgain_tlv),
};

static void tc94b26_monitor_task(struct work_struct *work)
{
	unsigned int hp_status=0,dvol=0,i=20000;
	unsigned long flags;

	/* serialize access to chip*/
	spin_lock_irqsave(&local_tc94b26_chip->lock, flags);
	while (local_tc94b26_chip->busy) {
		spin_unlock_irqrestore(&local_tc94b26_chip->lock, flags);
		if (wait_event_interruptible(local_tc94b26_chip->wait,
			(tc94b36_available())))
			return;
		spin_lock_irqsave(&local_tc94b26_chip->lock, flags);
	}

	local_tc94b26_chip->busy = 1;
	spin_unlock_irqrestore(&local_tc94b26_chip->lock, flags);


	tc94b26_read_reg_raw(TC94B26_INT_ACT);   //clear interrupt
	hp_status= tc94b26_read_reg_raw(TC94B26_STATUS)& TC94B26_STATUS_PDET_ST;
	if (hp_status!=local_tc94b26_chip->hp_connected_status){
		if(hp_status) //convert gain if headphone is connected
			dvol=convert_gain_level_for_hp();
		else
			dvol=local_tc94b26_chip->spkr_gain;
		while((tc94b26_read_reg_raw(TC94B26_STATUS) & TC94B26_STATUS_BUSY_ST0)&& (i>0)){
			i--;
		}
		if(i<0)
			printk(KERN_ERR "\n %s,%d tc94b26 busy",__func__,__LINE__);
		tc94b26_write_reg_raw(TC94B26_DVOLC,dvol);
		local_tc94b26_chip->hp_connected_status=hp_status;
	}
		/* release tc94b26 */
	spin_lock_irqsave(&local_tc94b26_chip->lock, flags);
	local_tc94b26_chip->busy = 0;
	spin_unlock_irqrestore(&local_tc94b26_chip->lock, flags);
	wake_up_interruptible(&local_tc94b26_chip->wait);
	return;
}

static irqreturn_t tc94b26_chip_irq(int irq, void *priv)
{
	struct tc94b26_private *chip = (struct tc94b26_private *)priv;
	//printk(KERN_DEBUG "\n %s",__func__);
	queue_work(chip->tc94b26_tasks, &chip->tc94b26_work);
	return IRQ_HANDLED;

}

static int tc94b26_codec_probe(struct snd_soc_codec *codec)
{
	struct tc94b26_private *tc94b26 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	local_tc94b26_chip = tc94b26;
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	if (tc94b26_wait_mic(codec))
			return -ETIMEDOUT;
	if (tc94b26_wait_spk(codec))
		return -ETIMEDOUT;

	/* Enable codec */
	tc94b26_protected_write(codec, TC94B26_CHIP_EN, TC94B26_ENABLE);


	/* Soft Reset Codec */
	tc94b26_protected_write(codec, TC94B26_SW_RESET, TC94B26_RESET);

	/* Configure slew rate */
	//snd_soc_write(codec, TC94B26_GSR_CFG0, 0x1B);
	//snd_soc_write(codec, TC94B26_GSR_CFG1, 0x1B);

	/* Limit volume to +16dB */
	tc94b26_protected_write(codec, TC94B26_DV_LIM, TC94B26_LIMIT_DEFAULT);

	if (tc94b26_wait_spk(codec))
		return -ETIMEDOUT;


	/* Unmute headphone + Speaker */
	snd_soc_write(codec, TC94B26_HP_SET0, 0);
	if (tc94b26_wait_spk(codec))
		return -ETIMEDOUT;

	snd_soc_write(codec, TC94B26_SPK_SET, 0);
	if (tc94b26_wait_mic(codec))
		return -ETIMEDOUT;
	/* unmute microphone */
	snd_soc_write(codec, TC94B26_MC_SET0, 0);

	if (!!tc94b26->hp_detect_pol) {
		snd_soc_write(codec, TC94B26_HP_SET1,
				(TC94B26_HP_DEBOUNCE | TC94B26_PDET_POL_H));
	} else {
		snd_soc_write(codec, TC94B26_HP_SET1, TC94B26_HP_DEBOUNCE);
	}

	/* Auto toggle setting */
	snd_soc_write(codec, TC94B26_VOL_TOG0, AT_CF0_DFL_VAL);		/*enable autoggle */
	snd_soc_write(codec, TC94B26_AUTO_TOGGLE_CFG1, AT_CF1_DFL_VAL);	/*disable automatic volume change
																	  * when toggling between SPK and HP */

	/* Mic Gain setting default value */
	snd_soc_write(codec, TC94B26_MC_SET1, MIC_GAIN_DFL_VAL); 
	/* speaker and headphone AGAIN default value */
	tc94b26_protected_write(codec, TC94B26_GAIN_LVL, (HP_AGAIN_DFL_VAL|SPKR_AGAIN_DFL_VAL|MIC_AGAIN_DFL_VAL));
    /*save headphone connect/disconnect status*/
	tc94b26->hp_connected_status= tc94b26_read_reg_raw(TC94B26_STATUS)& TC94B26_STATUS_PDET_ST;
	tc94b26->hp_gain_offset_dB=3;   //default to 3dB assuming locale is US or canada

	/* request GPIO input for TC94B26 IRQ */
	ret = gpio_request_one(AUDIO_INT, GPIOF_IN, "AUDIO INT");
	if (ret) {
		printk(KERN_ERR "%s.%d: failed to get AUDIO_INT GPIO %d\n",
				__FUNCTION__, __LINE__, AUDIO_INT);
		gpio_free(AUDIO_INT);
		return ret;
	}
	ret = request_irq(gpio_to_irq(AUDIO_INT),
			tc94b26_chip_irq,
			IRQF_TRIGGER_RISING | IRQF_DISABLED,
			"TC94B26 IRQ", tc94b26);
	if (ret) {
		printk(KERN_ERR "%s.%d: failed to get TC94B26 IRQ\n",
				__FUNCTION__, __LINE__);
		gpio_free(AUDIO_INT);
				return ret;
	}
	disable_irq(gpio_to_irq(AUDIO_INT));

	init_waitqueue_head(&tc94b26->wait);
	spin_lock_init(&tc94b26->lock);
	/* initialize worker thread which processes irq */
	tc94b26->tc94b26_tasks = create_singlethread_workqueue("tc94b26 tasks");
	INIT_WORK(&tc94b26->tc94b26_work, tc94b26_monitor_task);

	snd_soc_write(codec, TC94B26_INT_EN, (TC94B26_INT_EN_PDET_EN|TC94B26_INT_EN_GPIO_EN));
	enable_irq(gpio_to_irq(AUDIO_INT));
	return ret;
}

static struct snd_soc_dai_ops tc94b26_dai_ops = {
	.startup	= tc94b26_startup,
	.shutdown	= tc94b26_shutdown,
	.hw_params	= tc94b26_hw_params,
	.set_fmt	= tc94b26_set_dai_fmt,
	.digital_mute	= tc94b26_dai_mute,
};

#define TC94C26_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

#define TC94B26_RATES	(SNDRV_PCM_RATE_32000 | \
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 )


struct snd_soc_dai_driver tc94b26_dai = {
	.name		= "tc94b26-hifi",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= TC94B26_RATES,
		.formats	= TC94C26_FORMATS,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 2, //1,
		.channels_max	= 2, //1, Set channels as 2 by TAEC 
		.rates		= TC94B26_RATES,
		.formats	= TC94C26_FORMATS,
	},
	.ops		= &tc94b26_dai_ops,
};

static struct snd_soc_codec_driver soc_codec_device_tc94b26 = {
	.probe			= tc94b26_codec_probe,
	.controls		= tc94b26_snd_controls,
	.num_controls		= ARRAY_SIZE(tc94b26_snd_controls),
//	.reg_cache_size		= TC94B26_NUMREGS,
//	.reg_word_size		= sizeof(u8),
};

static int tc94b26_i2c_probe(struct i2c_client *i2c_client,
		const struct i2c_device_id *id)
{
	struct tc94b26_private *tc94b26;
	struct tc94b26_pdata *pdata = i2c_client->dev.platform_data;
	int ret;

	dev_info(&i2c_client->dev, "i2c-0x%X %s: %s\n", i2c_client->addr,
			__FUNCTION__, CODEC_NAME);

	tc94b26 = kzalloc(sizeof(struct tc94b26_private), GFP_KERNEL);
	if (!tc94b26) {
		dev_err(&i2c_client->dev, "could not allocate codec\n");
		return -ENOMEM;
	}

	if (pdata) {
		tc94b26->hp_detect_pol = pdata->hp_detect_pol;
		tc94b26->hp_spk_autosw_dis = pdata->hp_spk_autosw_dis;
	}

	i2c_set_clientdata(i2c_client, tc94b26);

	ret = snd_soc_register_codec(&i2c_client->dev,
			&soc_codec_device_tc94b26, &tc94b26_dai, 1);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "failed to register DAI\n");
		goto out_codec;
	}

	return 0;

out_codec:
	kfree(tc94b26);

	return ret;
}

static int tc94b26_i2c_remove(struct i2c_client *i2c_client)

{
	struct tc94b26_private *tc94b26 = i2c_get_clientdata(i2c_client);

	kfree(tc94b26);
	
	return 0;
}

static struct i2c_device_id tc94b26_id[] = {
	{"tc94b26", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tc94b26);

/* I2C bus identification */
static struct i2c_driver tc94b26_i2c_driver = {
	.driver = {
		.name	= "tc94b26-codec",
		.owner	= THIS_MODULE,
	},
	.id_table	= tc94b26_id,
	.probe		= tc94b26_i2c_probe,
	.remove		= tc94b26_i2c_remove,
};

static int __init tc94b26_init(void)
{
	int ret = i2c_add_driver(&tc94b26_i2c_driver);
	printk(KERN_DEBUG "%s: %s %d\n", __FUNCTION__, CODEC_NAME, ret);
	return ret;
}
module_init(tc94b26_init);

static void __exit tc94b26_exit(void)
{
	i2c_del_driver(&tc94b26_i2c_driver);
}
module_exit(tc94b26_exit);

MODULE_AUTHOR("TAEC");
MODULE_DESCRIPTION("TC94B26 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
