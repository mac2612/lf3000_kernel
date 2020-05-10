#ifndef __DIDJ_LFP100_H__
#define __DIDJ_LFP100_H__

#include <mach/lfp100.h>

/* register to value */
u8 lfp100_common_settings[][2] = {
	{LFP100_A_CONTROL, 0x00},  /* disable all DAC/ADC */
	{LFP100_FORMAT_PW, 0x66},  /* I2S Format Register, 32 bit frame   */
	{LFP100_A_CONTROL, 0x01},  /* enable microphone */
	{LFP100_MGAIN, 0x80},	   /* set master gain to 0dB and mute	  */
	{LFP100_VLIMIT_PW, 0x00},  /* allow full volume range */
	{LFP100_GAINADJ_PW, 0x59}, /* adjust hp:+5dB, mic:+36dB, spk:+21dB */
	{LFP100_SLEW_PW, 0x5E},	   /* force DCDC1 and DCDC2 to PWM mode */
	/*
	 * FIXME for 1p2 change to CHMOD=1 (per Ivo)
	 * this should speed the switching between headphone and speaker
	 */
	{LFP100_A_APOP_PW, 0x1D},  /* CHMOD=0, CHTIM=1200ms, RPOP=20K */
	{LFP100_FILTER, 0x01},  /* disable clipping PGA_CLIP=0 */
	{LFP100_MICGAIN, 0x08},    /* set default mic gain to +8dB */
};

u8 lfp100_rio_settings[][2] = {
	{LFP100_GAINADJ_PW, 0x5B}, /* adjust hp:+5dB, mic:+36dB, spk:+15dB */
};

u8 lfp100_cabo_settings[][2] = {
	{LFP100_GAINADJ_PW, 0x5A}, /* adjust hp:+5dB, mic:+36dB, spk:+18dB */
};

#endif /* __DIDJ_LFP100_H__ */
