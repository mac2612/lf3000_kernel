/*
 * Header for Toshiba's TC94B26 codec driver
 *
 * Copyright:   (C) 2014 Toshiba
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef TC94B26_H
#define TC94B26_H

/* REGISTERS */

#define TC94B26_CHIP_EN                         0x00
#define TC94B26_HP_SET0                         0x01
#define TC94B26_HP_SET1                         0x02
#define TC94B26_MC_SET0                         0x03
#define TC94B26_MC_SET1                         0x04
#define TC94B26_SPK_SET                         0x05
#define TC94B26_DVOLC							0x06
#define TC94B26_DV_LIM							0x07
#define TC94B26_VOL_TOG0						0x08
#define TC94B26_AUTO_TOGGLE_CFG1                0x09
#define TC94B26_INT_EN                          0x0A
#define TC94B26_INT_ACT                         0x0B
#define TC94B26_STATUS                          0x0C
#define TC94B26_GPIO_CFG                        0x0D
#define TC94B26_I2S_CFG0						0x0E
#define TC94B26_I2S_CFG1						0x0F
#define TC94B26_GAIN_SLEW_RATE_CFG0             0x10
#define TC94B26_GAIN_SLEW_RATE_CFG1             0x11
#define TC94B26_CHRG_TIME_CFG                   0x12
#define TC94B26_DISCHRG_TIME_CFG                0x13
#define TC94B26_GAIN_LVL						0x14
#define TC94B26_PWD                             0x15
#define TC94B26_CHIPID                          0x16
#define TC94B26_SW_RESET                        0x17
#define TC94B26_HPF_CFG                         0x18

/** CONFIG**/

#define TC94B26_ENABLE				1
#define TC94B26_RESET				0x6A
#define TC94B26_PW					0xD5
//#define TC94B26_LIMIT_DEFAULT		0x61
#define TC94B26_LIMIT_DEFAULT		0x4d   //setting it to the max volume setting

#define TC94B26_STATUS_BUSY_ST0			(1 << 1)
#define TC94B26_STATUS_BUSY_ST1			(1 << 2)
#define TC94B26_STATUS_PDET_ST			1

#define TC94B26_INT_EN_PDET_EN			1 <<1
#define TC94B26_INT_EN_GPIO_EN			1 <<5


#define TC94B26_HP_EN                         (1 << 7)
#define TC94B26_HP_MUTE                       (1 << 6)
#define TC94B26_HPM_INS                       (1 << 5)
#define TC94B26_HPM_REM                       (1 << 4)

#define TC94B26_MC_EN                         (1 << 7)
#define TC94B26_MC_MUTE                       (1 << 6)

#define TC94B26_SPK_EN                        (1 << 7)
#define TC94B26_SPK_MUTE                      (1 << 6)
#define TC94B26_SPKM_INS                      (1 << 5)
#define TC94B26_SPKM_REM                      (1 << 4)

#define TC94B26_AT				(1 << 7)
#define TC94B26_AVT				(1 << 7)

#define TC94B26_HP_DET                         (1 << 0)

#define TC94B26_HP_DEBOUNCE			(4 << 5)
#define TC94B26_PDET_POL_H			(0)
#define TC94B26_PDET_POL_L			(1 << 4)

#define TC94B26_MODE_MASK			0x07
#define TC94B26_MODE_LEFT_J			0x00
#define TC94B26_MODE_RIGHT_J			0x01
#define TC94B26_MODE_I2S			0x02

#define TC94B26_DBIT_MASK			(3 << 4)
#define TC94B26_DBIT_16				0
#define TC94B26_DBIT_24				(3 << 4)

#define TC94B26_SRATE_MASK			(3 << 4)
#define TC94B26_SRATE_32K			0
#define TC94B26_SRATE_44P1K			(1 << 4)
#define TC94B26_SRATE_48K			(2 << 4)

#define TC94B26_WCLK_POL			(1 << 7)
#define HP_STATUS 0x01
#define HP_NOT_DETECTED 0x00
#define AT_CF0_DFL_VAL              0x80  /* bit7: enable autotoggle*/
#define AT_CF1_DFL_VAL              0x00  /*bit7: enable aito volume change, disabled for FWBOG-254*/
#define MIC_GAIN_DFL_VAL            0x59  /* leaving it at 12dB, increase it if sound is too low.*/
#define HP_AGAIN_DFL_VAL            0x01 << 3 /* bits [4:3] of register GAIN_LVL*/
#define SPKR_AGAIN_DFL_VAL          0x00 << 0 /* bits [2:0] of register GAIN_LVL, gain revised to 24dB after audio quality analysis with Don*/
#define MIC_AGAIN_DFL_VAL           0x02 << 5 //changed to 30dB based on feedback from EE
											/* bits [6:5] of register GAIN_LVL  gain set to 36dB after experiments with Don*/

/* SIZE */
#define TC94B26_LASTREG          TC94B26_HPF_CFG
#define TC94B26_FIRSTREG         TC94B26_CHIP_EN
#define TC94B26_NUMREGS  (TC94B26_LASTREG - TC94B26_FIRSTREG + 1)

#define TC94B26_I2C_ADAPTER_0	0
#define TC94B26_ADDR			0x1A

struct tc94b26_private {
	wait_queue_head_t wait;
	spinlock_t lock;
	struct work_struct tc94b26_work;		/* task			*/
	struct workqueue_struct *tc94b26_tasks;	/* workqueue		*/
	bool	busy;				/* 1 = chip is busy	*/
	int hp_spk_autosw_dis;
	int hp_detect_pol;
	int mute;
	int hp_connected_status;
	char locale[5];
	int hp_gain_offset_dB;
	int spkr_gain;
};

static int tc94b26_write_reg_raw(unsigned int reg, unsigned int value);
static int tc94b26_read_reg_raw(unsigned int reg);

#endif /* TC94B26_H */
