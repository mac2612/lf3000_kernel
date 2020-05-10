/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <mach/platform.h>

#include <asm/system.h>

#include <mach/board_revisions.h>

/* debug macro */
#define DBGOUT(msg...)		{ printk(KERN_INFO msg); }


/*------------------------------------------------------------------------------
 *	GPIO LCD (Backlight Enable/PWM)
 */
#define	CFG_PWM_BLU_CTRL
#define	CFG_IO_LCD_PWR_ENB					((PAD_GPIO_E + 12) | PAD_FUNC_ALT0)		/* GPIO */

/*------------------------------------------------------------------------------
 *	TOUCH
 */
#define	CFG_IO_TOUCH_PENDOWN_DETECT			((PAD_GPIO_B + 25) | PAD_FUNC_ALT0)

/*------------------------------------------------------------------------------
 *	GPIO EEPROM
 */
#define CFG_IO_SPI_EEPROM_WP				((PAD_GPIO_C + 27) | PAD_FUNC_ALT1)		/* GPIO */

/*------------------------------------------------------------------------------
 *	CAMERA Power Down
 */
#define CFG_IO_CAMERA_POWER_DOWN			((PAD_GPIO_E + 13) | PAD_FUNC_ALT0)		/* GPIO */
#define CFG_IO_CAMERA_RESET					((PAD_GPIO_E +  7) | PAD_FUNC_ALT0)		/* GPIO */

/*------------------------------------------------------------------------------
 * get nexell soc pad func.
 */

static int lf3000_get_gpio_strength(U32 Group, U32 BitNumber)
{
	return (NX_GPIO_GetDRV0(Group) * 2 + NX_GPIO_GetDRV1(Group));
}
/*------------------------------------------------------------------------------
 * set nexell soc pad func.
 */

static void set_gpio_strength(U32 Group, U32 BitNumber, U32 mA)
{
	U32 drv1=0, drv0=0;
	U32 drv1_value, drv0_value;

	switch( mA )
	{
		case 0 : drv0 = 0; drv1 = 0; break;
		case 1 : drv0 = 0; drv1 = 1; break;
		case 2 : drv0 = 1; drv1 = 0; break;
		case 3 : drv0 = 1; drv1 = 1; break;
		default: drv0 = 0; drv1 = 0; break;
	}

	drv0_value  = NX_GPIO_GetDRV0(Group);
	drv0_value &= ~((U32)    1 << BitNumber);
	drv0_value |=  ((U32) drv0 << BitNumber);

	drv1_value  = NX_GPIO_GetDRV1( Group);
	drv1_value &= ~((U32)    1 << BitNumber);
	drv1_value |=  ((U32) drv1 << BitNumber);

	NX_GPIO_SetDRV0 (Group, drv0_value);
	NX_GPIO_SetDRV1 (Group, drv1_value);

}

static void bd_gpio_init(void)
{
	int index, bit;
	int mode, func, out, lv, plup, stren;
	U32 gpio;
	
	const U32 (*p_io_pad)[NUMBER_OF_GPIO_MODULE][32];
	
	printk(KERN_INFO "%s.%d:%s system_rev: 0x%X\n", __FILE__, __LINE__, __FUNCTION__, system_rev);

	switch(system_rev) {

	case LF3000_BOARD_BOGOTA:
	case LF3000_BOARD_BOGOTA_EXP_1:
	case LF3000_BOARD_BOGOTA_EXP_2:
	case LF3000_BOARD_BOGOTA_EXP_3:
	case LF3000_BOARD_BOGOTA_EXP_4:
	case LF3000_BOARD_BOGOTA_EXP_5:
	case LF3000_BOARD_BOGOTA_EXP_6:
		p_io_pad = &io_pad_bogota_alpha;
		break;

	case LF3000_BOARD_CABO:
		p_io_pad = &io_pad_cabo_alpha;
		break;

	case LF3000_BOARD_GLASGOW_ALPHA:
		p_io_pad = &io_pad_glasgow_alpha;
		break;

	case LF3000_BOARD_GLASGOW_BETA:
	case LF3000_BOARD_GLASGOW_FEP_984_666:
	case LF3000_BOARD_GLASGOW_FEP_984_800:
	case LF3000_BOARD_GLASGOW_TUNE_DDR1:
	case LF3000_BOARD_GLASGOW_TUNE_DDR2:
	case LF3000_BOARD_GLASGOW_TUNE_DDR3:
	case LF3000_BOARD_GLASGOW_TUNE_DDR4:
	case LF3000_BOARD_GLASGOW_TUNE_GPU1:
	case LF3000_BOARD_GLASGOW_TUNE_GPU2:
	case LF3000_BOARD_GLASGOW_TUNE_GPU3:
		p_io_pad = &io_pad_glasgow_beta;
		break;

	case LF3000_BOARD_XANADU:
	case LF3000_BOARD_XANADU_TI:
	case LF3000_BOARD_XANADU_TI_SS1:
	case LF3000_BOARD_XANADU_TI_SS2:
		p_io_pad = &io_pad_xanadu_alpha;
		break;

	case LF3000_BOARD_R3K:
		p_io_pad = &io_pad_r3k;
		break;

  case LF3000_BOARD_QUITO:
    p_io_pad = &io_pad_quito_alpha;
    break;

	default:
#if   defined(CONFIG_PLAT_NXP4330_BOGOTA)
		p_io_pad = &io_pad_bogota_alpha;
#elif defined(CONFIG_PLAT_NXP4330_CABO)
		p_io_pad = &io_pad_cabo_alpha;
#elif defined(CONFIG_PLAT_NXP4330_GLASGOW)
		p_io_pad = &io_pad_glasgow_alpha;
#elif defined(CONFIG_PLAT_NXP4330_R3K)
		p_io_pad = &io_pad_r3k;
#elif defined(CONFIG_PLAT_NXP4330_XANADU)
		p_io_pad = &io_pad_xanadu_alpha;
#elif defined(CONFIG_PLAT_NXP4330_QUITO)
    p_io_pad = &io_pad_quito_alpha;
#endif
		break;
	}

	/* GPIO pad function */
	for (index = 0; NUMBER_OF_GPIO_MODULE > index; index++) {

		NX_GPIO_ClearInterruptPendingAll(index);

		for (bit = 0; 32 > bit; bit++) {
			gpio  = (*p_io_pad)[index][bit];
			func  = PAD_GET_FUNC(gpio);
			mode  = PAD_GET_MODE(gpio);
			lv    = PAD_GET_LEVEL(gpio);
			stren = PAD_GET_STRENGTH(gpio);
			plup  = PAD_GET_PULLUP(gpio);

			/* get pad alternate function (0,1,2,4) */
			switch (func) {
			case PAD_GET_FUNC(PAD_FUNC_ALT0): func = NX_GPIO_PADFUNC_0;	break;
			case PAD_GET_FUNC(PAD_FUNC_ALT1): func = NX_GPIO_PADFUNC_1;	break;
			case PAD_GET_FUNC(PAD_FUNC_ALT2): func = NX_GPIO_PADFUNC_2;	break;
			case PAD_GET_FUNC(PAD_FUNC_ALT3): func = NX_GPIO_PADFUNC_3;	break;
			default: printk("ERROR, unknown alt func (%d.%02d=%d)\n", index, bit, func);
				continue;
			}

			switch (mode) {
			case PAD_GET_MODE(PAD_MODE_ALT): out = 0;
			case PAD_GET_MODE(PAD_MODE_IN ): out = 0;
			case PAD_GET_MODE(PAD_MODE_INT): out = 0; break;
			case PAD_GET_MODE(PAD_MODE_OUT): out = 1; break;
			default: printk("ERROR, unknown io mode (%d.%02d=%d)\n", index, bit, mode);
				continue;
			}

			NX_GPIO_SetPadFunction(index, bit, func);
			NX_GPIO_SetOutputEnable(index, bit, (out ? CTRUE : CFALSE));
			NX_GPIO_SetOutputValue(index, bit,  (lv  ? CTRUE : CFALSE));
			NX_GPIO_SetInterruptMode(index, bit, (lv));
			NX_GPIO_SetPullMode(index, bit, plup);
			set_gpio_strength(index, bit, stren); /* pad strength */
		}
	}
}

static void bd_alive_init(void)
{
	int index, bit;
	int mode, out, lv, plup, detect;
	U32 gpio;

	const U32 (*p_alv_pad)[ALVPADS];

	switch(system_rev) {

	case LF3000_BOARD_BOGOTA:
	case LF3000_BOARD_BOGOTA_EXP_1:
	case LF3000_BOARD_BOGOTA_EXP_2:
	case LF3000_BOARD_BOGOTA_EXP_3:
	case LF3000_BOARD_BOGOTA_EXP_4:
	case LF3000_BOARD_BOGOTA_EXP_5:
	case LF3000_BOARD_BOGOTA_EXP_6:
		p_alv_pad = &alv_pad_bogota_alpha;
		break;

	case LF3000_BOARD_CABO:
		p_alv_pad = &alv_pad_cabo_alpha;
		break;

	case LF3000_BOARD_GLASGOW_ALPHA:
		p_alv_pad = &alv_pad_glasgow_alpha;
		break;

	case LF3000_BOARD_GLASGOW_BETA:
	case LF3000_BOARD_GLASGOW_FEP_984_666:
	case LF3000_BOARD_GLASGOW_FEP_984_800:
	case LF3000_BOARD_GLASGOW_TUNE_DDR1:
	case LF3000_BOARD_GLASGOW_TUNE_DDR2:
	case LF3000_BOARD_GLASGOW_TUNE_DDR3:
	case LF3000_BOARD_GLASGOW_TUNE_DDR4:
	case LF3000_BOARD_GLASGOW_TUNE_GPU1:
	case LF3000_BOARD_GLASGOW_TUNE_GPU2:
	case LF3000_BOARD_GLASGOW_TUNE_GPU3:
		p_alv_pad = &alv_pad_glasgow_beta;
		break;

	case LF3000_BOARD_R3K:
		p_alv_pad = &alv_pad_r3k;
		break;

	case LF3000_BOARD_XANADU:
	case LF3000_BOARD_XANADU_TI:
	case LF3000_BOARD_XANADU_TI_SS1:
	case LF3000_BOARD_XANADU_TI_SS2:
		p_alv_pad = &alv_pad_xanadu_alpha;
		break;

  case LF3000_BOARD_QUITO:
    p_alv_pad = &alv_pad_quito_alpha;
    break;

	default:
#if    defined(CONFIG_PLAT_NXP4330_BOGOTA)
		p_alv_pad = &alv_pad_bogota_alpha;
#elif  defined(CONFIG_PLAT_NXP4330_CABO)
        p_alv_pad = &alv_pad_cabo_alpha;
#elif  defined(CONFIG_PLAT_NXP4330_GLASGOW)
        p_alv_pad = &alv_pad_glasgow_alpha;
#elif  defined(CONFIG_PLAT_NXP4330_R3K)
		p_alv_pad = &alv_pad_r3k;
#elif  defined(CONFIG_PLAT_NXP4330_XANADU)
		p_alv_pad = &alv_pad_xanadu_alpha;
#elif  defined(CONFIG_PLAT_NXP4330_QUITO)
        p_alv_pad = &alv_pad_quito_alpha;
#endif
		break;
	}
	
	index = sizeof(*p_alv_pad)/sizeof((*p_alv_pad)[0]);

	/* Alive pad function */
	for (bit = 0; index > bit; bit++) {
		NX_ALIVE_ClearInterruptPending(bit);
		gpio = (*p_alv_pad)[bit];
		mode = PAD_GET_MODE(gpio);
		lv   = PAD_GET_LEVEL(gpio);
		plup = PAD_GET_PULLUP(gpio);

		switch (mode) {
		case PAD_GET_MODE(PAD_MODE_IN ):
		case PAD_GET_MODE(PAD_MODE_INT): out = 0; break;
		case PAD_GET_MODE(PAD_MODE_OUT): out = 1; break;
		case PAD_GET_MODE(PAD_MODE_ALT):
			printk("ERROR, alive.%d not support alt function\n", bit);
			continue;
		default :
			printk("ERROR, unknown alive mode (%d=%d)\n", bit, mode);
			continue;
		}

		NX_ALIVE_SetOutputEnable(bit, (out ? CTRUE : CFALSE));
		NX_ALIVE_SetOutputValue (bit, (lv));
		NX_ALIVE_SetPullUpEnable(bit, (plup & 1 ? CTRUE : CFALSE));
		/* set interrupt mode */
		for (detect = 0; 6 > detect; detect++) {
			if (mode == PAD_GET_MODE(PAD_MODE_INT))
				NX_ALIVE_SetDetectMode(detect, bit, (lv == detect ? CTRUE : CFALSE));
			else
				NX_ALIVE_SetDetectMode(detect, bit, CFALSE);
		}
		NX_ALIVE_SetDetectEnable(bit, (mode == PAD_MODE_INT ? CTRUE : CFALSE));
	}
}

/*------------------------------------------------------------------------------
 * board interface
 * 		replaces Nexell's nxp_board_base_init in mach-nxp4330/xxx/board.c
 */
void lf_board_base_init(void)
{
	bd_gpio_init ();
	bd_alive_init();
	DBGOUT("%s : done gpio initialize ...\n\n", CFG_SYS_BOARD_NAME);
}

