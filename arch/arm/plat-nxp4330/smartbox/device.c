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
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/irq.h>
#include <linux/gpio.h>

/* nexell soc headers */
#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>

/*------------------------------------------------------------------------------
 * BUS Configure
 */
#if (CFG_BUS_RECONFIG_ENB == 1)
#include <mach/nxp4330_bus.h>

const u16 g_DrexQoS[2] = {
	0x100,		// S0
	0xFFF		// S1, Default value
};

const u8 g_TopBusSI[8] = {
	TOPBUS_SI_SLOT_DMAC0,
	TOPBUS_SI_SLOT_USBOTG,
	TOPBUS_SI_SLOT_USBHOST0,
	TOPBUS_SI_SLOT_DMAC1,
	TOPBUS_SI_SLOT_SDMMC,
	TOPBUS_SI_SLOT_USBOTG,
	TOPBUS_SI_SLOT_USBHOST1,
	TOPBUS_SI_SLOT_USBOTG
};

const u8 g_BottomBusSI[8] = {
	BOTBUS_SI_SLOT_1ST_ARM,
	BOTBUS_SI_SLOT_MALI,
	BOTBUS_SI_SLOT_DEINTERLACE,
	BOTBUS_SI_SLOT_1ST_CODA,
	BOTBUS_SI_SLOT_2ND_ARM,
	BOTBUS_SI_SLOT_SCALER,
	BOTBUS_SI_SLOT_TOP,
	BOTBUS_SI_SLOT_2ND_CODA
};

const u8 g_BottomQoSSI[2] = {
	1,	// Tidemark
	(1<<BOTBUS_SI_SLOT_1ST_ARM) |	// Control
	(1<<BOTBUS_SI_SLOT_2ND_ARM) |
	(1<<BOTBUS_SI_SLOT_MALI) |
	(1<<BOTBUS_SI_SLOT_TOP) |
	(1<<BOTBUS_SI_SLOT_DEINTERLACE) |
	(1<<BOTBUS_SI_SLOT_1ST_CODA)
};

const u8 g_DispBusSI[3] = {
	DISBUS_SI_SLOT_1ST_DISPLAY,
	DISBUS_SI_SLOT_2ND_DISPLAY,
	DISBUS_SI_SLOT_2ND_DISPLAY  //DISBUS_SI_SLOT_GMAC
};
#endif	/* #if (CFG_BUS_RECONFIG_ENB == 1) */

/*------------------------------------------------------------------------------
 * CPU Frequence
 */
#if defined(CONFIG_ARM_NXP4330_CPUFREQ)

static unsigned long dfs_freq_table[][2] = {
	{ 1200000, 1200 },
	{ 1100000, 1200 },
	{ 1000000, 1200 },
	{  900000, 1200 },
	{  800000, 1200 },
	{  780000, 1200 },
	{  562000, 1200 },
	{  533000, 1200 },
	{  490000, 1200 },
	{  400000, 1200 },
};

struct nxp_cpufreq_plat_data dfs_plat_data = {
	.pll_dev	   	= CONFIG_NXP4330_CPUFREQ_PLLDEV,
	.freq_table	   	= dfs_freq_table,
	.table_size	   	= ARRAY_SIZE(dfs_freq_table),
	.max_cpufreq    = 1200*1000,
	.max_retention  =   20*1000,
	.rest_cpufreq   =  400*1000,
	.rest_retention =    1*1000,
};

static struct platform_device dfs_plat_device = {
	.name			= DEV_NAME_CPUFREQ,
	.dev			= {
		.platform_data	= &dfs_plat_data,
	}
};static struct platform_device dfs_plat_device = {
	.name			= DEV_NAME_CPUFREQ,
	.dev			= {
		.platform_data	= &dfs_plat_data,
	}
};

#endif

/*------------------------------------------------------------------------------
 * DISPLAY (LVDS) / FB
 */
#if defined (CONFIG_FB_NEXELL)
#if defined (CONFIG_FB0_NEXELL)
static struct nxp_fb_plat_data fb0_plat_data = {
	.module			= CONFIG_FB0_NEXELL_DISPOUT,
	.layer			= CFG_DISP_PRI_SCREEN_LAYER,
	.format			= CFG_DISP_PRI_SCREEN_RGB_FORMAT,
	.bgcolor		= CFG_DISP_PRI_BACK_GROUND_COLOR,
	.bitperpixel	= CFG_DISP_PRI_SCREEN_PIXEL_BYTE * 8,
	.x_resol		= CFG_DISP_PRI_RESOL_WIDTH,
	.y_resol		= CFG_DISP_PRI_RESOL_HEIGHT,
	#ifdef CONFIG_ANDROID
	.buffers		= 3,
	.skip_pan_vsync	= 1,
	#else
	.buffers		= 2,
	#endif
	.lcd_with_mm	= CFG_DISP_PRI_LCD_WIDTH_MM,	/* 152.4 */
	.lcd_height_mm	= CFG_DISP_PRI_LCD_HEIGHT_MM,	/* 91.44 */
};

static struct platform_device fb0_device = {
	.name	= DEV_NAME_FB,
	.id		= 0,	/* FB device node num */
	.dev    = {
		.coherent_dma_mask 	= 0xffffffffUL,	/* for DMA allocate */
		.platform_data		= &fb0_plat_data
	},
};
#endif

static struct platform_device *fb_devices[] = {
	#if defined (CONFIG_FB0_NEXELL)
	&fb0_device,
	#endif
};
#endif /* CONFIG_FB_NEXELL */

/*------------------------------------------------------------------------------
 * NAND device
 */
#if defined(CONFIG_MTD_NAND_NEXELL)
#include <linux/mtd/partitions.h>
#include <asm-generic/sizes.h>

static struct mtd_partition nxp_nand_parts[] = {
	{
		.name		= "NAND  Boot",
		.offset		=   0 * SZ_1M,
		.size		=   8 * SZ_1M,
	}, {
		.name		= "NAND  Data",
		.offset		=   8 * SZ_1M,
		.size		=  56 * SZ_1M,
	},
};

static struct nxp_nand_plat_data nand_plat_data = {
	.parts		= nxp_nand_parts,
	.nr_parts	= ARRAY_SIZE(nxp_nand_parts),
	.chip_delay = 10,
};

static struct platform_device nand_plat_device = {
	.name	= DEV_NAME_NAND,
	.id		= -1,
	.dev	= {
		.platform_data	= &nand_plat_data,
	},
};
#endif	/* CONFIG_MTD_NAND_NEXELL */

/*------------------------------------------------------------------------------
 * Keypad platform device
 */
#if defined(CONFIG_KEYBOARD_NEXELL_KEY) || defined(CONFIG_KEYBOARD_NEXELL_KEY_MODULE)

#include <linux/input.h>

static unsigned int  button_gpio[] = CFG_KEYPAD_KEY_BUTTON;
static unsigned int  button_code[] = CFG_KEYPAD_KEY_CODE;

struct nxp_key_plat_data key_plat_data = {
	.bt_count	= ARRAY_SIZE(button_gpio),
	.bt_io		= button_gpio,
	.bt_code	= button_code,
	.bt_repeat	= CFG_KEYPAD_REPEAT,
};

static struct platform_device key_plat_device = {
	.name	= DEV_NAME_KEYPAD,
	.id		= -1,
	.dev    = {
		.platform_data	= &key_plat_data
	},
};
#endif	/* CONFIG_KEYBOARD_NEXELL_KEY || CONFIG_KEYBOARD_NEXELL_KEY_MODULE */

/*------------------------------------------------------------------------------
 * ASoC Codec platform device
 */
#if defined(CONFIG_SND_SPDIF_TRANSCIEVER) || defined(CONFIG_SND_SPDIF_TRANSCIEVER_MODULE)
static struct platform_device spdif_transciever = {
	.name	= "spdif-dit",
	.id		= -1,
};

struct nxp_snd_dai_plat_data spdif_trans_dai_data = {
	.sample_rate = 48000,
	.pcm_format	 = SNDRV_PCM_FMTBIT_S16_LE,
};

static struct platform_device spdif_trans_dai = {
	.name	= "spdif-transciever",
	.id		= -1,
	.dev	= {
		.platform_data	= &spdif_trans_dai_data,
	}
};
#endif

/*------------------------------------------------------------------------------
 *  * reserve mem
 *   */
#ifdef CONFIG_CMA
#include <linux/cma.h>
extern void nxp_cma_region_reserve(struct cma_region *, const char *);

void __init nxp_reserve_mem(void)
{
    static struct cma_region regions[] = {
        {
            .name = "ion",
#ifdef CONFIG_ION_NXP_CONTIGHEAP_SIZE
            .size = CONFIG_ION_NXP_CONTIGHEAP_SIZE * SZ_1K,
#endif
            {
                .alignment = PAGE_SIZE,
            }
        },
        {
            .size = 0
        }
    };

    static const char map[] __initconst =
        "ion-nxp=ion;"
        "nx_vpu=ion;";

    printk("%s: reserve CMA: size %d\n", __func__, CONFIG_ION_NXP_CONTIGHEAP_SIZE * SZ_1K);
    nxp_cma_region_reserve(regions, map);
}
#endif

/*------------------------------------------------------------------------------
 * PMIC platform device
 */
#if defined(CONFIG_REGULATOR_NXE2000)

#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/nxe2000.h>
#include <linux/io.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/nxe2000-regulator.h>
#include <linux/power/nxe2000_battery.h>
//#include <linux/rtc/rtc-nxe2000.h>
//#include <linux/rtc.h>

#define NXE2000_I2C_BUS		(0)
#define NXE2000_I2C_ADDR	(0x64 >> 1)
#define NXE2000_IRQ			(PAD_GPIO_A + 17)

#define PMC_CTRL			0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

/* NXE2000 IRQs */
#define NXE2000_IRQ_BASE	(IRQ_SYSTEM_END)
#define NXE2000_GPIO_BASE	(ARCH_NR_GPIOS) //PLATFORM_NXE2000_GPIO_BASE
#define NXE2000_GPIO_IRQ	(NXE2000_GPIO_BASE + 8)

//#define CONFIG_NXE2000_RTC


static struct regulator_consumer_supply nxe2000_dc1_supply_0[] = {
	REGULATOR_SUPPLY("vdd_arm_1.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_dc2_supply_0[] = {
	REGULATOR_SUPPLY("vdd_core_1.2V", NULL),
};
static struct regulator_consumer_supply nxe2000_dc3_supply_0[] = {
	REGULATOR_SUPPLY("vdd_sys_3.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_dc4_supply_0[] = {
	REGULATOR_SUPPLY("vdd_ddr_1.6V", NULL),
};
static struct regulator_consumer_supply nxe2000_dc5_supply_0[] = {
	REGULATOR_SUPPLY("vdd_sys_1.6V", NULL),
};

static struct regulator_consumer_supply nxe2000_ldo1_supply_0[] = {
	REGULATOR_SUPPLY("vgps_3.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo2_supply_0[] = {
	REGULATOR_SUPPLY("vcam1_1.8V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo3_supply_0[] = {
	REGULATOR_SUPPLY("vsys1_1.8V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo4_supply_0[] = {
	REGULATOR_SUPPLY("vsys_1.9V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo5_supply_0[] = {
	REGULATOR_SUPPLY("vcam_2.8V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo6_supply_0[] = {
	REGULATOR_SUPPLY("valive_3.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo7_supply_0[] = {
	REGULATOR_SUPPLY("vvid_2.8V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo8_supply_0[] = {
	REGULATOR_SUPPLY("vdumy0_3.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo9_supply_0[] = {
	REGULATOR_SUPPLY("vdumy1_3.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo10_supply_0[] = {
	REGULATOR_SUPPLY("vdumy2_1.2V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldortc1_supply_0[] = {
	REGULATOR_SUPPLY("valive_1.8V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldortc2_supply_0[] = {
	REGULATOR_SUPPLY("valive_1.0V", NULL),
};


#define NXE2000_PDATA_INIT(_name, _sname, _minuv, _maxuv, _always_on, _boot_on, \
	_init_uv, _init_enable, _slp_slots) \
	static struct nxe2000_regulator_platform_data pdata_##_name##_##_sname = \
	{	\
		.regulator = {	\
			.constraints = {	\
				.min_uV		= _minuv,	\
				.max_uV		= _maxuv,	\
				.valid_modes_mask	= (REGULATOR_MODE_NORMAL |	\
									REGULATOR_MODE_STANDBY),	\
				.valid_ops_mask		= (REGULATOR_CHANGE_MODE |	\
									REGULATOR_CHANGE_STATUS |	\
									REGULATOR_CHANGE_VOLTAGE),	\
				.always_on	= _always_on,	\
				.boot_on	= _boot_on,		\
				.apply_uV	= 1,			\
			},	\
			.num_consumer_supplies =		\
				ARRAY_SIZE(nxe2000_##_name##_supply_##_sname),	\
			.consumer_supplies	= nxe2000_##_name##_supply_##_sname, \
			.supply_regulator	= 0,	\
		},	\
		.init_uV		= _init_uv,		\
		.init_enable	= _init_enable,	\
		.sleep_slots	= _slp_slots,	\
	}
/* min_uV/max_uV : Please set the appropriate value for the devices that the power supplied within a*/
/*                 range from min to max voltage according to NXE2000 specification. */
NXE2000_PDATA_INIT(dc1,      0, 1000000, 2000000, 1, 1, 1300000, 1, 12);		/* 1.3V ARM */
NXE2000_PDATA_INIT(dc2,      0, 1000000, 2000000, 1, 1, 1200000, 1, 14);		/* 1.2V CORE */
NXE2000_PDATA_INIT(dc3,      0, 1000000, 3500000, 1, 1, 3300000, 1,  2);		/* 3.3V SYS */
NXE2000_PDATA_INIT(dc4,      0, 1000000, 2000000, 1, 1, 1600000, 1, -1);		/* 1.6V DDR */
NXE2000_PDATA_INIT(dc5,      0, 1000000, 2000000, 1, 1, 1600000, 1,  8);		/* 1.6V SYS */

NXE2000_PDATA_INIT(ldo1,     0, 1000000, 3500000, 1, 0, 3300000, 1,  2);		/* 3.3V GPS */
NXE2000_PDATA_INIT(ldo2,     0, 1000000, 3500000, 0, 0, 1800000, 0,  2);		/* 1.8V CAM1 */
NXE2000_PDATA_INIT(ldo3,     0, 1000000, 3500000, 1, 0, 1800000, 1,  6);		/* 1.8V SYS1 */
NXE2000_PDATA_INIT(ldo4,     0, 1000000, 3500000, 1, 0, 1900000, 1,  6);		/* 1.9V SYS */
NXE2000_PDATA_INIT(ldo5,     0, 1000000, 3500000, 0, 0, 2800000, 0,  1);		/* 2.8V VCAM */
NXE2000_PDATA_INIT(ldo6,     0, 1000000, 3500000, 1, 0, 3300000, 1, -1);		/* 3.3V ALIVE */
NXE2000_PDATA_INIT(ldo7,     0, 1000000, 3500000, 1, 0, 2800000, 1,  4);		/* 2.8V VID */
NXE2000_PDATA_INIT(ldo8,     0, 1000000, 3500000, 0, 0,      -1, 0,  0);		/* Not Use */
NXE2000_PDATA_INIT(ldo9,     0, 1000000, 3500000, 0, 0,      -1, 0,  0);		/* Not Use */
NXE2000_PDATA_INIT(ldo10,    0, 1000000, 3500000, 0, 0,      -1, 0,  0);		/* Not Use */
NXE2000_PDATA_INIT(ldortc1,  0, 1700000, 3500000, 1, 0, 1800000, 1, -1);		/* 1.8V ALIVE */
NXE2000_PDATA_INIT(ldortc2,  0, 1000000, 3500000, 1, 0, 1000000, 1, -1);		/* 1.0V ALIVE */


/*-------- if nxe2000 RTC exists -----------*/
#ifdef CONFIG_NXE2000_RTC
static struct nxe2000_rtc_platform_data rtc_data = {
	.irq	= NXE2000_IRQ_BASE,
	.time	= {
		.tm_year	= 1970,
		.tm_mon		= 0,
		.tm_mday	= 1,
		.tm_hour	= 0,
		.tm_min		= 0,
		.tm_sec		= 0,
	},
};

#define NXE2000_RTC_REG	\
{	\
	.id		= 0,	\
	.name	= "rtc_nxe2000",	\
	.platform_data	= &rtc_data,	\
}
#endif
/*-------- if Nexell RTC exists -----------*/

#define NXE2000_REG(_id, _name, _sname)	\
{	\
	.id		= NXE2000_ID_##_id,	\
	.name	= "nxe2000-regulator",	\
	.platform_data	= &pdata_##_name##_##_sname,	\
}

#define NXE2000_BATTERY_REG	\
{	\
    .id		= -1,	\
    .name	= "nxe2000-battery",	\
    .platform_data	= &nxe2000_battery_data,	\
}

//==========================================
//NXE2000 Power_Key device data
//==========================================
static struct nxe2000_pwrkey_platform_data nxe2000_pwrkey_data = {
	.irq 		= NXE2000_IRQ_BASE,
	.delay_ms 	= 20,
};
#define NXE2000_PWRKEY_REG		\
{	\
	.id 	= -1,	\
	.name 	= "nxe2000-pwrkey",	\
	.platform_data 	= &nxe2000_pwrkey_data,	\
}


static struct nxe2000_battery_platform_data nxe2000_battery_data = {
	.irq 				= NXE2000_IRQ_BASE,

	.input_power_type   = INPUT_POWER_TYPE_ADP,

	.gpio_otg_usbid		= CFG_GPIO_OTG_USBID_DET,
	.gpio_otg_vbus		= CFG_GPIO_OTG_VBUS_DET,
	.gpio_pmic_vbus		= CFG_GPIO_PMIC_VUSB_DET,
	.gpio_pmic_lowbat	= CFG_GPIO_PMIC_LOWBAT_DET,

	.alarm_vol_mv 		= 3412,
//	.adc_channel 		= NXE2000_ADC_CHANNEL_VBAT,
	.multiple			= 100, //100%
	.monitor_time		= 60,
		/* some parameter is depend of battery type */
	.type[0] = {
		.ch_vfchg		= 0x03,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.ch_vrchg		= 0x03,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset	= 0xFF,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 		= 0x07,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x18,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x04,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg		= 0x03,	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys	= 3000,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat	= 1000, /* This value is the target one to DSOC=0% */
		.fg_poff_vbat	= 0,	/* setting value of 0 per Vbat */
		.jt_en			= 0,	/* JEITA Enable	  = 0 or 1 (1:enable, 0:disable) */
		.jt_hw_sw		= 1,	/* JEITA HW or SW = 0 or 1 (1:HardWare, 0:SoftWare) */
		.jt_temp_h		= 50,	/* degree C */
		.jt_temp_l		= 12,	/* degree C */
		.jt_vfchg_h 	= 0x03,	/* VFCHG High  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_vfchg_l 	= 0,	/* VFCHG Low  	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
		.jt_ichg_h		= 0x07,	/* ICHG High  	= 0 - 0x1D (100mA - 3000mA) */
		.jt_ichg_l		= 0x04,	/* ICHG Low   	= 0 - 0x1D (100mA - 3000mA) */
	},
	/*
	.type[1] = {
		.ch_vfchg		= 0x0,
		.ch_vrchg		= 0x0,
		.ch_vbatovset	= 0x0,
		.ch_ichg			= 0x0,
		.ch_ilim_adp		= 0x0,
		.ch_ilim_usb		= 0x0,
		.ch_icchg		= 0x00,
		.fg_target_vsys	= 3300,//3000,
		.fg_target_ibat	= 1000,//1000,
		.jt_en			= 0,
		.jt_hw_sw		= 1,
		.jt_temp_h		= 40,
		.jt_temp_l		= 10,
		.jt_vfchg_h		= 0x0,
		.jt_vfchg_l		= 0,
		.jt_ichg_h		= 0x01,
		.jt_ichg_l		= 0x01,
	},
	*/

/*  JEITA Parameter
*
*          VCHG
*            |
* jt_vfchg_h~+~~~~~~~~~~~~~~~~~~~+
*            |                   |
* jt_vfchg_l-| - - - - - - - - - +~~~~~~~~~~+
*            |    Charge area    +          |
*  -------0--+-------------------+----------+--- Temp
*            !                   +
*          ICHG
*            |                   +
*  jt_ichg_h-+ - -+~~~~~~~~~~~~~~+~~~~~~~~~~+
*            +    |              +          |
*  jt_ichg_l-+~~~~+   Charge area           |
*            |    +              +          |
*         0--+----+--------------+----------+--- Temp
*            0   jt_temp_l      jt_temp_h   55
*/
};



#define NXE2000_DEV_REG 		\
	NXE2000_REG(DC1, dc1, 0),		\
	NXE2000_REG(DC2, dc2, 0),		\
	NXE2000_REG(DC3, dc3, 0),		\
	NXE2000_REG(DC4, dc4, 0),		\
	NXE2000_REG(DC5, dc5, 0),		\
	NXE2000_REG(LDO1, ldo1, 0),	\
	NXE2000_REG(LDO2, ldo2, 0),	\
	NXE2000_REG(LDO3, ldo3, 0),	\
	NXE2000_REG(LDO4, ldo4, 0),	\
	NXE2000_REG(LDO5, ldo5, 0),	\
	NXE2000_REG(LDO6, ldo6, 0),	\
	NXE2000_REG(LDO7, ldo7, 0),	\
	NXE2000_REG(LDO8, ldo8, 0),	\
	NXE2000_REG(LDO9, ldo9, 0),	\
	NXE2000_REG(LDO10, ldo10, 0),	\
	NXE2000_REG(LDORTC1, ldortc1, 0),	\
	NXE2000_REG(LDORTC2, ldortc2, 0)

static struct nxe2000_subdev_info nxe2000_devs_dcdc[] = {
	NXE2000_DEV_REG,
	NXE2000_BATTERY_REG,
	NXE2000_PWRKEY_REG,
#ifdef CONFIG_NXE2000_RTC
	NXE2000_RTC_REG,
#endif
};


#define NXE2000_GPIO_INIT(_init_apply, _output_mode, _output_val, _led_mode, _led_func) \
	{									\
		.output_mode_en = _output_mode,	\
		.output_val		= _output_val,	\
		.init_apply		= _init_apply,	\
		.led_mode		= _led_mode,	\
		.led_func		= _led_func,	\
	}
struct nxe2000_gpio_init_data nxe2000_gpio_data[] = {
	NXE2000_GPIO_INIT(false, false, 0, 0, 0),
	NXE2000_GPIO_INIT(false, false, 0, 0, 0),
	NXE2000_GPIO_INIT(false, false, 0, 0, 0),
	NXE2000_GPIO_INIT(false, false, 0, 0, 0),
	NXE2000_GPIO_INIT(false, false, 0, 0, 0),
};

static struct nxe2000_platform_data nxe2000_platform = {
	.num_subdevs		= ARRAY_SIZE(nxe2000_devs_dcdc),
	.subdevs			= nxe2000_devs_dcdc,
	.irq_base			= NXE2000_IRQ_BASE,
	.irq_type			= IRQ_TYPE_EDGE_FALLING,
	.gpio_base			= NXE2000_GPIO_BASE,
	.gpio_init_data		= nxe2000_gpio_data,
	.num_gpioinit_data	= ARRAY_SIZE(nxe2000_gpio_data),
	.enable_shutdown_pin	= true,
};

static struct i2c_board_info __initdata nxe2000_regulators[] = {
	{
		I2C_BOARD_INFO("nxe2000", NXE2000_I2C_ADDR),
		.irq		= NXE2000_IRQ,
		.platform_data	= &nxe2000_platform,
	},
};
#endif  /* CONFIG_REGULATOR_NXE2000 */

/*------------------------------------------------------------------------------
 * MPEGTS platform device
 */
#if defined(CONFIG_NXP4330_MP2TS_IF)
#include <mach/nxp4330_mp2ts.h>

#define NXP_TS_PAGE_NUM_0       (36)	// Variable
#define NXP_TS_BUF_SIZE_0       (TS_PAGE_SIZE * NXP_TS_PAGE_NUM_0)

#define NXP_TS_PAGE_NUM_1       (36)	// Variable
#define NXP_TS_BUF_SIZE_1       (TS_PAGE_SIZE * NXP_TS_PAGE_NUM_1)

#define NXP_TS_PAGE_NUM_CORE    (36)	// Variable
#define NXP_TS_BUF_SIZE_CORE    (TS_PAGE_SIZE * NXP_TS_PAGE_NUM_CORE)


static struct nxp_mp2ts_dev_info mp2ts_dev_info[2] = {
    {
        .demod_irq_num = CFG_GPIO_DEMOD_0_IRQ_NUM,
        .demod_rst_num = CFG_GPIO_DEMOD_0_RST_NUM,
        .tuner_rst_num = CFG_GPIO_TUNER_0_RST_NUM,
    },
    {
        .demod_irq_num = CFG_GPIO_DEMOD_1_IRQ_NUM,
        .demod_rst_num = CFG_GPIO_DEMOD_1_RST_NUM,
        .tuner_rst_num = CFG_GPIO_TUNER_1_RST_NUM,
    },
};

static struct nxp_mp2ts_plat_data mpegts_plat_data = {
    .dev_info       = mp2ts_dev_info,
    .ts_dma_size[0] = -1,                   // TS ch 0 - Static alloc size.
    .ts_dma_size[1] = NXP_TS_BUF_SIZE_1,    // TS ch 1 - Static alloc size.
    .ts_dma_size[2] = -1,                   // TS core - Static alloc size.
};

static struct platform_device mpegts_plat_device = {
    .name	= DEV_NAME_MPEGTSI,
    .id		= 0,
    .dev	= {
        .platform_data = &mpegts_plat_data,
    },
};
#endif  /* CONFIG_NXP4330_MP2TS_IF */

/*------------------------------------------------------------------------------
 * v4l2 platform device
 */
#if defined(CONFIG_V4L2_NEXELL) || defined(CONFIG_V4L2_NEXELL_MODULE)
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/nxp-v4l2-platformdata.h>
#include <mach/soc.h>

static int camera_power_enable(bool on)
{
#ifdef CFG_IO_CAMERA_POWER_DOWN
    static bool is_first = true;
    if (on && is_first) {
        /* reset signal */
        nxp_soc_gpio_set_out_value(CFG_IO_CAMERA_POWER_DOWN, 0);
        nxp_soc_gpio_set_io_dir(CFG_IO_CAMERA_POWER_DOWN, 1);
        nxp_soc_gpio_set_io_func(CFG_IO_CAMERA_POWER_DOWN,
                nxp_soc_gpio_get_altnum(CFG_IO_CAMERA_POWER_DOWN));
        mdelay(1);
        nxp_soc_gpio_set_out_value(CFG_IO_CAMERA_POWER_DOWN, 1);
        is_first = false;
    } else {
        /* nxp_soc_gpio_set_out_value(CFG_IO_CAMERA_POWER_DOWN, on); */
    }
#endif
    return 0;
}

static int camera_set_clock(ulong clk_rate)
{
    if (clk_rate > 0)
        nxp_soc_pwm_set_frequency(1, clk_rate, 50);
    else
        nxp_soc_pwm_set_frequency(1, 0, 0);
    msleep(1);
    return 0;
}

static void vin_setup_io(int module)
{
    u_int *pad;
    int i, len;
    u_int io, fn;

    /* VIP0:0 = VCLK, VID0 ~ 7 */
    const u_int port[][2] = {
        /* VCLK, HSYNC, VSYNC */
        { PAD_GPIO_C + 14, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 15, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 16, NX_GPIO_PADFUNC_3 },
        /* DATA */
        { PAD_GPIO_C + 17, NX_GPIO_PADFUNC_3 }, { PAD_GPIO_C + 18, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 19, NX_GPIO_PADFUNC_3 }, { PAD_GPIO_C + 20, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 21, NX_GPIO_PADFUNC_3 }, { PAD_GPIO_C + 22, NX_GPIO_PADFUNC_3 },
        { PAD_GPIO_C + 23, NX_GPIO_PADFUNC_3 }, { PAD_GPIO_C + 24, NX_GPIO_PADFUNC_3 },
    };

    pad = (u_int *)port;
    len = sizeof(port)/sizeof(port[0]);

    for (i = 0; i < len; i++) {
        io = *pad++;
        fn = *pad++;
        nxp_soc_gpio_set_io_dir(io, 0);
        nxp_soc_gpio_set_io_func(io, fn);
    }
}


static struct i2c_board_info s5k5cagx_i2c_boardinfo[] = {
    {
        I2C_BOARD_INFO("S5K5CAGX", 0x78>>1),
    },
};

/* for mipi camera: s5k4ecgx */
static int s5k4ecgx_power_enable(bool on)
{
#if defined(CFG_IO_MIPI_CAMERA_POWER_ENABLE) && defined(CFG_IO_MIPI_CAMERA_RESETN)
    static bool is_mipi_first = true;
    if (on && is_mipi_first) {
        /* power enable & clk generation */
        nxp_soc_gpio_set_out_value(CFG_IO_MIPI_CAMERA_POWER_ENABLE, 1);
        nxp_soc_gpio_set_io_dir(CFG_IO_MIPI_CAMERA_POWER_ENABLE, 1);
        nxp_soc_gpio_set_io_func(CFG_IO_MIPI_CAMERA_POWER_ENABLE,
                nxp_soc_gpio_get_altnum(CFG_IO_MIPI_CAMERA_POWER_ENABLE));
        /* reset */
        nxp_soc_gpio_set_out_value(CFG_IO_MIPI_CAMERA_RESETN, 0);
        nxp_soc_gpio_set_io_dir(CFG_IO_MIPI_CAMERA_RESETN, 1);
        nxp_soc_gpio_set_io_func(CFG_IO_MIPI_CAMERA_RESETN,
                nxp_soc_gpio_get_altnum(CFG_IO_MIPI_CAMERA_RESETN));
        mdelay(1);
        nxp_soc_gpio_set_out_value(CFG_IO_MIPI_CAMERA_RESETN, 1);
        is_mipi_first = false;
    } else {
        /* nxp_soc_gpio_set_out_value(CFG_IO_MIPI_CAMERA_POWER_ENABLE, on); */
    }
#endif
    return 0;
}

#if defined(CONFIG_NXP_CAPTURE_MIPI_CSI)
static int s5k4ecgx_set_clock(ulong clk_rate)
{
    /* do nothing */
    return 0;
}

static void mipi_vin_setup_io(int module)
{
    /* do nothing */
}

static int mipi_phy_enable(bool en)
{
    return 0;
}

static struct i2c_board_info s5k4ecgx_i2c_boardinfo[] = {
    {
        I2C_BOARD_INFO("S5K4ECGX", 0x5A>>1),
    },
};

struct nxp_mipi_csi_platformdata mipi_plat_data = {
    .module     = 0,
    .clk_rate   = 27000000, // 27MHz
    .lanes      = 4,
    .alignment = 0,
    .hs_settle  = 0,
    .width      = 640,
    .height     = 480,
    .fixed_phy_vdd = false,
    .irq        = 0, /* not used */
    .base       = 0, /* not used */
    .phy_enable = mipi_phy_enable
};
#endif

static struct nxp_v4l2_i2c_board_info sensor[] = {
    {
        .board_info = &s5k5cagx_i2c_boardinfo[0],
        .i2c_adapter_id = 0,
    },
#if defined(CONFIG_NXP_CAPTURE_MIPI_CSI)
    {
        .board_info = &s5k4ecgx_i2c_boardinfo[0],
        .i2c_adapter_id = 0,
    },
#endif
};

static struct nxp_capture_platformdata capture_plat_data[] = {
    {
        .sensor = &sensor[0],
        .type = NXP_CAPTURE_INF_PARALLEL,
        .parallel = {
            /* for 656 */
            .is_mipi        = false,
            .external_sync  = false,
            .h_active       = 640,
            .h_frontporch   = 7,
            .h_syncwidth    = 1,
            /* .h_backporch    = 0, */
            .h_backporch    = 10,
            .v_active       = 480,
            .v_frontporch   = 0,
            .v_syncwidth    = 1,
            .v_backporch    = 0,
            .clock_invert   = true,
            .port           = 1,
            .data_order     = NXP_VIN_CBY0CRY1,
            .interlace      = false,
            .clk_rate       = 24000000,
            .power_enable   = camera_power_enable,
            .set_clock      = camera_set_clock,
            .setup_io       = vin_setup_io,
        },
    },
#if defined(CONFIG_NXP_CAPTURE_MIPI_CSI)
    {
        .sensor = &sensor[1],
        .type = NXP_CAPTURE_INF_CSI,
        .parallel = {
            /* for mipi */
            .is_mipi        = true,
            .external_sync  = true,
            .h_active       = 640,
            .h_frontporch   = 100,
            .h_syncwidth    = 10,
            .h_backporch    = 100,
            .v_active       = 480,
            .v_frontporch   = 1,
            .v_syncwidth    = 1,
            .v_backporch    = 1,
            .clock_invert   = false,
            .port           = NX_VIP_INPUTPORT_B,
            .data_order     = NXP_VIN_CBY0CRY1,
            .interlace      = false,
            .clk_rate       = 27000000,
            .power_enable   = s5k4ecgx_power_enable,
            .set_clock      = s5k4ecgx_set_clock,
            .setup_io       = mipi_vin_setup_io,
        },
        .csi = &mipi_plat_data,
    },
#endif
    { NULL, 0, },
};

/* out platformdata */
static struct i2c_board_info hdmi_edid_i2c_boardinfo = {
    I2C_BOARD_INFO("nxp_edid", 0xA0>>1),
};

static struct nxp_v4l2_i2c_board_info edid = {
    .board_info = &hdmi_edid_i2c_boardinfo,
    .i2c_adapter_id = 0,
};

static struct i2c_board_info hdmi_hdcp_i2c_boardinfo = {
    I2C_BOARD_INFO("nxp_hdcp", 0x74>>1),
};

static struct nxp_v4l2_i2c_board_info hdcp = {
    .board_info = &hdmi_hdcp_i2c_boardinfo,
    .i2c_adapter_id = 0,
};


static void hdmi_set_int_external(int gpio)
{
    nxp_soc_gpio_set_int_enable(gpio, 0);
    nxp_soc_gpio_set_int_mode(gpio, 1); /* high level */
    nxp_soc_gpio_set_int_enable(gpio, 1);
    nxp_soc_gpio_clr_int_pend(gpio);
}

static void hdmi_set_int_internal(int gpio)
{
    nxp_soc_gpio_set_int_enable(gpio, 0);
    nxp_soc_gpio_set_int_mode(gpio, 0); /* low level */
    nxp_soc_gpio_set_int_enable(gpio, 1);
    nxp_soc_gpio_clr_int_pend(gpio);
}

static int hdmi_read_hpd_gpio(int gpio)
{
    return nxp_soc_gpio_get_in_value(gpio);
}

static struct nxp_out_platformdata out_plat_data = {
    .hdmi = {
        .internal_irq = 0,
        .external_irq = PAD_GPIO_A + 19,
        .set_int_external = hdmi_set_int_external,
        .set_int_internal = hdmi_set_int_internal,
        .read_hpd_gpio = hdmi_read_hpd_gpio,
        .edid = &edid,
        .hdcp = &hdcp,
    },
};

static struct nxp_v4l2_platformdata v4l2_plat_data = {
    .captures = &capture_plat_data[0],
    .out = &out_plat_data,
};

static struct platform_device nxp_v4l2_dev = {
    .name       = NXP_V4L2_DEV_NAME,
    .id         = 0,
    .dev        = {
        .platform_data = &v4l2_plat_data,
    },
};
#endif /* CONFIG_V4L2_NEXELL || CONFIG_V4L2_NEXELL_MODULE */


/*------------------------------------------------------------------------------
 * DW MMC board config
 */
#if defined(CONFIG_MMC_DW)
int _dwmci_ext_cd_init(void (*notify_func)(struct platform_device *, int state))
{
	return 0;
}

int _dwmci_ext_cd_cleanup(void (*notify_func)(struct platform_device *, int state))
{
	return 0;
}

#ifdef CONFIG_MMC_NEXELL_CH0
static int _dwmci0_init(u32 slot_id, irq_handler_t handler, void *data)
{
	struct dw_mci *host = (struct dw_mci *)data;
	int io  = CFG_SDMMC0_DETECT_IO;
	int irq = IRQ_GPIO_START + io;
	int id  = 0, ret = 0;

	printk("dw_mmc dw_mmc.%d: Using external card detect irq %3d (io %2d)\n", id, irq, io);

	ret  = request_irq(irq, handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				DEV_NAME_SDHC "0", (void*)host->slot[slot_id]);
	if (0 > ret)
		pr_err("dw_mmc dw_mmc.%d: fail request interrupt %d ...\n", id, irq);
	return 0;
}

static int _dwmci0_get_cd(u32 slot_id)
{
	int io = CFG_SDMMC0_DETECT_IO;
	return nxp_soc_gpio_get_in_value(io);
}

static struct dw_mci_board _dwmci0_data = {
	.quirks			= DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_CMD23,
	.detect_delay_ms= 200,
	.cd_type		= DW_MCI_CD_EXTERNAL,
	.init			= _dwmci0_init,
	.get_cd			= _dwmci0_get_cd,
	.ext_cd_init	= _dwmci_ext_cd_init,
	.ext_cd_cleanup	= _dwmci_ext_cd_cleanup,
};
#endif

#ifdef CONFIG_MMC_NEXELL_CH1
static struct dw_mci_board _dwmci1_data = {
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION |
				  	  DW_MCI_QUIRK_HIGHSPEED |
				  	  DW_MMC_QUIRK_HW_RESET_PW |
				      DW_MCI_QUIRK_NO_DETECT_EBIT,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 |
					  MMC_CAP_NONREMOVABLE |
			 	  	  MMC_CAP_4_BIT_DATA | MMC_CAP_CMD23 |
				  	  MMC_CAP_ERASE | MMC_CAP_HW_RESET,
	.desc_sz		= 4,
	.detect_delay_ms= 200,
	.sdr_timing		= 0x03020001,
	.ddr_timing		= 0x03030002,
};
#endif

#endif /* CONFIG_MMC_DW */

/*------------------------------------------------------------------------------
 * DW GMAC board config
 */
#if defined(CONFIG_NXPMAC_ETH)
#include <linux/phy.h>
#include <linux/nxpmac.h>

int  nxpmac_init(struct platform_device *pdev)
{
	u32 addr;

	/* Select PULL up/down. */
#if 0
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+ 7, CFALSE );              // PAD_GPIOE7,     GMAC0_PHY_TXD[0]
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+ 8, CFALSE );              // PAD_GPIOE8,     GMAC0_PHY_TXD[1]
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+ 9, CFALSE );              // PAD_GPIOE9,     GMAC0_PHY_TXD[2]
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+10, CFALSE );              // PAD_GPIOE10,    GMAC0_PHY_TXD[3]
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+11, CFALSE );              // PAD_GPIOE11,    GMAC0_PHY_TXEN
//	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+12, CFALSE );              // PAD_GPIOE12,    GMAC0_PHY_TXER
//	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+13, CFALSE );              // PAD_GPIOE13,    GMAC0_PHY_COL
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+14, CFALSE );              // PAD_GPIOE14,    GMAC0_PHY_RXD[0]
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+15, CFALSE );              // PAD_GPIOE15,    GMAC0_PHY_RXD[1]
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+16, CFALSE );              // PAD_GPIOE16,    GMAC0_PHY_RXD[2]
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+17, CFALSE );              // PAD_GPIOE17,    GMAC0_PHY_RXD[3]
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+18, CFALSE );              // PAD_GPIOE18,    GMAC0_CLK_RX
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+19, CFALSE );              // PAD_GPIOE19,    GMAC0_PHY_RX_DV
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+20, CFALSE );              // PAD_GPIOE20,    GMAC0_GMII_MDC
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+21, CFALSE );              // PAD_GPIOE21,    GMAC0_GMII_MDI
//	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+22, CFALSE );              // PAD_GPIOE22,    GMAC0_PHY_RXER
//	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+23, CFALSE );              // PAD_GPIOE23,    GMAC0_PHY_CRS
	nxp_soc_gpio_set_io_pull_sel( PAD_GPIO_E+24, CFALSE );              // PAD_GPIOE24,    GMAC0_GTX_CLK
#endif

	/* Set PULL enable */
#if 0
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+ 7, CFALSE );              // PAD_GPIOE7,     GMAC0_PHY_TXD[0]
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+ 8, CFALSE );              // PAD_GPIOE8,     GMAC0_PHY_TXD[1]
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+ 9, CFALSE );              // PAD_GPIOE9,     GMAC0_PHY_TXD[2]
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+10, CFALSE );              // PAD_GPIOE10,    GMAC0_PHY_TXD[3]
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+11, CFALSE );              // PAD_GPIOE11,    GMAC0_PHY_TXEN
//	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+12, CFALSE );              // PAD_GPIOE12,    GMAC0_PHY_TXER
//	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+13, CFALSE );              // PAD_GPIOE13,    GMAC0_PHY_COL
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+14, CFALSE );              // PAD_GPIOE14,    GMAC0_PHY_RXD[0]
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+15, CFALSE );              // PAD_GPIOE15,    GMAC0_PHY_RXD[1]
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+16, CFALSE );              // PAD_GPIOE16,    GMAC0_PHY_RXD[2]
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+17, CFALSE );              // PAD_GPIOE17,    GMAC0_PHY_RXD[3]
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+18, CFALSE );              // PAD_GPIOE18,    GMAC0_CLK_RX
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+19, CFALSE );              // PAD_GPIOE19,    GMAC0_PHY_RX_DV
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+20, CFALSE );              // PAD_GPIOE20,    GMAC0_GMII_MDC
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+21, CFALSE );              // PAD_GPIOE21,    GMAC0_GMII_MDI
//	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+22, CFALSE );              // PAD_GPIOE22,    GMAC0_PHY_RXER
//	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+23, CFALSE );              // PAD_GPIOE23,    GMAC0_PHY_CRS
	nxp_soc_gpio_set_io_pull_enb( PAD_GPIO_E+24, CFALSE );              // PAD_GPIOE24,    GMAC0_GTX_CLK
#endif

	/* Set altnate function */
#if 0
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+ 7, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE7,     GMAC0_PHY_TXD[0]
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+ 8, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE8,     GMAC0_PHY_TXD[1]
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+ 9, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE9,     GMAC0_PHY_TXD[2]
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+10, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE10,    GMAC0_PHY_TXD[3]
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+11, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE11,    GMAC0_PHY_TXEN
//	nxp_soc_gpio_set_io_func( PAD_GPIO_E+12, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE12,    GMAC0_PHY_TXER
//	nxp_soc_gpio_set_io_func( PAD_GPIO_E+13, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE13,    GMAC0_PHY_COL
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+14, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE14,    GMAC0_PHY_RXD[0]
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+15, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE15,    GMAC0_PHY_RXD[1]
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+16, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE16,    GMAC0_PHY_RXD[2]
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+17, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE17,    GMAC0_PHY_RXD[3]
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+18, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE18,    GMAC0_CLK_RX
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+19, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE19,    GMAC0_PHY_RX_DV
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+20, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE20,    GMAC0_GMII_MDC
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+21, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE21,    GMAC0_GMII_MDI
//	nxp_soc_gpio_set_io_func( PAD_GPIO_E+22, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE22,    GMAC0_PHY_RXER
//	nxp_soc_gpio_set_io_func( PAD_GPIO_E+23, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE23,    GMAC0_PHY_CRS
	nxp_soc_gpio_set_io_func( PAD_GPIO_E+24, NX_GPIO_PADFUNC_1 );       // PAD_GPIOE24,    GMAC0_GTX_CLK
#endif

	/* Set drive strength */
#if 1
	// 1000Base-T
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+ 7, 3 );        // PAD_GPIOE7,     GMAC0_PHY_TXD[0]
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+ 8, 3 );        // PAD_GPIOE8,     GMAC0_PHY_TXD[1]
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+ 9, 3 );        // PAD_GPIOE9,     GMAC0_PHY_TXD[2]
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+10, 3 );        // PAD_GPIOE10,    GMAC0_PHY_TXD[3]
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+11, 3 );        // PAD_GPIOE11,    GMAC0_PHY_TXEN
//	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+12, 3 );        // PAD_GPIOE12,    GMAC0_PHY_TXER
//	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+13, 3 );        // PAD_GPIOE13,    GMAC0_PHY_COL
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+14, 3 );        // PAD_GPIOE14,    GMAC0_PHY_RXD[0]
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+15, 3 );        // PAD_GPIOE15,    GMAC0_PHY_RXD[1]
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+16, 3 );        // PAD_GPIOE16,    GMAC0_PHY_RXD[2]
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+17, 3 );        // PAD_GPIOE17,    GMAC0_PHY_RXD[3]
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+18, 3 );        // PAD_GPIOE18,    GMAC0_RX_CLK
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+19, 3 );        // PAD_GPIOE19,    GMAC0_PHY_RX_DV
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+20, 3 );        // PAD_GPIOE20,    GMAC0_GMII_MDC
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+21, 3 );        // PAD_GPIOE21,    GMAC0_GMII_MDI
//	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+22, 3 );        // PAD_GPIOE22,    GMAC0_PHY_RXER
//	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+23, 3 );        // PAD_GPIOE23,    GMAC0_PHY_CRS
	nxp_soc_gpio_set_io_drv( PAD_GPIO_E+24, 3 );        // PAD_GPIOE24,    GMAC0_GTX_CLK

//	NX_CLKGEN_SetClockOutInv( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, CFALSE);    // TX Clk invert off
#endif
#if 0   //20140515
    // 100 & 10Base-T
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+ 7, 1 );        // PAD_GPIOE7,     GMAC0_PHY_TXD[0]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+ 8, 1 );        // PAD_GPIOE8,     GMAC0_PHY_TXD[1]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+ 9, 1 );        // PAD_GPIOE9,     GMAC0_PHY_TXD[2]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+10, 1 );        // PAD_GPIOE10,    GMAC0_PHY_TXD[3]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+11, 1 );        // PAD_GPIOE11,    GMAC0_PHY_TXEN
//  nxp_soc_gpio_set_io_drv( PAD_GPIO_E+12, 3 );        // PAD_GPIOE12,    GMAC0_PHY_TXER
//  nxp_soc_gpio_set_io_drv( PAD_GPIO_E+13, 3 );        // PAD_GPIOE13,    GMAC0_PHY_COL
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+14, 2 );        // PAD_GPIOE14,    GMAC0_PHY_RXD[0]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+15, 2 );        // PAD_GPIOE15,    GMAC0_PHY_RXD[1]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+16, 2 );        // PAD_GPIOE16,    GMAC0_PHY_RXD[2]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+17, 2 );        // PAD_GPIOE17,    GMAC0_PHY_RXD[3]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+18, 2 );        // PAD_GPIOE18,    GMAC0_RX_CLK
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+19, 2);        // PAD_GPIOE19,    GMAC0_PHY_RX_DV
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+20, 2 );        // PAD_GPIOE20,    GMAC0_GMII_MDC
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+21, 2 );        // PAD_GPIOE21,    GMAC0_GMII_MDI
//  nxp_soc_gpio_set_io_drv( PAD_GPIO_E+22, 3 );        // PAD_GPIOE22,    GMAC0_PHY_RXER
//  nxp_soc_gpio_set_io_drv( PAD_GPIO_E+23, 3 );        // PAD_GPIOE23,    GMAC0_PHY_CRS
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+24, 2 );        // PAD_GPIOE24,    GMAC0_GTX_CLK
#endif

	// Clock control
	NX_CLKGEN_Initialize();
	addr = NX_CLKGEN_GetPhysicalAddress(CLOCKINDEX_OF_DWC_GMAC_MODULE);
	NX_CLKGEN_SetBaseAddress( CLOCKINDEX_OF_DWC_GMAC_MODULE, (u32)IO_ADDRESS(addr) );
	NX_CLKGEN_SetClockSource( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, 4);     // Sync mode for 100 & 10Base-T : External RX_clk
	NX_CLKGEN_SetClockDivisor( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, 1);    // Sync mode for 100 & 10Base-T
	NX_CLKGEN_SetClockOutInv( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, CFALSE);    // TX Clk invert off
//	NX_CLKGEN_SetClockOutInv( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, CTRUE);     // TX clk invert on : 100 & 10Base-T, Strength 0

	NX_CLKGEN_SetClockDivisorEnable( CLOCKINDEX_OF_DWC_GMAC_MODULE, CTRUE);

	// Reset control
	NX_RSTCON_Initialize();
	addr = NX_RSTCON_GetPhysicalAddress();
	NX_RSTCON_SetBaseAddress( (u32)IO_ADDRESS(addr) );
	NX_RSTCON_SetnRST(RESETINDEX_OF_DWC_GMAC_MODULE_aresetn_i, RSTCON_ENABLE);
	udelay(100);
	NX_RSTCON_SetnRST(RESETINDEX_OF_DWC_GMAC_MODULE_aresetn_i, RSTCON_DISABLE);
	udelay(100);
	NX_RSTCON_SetnRST(RESETINDEX_OF_DWC_GMAC_MODULE_aresetn_i, RSTCON_ENABLE);
	udelay(100);

	return 0;
}

int gmac_phy_reset(void *priv)
{
	// Set GPIO nReset
	gpio_set_value(CFG_ETHER_GMAC_PHY_RST_NUM, 1 );
	udelay( 100 );
	gpio_set_value(CFG_ETHER_GMAC_PHY_RST_NUM, 0 );
	udelay( 100 );
	gpio_set_value(CFG_ETHER_GMAC_PHY_RST_NUM, 1 );
	msleep( 30 );

	return 0;
}

static struct stmmac_mdio_bus_data nxpmac0_mdio_bus = {
	.phy_reset = gmac_phy_reset,
	.phy_mask = 0,
	.probed_phy_irq = CFG_ETHER_GMAC_PHY_IRQ_NUM,
};

static struct plat_stmmacenet_data nxpmac_plat_data = {
//	.phy_addr = 3,	// Realtek
	.phy_addr = 7,	// Micrel
	.interface = PHY_INTERFACE_MODE_RGMII,
	.autoneg = AUTONEG_ENABLE, //AUTONEG_ENABLE or AUTONEG_DISABLE
	.speed = SPEED_100,
	.duplex = DUPLEX_FULL,
	.pbl = 16,          /* burst 16 */
	.clk_csr = 0x28,    /* clk_csr_i = 20-35MHz & MDC = clk_csr_i/8 */
	.has_gmac = 1,      /* GMAC ethernet    */
	.enh_desc = 0,
	.mdio_bus_data = &nxpmac0_mdio_bus,
	.init = &nxpmac_init,
};

/* DWC GMAC Controller registration */

static struct resource nxpmac_resource[] = {
    [0] = DEFINE_RES_MEM(PHY_BASEADDR_GMAC, SZ_8K),
    [1] = DEFINE_RES_IRQ_NAMED(IRQ_PHY_GMAC, "macirq"),
};

static u64 nxpmac_dmamask = DMA_BIT_MASK(32);

struct platform_device nxp_gmac_dev = {
    .name           = "stmmaceth",  //"nxp4330-gmac",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(nxpmac_resource),
    .resource       = nxpmac_resource,
    .dev            = {
        .dma_mask           = &nxpmac_dmamask,
        .coherent_dma_mask  = DMA_BIT_MASK(32),
        .platform_data      = &nxpmac_plat_data,
    }
};
#endif

/*------------------------------------------------------------------------------
 * USB HSIC power control.
 */
int nxp_hsic_phy_pwr_on(struct platform_device *pdev, bool on)
{
	return 0;
}
EXPORT_SYMBOL(nxp_hsic_phy_pwr_on);

/*------------------------------------------------------------------------------
 * register board platform devices
 */
void __init nxp_board_devices_register(void)
{
	printk("[Register board platform devices]\n");

#if defined(CONFIG_ARM_NXP4330_CPUFREQ)
	printk("plat: add dynamic frequency (pll.%d)\n", dfs_plat_data.pll_dev);
	platform_device_register(&dfs_plat_device);
#endif

#if defined (CONFIG_FB_NEXELL)
	printk("plat: add framebuffer\n");
	platform_add_devices(fb_devices, ARRAY_SIZE(fb_devices));
#endif

#if defined(CONFIG_MMC_DW)
	#ifdef CONFIG_MMC_NEXELL_CH0
	nxp_mmc_add_device(0, &_dwmci0_data);
	#endif
	#ifdef CONFIG_MMC_NEXELL_CH1
	nxp_mmc_add_device(1, &_dwmci1_data);
	#endif
#endif

#if defined(CONFIG_MTD_NAND_NEXELL)
	platform_device_register(&nand_plat_device);
#endif

#if defined(CONFIG_KEYBOARD_NEXELL_KEY) || defined(CONFIG_KEYBOARD_NEXELL_KEY_MODULE)
	printk("plat: add device keypad\n");
	platform_device_register(&key_plat_device);
#endif

#if defined(CONFIG_REGULATOR_NXE2000)
	printk("plat: add device nxe2000 pmic\n");
	i2c_register_board_info(NXE2000_I2C_BUS, nxe2000_regulators, ARRAY_SIZE(nxe2000_regulators));
#endif

#if defined(CONFIG_NXP4330_MP2TS_IF)
	printk("plat: add device misc mpegts\n");
	platform_device_register(&mpegts_plat_device);
#endif

#if defined(CONFIG_SND_SPDIF_TRANSCIEVER) || defined(CONFIG_SND_SPDIF_TRANSCIEVER_MODULE)
	printk("plat: add device spdif playback\n");
	platform_device_register(&spdif_transciever);
	platform_device_register(&spdif_trans_dai);
#endif

#if defined(CONFIG_V4L2_NEXELL) || defined(CONFIG_V4L2_NEXELL_MODULE)
    printk("plat: add device nxp-v4l2\n");
    platform_device_register(&nxp_v4l2_dev);
#endif

#if defined(CONFIG_NXPMAC_ETH)
    printk("plat: add device nxp-gmac\n");
    platform_device_register(&nxp_gmac_dev);
#endif

	/* END */
	printk("\n");
}
