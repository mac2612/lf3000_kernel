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

#include <linux/amba/pl022.h>
/* nexell soc headers */
#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>



/*------------------------------------------------------------------------------
 * DW MMC (Synopsys DesignWare Memory Card Interface)
 */
#if defined(CONFIG_ARM_NXP4330_CPUFREQ)

static unsigned long dfs_freq_table[][2] = {
//	{ 1000000, 1200 },
//	{  900000, 1200 },
	{  800000, 1200 },
	{  780000, 1200 },
	{  760000, 1200 },
	{  740000, 1200 },
	{  720000, 1200 },
	{  562000, 1200 },
	{  533000, 1200 },
	{  490000, 1200 },
	{  470000, 1200 },
	{  460000, 1200 },
	{  450000, 1200 },
	{  440000, 1200 },
	{  430000, 1200 },
	{  420000, 1200 },
	{  410000, 1200 },
	{  400000, 1200 },
	{  399000, 1200 },
	{  390000, 1200 },
	{  384000, 1200 },
	{  350000, 1200 },
	{  330000, 1200 },
	{  300000, 1200 },
	{  266000, 1200 },
	{  250000, 1200 },
	{  220000, 1200 },
	{  200000, 1200 },
	{  166000, 1200 },
	{  147500, 1200 },
	{  133000, 1200 },
	{  125000, 1200 },
	{  100000, 1200 },
};

struct nxp_cpufreq_plat_data dfs_plat_data = {
	.pll_dev	   	= CONFIG_NXP4330_CPUFREQ_PLLDEV,
	.freq_table	   	= dfs_freq_table,
	.table_size	   	= ARRAY_SIZE(dfs_freq_table),
//	.max_cpufreq   	= 700000,
//	.max_retention 	=  5,
//	.rest_cpufreq  	= 500000,
//	.rest_retention = 20,
};

static struct platform_device dfs_plat_device = {
	.name			= DEV_NAME_CPUFREQ,
	.dev			= {
		.platform_data	= &dfs_plat_data,
	}
};

#endif

/*------------------------------------------------------------------------------
 * Network DM9000
 */
#if defined(CONFIG_DM9000) || defined(CONFIG_DM9000_MODULE)
#include <linux/dm9000.h>

static struct resource dm9000_resource[] = {
	[0] = {
		.start	= CFG_ETHER_EXT_PHY_BASEADDR,
		.end	= CFG_ETHER_EXT_PHY_BASEADDR + 1,		// 1 (8/16 BIT)
		.flags	= IORESOURCE_MEM
	},
	[1] = {
		.start	= CFG_ETHER_EXT_PHY_BASEADDR + 4,		// + 4 (8/16 BIT)
		.end	= CFG_ETHER_EXT_PHY_BASEADDR + 5,		// + 5 (8/16 BIT)
		.flags	= IORESOURCE_MEM
	},
	[2] = {
		.start	= CFG_ETHER_EXT_IRQ_NUM,
		.end	= CFG_ETHER_EXT_IRQ_NUM,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	}
};

static struct dm9000_plat_data eth_plat_data = {
	.flags		= DM9000_PLATF_8BITONLY,	// DM9000_PLATF_16BITONLY
};

static struct platform_device dm9000_plat_device = {
	.name			= "dm9000",
	.id				= 0,
	.num_resources	= ARRAY_SIZE(dm9000_resource),
	.resource		= dm9000_resource,
	.dev			= {
		.platform_data	= &eth_plat_data,
	}
};
#endif	/* CONFIG_DM9000 || CONFIG_DM9000_MODULE */

/*------------------------------------------------------------------------------
 * Frame Buffer platform device
 */
#include "dev-fb.c"

/*------------------------------------------------------------------------------
 * backlight : generic pwm device
 */
#if defined(CONFIG_BACKLIGHT_PWM)
#include <linux/pwm_backlight.h>

static struct platform_pwm_backlight_data bl_plat_data = {
	.pwm_id			= CFG_LCD_PRI_PWM_CH,
	.max_brightness = 350,	/* 255 is 100%, set over 100% */
	.dft_brightness = 128,	/* 50% */
	.pwm_period_ns	= 1000000000/CFG_LCD_PRI_PWM_FREQ,
};

static struct platform_device bl_plat_device = {
	.name	= "pwm-backlight",
	.id		= -1,
	.dev	= {
		.platform_data	= &bl_plat_data,
	},
};
#endif

/*------------------------------------------------------------------------------
 * NAND device
 */
#if defined(CONFIG_MTD_NAND_NEXELL)
#include <linux/mtd/partitions.h>
#include <asm-generic/sizes.h>

static struct mtd_partition nxp_nand_parts[] = {
#if 0
	{
		.name           = "root",
		.offset         =   0 * SZ_1M,
	},
#else
	{
		.name		= "system",
		.offset		=  64 * SZ_1M,
		.size		= 512 * SZ_1M,
	}, {
		.name		= "cache",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 256 * SZ_1M,
	}, {
		.name		= "userdata",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	}
#endif
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
 * Touch platform device
 */
#if defined(CONFIG_TOUCHSCREEN_FT5X0X)
#include <linux/i2c.h>
#define	FT5X0X_I2C_BUS		(1)

struct nxp_ts_cali_plat_data ts_plat_data = {
	.touch_points	= 10,
	.x_resol	   	= CFG_DISP_PRI_RESOL_WIDTH,
	.y_resol	   	= CFG_DISP_PRI_RESOL_HEIGHT,
	.rotate			= 90,
};

static struct i2c_board_info __initdata ft5x0x_i2c_bdi = {
	.type	= "ft5x06_ts",
	.addr	= (0x70>>1),
    .irq    = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
	.platform_data = &ts_plat_data,
};
#endif


/*------------------------------------------------------------------------------
 * ANDROID timed gpio platform device
 */
#if defined(CONFIG_GPIOLIB) && defined(CONFIG_ANDROID_TIMED_GPIO)

#define CONFIG_ANDROID_VIBRATION
#include <../../../../drivers/staging/android/timed_gpio.h>

#define ANDROID_VIBRATION_GPIO    (PAD_GPIO_A + 18)
static struct timed_gpio android_vibration = {
    .name         = "vibrator",
    .gpio         = ANDROID_VIBRATION_GPIO,
    .max_timeout  = 15000, /* ms */
};

static struct timed_gpio_platform_data timed_gpio_data = {
    .num_gpios    = 1,
    .gpios        = &android_vibration,
};

static struct platform_device android_timed_gpios = {
    .name         = "timed-gpio",
    .id           = -1,
	.dev          = {
		.platform_data = &timed_gpio_data,
	},
};
#endif
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
	.bt_repeat	= 0,
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

#if defined(CONFIG_SND_CODEC_WM8976) || defined(CONFIG_SND_CODEC_WM8976_MODULE)
#include <linux/i2c.h>

#define WM8976_I2C_BUS          (0)

/* CODEC */
static struct i2c_board_info __initdata wm8976_i2c_bdi = {
        .type   = "wm8978",                     // compatilbe with wm8976
        .addr   = (0x34>>1),            // 0x1A (7BIT), 0x34(8BIT)
};

/* DAI */
struct nxp_snd_dai_plat_data i2s_dai_data = {
        .i2s_ch = 0,
        .sample_rate    = 48000,
        .hp_jack                = {
                .support        = 1,
                .detect_io              = PAD_GPIO_A + 0,
                .detect_level   = 1,
        },
};

static struct platform_device wm8976_dai = {
        .name                   = "wm8976-audio",
        .id                             = 0,
        .dev                    = {
                .platform_data  = &i2s_dai_data,
        }
};
#endif


#if defined(CONFIG_SND_CODEC_RT5631) || defined(CONFIG_SND_CODEC_RT5631_MODULE)
#include <linux/i2c.h>

#define	RT5631_I2C_BUS		(0)

/* CODEC */
static struct i2c_board_info __initdata rt5631_i2c_bdi = {
	.type	= "rt5631",
	.addr	= (0x34>>1),		// 0x1A (7BIT), 0x34(8BIT)
};

/* DAI */
struct nxp_snd_dai_plat_data i2s_dai_data = {
	.i2s_ch	= 0,
	.sample_rate	= 48000,
	.hp_jack 		= {
		.support    	= 1,
		.detect_io		= PAD_GPIO_A + 0,
		.detect_level	= 1,
	},
};

static struct platform_device rt5631_dai = {
	.name			= "rt5631-audio",
	.id				= 0,
	.dev			= {
		.platform_data	= &i2s_dai_data,
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
#else
			.size = 0,
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

#ifdef CONFIG_ION_NXP_CONTIGHEAP_SIZE
    printk("%s: reserve CMA: size %d\n", __func__, CONFIG_ION_NXP_CONTIGHEAP_SIZE * SZ_1K);
#endif
    nxp_cma_region_reserve(regions, map);
}
#endif

/*------------------------------------------------------------------------------
 * PMIC platform device
 */
#if defined(CONFIG_REGULATOR_NXE1999)

#include <linux/i2c.h>
#include <nxe1999.h>
#include <linux/regulator/machine.h>

#define NXE2000_I2C_BUS		0
#define NXE2000_I2C_ADDR	0x64
#define NXE2000_I2C_LEN		1
#define NXE2000_IRQ			(PAD_GPIO_ALV + 4)


/* NXE2000 regulators */
#define REGULATOR_CSM(_ldo, _name)  \
    static struct regulator_consumer_supply _ldo##_supply[] __initdata = {  \
        REGULATOR_SUPPLY(_name, NULL),  \
    }

#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask, _disabled) \
    static struct regulator_init_data nxp4330_##_ldo##_data __initdata = {  \
        .constraints = {                        \
            .name           = _name,            \
            .min_uV         = _min_uV,          \
            .max_uV         = _max_uV,          \
            .always_on      = _always_on,       \
            .boot_on        = _always_on,       \
            .apply_uV       = 1,                \
            .valid_ops_mask = _ops_mask,        \
            .state_mem      = {                 \
                .disabled   = (_disabled == -1 ? 0 : _disabled),    \
                .enabled    = (_disabled == -1 ? 0 : !(_disabled)), \
            },  \
        },      \
        .num_consumer_supplies = ARRAY_SIZE(_ldo##_supply), \
        .consumer_supplies = _ldo##_supply,                \
    }

REGULATOR_CSM(ldo1, "vgps_3.3V");
REGULATOR_INIT(ldo1, "VGPS_3.3V", 3300000, 3300000, 0,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldo2, "vcam1_1.8V");
REGULATOR_INIT(ldo2, "VCAM1_1.8V", 1800000, 1800000, 0,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldo3, "vsys1_1.8V");
REGULATOR_INIT(ldo3, "VSYS1_1.8V", 1800000, 1800000, 1,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldo4, "vsys_1.9V");
REGULATOR_INIT(ldo4, "VSYS_1.9V", 1900000, 1900000, 1,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldo5, "vcam_2.8V");
REGULATOR_INIT(ldo5, "VCAM_2.8V", 2800000, 2800000, 0,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldo6, "valive_3.3V");
REGULATOR_INIT(ldo6, "VALIVE_3.3V", 3300000, 3300000, 1,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldo7, "vvid_2.8V");
REGULATOR_INIT(ldo7, "VVID_2.8V", 2800000, 2800000, 0,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldo8, "vwifi_3.3V");
REGULATOR_INIT(ldo8, "VWIFI_3.3V", 3300000, 3300000, 0,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldo9, "vhub_3.3V");
REGULATOR_INIT(ldo9, "VHUB_3.3V", 3300000, 3300000, 0,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldo10, "vhsic_1.2V");
REGULATOR_INIT(ldo10, "VHSIC_1.2V", 1200000, 1200000, 0,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldortc1, "valive_1.8V");
REGULATOR_INIT(ldortc1, "VALIVE_1.8V", 1800000, 1800000, 1,
        REGULATOR_CHANGE_STATUS, 1);

REGULATOR_CSM(ldortc2, "valive_1.0V");
REGULATOR_INIT(ldortc2, "VALIVE_1.0V", 1000000, 1000000, 1,
        REGULATOR_CHANGE_STATUS, 1);

/* BUCK */
REGULATOR_CSM(buck1, "vdd_arm_1.3V");
REGULATOR_CSM(buck2, "vdd_core_1.1V");
REGULATOR_CSM(buck3, "vdd_sys_3.3V");
REGULATOR_CSM(buck4, "vdd_ddr_1.6V");
REGULATOR_CSM(buck5, "vdd_sys_1.6V");

static struct regulator_init_data nxp4330_buck1_data __initdata = {
    .constraints    = {
        .name       = "VDD_ARM_1.3V",
        .min_uV     = 1300000,
        .max_uV     = 1300000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
        .apply_uV   = 1,
        .always_on  = 1,
        .boot_on    = 1,
    },
    .num_consumer_supplies  = ARRAY_SIZE(buck1_supply),
    .consumer_supplies      = buck1_supply,
};

static struct regulator_init_data nxp4330_buck2_data __initdata = {
    .constraints    = {
        .name       = "VDD_CORE_1.1V",
        .min_uV     = 1100000,
        .max_uV     = 1100000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
        .apply_uV   = 1,
        .always_on  = 1,
        .boot_on    = 1,
    },
    .num_consumer_supplies  = ARRAY_SIZE(buck2_supply),
    .consumer_supplies      = buck2_supply,
};

static struct regulator_init_data nxp4330_buck3_data __initdata = {
    .constraints    = {
        .name       = "VDD_SYS_3.3V",
        .min_uV     = 3300000,
        .max_uV     = 3300000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
        .apply_uV   = 1,
        .always_on  = 1,
        .boot_on    = 1,
    },
    .num_consumer_supplies  = ARRAY_SIZE(buck3_supply),
    .consumer_supplies      = buck3_supply,
};

static struct regulator_init_data nxp4330_buck4_data __initdata = {
    .constraints    = {
        .name       = "VDD_DDR_1.6V",
        .min_uV     = 1600000,
        .max_uV     = 1600000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
        .apply_uV   = 1,
        .always_on  = 1,
        .boot_on    = 1,
    },
    .num_consumer_supplies  = ARRAY_SIZE(buck4_supply),
    .consumer_supplies      = buck4_supply,
};

static struct regulator_init_data nxp4330_buck5_data __initdata = {
    .constraints    = {
        .name       = "VDD_SYS_1.6V",
        .min_uV     = 1600000,
        .max_uV     = 1600000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
        .apply_uV   = 1,
        .always_on  = 1,
        .boot_on    = 1,
    },
    .num_consumer_supplies  = ARRAY_SIZE(buck5_supply),
    .consumer_supplies      = buck5_supply,
};

static struct nxe2000_regulator_data nxp4330_regulators[] __initdata = {
    { NXE2000_LDO1,     &nxp4330_ldo1_data },
    { NXE2000_LDO2,     &nxp4330_ldo2_data },
    { NXE2000_LDO3,     &nxp4330_ldo3_data },
    { NXE2000_LDO4,     &nxp4330_ldo4_data },
    { NXE2000_LDO5,     &nxp4330_ldo5_data },
    { NXE2000_LDO6,     &nxp4330_ldo6_data },
    { NXE2000_LDO7,     &nxp4330_ldo7_data },
    { NXE2000_LDO8,     &nxp4330_ldo8_data },
    { NXE2000_LDO9,     &nxp4330_ldo9_data },
    { NXE2000_LDO10,    &nxp4330_ldo10_data },
    { NXE2000_LDORTC1,  &nxp4330_ldortc1_data },
    { NXE2000_LDORTC2,  &nxp4330_ldortc2_data },
    { NXE2000_BUCK1,    &nxp4330_buck1_data },
    { NXE2000_BUCK2,    &nxp4330_buck2_data },
    { NXE2000_BUCK3,    &nxp4330_buck3_data },
    { NXE2000_BUCK4,    &nxp4330_buck4_data },
    { NXE2000_BUCK5,    &nxp4330_buck5_data },
};

#if 0
#define CFG_GPIO_OTG_USBID_DET		(PAD_GPIO_B + 30)
#define CFG_GPIO_OTG_VBUS_DET		(PAD_GPIO_B + 28)
#define CFG_GPIO_PMIC_VUSB_DET		(PAD_GPIO_ALV + 2)
#define CFG_GPIO_PMIC_LOWBAT_DET	(PAD_GPIO_ALV + 3)	/* Critical low battery detect */
#endif

static struct nxe2000_pdata nxp4330_nxe2000_pdata __initdata = {
    .gpio_eint          = NXE2000_IRQ,
    .irq_base           = IRQ_SYSTEM_END,
    .wakeup             = 1,

    .num_regulators     = ARRAY_SIZE(nxp4330_regulators),
    .regulators         = nxp4330_regulators,

    .gpio_otg_usbid     = CFG_GPIO_OTG_USBID_DET,
    .gpio_otg_vbus      = CFG_GPIO_OTG_VBUS_DET,
    .gpio_pmic_vbus     = CFG_GPIO_PMIC_VUSB_DET,
    .gpio_pmic_lowbat   = CFG_GPIO_PMIC_LOWBAT_DET,

    .have_battery       = 1,
    .rdstate_periodic   = 1000,             /* milisecond */

    .slp_prio_buck[0]   = 12,   // 0 ~ 14, off => 15
    .slp_prio_buck[1]   = 14,   // for suspend mode.
    .slp_prio_buck[2]   = 2,
    .slp_prio_buck[3]   = 0xF,
    .slp_prio_buck[4]   = 8,

    .slp_prio_ldo[0]    = 0,    // 0 ~ 14, off => 15
    .slp_prio_ldo[1]    = 0,
    .slp_prio_ldo[2]    = 6,
    .slp_prio_ldo[3]    = 6,
    .slp_prio_ldo[4]    = 0,
    .slp_prio_ldo[5]    = 0xF,
    .slp_prio_ldo[6]    = 4,
    .slp_prio_ldo[7]    = 0,
    .slp_prio_ldo[8]    = 0,
    .slp_prio_ldo[9]    = 10,

    /* power-supply shutoff */
    .slp_prio_pso[0]    = 0xF,  // 0 ~ 14, off => 15
    .slp_prio_pso[1]    = 0xF,
    .slp_prio_pso[2]    = 0xF,
    .slp_prio_pso[3]    = 0xF,
    .slp_prio_pso[4]    = 1,

    .slp_buck_vol[0]    = 1300000,      /* 1.3V ARM */
    .slp_buck_vol[1]    = 1100000,      /* 1.1V CORE */
    .slp_buck_vol[2]    = 3300000,      /* 3.3V SYS */
    .slp_buck_vol[3]    = 1600000,      /* 1.6V DDR */
    .slp_buck_vol[4]    = 1600000,      /* 1.6V SYS */

    .slp_ldo_vol[0]     = 3300000,      /* 3.3V GPS */
    .slp_ldo_vol[1]     = 1800000,      /* 1.8V CAM1 */
    .slp_ldo_vol[2]     = 1800000,      /* 1.8V SYS1 */
    .slp_ldo_vol[3]     = 1900000,      /* 1.9V SYS */
    .slp_ldo_vol[4]     = 2800000,      /* 2.8V CAM */
    .slp_ldo_vol[5]     = 3300000,      /* 3.3V ALIVE */
    .slp_ldo_vol[6]     = 2800000,      /* 2.8V VID */
    .slp_ldo_vol[7]     = 3300000,      /* 3.3V WIFI */
    .slp_ldo_vol[8]     = 3300000,      /* 3.3V HUB */
    .slp_ldo_vol[9]     = 1200000,      /* 1.2V HSIC */

    .bat_max_uV         = 4200000,      /* 4.2V */
    .bat_min_uV         = 3400000,      /* 3.4V */
    .bat_low_uV         = 3600000,      /* 3.6V */

    .adp_ilim_current   = 1500000,      /* 1.5A */
    .adp_chg_current    = 800000,       /* 800mA */

    .usb_ilim_current   = 500000,       /* 500mA */
    .usb_chg_current    = 500000,       /* 500mA */
};

static struct i2c_board_info nxe2000_i2c_pmic_devs[] __initdata = {
    {
        I2C_BOARD_INFO("nxe2000", NXE2000_I2C_ADDR >> 1),
        .platform_data  = &nxp4330_nxe2000_pdata,
        .irq            = PB_PIO_IRQ(NXE2000_IRQ),
    },
};
#endif	/* CONFIG_REGULATOR_NXE1999  */

#if defined(CONFIG_REGULATOR_NXE2000)

#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/nxe2000.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/nxe2000-regulator.h>
#include <linux/power/nxe2000_battery.h>
//#include <linux/rtc/rtc-nxe2000.h>
//#include <linux/rtc.h>

#define NXE2000_I2C_BUS		(0)
#define NXE2000_I2C_ADDR	(0x64 >> 1)
#define NXE2000_IRQ			(PAD_GPIO_ALV + 4)

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
	REGULATOR_SUPPLY("vwifi_3.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo9_supply_0[] = {
	REGULATOR_SUPPLY("vhub_3.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo10_supply_0[] = {
	REGULATOR_SUPPLY("vhsic_1.2V", NULL),
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
NXE2000_PDATA_INIT(dc1,      0, 1000000, 2000000, 1, 1, 1300000, 1, 12);	/* 1.3V ARM */
NXE2000_PDATA_INIT(dc2,      0, 1000000, 2000000, 1, 1, 1200000, 1, 14);	/* 1.2V CORE */
NXE2000_PDATA_INIT(dc3,      0, 1000000, 3500000, 1, 1, 3300000, 1,  2);	/* 3.3V SYS */
NXE2000_PDATA_INIT(dc4,      0, 1000000, 2000000, 1, 1, 1600000, 1, -1);	/* 1.6V DDR */
NXE2000_PDATA_INIT(dc5,      0, 1000000, 2000000, 1, 1, 1600000, 1,  8);	/* 1.6V SYS */

NXE2000_PDATA_INIT(ldo1,     0, 1000000, 3500000, 1, 0, 3300000, 1,  2);	/* 3.3V GPS */
NXE2000_PDATA_INIT(ldo2,     0, 1000000, 3500000, 0, 0, 1800000, 0,  2);	/* 1.8V CAM1 */
NXE2000_PDATA_INIT(ldo3,     0, 1000000, 3500000, 1, 0, 1800000, 1,  6);	/* 1.8V SYS1 */
NXE2000_PDATA_INIT(ldo4,     0, 1000000, 3500000, 1, 0, 1900000, 1,  6);	/* 1.9V SYS */
NXE2000_PDATA_INIT(ldo5,     0, 1000000, 3500000, 0, 0, 2800000, 0,  1);	/* 2.8V VCAM */
NXE2000_PDATA_INIT(ldo6,     0, 1000000, 3500000, 1, 0, 3300000, 1, -1);	/* 3.3V ALIVE */
NXE2000_PDATA_INIT(ldo7,     0, 1000000, 3500000, 1, 0, 2800000, 1,  4);	/* 2.8V VID */
NXE2000_PDATA_INIT(ldo8,     0, 1000000, 3500000, 0, 0, 3300000, 1,  2);	/* 3.3V WIFI */
NXE2000_PDATA_INIT(ldo9,     0, 1000000, 3500000, 1, 0, 3300000, 1,  2);	/* 3.3V HUB */
NXE2000_PDATA_INIT(ldo10,    0, 1000000, 3500000, 1, 0, 1200000, 0,  0);	/* 1.2V HSIC */
NXE2000_PDATA_INIT(ldortc1,  0, 1700000, 3500000, 1, 0, 1800000, 1, -1);	/* 1.8V ALIVE */
NXE2000_PDATA_INIT(ldortc2,  0, 1000000, 3500000, 1, 0, 1000000, 1, -1);	/* 1.0V ALIVE */


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
		.ch_vrchg		= 0xFF,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset	= 0xFF,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 		= 0x07,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x0E,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
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
    .dev_info = mp2ts_dev_info,
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
#ifdef CFG_IO_CAMERA_RESET
/*
    static bool is_first = true;
    PM_DBGOUT("%s: is_first %d, on %d\n", __func__, is_first, on);
    if (on) {
        if (is_first) {
            nxp_soc_gpio_set_out_value(CFG_IO_CAMERA_RESET, 0);
            nxp_soc_gpio_set_io_dir(CFG_IO_CAMERA_RESET, 1);
            nxp_soc_gpio_set_io_func(CFG_IO_CAMERA_RESET, nxp_soc_gpio_get_altnum(CFG_IO_CAMERA_RESET));
            mdelay(1);
            nxp_soc_gpio_set_out_value(CFG_IO_CAMERA_RESET, 1);
            is_first = false;
        }
    } else {
        is_first = true;
    }
*/
#endif
    return 0;
}

static int camera_set_clock(ulong clk_rate)
{
    PM_DBGOUT("%s: %d\n", __func__, (int)clk_rate);
    if (clk_rate > 0)
        nxp_soc_pwm_set_frequency(1, clk_rate, 50);
    else
        nxp_soc_pwm_set_frequency(1, 0, 0);
    msleep(1);
    return 0;
}

static void vin_setup_io(int module, bool force)
{

        u_int *pad;
        int   i, len;
        u_int io, fn;

        // VIP1:0 = VCLK, VID0 ~ 7
        const u_int port[][2] = {
            // VCLK, HSYNC, VSYNC
            { PAD_GPIO_E +  4, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_E +  5, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_E +  6, NX_GPIO_PADFUNC_1 },
            // DATA
            { PAD_GPIO_D + 28, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_D + 29, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_D + 30, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_D + 31, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_E +  0, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_E +  1, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_E +  2, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_E +  3, NX_GPIO_PADFUNC_1 },
        };

        printk("kjh kjh kjh  %s\n", __func__);

        pad = (u_int *)port;
        len = sizeof(port)/sizeof(port[0]);

        for (i = 0; i < len; i++) {
            io = *pad++;
            fn = *pad++;
            nxp_soc_gpio_set_io_dir(io, 0);
            nxp_soc_gpio_set_io_func(io, fn);
        }

}


static struct i2c_board_info __initdata tw9910_i2c_bdi = {
        I2C_BOARD_INFO("tw9910", 0x88>>1),
};

/* for mipi camera: s5k4ecgx */
static int s5k4ecgx_power_enable(bool on)
{
#if defined(CFG_IO_MIPI_CAMERA_POWER_ENABLE) && defined(CFG_IO_MIPI_CAMERA_RESETN)
    static bool is_mipi_first = true;
    PM_DBGOUT("%s: on %d, is_mipi_first %d\n", __func__, on, is_mipi_first);
    if (on) {
        if (is_mipi_first) {
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
        }
    } else {
        /* nxp_soc_gpio_set_out_value(CFG_IO_MIPI_CAMERA_POWER_ENABLE, on); */
        is_mipi_first = true;
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

static void mipi_vin_setup_io(int module, bool force)
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
        .board_info = &tw9910_i2c_bdi,
        .i2c_adapter_id = 0,
    },
#if defined(CONFIG_NXP_CAPTURE_MIPI_CSI)
    {
        .board_info = &s5k4ecgx_i2c_boardinfo[0],
        /* lynx */
        .i2c_adapter_id = 0,
        /* vtk */
        /* .i2c_adapter_id = 1, */
    },
#endif
};

static struct nxp_capture_platformdata capture_plat_data[] = {
    {
        .module = 1,
        .sensor = &sensor[0],
        .type = NXP_CAPTURE_INF_PARALLEL,
        .parallel = {
            /* for 656 */
            .is_mipi        = false,
            .external_sync  = false,
            .h_active       = 640,
            /*.h_frontporch   = 8,*/
            .h_frontporch   = 6,
            .h_syncwidth    = 256,
            //.h_backporch    = 10,
            .h_backporch    = 0,
            .v_active       = 480,
            .v_frontporch   = 0,
            .v_syncwidth    = 1,
            .v_backporch    = 0,
            .clock_invert   = false,
            .port           = 0,
            .data_order     = NXP_VIN_CBY0CRY1,
            //.data_order     = NXP_VIN_Y0CBY1CR,
            .interlace      = true,
            .clk_rate       = 27000000,
            .late_power_down = false,
            .power_enable   = camera_power_enable,
            .set_clock      = camera_set_clock,
            .setup_io       = vin_setup_io,

            /* for 601
            .is_mipi        = false,
            .external_sync  = true,
            .h_active       = 720,
            .h_frontporch   = 77,
            .h_syncwidth    = 52,
            //.h_backporch    = 48,
            .h_backporch    = 15,
            .v_active       = 480,
            .v_frontporch   = 1,
            .v_syncwidth    = 1,
            .v_backporch    = 1,
            .clock_invert   = true,
            .port           = 0,
            //.data_order     = NXP_VIN_Y0CBY1CR,
            .data_order     = NXP_VIN_CBY0CRY1,
            .interlace      = false,
            .clk_rate       = 27000000,
            .late_power_down = false,
            .power_enable   = camera_power_enable,
            .set_clock      = camera_set_clock,
            .setup_io       = vin_setup_io,
*/
        },
        .deci = {
            .start_delay_ms = 0,
            .stop_delay_ms  = 0,
        },
    },
#if defined(CONFIG_NXP_CAPTURE_MIPI_CSI)
    {
        .module = 1,
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
            .late_power_down = false,
            .power_enable   = s5k4ecgx_power_enable,
            .set_clock      = s5k4ecgx_set_clock,
            .setup_io       = mipi_vin_setup_io,
        },
        .deci = {
            .start_delay_ms = 0,
            .stop_delay_ms  = 0,
        },
        .csi = &mipi_plat_data,
    },
#endif
    { 0, NULL, 0, },
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


#if defined (CONFIG_INV_MPU_IIO) || defined (CONFIG_INV_MPU_IIO_MODULE)

#include <linux/mpu.h>
#include <linux/gpio.h>
#include <linux/akm8975.h>

#define MPUIRQ_GPIO         (PAD_GPIO_A + 20)
#define MPU_I2C_BUS 		(2)
static struct mpu_platform_data mpu_data = {
	.int_config = 0x00,
	.level_shifter = 0,
	.orientation = {
			1 ,0, 0,
			0 ,1 ,0 ,
			0 ,0, 1 },

	.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id = COMPASS_ID_AK8975,
	.secondary_i2c_addr = 0x0c,
	.secondary_orientation = {
				1, 0, 0,
			    0, 1, 0,
			    0, 0, -1 },
	.key = {0xdd, 0x16, 0xcd, 0x7, 0xd9, 0xba, 0x97, 0x37,
	        0xcd, 0xfe, 0x23, 0x90, 0xe1, 0x66, 0x2f, 0x32},
};
static struct i2c_board_info __initdata inv_mpu_i2c0_boardinfo[] =  {
	{
		I2C_BOARD_INFO("mpu9150", 0x68),
	    .irq            = PB_PIO_IRQ(MPUIRQ_GPIO),
	    .platform_data = &mpu_data,
     },
};


#endif
/*------------------------------------------------------------------------------
 * SSP/SPI
 */
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
#include <linux/spi/spi.h>
#if (CFG_SPI0_CS_GPIO_MODE)
static void spi0_cs(u32 chipselect)
{
	if(nxp_soc_gpio_get_io_func( CFG_SPI0_CS )!= nxp_soc_gpio_get_altnum( CFG_SPI0_CS))
		nxp_soc_gpio_set_io_func( CFG_SPI0_CS, nxp_soc_gpio_get_altnum( CFG_SPI0_CS));

	nxp_soc_gpio_set_io_dir( CFG_SPI0_CS,1);
	nxp_soc_gpio_set_out_value(	 CFG_SPI0_CS , chipselect);
}
#endif
struct pl022_config_chip spi0_info = {
    /* available POLLING_TRANSFER, INTERRUPT_TRANSFER, DMA_TRANSFER */
    .com_mode = CFG_SPI0_COM_MODE,
    .iface = SSP_INTERFACE_MOTOROLA_SPI,
    /* We can only act as master but SSP_SLAVE is possible in theory */
    .hierarchy = SSP_MASTER,
    /* 0 = drive TX even as slave, 1 = do not drive TX as slave */
    .slave_tx_disable = 1,
    .rx_lev_trig = SSP_RX_4_OR_MORE_ELEM,
    .tx_lev_trig = SSP_TX_4_OR_MORE_EMPTY_LOC,
    .ctrl_len = SSP_BITS_8,
    .wait_state = SSP_MWIRE_WAIT_ZERO,
    .duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
    /*
     * This is where you insert a call to a function to enable CS
     * (usually GPIO) for a certain chip.
     */
#if (CFG_SPI0_CS_GPIO_MODE)
    .cs_control = spi0_cs,
#endif
	.clkdelay = SSP_FEEDBACK_CLK_DELAY_1T,

};

static struct spi_board_info spi_plat_board[] __initdata = {
    [0] = {
        .modalias        = "spidev",    /* fixup */
        .max_speed_hz    = 3125000,     /* max spi clock (SCK) speed in HZ */
        .bus_num         = 0,           /* Note> set bus num, must be smaller than ARRAY_SIZE(spi_plat_device) */
        .chip_select     = 0,           /* Note> set chip select num, must be smaller than spi cs_num */
        .controller_data = &spi0_info,
        .mode            = SPI_MODE_3 | SPI_CPOL | SPI_CPHA,
    },
};

#endif
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
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_CMD23,
	.detect_delay_ms= 200,
//	.sdr_timing		= 0x03020001,
//	.ddr_timing		= 0x03030002,
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
	.caps2			= MMC_CAP2_PACKED_WR,
	.desc_sz		= 4,
	.detect_delay_ms= 200,
	.sdr_timing		= 0x03020001,
	.ddr_timing		= 0x03030002,
};
#endif

#endif /* CONFIG_MMC_DW */
/*------------------------------------------------------------------------------
 * register board platform devices
 */
void __init nxp_board_devices_register(void)
{
	printk("[Register board platform devices]\n");

	nxp_fb_device_register();

#if defined(CONFIG_ARM_NXP4330_CPUFREQ)
	printk("plat: add dynamic frequency (pll.%d)\n", dfs_plat_data.pll_dev);
	platform_device_register(&dfs_plat_device);
#endif

#if defined(CONFIG_MMC_DW)
	#ifdef CONFIG_MMC_NEXELL_CH0
	nxp_mmc_add_device(0, &_dwmci0_data);
	#endif
	#ifdef CONFIG_MMC_NEXELL_CH1
	nxp_mmc_add_device(1, &_dwmci1_data);
	#endif
#endif

#if defined(CONFIG_DM9000) || defined(CONFIG_DM9000_MODULE)
	printk("plat: add device dm9000 net\n");
	platform_device_register(&dm9000_plat_device);
#endif

#if defined(CONFIG_BACKLIGHT_PWM)
	printk("plat: add backlight pwm device\n");
	platform_device_register(&bl_plat_device);
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X0X)
	printk("plat: add touch(ftx0x) device\n");
	i2c_register_board_info(FT5X0X_I2C_BUS, &ft5x0x_i2c_bdi, 1);
#endif

#if defined(CONFIG_MTD_NAND_NEXELL)
	platform_device_register(&nand_plat_device);
#endif

#if defined(CONFIG_KEYBOARD_NEXELL_KEY) || defined(CONFIG_KEYBOARD_NEXELL_KEY_MODULE)
	printk("plat: add device keypad\n");
	platform_device_register(&key_plat_device);
#endif

#if defined(CONFIG_ANDROID_VIBRATION)
	printk("plat: add android timed gpio\n");
	platform_device_register(&android_timed_gpios);
#endif

#if defined(CONFIG_REGULATOR_NXE2000)
	printk("plat: add device nxe2000 pmic\n");
	i2c_register_board_info(NXE2000_I2C_BUS, nxe2000_regulators, ARRAY_SIZE(nxe2000_regulators));
#endif

#if defined(CONFIG_REGULATOR_NXE1999)
	printk("plat: add device nxe1999 pmic\n");
	i2c_register_board_info(NXE2000_I2C_BUS, nxe2000_i2c_pmic_devs, ARRAY_SIZE(nxe2000_i2c_pmic_devs));
#endif

#if defined(CONFIG_REGULATOR_NXE1100)
	printk("plat: add device nxe1100 pmic\n");
	i2c_register_board_info(NXE1100_I2C_BUS, nxe1100_i2c_pmic_devs, ARRAY_SIZE(nxe1100_i2c_pmic_devs));
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

#if defined(CONFIG_SND_CODEC_WM8976) || defined(CONFIG_SND_CODEC_WM8976_MODULE)
	printk("plat: add device asoc-wm8976\n");
	i2c_register_board_info(WM8976_I2C_BUS, &wm8976_i2c_bdi, 1);
	platform_device_register(&wm8976_dai);
#endif

#if defined(CONFIG_SND_CODEC_RT5631) || defined(CONFIG_SND_CODEC_RT5631_MODULE)
	printk("plat: add device asoc-rt5631\n");
	i2c_register_board_info(RT5631_I2C_BUS, &rt5631_i2c_bdi, 1);
	platform_device_register(&rt5631_dai);
#endif

#if defined(CONFIG_V4L2_NEXELL) || defined(CONFIG_V4L2_NEXELL_MODULE)
    printk("plat: nxp-v4l2 kjh\n");
    platform_device_register(&nxp_v4l2_dev);
#endif

#if defined (CONFIG_INV_MPU_IIO) || defined (CONFIG_INV_MPU_IIO_MODULE)
	printk("plat: add mpu9150\n");
	i2c_register_board_info (MPU_I2C_BUS, inv_mpu_i2c0_boardinfo,1);
#endif

#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
    spi_register_board_info(spi_plat_board, ARRAY_SIZE(spi_plat_board));
    printk("plat: register spidev\n");
#endif
	/* END */
	printk("\n");
}
