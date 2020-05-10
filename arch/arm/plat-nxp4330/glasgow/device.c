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
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/lf3000/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/dma-mapping.h>
#include <linux/amba/pl022.h>
#include <linux/wl12xx.h>

/* nexell soc headers */
#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>
#include <mach/lfp100.h>
#include <mach/tc7734.h>
#include <mach/bq24250-charger.h>

extern void lf2000_set_scratch_reboot(void);

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
	//{ 1400000, 1000000, },
	//{ 1300000, 1000000, },
	{ 1200000, 1000000, },
	{ 1100000, 1000000, },
	{ 1000000, 1000000, },
	{  900000, 1000000, },
	{  800000, 1000000, },
};

struct nxp_cpufreq_plat_data dfs_plat_data = {
	.pll_dev	   	= CONFIG_NXP4330_CPUFREQ_PLLDEV,
	.freq_table	   	= dfs_freq_table,
	.table_size	   	= ARRAY_SIZE(dfs_freq_table),
	.max_cpufreq    = 1200*1000,
	.max_retention  =   20*1000,
	.rest_cpufreq   =  800*1000,
	.rest_retention =    1*1000,
	//.supply_name 	= "vdd_arm_1.3V",
};

static struct platform_device dfs_plat_device = {
	.name			= DEV_NAME_CPUFREQ,
	.dev			= {
		.platform_data	= &dfs_plat_data,
	}
};

/* cpu over scaling */
static char *freq_proct_list[] = { "com.antutu", };

static struct nxp_cpufreq_limit_data freq_limit_data = {
	.limit_name		= freq_proct_list,
	.limit_num 		= ARRAY_SIZE(freq_proct_list),
	.aval_max_freq 	= 1600000,
	.op_max_freq	= 1200000,
};

static struct platform_device freq_limit_device = {
	.name			= "cpufreq-limit",
	.dev			= {
		.platform_data	= &freq_limit_data,
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
	.max_brightness = 255,
	.dft_brightness = 128,
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
#if 0	// FIXME: sesters, added in Linux 3.4.39
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
#endif

// sesters: from Linux 3.4.24 kernel
#if defined(CONFIG_MTD_NAND)
static struct platform_nand_data lf2000_platform_nand_data = {
	/* first the struct platform_nand_chip */
	{
		.nr_chips = 1
	},
	/* then the platform_nand_ctrl */
	{
		.priv = NULL
	}
};
static struct resource lf2000_nand_resource = {
	.start			= 0x2C000000,
	.end			= 0x2C000018,
	.flags			= IORESOURCE_MEM,
};
static struct platform_device lf2000_nand_plat_device = {
	.name		= DEV_NAME_NAND,
	.id		= -1,
	.num_resources	= 1,
	.resource	= &lf2000_nand_resource,
	.dev =  {
		.platform_data = &lf2000_platform_nand_data,
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.dma_mask = &lf2000_nand_plat_device.dev.coherent_dma_mask,
		}
};
#endif

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
 * BQ24250 charger device
 */
#ifdef CONFIG_BQ24250_CHARGER

#include <linux/i2c.h>
#define BQ24250_I2C_ADDR	0x6A //0xD4
#define BQ24250_I2C_BUS		1

struct bq24250_charger_platform_data bq24250_pdata = {
	.use_mains		= 1,
	.use_usb		= 1, 
	.max_charge_current	= 0,
	.termination_current	= 0,
	.mains_current_limit	= 0,
	.usb_hc_current_limit	= 0, 
	.max_charge_voltage	= 0,	
};

static struct i2c_board_info bq24250_i2c_device = {
	I2C_BOARD_INFO("bq24250", BQ24250_I2C_ADDR),
	.platform_data = &bq24250_pdata,
//	.irq = CHG_INT,
};

#endif

/*------------------------------------------------------------------------------
 * CTP device
 */
#if defined (CONFIG_TOUCHSCREEN_ICN83xx_IIC) || defined(CONFIG_TOUCHSCREEN_ICN83xx_IIC_MODULE)
#include <linux/i2c.h>
#include <linux/lf3000/gpio.h>
#define	ICN83XX_I2C_BUS		2
#define ICN83XX_I2C_ADDR	0x40
#define ICN83XX_I2C_LEN		1

static struct i2c_board_info __initdata icn83xx_i2c_info[] = {
	{
		I2C_BOARD_INFO("icn83xx", ICN83XX_I2C_ADDR),
		//.platform_data	= &icn83xx_info,
		.flags = 0,
		.irq = TOUCHSCREEN_X2,
	},
};
#endif

#if defined (CONFIG_TOUCHSCREEN_ICN85xx_IIC) || defined(CONFIG_TOUCHSCREEN_ICN85xx_IIC_MODULE)
#include <linux/i2c.h>
#include <linux/lf3000/gpio.h>
#define	ICN85XX_I2C_BUS		2
#define ICN85XX_I2C_ADDR	0x48
#define ICN85XX_I2C_LEN		1

static struct i2c_board_info __initdata icn85xx_i2c_info[] = {
	{
		I2C_BOARD_INFO("icn85xx", ICN85XX_I2C_ADDR),
		//.platform_data	= &icn85xx_info,
		.flags = 0,
		.irq = TOUCHSCREEN_X2,
	},
};
#endif

#if defined(CONFIG_TOUCHSCREEN_PAP11XX) || defined(CONFIG_TOUCHSCREEN_PAP11XX_MODULE)

#include <linux/i2c.h>
#include <pap11xx_touch.h>
#define	CTP_I2C_BUS		(2)

struct pap11xx_i2c_platform_data pap11xx_pdata = {
	/* LF ts_cali_plat_data. Will add them later if needed
	.touch_points	= 10,
	.x_resol	   	= CFG_DISP_PRI_RESOL_WIDTH,
	.y_resol	   	= CFG_DISP_PRI_RESOL_HEIGHT,
	.rotate			= 90,
	*/
	//.gpio_irq		= GPIO_B24,
	//.irq_cfg		= LEVEL_LOW,
	// Not used even define it there
	.screen_max_x = 272,
	.screen_max_y = 480,

#define	CFG_IO_TOUCH_SCREEN_RESET			((PAD_GPIO_B + 24))
	.gpio_reset	= CFG_IO_TOUCH_SCREEN_RESET,
	// .reset_polarity	= 0,
	.reset_cfg = 0,	// Active low
};

// defined in include/linux/i2c.h
static struct i2c_board_info  pap11xx_i2c_bdi = {
	/*Either
	.type	= "pap11xx_ts",
	.addr	= 0x33,
	 Or */
	I2C_BOARD_INFO ("pap11xx_ts", 0x33),
	// TOUCHSCREEN RST=GPIOB24, INT=GPIOB25
#define	CFG_IO_TOUCH_PENDOWN_DETECT			((PAD_GPIO_B + 25) | PAD_FUNC_ALT0)
	.irq = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
	.platform_data = &pap11xx_pdata,
};

#endif


#if defined(CONFIG_TOUCHSCREEN_ELAN_KTF2K) || defined(CONFIG_TOUCHSCREEN_ELAN_KTF2K_MODULE)

#include <linux/i2c.h>
#include <plat/ektf2k.h>
#define	CTP_I2C_BUS		(2)

struct elan_ktf2k_i2c_platform_data ts_elan_ktf2k_data[] = {
	{
		.version = 0x0001,
		.abs_x_min = 0,
		.abs_x_max = ELAN_X_MAX,   // The max resolution of x
		.abs_y_min = 0,
		.abs_y_max = ELAN_Y_MAX,   // The max resolution of y
		//.intr_gpio = TEGRA_GPIO_PJ7,
		.intr_gpio = (PAD_GPIO_B + 25),
		//.power = elan_ktf2k_ts_power,
	},
};
static struct i2c_board_info elan_i2c_devices[] = {
	{
		I2C_BOARD_INFO(ELAN_KTF2K_NAME, 0x15),
		.platform_data = &ts_elan_ktf2k_data[0],
#define	CFG_IO_TOUCH_PENDOWN_DETECT			((PAD_GPIO_B + 25) | PAD_FUNC_ALT0)
		.irq = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
		//.irq = (INT_GPIO_BASE + TEGRA_GPIO_PJ7),
	},
};

#endif


#if defined(CONFIG_TOUCHSCREEN_ELAN_KTF2K_7INCH) || defined(CONFIG_TOUCHSCREEN_ELAN_KTF2K_7INCH_MODULE)

#include <linux/i2c.h>
#include <plat/ektf2k7.h>
#define	CTP_I2C_BUS		(2)

struct elan_ktf2k_i2c_platform_data ts_elan_ktf2k_data7[] = {
	{
		.version = 0x0001,
		.abs_x_min = 0,
		.abs_x_max = ELAN_X_MAX,   // The max resolution of x
		.abs_y_min = 0,
		.abs_y_max = ELAN_Y_MAX,   // The max resolution of y
		//.intr_gpio = TEGRA_GPIO_PJ7,
		.intr_gpio = (PAD_GPIO_B + 25),
		//.power = elan_ktf2k_ts_power,
	},
};
static struct i2c_board_info elan_i2c_devices7[] = {
	{
		I2C_BOARD_INFO(ELAN_KTF2K_NAME7, 0x10),
		.platform_data = &ts_elan_ktf2k_data7[0],
#define	CFG_IO_TOUCH_PENDOWN_DETECT			((PAD_GPIO_B + 25) | PAD_FUNC_ALT0)
		.irq = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
		//.irq = (INT_GPIO_BASE + TEGRA_GPIO_PJ7),
	},
};

#endif


#if defined(CONFIG_TOUCHSCREEN_BCD_AP386X) || defined(CONFIG_TOUCHSCREEN_BCD_AP386X_MODULE)

#include <linux/i2c.h>
#include <tu_drvs.h>
#define	CTP_I2C_BUS		(2)

struct tu_platform_data tu_pdata = {
		.version = 0x0001,
		.abs_x_min = 0,
		.abs_x_max = AA_X_SIZE,   // The max resolution of x
		.abs_y_min = 0,
		.abs_y_max = AA_Y_SIZE,   // The max resolution of y
		//.intr_gpio = TEGRA_GPIO_PJ7,
		.intr_gpio = (PAD_GPIO_B + 25),

#define	CFG_IO_TOUCH_SCREEN_RESET			((PAD_GPIO_B + 24))
	.gpio_reset	= CFG_IO_TOUCH_SCREEN_RESET,
	// .reset_polarity	= 0,
	.reset_cfg = 0,	// Active low
		//.power = elan_ktf2k_ts_power,
};
static struct i2c_board_info bcd_i2c_device = {

		I2C_BOARD_INFO(TU_I2C_NAME, 0x5f),
		.platform_data = &tu_pdata,
#define	CFG_IO_TOUCH_PENDOWN_DETECT			((PAD_GPIO_B + 25) | PAD_FUNC_ALT0)
		.irq = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
		//.irq = (INT_GPIO_BASE + TEGRA_GPIO_PJ7),

};

#endif

#if defined(CONFIG_TOUCHSCREEN_MStar_MSG21xxA) || defined(CONFIG_TOUCHSCREEN_MStar_MSG21xxA_MODULE)

#include <linux/i2c.h>
#define	CTP_I2C_BUS		(2)

/* from i2c.h:
		struct i2c_board_info {
			char		type[I2C_NAME_SIZE];
			unsigned short	flags;
			unsigned short	addr;
			void		*platform_data;
			struct dev_archdata	*archdata;
			struct device_node *of_node;
			int		irq;
		};
*/

static struct i2c_board_info mstar_i2c_device = {

		I2C_BOARD_INFO("ms-msg21xx",0x26),
		// .platform_data = &mstar_pdata,
#define	CFG_IO_TOUCH_PENDOWN_DETECT			((PAD_GPIO_B + 25) | PAD_FUNC_ALT0)
		.irq = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
};

#endif


#if defined(CONFIG_TOUCHSCREEN_Ilitek_2839) || defined(CONFIG_TOUCHSCREEN_Ilitek_2839_MODULE)

#include <linux/i2c.h>
#define	CTP_I2C_BUS		(2)

static struct i2c_board_info ilitek_i2c_device = {

		I2C_BOARD_INFO("ilitek_ts",0x41),
#define	CFG_IO_TOUCH_PENDOWN_DETECT			((PAD_GPIO_B + 25) | PAD_FUNC_ALT0)
		.irq = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
};

#endif


#if defined(CONFIG_TOUCHSCREEN_Ilitek_2839_7INCH) || defined(CONFIG_TOUCHSCREEN_Ilitek_2839_7INCH_MODULE)

#include <linux/i2c.h>
#define	CTP_I2C_BUS		(2)

static struct i2c_board_info ilitek7_i2c_device = {

		I2C_BOARD_INFO("ilitek_ts7",0x41),
#define	CFG_IO_TOUCH_PENDOWN_DETECT			((PAD_GPIO_B + 25) | PAD_FUNC_ALT0)
		.irq = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
};

#endif


#if defined(CONFIG_TOUCHSCREEN_Goodix_GT967_7INCH) || defined(CONFIG_TOUCHSCREEN_Goodix_GT967_7INCH_MODULE)

#include <linux/i2c.h>
#define	CTP_I2C_BUS		(2)

static struct i2c_board_info GT967_i2c_device = {

		//I2C_BOARD_INFO("CTPM_ts",0x14),
		I2C_BOARD_INFO("CTPM_ts",0x5d),
#define	CFG_IO_TOUCH_PENDOWN_DETECT			((PAD_GPIO_B + 25) | PAD_FUNC_ALT0)
		.irq = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
};

#endif



/*------------------------------------------------------------------------------
 * SSP/SPI
 */
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
#include <linux/spi/spi.h>
static void spi0_cs(u32 chipselect)
{
#if (CFG_SPI0_CS_GPIO_MODE)
    if(nxp_soc_gpio_get_io_func( CFG_SPI0_CS )!= nxp_soc_gpio_get_altnum( CFG_SPI0_CS))
        nxp_soc_gpio_set_io_func( CFG_SPI0_CS, nxp_soc_gpio_get_altnum( CFG_SPI0_CS));

    nxp_soc_gpio_set_io_dir( CFG_SPI0_CS,1);
    nxp_soc_gpio_set_out_value(  CFG_SPI0_CS , chipselect);
#else
    ;
#endif
}
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
        .mode            = SPI_MODE_0,
    },
};

#endif



/*------------------------------------------------------------------------------
 * LED GPIO
 */
#if defined(CONFIG_LEDS_GPIO)
#include <linux/leds.h>

static struct gpio_led gpio_leds[] = {
	{
		.name			= "CPU Live Gpio",
		.default_trigger	= "heartbeat",
		.gpio			= (PAD_GPIO_ALV + 0),
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_leds_pdata = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device gpio_leds_device = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_leds_pdata,
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
	.sample_rate = CFG_AUDIO_SPDIF_TX_SAMPLE_RATE,
	.pcm_format	 = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
};

static struct platform_device spdif_trans_dai = {
	.name	= "spdif-transciever",
	.id		= -1,
	.dev	= {
		.platform_data	= &spdif_trans_dai_data,
	}
};
#endif


#if defined(CONFIG_SND_SPDIF_RECEIVER) || defined(CONFIG_SND_SPDIF_RECEIVER_MODULE)
static struct platform_device spdif_receiver = {
	.name	= "spdif-dit-receiver",
	.id		= -1,
};

struct nxp_snd_dai_plat_data spdif_receiver_dai_data = {
	.sample_rate = CFG_AUDIO_SPDIF_RX_SAMPLE_RATE,
	.pcm_format	 = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
};

static struct platform_device spdif_receiver_dai = {
	.name	= "spdif-receiver",
	.id		= -1,
	.dev	= {
		.platform_data	= &spdif_receiver_dai_data,
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
 * PMIC platform device
 */
#if defined(CONFIG_REGULATOR_NXE2000)

#include <linux/i2c.h>
#include <nxe2000.h>
#include <linux/regulator/machine.h>

#define NXE2000_I2C_BUS		0
#define NXE2000_I2C_ADDR	0x64
#define NXE2000_I2C_LEN		1
#define NXE2000_IRQ			(PAD_GPIO_B + 26)


/* NXE2000 regulators */
static struct regulator_init_data __initdata nxp4330_ldo1_data = {
    .constraints    = {
        .name       = "VGPS_3.3V",
        .min_uV     = 3300000,
        .max_uV     = 3300000,
        .apply_uV   = 1,
//        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldo2_data = {
    .constraints    = {
        .name       = "VCAM1_1.8V",
        .min_uV     = 1800000,
        .max_uV     = 1800000,
        .apply_uV   = 1,
//        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldo3_data = {
    .constraints    = {
        .name       = "VSYS1_1.8V",
        .min_uV     = 1800000,
        .max_uV     = 1800000,
        .apply_uV   = 1,
        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldo4_data = {
    .constraints    = {
        .name       = "VSYS_1.8V",
        .min_uV     = 1800000,
        .max_uV     = 1800000,
        .apply_uV   = 1,
        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldo5_data = {
    .constraints    = {
        .name       = "VCAM_2.8V",
        .min_uV     = 2800000,
        .max_uV     = 2800000,
        .apply_uV   = 1,
//        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldo6_data = {
    .constraints    = {
        .name       = "VALIVE_3.3V",
        .min_uV     = 3300000,
        .max_uV     = 3300000,
        .apply_uV   = 1,
        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldo7_data = {
    .constraints    = {
        .name       = "VVID_2.8V",
        .min_uV     = 2800000,
        .max_uV     = 2800000,
        .apply_uV   = 1,
//        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldo8_data = {
    .constraints    = {
        .name       = "VWIFI_3.3V",
        .min_uV     = 3300000,
        .max_uV     = 3300000,
        .apply_uV   = 1,
//        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldo9_data = {
    .constraints    = {
        .name       = "VHUB_3.3V",
        .min_uV     = 3300000,
        .max_uV     = 3300000,
        .apply_uV   = 1,
//        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldo10_data = {
    .constraints    = {
        .name       = "VHSIC_1.2V",
        .min_uV     = 1200000,
        .max_uV     = 1200000,
        .apply_uV   = 1,
//        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldortc1_data = {
    .constraints    = {
        .name       = "VALIVE_1.8V",
        .min_uV     = 1800000,
        .max_uV     = 1800000,
        .apply_uV   = 1,
        .always_on  = 1,
    },
};

static struct regulator_init_data __initdata nxp4330_ldortc2_data = {
    .constraints    = {
        .name       = "VALIVE_1.0V",
        .min_uV     = 1000000,
        .max_uV     = 1000000,
        .apply_uV   = 1,
        .always_on  = 1,
    },
};

/* BUCK */
static struct regulator_consumer_supply __initdata nxe2000_buck1_csm[] = {
    REGULATOR_SUPPLY("V1.1_ARM", NULL),
};

static struct regulator_consumer_supply __initdata nxe2000_buck2_csm[] = {
    REGULATOR_SUPPLY("V1.0_CORE", NULL),
};

static struct regulator_consumer_supply __initdata nxe2000_buck3_csm[] = {
    REGULATOR_SUPPLY("V3.3_SYS", NULL),
};

static struct regulator_consumer_supply __initdata nxe2000_buck4_csm[] = {
    REGULATOR_SUPPLY("V1.5_DDR", NULL),
};

static struct regulator_consumer_supply __initdata nxe2000_buck5_csm[] = {
    REGULATOR_SUPPLY("V1.5_SYS", NULL),
};

static struct regulator_init_data __initdata nxp4330_buck1_data = {
    .constraints    = {
        .name       = "VARM_1.1V",
        .min_uV     = 1100000,
        .max_uV     = 1100000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
                      REGULATOR_CHANGE_STATUS,
        .apply_uV   = 1,
        .always_on  = 1,
        .state_mem  = {
            .enabled = 1,
        },
    },
    .num_consumer_supplies  = ARRAY_SIZE(nxe2000_buck1_csm),
    .consumer_supplies  = nxe2000_buck1_csm,
};

static struct regulator_init_data __initdata nxp4330_buck2_data = {
    .constraints    = {
        .name       = "VCORE_1.0V",
        .min_uV     = 1000000,
        .max_uV     = 1000000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
                      REGULATOR_CHANGE_STATUS,
        .apply_uV   = 1,
        .always_on  = 1,
        .state_mem  = {
            .enabled = 1,
        },
    },
    .num_consumer_supplies  = ARRAY_SIZE(nxe2000_buck2_csm),
    .consumer_supplies  = nxe2000_buck2_csm,
};

static struct regulator_init_data __initdata nxp4330_buck3_data = {
    .constraints    = {
        .name       = "VSYS_3.3V",
        .min_uV     = 3300000,
        .max_uV     = 3300000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
                      REGULATOR_CHANGE_STATUS,
        .apply_uV   = 1,
        .always_on  = 1,
        .state_mem  = {
            .enabled = 1,
        },
    },
    .num_consumer_supplies  = ARRAY_SIZE(nxe2000_buck3_csm),
    .consumer_supplies  = nxe2000_buck3_csm,
};

static struct regulator_init_data __initdata nxp4330_buck4_data = {
    .constraints    = {
        .name       = "VDDR_1.5V",
        .min_uV     = 1500000,
        .max_uV     = 1500000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
                      REGULATOR_CHANGE_STATUS,
        .apply_uV   = 1,
        .always_on  = 1,
        .state_mem  = {
            .enabled = 1,
        },
    },
    .num_consumer_supplies  = ARRAY_SIZE(nxe2000_buck4_csm),
    .consumer_supplies  = nxe2000_buck4_csm,
};

static struct regulator_init_data __initdata nxp4330_buck5_data = {
    .constraints    = {
        .name       = "VSYS_1.5V",
        .min_uV     = 1500000,
        .max_uV     = 1500000,
        .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
                      REGULATOR_CHANGE_STATUS,
        .apply_uV   = 1,
        .always_on  = 1,
        .state_mem  = {
            .enabled = 1,
        },
    },
    .num_consumer_supplies  = ARRAY_SIZE(nxe2000_buck5_csm),
    .consumer_supplies  = nxe2000_buck5_csm,
};

static struct nxe2000_regulator_data __initdata nxp4330_regulators[] = {
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

static struct nxe2000_pdata __initdata nxp4330_nxe2000_pdata = {
    .gpio_eint          = NXE2000_IRQ,
    .irq_base           = IRQ_SYSTEM_END,
    .wakeup             = 1,

    .num_regulators     = ARRAY_SIZE(nxp4330_regulators),
    .regulators         = nxp4330_regulators,

    .have_battery       = 0,
    .rdstate_periodic   = 1000,         /* milisecond */

    .buck1_set          = 0,            /* TODO */
    .buck2_set          = 0,            /* TODO */
    .buck_voltage[0]    = 1100000,      /* 1.1V */
    .buck_voltage[1]    = 1000000,      /* 1.0V */
    .buck_voltage[2]    = 3300000,      /* 3.3V */
    .buck_voltage[3]    = 1500000,      /* 1.5V */
    .buck_voltage[4]    = 1500000,      /* 1.5V */
    .batt_volt_max      = 4200000,      /* 4.2V */
    .batt_volt_min      = 3300000,      /* 3.3V */
    .batt_cap_level     = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL,

    .adp_ilim_current   = 2500000,      /* 2.5A */  /* Limit of total current   */
    .usb_ilim_current   = 1500000,      /* 1.5A */
    .chg_current        = 1000000,      /* 1.0A */
};

static struct i2c_board_info __initdata nxe2000_i2c_pmic_devs[] = {
    {
        I2C_BOARD_INFO("nxe2000", NXE2000_I2C_ADDR >> 1),
        .platform_data  = &nxp4330_nxe2000_pdata,
        .irq            = PB_PIO_IRQ(NXE2000_IRQ),
    },
};
#endif	/* CONFIG_REGULATOR_NXE2000  */

#if defined(CONFIG_REGULATOR_NXE1100)

#include <linux/i2c.h>
#include <nxe1100.h>
#include <linux/regulator/machine.h>

#define NXE1100_I2C_BUS		0
#define NXE1100_I2C_ADDR	0x64
#define NXE1100_I2C_LEN		1
#define NXE1100_IRQ			(PAD_GPIO_B + 26)


/* NXE1100 regulators */
static struct regulator_init_data __initdata nxp4330_ldo1_data = {
	.constraints	= {
		.name		= "VGPS_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
//		.always_on	= 1,
	},
};

static struct regulator_init_data __initdata nxp4330_ldo2_data = {
	.constraints	= {
		.name		= "VWIFI_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
//		.always_on	= 1,
	},
};

static struct regulator_init_data __initdata nxp4330_ldo3_data = {
	.constraints	= {
		.name		= "VCAM_2.8V",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.always_on	= 1,
	},
};

static struct regulator_init_data __initdata nxp4330_ldo4_data = {
	.constraints	= {
		.name		= "VHUB_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.always_on	= 1,
	},
};

static struct regulator_init_data __initdata nxp4330_ldo5_data = {
	.constraints	= {
		.name		= "VHSIC_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
	},
};

/* BUCK */
static struct regulator_consumer_supply __initdata nxe1100_buck1_csm[] = {
	REGULATOR_SUPPLY("vddarm", NULL),
};

static struct regulator_consumer_supply __initdata nxe1100_buck2_csm[] = {
	REGULATOR_SUPPLY("vddcore", NULL),
};

static struct regulator_init_data __initdata nxp4330_buck1_data = {
	.constraints	= {
		.name		= "VARM_1.1V",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(nxe1100_buck1_csm),
	.consumer_supplies	= nxe1100_buck1_csm,
};

static struct regulator_init_data __initdata nxp4330_buck2_data = {
	.constraints	= {
		.name		= "VCORE_1.0V",
		.min_uV		= 1000000,
		.max_uV		= 1000000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(nxe1100_buck2_csm),
	.consumer_supplies	= nxe1100_buck2_csm,
};

static struct regulator_init_data __initdata nxp4330_buck3_data = {
	.constraints	= {
		.name		= "VRAM_1.5V",
		.min_uV		= 1500000,
		.max_uV		= 1500000,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					  REGULATOR_CHANGE_STATUS,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.enabled = 1,
		},
	},
};

static struct nxe1100_regulator_data __initdata nxp4330_regulators[] = {
	{ NXE1100_LDO1,  &nxp4330_ldo1_data },
	{ NXE1100_LDO2,  &nxp4330_ldo2_data },
	{ NXE1100_LDO3,  &nxp4330_ldo3_data },
	{ NXE1100_LDO4,  &nxp4330_ldo4_data },
	{ NXE1100_LDO5,  &nxp4330_ldo5_data },
	{ NXE1100_BUCK1, &nxp4330_buck1_data },
	{ NXE1100_BUCK2, &nxp4330_buck2_data },
	{ NXE1100_BUCK3, &nxp4330_buck3_data },
};

static struct nxe1100_pdata __initdata nxp4330_nxe1100_pdata = {
	.gpio_eint			= NXE1100_IRQ,
	.irq_base			= IRQ_SYSTEM_END,
	.wakeup 			= 1,

	.num_regulators		= ARRAY_SIZE(nxp4330_regulators),
	.regulators			= nxp4330_regulators,

	.have_battery		= 0,
	.rdstate_periodic	= 1000,			/* milisecond */

	.buck1_set			= 0,			/* TODO */
	.buck2_set			= 0,			/* TODO */
	.buck_voltage[0]	= 1100000,		/* 1.1V */
	.buck_voltage[1]	= 1000000,		/* 1.0V */
	.buck_voltage[2]	= 1500000,		/* 1.5V */
	.batt_volt_max		= 4200000,		/* 4.2V */
	.batt_volt_min		= 3300000,		/* 3.3V */
	.batt_cap_level		= POWER_SUPPLY_CAPACITY_LEVEL_NORMAL,

	.adp_ilim_current	= 2500000,		/* 2.5A */  /* Limit of total current   */
	.usb_ilim_current	= 1500000,		/* 1.5A */
	.chg_current		= 1000000,		/* 1.0A */
};

static struct i2c_board_info __initdata nxe1100_i2c_pmic_devs[] = {
	{
		I2C_BOARD_INFO("nxe1100", NXE1100_I2C_ADDR >> 1),
		.platform_data	= &nxp4330_nxe1100_pdata,
		.irq			= PB_PIO_IRQ(NXE1100_IRQ),
	},
};
#endif	/* CONFIG_REGULATOR_NXE1100 */

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
	static bool enabled = false;

	printk("%s: %d, enabled=%d\n", __func__, on, enabled);
	if (on == enabled)
		return 0;

    if (on) {
    	printk("%s: Front Camera ON\n", __func__);
        /* reset signal */
    	gpio_set_value(FRONT_CAM_ENABLE_L, 0);
        msleep(10);
        gpio_set_value(FRONT_CAM_RESET_L, 1);
        msleep(10);
    } else {
    	printk("%s: Front Camera OFF\n", __func__);
        gpio_set_value(FRONT_CAM_RESET_L, 0);
        msleep(10);
    	gpio_set_value(FRONT_CAM_ENABLE_L, 1);
        msleep(10);
    }
    enabled = on;
    return 0;
}

static int camera_power_enable2(bool on)
{
	static bool enabled = false;

	printk("%s: %d, enabled=%d\n", __func__, on, enabled);
	if (on == enabled)
		return 0;

#if defined(CONFIG_PLAT_NXP4330_R3K)
    /* enable lines shared on some R3K boards */
    camera_power_enable(on);
#endif

    if (on) {
    	printk("%s: Rear Camera ON\n", __func__);
        /* reset signal */
    	gpio_set_value(REAR_CAM_ENABLE_L, 0);
        msleep(10);
        gpio_set_value(REAR_CAM_RESET_L, 1);
        msleep(10);
    } else {
    	printk("%s: Rear Camera OFF\n", __func__);
        gpio_set_value(REAR_CAM_RESET_L, 0);
        msleep(10);
    	gpio_set_value(REAR_CAM_ENABLE_L, 1);
        msleep(10);
    }
    enabled = on;
    return 0;
}

static int camera_set_clock(ulong clk_rate)
{
	static bool runonce = false;

	if (runonce)
		return 0;
	runonce = true;

    printk("%s: %ld\n", __func__, clk_rate);

    if (clk_rate > 0)
        nxp_soc_pwm_set_frequency(1, clk_rate, 50);
    else
        nxp_soc_pwm_set_frequency(1, 0, 0);
    msleep(1);
    return 0;
}

static int camera_set_clock2(ulong clk_rate)
{
    printk("%s: %ld\n", __func__, clk_rate);

    /* clock lines shared on R3K boards */
    camera_set_clock(clk_rate);
    return 0;
}

static void vin_setup_io(int module, bool on)
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

#if defined(CONFIG_NXP_CAPTURE_MIPI_CSI)

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
        .i2c_adapter_id = 2,
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
        .module = 0,
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
            .power_enable   = s5k4ecgx_power_enable,
            .set_clock      = s5k4ecgx_set_clock,
            .setup_io       = mipi_vin_setup_io,
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
 * DW MMC board config
 */
#if defined(CONFIG_MMC_DW)
#include <linux/mmc/dw_mmc.h>

int _dwmci_ext_cd_init(void (*notify_func)(struct platform_device *, int state))
{
	return 0;
}

int _dwmci_ext_cd_cleanup(void (*notify_func)(struct platform_device *, int state))
{
	return 0;
}

static int _dwmci_get_ro(u32 slot_id)
{
	return 0;
}

#ifdef CONFIG_MMC_NEXELL_CH0
static int _dwmci0_init(u32 slot_id, irq_handler_t handler, void *data)
{
#ifdef CONFIG_PLAT_NXP4330_VTK
	struct dw_mci *host = (struct dw_mci *)data;
	int io  = CFG_SDMMC0_DETECT_IO;
	int irq = IRQ_GPIO_START + io;
	int id  = 0, ret = 0;

	printk("dw_mmc dw_mmc.%d: Using external card detect irq %3d (io %2d)\n", id, irq, io);

	ret  = request_irq(irq, handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				DEV_NAME_SDHC "0", (void*)host->slot[slot_id]);
	if (0 > ret)
		pr_err("dw_mmc dw_mmc.%d: fail request interrupt %d ...\n", id, irq);
#endif	/* CONFIG_PLAT_NXP4330_VTK */
	return 0;
}

static int _dwmci0_get_cd(u32 slot_id)
{
#ifdef CONFIG_PLAT_NXP4330_VTK
	int io = CFG_SDMMC0_DETECT_IO;
	int ret = nxp_soc_gpio_get_in_value(io);
	return ret;
#else
	return 0;
#endif
}

static struct dw_mci_board _dwmci0_data = {
	.quirks		= DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz		= 100 * 1000 * 1000,
	.caps		= MMC_CAP_CMD23 | MMC_CAP_4_BIT_DATA | MMC_CAP_NONREMOVABLE,
	.detect_delay_ms= 200,
//	.sdr_timing	= 0x03020001,
//	.ddr_timing	= 0x03030002,
	.cd_type	= DW_MCI_CD_EXTERNAL,
	.init		= _dwmci0_init,
	.get_ro		= _dwmci_get_ro,
	.get_cd		= _dwmci0_get_cd,
	.ext_cd_init	= _dwmci_ext_cd_init,
	.ext_cd_cleanup	= _dwmci_ext_cd_cleanup,
};
#endif

#ifdef CONFIG_MMC_NEXELL_CH1
int glasgow_cdetect_enabled = 1;
static int __init get_cmdline_cdetect(char *str)
{
  int val;

  if ( get_option(&str, &val) ) {
    glasgow_cdetect_enabled = val;
    return 0;
  }

  return -EINVAL;
}
early_param("cdetect", get_cmdline_cdetect);

static int _dwmci1_init(u32 slot_id, irq_handler_t handler, void *data)
{
#if defined(CONFIG_PLAT_NXP4330_GLASGOW) && !defined(CONFIG_AIO) && defined(CONFIG_MMC_NEXELL_CH0_CDETECT)
	struct dw_mci *host = (struct dw_mci *)data;
	int io  = CFG_SDMMC0_DETECT_IO;
	int irq = IRQ_GPIO_START + io;
	int id  = 1, ret = 0;

	/*
	 * Check Kernel cmdline option "cdetect" [ see get_cmdline_cdetect() ]
	 *  if option = 0, then card detect disabled -- in SD boot mode
	 *  if option = 1, then card detect enabled  -- in eMMC boot mode
	 */
	switch ( glasgow_cdetect_enabled ) {
	  case 0:
	    printk(KERN_WARNING "dw_mmc dw_mmc.%d: External card detect DISABLED!\n", id);
	    break;

	  case 1:
      printk(KERN_WARNING "dw_mmc dw_mmc.%d: External card detect ENABLED!\n", id);
	    /* fall through */

	  default:
	    printk(KERN_WARNING "dw_mmc dw_mmc.%d: Using external card detect irq %3d (io %2d)\n", id, irq, io);
      ret  = request_irq(irq, handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
            DEV_NAME_SDHC "1", (void*)host->slot[slot_id]);
      if (0 > ret)
        pr_err("dw_mmc dw_mmc.%d: fail request interrupt %d ...\n", id, irq);
	}
#endif
	return 0;
}

static int _dwmci1_get_cd(u32 slot_id)
{
#if defined(CONFIG_PLAT_NXP4330_GLASGOW) && !defined(CONFIG_AIO) && defined(CONFIG_MMC_NEXELL_CH0_CDETECT)
	int io = CFG_SDMMC0_DETECT_IO;
	int ret;

	if ( !glasgow_cdetect_enabled )
	  return 0;

	ret = nxp_soc_gpio_get_in_value(io);
  return !ret;
#else
	return 0;
#endif
}

static struct dw_mci_board _dwmci1_data = {
	.quirks		= DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz		= 100 * 1000 * 1000,
	.caps		= MMC_CAP_CMD23 | MMC_CAP_4_BIT_DATA,
	.detect_delay_ms= 200,
//	.sdr_timing	= 0x03020001,
//	.ddr_timing	= 0x03030002,
	.cd_type	= DW_MCI_CD_EXTERNAL,
	.init		= _dwmci1_init,
	.get_ro		= _dwmci_get_ro,
	.get_cd		= _dwmci1_get_cd,
	.ext_cd_init	= _dwmci_ext_cd_init,
	.ext_cd_cleanup	= _dwmci_ext_cd_cleanup,
};
#endif

#ifdef CONFIG_MMC_NEXELL_CH2
static int _dwmci2_init(u32 slot_id, irq_handler_t handler, void *data)
{
#ifdef CONFIG_PLAT_NXP4330_VTK
	struct dw_mci *host = (struct dw_mci *)data;
	int io  = CFG_SDMMC0_DETECT_IO;
	int irq = IRQ_GPIO_START + io;
	int id  = 0, ret = 0;

	printk("dw_mmc dw_mmc.%d: Using external card detect irq %3d (io %2d)\n", id, irq, io);

	ret  = request_irq(irq, handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				DEV_NAME_SDHC "2", (void*)host->slot[slot_id]);
	if (0 > ret)
		pr_err("dw_mmc dw_mmc.%d: fail request interrupt %d ...\n", id, irq);
#endif	/* CONFIG_PLAT_NXP4330_VTK */
	return 0;
}

static int _dwmci2_get_cd(u32 slot_id)
{
#ifdef CONFIG_PLAT_NXP4330_VTK
	int io = CFG_SDMMC0_DETECT_IO;
	int ret = nxp_soc_gpio_get_in_value(io);
	return ret;
#else
	return 0;
#endif
}

static struct dw_mci_board _dwmci2_data = {
	.quirks		= DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz		= 100 * 1000 * 1000,
#ifndef CONFIG_PLAT_NXP4330_VTK
	.caps		= MMC_CAP_CMD23 | MMC_CAP_4_BIT_DATA 
					| MMC_CAP_NONREMOVABLE
			  		| MMC_CAP_POWER_OFF_CARD,

#else	/* since VTK doesn't have eMMC, don't specify DDR */
	.caps		= MMC_CAP_CMD23 | MMC_CAP_4_BIT_DATA,
#endif
	.detect_delay_ms= 200,
//	.sdr_timing	= 0x03020001,
//	.ddr_timing	= 0x03030002,
	.cd_type	= DW_MCI_CD_EXTERNAL,
	.init		= _dwmci2_init,
	.get_ro		= _dwmci_get_ro,
	.get_cd		= _dwmci2_get_cd,
	.ext_cd_init	= _dwmci_ext_cd_init,
	.ext_cd_cleanup	= _dwmci_ext_cd_cleanup,
};
#endif

#endif /* CONFIG_MMC_DW */

/*------------------------------------------------------------------------------
 * TI WIFI module
 */
#if defined(CONFIG_WL12XX_PLATFORM_DATA)

struct wl12xx_platform_data nxp4430_wlan_data;

static void wl_set_power(u32 slot_id, u32 on)
{
	if (on) {
		printk("WIFI_RESET: ON\n");
		gpio_set_value(WIFI_RESET, 1);
		mdelay(70);
	}
	else
	{
		printk("WIFI_RESET: OFF\n");
		gpio_set_value(WIFI_RESET, 0);
	}
}

static void wl12xx_init()
{
	struct device *dev;
	struct omap_mmc_platform_data *pdata;

	nxp4430_wlan_data.irq = gpio_to_irq(WIFI_HOST_WAKE);
	nxp4430_wlan_data.platform_quirks = WL12XX_PLATFORM_QUIRK_EDGE_IRQ;
	if(wl12xx_set_platform_data(&nxp4430_wlan_data))
		pr_err("error setting wl12xx data\n");

	_dwmci2_data.setpower = wl_set_power;


	return;
}

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
 * GPIO Button driver
 */
#if defined(CONFIG_KEYBOARD_GPIO)
extern struct platform_device lf3000_gpio_buttons;
#endif

/*------------------------------------------------------------------------------
 * ASoC (LFP100) platform device
 */
#if defined(CONFIG_SOC_LFP100)
static struct platform_device lfp100_plat_device = {
	.name	= "lfp100-chip",
	.id		= -1,
};
#endif

#if defined(CONFIG_TC7734_PMIC)
static struct platform_device tc7734_plat_device = {
        .name   = "tc7734-chip",
        .id             = -1,
};
#endif

#if defined(CONFIG_SND_CODEC_LFP100) || defined(CONFIG_SND_CODEC_LFP100_MODULE)

static struct i2c_board_info lfp100_i2c_codec_bdi = {
	.type	= "lfp100",
	.addr	= LFP100_ADDR,
};

/* DAI */
struct nxp_snd_dai_plat_data lfp100_i2s_dai_data = {
	.i2s_ch		= 0,
	.sample_rate	= 32000,
};

static struct platform_device lfp100_asoc_device = {
	.name	= "lfp100-asoc",
	.id		= -1,
	.dev	= {
		.platform_data	= &lfp100_i2s_dai_data,
	}
};
#endif

#if defined(CONFIG_SND_CODEC_TC94B26) || defined(CONFIG_SND_CODEC_TC94B26_MODULE)

#define TC94B26_ADDR	0x1A

static struct i2c_board_info tc94b26_i2c_codec_bdi = {
	.type	= "tc94b26",
	.addr	= TC94B26_ADDR,
};

/* DAI */
struct nxp_snd_dai_plat_data tc94b26_i2s_dai_data = {
	.i2s_ch		= 0,
	.sample_rate	= 32000,
};

static struct platform_device tc94b26_asoc_device = {
	.name	= "nxp-tc94b26-asoc",
	.id		= -1,
	.dev	= {
		.platform_data	= &tc94b26_i2s_dai_data,
	}
};
#endif

#if defined(CONFIG_SENSORS_LF3000)
static struct platform_device power_plat_device = {
	.name	= DEV_NAME_POWER,
	.id		= -1,
};
#endif

#if defined(CONFIG_SENSORS_GLASGOW)
static struct platform_device glasgow_power_plat_device = {
	.name	= "glasgow-power",
	.id		= -1,
};
#endif

#if defined(CONFIG_BACKLIGHT_LF3000)
static struct platform_device backlight_plat_device = {
	.name	= DEV_NAME_LF_BL,
	.id		= -1,
};
#endif


/*------------------------------------------------------------------------------
 * Performance Monitoring Unit platform device
 */
#if defined(CONFIG_HW_PERF_EVENTS)
static struct resource lf3000_pmu_resources[] = {
	{
		.start	= IRQ_GIC_PMUIRQ0,
		.end	= IRQ_GIC_PMUIRQ0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= IRQ_GIC_PMUIRQ1,
		.end	= IRQ_GIC_PMUIRQ1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= IRQ_GIC_PMUIRQ2,
		.end	= IRQ_GIC_PMUIRQ2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= IRQ_GIC_PMUIRQ3,
		.end	= IRQ_GIC_PMUIRQ3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device lf3000_pmu_device = {
		.name		= "arm-pmu",
		.id			= ARM_PMU_DEVICE_CPU,
		.resource	= lf3000_pmu_resources,
		.num_resources	= ARRAY_SIZE(lf3000_pmu_resources),
};
#endif

/*------------------------------------------------------------------------------
 * Acclerometer platform device
 */
#if defined(CONFIG_INPUT_LF2000_ACLMTR)
static struct platform_device aclmtr_plat_device = {
	.name	= DEV_NAME_LF_ACLMTR,
	.id		= -1,
};
#endif

extern int lfp100_dump_registers(void);

/* define routine only if using supported PMIC */
#if defined(CONFIG_SOC_LFP100) || defined(CONFIG_TC7734_PMIC)
void lf3000_poweroff(void)
{

#if defined(CONFIG_POWER_DOWN_OFF)
	printk(KERN_ALERT "Power Down Off.\n");

#if defined(CONFIG_SOC_LFP100)
		lfp100_set_power_off();
#elif defined (CONFIG_TC7734_PMIC)
		tc7734_set_power_off();
#else
#error Shutdown not defined
#endif

#elif defined(CONFIG_POWER_DOWN_STANDBY)
		printk(KERN_ALERT "Power Down Standby.\n");

#if defined(CONFIG_SOC_LFP100)
		lfp100_set_power_standby();
#elif defined (CONFIG_TC7734_PMIC)
		tc7734_set_power_standby();
#else
#error Shutdown not defined
#endif

#else
#error Expected a CONFIG_POWER_DOWN_xxx configuration
#endif

}
EXPORT_SYMBOL(lf3000_poweroff);
#endif

/*------------------------------------------------------------------------------
 * register board platform devices
 */
void __init nxp_board_devices_register(void)
{
	printk("[Register board platform devices]\n");

#if defined(CONFIG_ARM_NXP4330_CPUFREQ)
	printk("plat: add dynamic frequency (pll.%d)\n", dfs_plat_data.pll_dev);
	platform_device_register(&dfs_plat_device);
	//platform_device_register(&freq_limit_device);
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
	#ifdef CONFIG_MMC_NEXELL_CH2
	nxp_mmc_add_device(2, &_dwmci2_data);
	#endif
#endif

#if defined(CONFIG_DM9000) || defined(CONFIG_DM9000_MODULE)
	printk("plat: add device dm9000 net\n");
	platform_device_register(&dm9000_plat_device);
#endif

#if defined(CONFIG_SND_SPDIF_TRANSCIEVER) || defined(CONFIG_SND_SPDIF_TRANSCIEVER_MODULE)
	printk("plat: add device spdif playback\n");
	platform_device_register(&spdif_transciever);
	platform_device_register(&spdif_trans_dai);
#endif

#if defined(CONFIG_HW_PERF_EVENTS)
	printk("plat: add device arm-pmu\n");
	platform_device_register(&lf3000_pmu_device);
#endif

#if defined(CONFIG_SND_SPDIF_RECEIVER) || defined(CONFIG_SND_SPDIF_RECEIVER_MODULE)
	printk("plat: add device spdif capture\n");
	platform_device_register(&spdif_receiver);
	platform_device_register(&spdif_receiver_dai);
#endif

#if defined(CONFIG_SND_CODEC_RT5631) || defined(CONFIG_SND_CODEC_RT5631_MODULE)
	printk("plat: add device asoc-rt5631\n");
	i2c_register_board_info(RT5631_I2C_BUS, &rt5631_i2c_bdi, 1);
	platform_device_register(&rt5631_dai);
#endif

#if defined(CONFIG_BACKLIGHT_PWM)
	printk("plat: add backlight pwm device\n");
	platform_device_register(&bl_plat_device);
#endif

#if 0	// FIXME: sesters, added in Linux 3.4.39
#if defined(CONFIG_MTD_NAND_NEXELL)
	platform_device_register(&nand_plat_device);
#endif
#endif

#if defined(CONFIG_KEYBOARD_NEXELL_KEY) || defined(CONFIG_KEYBOARD_NEXELL_KEY_MODULE)
	printk("plat: add device keypad\n");
	platform_device_register(&key_plat_device);
#endif

#if defined(CONFIG_INPUT_LF2000_ACLMTR) || defined(CONFIG_INPUT_LF2000_ACLMTR_MODULE)
	printk("plat: add device aclmtr\n");
	platform_device_register(&aclmtr_plat_device);
#endif

#if defined(CONFIG_SOC_LFP100)
	printk("plat: register lfp100 chip\n");
	platform_device_register(&lfp100_plat_device);
#endif

#if defined(CONFIG_TC7734_PMIC)
	printk("plat: register tc7734 chip\n");
	platform_device_register(&tc7734_plat_device);
#endif

#if defined(CONFIG_SND_CODEC_LFP100) || defined(CONFIG_SND_CODEC_LFP100_MODULE)
	printk("plat: register asoc-lfp100\n");
	i2c_register_board_info(LFP100_I2C_ADAPTER_0, &lfp100_i2c_codec_bdi, 1);
	platform_device_register(&lfp100_asoc_device);
#endif

#if defined(CONFIG_SND_CODEC_TC94B26) || defined(CONFIG_SND_CODEC_TC94B26_MODULE)
#define TC94B26_I2C_ADAPTER_0	0
#define TC94B26_I2C_ADAPTER_1	1

	printk("plat: register asoc-tc94b26\n");
	i2c_register_board_info(TC94B26_I2C_ADAPTER_0, &tc94b26_i2c_codec_bdi, 1);
	platform_device_register(&tc94b26_asoc_device);
#endif

#if defined(CONFIG_SENSORS_LF3000)
	printk("plat: register power monitor\n");
	platform_device_register(&power_plat_device);
#endif

#if defined(CONFIG_SENSORS_GLASGOW)
	printk("plat: register Glasgow power monitor\n");
	platform_device_register(&glasgow_power_plat_device);
#endif

#if defined(CONFIG_BACKLIGHT_LF3000)
	printk("plat: register backlight\n");
	platform_device_register(&backlight_plat_device);
#endif

#if defined(CONFIG_BACKLIGHT_PWM)
	printk("plat: add backlight pwm device\n");
	platform_device_register(&bl_plat_device);
#endif

#if defined(CONFIG_TOUCHSCREEN_FT5X0X)
	printk("plat: add touch(ftx0x) device\n");
	i2c_register_board_info(FT5X0X_I2C_BUS, &ft5x0x_i2c_bdi, 1);
#endif

#if defined(CONFIG_TOUCHSCREEN_ICN83xx_IIC) || defined (CONFIG_TOUCHSCREEN_ICN83xx_IIC_MODULE)
	printk("plat: add touch(ICN83XX_IIC) device\n");
	i2c_register_board_info(ICN83XX_I2C_BUS, icn83xx_i2c_info, ARRAY_SIZE(icn83xx_i2c_info));
#endif

#if defined(CONFIG_TOUCHSCREEN_ICN85xx_IIC) || defined (CONFIG_TOUCHSCREEN_ICN85xx_IIC_MODULE)
	printk("plat: add touch(ICN85XX_IIC) device\n");
	i2c_register_board_info(ICN85XX_I2C_BUS, icn85xx_i2c_info, ARRAY_SIZE(icn85xx_i2c_info));
#endif

#if defined(CONFIG_TOUCHSCREEN_PAP11XX) || defined(CONFIG_TOUCHSCREEN_PAP11XX_MODULE)
	printk("plat: add touch(pap11xx) device\n");
	i2c_register_board_info(CTP_I2C_BUS, &pap11xx_i2c_bdi, 1);
#endif

#if defined(CONFIG_TOUCHSCREEN_ELAN_KTF2K) || defined(CONFIG_TOUCHSCREEN_ELAN_KTF2K_MODULE)
	printk("plat: add touch(ELAN etkf2k) device\n");
	i2c_register_board_info(CTP_I2C_BUS, &elan_i2c_devices[0], 1);
#endif

#if defined(CONFIG_TOUCHSCREEN_ELAN_KTF2K_7INCH) || defined(CONFIG_TOUCHSCREEN_ELAN_KTF2K_7INCH_MODULE)
	printk("plat: add touch(ELAN etkf2k7) device\n");
	i2c_register_board_info(CTP_I2C_BUS, &elan_i2c_devices7[0], 1);
#endif

#if defined(CONFIG_TOUCHSCREEN_BCD_AP386X) || defined(CONFIG_TOUCHSCREEN_BCD_AP386X_MODULE)
	printk("plat: add touch(BCD tu_drv) device\n");
	i2c_register_board_info(CTP_I2C_BUS, &bcd_i2c_device, 1);
#endif

#if defined(CONFIG_TOUCHSCREEN_MStar_MSG21xxA) || defined(CONFIG_TOUCHSCREEN_MStar_MSG21xxA_MODULE)
	printk("plat: add touch(MStar) device\n");
	i2c_register_board_info(CTP_I2C_BUS, &mstar_i2c_device, 1);
#endif

#if defined(CONFIG_TOUCHSCREEN_Ilitek_2839) || defined(CONFIG_TOUCHSCREEN_Ilitek_2839_MODULE)
	printk("plat: add touch(Ilitek) device\n");
	i2c_register_board_info(CTP_I2C_BUS, &ilitek_i2c_device, 1);
#endif

#if defined(CONFIG_TOUCHSCREEN_Ilitek_2839_7INCH) || defined(CONFIG_TOUCHSCREEN_Ilitek_2839_7INCH_MODULE)
	printk("plat: add touch(Ilitek7) device\n");
	i2c_register_board_info(CTP_I2C_BUS, &ilitek7_i2c_device, 1);
#endif

#if defined(CONFIG_TOUCHSCREEN_Goodix_GT967_7INCH) || defined(CONFIG_TOUCHSCREEN_Goodix_GT967_7INCH_MODULE)
	printk("plat: add touch(Goodix) device\n");
	i2c_register_board_info(CTP_I2C_BUS, &GT967_i2c_device, 1);
#endif


#if defined(CONFIG_LEDS_GPIO)
	printk("plat: add heatbeat device\n");
	platform_device_register(&gpio_leds_device);
#endif


#if defined(CONFIG_REGULATOR_NXE2000)
	printk("plat: add device nxe2000 pmic\n");
	i2c_register_board_info(NXE2000_I2C_BUS, nxe2000_i2c_pmic_devs, ARRAY_SIZE(nxe2000_i2c_pmic_devs));
#endif

#if defined(CONFIG_REGULATOR_NXE1100)
	printk("plat: add device nxe1100 pmic\n");
	i2c_register_board_info(NXE1100_I2C_BUS, nxe1100_i2c_pmic_devs, ARRAY_SIZE(nxe1100_i2c_pmic_devs));
#endif

#if defined(CONFIG_V4L2_NEXELL) || defined(CONFIG_V4L2_NEXELL_MODULE)
    printk("plat: add device nxp-v4l2\n");
    platform_device_register(&nxp_v4l2_dev);
#endif

#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
    spi_register_board_info(spi_plat_board, ARRAY_SIZE(spi_plat_board));
    printk("plat: register spidev\n");
#endif

#if defined(CONFIG_V4L2_LF2000) || defined(CONFIG_V4L2_LF2000_MODULE)
    printk("plat: add device lf2k_camera\n");
    platform_device_register(&lf2k_camera[0]);
    platform_device_register(&lf2k_camera[1]);
    platform_device_register(&vip_plat_device);
#endif

#if defined(CONFIG_KEYBOARD_GPIO)
	printk("plat: add device gpio_keyboard\n");
	platform_device_register(&lf3000_gpio_buttons);
#endif

#if defined(CONFIG_MTD_NAND)
	printk("plat: add device %s\n", DEV_NAME_LF_NAND);
	platform_device_register(&lf2000_nand_plat_device);
#endif

#ifdef CONFIG_BQ24250_CHARGER

     	printk("\nplat: register bq24250 chip\n");
      	i2c_register_board_info(BQ24250_I2C_BUS, &bq24250_i2c_device, 1);
#endif

#if defined(CONFIG_WL12XX_PLATFORM_DATA)
	printk("plat: add device wl12xx\n");
	wl12xx_init();
#endif

	/* END */
	printk("\n");
}
