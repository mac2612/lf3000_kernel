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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/irq.h>
#include <linux/amba/pl022.h>

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
//	{ 1600000, 1200 },
//	{ 1500000, 1200 },
//	{ 1400000, 1200 },
//	{ 1300000, 1200 },
//	{ 1200000, 1200 },
//	{ 1100000, 1200 },
//	{ 1000000, 1200 },
//	{  900000, 1200 },
	{  800000, 1200 },
#if 1
	{  780000, 1200 },
	{  562000, 1200 },
	{  533000, 1200 },
	{  490000, 1200 },
	{  400000, 1200 },
#endif
};

struct nxp_cpufreq_plat_data dfs_plat_data = {
	.pll_dev	   	= CONFIG_NXP4330_CPUFREQ_PLLDEV,
	.freq_table	   	= dfs_freq_table,
	.table_size	   	= ARRAY_SIZE(dfs_freq_table),
	.max_cpufreq    = 800*1000,
	.max_retention  =   30*1000,
	.rest_cpufreq   =  400*1000,
	.rest_retention =    1*1000,
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
 * DW GMAC board config
 */
#if defined(CONFIG_NXPMAC_ETH)
#include <linux/phy.h>
#include <linux/nxpmac.h>
#include <linux/delay.h>
#include <linux/gpio.h>
int  nxpmac_init(struct platform_device *pdev)
{
    u32 addr;
  
#if defined (CONFIG_REALTEK_PHY_RTL8201)	// 20140515
	// 100 & 10Base-T
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+ 7, 0 );        // PAD_GPIOE7,     GMAC0_PHY_TXD[0]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+ 8, 0 );        // PAD_GPIOE8,     GMAC0_PHY_TXD[1]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+ 9, 0 );        // PAD_GPIOE9,     GMAC0_PHY_TXD[2]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+10, 0 );        // PAD_GPIOE10,    GMAC0_PHY_TXD[3]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+11, 0 );        // PAD_GPIOE11,    GMAC0_PHY_TXEN
//  nxp_soc_gpio_set_io_drv( PAD_GPIO_E+12, 3 );        // PAD_GPIOE12,    GMAC0_PHY_TXER
//  nxp_soc_gpio_set_io_drv( PAD_GPIO_E+13, 3 );        // PAD_GPIOE13,    GMAC0_PHY_COL
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+14, 2 );        // PAD_GPIOE14,    GMAC0_PHY_RXD[0]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+15, 2 );        // PAD_GPIOE15,    GMAC0_PHY_RXD[1]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+16, 2 );        // PAD_GPIOE16,    GMAC0_PHY_RXD[2]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+17, 2 );        // PAD_GPIOE17,    GMAC0_PHY_RXD[3]
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+18, 3 );        // PAD_GPIOE18,    GMAC0_RX_CLK
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+19, 3);        // PAD_GPIOE19,    GMAC0_PHY_RX_DV
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+20, 3 );        // PAD_GPIOE20,    GMAC0_GMII_MDC
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+21, 3 );        // PAD_GPIOE21,    GMAC0_GMII_MDI
//  nxp_soc_gpio_set_io_drv( PAD_GPIO_E+22, 3 );        // PAD_GPIOE22,    GMAC0_PHY_RXER
//  nxp_soc_gpio_set_io_drv( PAD_GPIO_E+23, 3 );        // PAD_GPIOE23,    GMAC0_PHY_CRS
    nxp_soc_gpio_set_io_drv( PAD_GPIO_E+24, 0 );        // PAD_GPIOE24,    GMAC0_GTX_CLK

#else	// 1000Base-T
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E +  7), 3 );     // PAD_GPIOE7,     GMAC0_PHY_TXD[0]
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E +  8), 3 );     // PAD_GPIOE8,     GMAC0_PHY_TXD[1]
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E +  9), 3 );     // PAD_GPIOE9,     GMAC0_PHY_TXD[2]
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 10), 3 );     // PAD_GPIOE10,    GMAC0_PHY_TXD[3]
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 11), 3 );     // PAD_GPIOE11,    GMAC0_PHY_TXEN
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 14), 3 );     // PAD_GPIOE14,    GMAC0_PHY_RXD[0]
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 15), 3 );     // PAD_GPIOE15,    GMAC0_PHY_RXD[1]
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 16), 3 );     // PAD_GPIOE16,    GMAC0_PHY_RXD[2]
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 17), 3 );     // PAD_GPIOE17,    GMAC0_PHY_RXD[3]
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 18), 3 );     // PAD_GPIOE18,    GMAC0_RX_CLK
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 19), 3 );     // PAD_GPIOE19,    GMAC0_PHY_RX_DV
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 20), 3 );     // PAD_GPIOE20,    GMAC0_GMII_MDC
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 21), 3 );     // PAD_GPIOE21,    GMAC0_GMII_MDI
	nxp_soc_gpio_set_io_drv( (PAD_GPIO_E + 24), 3 );     // PAD_GPIOE24,    GMAC0_GTX_CLK
#endif


	// Clock control
	NX_CLKGEN_Initialize();
	addr = NX_CLKGEN_GetPhysicalAddress(CLOCKINDEX_OF_DWC_GMAC_MODULE);
	NX_CLKGEN_SetBaseAddress( CLOCKINDEX_OF_DWC_GMAC_MODULE, (u32)IO_ADDRESS(addr) );

	NX_CLKGEN_SetClockSource( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, 4);     // Sync mode for 100 & 10Base-T : External RX_clk
	NX_CLKGEN_SetClockDivisor( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, 1);    // Sync mode for 100 & 10Base-T

	NX_CLKGEN_SetClockOutInv( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, CFALSE);    // TX Clk invert off : 100 & 10Base-T
//	NX_CLKGEN_SetClockOutInv( CLOCKINDEX_OF_DWC_GMAC_MODULE, 0, CTRUE);     // TX clk invert on : 100 & 10Base-T

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


    gpio_request(CFG_ETHER_GMAC_PHY_RST_NUM,"Ethernet Rst pin");
	gpio_direction_output(CFG_ETHER_GMAC_PHY_RST_NUM, 1 );
	udelay( 100 );
	gpio_set_value(CFG_ETHER_GMAC_PHY_RST_NUM, 0 );
	udelay( 100 );
	gpio_set_value(CFG_ETHER_GMAC_PHY_RST_NUM, 1 );

    gpio_free(CFG_ETHER_GMAC_PHY_RST_NUM);

     printk("NXP mac init ..................\n");
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
#if defined (CONFIG_REALTEK_PHY_RTL8201)
	.phy_addr = 3,  // 7 for 8211 3 for 8201
    .clk_csr = 0x25, 
    .speed = SPEED_100,
#else
    .phy_addr = 7,// 7 for 8211 3 for 8201
    .clk_csr = 0x28,
    .speed = SPEED_1000,
#endif
    .interface = PHY_INTERFACE_MODE_RGMII,
	.autoneg = AUTONEG_ENABLE, //AUTONEG_ENABLE or AUTONEG_DISABLE
	.duplex = DUPLEX_FULL,
	.pbl = 16,          /* burst 16 */
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
#if defined(CONFIG_SND_CODEC_WM8976) || defined(CONFIG_SND_CODEC_WM8976_MODULE)
#include <linux/i2c.h>

#define	WM8976_I2C_BUS		(0)

/* CODEC */
static struct i2c_board_info __initdata wm8976_i2c_bdi = {
	.type	= "wm8978",			// compatilbe with wm8976
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

static struct platform_device wm8976_dai = {
	.name			= "wm8976-audio",
	.id				= 0,
	.dev			= {
		.platform_data	= &i2s_dai_data,
	}
};
#endif

#if defined(CONFIG_SND_CODEC_ALC5623)
#include <linux/i2c.h>

#define	WM8976_I2C_BUS		(0)

/* CODEC */
static struct i2c_board_info __initdata alc5623_i2c_bdi = {
	.type	= "alc562x-codec",			// compatilbe with wm8976
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

static struct platform_device alc5623_dai = {
	.name			= "alc5623-audio",
	.id				= 0,
	.dev			= {
		.platform_data	= &i2s_dai_data,
	}
};
#endif

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
 * v4l2 platform device
 */
#if defined(CONFIG_V4L2_NEXELL) || defined(CONFIG_V4L2_NEXELL_MODULE)
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <mach/nxp-v4l2-platformdata.h>
#include <mach/soc.h>

static int camera_common_set_clock(ulong clk_rate)
{
    PM_DBGOUT("%s: %d\n", __func__, (int)clk_rate);
    if (clk_rate > 0)
        nxp_soc_pwm_set_frequency(1, clk_rate, 50);
    else
        nxp_soc_pwm_set_frequency(1, 0, 0);
    msleep(1);
    return 0;
}

static bool is_camera_port_configured = false;
static void camera_common_vin_setup_io(int module, bool force)
{
    if (!force && is_camera_port_configured)
        return;
    else {
        u_int *pad;
        int i, len;
        u_int io, fn;


        /* VIP0:0 = VCLK, VID0 ~ 7 */
        const u_int port[][2] = {
            /* VCLK, HSYNC, VSYNC */
            { PAD_GPIO_E +  4, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_E +  5, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_E +  6, NX_GPIO_PADFUNC_1 },
            /* DATA */
            { PAD_GPIO_D + 28, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_D + 29, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_D + 30, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_D + 31, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_E +  0, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_E +  1, NX_GPIO_PADFUNC_1 },
            { PAD_GPIO_E +  2, NX_GPIO_PADFUNC_1 }, { PAD_GPIO_E +  3, NX_GPIO_PADFUNC_1 },
        };

        printk("%s\n", __func__);

        pad = (u_int *)port;
        len = sizeof(port)/sizeof(port[0]);

        for (i = 0; i < len; i++) {
            io = *pad++;
            fn = *pad++;
            nxp_soc_gpio_set_io_dir(io, 0);
            nxp_soc_gpio_set_io_func(io, fn);
        }

        is_camera_port_configured = true;
    }
}

static bool is_back_camera_enabled = false;
static bool is_back_camera_power_state_changed = false;
static bool is_front_camera_enabled = false;
static bool is_front_camera_power_state_changed = false;

#include <linux/gpio.h>
 static int front_camera_power_enable(bool on)
 {
 	unsigned int io = CFG_IO_CAMERA_FRONT_POWER_DOWN;
 	unsigned int reset_io = CFG_IO_CAMERA_RESET;
 	int err = 0;
 	printk("%s: is_front_camera_enabled %d, on %d\n", __func__, is_front_camera_enabled, on);

    err = gpio_request(io, "CAMERA_PN");
    if (err) {
        printk("error CAMERA_PN pin.%d\n", io);

    }
    err = gpio_request(reset_io, "CAMERA_reset");
    if (err) {
        printk("error CAMERA_reset pin.%d\n", reset_io);
    }
 	if (on) {
 		if (!is_front_camera_enabled) {

            gpio_direction_output(reset_io, 0);
            gpio_direction_output(io, 0);
            mdelay(1);
     		gpio_direction_output(io, 1);
            mdelay(30);
 			camera_common_set_clock(24000000);
 			mdelay(10);
 			gpio_direction_output(io, 0);
 			mdelay(10);
 			gpio_direction_output(reset_io, 1);

 			mdelay(5);


 			is_front_camera_enabled = true;
 			is_front_camera_power_state_changed = true;
 		}
        else
        {
 			is_front_camera_power_state_changed = false;
 		}
 	}
 	else
 	{
 		if (is_front_camera_enabled) {
            gpio_direction_output(io, 1);
            is_front_camera_enabled = false;
            is_front_camera_power_state_changed = true;
        } else {
            gpio_direction_output(io, 1);
            is_front_camera_power_state_changed = false;
        }
    }
    gpio_free(io);
    gpio_free(reset_io);
    return 0;
}

static bool front_camera_power_state_changed(void)
{
    return is_front_camera_power_state_changed;
}

static struct i2c_board_info front_camera_i2c_boardinfo[] = {
    {
        I2C_BOARD_INFO("HM2057", 0x48>>1),
    },
};

static struct nxp_v4l2_i2c_board_info sensor[] = {
    {
        .board_info = &front_camera_i2c_boardinfo[0],
        .i2c_adapter_id = 0,
    },
};

static struct nxp_capture_platformdata capture_plat_data[] = {
    {
        /* front_camera 601 interface */
        .module = 1,
        .sensor = &sensor[0],
        .type = NXP_CAPTURE_INF_PARALLEL,
        .parallel = {
            .is_mipi        = false,
            .external_sync  = true,
            .h_active       = 640,
            .h_frontporch   = 0,
            .h_syncwidth    = 0,
            .h_backporch    = 2,
            .v_active       = 480,
            .v_frontporch   = 0,
            .v_syncwidth    = 0,
            .v_backporch    = 2,
            .clock_invert   = false,
            .port           = 0,
            .data_order     = NXP_VIN_CRY1CBY0,
            .interlace      = false,
            .clk_rate       = 24000000,
            .late_power_down = true,
            .power_enable   = front_camera_power_enable,
            .power_state_changed = front_camera_power_state_changed,
            .set_clock      = camera_common_set_clock,
            .setup_io       = camera_common_vin_setup_io,
        },
        .deci = {
            .start_delay_ms = 0,
            .stop_delay_ms  = 0,
        },
    },
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
//        .external_irq = PAD_GPIO_A + 19,
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
static int _dwmci_ext_cd_init(void (*notify_func)(struct platform_device *, int state))
{
	return 0;
}

static int _dwmci_ext_cd_cleanup(void (*notify_func)(struct platform_device *, int state))
{
	return 0;
}

static int _dwmci_get_ro(u32 slot_id)
{
	return 0;
}

#ifdef CONFIG_MMC_NEXELL_CH0
static struct dw_mci_board _dwmci0_data = {
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
	//.clk_dly        = DW_MMC_DRIVE_DELAY(0) | DW_MMC_SAMPLE_DELAY(0x1c) | DW_MMC_DRIVE_PHASE(2) | DW_MMC_SAMPLE_PHASE(1),
    .clk_dly        = DW_MMC_DRIVE_DELAY(0) | DW_MMC_SAMPLE_DELAY(0) | DW_MMC_DRIVE_PHASE(3) | DW_MMC_SAMPLE_PHASE(0),
};
#endif

#ifdef CONFIG_MMC_NEXELL_CH1

static struct dw_mci_board _dwmci1_data = {
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION,
	.bus_hz			= 50 * 1000 * 1000,
	.caps = MMC_CAP_CMD23|MMC_CAP_NONREMOVABLE,
	.detect_delay_ms= 200,
	.cd_type = DW_MCI_CD_NONE,
	.pm_caps        = MMC_PM_KEEP_POWER | MMC_PM_IGNORE_PM_NOTIFY,
	.clk_dly        = DW_MMC_DRIVE_DELAY(0) | DW_MMC_SAMPLE_DELAY(0) | DW_MMC_DRIVE_PHASE(0) | DW_MMC_SAMPLE_PHASE(0),
};
#endif

#ifdef CONFIG_MMC_NEXELL_CH2
static int _dwmci2_init(u32 slot_id, irq_handler_t handler, void *data)
{
	struct dw_mci *host = (struct dw_mci *)data;
	int io  = CFG_SDMMC0_DETECT_IO;
	int irq = IRQ_GPIO_START + io;
	int id  = 0, ret = 0;

	printk("dw_mmc dw_mmc.%d: Using external card detect irq %3d (io %2d)\n", id, irq, io);

	ret  = request_irq(irq, handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				DEV_NAME_SDHC "2", (void*)host->slot[slot_id]);
	if (0 > ret)
		pr_err("dw_mmc dw_mmc.%d: fail request interrupt %d ...\n", id, irq);
	return 0;
}

static int _dwmci2_get_cd(u32 slot_id)
{
	int io = CFG_SDMMC0_DETECT_IO;
	return nxp_soc_gpio_get_in_value(io);
}

static struct dw_mci_board _dwmci2_data = {
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_CMD23,
	.detect_delay_ms= 200,
//	.sdr_timing		= 0x03020001,
//	.ddr_timing		= 0x03030002,
	.cd_type		= DW_MCI_CD_EXTERNAL,
	.clk_dly        = DW_MMC_DRIVE_DELAY(0) | DW_MMC_SAMPLE_DELAY(0) | DW_MMC_DRIVE_PHASE(2) | DW_MMC_SAMPLE_PHASE(1),
	.init			= _dwmci2_init,
	.get_ro         = _dwmci_get_ro,
	.get_cd			= _dwmci2_get_cd,
	.ext_cd_init	= _dwmci_ext_cd_init,
	.ext_cd_cleanup	= _dwmci_ext_cd_cleanup,
};
#endif



#endif /* CONFIG_MMC_DW */

/*------------------------------------------------------------------------------
 * RFKILL driver
 */
#if defined(CONFIG_RFKILL_NEXELL)

struct rfkill_dev_data  rfkill_dev_data =
{
	.supply_name 	= "vgps_3.3V",	// vwifi_3.3V, vgps_3.3V
	.module_name 	= "wlan",
	.initval		= RFKILL_INIT_SET | RFKILL_INIT_OFF,
    .delay_time_off	= 1000,
};

struct nxp_rfkill_plat_data rfkill_plat_data = {
	.name		= "WiFi-Rfkill",
	.type		= RFKILL_TYPE_WLAN,
	.rf_dev		= &rfkill_dev_data,
    .rf_dev_num	= 1,
};

static struct platform_device rfkill_device = {
	.name			= DEV_NAME_RFKILL,
	.dev			= {
		.platform_data	= &rfkill_plat_data,
	}
};
#endif	/* CONFIG_RFKILL_NEXELL */

#if defined(CONFIG_PPM_NEXELL)
#include <mach/ppm.h>
struct nxp_ppm_platform_data ppm_plat_data = {
	.input_polarity = NX_PPM_INPUTPOL_INVERT,//NX_PPM_INPUTPOL_INVERT  or  NX_PPM_INPUTPOL_BYPASS
};

static struct platform_device ppm_device = {
	.name			= DEV_NAME_PPM,
	.dev			= {
		.platform_data	= &ppm_plat_data,
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
	#ifdef CONFIG_MMC_NEXELL_CH2
	nxp_mmc_add_device(2, &_dwmci2_data);
	#endif
#endif

#if defined(CONFIG_KEYBOARD_NEXELL_KEY) || defined(CONFIG_KEYBOARD_NEXELL_KEY_MODULE)
	printk("plat: add device keypad\n");
	platform_device_register(&key_plat_device);
#endif

#if defined(CONFIG_SND_CODEC_WM8976) || defined(CONFIG_SND_CODEC_WM8976_MODULE)
	printk("plat: add device asoc-wm8976\n");
	i2c_register_board_info(WM8976_I2C_BUS, &wm8976_i2c_bdi, 1);
	platform_device_register(&wm8976_dai);
#endif

#if defined(CONFIG_SND_CODEC_ALC5623)
	printk("plat: add device asoc-alc5623\n");
	i2c_register_board_info(0, &alc5623_i2c_bdi, 1);
	platform_device_register(&alc5623_dai);
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

#if defined(CONFIG_RFKILL_NEXELL)
    printk("plat: add device rfkill\n");
    platform_device_register(&rfkill_device);
#endif

#if defined(CONFIG_NXPMAC_ETH)
    printk("plat: add device nxp-gmac\n");
    platform_device_register(&nxp_gmac_dev);
#endif

#if defined(CONFIG_PPM_NEXELL)
    printk("plat: add device ppm\n");
    platform_device_register(&ppm_device);
#endif

	/* END */
	printk("\n");
}
