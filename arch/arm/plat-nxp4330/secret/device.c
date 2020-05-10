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
	//{ 1200000, 1200 },
	//{ 1100000, 1200 },
	{ 1000000, 1200 },
	{  900000, 1200 },
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
 * DISPLAY MIPI device
 */
#if defined (CONFIG_NEXELL_DISPLAY_MIPI)
#include <linux/delay.h>

#if defined(CONFIG_SECRET_2P1ND_BOARD)||defined(CONFIG_SECRET_3RD_BOARD)||defined(CONFIG_SECRET_3P1RD_BOARD)
#define	MIPI_BITRATE_480M
#else
#define	MIPI_BITRATE_512M
#endif

#ifdef MIPI_BITRATE_1G
#define	PLLPMS		0x33E8
#define	BANDCTL		0xF
#elif defined(MIPI_BITRATE_900M)
#define	PLLPMS		0x2258
#define	BANDCTL		0xE
#elif defined(MIPI_BITRATE_840M)
#define	PLLPMS		0x2230
#define	BANDCTL		0xD
#elif defined(MIPI_BITRATE_750M)
#define	PLLPMS		0x43E8
#define	BANDCTL		0xC
#elif defined(MIPI_BITRATE_660M)
#define	PLLPMS		0x21B8
#define	BANDCTL		0xB
#elif defined(MIPI_BITRATE_600M)
#define	PLLPMS		0x2190
#define	BANDCTL		0xA
#elif defined(MIPI_BITRATE_540M)
#define	PLLPMS		0x2168
#define	BANDCTL		0x9
#elif defined(MIPI_BITRATE_512M)
#define	PLLPMS		0x3200
#define	BANDCTL		0x9
#elif defined(MIPI_BITRATE_480M)
#define	PLLPMS		0x2281
#define	BANDCTL		0x8
#elif defined(MIPI_BITRATE_420M)
#define	PLLPMS		0x2231
#define	BANDCTL		0x7
#elif defined(MIPI_BITRATE_402M)
#define	PLLPMS		0x2219
#define	BANDCTL		0x7
#elif defined(MIPI_BITRATE_210M)
#define	PLLPMS		0x2232
#define	BANDCTL		0x4
#endif

#define	PLLCTL		0
#define	DPHYCTL		0

#define MIPI_DELAY 0xFF

struct data_val{
	u8 data[48];
};

struct mipi_reg_val{
	u32 cmd;
	u32 addr;
	u32 cnt;
	struct data_val data;
};

#if defined(CONFIG_SECRET_2P1ND_BOARD)||defined(CONFIG_SECRET_3RD_BOARD)||defined(CONFIG_SECRET_3P1RD_BOARD)

static struct mipi_reg_val mipi_init_data_tiny[]= // BOE init code tiny
{
 {MIPI_DELAY,  5, 0, {0}},
 {0x39, 0xF0,  2, {0x5A, 0x5A}},
 {0x39, 0xD0,  2, {0x00, 0x10}},

 {MIPI_DELAY,  1, 0, {0}},
 {0x39, 0xC3,  3, {0x40, 0x00, 0x28}},

 {MIPI_DELAY,  5, 0, {0}},
 {0x05, 0x00,  1, {0x11}},

 {MIPI_DELAY,120, 0, {0}},
 {0x39, 0xF0,  2, {0x5A, 0x5A}},
 {0x15, 0x35,  1, {0x00}},
 {0x05, 0x00,  1, {0x29}},
};

static struct mipi_reg_val mipi_init_data[]= // BOE init code
{
 {0x39, 0xF0,  2, {0x5A, 0x5A}},
 {0x39, 0xF1,  2, {0x5A, 0x5A}},
 {0x39, 0xFC,  2, {0xA5, 0xA5}},
 {0x15, 0xB1,  1, {0x10}},
 {0x39, 0xB2,  4, {0x14, 0x22, 0x2F, 0x04}},
 {0x39, 0xD0,  2, {0x00, 0x10}},
 {0x39, 0xF2,  5, {0x02, 0x08, 0x08, 0x40, 0x10}},
 {0x15, 0xB0,  1, {0x03}},
 {0x39, 0xFD,  2, {0x23, 0x09}},
 {0x39, 0xF3, 10, {0x01, 0xD7, 0xE2, 0x62, 0xF4, 0xF7, 0x77, 0x3C, 0x26, 0x00}},
 {0x39, 0xF4, 45, {0x00, 0x02, 0x03, 0x26, 0x03, 0x02, 0x09, 0x00, 0x07, 0x16, 0x16, 0x03, 0x00, 0x08, 0x08, 0x03, 0x0E, 0x0F, 0x12, 0x1C, 0x1D, 0x1E, 0x0C, 0x09, 0x01, 0x04, 0x02, 0x61, 0x74, 0x75, 0x72, 0x83, 0x80, 0x80, 0xB0, 0x00, 0x01, 0x01, 0x28, 0x04, 0x03, 0x28, 0x01, 0xD1, 0x32}},
 {0x39, 0xF5, 26, {0xA2, 0x2F, 0x2F, 0x3A, 0xAB, 0x98, 0x52, 0x0F, 0x33, 0x43, 0x04, 0x59, 0x54, 0x52, 0x05, 0x40, 0x60, 0x4E, 0x60, 0x40, 0x27, 0x26, 0x52, 0x25, 0x6D, 0x18}},
 {0x39, 0xEE,  8, {0x22, 0x00, 0x22, 0x00, 0x22, 0x00, 0x22, 0x00}},
 {0x39, 0xEF,  8, {0x12, 0x12, 0x43, 0x43, 0xA0, 0x04, 0x24, 0x81}},
 {0x39, 0xF7, 32, {0x0A, 0x0A, 0x08, 0x08, 0x0B, 0x0B, 0x09, 0x09, 0x04, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0A, 0x0A, 0x08, 0x08, 0x0B, 0x0B, 0x09, 0x09, 0x04, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01}},
 {0x39, 0xBC,  3, {0x01, 0x4E, 0x0A}},
 {0x39, 0xE1,  5, {0x03, 0x10, 0x1C, 0xA0, 0x10}},
 {0x39, 0xF6,  6, {0x60, 0x21, 0xA6, 0x00, 0x00, 0x00}},
 {0x39, 0xFE,  6, {0x00, 0x0D, 0x03, 0x21, 0x00, 0x78}},
 {0x15, 0xB0,  1, {0x22}},
 {0x39, 0xFA, 17, {0x00, 0x35, 0x08, 0x10, 0x08, 0x0F, 0x14, 0x14, 0x17, 0x1F, 0x23, 0x24, 0x25, 0x24, 0x23, 0x23, 0x29}},
 {0x15, 0xB0,  1, {0x22}},
 {0x39, 0xFB, 17, {0x00, 0x35, 0x08, 0x10, 0x08, 0x0F, 0x14, 0x14, 0x17, 0x1F, 0x23, 0x24, 0x25, 0x24, 0x23, 0x23, 0x29}},
 {0x15, 0xB0,  1, {0x11}},
 {0x39, 0xFA, 17, {0x0B, 0x35, 0x11, 0x18, 0x0F, 0x14, 0x1A, 0x19, 0x1A, 0x22, 0x24, 0x25, 0x26, 0x25, 0x24, 0x24, 0x2A}},
 {0x15, 0xB0,  1, {0x11}},
 {0x39, 0xFB, 17, {0x0B, 0x35, 0x11, 0x18, 0x0F, 0x14, 0x1A, 0x19, 0x1A, 0x22, 0x24, 0x25, 0x26, 0x25, 0x24, 0x24, 0x2A}},
 {0x39, 0xFA, 17, {0x1F, 0x35, 0x22, 0x24, 0x18, 0x1B, 0x1D, 0x1A, 0x1B, 0x20, 0x26, 0x25, 0x28, 0x27, 0x25, 0x25, 0x2B}},
 {0x39, 0xFB, 17, {0x1F, 0x35, 0x22, 0x24, 0x18, 0x1B, 0x1D, 0x1A, 0x1B, 0x20, 0x26, 0x25, 0x28, 0x27, 0x25, 0x25, 0x2B}},
 {MIPI_DELAY, 10, 0, {0}},
 {0x39, 0xC3,  3, {0x40, 0x00, 0x28}},
 {MIPI_DELAY,120, 0, {0}},
 {0x15, 0x35,  1, {0x00}},
 {0x05, 0x00,  1, {0x11}},
 {0x05, 0x00,  1, {0x29}},
};

#elif defined(CONFIG_SECRET_2ND_BOARD)

static struct mipi_reg_val mipi_init_data[]= // INNOLUX init code
{
 {0x39, 0xFF, 4, {0xAA,0x55,0xA5,0x80}},
 {0x39, 0x6F, 2, {0x11,0x00}},
 {0x39, 0xF7, 2, {0x20,0x00}},
 {0x15, 0x6F, 1, {0x06}},
 {0x15, 0xF7, 1, {0xA0}},
 {0x15, 0x6F, 1, {0x19}},
 {0x15, 0xF7, 1, {0x12}},
 {0x39, 0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}},
 {0x39, 0xEF, 2, {0x07,0xFF}},
 {0x39, 0xEE, 4, {0x87,0x78,0x08,0x40}},
 {0x15, 0xC8, 1, {0x80}},
 {0x39, 0xB1, 2, {0x6C,0x01}},
 {0x15, 0xB6, 1, {0x08}},
 {0x15, 0x6F, 1, {0x02}},
 {0x15, 0xB8, 1, {0x08}},
 {0x39, 0xBB, 2, {0x54,0x54}},
 {0x39, 0xBC, 2, {0x05,0x05}},
 {0x15, 0xC7, 1, {0x01}},
 {0x39, 0xBD, 5, {0x02,0xB0,0x0C,0x0A,0x00}},
 {0x39, 0xF0, 5, {0x55,0xAA,0x52,0x08,0x01}},
 {0x39, 0xB0, 2, {0x05,0x05}},
 {0x39, 0xB1, 2, {0x05,0x05}},
 {0x39, 0xBC, 2, {0x8E,0x00}},
 {0x39, 0xBD, 2, {0x92,0x00}},
 {0x15, 0xCA, 1, {0x00}},
 {0x15, 0xC0, 1, {0x04}},
 {0x39, 0xB3, 2, {0x19,0x19}},
 {0x39, 0xB4, 2, {0x12,0x12}},
 {0x39, 0xB9, 2, {0x24,0x24}},
 {0x39, 0xBA, 2, {0x14,0x14}},
 {0x39, 0xF0, 5, {0x55,0xAA,0x52,0x08,0x02}},
 {0x15, 0xEE, 1, {0x02}},
 {0x39, 0xEF, 4, {0x09,0x06,0x15,0x18}},
 {0x39, 0xB0, 6, {0x00,0x00,0x00,0x11,0x00,0x27}},
 {0x15, 0x6F, 1, {0x06}},
 {0x39, 0xB0, 6, {0x00,0x36,0x00,0x45,0x00,0x5F}},
 {0x15, 0x6F, 1, {0x0C}},
 {0x39, 0xB0, 4, {0x00,0x74,0x00,0xA5}},
 {0x39, 0xB1, 6, {0x00,0xCF,0x01,0x13,0x01,0x47}},
 {0x15, 0x6F, 1, {0x06}},
 {0x39, 0xB1, 6, {0x01,0x9B,0x01,0xDF,0x01,0xE1}},
 {0x15, 0x6F, 1, {0x0C}},
 {0x39, 0xB1, 4, {0x02,0x23,0x02,0x6C}},
 {0x39, 0xB2, 6, {0x02,0x9A,0x02,0xD7,0x03,0x05}},
 {0x15, 0x6F, 1, {0x06}},
 {0x39, 0xB2, 6, {0x03,0x42,0x03,0x68,0x03,0x91}},
 {0x15, 0x6F, 1, {0x0C}},
 {0x39, 0xB2, 4, {0x03,0xA5,0x03,0xBD}},
 {0x39, 0xB3, 4, {0x03,0xD7,0x03,0xFF}},
 {0x39, 0xBC, 6, {0x00,0x00,0x00,0x11,0x00,0x27}},
 {0x15, 0x6F, 1, {0x06}},
 {0x39, 0xBC, 6, {0x00,0x38,0x00,0x47,0x00,0x61}},
 {0x15, 0x6F, 1, {0x0C}},
 {0x39, 0xBC, 4, {0x00,0x78,0x00,0xAB}},
 {0x39, 0xBD, 6, {0x00,0xD7,0x01,0x1B,0x01,0x4F}},
 {0x15, 0x6F, 1, {0x06}},
 {0x39, 0xBD, 6, {0x01,0xA1,0x01,0xE5,0x01,0xE7}},
 {0x15, 0x6F, 1, {0x0C}},
 {0x39, 0xBD, 4, {0x02,0x27,0x02,0x70}},
 {0x39, 0xBE, 6, {0x02,0x9E,0x02,0xDB,0x03,0x07}},
 {0x15, 0x6F, 1, {0x06}},
 {0x39, 0xBE, 6, {0x03,0x44,0x03,0x6A,0x03,0x93}},
 {0x15, 0x6F, 1, {0x0C}},
 {0x39, 0xBE, 4, {0x03,0xA5,0x03,0xBD}},
 {0x39, 0xBF, 4, {0x03,0xD7,0x03,0xFF}},
 {0x39, 0xF0, 5, {0x55,0xAA,0x52,0x08,0x06}},
 {0x39, 0xB0, 2, {0x00,0x17}},
 {0x39, 0xB1, 2, {0x16,0x15}},
 {0x39, 0xB2, 2, {0x14,0x13}},
 {0x39, 0xB3, 2, {0x12,0x11}},
 {0x39, 0xB4, 2, {0x10,0x2D}},
 {0x39, 0xB5, 2, {0x01,0x08}},
 {0x39, 0xB6, 2, {0x09,0x31}},
 {0x39, 0xB7, 2, {0x31,0x31}},
 {0x39, 0xB8, 2, {0x31,0x31}},
 {0x39, 0xB9, 2, {0x31,0x31}},
 {0x39, 0xBA, 2, {0x31,0x31}},
 {0x39, 0xBB, 2, {0x31,0x31}},
 {0x39, 0xBC, 2, {0x31,0x31}},
 {0x39, 0xBD, 2, {0x31,0x09}},
 {0x39, 0xBE, 2, {0x08,0x01}},
 {0x39, 0xBF, 2, {0x2D,0x10}},
 {0x39, 0xC0, 2, {0x11,0x12}},
 {0x39, 0xC1, 2, {0x13,0x14}},
 {0x39, 0xC2, 2, {0x15,0x16}},
 {0x39, 0xC3, 2, {0x17,0x00}},
 {0x39, 0xE5, 2, {0x31,0x31}},
 {0x39, 0xC4, 2, {0x00,0x17}},
 {0x39, 0xC5, 2, {0x16,0x15}},
 {0x39, 0xC6, 2, {0x14,0x13}},
 {0x39, 0xC7, 2, {0x12,0x11}},
 {0x39, 0xC8, 2, {0x10,0x2D}},
 {0x39, 0xC9, 2, {0x01,0x08}},
 {0x39, 0xCA, 2, {0x09,0x31}},
 {0x39, 0xCB, 2, {0x31,0x31}},
 {0x39, 0xCC, 2, {0x31,0x31}},
 {0x39, 0xCD, 2, {0x31,0x31}},
 {0x39, 0xCE, 2, {0x31,0x31}},
 {0x39, 0xCF, 2, {0x31,0x31}},
 {0x39, 0xD0, 2, {0x31,0x31}},
 {0x39, 0xD1, 2, {0x31,0x09}},
 {0x39, 0xD2, 2, {0x08,0x01}},
 {0x39, 0xD3, 2, {0x2D,0x10}},
 {0x39, 0xD4, 2, {0x11,0x12}},
 {0x39, 0xD5, 2, {0x13,0x14}},
 {0x39, 0xD6, 2, {0x15,0x16}},
 {0x39, 0xD7, 2, {0x17,0x00}},
 {0x39, 0xE6, 2, {0x31,0x31}},
 {0x39, 0xD8, 5, {0x00,0x00,0x00,0x00,0x00}},
 {0x39, 0xD9, 5, {0x00,0x00,0x00,0x00,0x00}},
 {0x15, 0xE7, 1, {0x00}},
 {0x39, 0xF0, 5, {0x55,0xAA,0x52,0x08,0x03}},
 {0x39, 0xB0, 2, {0x20,0x00}},
 {0x39, 0xB1, 2, {0x20,0x00}},
 {0x39, 0xB2, 5, {0x05,0x00,0x42,0x00,0x00}},
 {0x39, 0xB6, 5, {0x05,0x00,0x42,0x00,0x00}},
 {0x39, 0xBA, 5, {0x53,0x00,0x42,0x00,0x00}},
 {0x39, 0xBB, 5, {0x53,0x00,0x42,0x00,0x00}},
 {0x15, 0xC4, 1, {0x40}},
 {0x39, 0xF0, 5, {0x55,0xAA,0x52,0x08,0x05}},
 {0x39, 0xB0, 2, {0x17,0x06}},
 {0x15, 0xB8, 1, {0x00}},
 {0x39, 0xBD, 5, {0x03,0x01,0x01,0x00,0x01}},
 {0x39, 0xB1, 2, {0x17,0x06}},
 {0x39, 0xB9, 2, {0x00,0x01}},
 {0x39, 0xB2, 2, {0x17,0x06}},
 {0x39, 0xBA, 2, {0x00,0x01}},
 {0x39, 0xB3, 2, {0x17,0x06}},
 {0x39, 0xBB, 2, {0x0A,0x00}},
 {0x39, 0xB4, 2, {0x17,0x06}},
 {0x39, 0xB5, 2, {0x17,0x06}},
 {0x39, 0xB6, 2, {0x14,0x03}},
 {0x39, 0xB7, 2, {0x00,0x00}},
 {0x39, 0xBC, 2, {0x02,0x01}},
 {0x15, 0xC0, 1, {0x05}},
 {0x15, 0xC4, 1, {0xA5}},
 {0x39, 0xC8, 2, {0x03,0x30}},
 {0x39, 0xC9, 2, {0x03,0x51}},
 {0x39, 0xD1, 5, {0x00,0x05,0x03,0x00,0x00}},
 {0x39, 0xD2, 5, {0x00,0x05,0x09,0x00,0x00}},
 {0x15, 0xE5, 1, {0x02}},
 {0x15, 0xE6, 1, {0x02}},
 {0x15, 0xE7, 1, {0x02}},
 {0x15, 0xE9, 1, {0x02}},
 {0x15, 0xED, 1, {0x33}},
 {0x05, 0x00, 1, {0x11}},
 {0x05, 0x00, 1, {0x29}},
};

#else

static struct mipi_reg_val mipi_init_data[]=
{
 {0x15, 0xB2,  1, {0x7D}},
 {0x15, 0xAE,  1, {0x0B}},
 {0x15, 0xB6,  1, {0x18}},
 {0x15, 0xD2,  1, {0x64}},
};

#endif


static void  mipilcd_dcs_long_write(U32 cmd, U32 ByteCount, U8* pByteData )
{
	U32 DataCount32 = (ByteCount+3)/4;
	int i = 0;
	U32 index = 0;
	volatile NX_MIPI_RegisterSet* pmipi = (volatile NX_MIPI_RegisterSet*)IO_ADDRESS(NX_MIPI_GetPhysicalAddress(index));

	NX_ASSERT( 512 >= DataCount32 );

	//for(i=0; i< ByteCount; i++)
	//	printf(",0x%02x", pByteData[i]);
	//printf("\n");

	for( i=0; i<DataCount32; i++ )
	{
		pmipi->DSIM_PAYLOAD = (pByteData[3]<<24)|(pByteData[2]<<16)|(pByteData[1]<<8)|pByteData[0];
		pByteData += 4;
	}
	pmipi->DSIM_PKTHDR  = (cmd & 0xff) | (ByteCount<<8);
}

static void mipilcd_dcs_write( unsigned int id, unsigned int data0, unsigned int data1 )
{
	U32 index = 0;
	volatile NX_MIPI_RegisterSet* pmipi = (volatile NX_MIPI_RegisterSet*)IO_ADDRESS(NX_MIPI_GetPhysicalAddress(index));

#if 0
	int i = 0;
	switch(id)
	{
		case 0x05:
			printf(",0x%02x\n", data0);
			break;

		case 0x15:
			printf(",0x%02x,0x%02x\n", data0, data1);
			break;
	}
#endif

	pmipi->DSIM_PKTHDR = id | (data0<<8) | (data1<<16);
}


static int MIPI_LCD_INIT(int width, int height, void *data)
{
	int i=0;
	U32 index = 0;
	U32 value = 0;
	u8 pByteData[48];
	int size=ARRAY_SIZE(mipi_init_data);
	//struct mipi_reg_val *lcd_init_data = &mipi_init_data;

	volatile NX_MIPI_RegisterSet* pmipi = (volatile NX_MIPI_RegisterSet*)IO_ADDRESS(NX_MIPI_GetPhysicalAddress(index));
	value = pmipi->DSIM_ESCMODE;
	pmipi->DSIM_ESCMODE = value|(3 << 6);
	value = pmipi->DSIM_ESCMODE;

	mdelay(10);

	for(i=0; i<size; i++)
	{
		switch(mipi_init_data[i].cmd)
		{
#if 0 // all long packet
			case 0x05:
				//pByteData[0] = mipi_init_data[i].addr;
				//memcpy(&pByteData[1], &mipi_init_data[i].data.data[0], 7);
				mipilcd_dcs_long_write(0x39, mipi_init_data[i].cnt, &mipi_init_data[i].data.data[0]);
				break;
			case 0x15:
				pByteData[0] = mipi_init_data[i].addr;
				memcpy(&pByteData[1], &mipi_init_data[i].data.data[0], 7);
				mipilcd_dcs_long_write(0x39, mipi_init_data[i].cnt+1, &pByteData);
				break;
#else
			case 0x05:
				mipilcd_dcs_write(mipi_init_data[i].cmd, mipi_init_data[i].data.data[0], 0x00);
				break;
			case 0x15:
				mipilcd_dcs_write(mipi_init_data[i].cmd, mipi_init_data[i].addr, mipi_init_data[i].data.data[0]);
				break;
#endif
 			case 0x39:
				pByteData[0] = mipi_init_data[i].addr;
				memcpy(&pByteData[1], &mipi_init_data[i].data.data[0], 48);
				mipilcd_dcs_long_write(mipi_init_data[i].cmd, mipi_init_data[i].cnt+1, &pByteData[0]);
				break;
			case MIPI_DELAY:
				mdelay(mipi_init_data[i].addr);
				break;
		}
		mdelay(1);
	}

	value = pmipi->DSIM_ESCMODE;
	pmipi->DSIM_ESCMODE = value&(~(3 << 6));
	value = pmipi->DSIM_ESCMODE;
	mdelay(10);
	return 0;
}

static struct disp_vsync_info mipi_vsync_param = {
	.h_active_len	= CFG_DISP_PRI_RESOL_WIDTH,
	.h_sync_width	= CFG_DISP_PRI_HSYNC_SYNC_WIDTH,
	.h_back_porch	= CFG_DISP_PRI_HSYNC_BACK_PORCH,
	.h_front_porch	= CFG_DISP_PRI_HSYNC_FRONT_PORCH,
	.h_sync_invert	= CFG_DISP_PRI_HSYNC_ACTIVE_HIGH,
	.v_active_len	= CFG_DISP_PRI_RESOL_HEIGHT,
	.v_sync_width	= CFG_DISP_PRI_VSYNC_SYNC_WIDTH,
	.v_back_porch	= CFG_DISP_PRI_VSYNC_BACK_PORCH,
	.v_front_porch	= CFG_DISP_PRI_VSYNC_FRONT_PORCH,
	.v_sync_invert	= CFG_DISP_PRI_VSYNC_ACTIVE_HIGH,
	.pixel_clock_hz	= CFG_DISP_PRI_PIXEL_CLOCK,
	.clk_src_lv0	= CFG_DISP_PRI_CLKGEN0_SOURCE,
	.clk_div_lv0	= CFG_DISP_PRI_CLKGEN0_DIV,
	.clk_src_lv1	= CFG_DISP_PRI_CLKGEN1_SOURCE,
	.clk_div_lv1	= CFG_DISP_PRI_CLKGEN1_DIV,
};

static struct disp_syncgen_par mipi_syncgen_param = {
#if 1
	.delay_mask			= DISP_SYNCGEN_DELAY_RGB_PVD |
						  DISP_SYNCGEN_DELAY_HSYNC_CP1 |
					 	  DISP_SYNCGEN_DELAY_VSYNC_FRAM |
					 	  DISP_SYNCGEN_DELAY_DE_CP,
	.d_rgb_pvd			= 0,
	.d_hsync_cp1		= 0,
	.d_vsync_fram		= 0,
	.d_de_cp2			= 7,
	.vs_start_offset 	= CFG_DISP_PRI_HSYNC_FRONT_PORCH +
						  CFG_DISP_PRI_HSYNC_SYNC_WIDTH +
						  CFG_DISP_PRI_HSYNC_BACK_PORCH +
						  CFG_DISP_PRI_RESOL_WIDTH - 1,
	.ev_start_offset 	= CFG_DISP_PRI_HSYNC_FRONT_PORCH +
						  CFG_DISP_PRI_HSYNC_SYNC_WIDTH +
						  CFG_DISP_PRI_HSYNC_BACK_PORCH +
						  CFG_DISP_PRI_RESOL_WIDTH - 1,
	.vs_end_offset 		= 0,
	.ev_end_offset 		= 0,
#else
	.interlace 		= CFG_DISP_PRI_MLC_INTERLACE,
	.out_format		= CFG_DISP_PRI_OUT_FORMAT,
	.invert_field 	= CFG_DISP_PRI_OUT_INVERT_FIELD,
	.swap_RB		= CFG_DISP_PRI_OUT_SWAPRB,
	.yc_order		= CFG_DISP_PRI_OUT_YCORDER,
	.delay_mask		= DISP_SYNCGEN_DELAY_RGB_PVD|DISP_SYNCGEN_DELAY_HSYNC_CP1|DISP_SYNCGEN_DELAY_VSYNC_FRAM|DISP_SYNCGEN_DELAY_DE_CP,
	.d_rgb_pvd		= 0,
	.d_hsync_cp1	= 0,
	.d_vsync_fram	= 0,
	.d_de_cp2		= 7,
	.vs_start_offset = (CFG_DISP_PRI_HSYNC_FRONT_PORCH + CFG_DISP_PRI_HSYNC_SYNC_WIDTH + CFG_DISP_PRI_HSYNC_BACK_PORCH + CFG_DISP_PRI_RESOL_WIDTH - 1),
	.vs_end_offset	= 0,
	.ev_start_offset = (CFG_DISP_PRI_HSYNC_FRONT_PORCH + CFG_DISP_PRI_HSYNC_SYNC_WIDTH + CFG_DISP_PRI_HSYNC_BACK_PORCH + CFG_DISP_PRI_RESOL_WIDTH - 1),
	.ev_end_offset	= 0,
	.vclk_select	= CFG_DISP_PRI_PADCLKSEL,
	.clk_delay_lv0	= CFG_DISP_PRI_CLKGEN0_DELAY,
	.clk_inv_lv0	= CFG_DISP_PRI_CLKGEN0_INVERT,
	.clk_delay_lv1	= CFG_DISP_PRI_CLKGEN1_DELAY,
	.clk_inv_lv1	= CFG_DISP_PRI_CLKGEN1_INVERT,
	.clk_sel_div1	= CFG_DISP_PRI_CLKSEL1_SELECT,
#endif
};

static struct disp_mipi_param mipi_param = {
	.pllpms 	= PLLPMS,
	.bandctl	= BANDCTL,
	.pllctl		= PLLCTL,
	.phyctl		= DPHYCTL,
	.lcd_init	= MIPI_LCD_INIT
};
#endif

/*------------------------------------------------------------------------------
 * backlight : generic pwm device
 */
#if defined(CONFIG_BACKLIGHT_PWM)
#include <linux/pwm_backlight.h>

static struct platform_pwm_backlight_data bl_plat_data = {
	.pwm_id			= CFG_LCD_PRI_PWM_CH,
#if defined(CONFIG_SECRET_2P1ND_BOARD)||defined(CONFIG_SECRET_3RD_BOARD)||defined(CONFIG_SECRET_3P1RD_BOARD)
	.max_brightness = 255,		/* 255 is 100% */
#else
	.max_brightness = 255 + 255,	/* 255 is 100%, set over 100% */
#endif
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
 * Touch platform device
 */
#if defined(CONFIG_TOUCHSCREEN_AW5306)
#include <linux/i2c.h>
#define	AW5306_I2C_BUS		(1)

static struct i2c_board_info __initdata aw5306_i2c_bdi = {
	.type	= "aw5306_ts",
	.addr	= (0x70>>1),
    .irq    = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
};
#endif

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT640T)
#include <linux/i2c.h>
#include <linux/i2c/mxts.h>

#define	MXT604T_I2C_BUS		(1)


static int _atmel604t_power(bool on)
{
    printk("%s %d\n", __func__, on);

    if (on)
	{

		nxp_soc_gpio_set_out_value(CFG_IO_TOUCH_RESET_PIN, 1);

	}

    return 0;
}

static bool _atmel604t_read_change( )
{
    printk("%s %d\n", __func__);

    return 0;
}

static struct mxt_platform_data mxt_platform_data = {
	.num_xnode=32,
	.num_ynode=20,
	.max_x=1280,
	.max_y=800,
	.irqflags = 0, // irq high or low
	.boot_address = 0x26,
	.revision = 0x01,
#if defined(CONFIG_TOUCHSCREEN_SMAC)
	.firmware_name = MXT_SMAC_FIRMWARE_NAME,		// MXT_SMAC_FIRMWARE_NAME or MXT_TRAIS_FIRMWARE_NAME
#else
	.firmware_name = MXT_TRAIS_FIRMWARE_NAME,		// MXT_SMAC_FIRMWARE_NAME or MXT_TRAIS_FIRMWARE_NAME
#endif
//	.tsp_en;		/* enable LDO 3.3V */
//	.tsp_en1;		/* enable LDO 8.2V */
	.tsp_int = CFG_IO_TOUCH_PENDOWN_DETECT,
//	.tsp_rst

	.project_name = "PXD",
	.model_name = "QFD",
//	const char *config_ver;
    .config = 0,
	.read_chg = _atmel604t_read_change,
	.power_onoff = _atmel604t_power,
//	.register_cb = NULL,
};

static struct i2c_board_info __initdata mxt604t_i2c_bdi = {
	.type	= "Atmel MXT640T",
	.addr	= MXT_APP_LOW,
	.irq    = PB_PIO_IRQ(CFG_IO_TOUCH_PENDOWN_DETECT),
	.platform_data	= &mxt_platform_data,
};

#endif

/*------------------------------------------------------------------------------
 * Keypad platform device
 */
#if defined(CONFIG_KEYBOARD_NEXELL_KEY) || defined(CONFIG_KEYBOARD_NEXELL_KEY_MODULE)

#include <linux/input.h>

static unsigned int  button_gpio[] = CFG_KEYPAD_KEY_BUTTON;
static unsigned int  button_code[] = CFG_KEYPAD_KEY_CODE;
#if defined(CONFIG_SECRET_3RD_BOARD)||defined(CONFIG_SECRET_3P1RD_BOARD)
static unsigned int  button_detect[] = {0, 0, 0, 0, 1};
#else
static unsigned int  button_detect[] = {0, 0, 0, 0, 0, 1};
#endif

struct nxp_key_plat_data key_plat_data = {
	.bt_count	= ARRAY_SIZE(button_gpio),
	.bt_io		= button_gpio,
	.bt_code	= button_code,
	.bt_repeat	= CFG_KEYPAD_REPEAT,
	.bt_detect_high	= button_detect,
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
		.detect_io		= CFG_IO_MCU_HP_DET,
		.detect_level	= 0,
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
		.detect_io		= CFG_IO_MCU_HP_DET,
		.detect_level	= 0,
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
 * Micom platform device
 */
#if 0//defined(CONFIG_PM_MICOM) || defined(CONFIG_PM_MICOM_MODULE)
#include <linux/i2c.h>

#define	MICOM_I2C_BUS		(3)

static struct i2c_board_info micom_i2c_bdi = {
	I2C_BOARD_INFO("pm-micom", (0x30)),
};
#endif

/*------------------------------------------------------------------------------
 * Accel-Sensor platform device
 */
#if defined(CONFIG_SENSORS_BMA2X2) || defined(CONFIG_SENSORS_BMA2X2_MODULE)
#include <linux/i2c.h>
#include <linux/bst_sensor_common.h>

static struct bosch_sensor_specific bss_bma2x2 = {
	.name = "bma2x2",
	.place = 7,
};

#define	BMA2X2_I2C_BUS		(2)
#define BMA2X2_IRQ			(PAD_GPIO_E + 2)

static struct i2c_board_info bma2x2_i2c_bdi = {
	.type	= "bma2x2",
	.irq	= PB_PIO_IRQ(BMA2X2_IRQ),
	.addr	= (0x18),
	.platform_data = &bss_bma2x2,
};
#endif

/*------------------------------------------------------------------------------
 * Gyro-Sensor platform device
 */
#if defined(CONFIG_SENSORS_BMG) || defined(CONFIG_SENSORS_BMG_MODULE)
#include <linux/i2c.h>
#include <linux/bst_sensor_common.h>

#define	BMG160_I2C_BUS		(2)

static struct bosch_sensor_specific bss_bmg160 = {
	.name = "bmg160",
	.place = 7,
};

static struct i2c_board_info bmg160_i2c_bdi = {
	.type	= "bmg160",
	.addr	= (0x68),
	.platform_data = &bss_bmg160,
};
#endif

/*------------------------------------------------------------------------------
 * Haptic platform device
 */
#if 0//defined(CONFIG_HAPTIC_DRV2605) || defined(CONFIG_HAPTIC_DRV2605_MODULE)
#include <linux/i2c.h>

#define	DRV2605_I2C_BUS		(0)

static struct i2c_board_info __initdata drv2605_i2c_bdi = {
	.type	= "drv2605",
	.addr	= (0x5A),
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


#if defined(CONFIG_I2C_NEXELL)
#define I2CUDELAY(x)	1000000/x/2
/* gpio i2c 3 */
#define	I2C3_SCL	PAD_GPIO_A + 25
#define	I2C3_SDA	PAD_GPIO_A + 26

static struct i2c_gpio_platform_data nxp_i2c_gpio_port3 = {
	.sda_pin	= I2C3_SDA,
	.scl_pin	= I2C3_SCL,
	.udelay		= I2CUDELAY(CFG_I2C2_CLK),				/* Gpio_mode CLK Rate = 1/( udelay*2) * 1000000 */
	.timeout	= 10,
};


static struct platform_device i2c_device_ch3 = {
	.name	= "i2c-gpio",
	.id		= 3,
	.dev    = {
		.platform_data	= &nxp_i2c_gpio_port3,
	},
};

static struct platform_device *i2c_devices[] = {
	&i2c_device_ch3,
};
#endif /* CONFIG_I2C_NEXELL */


/*------------------------------------------------------------------------------
 * PMIC platform device
 */
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

#define NXE2000_I2C_BUS		(3)
#define NXE2000_I2C_ADDR	(0x64 >> 1)
#define NXE2000_IRQ			CFG_GPIO_PMIC_INTR

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
	REGULATOR_SUPPLY("vwifi_3.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo2_supply_0[] = {
	REGULATOR_SUPPLY("vlcd_1.8V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo3_supply_0[] = {
	REGULATOR_SUPPLY("vsys1_1.8V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo4_supply_0[] = {
	REGULATOR_SUPPLY("vsys_1.8V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo5_supply_0[] = {
	REGULATOR_SUPPLY("vdumy0", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo6_supply_0[] = {
	REGULATOR_SUPPLY("valive_3.3V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo7_supply_0[] = {
	REGULATOR_SUPPLY("vvid_3.3V", NULL),
};
#if defined(CONFIG_SECRET_2ND_BOARD)||defined(CONFIG_SECRET_2P1ND_BOARD)
static struct regulator_consumer_supply nxe2000_ldo8_supply_0[] = {
	REGULATOR_SUPPLY("vdumy3", NULL),
};
#else
static struct regulator_consumer_supply nxe2000_ldo8_supply_0[] = {
	REGULATOR_SUPPLY("vaudio_3.3V", NULL),
};
#endif
static struct regulator_consumer_supply nxe2000_ldo9_supply_0[] = {
	REGULATOR_SUPPLY("vdumy1", NULL),
};
static struct regulator_consumer_supply nxe2000_ldo10_supply_0[] = {
	REGULATOR_SUPPLY("vdumy2", NULL),
};
#if defined(CONFIG_SECRET_2ND_BOARD)||defined(CONFIG_SECRET_2P1ND_BOARD)
static struct regulator_consumer_supply nxe2000_ldortc1_supply_0[] = {
	REGULATOR_SUPPLY("vdumy4", NULL),
};
static struct regulator_consumer_supply nxe2000_ldortc2_supply_0[] = {
	REGULATOR_SUPPLY("vdumy5", NULL),
};
#else
static struct regulator_consumer_supply nxe2000_ldortc1_supply_0[] = {
	REGULATOR_SUPPLY("valive_1.8V", NULL),
};
static struct regulator_consumer_supply nxe2000_ldortc2_supply_0[] = {
	REGULATOR_SUPPLY("valive_1.0V", NULL),
};
#endif


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
#if defined(CONFIG_SECRET_3RD_BOARD)||defined(CONFIG_SECRET_3P1RD_BOARD)
NXE2000_PDATA_INIT(dc1,      0,	1000000, 2000000, 1, 1, 1100000, 1, 12);	/* 1.1V ARM */
NXE2000_PDATA_INIT(dc2,      0,	1000000, 2000000, 1, 1, 1000000, 1, 14);	/* 1.0V CORE */
NXE2000_PDATA_INIT(dc3,      0,	1000000, 3500000, 1, 1, 3300000, 1,  2);	/* 3.3V SYS */
NXE2000_PDATA_INIT(dc4,      0,	1000000, 2000000, 1, 1, 1500000, 1, -1);	/* 1.5V DDR */
NXE2000_PDATA_INIT(dc5,      0,	1000000, 2000000, 1, 1, 1500000, 1,  8);	/* 1.5V SYS */
NXE2000_PDATA_INIT(ldo1,     0,	1000000, 3500000, 1, 0, 3300000, 1,  2);	/* 3.3V TOUCH */
NXE2000_PDATA_INIT(ldo2,     0,	1000000, 3500000, 1, 1, 1800000, 1,  2);	/* 1.8V LCD */
NXE2000_PDATA_INIT(ldo3,     0,	1000000, 3500000, 1, 0, 1800000, 1,  6);	/* 1.8V SYS1 */
NXE2000_PDATA_INIT(ldo4,     0,	1000000, 3500000, 1, 0, 1800000, 1,  6);	/* 1.8V SYS */
NXE2000_PDATA_INIT(ldo5,     0,	1000000, 3500000, 1, 0, 3300000, 1, -1);	/* 3.3V SENSOR */
NXE2000_PDATA_INIT(ldo6,     0,	1000000, 3500000, 1, 0, 3300000, 1, -1);	/* 3.3V ALIVE */
NXE2000_PDATA_INIT(ldo7,     0,	1000000, 3500000, 1, 0, 1800000, 1,  2);	/* 1.8V VID2 */
NXE2000_PDATA_INIT(ldo8,     0,	1000000, 3500000, 1, 0, 3300000, 1,  2);	/* 3.3V AUDIO */
NXE2000_PDATA_INIT(ldo9,     0,	1000000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */
NXE2000_PDATA_INIT(ldo10,    0,	1000000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */
NXE2000_PDATA_INIT(ldortc1,  0,	1700000, 3500000, 1, 0, 1800000, 1, -1);	/* 1.8V ALIVE */
NXE2000_PDATA_INIT(ldortc2,  0,	1000000, 3500000, 1, 0, 1000000, 1, -1);	/* 1.0V ALIVE */

#elif defined(CONFIG_SECRET_2ND_BOARD)||defined(CONFIG_SECRET_2P1ND_BOARD)
NXE2000_PDATA_INIT(dc1,      0,	1000000, 2000000, 1, 1, 1100000, 1, 12);	/* 1.1V ARM */
NXE2000_PDATA_INIT(dc2,      0,	1000000, 2000000, 1, 1, 1000000, 1, 14);	/* 1.0V CORE */
NXE2000_PDATA_INIT(dc3,      0,	1000000, 3500000, 1, 1, 3300000, 1,  2);	/* 3.3V SYS */
NXE2000_PDATA_INIT(dc4,      0,	1000000, 2000000, 1, 1, 1500000, 1, -1);	/* 1.5V DDR */
NXE2000_PDATA_INIT(dc5,      0,	1000000, 2000000, 1, 1, 1500000, 1,  8);	/* 1.5V SYS */
NXE2000_PDATA_INIT(ldo1,     0,	1000000, 3500000, 1, 0, 3300000, 1,  -1);	/* 3.3V WIFI */
NXE2000_PDATA_INIT(ldo2,     0,	1000000, 3500000, 1, 1, 1800000, 1,  2);	/* 1.8V LCD */
NXE2000_PDATA_INIT(ldo3,     0,	1000000, 3500000, 1, 0, 1800000, 1,  6);	/* 1.8V SYS1 */
NXE2000_PDATA_INIT(ldo4,     0,	1000000, 3500000, 1, 0, 1800000, 1,  6);	/* 1.8V SYS */
NXE2000_PDATA_INIT(ldo5,     0,	1000000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */
NXE2000_PDATA_INIT(ldo6,     0,	1000000, 3500000, 1, 0, 3300000, 1, -1);	/* 3.3V ALIVE */
NXE2000_PDATA_INIT(ldo7,     0,	1000000, 3500000, 1, 0, 3300000, 1,  2);	/* 3.3V VID */
NXE2000_PDATA_INIT(ldo8,     0,	1000000, 3500000, 1, 0, 3300000, 1,  -1);	/* 3.3V WIFI */
NXE2000_PDATA_INIT(ldo9,     0,	1000000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */
NXE2000_PDATA_INIT(ldo10,    0,	1000000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */
NXE2000_PDATA_INIT(ldortc1,  0,	1700000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */
NXE2000_PDATA_INIT(ldortc2,  0,	1000000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */

#else
NXE2000_PDATA_INIT(dc1,      0,	1000000, 2000000, 1, 1, 1100000, 1, 12);	/* 1.1V ARM */
NXE2000_PDATA_INIT(dc2,      0,	1000000, 2000000, 1, 1, 1000000, 1, 14);	/* 1.0V CORE */
NXE2000_PDATA_INIT(dc3,      0,	1000000, 3500000, 1, 1, 3300000, 1,  2);	/* 3.3V SYS */
NXE2000_PDATA_INIT(dc4,      0,	1000000, 2000000, 1, 1, 1500000, 1, -1);	/* 1.5V DDR */
NXE2000_PDATA_INIT(dc5,      0,	1000000, 2000000, 1, 1, 1500000, 1,  8);	/* 1.5V SYS */
NXE2000_PDATA_INIT(ldo1,     0,	1000000, 3500000, 1, 0, 3300000, 1,  -1);	/* 3.3V WIFI */
NXE2000_PDATA_INIT(ldo2,     0,	1000000, 3500000, 1, 1, 1800000, 1,  2);	/* 1.8V LCD */
NXE2000_PDATA_INIT(ldo3,     0,	1000000, 3500000, 1, 0, 1800000, 1,  6);	/* 1.8V SYS1 */
NXE2000_PDATA_INIT(ldo4,     0,	1000000, 3500000, 1, 0, 1800000, 1,  6);	/* 1.8V SYS */
NXE2000_PDATA_INIT(ldo5,     0,	1000000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */
NXE2000_PDATA_INIT(ldo6,     0,	1000000, 3500000, 1, 0, 3300000, 1, -1);	/* 3.3V ALIVE */
NXE2000_PDATA_INIT(ldo7,     0,	1000000, 3500000, 1, 0, 3300000, 1,  2);	/* 3.3V VID */
NXE2000_PDATA_INIT(ldo8,     0,	1000000, 3500000, 0, 0, 3300000, 0,  0);	/* 3.3V AUDIO */
NXE2000_PDATA_INIT(ldo9,     0,	1000000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */
NXE2000_PDATA_INIT(ldo10,    0,	1000000, 3500000, 0, 0,      -1, 0,  0);	/* Not Use */
NXE2000_PDATA_INIT(ldortc1,  0,	1700000, 3500000, 1, 0, 1800000, 1, -1);	/* 1.8V ALIVE */
NXE2000_PDATA_INIT(ldortc2,  0,	1000000, 3500000, 1, 0, 1000000, 1, -1);	/* 1.0V ALIVE */
#endif

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

	.input_power_type	= INPUT_POWER_TYPE_UBC,

	.gpio_otg_usbid		= CFG_GPIO_OTG_USBID_DET,
	.gpio_otg_vbus		= CFG_GPIO_OTG_VBUS_DET,
	.gpio_pmic_vbus		= CFG_GPIO_PMIC_VUSB_DET,
	.gpio_pmic_lowbat	= CFG_GPIO_PMIC_LOWBAT_DET,

	.alarm_vol_mv		= 3450,
//	.alarm_vol_mv		= 3412,
	.bat_impe			= 1500,
	.slp_ibat			= 3450,
//	.adc_channel		= NXE2000_ADC_CHANNEL_VBAT,
	.multiple			= 100,	//100%
	.monitor_time		= 60,
		/* some parameter is depend of battery type */
	.type[0] = {
#if defined(CONFIG_SECRET_3RD_BOARD)||defined(CONFIG_SECRET_3P1RD_BOARD)
		.ch_vfchg		= 0x04,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
#else
		.ch_vfchg		= 0x03,	/* VFCHG	= 0 - 4 (4.05v, 4.10v, 4.15v, 4.20v, 4.35v) */
#endif
		.ch_vrchg		= 0x03,	/* VRCHG	= 0 - 4 (3.85v, 3.90v, 3.95v, 4.00v, 4.10v) */
		.ch_vbatovset	= 0xFF,	/* VBATOVSET	= 0 or 1 (0 : 4.38v(up)/3.95v(down) 1: 4.53v(up)/4.10v(down)) */
		.ch_ichg 		= 0x0E,	/* ICHG		= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_adp 	= 0x18,	/* ILIM_ADP	= 0 - 0x1D (100mA - 3000mA) */
		.ch_ilim_usb 	= 0x04,	/* ILIM_USB	= 0 - 0x1D (100mA - 3000mA) */
		.ch_icchg		= 0x03,	/* ICCHG	= 0 - 3 (50mA 100mA 150mA 200mA) */
		.fg_target_vsys	= 3450,	/* This value is the target one to DSOC=0% */
		.fg_target_ibat	= 1000,	/* This value is the target one to DSOC=0% */
		.fg_poff_vbat	= 3450,	/* setting value of 0 per Vbat */
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
		.ch_ichg		= 0x0,
		.ch_ilim_adp	= 0x0,
		.ch_ilim_usb	= 0x0,
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
	NXE2000_REG(DC1, dc1, 0),	\
	NXE2000_REG(DC2, dc2, 0),	\
	NXE2000_REG(DC3, dc3, 0),	\
	NXE2000_REG(DC4, dc4, 0),	\
	NXE2000_REG(DC5, dc5, 0),	\
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
	.irq_type			= IRQ_TYPE_EDGE_RISING,
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
 * v4l2 platform device
 */
#if defined(CONFIG_V4L2_NEXELL) || defined(CONFIG_V4L2_NEXELL_MODULE)
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <mach/nxp-v4l2-platformdata.h>
#include <mach/soc.h>

static struct nxp_capture_platformdata capture_plat_data[] = {
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
	//.caps2			= MMC_CAP2_PACKED_WR,
	.desc_sz		= 4,
	.detect_delay_ms= 200,
	.sdr_timing		= 0x01010001,
	.ddr_timing		= 0x03030002,
};
#endif

#ifdef CONFIG_MMC_NEXELL_CH1

#include <linux/mmc/card.h>

#if defined(CONFIG_BROADCOM_WIFI) || defined(CONFIG_BCMDHD)
static void (*wifi_status_cb)(struct platform_device *, int state);
#endif
int _dwmci1_ext_cd_init(void (*notify_func)(struct platform_device *, int state))
{
#if defined(CONFIG_BROADCOM_WIFI) || defined(CONFIG_BCMDHD)
	u_int io = PAD_GPIO_A + 19;
	wifi_status_cb = notify_func;
	nxp_soc_gpio_set_out_value(io, 0);
#endif
	return 0;
}

int _dwmci1_ext_cd_cleanup(void (*notify_func)(struct platform_device *, int state))
{
#if defined(CONFIG_BROADCOM_WIFI) || defined(CONFIG_BCMDHD)
	u_int io = PAD_GPIO_A + 19;
	wifi_status_cb = NULL;
	nxp_soc_gpio_set_out_value(io, 1);
#endif
	return 0;
}

static int _dwmci1_init(u32 slot_id, irq_handler_t handler, void *data)
{
#if defined(CONFIG_BROADCOM_WIFI) || defined(CONFIG_BCMDHD)
	struct dw_mci *host = (struct dw_mci *)data;
	u_int io = PAD_GPIO_A + 19;
	int irq = IRQ_GPIO_START + io;
	int id = 1;
	int ret = 0;

	printk("dw_mmc dw_mmc.%d: Using external card detect irq %3d (io %2d)\n", id, irq, io);

	nxp_soc_gpio_set_io_dir(io, 1);
	nxp_soc_gpio_set_out_value(io, 0);
	ret = request_irq(irq, handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, DEV_NAME_SDHC "1", (void *)host->slot[slot_id]);
	if (0 > ret)
		pr_err("dw_mmc.%d: failed to request irq %d\n", slot_id, irq);

	/* MCU_WL_REG_ON */
	io = CFG_IO_MCU_WL_REG_ON;
	nxp_soc_gpio_set_out_value(io, 0);
	nxp_soc_gpio_set_io_dir(io, 1);
	nxp_soc_gpio_set_io_func(io, 0);
#endif
	return 0;
}

static struct dw_mci_board _dwmci1_data = {
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION |
					DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps           = MMC_CAP_CMD23,
	.pm_caps        = MMC_PM_KEEP_POWER | MMC_PM_IGNORE_PM_NOTIFY,
	.detect_delay_ms= 200,
	.cd_type        = DW_MCI_CD_EXTERNAL,
	.init			= _dwmci1_init,
	.ext_cd_init	= _dwmci1_ext_cd_init,
	.ext_cd_cleanup	= _dwmci1_ext_cd_cleanup,
	.clk_dly		= DW_MMC_DRIVE_DELAY(0) | DW_MMC_SAMPLE_DELAY(0) | DW_MMC_DRIVE_PHASE(2) | DW_MMC_SAMPLE_PHASE(1),
};

#if defined(CONFIG_BROADCOM_WIFI) || defined(CONFIG_BCMDHD)

#include <linux/if.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>

static int _wifi_power(int on)
{
	printk("%s %d\n", __func__, on);
	nxp_soc_gpio_set_out_value(CFG_IO_MCU_WL_REG_ON, on);
	return 0;
}

static int _wifi_reset(int on)
{
	printk("%s %d\n", __func__, on);
	return 0;
}

extern struct platform_device dwmci_dev_ch1;
static int _wifi_set_carddetect(int val)
{
	printk("%s %d\n", __func__, val);

	if (wifi_status_cb)
	wifi_status_cb(&dwmci_dev_ch1, val);
	else
	printk("%s: Nobody to notify\n", __func__);;
	return 0;
}

static unsigned char _wifi_mac_addr[IFHWADDRLEN] = { 0, 0x90, 0x4c, 0, 0, 0 };

static int _wifi_get_mac_addr(unsigned char *buf)
{
	memcpy(buf, _wifi_mac_addr, IFHWADDRLEN);
	return 0;
}

static struct wifi_platform_data _wifi_control = {
	.set_power          = _wifi_power,
	.set_reset          = _wifi_reset,
	.set_carddetect     = _wifi_set_carddetect,
	.mem_prealloc       = NULL,
	.get_mac_addr       = _wifi_get_mac_addr,
	/*.get_country_code   = _wifi_get_country_code,*/
	.get_country_code   = NULL,
};

/* for OOB interrupt */
#ifdef CONFIG_BROADCOM_WIFI_USE_OOB
static struct resource brcm_wlan_resouces[] = {
    [0] = {
        .name = "bcmdhd_wlan_irq",
#if defined(CONFIG_OOB_INTR_ALIVE_0)
        .start = IRQ_ALIVE_0,
        .end   = IRQ_ALIVE_0,
#elif defined(CONFIG_OOB_INTR_ALIVE_1)
        .start = IRQ_ALIVE_1,
        .end   = IRQ_ALIVE_1,
#elif defined(CONFIG_OOB_INTR_ALIVE_2)
        .start = IRQ_ALIVE_2,
        .end   = IRQ_ALIVE_2,
#elif defined(CONFIG_OOB_INTR_ALIVE_3)
        .start = IRQ_ALIVE_3,
        .end   = IRQ_ALIVE_3,
#elif defined(CONFIG_OOB_INTR_ALIVE_4)
        .start = IRQ_ALIVE_4,
        .end   = IRQ_ALIVE_4,
#elif defined(CONFIG_OOB_INTR_ALIVE_5)
        .start = IRQ_ALIVE_5,
        .end   = IRQ_ALIVE_5,
#elif defined(CONFIG_OOB_INTR_ALIVE_6)
        .start = IRQ_ALIVE_6,
        .end   = IRQ_ALIVE_6,
#elif defined(CONFIG_OOB_INTR_ALIVE_7)
        .start = IRQ_ALIVE_7,
        .end   = IRQ_ALIVE_7,
#endif
        .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
    }
};
#endif

static struct platform_device bcm_wifi_device = {
	.name  = "bcmdhd_wlan",
	.id    = 0,
#ifdef CONFIG_BROADCOM_WIFI_USE_OOB
    .num_resources = ARRAY_SIZE(brcm_wlan_resouces),
    .resource = brcm_wlan_resouces,
#endif
	.dev   = {
		.platform_data = &_wifi_control,
	},
};

#endif /* CONFIG_BROADCOM_WIFI */
#endif

#ifdef CONFIG_MMC_NEXELL_CH2
static struct dw_mci_board _dwmci2_data = {
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION |
					  DW_MCI_QUIRK_HIGHSPEED |
				  	  DW_MMC_QUIRK_HW_RESET_PW |
				      DW_MCI_QUIRK_NO_DETECT_EBIT,
	.bus_hz			= 200 * 1000 * 1000,
	.hs_over_clk	=  50 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 | //MMC_CAP_1_8V_DDR |
					  MMC_CAP_NONREMOVABLE |
			 	  	  MMC_CAP_4_BIT_DATA | MMC_CAP_CMD23 |
				  	  MMC_CAP_ERASE | MMC_CAP_HW_RESET,
	//.caps2			= MMC_CAP2_PACKED_WR,
	.desc_sz		= 4,
	.detect_delay_ms= 200,
	.clk_dly		= DW_MMC_DRIVE_DELAY(0x0) | DW_MMC_SAMPLE_DELAY(0x1c) | 	\
						DW_MMC_DRIVE_PHASE(2) | DW_MMC_SAMPLE_PHASE(1),	// 100 = (1) (1), 200 = (0) (2)

};
#endif

#endif /* CONFIG_MMC_DW */


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

#if defined (CONFIG_NEXELL_DISPLAY_MIPI)
	nxp_platform_disp_device_data(DISP_DEVICE_MIPI, &mipi_vsync_param, (void*)&mipi_param, &mipi_syncgen_param);
#endif

#if defined(CONFIG_MMC_DW)
	#ifdef CONFIG_MMC_NEXELL_CH0
	nxp_mmc_add_device(0, &_dwmci0_data);
	#endif
	#ifdef CONFIG_MMC_NEXELL_CH1
	nxp_mmc_add_device(1, &_dwmci1_data);
        #if defined(CONFIG_BROADCOM_WIFI) || defined(CONFIG_BCMDHD)
    printk("plat: register broadcom wifi device\n");
    platform_device_register(&bcm_wifi_device);
        #endif
	#endif
	#ifdef CONFIG_MMC_NEXELL_CH2
	nxp_mmc_add_device(2, &_dwmci2_data);
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

#if defined(CONFIG_KEYBOARD_NEXELL_KEY) || defined(CONFIG_KEYBOARD_NEXELL_KEY_MODULE)
	printk("plat: add device keypad\n");
	platform_device_register(&key_plat_device);
#endif

#if defined(CONFIG_I2C_NEXELL)
    platform_add_devices(i2c_devices, ARRAY_SIZE(i2c_devices));
#endif

#if defined(CONFIG_REGULATOR_NXE2000)
	printk("plat: add device nxe2000 pmic\n");
	i2c_register_board_info(NXE2000_I2C_BUS, nxe2000_regulators, ARRAY_SIZE(nxe2000_regulators));
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
    printk("plat: add device nxp-v4l2\n");
    platform_device_register(&nxp_v4l2_dev);
#endif

#if defined(CONFIG_TOUCHSCREEN_AW5306)
	printk("plat: add touch(aw5306) device\n");
	i2c_register_board_info(AW5306_I2C_BUS, &aw5306_i2c_bdi, 1);
#endif

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT640T)
	printk("plat: add touch(ATMEL MXT604T) device\n");
	i2c_register_board_info(MXT604T_I2C_BUS, &mxt604t_i2c_bdi, 1);
#endif

#if 0//defined(CONFIG_PM_MICOM) || defined(CONFIG_PM_MICOM_MODULE)
	printk("plat: add pm-micom\n");
	i2c_register_board_info(MICOM_I2C_BUS, &micom_i2c_bdi, 1);
#endif

#if defined(CONFIG_SENSORS_BMA2X2) || defined(CONFIG_SENSORS_BMA2X2_MODULE)
	printk("plat: add accel-sensor(bma2x2)\n");
	i2c_register_board_info(BMA2X2_I2C_BUS, &bma2x2_i2c_bdi, 1);
#endif

#if defined(CONFIG_SENSORS_BMG) || defined(CONFIG_SENSORS_BMG_MODULE)
	printk("plat: add gyro-sensor(bmg160)\n");
	i2c_register_board_info(BMG160_I2C_BUS, &bmg160_i2c_bdi, 1);
#endif

#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	spi_register_board_info(spi_plat_board, ARRAY_SIZE(spi_plat_board));
	printk("plat: register spidev\n");
#endif

#if 0//defined(CONFIG_HAPTIC_DRV2605) || defined(CONFIG_HAPTIC_DRV2605_MODULE)
	printk("plat: add haptic(drv2605)\n");
	i2c_register_board_info(DRV2605_I2C_BUS, &drv2605_i2c_bdi, 1);
#endif

	/* END */
	printk("\n");
}
