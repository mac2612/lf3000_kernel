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

/*------------------------------------------------------------------------------
 * Frame Buffer (Primary)
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
 * MIPI device
 */
#if defined (CONFIG_NEXELL_SOC_DISP_MIPI)
#include <linux/delay.h>

#define	MIPI_BITRATE_750M

#ifdef MIPI_BITRATE_1G
#define	PLLPMS		0x033E8
#define	BANDCTL		0xF
#elif defined(MIPI_BITRATE_750M)
#define	PLLPMS		0x043E8
#define	BANDCTL		0xC
#elif defined(MIPI_BITRATE_420M)
#define	PLLPMS		0x2231
#define	BANDCTL		0x7
#elif defined(MIPI_BITRATE_402M)
#define	PLLPMS		0x2219
#define	BANDCTL		0x7
#endif
#define	PLLCTL		0
#define	DPHYCTL		0

static void mipilcd_write( unsigned int id, unsigned int data0, unsigned int data1 )
{
    U32 index = 0;
    volatile NX_MIPI_RegisterSet* pmipi =
    	(volatile NX_MIPI_RegisterSet*)IO_ADDRESS(NX_MIPI_GetPhysicalAddress(index));
    pmipi->DSIM_PKTHDR = id | (data0<<8) | (data1<<16);
}

static int LD070WX3_SL01(int width, int height, void *data)
{
	msleep(10);
    mipilcd_write(0x15, 0xB2, 0x7D);
    mipilcd_write(0x15, 0xAE, 0x0B);
    mipilcd_write(0x15, 0xB6, 0x18);
    mipilcd_write(0x15, 0xD2, 0x64);
	msleep(10);

	return 0;
}

struct disp_syncgen_par mipi_syncgen_par = {
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
};

static struct disp_mipi_param mipi_param = {
	.pllpms 	= PLLPMS,
	.bandctl	= BANDCTL,
	.pllctl		= PLLCTL,
	.phyctl		= DPHYCTL,
	.lcd_init	= LD070WX3_SL01
};

#endif /* MIPI */

/*------------------------------------------------------------------------------
 *	platform devices
 */
static void __init nxp_fb_device_register(void)
{
#if defined(CONFIG_FB_NEXELL)
	printk("plat: add device frame buffer\n");
	platform_add_devices(fb_devices, ARRAY_SIZE(fb_devices));
#endif

#if defined (CONFIG_NEXELL_SOC_DISP_MIPI)
	printk("plat: add device mipi \n");
	nxp_platform_disp_device_data(DISP_DEVICE_MIPI, NULL, (void*)mipi_param, mipi_syncgen_par);
#endif
};
