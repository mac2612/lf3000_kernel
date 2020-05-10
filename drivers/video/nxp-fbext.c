/*
 * (C) Copyright 2010
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/div64.h>
#include <linux/major.h>
#include <linux/pm.h>

#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>
#include <mach/fourcc.h>

#ifdef CONFIG_FB_NEXELL_ION_MEM
#include <linux/dma-buf.h>
#include <linux/nxp_ion.h>
#include <linux/ion.h>
#include <linux/highmem.h>
#include <linux/memblock.h>

extern struct ion_device *get_global_ion_device(void);
#endif

#ifdef CONFIG_FB_NEXELL_LFEXT
#include <linux/lf1000/lf1000fb.h>
#endif

#if (0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "fb: " msg); }
#define	DUMP_VAR_SCREENINFO
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define	FB_DEV_PIXELCLOCK	(27000000)	/* 27Mhz */
#define	FB_CLEAR_COLOR		(0x0)
#define FB_PALETTE_CLEAR 	(0x80000000)

#ifdef CONFIG_FB_NEXELL_ION_MEM
struct dma_buf_context {
    struct ion_handle *ion_handle;
    struct dma_buf    *dma_buf;
    struct dma_buf_attachment *attachment;
    struct sg_table   *sg_table;
    dma_addr_t         dma_addr;
    void              *virt;
};
struct nxp_fb_dma_buf_data {
    struct ion_client *ion_client;
    struct dma_buf_context context[3];
};
#endif
/*------------------------------------------------------------------------------
 *
 */
struct nxp_fb_device {
	int 		  device_id;	/* device 0 or 1 */
	int			  layer;
	int			  vid_layer;	/* video layer priority */
	int			  x_resol_max;	/* x max resolution */
	int			  y_resol_max;	/* y max resolution */
	int			  x_resol;		/* x resolution */
	int			  y_resol;		/* y resolution */
	int			  x_virt;		/* virtual x resolution */
	int			  y_virt;		/* virtual y resolution */
	int			  buffer_num;
	int			  pixelbit;		/* bit per pixel */
	unsigned int  format;		/* frame buffer format */
	unsigned int  bgcolor;
	int			  lcd_w_mm;		/* width  mm for dpi */
	int			  lcd_h_mm;		/* hegiht  mm for dpi */
	unsigned int  hs_left;
	unsigned int  hs_right;
	unsigned int  vs_upper;
	unsigned int  vs_lower;
	unsigned int  pixelclk;
	/* frame buffer */
	unsigned int  fb_phy_base;
	unsigned int  fb_phy_end;
	unsigned int  fb_phy_len;
	void		 *fb_vir_base;
	unsigned int  fb_pan_phys;
	int			  fb_remapped;
	int			  skip_vsync;
	int			  rotate;		/* for LCD flip 180 */
#ifdef CONFIG_FB_NEXELL_ION_MEM
    struct device *dev;
    struct nxp_fb_dma_buf_data dma_buf_data;
#endif
};

struct nxp_fb_param {
	struct fb_info	*		info;
	u32						palette_buffer[256];
	u32						pseudo_pal[16];

	/* fb hw device */
	struct nxp_fb_device 	fb_dev;
	unsigned int			status;
};

#define	_R_BIT(bpi)	 (16 == bpi ? 5 : 8)
#define	_G_BIT(bpi)	 (16 == bpi ? 6 : 8)
#define	_B_BIT(bpi)	 (16 == bpi ? 5 : 8)
#define	_T_BIT(bpi)	 (32 == bpi ? 8 : 0)

#define	FB_STAT_INIT		(1)

#define FB_NUM_LAYERS		MULTI_LAYER_NUM
#define IS_YUV_LAYER(x)		(x == LAYER_VIDEO_IDX)

/*---------------------------------------------------------------------------------------------
 * 	FB device
 */
static int nxp_fb_setup_display_out(struct nxp_fb_param *par, int enable)
{
	int module = par->fb_dev.device_id;
	nxp_soc_disp_device_enable_all(module, enable);
	return 0;
}

//Determine format code
static int fb_get_var_format(struct fb_var_screeninfo *var)
{
	switch (var->bits_per_pixel) {
		case 1:
		case 2:
		case 4:
		case 8:
			return 0;
		case 16:
			if (var->transp.offset == 12) {
				if (var->blue.offset == 8)
					return MLC_RGBFMT_A4B4G4R4;
				return MLC_RGBFMT_A4R4G4B4;
			}
			if (var->blue.offset == 11)
				return MLC_RGBFMT_B5G6R5;
			return MLC_RGBFMT_R5G6B5;
		case 24:
			if (var->blue.offset == 16)
				return MLC_RGBFMT_B8G8R8;
			return MLC_RGBFMT_R8G8B8;
		case 32:
			if (var->transp.length == 0) {
				if (var->blue.offset == 16)
					return MLC_RGBFMT_X8B8G8R8;
				return MLC_RGBFMT_X8R8G8B8;
			}
			if (var->blue.offset == 16)
				return MLC_RGBFMT_A8B8G8R8;
			return MLC_RGBFMT_A8R8G8B8;
	}
	return 0;
}

static void inline
nxp_fb_init_display(struct fb_info *info)
{
	struct nxp_fb_param  *par = info->par;
	int module = par->fb_dev.device_id;
	int layer  = par->fb_dev.layer;
	int xres   = info->var.xres;
	int yres   = info->var.yres;
	int pixel  = info->var.bits_per_pixel >> 3;
	u32 base   = par->fb_dev.fb_phy_base;

	if (layer == 0) {
		u32 phys, stride;
		DBGOUT(" layer =%d ... don't do anything....\n", layer);
		nxp_soc_disp_rgb_get_address(module, layer, &phys, &pixel, &stride);
		info->var.yoffset = (phys - base) / stride;
		info->var.xoffset = ((phys - base) % stride) / pixel;
		if (info->var.xoffset + info->var.xres > info->var.xres_virtual)
			info->var.xoffset = info->var.xres_virtual - info->var.xres;
		if (info->var.yoffset + info->var.yres > info->var.yres_virtual)
			info->var.yoffset = info->var.yres_virtual - info->var.yres - 1;
		printk("%s: layer=%d, phys=0x%x, base=0x%x, offset=0x%x (%d,%d), stride=%d\n", __func__, layer, phys, base, phys-base, info->var.xoffset, info->var.yoffset, stride);
		nxp_soc_disp_rgb_set_enable(module, layer, 1);
		return;
	}

	if (IS_YUV_LAYER(layer)) {
		nxp_soc_disp_video_set_format(module, par->fb_dev.format, xres, yres);
		nxp_soc_disp_video_set_address(module, base, 4096, base+2048, 4096, base+2048+4096*yres/2, 4096, 0);
		nxp_soc_disp_video_set_position(module, 0, 0, xres, yres, 0);
		nxp_soc_disp_video_set_priority(module, par->fb_dev.vid_layer);
		nxp_soc_disp_video_set_enable(module, 0);
		goto out;
	}

	nxp_soc_disp_rgb_set_fblayer(module, layer);

	#if defined(CONFIG_LOGO_NEXELL_COPY)
	{
		unsigned phy = 0, vir = 0;
		unsigned vfb = (unsigned int)par->fb_dev.fb_vir_base;
		unsigned len = (xres * yres * pixel);
		nxp_soc_disp_rgb_get_address(module, layer, &phy, NULL, NULL);
		if (phy) {
			unsigned offs = (phy-0x40000000);
			vir = (unsigned)phys_to_virt(phy);
			printk("%s: BootLogo copy 0x%x (%d MB area)", info->fix.id, phy, (phy-0x40000000)>>20);
			if (((760-2)<<20) > offs) { /* Under 7(60 - FB size) MB */
				printk("to 0x%x (0x%x)\n", vfb, par->fb_dev.fb_phy_base);
				memcpy((void *)vfb, (const void *)vir, len);
			} else {
				printk("Fail, over phy area %dMB\n", ((760-2)<<20));
			}
		}
	}
	#endif

	#if !defined(CONFIG_BACKLIGHT_PWM)
	//nxp_fb_setup_display_out(par, 0);	/* display out : off */
    #endif

	nxp_soc_disp_set_bg_color(module, par->fb_dev.bgcolor);
	nxp_soc_disp_rgb_set_format(module, layer, fb_get_var_format(&info->var), xres, yres, pixel);
	nxp_soc_disp_rgb_set_position(module, layer, 0, 0, 0);
	nxp_soc_disp_rgb_set_address(module, layer, base, pixel, xres*pixel, 1);
	nxp_soc_disp_rgb_set_enable(module, layer, 1);

out:
	/* display out : on */
	if (!nxp_soc_disp_device_stat_enable(DISP_DEVICE_SYNCGEN0 + module))
		nxp_fb_setup_display_out(par, 1);

	par->status = FB_STAT_INIT;

	printk(KERN_INFO "%s: %d * %d - %d bpp, layer: %d (phys:%08x virt:0x%p max:%d)\n",
		info->fix.id, xres, yres, pixel<<3, layer,
		par->fb_dev.fb_phy_base, par->fb_dev.fb_vir_base, par->fb_dev.fb_phy_len);
}

static void inline
nxp_fb_setup_display(struct fb_info *info)
{
	struct nxp_fb_param  *par = info->par;

	int module = par->fb_dev.device_id;
	int layer  = par->fb_dev.layer;
	int xres   = info->var.xres;
	int yres   = info->var.yres;
	int pixel  = info->var.bits_per_pixel >> 3;
	u32 base   = par->fb_dev.fb_pan_phys;

	printk(KERN_INFO "%s: %d * %d - %d bpp (phys:%08x)\n",
		info->fix.id, xres, yres, pixel<<3,par->fb_dev.fb_phy_base);

	if (IS_YUV_LAYER(layer)) {
		int stride, format, x, y, w, h;
		nxp_soc_disp_video_get_format(module, &format, &xres, &yres);
		nxp_soc_disp_video_get_position(module, &x, &y, &w, &h);
		nxp_soc_disp_video_set_format(module, par->fb_dev.format, xres, yres);
		stride = (par->fb_dev.format == FOURCC_YUYV) ? xres * 2 : 4096;
		nxp_soc_disp_video_set_position(module, x, y, info->var.xres, info->var.yres, 0);
		nxp_soc_disp_video_set_address(module, base, stride, base+(stride>>1), stride, base+(stride>>1)+stride*yres/2, stride, 1);
		nxp_soc_disp_video_set_priority(module, par->fb_dev.vid_layer);
	}
	else {
		int x, y;
		nxp_soc_disp_rgb_get_position(module, layer, &x, &y);
		nxp_soc_disp_rgb_set_format(module, layer, fb_get_var_format(&info->var), xres, yres, pixel);
		nxp_soc_disp_rgb_set_position(module, layer, x, y, 0);
		nxp_soc_disp_rgb_set_address(module, layer, base, pixel, xres*pixel, 1);
	}
	par->status = FB_STAT_INIT;

	DBGOUT("DONE\n");
}

static void inline
nxp_fb_update_buffer(struct fb_info *info, int waitvsync)
{
	struct nxp_fb_param  *par = info->par;
	int module = par->fb_dev.device_id;
	int layer  = par->fb_dev.layer;
	int xres   = info->var.xres;
	int yres   = info->var.yres;
	int pixel  = info->var.bits_per_pixel >> 3;
	int pbase  = par->fb_dev.fb_pan_phys;

	waitvsync  = par->fb_dev.skip_vsync ? 0 : waitvsync;

	if (IS_YUV_LAYER(layer)) {
		int stride = (par->fb_dev.format == FOURCC_YUYV) ? xres * 2 : 4096;
		nxp_soc_disp_video_get_format(module, &par->fb_dev.format, &xres, &yres);
		stride = (par->fb_dev.format == FOURCC_YUYV) ? xres * 2 : 4096;
		nxp_soc_disp_video_set_address(module, pbase, stride, pbase+(stride>>1), stride, pbase+(stride>>1)+stride*yres/2, stride, waitvsync);
	}
	else
	if (par->status & FB_STAT_INIT)
		nxp_soc_disp_rgb_set_address(module, layer, pbase, pixel, xres*pixel, waitvsync);

	DBGOUT("%s: %s, phys=0x%08x (init=%s)\n", __func__,
		info->fix.id, pbase, par->status&FB_STAT_INIT?"yes":"no");
}

static int inline
nxp_fb_verify_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	return 0;
}

#ifdef DUMP_VAR_SCREENINFO
static void nxp_fb_dump_var(struct fb_var_screeninfo *var)
{
	DBGOUT(": var->xres                 = %d\n", var->xres);
	DBGOUT(": var->yres                 = %d\n", var->yres);
	DBGOUT(": var->xres_virtual         = %d\n", var->xres_virtual);
	DBGOUT(": var->yres_virtual         = %d\n", var->yres_virtual);
	DBGOUT(": var->xoffset              = %d\n", var->xoffset);
	DBGOUT(": var->yoffset              = %d\n", var->yoffset);
	DBGOUT(": var->bits_per_pixel       = %d\n", var->bits_per_pixel);
	DBGOUT(": var->grayscale            = %d\n", var->grayscale);
	DBGOUT(": var->nonstd               = %d\n", var->nonstd);
	DBGOUT(": var->activate             = %d\n", var->activate);
	DBGOUT(": var->height               = %d\n", var->height);
	DBGOUT(": var->width                = %d\n", var->width);
	DBGOUT(": var->accel_flags          = %d\n", var->accel_flags);
	DBGOUT(": var->pixclock             = %d\n", var->pixclock);
	DBGOUT(": var->left_margin          = %d\n", var->left_margin);
	DBGOUT(": var->right_margin         = %d\n", var->right_margin);
	DBGOUT(": var->upper_margin         = %d\n", var->upper_margin);
	DBGOUT(": var->lower_margin         = %d\n", var->lower_margin);
	DBGOUT(": var->hsync_len            = %d\n", var->hsync_len);
	DBGOUT(": var->vsync_len            = %d\n", var->vsync_len);
	DBGOUT(": var->sync                 = %d\n", var->sync);
	DBGOUT(": var->vmode                = %d\n", var->vmode);
	DBGOUT(": var->rotate               = %d\n", var->rotate);
	DBGOUT(": var->red.offset           = %d\n", var->red.offset);
	DBGOUT(": var->red.length           = %d\n", var->red.length);
	DBGOUT(": var->red.msb_right        = %d\n", var->red.msb_right);
	DBGOUT(": var->green.offset         = %d\n", var->green.offset);
	DBGOUT(": var->green.length         = %d\n", var->green.length);
	DBGOUT(": var->green.msb_right      = %d\n", var->green.msb_right);
	DBGOUT(": var->blue.offset          = %d\n", var->blue.offset);
	DBGOUT(": var->blue.length          = %d\n", var->blue.length);
	DBGOUT(": var->blue.msb_right       = %d\n", var->blue.msb_right);
	DBGOUT(": var->transp.offset        = %d\n", var->transp.offset);
	DBGOUT(": var->transp.length        = %d\n", var->transp.length);
	DBGOUT(": var->transp.msb_right     = %d\n", var->transp.msb_right);
}
#endif

/*---------------------------------------------------------------------------------------------
 * 	FB
 */
static struct fb_ops nxp_fb_ops;

static struct fb_info *
nxp_fb_init_fb(int fb, struct device *dev)
{
	struct fb_info * info = NULL;
	char name[256];

	/*
	 * 	allocate :
	 *	fb_info + sizeof(struct nxp_fb_param)
	 *	info->par = struct nxp_fb_param
	 *
	 */
	info = framebuffer_alloc(sizeof(struct nxp_fb_param), dev);
	if (! info) {
		printk(KERN_ERR "fail, unable to allocate frame buffer(%d) info ...\n", fb);
		return NULL;
	}

	/* fb_info members */
	sprintf(name, "%s %d", DEV_NAME_FB, fb);
	strcpy(info->fix.id, name);			/* name filed */

	/* default fixed information */
	info->fix.type	    	= FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux		= 0;
	info->fix.xpanstep		= 1;	/* not suppor horizontal pan (fb_mem.c) */
	info->fix.ypanstep		= 1;	/* when pan, check yoffset align with ypanstep value (fb_mem.c) */
	info->fix.ywrapstep		= 0;
	info->fix.accel	    	= FB_ACCEL_NONE;

	/* default variable information */
	info->var.nonstd	   	= 0;
	info->var.activate		= FB_ACTIVATE_NOW;
	info->var.accel_flags	= 0;
	info->var.vmode	    	= FB_VMODE_NONINTERLACED;

	/* link machind file operation to fb_info's file operation */
	info->fbops	= &nxp_fb_ops;
	info->flags	= FBINFO_FLAG_DEFAULT;

	/* default colormap: palette */
	if(fb_alloc_cmap(&info->cmap, 256, 0)) {
		printk(KERN_ERR "fail, unable to allocate cmap for frame buffer(%d) ...\n", fb);
		framebuffer_release(info);
		return NULL;
	}
	return info;
}

static void nxp_fb_exit_fb(struct fb_info *info)
{
	fb_dealloc_cmap(&info->cmap);
	framebuffer_release(info);
}

static void nxp_fb_setup_param(struct fb_info *info, void *data, int layer)
{
	struct nxp_fb_plat_data *plat = data;
	struct nxp_fb_param  *par = info->par;
	struct nxp_fb_device *dev = &par->fb_dev;
	struct disp_vsync_info vsync;
	int display_out = 0;
	int i;

	par->info   = info;
	par->status = 0;

	/* clear palette buffer */
	for(i = 0; i < 256; i++)
		par->palette_buffer[i] = FB_PALETTE_CLEAR;

	/* set hw variables */
	dev->device_id    = plat->module;
	dev->layer    	  = layer; //plat->layer;
	dev->x_resol_max  = plat->x_resol_max ? plat->x_resol_max : plat->x_resol;
	dev->y_resol_max  = plat->y_resol_max ? plat->y_resol_max : plat->y_resol;
	dev->x_resol 	  = plat->x_resol;
	dev->y_resol 	  = plat->y_resol;
	dev->x_virt		  = plat->x_resol;
	dev->y_virt		  = plat->y_resol * plat->buffers;
	dev->buffer_num	  = plat->buffers;
	dev->pixelbit 	  = plat->bitperpixel;
	dev->format		  = plat->format;
	dev->bgcolor	  = plat->bgcolor;
	dev->lcd_w_mm 	  = plat->lcd_with_mm;
	dev->lcd_h_mm	  = plat->lcd_height_mm;
	dev->fb_phy_base  = plat->fb_mem_base;
	dev->fb_phy_end   = plat->fb_mem_end;
	dev->fb_pan_phys  = plat->fb_mem_base;

	if (!plat->vsync) {
		display_out = 0 == plat->module ? DISP_DEVICE_SYNCGEN0 : DISP_DEVICE_SYNCGEN1;
		nxp_soc_disp_device_get_vsync(display_out, &vsync);
		plat->vsync = &vsync;
	}

	dev->hs_left	  = plat->vsync->h_sync_width + plat->vsync->h_back_porch;
	dev->hs_right	  = plat->vsync->h_front_porch;
	dev->vs_upper	  = plat->vsync->v_sync_width + plat->vsync->v_back_porch;
	dev->vs_lower	  = plat->vsync->v_front_porch;
	dev->pixelclk	  = plat->vsync->pixel_clock_hz ? plat->vsync->pixel_clock_hz : FB_DEV_PIXELCLOCK;
	dev->skip_vsync   = plat->skip_pan_vsync ? 1 : 0;
	dev->rotate       = 0;

	dev->fb_vir_base  = 0;
	dev->fb_remapped  = 0;

	if (IS_YUV_LAYER(dev->layer)) {
		dev->x_virt       =
		dev->x_resol_max  = 4096;
		dev->y_virt       =
		dev->y_resol_max  = 2048;
		dev->pixelbit     = 8;
		dev->format       = FOURCC_YV12;
		dev->buffer_num	  = 1;
		dev->vid_layer    = 2;
	}

	DBGOUT("%s (out dev=%d)\n", __func__, dev->device_id);
}

static void nxp_fb_setup_info(struct fb_info *info)
{
	struct nxp_fb_param  *par = info->par; /* get nxp_fb_param base */
	struct nxp_fb_device *dev = &par->fb_dev;

	int x_v, y_v, bpp = 0;

	DBGOUT("%s\n", __func__);

	bpp = dev->pixelbit;
	x_v = dev->x_resol > dev->x_virt ? dev->x_resol : dev->x_virt;
	y_v = dev->y_resol > dev->y_virt ? dev->y_resol : dev->y_virt;

	/* other variable information */
	info->var.width	    	= dev->lcd_w_mm; 	/* width  mm for dpi */
	info->var.height	    = dev->lcd_h_mm; 	/* height  mm for dpi */

	/* get frame rate */
	info->var.left_margin	= dev->hs_left;
	info->var.right_margin	= dev->hs_right;
	info->var.upper_margin	= dev->vs_upper;
	info->var.lower_margin	= dev->vs_lower;
	info->var.pixclock		= (1000000000 / dev->pixelclk) * 1000;	/* pico second */

	/* get resolution */
	info->var.xres	    	= dev->x_resol;
	info->var.yres	    	= dev->y_resol;
	info->var.bits_per_pixel= dev->pixelbit;
	info->var.xres_virtual 	= x_v;
	info->var.yres_virtual 	= y_v;

	/* get pixel format */
	info->var.red.offset  	= _G_BIT(bpp) + _B_BIT(bpp);
	info->var.green.offset  = _B_BIT(bpp);
	info->var.blue.offset   = 0;
	info->var.transp.offset = _T_BIT(bpp) ? (_R_BIT(bpp) + _G_BIT(bpp) + _B_BIT(bpp)) : 0;
	info->var.red.length    = _R_BIT(bpp);
	info->var.green.length  = _G_BIT(bpp);
	info->var.blue.length   = _B_BIT(bpp);
	info->var.transp.length = _T_BIT(bpp);

	/* other palette & fixed */
	info->pseudo_palette  = &par->pseudo_pal;
	info->fix.smem_len    = x_v * y_v * (bpp >> 3);
	info->fix.line_length = (info->var.xres * info->var.bits_per_pixel) >> 3;

	if (IS_YUV_LAYER(dev->layer)) {
		info->fix.smem_len    = 4096 * y_v;
		info->fix.line_length = (dev->format == FOURCC_YUYV) ? x_v * 2 : 4096;
		info->fix.type	      = FB_TYPE_PLANES;
		info->var.nonstd      = (dev->format == FOURCC_YUYV ? LAYER_FORMAT_YUV422 : LAYER_FORMAT_YUV420) << LF1000_NONSTD_FORMAT;
		info->var.nonstd     |= (dev->vid_layer << LF1000_NONSTD_PRIORITY);
	}

	DBGOUT("res: %d by %d, virtual: %d by %d, length:%d\n",
		dev->x_resol, dev->y_resol, x_v, y_v, info->fix.smem_len);
}

#ifdef CONFIG_FB_NEXELL_ION_MEM
static int nxp_fb_setup_ion(struct nxp_fb_dma_buf_data *d)
{
    struct ion_device *ion_dev = get_global_ion_device();

    if (!ion_dev) {
        pr_err("%s error: no ion device!!!\n", __func__);
        return -EINVAL;
    }

    d->ion_client = ion_client_create(ion_dev,
            // ION_HEAP_NXP_CONTIG_MASK,  /* FIXME: sesters, removed from Android */
            "nxp-fb");
    if (IS_ERR(d->ion_client)) {
        pr_err("%s error: fail to ion_client_create()\n", __func__);
        return -EINVAL;
    }

    return 0;
}

static int nxp_fb_map_ion_handle(struct nxp_fb_device *fb_dev,
        struct dma_buf_context *ctx,
        struct ion_handle *ion_handle, int fd)
{
    int ret = 0;

    ctx->dma_buf = dma_buf_get(fd);
    if (IS_ERR_OR_NULL(ctx->dma_buf)) {
        pr_err("%s error: fail to dma_buf_get(%d)\n", __func__, fd);
        return -EINVAL;
    }

    ctx->attachment = dma_buf_attach(ctx->dma_buf, fb_dev->dev);
    if (IS_ERR_OR_NULL(ctx->attachment)) {
        pr_err("%s error: fail to dma_buf_attach()\n", __func__);
        ret = -EINVAL;
        goto err_attachment;
    }

    ctx->sg_table = dma_buf_map_attachment(ctx->attachment,
            DMA_BIDIRECTIONAL);
    if (IS_ERR_OR_NULL(ctx->sg_table)) {
        pr_err("%s error: fail to dma_buf_map_attachment()\n", __func__);
        ret = -EINVAL;
        goto err_map_attachment;
    }

    ctx->dma_addr = sg_phys(ctx->sg_table->sgl);
    ctx->virt     = sg_virt(ctx->sg_table->sgl);
	ctx->ion_handle = ion_handle;

    printk(KERN_INFO "%s %d: dma addr = 0x%x\n",
    	DEV_NAME_FB, fb_dev->device_id, ctx->dma_addr);
    return 0;

err_map_attachment:
    dma_buf_detach(ctx->dma_buf, ctx->attachment);
    ctx->attachment = NULL;
err_attachment:
    dma_buf_put(ctx->dma_buf);
    ctx->dma_buf = NULL;
    return ret;
}

static void nxp_fb_free_dma_buf(struct nxp_fb_device *fb_dev,
        struct nxp_fb_dma_buf_data *d)
{
    int i;
    struct dma_buf_context *ctx;

    for (i = 0, ctx = &d->context[0]; i < fb_dev->buffer_num; i++, ctx++) {
        if (!ctx->dma_addr)
            continue;

        dma_buf_unmap_attachment(ctx->attachment, ctx->sg_table,
                DMA_BIDIRECTIONAL);
        ctx->dma_addr = 0;
        ctx->virt     = NULL;
        ctx->sg_table = NULL;
        dma_buf_detach(ctx->dma_buf, ctx->attachment);
        ctx->attachment = NULL;
        dma_buf_put(ctx->dma_buf);
        ctx->dma_buf = NULL;
        ion_free(d->ion_client, ctx->ion_handle);
        ctx->ion_handle = NULL;
    }
}

static int __init nxp_fb_ion_alloc_mem(struct nxp_fb_device *fb_dev)
{
    int ret;
    int fd;
    unsigned int size;
    struct file *file;
    struct ion_handle *handle;
    struct nxp_fb_dma_buf_data *d = &fb_dev->dma_buf_data;
    struct dma_buf_context *ctx;
    int length = ((fb_dev->x_resol_max * fb_dev->y_resol_max * fb_dev->pixelbit)>>3);
    int i;

    size = PAGE_ALIGN(length);

    for (i = 0, ctx = &d->context[0]; i < fb_dev->buffer_num; i++, ctx++) {
        handle = ion_alloc(d->ion_client, (size_t)size, 0,
                ION_HEAP_NXP_CONTIG_MASK, 0);
        if (IS_ERR(handle)) {
            pr_err("%s error: fail to ion_alloc()\n", __func__);
            return -ENOMEM;
        }

        fd = ion_share_dma_buf(d->ion_client, handle);
        if (fd < 0) {
            pr_err("%s error: fail to ion_share_dma_buf()\n", __func__);
            ret = -EINVAL;
            goto err_share_dma_buf;
        }

        ret = nxp_fb_map_ion_handle(fb_dev, ctx, handle, fd);
        if (ret) {
            pr_err("%s error: fail to nxp_fb_map_ion_handle()\n", __func__);
            goto err_map;
        }
        // clear fb
        //memset(ctx->virt, 0, size);
    }
    return 0;

err_map:
    file = fget(fd);
    fput(file);
    fput(file);
err_share_dma_buf:
    ion_free(d->ion_client, handle);

    return ret;
}
#endif // CONFIG_FB_NEXELL_ION_MEM

static int nxp_fb_alloc_mem(struct fb_info *info)
{
	struct nxp_fb_param  *par = info->par;
	struct nxp_fb_device *dev = &par->fb_dev;
	unsigned int length;

	length = ((dev->x_resol_max * dev->y_resol_max * dev->pixelbit)>>3) *
			  dev->buffer_num;
	if(! length)
		return 0;

	DBGOUT("%s: %s fb%d (%d * %d - %d bpp), len:%d, align:%d, fb_phys=%u\n",
		__func__, dev_name(par->info->device), dev->device_id,
		dev->x_resol_max, dev->y_resol_max, dev->pixelbit,
		length, PAGE_ALIGN(length), dev->fb_phy_base);

#ifdef CONFIG_FB_NEXELL_ION_MEM
    if (nxp_fb_ion_alloc_mem(dev)) {
        printk(KERN_ERR "fail to nxp_fb_ion_alloc_mem()\n");
        return -ENOMEM;
    }

    dev->fb_phy_base = dev->dma_buf_data.context[0].dma_addr;
    dev->fb_pan_phys = dev->fb_phy_base;
    dev->fb_phy_len  = PAGE_ALIGN(length);
    dev->fb_vir_base = dev->dma_buf_data.context[0].virt;
    dev->fb_remapped = 0;
#else
	if (dev->fb_phy_base) {
		/*
		 * remmap fb memory
		 */
		 if ((dev->fb_phy_base + length) > dev->fb_phy_end) {
		 	printk(KERN_ERR "Error: request fb mem 0x%x~0x%x execced fb region 0x%x~0x%x\n",
		 		dev->fb_phy_base, dev->fb_phy_base + length,
		 		dev->fb_phy_base, dev->fb_phy_end);
		 	return -ENOMEM;
		 }
		 dev->fb_vir_base = ioremap_nocache(dev->fb_phy_base, length);
		 dev->fb_pan_phys = dev->fb_phy_base;
		 dev->fb_phy_len  = PAGE_ALIGN(length);
		 dev->fb_remapped = 1;
	} else {
		struct device *device = par->info->device;
		/*
		 * allocate from system memory
		 */
		dev->fb_phy_len  = PAGE_ALIGN(length);
		dev->fb_vir_base = dma_alloc_coherent(device, dev->fb_phy_len,
								&dev->fb_phy_base, GFP_KERNEL);
		dev->fb_pan_phys = dev->fb_phy_base;
		dev->fb_remapped = 0;
	}
#endif

#if 0  // FIXME: obsolete on LF3000?
	if (IS_YUV_LAYER(dev->layer))
		dev->fb_pan_phys = dev->fb_phy_base |= 0x20000000UL;
#endif

	if(dev->fb_vir_base) {
		memset((void*)dev->fb_vir_base, FB_CLEAR_COLOR, dev->fb_phy_len);
		info->screen_base	 = dev->fb_vir_base;
		info->fix.smem_start = dev->fb_phy_base;
	}

	return dev->fb_vir_base ? 0 : -ENOMEM;
}

static void nxp_fb_free_mem(struct fb_info *info)
{
	struct nxp_fb_param *par = info->par;
	struct nxp_fb_device *dev = &par->fb_dev;
	DBGOUT("%s\n", __func__);

	if(dev->fb_vir_base) {
#ifdef CONFIG_FB_NEXELL_ION_MEM
        nxp_fb_free_dma_buf(dev, &dev->dma_buf_data);
#else
		if (dev->fb_remapped) {
			iounmap(dev->fb_vir_base);
		} else {
			struct device *device = par->info->device;
			dma_free_writecombine(device, dev->fb_phy_len,
					dev->fb_vir_base, dev->fb_phy_base);
		}
#endif
		dev->fb_vir_base = 0;
		dev->fb_remapped  = 0;
	}
}

static int nxp_fb_set_var_pixfmt(struct fb_var_screeninfo *var, struct fb_info *info)
{
	DBGOUT("%s (bpp:%d)\n", __func__, var->bits_per_pixel);

	switch (var->bits_per_pixel) {
		case 1:
		case 2:
		case 4:
			var->red.offset    	= 0;
			var->red.length    	= var->bits_per_pixel;
			var->green         	= var->red;
			var->blue          	= var->red;
			var->transp.offset 	= 0;
			var->transp.length 	= 0;
			break;
		case 8:
			var->red.offset    	= 0;
			var->red.length    	= var->bits_per_pixel;
			var->green         	= var->red;
			var->blue          	= var->red;
			var->transp.offset 	= 0;
			var->transp.length 	= 0;
			break;
		case 16:
			/* 16 bpp, 565 format */
			var->red.length		= 5;
			var->red.offset		= 11;
			var->green.length	= 6;
			var->green.offset	= 5;
			var->blue.length	= 5;
			var->blue.offset	= 0;
			var->transp.length	= 0;
			break;
		case 24:
			/* 24 bpp 888 */
			var->red.length		= 8;
			var->red.offset		= 16;
			var->green.length	= 8;
			var->green.offset	= 8;
			var->blue.length	= 8;
			var->blue.offset	= 0;
			var->transp.length	= 0;
			break;
		case 32:
			/* 32 bpp 888 */
			var->red.length		= info->var.red.length;
			var->red.offset		= info->var.red.offset;
			var->green.length	= info->var.green.length;
			var->green.offset	= info->var.green.offset;
			var->blue.length	= info->var.blue.length;
			var->blue.offset	= info->var.blue.offset;
			var->transp.length	= info->var.transp.length;
			var->transp.offset	= info->var.transp.offset;
			break;
		default:
			printk(KERN_ERR "error, not support fb bpp (%d)\n", var->bits_per_pixel);
	}

	return 0;
}

/*---------------------------------------------------------------------------------------------
 * 	FB ops functions
 */
/*
 *	nxp_fb_check_var:
 *	Get the video params out of 'var'. If a value doesn't fit, round it up,
 *	if it's too big, return -EINVAL.
 *
 */
static int nxp_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	int ret = 0;
	DBGOUT("%s (xres:%d, yres:%d, bpp:%d)\n",
		__func__, var->xres, var->yres, var->bits_per_pixel);

	ret = nxp_fb_verify_var(var, info);
	if (0 > ret)
		return ret;

	return ret;
}

/*
 *	nxp_fb_set_par - Optional function. Alters the hardware state.
 *  @info: frame buffer structure that represents a single frame buffer
 *
 */
extern void lcd_flip(int module, int flip);
static int nxp_fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	struct nxp_fb_param  *par = info->par;
	struct nxp_fb_device *dev = &par->fb_dev;
	bool change_pending = false;

	DBGOUT("%s (xres:%d, yres:%d, bpp:%d)\n",
		__func__, var->xres, var->yres, var->bits_per_pixel);

	switch (var->bits_per_pixel) {
		case 32:
		case 24:
		case 16:
			info->fix.visual = FB_VISUAL_TRUECOLOR;
			break;
		case 1:
			info->fix.visual = FB_VISUAL_MONO01;
			break;
		default:
			info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
			break;
	}

	info->fix.line_length = (var->xres * var->bits_per_pixel) >> 3;
	info->fix.ypanstep	  = 1;	/* when pan, check yoffset align with ypanstep value (fb_mem.c) */

	if (IS_YUV_LAYER(dev->layer)) {
		dev->vid_layer = NONSTD_TO_POS(var->nonstd);
		dev->format    = (NONSTD_TO_PFOR(var->nonstd) == LAYER_FORMAT_YUV422) ? FOURCC_YUYV : FOURCC_YV12;
		change_pending = true;
		info->fix.line_length = (dev->format == FOURCC_YUYV) ? var->xres * 2 : 4096;
	}

#if defined(CONFIG_NEXELL_DISPLAY_LCD)
	if (var->rotate != dev->rotate) {
		change_pending = true;
		lcd_flip(0, var->rotate);
		dev->rotate = var->rotate;
		printk("%s: rotate=%d\n", __func__, var->rotate);
	}
#endif

	change_pending = true;
    if (change_pending ||
        dev->x_resol != var->xres ||
        dev->y_resol != var->yres ||
        dev->pixelbit != var->bits_per_pixel) {
        printk("%s: resetting xres(%d) yres(%d) bps(%d)\n",
                __func__, var->xres, var->yres, var->bits_per_pixel);

#if 0 /* RA: Do not reset/set preset, was done in u-boot */
#if defined(CONFIG_NEXELL_SOC_DISP_HDMI)
        // HDMI case
        if (dev->layer == 0) {
            printk("%s: resetting HDMI mode\n", __func__);
        	nxp_soc_disp_hdmi_enable(false);
        	if (var->xres <= 720 && var->yres <= 480)
        		nxp_soc_disp_hdmi_set_preset(NXP_HDMI_PRESET_480P);
        	else if (var->xres <= 1280 && var->yres <= 800)
        		nxp_soc_disp_hdmi_set_preset(NXP_HDMI_PRESET_720P);
        	else
        		nxp_soc_disp_hdmi_set_preset(NXP_HDMI_PRESET_1080P);
        }
        dev->x_virt	     = dev->x_resol_max;
        dev->y_virt	     = dev->y_resol_max * dev->buffer_num;
        info->fix.line_length = (dev->x_resol_max * var->bits_per_pixel) >> 3;
#endif
#endif
        dev->x_resol     = var->xres;
        dev->y_resol     = var->yres;
        dev->pixelbit    = var->bits_per_pixel;
        //dev->x_virt	     = dev->x_resol;
        //dev->y_virt	     = dev->y_resol * dev->buffer_num;
        dev->fb_pan_phys = dev->fb_phy_base;	/* pan restore */

        nxp_fb_setup_display(info);

#if 0 /* RA: Do not reset/set preset, was done in u-boot */
#if defined(CONFIG_NEXELL_SOC_DISP_HDMI)
        // HDMI case
        if (dev->layer == 0) {
            printk("%s: re-enabling HDMI mode\n", __func__);
        	nxp_soc_disp_hdmi_enable(true);
        }
        info->fix.line_length = (dev->x_resol_max * info->var.bits_per_pixel) >> 3;
#endif
#endif
    } else {
        if (par->status != FB_STAT_INIT) {
            nxp_fb_setup_display(info);
        }
        else if (IS_YUV_LAYER(dev->layer)) {
        	info->fix.line_length = (dev->format == FOURCC_YUYV) ? var->xres * 2 : 4096;
        	info->fix.xpanstep    = 1;
        }
    }

#ifdef DUMP_VAR_SCREENINFO
	nxp_fb_dump_var(var);
#endif
	return 0;
}

/**
 *  nxp_fb_blank
 *	@blank_mode: the blank mode we want.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Blank the screen if blank_mode != 0, else unblank. Return 0 if
 *	blanking succeeded, != 0 if un-/blanking failed due to e.g. a
 *	video mode which doesn't support it. Implements VESA suspend
 *	and powerdown modes on hardware that supports disabling hsync/vsync:
 *	blank_mode == 2: suspend vsync
 *	blank_mode == 3: suspend hsync
 *	blank_mode == 4: powerdown
 *
 *	Returns negative errno on error, or zero on success.
 *
 */
static int nxp_fb_blank(int blank_mode, struct fb_info *info)
{
	struct nxp_fb_param  *par = info->par;
	int module = par->fb_dev.device_id;
	int layer  = par->fb_dev.layer;

	DBGOUT("%s (mode=%d)\n", __func__, blank_mode);

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		if (IS_YUV_LAYER(layer))
			nxp_soc_disp_video_set_enable(module, 1);
		else
		nxp_soc_disp_rgb_set_enable(module, layer, 1);
		break;
	case FB_BLANK_NORMAL:
		if (IS_YUV_LAYER(layer))
			nxp_soc_disp_video_set_enable(module, 0);
		else
		nxp_soc_disp_rgb_set_enable(module, layer, 0);
		break;
	}

	/* en/disable LCD */
	return 0;
}

/**
 *	nxp_fb_setcolreg
 *	- framebuffer layer request to change palette.
 * 	@regno: The palette index to change.
 * 	@red: The red field for the palette data.
 * 	@green: The green field for the palette data.
 * 	@blue: The blue field for the palette data.
 * 	@trans: The transparency (alpha) field for the palette data.
 * 	@info: The framebuffer being changed.
 */
inline static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static void schedule_palette_update(struct fb_info *info, unsigned int regno, unsigned int val)
{
	unsigned long flags;
	struct nxp_fb_param *par = info->par;

	local_irq_save(flags);
	par->palette_buffer[regno] = val;
	local_irq_restore(flags);
}

static int nxp_fb_setcolreg(unsigned regno,
			       unsigned red, unsigned green, unsigned blue,
			       unsigned transp, struct fb_info *info)
{
	struct nxp_fb_param *par = info->par;
	unsigned int val;
/*
	DBGOUT("%s (setcol: regno=%d, r=0x%x, g=0x%x, b=0x%x, t=0x%x)\n",
		__func__, regno, red, green, blue, transp);
*/
	switch (par->info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/* true-colour, use pseuo-palette */
		if (regno < 16) {
			u32 *pal = par->info->pseudo_palette;
			val  = chan_to_field(red,   &par->info->var.red);
			val |= chan_to_field(green, &par->info->var.green);
			val |= chan_to_field(blue,  &par->info->var.blue);
			pal[regno] = val;
		}
		break;
	case FB_VISUAL_PSEUDOCOLOR:
		if (regno < 256) {
			/* currently assume RGB 5-6-5 mode */
			val  = ((red   >>  0) & 0xf800);
			val |= ((green >>  5) & 0x07e0);
			val |= ((blue  >> 11) & 0x001f);
			schedule_palette_update(info, regno, val);
		}
		break;
	default:
		printk(KERN_ERR "fail, setcolreg return unknown type\n");
		return 1;   /* unknown type */
	}
	return 0;
}

int nxp_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	/* not implementation */
	return 0;
}

static int nxp_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct nxp_fb_param  *par = info->par;
	struct nxp_fb_device *dev = &par->fb_dev;
	unsigned int offset = 0;
	unsigned int fb_old = dev->fb_pan_phys;
	unsigned int align  = (dev->x_resol * dev->y_resol * dev->pixelbit) >> 3;

	offset = (var->yoffset * info->fix.line_length) +
			 (var->xoffset * (var->bits_per_pixel>>3));
#if 0 // disabled for arbitrary panning
	if (offset % align)
		offset = (offset + align - 1)/align * align;
#else
	(void)align;
#endif

#ifdef CONFIG_FB_NEXELL_ION_MEM
    {
        //unsigned int index = offset / align;
        //Arbitrary panning for ION, is the second buffer always right after the first?
        dev->fb_pan_phys = dev->dma_buf_data.context[0].dma_addr + offset;
    }
#else
	dev->fb_pan_phys = info->fix.smem_start + offset;
#endif

	DBGOUT("%s (offset:0x%x, %s)\n",
		__func__, offset, fb_old!=dev->fb_pan_phys?"up":"pass");

	/* change window layer base */
	if (fb_old != dev->fb_pan_phys)
		nxp_fb_update_buffer(info, 1);

	return 0;
}

#ifdef CONFIG_FB_NEXELL_ION_MEM
#define NXPFB_GET_FB_FD _IOWR('N', 101, __u32)
#define NXPFB_SET_FB_FD _IOW('N', 102, __u32)
#endif

#ifdef CONFIG_FB_NEXELL_LFEXT
#define FBIOCG_BGCOLOR _IOR('C', 110, __u32)
#define FBIOCS_BGCOLOR _IOW('C', 111, __u32)
#endif

static int nxp_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
	struct nxp_fb_param *par = info->par;
	struct nxp_fb_device *dev = &par->fb_dev;
#ifdef CONFIG_FB_NEXELL_LFEXT
	void __user *argp = (void __user *)arg;
	union lf1000fb_cmd c, s;
	int format, width, height, depth, enable;
	int layer = dev->layer; //IS_YUV_LAYER(dev->layer) ? dev->layer : NONSTD_TO_POS(info->var.nonstd);
	int module = par->fb_dev.device_id;
	int pbase  = par->fb_dev.fb_pan_phys;
	unsigned int bgcolor;
#endif

	DBGOUT("%s (cmd:0x%x, type:%c, nr:%d) \n\n", __func__, cmd, _IOC_TYPE(cmd), _IOC_NR(cmd));

#ifdef CONFIG_FB_NEXELL_ION_MEM
    switch (cmd) {
    case NXPFB_GET_FB_FD:
        {
            int index;
            struct nxp_fb_dma_buf_data *d = &dev->dma_buf_data;
            int fd;

            if (get_user(index, (u32 __user *)arg)) {
                ret = -EFAULT;
                break;
            }
            printk("%s: NXPFB_GET_FB_FD index(%d)\n", __func__, index);
            fd = dma_buf_fd(d->context[index].dma_buf, 0);
            if (fd < 0) {
                printk("%s NXPFB_GET_FB_FD failed: fail to dma_buf_fd()\n", __func__);
                ret = -EINVAL;
            } else {
                printk("fd: %d\n", fd);
                if (put_user(fd, (int __user *)arg)) {
                    ret = -EFAULT;
                    break;
                }
                printk("success!!!\n");
            }
        }
        break;
	#if 0
    case NXPFB_SET_FB_FD:
        {
            u32 import_fd;
            struct ion_handle *handle;
            struct nxp_fb_dma_buf_data dma_buf_data;
            struct nxp_fb_dma_buf_data *d = &dev->dma_buf_data;

            if (get_user(import_fd, (u32 __user *)arg)) {
                ret = -EFAULT;
                break;
            }

            handle = ion_import_dma_buf(d->ion_client, import_fd);
            if (IS_ERR(handle)) {
                pr_err("%s error: NXPFB_SET_FB_FD, fail to ion_import_dma_buf()\n", __func__);
                ret = PTR_ERR(handle);
                break;
            }

            if (nxp_fb_map_ion_handle(dev, &dma_buf_data, handle, import_fd)) {
                pr_err("%s error: NXPFB_SET_FB_FD, fail to nxp_fb_map_ion_handle()\n", __func__);
                ion_free(d->ion_client, handle);
                ret = -EINVAL;
                break;
            }

            nxp_fb_update_from_dma_buf_data(info, &dma_buf_data);
            nxp_fb_copy_dma_buf_data(dev, &dma_buf_data);
        }
        break;
	#endif
    }
#endif
#ifdef CONFIG_FB_NEXELL_LFEXT
	switch (cmd) {
		case FBIO_WAITFORVSYNC:
			// return vsync status without waiting
			return NX_MLC_GetDirtyFlag(module, layer);
			break;

		case LF1000FB_IOCSALPHA:
			if (!(_IOC_DIR(cmd) & _IOC_WRITE))
				return -EFAULT;
			if (copy_from_user((void *)&c, argp, sizeof(struct lf1000fb_blend_cmd)))
				return -EFAULT;
			if (IS_YUV_LAYER(dev->layer))
				nxp_soc_disp_video_set_color(module, VIDEO_COLOR_ALPHA, c.blend.alpha, c.blend.enable);
			else
				nxp_soc_disp_rgb_set_color(module, layer, RGB_COLOR_ALPHA, c.blend.alpha, c.blend.enable);
			break;

		case LF1000FB_IOCGALPHA:
			if (!(_IOC_DIR(cmd) & _IOC_READ))
				return -EFAULT;
			if (IS_YUV_LAYER(dev->layer))
				c.blend.alpha = nxp_soc_disp_video_get_color(module, VIDEO_COLOR_ALPHA);
			else
				c.blend.alpha = nxp_soc_disp_rgb_get_color(module, layer, RGB_COLOR_ALPHA);
			if (copy_to_user(argp, (void *)&c, sizeof(struct lf1000fb_blend_cmd)))
				return -EFAULT;
			break;

		case LF1000FB_IOCSPOSTION:
			if (!(_IOC_DIR(cmd) & _IOC_WRITE))
				return -EINVAL;
			if (copy_from_user((void *)&c, argp, sizeof(struct lf1000fb_position_cmd)))
				return -EFAULT;
			if (IS_YUV_LAYER(dev->layer))
				nxp_soc_disp_video_set_position(module, c.position.left, c.position.top, c.position.right, c.position.bottom, false);
			else {
				nxp_soc_disp_rgb_get_format(module, layer, &format, &width, &height, &depth);
				width = c.position.right - c.position.left;
				height = c.position.bottom - c.position.top;
				enable = nxp_soc_disp_rgb_stat_enable(module, layer);
				nxp_soc_disp_rgb_set_enable(module, layer, 0);
				nxp_soc_disp_rgb_set_format(module, layer, format, width, height, depth);
				nxp_soc_disp_rgb_set_position(module, layer, c.position.left, c.position.top, false);
				nxp_soc_disp_rgb_set_enable(module, layer, enable);
			}
			break;

		case LF1000FB_IOCGPOSTION:
			if (!(_IOC_DIR(cmd) & _IOC_READ))
				return -EINVAL;
			if (IS_YUV_LAYER(dev->layer))
				nxp_soc_disp_video_get_position(module, &c.position.left, &c.position.top, &c.position.right, &c.position.bottom);
			else {
				nxp_soc_disp_rgb_get_format(module, layer, &format, &width, &height, &depth);
				nxp_soc_disp_rgb_get_position(module, layer, &c.position.left, &c.position.top);
				c.position.right = c.position.left + width;
				c.position.bottom = c.position.top + height;
			}
			if (copy_to_user(argp, (void *)&c, sizeof(struct lf1000fb_position_cmd)))
				return -EFAULT;
			break;

		case LF1000FB_IOCSVIDSCALE:
			if (!(_IOC_DIR(cmd) & _IOC_WRITE))
				return -EINVAL;
			if (copy_from_user((void *)&c, argp, sizeof(struct lf1000fb_vidscale_cmd)))
				return -EFAULT;
			if (!IS_YUV_LAYER(dev->layer))
				return -EINVAL;
			nxp_soc_disp_video_get_position(module, &s.position.left, &s.position.top, &s.position.right, &s.position.bottom);
			nxp_soc_disp_video_set_format(module, dev->format, c.vidscale.sizex, c.vidscale.sizey);
			nxp_soc_disp_video_set_position(module, s.position.left, s.position.top, s.position.right, s.position.bottom, false);
			if (dev->format == FOURCC_YUYV)
				nxp_soc_disp_video_set_address(module, pbase, c.vidscale.sizex * 2, pbase+2048, c.vidscale.sizex * 2, pbase+2048+c.vidscale.sizex * 2*c.vidscale.sizey/2, c.vidscale.sizex * 2, 0);
			else
				nxp_soc_disp_video_set_address(module, pbase, 4096, pbase+2048, 4096, pbase+2048+4096*c.vidscale.sizey/2, 4096, 0);
			break;

		case LF1000FB_IOCGVIDSCALE:
			if (!(_IOC_DIR(cmd) & _IOC_READ))
				return -EINVAL;
			if (!IS_YUV_LAYER(dev->layer))
				return -EINVAL;
			nxp_soc_disp_video_get_format(module, &format, &c.vidscale.sizex, &c.vidscale.sizey);
			if (copy_to_user(argp, (void *)&c, sizeof(struct lf1000fb_vidscale_cmd)))
				return -EFAULT;
			break;

		case FBIOGET_VBLANK:
			{
			struct fb_vblank vblank;
			memset(&vblank, 0, sizeof(vblank));
			vblank.flags = FB_VBLANK_HAVE_COUNT;
			vblank.count = nxp_soc_disp_stat_vertical_sync(module);
			if (copy_to_user(argp, (void *)&vblank, sizeof(struct fb_vblank)))
				return -EFAULT;
			}
			break;

		case FBIOCG_BGCOLOR:
			bgcolor = dev->bgcolor;
			if (copy_to_user(argp, (void *)&bgcolor, sizeof(bgcolor)))
				return -EFAULT;
			break;

		case FBIOCS_BGCOLOR:
			if (!(_IOC_DIR(cmd) & _IOC_WRITE))
				return -EFAULT;
			if (copy_from_user((void *)&bgcolor, argp, sizeof(bgcolor)))
				return -EFAULT;
			if (IS_YUV_LAYER(dev->layer))
				return -EFAULT;
			else {
				dev->bgcolor = bgcolor;
				nxp_soc_disp_set_bg_color(module, bgcolor);
			}
			break;

		default:
			return -ENOIOCTLCMD;
	}
#endif
	return ret;
}

static struct fb_ops nxp_fb_ops = {
	.owner			= THIS_MODULE,
	.fb_check_var	= nxp_fb_check_var,
	.fb_set_par		= nxp_fb_set_par,
	.fb_blank		= nxp_fb_blank,
	.fb_setcolreg	= nxp_fb_setcolreg,
	.fb_cursor		= nxp_fb_cursor,		/* Optional !!! */
	.fb_pan_display	= nxp_fb_pan_display,
	.fb_ioctl		= nxp_fb_ioctl,
	/* Call FB function */
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

/*--------------------------------------------------------------------------------
 * Platform driver
 */
static int nxp_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int nxp_fb_resume(struct platform_device *pdev)
{
	return 0;
}

static int nxp_fb_probe(struct platform_device *pdev)
{
	struct nxp_fb_plat_data * plat = pdev->dev.platform_data;
	struct fb_info *info = NULL;
	int ret = 0;
	int i;
    struct nxp_fb_param *par = NULL; //info->par;
    struct nxp_fb_device *fb_dev = NULL; //&par->fb_dev;
    struct nxp_fb_device *fb_dev_layer = NULL;

	DBGOUT("\n%s (name=%s, id=%d)\n", __func__, dev_name(&pdev->dev), pdev->id);

  for (i = 0; i < FB_NUM_LAYERS; i++) {

	/*	allocate fb_info and init */
	info = nxp_fb_init_fb(pdev->id, &pdev->dev);

	if(! info) {
		ret = -ENOMEM;
		goto err_fb;
	}

	nxp_fb_setup_param(info, plat, i);
	nxp_fb_setup_info (info);

    par = info->par;
    fb_dev = &par->fb_dev;

#ifdef CONFIG_FB_NEXELL_ION_MEM
    {
        struct nxp_fb_param *par = info->par;
        struct nxp_fb_device *fb_dev = &par->fb_dev;
        fb_dev->dev = &pdev->dev;
        if (1 == i)
        	fb_dev->dma_buf_data = fb_dev_layer->dma_buf_data;
        else
        	ret = nxp_fb_setup_ion(&par->fb_dev.dma_buf_data);
    }
    if (ret) {
        printk(KERN_ERR "fail, fail to setup ion\n");
        goto err_map;
    }
#endif

	/*	allocate frame buffer memory from here */
    if (1 == i) {
    	/* upper and lower RGB layers share framebuffer memory */
	    fb_dev->fb_phy_base = fb_dev_layer->fb_phy_base;
	    fb_dev->fb_pan_phys = fb_dev_layer->fb_pan_phys;
	    fb_dev->fb_phy_len  = fb_dev_layer->fb_phy_len ;
	    fb_dev->fb_vir_base = fb_dev_layer->fb_vir_base;
	    fb_dev->fb_remapped = fb_dev_layer->fb_remapped;
		info->screen_base	 = fb_dev_layer->fb_vir_base;
		info->fix.smem_start = fb_dev_layer->fb_phy_base;
		ret = 0;
    }
    else
    	ret = nxp_fb_alloc_mem(info);
	if(ret) {
		printk(KERN_ERR "fail, unable to allcate frame buffer (%d)\n", pdev->id);
		goto err_map;
	}
	nxp_fb_init_display(info);

	fb_dev_layer = fb_dev;

	/*
 	 * 	device_create '/proc/fb0' & fb class
	 * 	register machine file operation to frame buffer file operation
 	 * 	registered_fb[]
 	 * 	(drivers/video/fbmem.c)
 	 */
	ret = register_framebuffer(info);
	if(ret < 0) {
		printk(KERN_ERR "fail, unable to register frame buffer(%d)\n", pdev->id);
		goto err_reg;
	}

  } /* for */

	/* register to driver data, use platform_get_drvdata */
	platform_set_drvdata(pdev, info);
	return ret;

err_reg:
	unregister_framebuffer(info);
err_map:
	nxp_fb_free_mem(info);
err_fb:
	nxp_fb_exit_fb(info);

	return ret;
}

static int nxp_fb_remove(struct platform_device *pdev)
{
	struct fb_info   * info = platform_get_drvdata(pdev);

	DBGOUT("%s\n", __func__);

	unregister_framebuffer(info);
	nxp_fb_free_mem(info);
	nxp_fb_exit_fb(info);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver fb_plat_driver = {
	.probe		= nxp_fb_probe,
	.remove		= __devexit_p(nxp_fb_remove),
	.suspend	= nxp_fb_suspend,
	.resume		= nxp_fb_resume,
	.driver		= {
		.name	= DEV_NAME_FB,
		.owner	= THIS_MODULE,
	},
};

static int __init nxp_fb_init(void)
{
	DBGOUT("%s\n", __func__);
	return platform_driver_register(&fb_plat_driver);
}

static void __exit nxp_fb_exit(void)
{
	DBGOUT("%s\n", __func__);
	platform_driver_unregister(&fb_plat_driver);
}

module_init(nxp_fb_init);
module_exit(nxp_fb_exit);

MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("Framebuffer driver for the Nexell");
MODULE_LICENSE("GPL");

