/*
 * Hynix YAC-family Camera Driver
 *
 * Justin Eno <jeno@leapfrog.com>
 *
 * Copyright (c) 2011-2012, LeapFrog Enterprises, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Structure originally based on ov772x driver, functionality derived from
 * Nexell sr300pc10 decoder driver.
 *
 * This driver supports image sensors in the Hynix YAC* family.
 * Supported sensors:
 *	-2.0MP Hynix Hi-253 (YACD511SBDBC)	(basis of driver)
 *	-3.0MP SiliconFile SR300PC10		(partial support, untested)
 *	-1.3MP Hynix Hi-161 (YACC6A1BDBS)	(stub support, untested)
 *
 * (As of this writing, SiliconFile is a subsidiary of Hynix.)
 *
 * This driver is written to be able to support any imager compatible
 * with the Hi-253.  The list above includes only known variants.
 * 
 * NOTE: hardware appears similar to sr030pc30.c.
 * NOTE: hardware appears similar to sr200pc10 in Samsung GT-I5500 (europa_rev02_defconfig):
 *  git://opensource.samsung.com/GT-I5500/GT-I5500_Kernel/kernel/drivers/media/video/msm/sr200pc10*
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>
#include <linux/gcd.h>
#ifdef CONFIG_SOC_CAMERA_HYNIX_DEBUGFS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#endif
#ifdef CONFIG_SOC_CAMERA_HYNIX_SYSFS
#include <linux/sysfs.h>
#endif

#if defined(CONFIG_PLAT_NXP3200_RIO)
#define FLIP(x)		(x | 0x03)
#else
#define FLIP(x)		(x)
#endif

#if defined(CONFIG_PLAT_NXP3200_VALENCIA_CIP) || defined(CONFIG_PLAT_NXP3200_RIO)
#define PLLX(x)		(x | 0x02)
#else
#define PLLX(x)		(x | 0x03)
#endif

#include <asm/div64.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/soc_camera.h>

#include <media/hynix_yac.h>
#include "hynix_yac_dat.h"
#include "hynix_yac_regs.h"

#include "hi253_default.h"
#include "hi253_capture.h"

#if defined(DEBUG)
#define ASSERT(x)	BUG_ON(!(x))
#else
#define ASSERT(x)
#endif

/* Table 2. AC Characteristics */
#define PCLK_MAX_HZ \
	72000000

#define MCLK_MIN_HZ \
	18000000

#define MCLK_MAX_HZ \
	48000000

/* 4.16. Timing Description */
#define OPCLK_MIN_HZ \
	12000000

/* ISPCTL1 formats */
static const enum v4l2_mbus_pixelcode hynix_fmts[] = {
	V4L2_MBUS_FMT_YUYV8_2X8,
	V4L2_MBUS_FMT_YVYU8_2X8,
	V4L2_MBUS_FMT_UYVY8_2X8,
	V4L2_MBUS_FMT_VYUY8_2X8,
	/* TODO:
	 * V4L2_MBUS_FMT_RGB565_2X8_LE,
	 * V4L2_MBUS_FMT_BGR565_2X8_LE,
	 * V4L2_MBUS_FMT_RGB444_2X8_PADHI_LE,
	 */
};

/*
 * Unfortunately, the register pages differ slightly between imager models.
 * This is dealt with by reg_map[][] below.  Worse, some register addresses
 * also differ.  This is dealt with by _$(MODEL) suffixes on register names.
 */

/* Sensors - used as indicies to mapping table */
enum sensor_type {
	SR300PC10	= 0,
	HI253,
	HI161,
};

static const u8 reg_map[][NO_SUCH_PAGE] = {
	[SR300PC10]	= {
		[WINDOW_PAGE] 		= 0x00,
		[FORMAT_PAGE] 		= 0x10,
		[NOISE1_PAGE] 		= 0x11,
		[DEBLUR_PAGE] 		= 0x12,
		[NOISE2_PAGE] 		= 0x13,
		[EDGE_PAGE] 		= 0x14,
		[LENS_PAGE] 		= 0x15,
		[COLOR_PAGE] 		= 0x16,
		[GAMMA_PAGE] 		= 0x17,
		[FLICKER_PAGE] 		= 0x18,
		[SCALING_PAGE] 		= 0x19,
		[AUTOEXP_PAGE] 		= 0x20,
		[AUTOWB_PAGE] 		= 0x22,
		[AUTOFOCUS_PAGE] 	= 0x24,
		[MCU_PAGE] 		= 0x30,

		[UNDOC_0x02_PAGE]	= 0x02,
	},
	[HI253]	= {
		[WINDOW_PAGE]		= 0x00,
		[FORMAT_PAGE]		= 0x10,
		[NOISE1_PAGE]		= 0x11,
		[NOISE2_PAGE]		= 0x12,
		[EDGE_PAGE]		= 0x13,
		[LENS_PAGE]		= 0x14,
		[COLOR_PAGE]		= 0x15,
		[GAMMA_PAGE]		= 0x16,
		[FLICKER_PAGE]		= 0x17,
		[SCALING_PAGE]		= 0x18,
		[AUTOEXP_PAGE]		= 0x20,
		[AUTOWB_PAGE]		= 0x22,
		[AUTOFOCUS_PAGE]	= 0x24,

		[UNDOC_0x02_PAGE]	= 0x02,
		[UNDOC_0x03_PAGE]	= 0x03,
	},
	[HI161]	= {
		[WINDOW_PAGE]		= 0x00,
		[FORMAT_PAGE]		= 0x10,
		[NOISE1_PAGE]		= 0x11,
		[NOISE2_PAGE]		= 0x12,
		[EDGE_PAGE]		= 0x13,
		[LENS_PAGE]		= 0x14,
		[COLOR_PAGE]		= 0x15,
		[GAMMA_PAGE]		= 0x16,
		[FLICKER_PAGE]		= 0x17,
		[SCALING_PAGE]		= 0x18,
		[AUTOEXP_PAGE]		= 0x20,
		[AUTOWB_PAGE]		= 0x22,
	},
};

/*
 * Frame rate selection is achieved by setting the EXP* registers in the
 * autoexposure page.  These settings ultimately depend on OPCLK, so we
 * modify OPCLK as necessary to achieve arbitrary framerates for a given
 * sub-sampling (full-scale, 1/4 scale, 1/16 scale).
 *
 * A list of valid an unique "clock_combo"s are dynamically generated during
 * probe(), which we later search to find a suitable OPCLK.
 */
struct clock_combo {
	u8 sub_sample;	/* VDOCTL1[5:4] */
	u8 clk_div;	/* SYNCCTL[1:0] */
	u8 pll_scale;	/* PLLCTL1[2:0], 1 means off */
	u32 opclk_hz;	/* pre-calculated for convenience */
};

#define	PREVIEW_NONE	0
#define PREVIEW_PRE1	1
#define PREVIEW_PRE2	2
/* TODO: SR300PC10 PREVIEW_PRE3 (VDOCTL[1:0] = 0x2) for VGA mode */
#define PREVIEW_PRE3	3

/* indexed by PREVIEW_* */
static const u16 vertical_lines[][4] = {
	[SR300PC10] = {
		1584,
		800,
		400, /* TODO: verify */
		538, /* TODO: VGA mode */
	},
	[HI253] = {
		1248,
		632,
		316,
		/* Unsupported? */
	},
	[HI161] = {
		1052,
		528,
		264, /* TODO: verify */
		/* Unsupported? */
	},
};

static const u16 horizontal_width[] = {
	[SR300PC10]	= 2088,	/* Datasheet uses 1640 in register defs? */
	[HI253]		= 1640,
	[HI161]		= 1320,
};

/*
 * Frame rate selection is achieved by applying any register settings that
 * differ from the initialization sequence (e.g., hi253_svga_18fps_fixed[][]),
 * plus any settings that might have been modified by a previous frame rate.
 * This implies that all frame rate sequences must modifify the same registers.
 *
 * SR300PC10 is not supported.
 */
struct frame_rate {
	const struct v4l2_fract		tpf;		/* time per frame */
	const bool			variable;	/* fixed? */
	const char			(*seq)[3];	/* registers settings */
	int				len;		/* length of sequence */
};

static const struct frame_rate hi253_uxga[] = {
#if 0
	/* UXGA, 7.5 fps fixed */
	{
		.tpf.numerator		= 1000,
		.tpf.denominator	= 7500,
		.variable		= false,
		.seq			= hi253_uxga_7_5fps_fixed,
		.len			= ARRAY_SIZE(hi253_uxga_7_5fps_fixed),
	},
	/* UXGA, 7.5 fps variable */
	{
		.tpf.numerator		= 1000,
		.tpf.denominator	= 7500,
		.variable		= true,
		.seq			= hi253_uxga_7_5fps_variable,
		.len			= ARRAY_SIZE(hi253_uxga_7_5fps_variable),
	},
#endif
	/* UXGA, 15 fps fixed */
	{
		.tpf.numerator		= 1000,
		.tpf.denominator	= 15000,
		.variable		= false,
		.seq			= HI253_Capture_UXGA, //hi253_uxga_15fps_fixed,
		.len			= ARRAY_SIZE(HI253_Capture_UXGA), //ARRAY_SIZE(hi253_uxga_15fps_fixed),
	},
};

static const struct frame_rate hi253_svga[] = {
	/* SVGA, 15 fps fixed */
	{
		.tpf.numerator		= 1000,
		.tpf.denominator	= 15000,
		.variable		= false,
		.seq			= HI253_Capture_SVGA, //hi253_svga_15fps_fixed,
		.len			= ARRAY_SIZE(HI253_Capture_SVGA), //ARRAY_SIZE(hi253_svga_15fps_fixed),
	},
#if 0
	/* SVGA, 15 fps variable */
	{
		.tpf.numerator		= 1000,
		.tpf.denominator	= 15000,
		.variable		= true,
		.seq			= hi253_svga_15fps_variable,
		.len			= ARRAY_SIZE(hi253_svga_15fps_variable),
	},
	/* SVGA, 30 fps fixed */
	{
		.tpf.numerator		= 1000,
		.tpf.denominator	= 30000,
		.variable		= false,
		.seq			= hi253_svga_30fps_fixed,
		.len			= ARRAY_SIZE(hi253_svga_30fps_fixed),
	},
#endif
};

static const struct frame_rate hi253_q_svga[] = {
	/* 1/4-SVGA, 15 fps fixed */
	{
		.tpf.numerator		= 1000,
		.tpf.denominator	= 15000,
		.variable		= false,
		.seq			= HI253_Capture_QSVGA, //hi253_q_svga_15fps_fixed,
		.len			= ARRAY_SIZE(HI253_Capture_QSVGA), //ARRAY_SIZE(hi253_q_svga_15fps_fixed),
	},
#if 0
	/* 1/4-SVGA, 15 fps variable */
	/* TODO: sync problem! */
	{
		.tpf.numerator		= 1000,
		.tpf.denominator	= 15000,
		.variable		= true,
		.seq			= hi253_q_svga_15fps_variable,
		.len			= ARRAY_SIZE(hi253_q_svga_15fps_variable),
	},
	/* 1/4-SVGA, 30 fps fixed */
	{
		.tpf.numerator		= 1000,
		.tpf.denominator	= 30000,
		.variable		= false,
		.seq			= hi253_q_svga_30fps_fixed,
		.len			= ARRAY_SIZE(hi253_q_svga_30fps_fixed),
	},
#endif
};

/*
 * Resolutions supported directly by sub-sampling (and binning?).
 * Arbitrary smaller resolutions can be supported with downscaling (zoom) and
 * cropping (window).
 */

/*
 * Previewing, binning, and sub-sampling and not explained in the datasheet:
 * -It appears that sub-sampling and binning cannot be applied simultaneously.
 * -It appears that preview mode 3 is required for all binning modes.
 * -It appears that binning modes 1 & 2 bias toward the imager's top-left corner
 * 
 */

struct hynix_yac_frmsize {
	const u16		width;
	const u16		height;
	const u8		vdoctl;
	const struct frame_rate	*frame_rates;
	const int		num_frame_rates;
};

static const struct hynix_yac_frmsize sr300pc10_sizes[] = {
	/* No sub-sampling: QXGA */
	{
		.width	= 2048,
		.height	= 1536,
		.vdoctl	= 0x00,
	},
	/* 1/2 sub-sampling: XGA */
	{
		.width	= 1024,
		.height	= 768,
		.vdoctl	= 0x10,
	},
#if 0 /* TODO */
	/* No sub-sampling, Binning 1 or 2: VGA */
	{
		.width 	= 640,
		.height	= 480,
		.vdoctl	= 0xC2,
	},
#endif
	/* 1/4 sub-sampling: 1/4 XGA */
	{
		.width	= 512,
		.height	= 384,
		.vdoctl	= 0x20,
	},
#if 0 /* TODO */
	/* No sub-sampling, Binning 1 or 2: QVGA */
	{
		.width 	= 320,
		.height	= 240,
		.vdoctl	= 0x82,	/* also 0x42? */
	},
#endif
};

static const struct hynix_yac_frmsize hi253_sizes[] = {
	/* No sub-sampling: UXGA */
	{
		.width			= 1600,
		.height			= 1200,
		.vdoctl			= 0x00,
		.frame_rates		= hi253_uxga,
		.num_frame_rates	= ARRAY_SIZE(hi253_uxga),
	},
	/* 1/2 sub-sampling: SVGA */
	{
		.width			= 800,
		.height			= 600,
		.vdoctl			= 0x10,
		.frame_rates		= hi253_svga,
		.num_frame_rates	= ARRAY_SIZE(hi253_svga),
	},
	/* 1/4 sub-sampling: 1/4 SVGA */
	{
		.width			= 400,
		.height			= 300,
		.vdoctl			= 0x20,
		.frame_rates		= hi253_q_svga,
		.num_frame_rates	= ARRAY_SIZE(hi253_q_svga),
	},
};

static const struct hynix_yac_frmsize hi161_sizes[] = {
	/* No sub-sampling: SXGA */
	{
		.width			= 1280,
		.height			= 1024,
		.vdoctl			= 0x00,
	},
	/* 1/2 sub-sampling: VGA */
	{
		.width			= 640,
		.height			= 480,
		.vdoctl			= 0x10,
	},
	/* 1/4 sub-sampling: QVGA */
	{
		.width			= 320,
		.height			= 240,
		.vdoctl			= 0x20,
	},
};

/* Master data structure */
struct hynix_priv {
	struct v4l2_subdev		subdev;
	struct v4l2_ctrl_handler	hdl;
	struct hynix_yac_camera_info	*info;

	/* clocks */
	struct clock_combo		*clks;
	int				num_clks;

	/* resolutions */
	const struct hynix_yac_frmsize	*frmsizes;
	unsigned int			num_frmsizes;

	/* register access stuff */
	enum sensor_type		type;
	enum register_page		curr_page;

	/* current settings */
	const struct hynix_yac_frmsize	*curr_frmsize;
	struct v4l2_mbus_framefmt 	mf;  /* resolution, colorspace, etc. */
	struct v4l2_captureparm		parm; /* time per frame, ext. mode */

#ifdef CONFIG_SOC_CAMERA_HYNIX_DEBUGFS
	struct dentry			*debug;
#endif

#ifdef CONFIG_SOC_CAMERA_HYNIX_SYSFS
	rwlock_t			sysfs_lock;
#endif
};

/* Local-scope prototypes */
static int hynix_switch_page(struct i2c_client *client,
				const enum register_page page);
static int hynix_read(struct i2c_client *client, const enum register_page page,
				const u8 addr, u8 *val);
static int hynix_find_frmsize(const struct hynix_priv *priv,
				const __u32 width, const __u32 height );
static int apply_seq(struct i2c_client *client, char (*seq)[3], int len);
static int fetch_seq(struct i2c_client *client, char (*seq)[3], int len);

#ifdef CONFIG_SOC_CAMERA_HYNIX_DEBUGFS

/* instead of "gcc --combine" */
#include "hynix_yac_debug.c"

#endif

#ifdef CONFIG_SOC_CAMERA_HYNIX_SYSFS

/* instead of "gcc --combine" */
#include "hynix_yac_sysfs.c"

#endif

static inline struct hynix_priv *to_hynix_i2c(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct hynix_priv,
			    subdev);
}

static inline struct hynix_priv *to_hynix_sd(const struct v4l2_subdev *sd)
{
	return container_of(sd, struct hynix_priv, subdev);
}

/* returns 0 or -errno */
static int hynix_switch_page(struct i2c_client *client,
				const enum register_page page)
{
	struct hynix_priv *priv = to_hynix_i2c(client);
	u8 buf[2] = {PAGEMODE, reg_map[priv->type][page]};
	int ret;

	/* TODO: lock-protect this? */
	if(priv->curr_page != page)
	{
		if(2 != (ret = i2c_master_send(client, buf, 2)))
			goto out;
		priv->curr_page = page;
//		printk("%s: %02x: %02x=%02x, ret=%d\n", __func__, (int)page, buf[0], buf[1], ret);
	}
	ret = 0;
out:
	if (ret != 0)
		printk("%s: err=%d\n", __func__, ret);
	return ret;
}

/* returns 0 or -errno */
static int hynix_read(struct i2c_client *client, const enum register_page page,
				const u8 addr, u8 *val)
{
	int ret;

	if(0 != (ret = hynix_switch_page(client, page)))
		goto out;

	if(1 != (ret = i2c_master_send(client, &addr, 1)))
		goto out;

	ret = i2c_master_recv(client, val, 1);
//	printk("%s: %02x: %02x=%02x, ret=%d\n", __func__, (int)page, addr, *val, ret);

	ret = (ret == 1) ? 0 : ret;
out:
	if (ret != 0)
		printk("%s: err=%d\n", __func__, ret);
	return ret;
}

/* returns 0 or -errno */
static int hynix_write(struct i2c_client *client, const enum register_page page,
				const u8 addr, const u8 val)
{
	int ret = 0;
	u8 buf[2] = {addr, val};

	if(addr == PAGEMODE)
		goto out;

	if(0 != (ret = hynix_switch_page(client, page)))
		goto out;

	ret = i2c_master_send(client, buf, 2);
//	printk("%s: %02x: %02x=%02x, ret=%d\n", __func__, (int)page, addr, val, ret);

	ret = (ret == 2) ? 0 : ret;
out:
	if (ret != 0)
		printk("%s: err=%d\n", __func__, ret);
	return ret;
}

/*
 * Performs read-modify-write on register.
 * returns 0 or -errno
 */
static int hynix_rmw(struct i2c_client *client,
			const enum register_page page, const u8 addr,
			const u8 set, const u8 clear)
{
	int ret = -1;
	u8 tmp = 0;

	if(0 != (ret = hynix_read(client, page, addr, &tmp))) {
		dev_err(&client->dev, "hynix_rmw: read err %d\n", ret);
		goto out;
	}

	tmp &= ~clear;
	tmp |= set;

	if(0 != (ret = hynix_write(client, page, addr, tmp))) {
		dev_err(&client->dev, "hynix_rmw: write err %d\n", ret);
		goto out;
	}

out:
	return ret;
}

static int hynix_reset(struct i2c_client *client)
{
	struct hynix_priv *priv = to_hynix_i2c(client);
	int ret;
	
	dev_info(&client->dev, "hynix: %s\n", __FUNCTION__);

	/* Force page switch */
	priv->curr_page = NO_SUCH_PAGE;

	/* Apply soft reset */
	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, PWRCTL, 0x02, 0x00))) {
		dev_err(&client->dev, "hynix soft reset %d\n", ret);
		goto out;
	}

	/* Release reset */
	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, PWRCTL, 0x00, 0x02))) {
		dev_err(&client->dev, "hynix release reset %d\n", ret);
		goto out;
	}

out:
	return ret;
}

static int fetch_seq(struct i2c_client *client, char (*seq)[3], int len)
{
	int ret, i;

	for(i = 0; i < len; i++)
	{
		if(0 != (ret = hynix_read(client,
			seq[i][0],
			seq[i][1],
			&seq[i][2])))
			break;
//		udelay(1000);
	}

	return ret;
}

static int apply_seq(struct i2c_client *client, char (*seq)[3], int len)
{
	int ret, i;

	for(i = 0; i < len; i++)
	{
		if(0 != (ret = hynix_write(client,
			seq[i][0],
			seq[i][1],
			seq[i][2])))
			break;
//		udelay(1000);
	}

	return ret;
}

static int hynix_init_seq(struct i2c_client *client)
{
	struct hynix_priv *priv = to_hynix_i2c(client);
	int ret, i, len;
	int ret2, retry = 3;
	u8  query;
	const char (*seq)[3];

	switch(priv->type) {
	case SR300PC10:
		seq = my_hynix_1024x768; 
		len = ARRAY_SIZE(my_hynix_1024x768);
		priv->mf.width		= 1024;
		priv->mf.height		= 768;
		priv->parm.timeperframe.numerator	= 1000;
		priv->parm.timeperframe.denominator	= 10000;
		priv->frmsizes		= sr300pc10_sizes;
		priv->num_frmsizes	= ARRAY_SIZE(sr300pc10_sizes);
		break;
	case HI253:
		seq = HI253_Default; //HI253_Best_Initial;
		len = ARRAY_SIZE(HI253_Default); //ARRAY_SIZE(HI253_Best_Initial);
		priv->mf.width		= 800;
		priv->mf.height		= 600;
		/* TODO: calculate a precise fps value */
		priv->parm.timeperframe.numerator	= 1000;
		priv->parm.timeperframe.denominator	= 15000;
		priv->frmsizes		= hi253_sizes;
		priv->num_frmsizes	= ARRAY_SIZE(hi253_sizes);

		i = hynix_find_frmsize(priv, priv->mf.width, priv->mf.height);
		priv->curr_frmsize	= &hi253_sizes[i];
		break;
	case HI161:
		priv->frmsizes		= hi161_sizes;
		priv->num_frmsizes	= ARRAY_SIZE(hi161_sizes);
		break;
	}
	priv->parm.capability = V4L2_CAP_TIMEPERFRAME;

	do {
		ret = apply_seq(client, seq, len);
		if (ret)
			dev_err(&client->dev, "%s: apply_seq error = %d, retry %d\n", __FUNCTION__, ret, retry);

		ret2 = hynix_read(client, WINDOW_PAGE, PLLCTL1, &query);
		if (ret2 != 0 || query != PLLX(0x70)) {
			dev_err(&client->dev, "%s: pll query error = %d, %02x, retry %d\n", __FUNCTION__, ret2, query, retry);
			ret = -EIO;
		}
	} while (ret != 0 && --retry);

	priv->mf.code		= V4L2_MBUS_FMT_YUYV8_2X8; 
	priv->mf.field		= V4L2_FIELD_NONE;  /* progressive-scan only */
	priv->mf.colorspace	= V4L2_COLORSPACE_SMPTE170M;

	return ret;
}

static int hynix_set_itu656(struct i2c_client *client)
{
	struct hynix_priv *priv = to_hynix_i2c(client);
	int ret;
	u8 tmp;

	/* Enable ITU656 */
	if(0 != (ret = hynix_rmw(client, FORMAT_PAGE, ISPCTL1, 0x04, 0x00)))
		goto out;

	/* Enable codes */
	if(0 != (ret = hynix_rmw(client, FORMAT_PAGE, ITUCTL, 0x05, 0x00)))
		goto out;

	priv->mf.colorspace = V4L2_COLORSPACE_SMPTE170M;

	/* Set codes */
	tmp = priv->info->sof;
	if(0 != (ret = hynix_write(client, FORMAT_PAGE, ITUSOF, tmp)))
		goto out;

	tmp = priv->info->sol;
	if(0 != (ret = hynix_write(client, FORMAT_PAGE, ITUSOL, tmp)))
		goto out;

	tmp = priv->info->eof;
	if(0 != (ret = hynix_write(client, FORMAT_PAGE, ITUEOF, tmp)))
		goto out;

	tmp = priv->info->eol;
	if(0 != (ret = hynix_write(client, FORMAT_PAGE, ITUEOL, tmp)))
		goto out;

out:
	return ret;
}

static int hynix_video_probe(struct i2c_client *client)
{
	struct hynix_priv *priv = to_hynix_i2c(client);
	int ret;
	u8 devid;

	//dev_dbg(&client->dev, "%s\n", __FUNCTION__);
	dev_info(&client->dev, "hynix: %s\n", __FUNCTION__);
	

	/* Need an intial register map to identify sensor */
	priv->type = SR300PC10;

	/* Apply soft reset */
	if(0 != (ret = hynix_reset(client))) {
		dev_err(&client->dev, "hynix reset %d\n", ret);
		goto out;
	}

	if(0 != (ret = hynix_read(client, WINDOW_PAGE, DEVID, &devid))) {
		dev_err(&client->dev, "hynix read devid %d\n", ret);
		goto out;
	}

	switch(devid) {
	case HI253_ID:
		priv->type = HI253;
		break;
	case SR300PC10_ID:
		priv->type = SR300PC10;
		break;
	case HI161_ID:
		priv->type = HI161;
		break;
	default:
		ret = -ENODEV;
		dev_err(&client->dev, "Unrecognized sensor.\n");
		goto out;
	}

	dev_info(&client->dev, "Device ID: %#x\n", devid);

	if(0 != (ret = hynix_init_seq(client)))
	{
		dev_err(&client->dev, "Initialization sequence failed.\n");
		goto out;
	}

	if(priv->info->flags & HYNIX_YAC_ITU656)
	{
		if(0 != (ret = hynix_set_itu656(client)))
		{
			dev_err(&client->dev, "Could not set ITU656.\n");
			goto out;
		}
	}
	else
	{
		/* TODO */
		hynix_rmw(client, FORMAT_PAGE, ISPCTL1, 0x00, 0x04);
	}

	ret = v4l2_ctrl_handler_setup(&priv->hdl);
	if (0 != ret)
		dev_err(&client->dev, "v4l2_ctrl_handler_setup error %d\n", ret);

out:
	return ret;
}

static int hynix_video_remove(struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int ret;

	ret = v4l2_subdev_call(sd, video, s_stream, 0);

	return ret;
}

static int hynix_s_ctrl(struct v4l2_ctrl *ctrl)
{
	printk(KERN_INFO "hynix_s_ctrl: %d, %d\n", ctrl->id, ctrl->val);
	return 0;
}

static const struct v4l2_ctrl_ops hynix_ctrl_ops = {
	.s_ctrl = NULL, //hynix_s_ctrl,
};

static struct v4l2_subdev_core_ops hynix_subdev_core_ops = {
	.g_chip_ident	= NULL,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= NULL,
	.s_register	= NULL,
#endif
	.queryctrl 		= v4l2_subdev_queryctrl,
	.querymenu 		= v4l2_subdev_querymenu,
	.g_ctrl 		= v4l2_subdev_g_ctrl,
	.s_ctrl 		= v4l2_subdev_s_ctrl,
	.g_ext_ctrls 	= v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls 	= v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls 	= v4l2_subdev_s_ext_ctrls,
};

static int hynix_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_link *icl = soc_camera_i2c_to_link(client);

	cfg->flags = V4L2_MBUS_MASTER | V4L2_MBUS_HSYNC_ACTIVE_HIGH |
		V4L2_MBUS_HSYNC_ACTIVE_LOW | V4L2_MBUS_VSYNC_ACTIVE_HIGH |
		V4L2_MBUS_VSYNC_ACTIVE_LOW | V4L2_MBUS_PCLK_SAMPLE_RISING |
		V4L2_MBUS_PCLK_SAMPLE_FALLING | V4L2_MBUS_DATA_ACTIVE_HIGH;
	cfg->type = V4L2_MBUS_BT656;
	cfg->flags = soc_camera_apply_board_flags(icl, cfg);

	return 0;
}

static int hynix_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
                           enum v4l2_mbus_pixelcode *code)
{
	dev_info(sd->v4l2_dev->dev, "%s\n", __FUNCTION__);

	if ((unsigned int)index >= ARRAY_SIZE(hynix_fmts))
		return -EINVAL;

        *code = hynix_fmts[index];

	return 0;
}

static int hynix_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *p)
{
	struct hynix_priv *priv = to_hynix_sd(sd);

	dev_info(sd->v4l2_dev->dev, "%s\n", __FUNCTION__);

	p->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	p->parm.capture = priv->parm;

	return 0;
}

/* Frame rate calculation helpers. */

static u16 calc_exp1x0(u32 afc_hz, u32 opclk_hz)
{
	/* Anti-flicker off */
	if(!afc_hz)
		return 0;

	return (opclk_hz / (afc_hz * 8));
}

static u16 calc_hblank(enum sensor_type type, u16 exp100, u16 exp120)
{
	const u16 min[] = {
		/* HBLANK > 412, multiple of 4, > 412 */
		[SR300PC10]	= 416,
		/* HBLANK > 244, multiple of 4, > 256 */
		[HI253]		= 260,
		/* HBLANK > 280, multiple of 4, > 256 */
		[HI161]		= 284,
	};

	u16 tmp = min[type];
	u16 w = horizontal_width[type];

	while( ((exp100 * 8) % (w + tmp)) && ((exp120 * 8) % (w + tmp)) )
	{
		tmp += 4;
		/* rollover - no match found */
		if(tmp < min[type])
			return 0;
	}

	return tmp;
}

static u32 calc_expfix(u16 hblank, u16 lines, u16 width, u16 afc_hz, u32 opclk_hz, u16 vsync, struct v4l2_fract *tpf)
{
	unsigned long long exp;
	u32 limit, div;

	/* start at target frame rate, work up to limit */
	exp = (unsigned long long)opclk_hz * tpf->numerator;
	do_div(exp, tpf->denominator);

	exp -= ((width + hblank) * vsync);

	/* Units are 8*Topclk */
	do_div(exp, 8);

	limit = ((width + hblank) * lines) + (opclk_hz/afc_hz) + 24;

	limit /= 8;

	/* TODO: Some other restriction? */
	exp = max(exp, (unsigned long long)limit);

	/* actual rate */
	/* TODO: appears about 1 fps off */
	tpf->numerator = 8*exp + ((width + hblank) * vsync);
	tpf->denominator =  opclk_hz;

	/* reduce the fraction */
	div = gcd(tpf->numerator, tpf->denominator);
	tpf->numerator /= div;
	tpf->denominator /= div;

	return (u32)exp;
}

static u32 calc_expmax(u16 hblank, u16 lines, u16 width, u16 afc1_hz, u16 afc2_hz, u32 opclk_hz)
{
	u32 exp, div;
	u32 ratio = opclk_hz / afc1_hz;

	/* start at max, truncate to highest multiple */
	exp = (width + hblank) * lines;
	div = exp / ratio;
	exp = div * ratio;

	return exp;
}

/* Math helpers for frame rate calculations */

/*
 * Similar to lib/gcd.c, but 64-bit capable.
 *
 * Requires a and b to be divisble into 2 unsigned longs.
 */
static unsigned long gcd64(unsigned long long a, unsigned long long b)
{
	unsigned long long tmpa, tmpb;
	unsigned long div, rema, remb;

	/* ULONG_MAX squared = 0xfffffffe00000001 */
	ASSERT((a < 0xfffffffe00000001) && (b < 0xfffffffe00000001));

	div = ULONG_MAX;
	tmpa = a;
	tmpb = b;
	rema = do_div(tmpa, div);
	remb = do_div(tmpb, div);

	/* don't call gcd() with 0 */
	if(min(rema, remb) == 0)
		return max(rema, remb);

	return gcd(rema, remb);
}

/*
 * Absolute value of difference between two fractions, scaled by 1000000 for
 * accuracy.
 *
 * Given A/B, C/D, returns abs(A/B - C/D) * 1000000
 *
 * Assumes both A/B and C/D are < 1.
 */
static unsigned long fract_diff(const struct v4l2_fract *f1, const struct v4l2_fract *f2)
{
	/*
	 * The fractions' terms are unsigned int (__u32), so use
	 * unsigned long long for calculations to preserved accuracy
	 * while avoiding overflow and algebraic contortions.
	 */
	unsigned long long n1, n2, d, tmp;
	unsigned long div;

	ASSERT(!(f1->numerator / f1->denominator) && !(f2->numerator / f2->denominator));

	/* Explicit cast required */
	n1 = (unsigned long long)f1->numerator * f2->denominator;
	n2 = (unsigned long long)f2->numerator * f1->denominator;
	d = (unsigned long long)f1->denominator * f2->denominator;

	/* Use n1 for difference */
	if(n1 > n2)
		n1 = n1 - n2;
	else
		n1 = n2 - n1;

	/*
	 * Reduce the fraction.
	 *
	 * The fraction's terms (n1, d) are products of two unsigned longs,
	 * so they must be <= (ULONG_MAX)*(ULONG_MAX), and therefore safely
	 * divisible into unsigned longs.
	 */
	div = gcd64(n1, d);
	do_div(n1, div);
	do_div(d, div);

	/* n1 = n1 * 1000000 / d */
	/*
	 * To preserve accuracy, use maximum possible multiplier on n1, and
	 * decrease d accordingly.
	 */
	/* start at div = 1000000 (tmp = ULLONG_MAX / 1000000) */
	tmp = 0x10c6f7a0b5ed;
	div = 1000000;
	while((n1 > tmp) && (d > 1))
	{
		div >>= 1;	
		d >>= 1;
		tmp = ULLONG_MAX;
		do_div(tmp, div);
	}

	/* TODO: how to recover from this? */
	ASSERT(n1 < tmp);

	n1 = n1 * div;

	/* Reduce the scaled fraction */
	div = gcd64(n1, d);
	do_div(n1, div);
	do_div(d, div);

	while(d > ULONG_MAX)
	{
		n1 >>= 1;
		d >>= 1;
	}

	div = (d & ULONG_MAX);
	div = do_div(n1, div);
	n1 += div / 1000000;

	return n1;
}

static int search_clocks(struct hynix_priv *priv, struct v4l2_fract *target_tpf,
	int *index, int *preview)
{
	int ret = -EINVAL;
	/*
	 * 'i' iterates through the clock list, and 'pre' iterates through
	 * each preview mode for a given clock list entry.  We scan through
	 * the list, keeping track of the best candidate seen so far, which
	 * is returned in 'index' and 'preview'.  "Best" is determined by
	 * the smallest difference from target_tpf.
	 */
	int			i;		/* clock list iterator */
	int			pre;		/* preview iterator */
	int			limit;		/* preview limiter */
	struct v4l2_fract	new_tpf, best_tpf;
	unsigned long int	new_diff, best_diff;

	/* Intermediates required for calculation */
	u16 hblank;	/* HBLANK[0x40, 0x41:P0] */

	u32 expmax;	/* EXPMAX[0x88, 0x89, 0x8A:P20] */
	u16 exp100;	/* EXP100[0x8B, 0x8C:P20] */
	u16 exp120;	/* EXP120[0x8D, 0x8E:P20] */
	u32 expfix;	/* EXPFIX[0x91, 0x92, 0x93:P20] */

	/* set 0.0 fps, but don't divide by zero */
	best_tpf.numerator = 0;
	best_tpf.denominator = 1;
	best_diff = ULONG_MAX;

	/* Search valid clocks arrangements for the best match */
	for(i = 0; i < priv->num_clks; i++)
	{
		if(priv->clks[i].sub_sample != (priv->curr_frmsize->vdoctl >> 4))
			continue;

		exp100 = calc_exp1x0(50*2, priv->clks[i].opclk_hz);
		exp120 = calc_exp1x0(60*2, priv->clks[i].opclk_hz);

		hblank = calc_hblank(priv->type, exp100, exp120);

		if(!hblank)
			continue;

		/* Preview doesn't work in full-scale */
		limit = (priv->clks[i].sub_sample == 0) ? PREVIEW_NONE : PREVIEW_PRE2;
		for(pre = PREVIEW_NONE; pre <= limit; pre++)
		{
			new_tpf = *target_tpf;
			expfix = calc_expfix(hblank,
				vertical_lines[priv->type][pre],
				horizontal_width[priv->type], 60*2,
				priv->clks[i].opclk_hz, priv->info->vsync, &new_tpf);

			if(expfix & 0xFF000000)
				continue;

			expmax = calc_expmax(hblank,
				vertical_lines[priv->type][pre],
				horizontal_width[priv->type], 50*2, 60*2,
				priv->clks[i].opclk_hz);

			if(expmax & 0xFF000000)
				continue;

			/* fract_diff() is expensive, so cache results */
			new_diff = fract_diff(target_tpf, &new_tpf);

			/* This is a better candidate */
			if(new_diff < best_diff)
			{
				best_diff = new_diff;
				best_tpf = new_tpf;
				*preview = pre;
				*index = i;
				ret = 0;
			}
			/* Exact match */
			if(new_diff == 0)
			{
				i = priv->num_clks;
				break;
			}
		}
	}

	return ret;
}

/* Register setting helpers */
static int set_pll(struct hynix_priv *priv, int index)
{
	int ret = -EINVAL;
	u8 tmp;
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);

	/* Enable sleep */
	if(0 != (ret = hynix_read(client, WINDOW_PAGE, PWRCTL, &tmp)))
		return ret;

	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, PWRCTL, 0x01, 0x00)))
		goto out;

	/* Set PLL lock time and PLL mode */
	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, PLLCTL1,
		priv->clks[index].pll_scale, 0x07)))
		goto out;
	
	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, PLLCTL2, 0xE0, 0x00)))
		goto out;

	if(priv->clks[index].pll_scale > 1)
	{
		/* Enable PLL1 and PLL2 */
		if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, PLLCTL1, 0x60, 0x00)))
			goto out;

		/* Wait PLL lock time */
		msleep(10);
	}

	/* Set clock divider */
	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, SYNCCTL,
		priv->clks[index].clk_div, 0x03)))
		goto out;

out:
	/* Restore sleep status */
	if(0 != (ret = hynix_write(client, WINDOW_PAGE, PWRCTL, tmp)))
		goto out;

	return ret;
}

static int set_hblank(struct i2c_client *client, u16 hblank)
{
	char seq[][3] = {
		{ WINDOW_PAGE, HBLANKH, hblank >> 8 },
		{ WINDOW_PAGE, HBLANKL, hblank & 0xFF },
	};

	return apply_seq(client, seq, ARRAY_SIZE(seq));
}

static int set_expmax(struct i2c_client *client, u32 expmax)
{
	char seq[][3] = {
		{ AUTOEXP_PAGE, EXPMAXH, expmax >> 16 },
		{ AUTOEXP_PAGE, EXPMAXM, expmax >> 8 },
		{ AUTOEXP_PAGE, EXPMAXL, expmax  & 0xFF },
	};

	return apply_seq(client, seq, ARRAY_SIZE(seq));
}

static int set_exp100(struct i2c_client *client, u16 exp100)
{
	char seq[][3] = {
		{ AUTOEXP_PAGE, EXP100H, exp100 >> 8 },
		{ AUTOEXP_PAGE, EXP100L, exp100 & 0xFF },
	};

	return apply_seq(client, seq, ARRAY_SIZE(seq));
}

static int set_exp120(struct i2c_client *client, u16 exp120)
{
	char seq[][3] = {
		{ AUTOEXP_PAGE, EXP120H, exp120 >> 8 },
		{ AUTOEXP_PAGE, EXP120L, exp120 & 0xFF },
	};

	return apply_seq(client, seq, ARRAY_SIZE(seq));
}

static int set_expfix(struct i2c_client *client, u32 expfix)
{
	char seq[][3] = {
		{ AUTOEXP_PAGE, EXPFIXH, expfix >> 16 },
		{ AUTOEXP_PAGE, EXPFIXM, expfix >> 8 },
		{ AUTOEXP_PAGE, EXPFIXL, expfix  & 0xFF },
	};

	return apply_seq(client, seq, ARRAY_SIZE(seq));
}

static int apply_clocks(struct hynix_priv *priv, struct v4l2_fract *target_tpf,
	int index, int preview)
{
	int ret = -EINVAL;
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);

	u16 hblank;	/* HBLANK[0x40, 0x41:P0] */

	u32 expmax;	/* EXPMAX[0x88, 0x89, 0x8A:P20] */
	u16 exp100;	/* EXP100[0x8B, 0x8C:P20] */
	u16 exp120;	/* EXP120[0x8D, 0x8E:P20] */
	u32 expfix;	/* EXPFIX[0x91, 0x92, 0x93:P20] */

	u8 set;

	/* calculate settings */
	exp100 = calc_exp1x0(50*2, priv->clks[index].opclk_hz);
	exp120 = calc_exp1x0(60*2, priv->clks[index].opclk_hz);

	hblank = calc_hblank(priv->type, exp100, exp120);

	expfix = calc_expfix(hblank, vertical_lines[priv->type][preview],
		horizontal_width[priv->type], 60*2, priv->clks[index].opclk_hz,
		priv->info->vsync, target_tpf);
	expmax = calc_expmax(hblank, vertical_lines[priv->type][preview],
		horizontal_width[priv->type], 50*2, 60*2,
		priv->clks[index].opclk_hz);

	/* Update registers */
	ret = set_pll(priv, index);
	if(ret)
		goto out;

	set = (preview == PREVIEW_PRE2) ? 3 : preview;

	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, VDOCTL1, set, 0x03)))
		goto out;

	ret = set_hblank(client, hblank);
	if(ret)
		goto out;

#if 0
	ret = set_expmax(client, expmax);
	if(ret)
		goto out;
#endif

	ret = set_exp100(client, exp100);
	if(ret)
		goto out;

	ret = set_exp120(client, exp120);
	if(ret)
		goto out;

	ret = set_expfix(client, expfix);
	if(ret)
		goto out;

out:
	return ret;
}

static int hynix_s_parm_EXPERIMENTAL(struct v4l2_subdev *sd, struct v4l2_streamparm *p)
{
	struct hynix_priv		*priv = to_hynix_sd(sd);
	struct i2c_client		*client = v4l2_get_subdevdata(sd);
	struct v4l2_captureparm		*cp = &p->parm.capture;
	struct v4l2_fract		*target_tpf = &cp->timeperframe;

	int			ret = -EINVAL;
	int			index;		/* clock list index */
	int			preview;	/* preview iterator */
	bool			var_fr;		/* Variable frame rate? */
	unsigned long int	div;

	dev_info(sd->v4l2_dev->dev, "%s\n", __FUNCTION__);

	if(priv->type != HI253)
		return -ENOSYS;

	if((target_tpf->denominator == 0) || (target_tpf->numerator == 0))
		goto out;

	/* TODO: < 1.0 fps support */
	if(target_tpf->numerator > target_tpf->denominator)
		goto out;

	/* requested a variable frame rate */
	var_fr = (unlikely(PARM_EXTMODE_TO_FRAME_RATE(cp->extendedmode) == HYNIX_YAC_VARIABLE_FRAME_RATE)) ? true : false;

	/* reduce the fraction */
	div = gcd(target_tpf->numerator, target_tpf->denominator);
	target_tpf->numerator /= div;
	target_tpf->denominator /= div;

	/* find best candidate */
	ret = search_clocks(priv, target_tpf, &index, &preview);
	if(ret)
		goto out;

	/* apply best canddiate - this updates target_tpf */
	ret = apply_clocks(priv, target_tpf, index, preview);
	if(ret)
		goto out;

	/* save applied settings */
	priv->parm.timeperframe = *target_tpf;

	/* Fill in other 'parm' fields */
	p->parm.capture = priv->parm;

	p->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
out:
	return ret;
}

static int hynix_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *p)
{
	struct hynix_priv		*priv = to_hynix_sd(sd);
	struct i2c_client		*client = v4l2_get_subdevdata(sd);
	struct v4l2_captureparm		*cp = &p->parm.capture;
	const struct v4l2_fract		*tpf = &cp->timeperframe;

	const struct frame_rate		*fr;
	bool				var_fr;
	int				i, len, ret = -EINVAL;
	const char			(*seq)[3] = NULL;
	u8				set, clear, query;
	int 			ret2, retry = 3;

	dev_info(sd->v4l2_dev->dev, "%s\n", __FUNCTION__);

	if(priv->type != HI253)
		return -ENOSYS;

	/* TODO: support arbitrary frame rates by calculating Hblank, etc. */

	/* requested a variable frame rate */
	var_fr = (unlikely(PARM_EXTMODE_TO_FRAME_RATE(cp->extendedmode) == HYNIX_YAC_VARIABLE_FRAME_RATE)) ? true : false;

	dev_info(sd->v4l2_dev->dev, "%s: %dx%d, vdoctl=%02x, var=%d\n",
			__FUNCTION__, priv->curr_frmsize->width, priv->curr_frmsize->height, priv->curr_frmsize->vdoctl, var_fr);

	for(i = 0; i < priv->curr_frmsize->num_frame_rates; i++)
	{
		fr = &priv->curr_frmsize->frame_rates[i];
#if 0
		if(unlikely(var_fr != fr->variable))
			continue;

		/* frame rate found */
		/* TODO: support a "close enough" frame rate instead of just a strict match*/
		if((fr->tpf.numerator == tpf->numerator) && (fr->tpf.denominator == tpf->denominator))
#endif
		{
			seq = fr->seq;
			len = fr->len;
			var_fr = fr->variable;
			break;
		}
	}
	dev_info(sd->v4l2_dev->dev, "%s: %d, seq=%p, len=%d, var=%d\n",
			__FUNCTION__, i, seq, len, var_fr);

	if(!seq)
		goto out;

	hynix_write(client, WINDOW_PAGE, PWRCTL, 0xF0);
	hynix_write(client, WINDOW_PAGE, PLLCTL1, PLLX(0x70));
	
	do {
		ret = apply_seq(client, seq, len);
		if (ret)
			dev_err(sd->v4l2_dev->dev, "%s: apply_seq error = %d, retry %d\n", __FUNCTION__, ret, retry);

		ret2 = hynix_read(client, WINDOW_PAGE, PLLCTL1, &query);
		if (ret2 != 0 || query != PLLX(0x70)) {
			dev_err(sd->v4l2_dev->dev, "%s: pll query error = %d, %02x, retry %d\n", __FUNCTION__, ret2, query, retry);
			ret = -EIO;
		}
	} while (ret != 0 && --retry);
	if(ret)
		goto out;

	/* apply variable or fixed frame rate */
	if(var_fr)
		set = 0x00, clear = 0x04;
	else
		set = 0x04, clear = 0x00;

#if defined(CONFIG_PLAT_NXP3200_RIO)
	/* apply x/y flip, per platform mounting */
	if (strstr(priv->info->name, "front"))
		clear |= FLIP(clear);
	else
		set |= FLIP(set);
#endif

	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, VDOCTL2, set, clear)))
		goto out;

	/* save applied settings */
	p->parm.capture.timeperframe.numerator = fr->tpf.numerator;
	p->parm.capture.timeperframe.denominator = fr->tpf.denominator;
	priv->parm = p->parm.capture;

	p->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
out:
	return ret;
}

static int hynix_find_frmsize(const struct hynix_priv *priv,
				const __u32 width, const __u32 height )
{
	int i;

	/* Find the nearest size that's at least as large */
	for(i = 0; i < priv->num_frmsizes; i++)
	{
		if((width > priv->frmsizes[i].width) ||
		   (height > priv->frmsizes[i].height))
		{
			break;
		}
	}

	i = (i == 0) ? i : --i;

	return i;
}

static int hynix_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct hynix_priv *priv = to_hynix_sd(sd);
	int i;

	dev_info(sd->v4l2_dev->dev, "%s\n", __FUNCTION__);

	i = hynix_find_frmsize(priv, mf->width, mf->height);

	/* TODO: allow for downscaling */

	mf->width       = priv->frmsizes[i].width;
	mf->height      = priv->frmsizes[i].height;
	mf->field       = V4L2_FIELD_NONE;	/* progressive-scan only */

	return 0;
}

static int hynix_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct hynix_priv *priv = to_hynix_sd(sd);

	*mf = priv->mf;

	return 0;
}

static int hynix_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hynix_priv *priv = to_hynix_sd(sd);
	int i, ret = -EINVAL;
	u8 set, clear = 0xF3;

	dev_info(sd->v4l2_dev->dev, "%s\n", __FUNCTION__);

	hynix_try_fmt(sd, mf);

	/* Adjust mediabus */
	switch(mf->code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
		set = 0x03;
		break;
	case V4L2_MBUS_FMT_YVYU8_2X8:
		set = 0x02;
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8:
		set = 0x01;
		break;
	case V4L2_MBUS_FMT_VYUY8_2X8:
		set = 0x00;
		break;
	/* TODO: RGB 5:6:5 / RGB 4:4:4 */
	default:
		goto out;
	}

	if(0 != (ret = hynix_rmw(client, FORMAT_PAGE, ISPCTL1, set, clear)))
		goto out;

	/* Adjust framesize */
	i = hynix_find_frmsize(priv, mf->width, mf->height);

	/* modify only Sub-sampling field */
	set = priv->frmsizes[i].vdoctl & 0x30;
	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, VDOCTL1, set, 0x30)))
		goto out;

	priv->curr_frmsize = &hi253_sizes[i];

	/* TODO: apply a valid frame rate */

	/* TODO: downscaling */

	priv->mf = *mf;

out:
	return ret;
}

static int hynix_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	u8 set = 0x00, clear = 0x00;

	dev_info(&client->dev, "hynix: %s, enable=%d\n", __FUNCTION__, enable);

	if(enable)
		clear = 0x01;	/* Power sleep */
	else
		set = 0x01;

	if(0 != (ret = hynix_rmw(client, WINDOW_PAGE, PWRCTL, set, clear)))
		goto out;

out:
	return ret;
}

static struct v4l2_subdev_video_ops hynix_subdev_video_ops = {
	.s_stream	= hynix_s_stream,
	.g_mbus_fmt	= hynix_g_fmt,
	.s_mbus_fmt	= hynix_s_fmt,
	.try_mbus_fmt	= hynix_try_fmt,
	/* TODO: windowing */
	.cropcap	= NULL,
	.g_crop		= NULL,
	.s_crop		= NULL,
	.enum_mbus_fmt	= hynix_enum_fmt,
	.g_parm		= hynix_g_parm,
	.s_parm		= hynix_s_parm,
	.g_mbus_config	= hynix_g_mbus_config,
};

static struct v4l2_subdev_ops hynix_subdev_ops = {
	.core	= &hynix_subdev_core_ops,
	.video	= &hynix_subdev_video_ops,
};

/*
 * Check if the clock setting @limit is unique (not seen before limit).
 */
static bool uniq_clock(struct hynix_priv *priv, int limit)
{
	int i;
	struct clock_combo *clk = &priv->clks[limit];

	for(i = 0; i < limit; i++)
	{
		/*
		 * For uniqueness, we care only about sub_sample and opclk_hz.
		 * The other fields are purely internal.
		 */
		if( (priv->clks[i].sub_sample	== clk->sub_sample)	&&
		    (priv->clks[i].opclk_hz	== clk->opclk_hz) )
			return false;
	}

	return true;
}

/*
 * Resize priv->clks to remove duplicate entries
 *
 * Returns new length of priv->clks.  0 indicates error.
 */
static int dedup_clocks(struct hynix_priv *priv, int len)
{
	int i, j;
	bool *uniq;
	struct clock_combo *uniq_clks;

	uniq = kzalloc(len * sizeof(*uniq), GFP_KERNEL);
	if(!uniq)
	{
		kfree(priv->clks);
		return 0;
	}

	/* Mark unique combinations */
	for(i = 0, j = 0; i < len; i++)
	{
		/* invalid */
		if(priv->clks[i].opclk_hz == 0)
			continue;

		uniq[i] = uniq_clock(priv, i);
		if(uniq[i])
			j++;
	}

	/* Shrink array */
	uniq_clks = kzalloc(j * sizeof(*uniq_clks), GFP_KERNEL);
	if(!uniq_clks)
	{
		kfree(priv->clks);
		len = 0;
		goto out;
	}

	/* de-dup */
	for(i = 0, j = 0; i < len; i++)
	{
		if(uniq[i])
		{
			uniq_clks[j].sub_sample	= priv->clks[i].sub_sample;
			uniq_clks[j].clk_div	= priv->clks[i].clk_div;
			uniq_clks[j].pll_scale	= priv->clks[i].pll_scale;
			uniq_clks[j].opclk_hz	= priv->clks[i].opclk_hz;
			j++;
		}
	}

	kfree(priv->clks);
	priv->clks = uniq_clks;
	len = j;
out:
	kfree(uniq);
	return len;
}

/*
 * Generate valid and unique clock configurations based on MCLK.
 *
 * Returns number of valid clocks, and populates priv->clks accordingly.
 * Return of 0 indicates an error.
 */
static int alloc_clocks(struct hynix_priv *priv)
{
	int ret = 0;
	int num_clks;

	struct clock_combo *clk;

	u32 pllclk_hz, opclk_hz, pclk_hz, pclk_max_hz, mclk_hz;
	u8 pll_scale, clk_div, sub_sample;

	mclk_hz = priv->info->mclk_hz;

	/* out of range */
	if((mclk_hz < MCLK_MIN_HZ) || (mclk_hz > MCLK_MAX_HZ))
		goto out;

	pclk_max_hz = min(priv->info->host_max_pclk_hz, (unsigned long)PCLK_MAX_HZ);

	/*
	 * We must check the following combinations:
	 * -sub_sample:	0, 1, 2
	 * -clk_div:	0, 1, 2, 3
	 * -pll_scale:	1, 2, 3, 4, 5, 6, 7
	 *
	 * This yields 3 * 4 * 7 = 84 possible settings.  However, not all
	 * of these will be unique.  For example, clk_div = 1 and
	 * pll_scale = 2 will yield the same OPCLK and PCLK for a given
	 * sub_sample as clk_div = 2 and pll_scale = 4.
	 *
	 * We first compute all the combinations, then discard all of the
	 * invalid/duplicate entries.
	 */

	num_clks = 3 * 4 * 7;
	priv->clks = kzalloc(num_clks * sizeof(*(priv->clks)), GFP_KERNEL);
	if(!priv->clks)
		goto out;

	/* 0 means full scale, 1 means 1/2, 2 means 1/4 */
	for(sub_sample = 0; sub_sample < 3; sub_sample++)
	{
		/* 1 means PLL off, 2-7 mean bit[2:0] of PLLCTL1 */
		for(pll_scale = 1; pll_scale < 8; pll_scale++)
		{
			pllclk_hz = ((pll_scale + 1) * mclk_hz) / 2;

			/* bit[1:0] of SYNCCTL */
			for(clk_div = 0; clk_div < 4; clk_div++)
			{
				opclk_hz = (pllclk_hz >> 1) >> clk_div;

				/* too slow - skip */
				if(opclk_hz < OPCLK_MIN_HZ)
					continue;

				pclk_hz = ((pllclk_hz >> clk_div) >> sub_sample);

				/* too fast - skip */
				if(pclk_hz > pclk_max_hz)
					continue;

				/* valid combination */
				clk		= &priv->clks[ret++];
				clk->sub_sample	= sub_sample;
				clk->clk_div	= clk_div;
				clk->pll_scale	= pll_scale;
				clk->opclk_hz	= opclk_hz;
			}
		}
	}

	ret = dedup_clocks(priv, num_clks);
out:
	return ret;
}

/*
 * i2c_driver function
 */

static int hynix_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);
	struct hynix_yac_camera_info	*info;
	struct hynix_priv		*priv;
	struct soc_camera_link		*icl = soc_camera_i2c_to_link(client);
	int				ret;

	//dev_dbg(&client->dev, "%s\n", __FUNCTION__);
	dev_info(&client->dev, "hynix: %s\n", __FUNCTION__);

	if(!icl || !icl->priv) {
		dev_err(&client->dev, "hynix_yac: missing platform data!\n");
		return -EINVAL;
	}

	if(!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	{
		dev_err(&adapter->dev,
			"I2C-Adapter doesn't support "
			"I2C_FUNC_SMBUS_BYTE_DATA\n");
		return -EIO;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if(!priv)
		return -ENOMEM;

	priv->info = icl->priv;

	priv->num_clks = alloc_clocks(priv);
	if(!priv->num_clks)
	{
		dev_err(&client->dev, "No valid clocks found!\n");
		kfree(priv);
		return -EINVAL;
	}

	v4l2_i2c_subdev_init(&priv->subdev, client, &hynix_subdev_ops);
	v4l2_ctrl_handler_init(&priv->hdl, 0);
	if (priv->hdl.error)
	{
		ret = priv->hdl.error;
		dev_err(&client->dev, "hynix_yac: control handler error = %d\n", ret);
		v4l2_ctrl_handler_free(&priv->hdl);
		kfree(priv->clks);
		kfree(priv);
		return ret;
	}
	priv->subdev.ctrl_handler = &priv->hdl;

	dev_info(&client->dev, "hynix: calling video probe\n");
	ret = hynix_video_probe(client);
	if(ret)
	{
		kfree(priv->clks);
		kfree(priv);
	}
	else
	{
#ifdef CONFIG_SOC_CAMERA_HYNIX_DEBUGFS
		hynix_init_debugfs(priv);
#endif

#ifdef CONFIG_SOC_CAMERA_HYNIX_SYSFS
		rwlock_init(&priv->sysfs_lock);
		sysfs_create_group(&client->dev.kobj, &hynix_attr_group);
#endif
		priv->info->attached = true;
	}

	return ret;
}

static int hynix_remove(struct i2c_client *client)
{
	struct hynix_priv	*priv = to_hynix_i2c(client);
	struct soc_camera_link	*icl  = soc_camera_i2c_to_link(client);

	dev_info(&client->dev, "%s\n", __FUNCTION__);

#ifdef CONFIG_SOC_CAMERA_HYNIX_SYSFS
	sysfs_remove_group(&client->dev.kobj, &hynix_attr_group);
#endif

#ifdef CONFIG_SOC_CAMERA_HYNIX_DEBUGFS
	if(priv->debug)
		debugfs_remove_recursive(priv->debug);
#endif

	priv->info->attached = false;
	/* Workaround for soc_camera power sequencing */
	if(icl->power)
		icl->power(&client->dev, 0);

	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);
	kfree(priv->clks);
	kfree(priv);
	return 0;
}

static const struct i2c_device_id hynix_id[] = {
	{ "hynix_yac", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hynix_id);

static struct i2c_driver hynix_i2c_driver = {
	.driver = {
		/* printk/debugfs/sysfs name */
		.name = "hynix_yac",
	},
	.probe    = hynix_probe,
	.remove   = hynix_remove,
	.id_table = hynix_id,
};

/*
 * module function
 */

static int __init hynix_module_init(void)
{
	return i2c_add_driver(&hynix_i2c_driver);
}

static void __exit hynix_module_exit(void)
{
	i2c_del_driver(&hynix_i2c_driver);
}

module_init(hynix_module_init);
module_exit(hynix_module_exit);

MODULE_AUTHOR("Justin Eno <jeno@leapfrog.com>");
MODULE_DESCRIPTION("SoC Camera driver for Hynix YAC imagers");
MODULE_LICENSE("GPL v2");
