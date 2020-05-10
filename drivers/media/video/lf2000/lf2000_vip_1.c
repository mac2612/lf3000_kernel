/*
 * V4L2 soc_camera_host driver for LF2000 Video Input Port
 *
 * Copyright (C) 2011, LeapFrog Enterprises, Inc.
 *
 * Based on V4L2 Driver for OMAP1 camera host and Nexell NXP3200 VIP driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/version.h>
#ifdef CONFIG_VIDEO_LF2000_DEBUGFS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif
#ifdef CONFIG_VIDEO_LF2000_SYSFS
#include <linux/sysfs.h>
#endif
#include <linux/rwsem.h>
#include <linux/gcd.h>
#include <linux/module.h>

#include <linux/lf2000/lf2000vip.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/videobuf-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include <asm/io.h>
#include <asm/system_info.h>

#include <mach/platform.h>
//#include <mach/platform_id.h> // FIXME
#include <mach/devices.h>
#include <mach/soc.h>
#include <mach/lf2000_vip.h>
/* FIXME */
//#include <../../../arch/arm/mach-nxp3200/board_revisions.h>

#include "lf2000_vip.h"

/* For optional double-buffering of overlay */
#include <linux/fb.h>

#define DEFAULT_LF2000_SKIP_FRAMES	3
/* TODO: get rid of this */
#define MAX_VIDEO_MEM	16

struct lf2000_buffer {
	struct videobuf_buffer		vb;
	dma_addr_t			addr;
	enum v4l2_mbus_pixelcode	code;
};

/* NOTE: These are used as array indicies.  Do not change. */
enum lf2000_vip_pipe {
	LF2000_VIP_PIPE_IDLE = -1,	/* pipe (clipper, decimator) inactive */
	LF2000_VIP_PIPE_CAPTURE = 0,	/* pipe active with capture */
	LF2000_VIP_PIPE_OVERLAY = 1,	/* pipe active with overlay */
};

struct lf2000_vip_dev {
	struct platform_device		*pdev;
	struct lf2000_vip_platform_data	*pdata;

	struct soc_camera_host		soc_host;
	struct soc_camera_device	*icd;
	struct v4l2_ctrl_handler 	ctrl_handler;

	unsigned int			irq;
	void __iomem			*base;
	struct resource			*res;

	struct lf2000_buffer		*active;

	struct list_head		cap_list;

	enum v4l2_memory		buf_type;

	char				name[10];

	spinlock_t			lock;

	bool				interlaced;

	/*
	 * There are 2 output pipes, the clipper and decimator.
	 * output_pipe[0] is the clipper and output_pipe[1] is the decimator,
	 * corresponding to VIP_OUTPUT_PIPE_*
	 */
	enum lf2000_vip_pipe		output_pipe[2];

	/*
	 * There are 2 formats, one for capture and one for overlay.
	 * pipe_format[0] is capture and pipe_format[1] is overlay,
	 * corresponding to LF2000_VIP_PIPE_*
	 */
	struct v4l2_pix_format		pipe_format[2];
	int				target_height[2];

	struct v4l2_pix_format		sensor_format;
	struct v4l2_rect		clip_rect;

	/* Function activity */
	bool				capture;
	bool				overlay;

	/* for optionally double-buffering overlay */
	bool				doublebuf;
	bool				unblank;
	dma_addr_t			fbbase;
	struct fb_info			*fb;

	/* for skipping initial bad frames */
	unsigned int			skip_frames;
	unsigned int			bad_frame;
	u16				cd_enb;
	struct rw_semaphore		skip_mutex;

#ifdef CONFIG_VIDEO_LF2000_DEBUGFS
	struct dentry			*debug;
	struct dentry			*debug_registers;
	struct dentry			*debug_clip_registers;
	struct dentry			*debug_deci_registers;
#endif

#ifdef CONFIG_VIDEO_LF2000_SYSFS
	rwlock_t			sysfs_lock;
	unsigned int			frame_count;
	unsigned int			flip_mask;
#endif

	/* separate flip/rotate flags for clipper vs decimator */
	unsigned int			flip_flags[2];
};

struct planar_coords {
	u16			seg[3];
	struct v4l2_rect	rect[3];
};

/* Local-scope prototypes */
static int setup_pipe(struct lf2000_vip_dev *pcdev, const dma_addr_t base, 
	const struct v4l2_pix_format *pix, enum lf2000_vip_pipe pipe_func);
static int enable_pipe(struct lf2000_vip_dev *pcdev,
		const enum lf2000_vip_pipe pipe, const bool enable);
static void update_sync(struct lf2000_vip_dev *pcdev, bool itu656);
static void set_flip_rotate(struct lf2000_vip_dev *pcdev, unsigned int flags,
		enum lf2000_vip_pipe pipe);
static int lf2000_camera_add_controls(struct soc_camera_device *icd);

#ifdef CONFIG_VIDEO_LF2000_DEBUGFS

/* instead of "gcc --combine" */
#include "lf2000_vip_debug.c"

#endif

#ifdef CONFIG_VIDEO_LF2000_SYSFS

/* instead of "gcc --combine" */
#include "lf2000_vip_sysfs.c"

#endif

/*****************************************************************************
 * videobuf_queue_ops  See Documentation/video4linux/videobuf
 *****************************************************************************/

/*****************************************************************************
 * videobuf_queue_ops.buf_setup()
 *
 * Return frame count (maximum allowed to be allocated) and frame size.
 ****************************************************************************/
static int lf2000_videobuf_setup(struct videobuf_queue *vq, unsigned int *count,
				unsigned int *size)
{
	struct soc_camera_device	*icd = vq->priv_data;
	struct soc_camera_host	*ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev	*pcdev = ici->priv;
	struct v4l2_pix_format	*fmt = &pcdev->pipe_format[LF2000_VIP_PIPE_CAPTURE];
	int ret = 0;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	pcdev->active = NULL;
	*size = fmt->bytesperline * fmt->height;

	if(!*count)
		*count = fmt->bytesperline / 4 / fmt->width;

	dev_dbg(icd->parent, "count=%d, size=%d\n", *count, *size);

	return ret;
}

/*****************************************************************************
 * videobuf helper
 *
 ****************************************************************************/
static void free_buffer(struct videobuf_queue *vq, struct lf2000_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host	*ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev	*pcdev = ici->priv;
	struct videobuf_buffer *vb = &buf->vb;

	BUG_ON(in_interrupt());

	/*
	 * This waits until this buffer is out of danger, i.e., until it is no
	 * longer in STATE_QUEUED or STATE_ACTIVE
	 */
	videobuf_waiton(vq, vb, 0, 0);
	if(pcdev->buf_type == V4L2_MEMORY_MMAP)
		videobuf_dma_contig_free(vq, vb);

	vb->state = VIDEOBUF_NEEDS_INIT;
}

/*****************************************************************************
 * videobuf_queue_ops.buf_prepare()
 *
 * Called once per requested buffer.  Allocates buffers and sets parameters
 * (size, width, height, field).
 ****************************************************************************/
static int lf2000_videobuf_prepare(struct videobuf_queue *vq,
			struct videobuf_buffer *vb, enum v4l2_field field)
{
	struct soc_camera_device	*icd = vq->priv_data;
	struct soc_camera_host	*ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev	*pcdev = ici->priv;
	struct v4l2_pix_format	*fmt = &pcdev->pipe_format[LF2000_VIP_PIPE_CAPTURE];

	struct lf2000_buffer *buf = container_of(vb, struct lf2000_buffer, vb);
	int ret = 0;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	BUG_ON(NULL == icd->current_fmt);

	if(	buf->code	!= icd->current_fmt->code ||
		vb->width	!= icd->user_width ||
		vb->height	!= icd->user_height ||
		vb->field	!= field)
	{
		buf->code	= icd->current_fmt->code;
		vb->width	= icd->user_width;
		vb->height	= icd->user_height;
		vb->field	= field;
		vb->state	= VIDEOBUF_NEEDS_INIT;
	}

	vb->size = fmt->bytesperline * fmt->height;
	if(0 != vb->baddr && vb->bsize < vb->size)
	{
		ret = -EINVAL;
		goto out;
	}

	if(vb->state == VIDEOBUF_NEEDS_INIT)
	{
		/* This handles the V4L2_MEMORY_USERPTR case */
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;

		buf->addr = videobuf_to_dma_contig(vb);
		vb->state = VIDEOBUF_PREPARED;		
	}

	return 0;

fail:
	free_buffer(vq, buf);
out:
	return ret;
}

/*****************************************************************************
 * videobuf_queue_ops.buf_queue()
 *
 * Queue a buffer for I/O.  spin_lock_irq()-protected by caller.
 ****************************************************************************/
static void lf2000_videobuf_queue(struct videobuf_queue *vq,
					struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct lf2000_buffer *buf = container_of(vb, struct lf2000_buffer, vb);
	struct v4l2_pix_format	*pix = &pcdev->pipe_format[LF2000_VIP_PIPE_CAPTURE];

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);


	list_add_tail(&vb->queue, &pcdev->cap_list);

	vb->state = VIDEOBUF_QUEUED;

	/* First buffer queued. */
	if(!pcdev->active)
	{
		/* Load proper registers with buffer address */
		setup_pipe(pcdev, buf->addr, pix, LF2000_VIP_PIPE_CAPTURE);

		/* Start streaming */
		enable_pipe(pcdev, LF2000_VIP_PIPE_CAPTURE, true);

		vb->state = VIDEOBUF_ACTIVE;

		pcdev->active = buf;

	}

	return;
}

/*****************************************************************************
 * videobuf_queue_ops.buf_release()
 *
 * Free a buffer.
 *
 * TODO: remove debugging prints
 ****************************************************************************/
static void lf2000_videobuf_release(struct videobuf_queue *vq,
					 struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct lf2000_buffer *buf = container_of(vb, struct lf2000_buffer, vb);
	struct device *dev = icd->parent;
	unsigned long flags;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

#if 1
	/* TODO: remove this */
	dev_dbg(dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);

	switch (vb->state) {
	case VIDEOBUF_ACTIVE:
		dev_dbg(dev, "%s (active)\n", __func__);
		break;
	case VIDEOBUF_QUEUED:
		dev_dbg(dev, "%s (queued)\n", __func__);
		break;
	case VIDEOBUF_PREPARED:
		dev_dbg(dev, "%s (prepared)\n", __func__);
		break;
	default:
		dev_dbg(dev, "%s (unknown) %d\n", __func__, vb->state);
		break;
	}
#endif

	spin_lock_irqsave(&pcdev->lock, flags);
	if(vb->state == VIDEOBUF_QUEUED)
	{
		list_del(&vb->queue);
		vb->state = VIDEOBUF_ERROR;
	}
	spin_unlock_irqrestore(&pcdev->lock, flags);

	free_buffer(vq, buf);

	/* Last buffer released - stop streaming */
	spin_lock_irqsave(&pcdev->lock, flags);
	if(pcdev->active == NULL)
	{
		enable_pipe(pcdev, LF2000_VIP_PIPE_CAPTURE, false);
	}
	spin_unlock_irqrestore(&pcdev->lock, flags);
}

static struct videobuf_queue_ops lf2000_videobuf_ops = {
	.buf_setup	= lf2000_videobuf_setup,
	.buf_prepare	= lf2000_videobuf_prepare,
	.buf_queue	= lf2000_videobuf_queue,
	.buf_release	= lf2000_videobuf_release,
};

/*****************************************************************************
 * hardware helpers
 *****************************************************************************/

static void vip_set_yuyv(struct lf2000_vip_dev *pcdev, const int enable)
{
	u16 tmp = readl(pcdev->base + VIP_CLIP_YUYVENB);

	if(enable)
	{
		tmp |= 0x1 << YUYVENB;
	}
	else
	{
		tmp &= ~(0x1 << YUYVENB);
	}

	writel(tmp, pcdev->base + VIP_CLIP_YUYVENB);
}

static void vip_set_linear(struct lf2000_vip_dev *pcdev, const dma_addr_t base,
				const u32 stride)
{
	u16 tmp;

	tmp = base >> 16;
	writel(tmp, pcdev->base + VIP_CLIP_BASEADDRH);

	tmp = base & 0xFFFF;
	writel(tmp, pcdev->base + VIP_CLIP_BASEADDRL);

	tmp = stride >> 16;
	writel(tmp, pcdev->base + VIP_CLIP_STRIDEH);

	tmp = stride & 0xFFFF;
	writel(tmp, pcdev->base + VIP_CLIP_STRIDEL);
}

static void vip_set_planar(void __iomem *base, 
	const unsigned char fmt, const struct planar_coords *coords)
{
	/*
	 * Clipper and Decimator have identical planar registers -
	 * use decimator offsets to calculate both.
	 */
	u16 tmp = readl(base + VIP_DECI_FORMAT - VIP_DECI_LUSEG);

	tmp &= ~(0x3 << CLIP_FORMATSEL);
	tmp |= fmt;

	writel(tmp, base + VIP_DECI_FORMAT - VIP_DECI_LUSEG);

	writel(coords->seg[0], base + VIP_DECI_LUSEG - VIP_DECI_LUSEG);
	writel(coords->rect[0].left, base + VIP_DECI_LULEFT - VIP_DECI_LUSEG);
	writel(coords->rect[0].top, base + VIP_DECI_LUTOP - VIP_DECI_LUSEG);
	writel(coords->rect[0].width, base + VIP_DECI_LURIGHT - VIP_DECI_LUSEG);
	writel(coords->rect[0].height, base + VIP_DECI_LUBOTTOM - VIP_DECI_LUSEG);

	writel(coords->seg[1], base + VIP_DECI_CBSEG - VIP_DECI_LUSEG);
	writel(coords->rect[1].left, base + VIP_DECI_CBLEFT - VIP_DECI_LUSEG);
	writel(coords->rect[1].top, base + VIP_DECI_CBTOP - VIP_DECI_LUSEG);
	writel(coords->rect[1].width, base + VIP_DECI_CBRIGHT - VIP_DECI_LUSEG);
	writel(coords->rect[1].height, base + VIP_DECI_CBBOTTOM - VIP_DECI_LUSEG);

	writel(coords->seg[2], base + VIP_DECI_CRSEG - VIP_DECI_LUSEG);
	writel(coords->rect[2].left, base + VIP_DECI_CRLEFT - VIP_DECI_LUSEG);
	writel(coords->rect[2].top, base + VIP_DECI_CRTOP - VIP_DECI_LUSEG);
	writel(coords->rect[2].width, base + VIP_DECI_CRRIGHT - VIP_DECI_LUSEG);
	writel(coords->rect[2].height, base + VIP_DECI_CRBOTTOM - VIP_DECI_LUSEG);
}

static void set_clipper(struct lf2000_vip_dev *pcdev,
				const struct v4l2_rect *rect)
{
	u16 tmp;

	tmp = rect->left;
	writel(tmp, pcdev->base + VIP_CLIP_LEFT);
	tmp += rect->width;
	writel(tmp, pcdev->base + VIP_CLIP_RIGHT);

	tmp = rect->top;
	writel(tmp, pcdev->base + VIP_CLIP_TOP);
	tmp += rect->height;
	writel(tmp, pcdev->base + VIP_CLIP_BOTTOM);
}

static void set_decimator(struct lf2000_vip_dev *pcdev, u16 width, u16 height)
{
	u16 tmp, src_w, src_h;

	src_w = pcdev->clip_rect.width; // - pcdev->clip_rect.left;
	src_h = pcdev->clip_rect.height; // - pcdev->clip_rect.top;

	writel(width, pcdev->base + VIP_DECI_TARGETW);
	writel(height, pcdev->base + VIP_DECI_TARGETH);

	tmp = src_w - width;
	writel(tmp, pcdev->base + VIP_DECI_DELTAW);
	tmp = src_h - height;
	writel(tmp, pcdev->base + VIP_DECI_DELTAH);

	/* 2's complement registers */
	tmp = (s16)((width << 1) - src_w);
	writel(tmp, pcdev->base + VIP_DECI_CLEARW);
	tmp = (s16)((height << 1) - src_h);
	writel(tmp, pcdev->base + VIP_DECI_CLEARH);
}

static void update_sync(struct lf2000_vip_dev *pcdev, bool itu656)
{
	u16 tmp;

	if(itu656)
	{
		tmp = pcdev->pdata->vfp + 1;
		writel(tmp, pcdev->base + VIP_VBEGIN);
		tmp += pcdev->pdata->vsw;
		writel(tmp, pcdev->base + VIP_VEND);

		tmp = pcdev->pdata->hfp - 7;
		writel(tmp, pcdev->base + VIP_HBEGIN);
		tmp += pcdev->pdata->hsw;
		writel(tmp, pcdev->base + VIP_HEND);
	}
	else
	{
		tmp = pcdev->pdata->vbp - 1;
		writel(tmp, pcdev->base + VIP_VBEGIN);
		tmp += pcdev->sensor_format.height;
		writel(tmp, pcdev->base + VIP_VEND);

		tmp = pcdev->pdata->hbp - 1;
		writel(tmp, pcdev->base + VIP_HBEGIN);
		tmp += pcdev->sensor_format.width;
		writel(tmp, pcdev->base + VIP_HEND);
	}
}

static void vip_calculate_planar_addr(struct planar_coords *coords,
		const dma_addr_t ptr, const struct v4l2_pix_format *fmt, int height)
{
	uintptr_t offset, base;
	u16 tmp;
	int hdiv, wdiv, pitch = fmt->bytesperline;

	switch(fmt->pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
		hdiv = 1;
		wdiv = 2;
		break;
	case V4L2_PIX_FMT_YUV420:
		hdiv = wdiv = 2;
		break;
	}

	/* TODO: simplifiy calculations into shifts & masks */
	base = (uintptr_t)ptr;

	/* Set Y boundaries */
	tmp = base >> 24;
	coords->seg[0] = tmp;

	/* start at upper left corner */
	offset = base & 0xFFFFFF;
	tmp = offset % pitch;
	coords->rect[0].left = tmp;
	tmp = offset / pitch;
	coords->rect[0].top = tmp;

	tmp = (offset % pitch) + fmt->width;
	coords->rect[0].width = tmp;	/* actually right boundary */

	tmp = (offset / pitch) + fmt->height;
	coords->rect[0].height = tmp;	/* actually bottom boundary */

	/* Set U boundaries */
	/* Start at upper left corner */
	base = (uintptr_t)ptr + pitch/2;
	tmp = base >> 24;
	coords->seg[1] = tmp;

	offset = base & 0xFFFFFF;

	tmp = offset % pitch;
	coords->rect[1].left = tmp;
	tmp = offset / pitch;
	coords->rect[1].top = tmp;

	tmp = (offset % pitch) + (fmt->width/wdiv);
	coords->rect[1].width = tmp;

	tmp = (offset / pitch) + (fmt->height/hdiv);
	coords->rect[1].height = tmp;

	/* Set V boundaires */
	/* Start at upper left corner */
	base = (uintptr_t)ptr + pitch/2 + pitch * height/2;
	tmp = base >> 24;
	coords->seg[2] = tmp;

	offset = base & 0xFFFFFF;

	tmp = offset % pitch;
	coords->rect[2].left = tmp;
	tmp = offset / pitch;
	coords->rect[2].top = tmp;

	tmp = (offset % pitch) + (fmt->width/wdiv);
	coords->rect[2].width = tmp;

	tmp = (offset / pitch) + (fmt->height/hdiv);
	coords->rect[2].height = tmp;
}

static int setup_pipe(struct lf2000_vip_dev *pcdev, const dma_addr_t base, 
	const struct v4l2_pix_format *pix, enum lf2000_vip_pipe pipe_func)
{
	struct planar_coords coords;
	unsigned char fmt;
	int ret = 0, pipe_idx, height;
	u16 tmp;

	pipe_idx = FMT_PRIV_TO_PIPE(pix->priv);

	height = pcdev->target_height[pipe_func];

	switch(pipe_idx) {
	case VIP_OUTPUT_PIPE_CLIPPER:
		if(pix->pixelformat == V4L2_PIX_FMT_YUYV)
		{
			vip_set_yuyv(pcdev, 1);
			vip_set_linear(pcdev, base, 2*pix->width);
			break;
		}

		vip_set_yuyv(pcdev, 0);

		fmt = (pix->pixelformat == V4L2_PIX_FMT_YUV422P ) ?
			VIP_YUV422 : VIP_YUV420;

		vip_calculate_planar_addr(&coords, base, pix, height);
		vip_set_planar(pcdev->base + VIP_CLIP_LUSEG, fmt, &coords);
		break;

	case VIP_OUTPUT_PIPE_DECIMATOR:
		if(pix->pixelformat == V4L2_PIX_FMT_YUYV)
		{
			ret = -EINVAL;
			break;
		}

		fmt = (pix->pixelformat == V4L2_PIX_FMT_YUV422P ) ?
			VIP_YUV422 : VIP_YUV420;

		vip_calculate_planar_addr(&coords, base, pix, height);
		vip_set_planar(pcdev->base + VIP_DECI_LUSEG, fmt, &coords);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	if(!ret)
	{
		/* Keep track of current format for capture/overlay */
		pcdev->pipe_format[pipe_func] = *pix;
	}

	return ret;
}

#define VIPX 	1	// VIP0 vs VIP1?

static int enable_pipe(struct lf2000_vip_dev *pcdev,
		const enum lf2000_vip_pipe pipe, const bool enable)
{
	struct v4l2_pix_format	*fmt = &pcdev->pipe_format[pipe];
	int pipe_idx = FMT_PRIV_TO_PIPE(fmt->priv);
	u16 config, enb, bit, intc, odint;
	int ret = 0;

	dev_dbg(&pcdev->pdev->dev, "%s: enable=%d, pipe=%d\n", __FUNCTION__, enable, pipe_idx);

	switch(pipe_idx) {
	case VIP_OUTPUT_PIPE_CLIPPER:
		bit = (0x1 << CLIPENB);
		break;
	case VIP_OUTPUT_PIPE_DECIMATOR:
		bit = (0x1 << DECIENB);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if(ret)
		goto out;

	enb	= readl(pcdev->base + VIP_CDENB);
	config	= readl(pcdev->base + VIP_CONFIG);
	intc	= readl(pcdev->base + VIP_INTCTRL) & (0x1 << VSINTENB);

	if(enable)
	{
		enb	|= (0x1 << SEPENB) | bit;
		config	|= (0x1 << VIPENB);

		if((pcdev->capture == false) && (pcdev->overlay == false))
		{
			// Streaming starts here.  Reset bad frame counter.
			// FIXME: enable lock when up_read() below is balanced
			//down_read(&pcdev->skip_mutex);
			pcdev->bad_frame = 1;

			intc	|= (0x1 << VSINTENB);
			intc	|= (0x1 << HSINTENB);

			/* Delay activation until bad frames are dropped. */
			pcdev->cd_enb = enb;
		}
		else
		{
			/* Already running */

			pcdev->cd_enb |= enb;

			if(pcdev->bad_frame > pcdev->skip_frames)
			{
				pcdev->bad_frame -= 2;
				intc	|= (0x1 << VSINTENB);
				intc	|= (0x1 << HSINTENB);
			}
		}

		switch(pipe) {
		case LF2000_VIP_PIPE_CAPTURE:
			pcdev->capture = true;
			break;
		case LF2000_VIP_PIPE_OVERLAY:
			pcdev->overlay = true;
			break;
		}
#if 0
		/*
		 * TODO: Optimization: Do this here intead of in setup_pipe()
		 * because it's redundant at ISR time.
		 */
		if(fmt->pixelformat == V4L2_PIX_FMT_YUYV)
		{
			vip_set_yuyv(pcdev, enable);
		}
		else if(pipe_idx == VIP_OUTPUT_PIPE_CLIPPER)
		{
			vip_set_yuyv(pcdev, !enable);
		}
#endif
		pcdev->output_pipe[pipe_idx] = pipe;
	}
	else
	{
		enb &= ~bit;

		switch(pipe) {
		case LF2000_VIP_PIPE_CAPTURE:
			pcdev->capture = false;
			if(pcdev->active)
			{
			/* TODO: dirty shutdown - closing while stream is active */
			}
			break;
		case LF2000_VIP_PIPE_OVERLAY:
			pcdev->overlay = false;
			pcdev->doublebuf = false;
			break;
		}

		if((pcdev->capture == false) && (pcdev->overlay == false))
		{
			if(enb & (0x1 << SEPENB))
			{
				// FIXME: disable lock when down_read() above is balanced
				//up_read(&pcdev->skip_mutex);
			}

			odint	= readl(pcdev->base + VIP_ODINT);

			enb	&= ~(0x1 << SEPENB);
			config	&= ~(0x1 << VIPENB);
			intc	&= ~(0x1 << VSINTENB);
			intc	|= (0x1 << VSINTPEND);
			odint	&= ~(0x1 << ODINTENB);
			odint	|= (0x1 << ODINTPEND);

//			writel(odint, pcdev->base + VIP_ODINT);
		}

		pcdev->cd_enb = enb;
//		writel(pcdev->cd_enb, pcdev->base + VIP_CDENB);

		pcdev->output_pipe[pipe_idx] = LF2000_VIP_PIPE_IDLE;
	}

//	writel(config, pcdev->base + VIP_CONFIG);
//	writel(intc, pcdev->base + VIP_INTCTRL);
	printk("%s: VIP_CONFIG: %08x, VIP_INTCTRL: %08x\n", __func__, config, intc);

	if (enable) {
	    NX_VIP_SetInterruptEnableAll(VIPX, CFALSE);
	    NX_VIP_ClearInterruptPendingAll(VIPX);
		NX_VIP_SetInterruptEnable(VIPX, 2, enable);
		NX_VIP_SetVIPEnable(VIPX, enable, enable, pcdev->capture, pcdev->overlay);
	}
	else {
		NX_VIP_SetVIPEnable(VIPX, enable, enable, CFALSE, CFALSE);
	    NX_VIP_SetInterruptEnableAll(VIPX, CFALSE);
	    NX_VIP_ClearInterruptPendingAll(VIPX);
	}

	config = readl(pcdev->base + VIP_CONFIG);
	intc   = readl(pcdev->base + VIP_INTCTRL);
	printk("%s: VIP_CONFIG: %08x, VIP_INTCTRL: %08x\n", __func__, config, intc);

out:
	return ret;
}

static void set_flip_rotate(struct lf2000_vip_dev *pcdev, unsigned int flags, enum lf2000_vip_pipe pipe)
{
	U16 tmp, rotate = 0;

#ifdef CONFIG_VIDEO_LF2000_SYSFS
	if (pcdev->flip_mask)
		flags = 0;
#endif

	if(flags & LF2000_VIP_ROTATE)
	{
		rotate |= (0x1 << CLIP_ROTATION);
	}
	if(flags & LF2000_VIP_HFLIP)
	{
		rotate |= (0x1 << CLIP_HFLIP);
	}
	if(flags & LF2000_VIP_VFLIP)
	{
		rotate |= (0x1 << CLIP_VFLIP);
	}

	if (pipe == LF2000_VIP_PIPE_CAPTURE)
	{
		tmp = readl(pcdev->base + VIP_CLIP_ROTFLIP);
		tmp &= ~(0x7);
		tmp |= rotate;
		writel(tmp, pcdev->base + VIP_CLIP_ROTFLIP);
	}
	else
	{
		tmp = readl(pcdev->base + VIP_DECI_ROTFLIP);
		tmp &= ~(0x7);
		tmp |= rotate;
		writel(tmp, pcdev->base + VIP_DECI_ROTFLIP);
	}
}

/*****************************************************************************
 * soc_camera_host_ops
 *****************************************************************************/

static void lf2000_camera_set_vip_port(struct lf2000_vip_dev *pcdev)
{
#if 0 // FIXME
	U16 tmp;
	switch(system_rev) {
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
		case LF2000_BOARD_VALENCIA_CIP:
			tmp = readl(pcdev->base + VIP_VIP1);
			tmp &= ~(1 << VIPPORTSEL);	
			writel(tmp, pcdev->base + VIP_VIP1);
			dev_info(&pcdev->pdev->dev,"port 0, rear camera \n"); 
			break;
		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_PP:
		case LF2000_BOARD_LUCY_CIP:
		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF2000_BOARD_VTK:
			break;
		default: 
			printk(KERN_ERR "%s Invalid Board Revision = 0x%X\n",
				__func__, system_rev);
			break;
	}
#endif
}

/*****************************************************************************
 * soc_camera_host_ops.add_device()
 *
 * Provided for soc_camera framework to associate a camera to a camera host.
 *
 * TODO: activate clocks?
 ****************************************************************************/
static int lf2000_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	int ret = 0;
	U16 tmp, config, rotate;

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

	if(pcdev->icd)
	{
		ret = -EBUSY;
		goto ebusy;
	}

	dev_info(&pcdev->pdev->dev,
		 "%s: LF2000 VIP driver attached to camera %d\n", __FUNCTION__, icd->devnum);

	pcdev->icd = icd;

	config = rotate = 0x0;

	/* Reserved bit - copied from Nexell driver */	
	config &= ~(0x1 << DRANGE);

	if(pcdev->pdata->flags & LF2000_VIP_ITU656)
	{
		config &= ~(0x1 << EXTSYNCENB);
	}
	else
	{
		/* TODO: this is untested */

		config |= (0x1 << EXTSYNCENB);
	}

	update_sync(pcdev, (pcdev->pdata->flags & LF2000_VIP_ITU656));

	/* mirror flip front camera viewfinder only */
	if(icd->devnum == 1)
	{
#if 0 // FIXME
		if (get_leapfrog_platform() == RIO)
			pcdev->flip_flags[LF2000_VIP_PIPE_OVERLAY] |= LF2000_VIP_HFLIP;
		else
#endif
			pcdev->flip_flags[LF2000_VIP_PIPE_OVERLAY] |= LF2000_VIP_HFLIP;
	}
	else
	{
		pcdev->flip_flags[LF2000_VIP_PIPE_OVERLAY] &= ~LF2000_VIP_HFLIP;
		pcdev->flip_flags[LF2000_VIP_PIPE_OVERLAY] &= ~LF2000_VIP_VFLIP;
	}

	set_flip_rotate(pcdev, pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE], LF2000_VIP_PIPE_CAPTURE);
	set_flip_rotate(pcdev, pcdev->flip_flags[LF2000_VIP_PIPE_OVERLAY], LF2000_VIP_PIPE_OVERLAY);

	/* The datasheet says: Reserved. Must write '3'. */
	/* Reserved bits - copied from Nexell driver */
	tmp = readl(pcdev->base + VIP_SYNCCTRL);
	tmp |= (0x3 << SYNCCTRL_RES);
	writel(tmp, pcdev->base + VIP_SYNCCTRL);

	/* Enable VIP */
	tmp = config | (0x1 << DWIDTH);

	writel(tmp, pcdev->base + VIP_CONFIG);

	pcdev->cd_enb = readl(pcdev->base + VIP_CDENB);
	pcdev->cd_enb &= ~((0x1 << SEPENB) | (0x1 << CLIPENB) | (0x1 << DECIENB));
	
	/* Select input port if front camera */
	if(icd->devnum == 0)
	{
		lf2000_camera_set_vip_port(pcdev);
	}
	else
	{
		dev_info(&pcdev->pdev->dev,"port 1, front camera \n"); 
	}

	/* append VIP host controls to H253 device control handler */
	dev_info(&pcdev->pdev->dev, "add_device: adding controls to %p\n",  &icd->ctrl_handler);
	ret = lf2000_camera_add_controls(icd);
		
ebusy:
	return ret;
}

/*****************************************************************************
 * soc_camera_host_ops.remove_device()
 *
 * Provided for soc_camera framework to dissociate a camera from a camera
 * host.
 *
 * TODO: deactivate clocks?
 ****************************************************************************/
static void lf2000_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	U16 tmp;

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

	BUG_ON(icd != pcdev->icd);

	enable_pipe(pcdev, pcdev->output_pipe[VIP_OUTPUT_PIPE_CLIPPER], false);
	enable_pipe(pcdev, pcdev->output_pipe[VIP_OUTPUT_PIPE_DECIMATOR], false);

	/* This VIDIOC_STREAMOFF is redundant for well-behaved applications */
	v4l2_subdev_call(sd, video, s_stream, 0);

	/* Disable interrupt sources */
	tmp = readl(pcdev->base + VIP_ODINT);
	tmp &= ~(0x1 << ODINTENB);
	writel(tmp, pcdev->base + VIP_ODINT);

	tmp = readl(pcdev->base + VIP_INTCTRL);
	tmp &= ~((0x1 << HSINTENB) | (0x1 << VSINTENB));
	writel(tmp, pcdev->base + VIP_INTCTRL);

	/* Disable separator */
	tmp = readl(pcdev->base + VIP_CDENB);
	tmp &= ~(0x1 << SEPENB);
	writel(tmp, pcdev->base + VIP_CDENB);
	
#if 0	// not LF3000
	/* Reset back to VIP0:1 Port */
	tmp = readl(pcdev->base + VIP_VIP1);
	tmp |= (1 << VIPPORTSEL);
	writel(tmp, pcdev->base + VIP_VIP1);
#endif

	/* Disable VIP */
	tmp = readl(pcdev->base + VIP_CONFIG);
	tmp &= ~(0x1 << VIPENB);
	writel(tmp, pcdev->base + VIP_CONFIG);

	dev_info(&pcdev->pdev->dev,
		 "%s: LF2000 VIP driver detached from camera %d\n", __FUNCTION__, icd->devnum);

	pcdev->icd = NULL;

	return;
}

/*****************************************************************************
 * soc_camera_host_ops.set_bus_param()
 *
 * Provided for soc_camera framework to set physical bus parameters, e.g., bus
 * width, clock polarity, etc.
 ****************************************************************************/
static int lf2000_camera_set_bus_param(struct soc_camera_device *icd,
					__u32 pixfmt)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_mbus_config cfg = {.type = V4L2_MBUS_PARALLEL,};
	unsigned long common_flags;
	int ret;
	U32 tmp;

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if (!ret) {
		common_flags = soc_mbus_config_compatible(&cfg,
					VIP_BUS_FLAGS);
		if (!common_flags) {
			dev_warn(icd->parent,
				"Flags incompatible: camera 0x%x, host 0x%x\n",
				cfg.flags, VIP_BUS_FLAGS);
			return -EINVAL;
		}
	} else if (ret != -ENOIOCTLCMD) {
		return ret;
	} else {
		common_flags = VIP_BUS_FLAGS;
	}

	dev_dbg(icd->parent, "Flags cam: 0x%x host: 0x%x common: 0x%lx\n",
		cfg.flags, VIP_BUS_FLAGS, common_flags);

	if(common_flags & V4L2_MBUS_PCLK_SAMPLE_RISING)
		common_flags &= ~V4L2_MBUS_PCLK_SAMPLE_FALLING;
	else
		common_flags &= ~V4L2_MBUS_PCLK_SAMPLE_RISING;

	/* Apply shared bus settings to sensor */
	cfg.flags = common_flags;
	ret = v4l2_subdev_call(sd, video, s_mbus_config, &cfg);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		dev_dbg(icd->parent, "camera s_mbus_config(0x%lx) returned %d\n", 
			common_flags, ret);
		return ret;
	}

	/* Apply to VIP */
	tmp = readl(pcdev->base + VIP_CLKGENL);
	tmp &= ~( 0x7 << CLKSRCSEL);

	if(common_flags & V4L2_MBUS_PCLK_SAMPLE_RISING)
	{
		tmp |= 0x3 << CLKSRCSEL;
	}
	else
	{
		tmp |= 0x4 << CLKSRCSEL;
	}
	writel(tmp, pcdev->base + VIP_CLKGENL);

	return 0;
}

/*
 * These are the supported *PIX_FMT* formats, as reported in .get_formats()
 * The supported *MBUS_FMT* formats are the whole *YUYV8* family.  We support
 * every combination, i.e., PIX_FMT and MBUS_FMT are independent.
 */
static const struct soc_mbus_pixelfmt lf2000_camera_fmt[] = {
	{
		.fourcc			= V4L2_PIX_FMT_YUV422P,
		.name			= "Planar YUV 422",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc			= V4L2_PIX_FMT_YUV420,
		.name			= "Planar YUV 420",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
	/* TODO: YCbCr 4:4:4 - no such FourCC? Custom FMT? */
	/* Packed format must come last */
	{
		.fourcc			= V4L2_PIX_FMT_YUYV,
		.name			= "Packed YUYV",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
};

static const struct soc_mbus_pixelfmt *lf2000_camera_format(const struct v4l2_pix_format *pix)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(lf2000_camera_fmt); i++)
	{
		if(lf2000_camera_fmt[i].fourcc == pix->pixelformat)
			return &lf2000_camera_fmt[i];
	}

	return NULL;
}

/*****************************************************************************
 * soc_camera_host_ops.set_fmt()
 *
 * Provided for userspace to negotiate the format of the data to be exchanged
 * with the driver.  May return -EBUSY.  The framework calls try_fmt first,
 * so the requested format is known to be already valid.
 *
 * According to the example given here:
 * http://v4l2spec.bytesex.org/spec/x1904.htm#AEN1954
 * , the proper procedure is to:
 * -First, scale the image as close as possible to the requested dimensions,
 *  maintaining aspect ratio when possible, and keeping the scaled dimensions
 *  >= the requested dimensions
 * -Next, crop the scaled image as close as possible to the requested
 *  dimensions
 *
 * TODO: 
 * This above order of operations is invalid for the LF2000, because the
 * clipper precedes the decimator in the image processing pipeline.  Also,
 * since the clipper and decimator can independently write to memory, we would
 * like to potentially use both for simultaenous operation of:
 *  1.) A hardware viewfinder (i.e., direct overlay to MLC video layer)
 *  2.) A software frame grabber (traditional "snapshot" camera)
 *
 * Note that only the clipper supports linear YUYV.
 *
 * An application can further request a cropped image via VIDIOC_S_CROP, which
 * is not supposed to adjust current scaling.
 *
 * It is probably preferable to perform cropping at the sensor level to reduce
 * bus bandwidth.  In contrast, scaling is an implementation-dependent
 * operation, and the LF2000 may deliver superior results.
 ****************************************************************************/
static int lf2000_camera_set_fmt(struct soc_camera_device *icd,
				struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	const struct soc_camera_format_xlate *xlate;
	int ret = 0, pipe_idx;
	u16 tmp;
	int rotated = 0;
	int rot_width = 0, rot_height = 0;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	/* Check if the requested output pipe is already active */
	pipe_idx = FMT_PRIV_TO_PIPE(pix->priv);
	if(pcdev->output_pipe[pipe_idx] != LF2000_VIP_PIPE_IDLE)
	{
		ret = -EBUSY;
		goto out;
	}

	/* Rotated image case needs special handling.
	 * VIP sensor geometry setup for horizontal aspect ratio.
	 * V4L buffer addressing setup for vertical aspect ratio.
	 */
	if (pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] & LF2000_VIP_ROTATE)
	{
		rotated = 1;
		rot_width  = pix->width;
		rot_height = pix->height;
		if (pix->width == pix->height) {
			dev_dbg(icd->parent, "%s: rotate square aspect ratio %dx%d\n", __func__, pix->width, pix->height);
			rot_width = pix->height = pix->width * 3 / 4;
		}
		else if (pix->width < pix->height) {
			dev_dbg(icd->parent, "%s: rotate vertical aspect ratio %dx%d\n", __func__, pix->width, pix->height);
			pix->width  = rot_height;
			pix->height = rot_width;
		}
		else {
			dev_dbg(icd->parent, "%s: rotate horizontal aspect ratio %dx%d\n", __func__, pix->width, pix->height);
			rot_height = pix->width;
			rot_width  = pix->height;
		}
	}

	/* Setup sensor */
	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;

	/*
	 * Returns the first MBUS_FMT with a matching PIX_FMT (fourcc).
	 * Valid combinations (overlaping MBUS_FMTs between LF2000 and
	 * sensor) are filtered by .get_formats()
	 */
	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	/*
	 * xlate must be non-NULL here.  The soc framework fails initialization
	 * when .get_formats() provides nothing.
	 */

	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if(ret < 0)
		goto out;

	/* Setup VIP */
	/* byte order */
	tmp = readl(pcdev->base + VIP_CONFIG);
	tmp &= ~(0x3 << DORDER);

	switch(mf.code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
		tmp |= 0x2 << DORDER;
		break;
	case V4L2_MBUS_FMT_YVYU8_2X8:
		tmp |= 0x3 << DORDER;
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8:
		tmp |= 0x0 << DORDER;
		break;
	case V4L2_MBUS_FMT_VYUY8_2X8:
		tmp |= 0x1 << DORDER;
		break;
	default:
		/* not reached */
		dev_warn(icd->parent, "No mbus format supported\n");
		ret = -EINVAL;
		goto out;
	}

	writel(tmp, pcdev->base + VIP_CONFIG);

	/* interlacing */
	tmp = readl(pcdev->base + VIP_SCANMODE);
	tmp &= ~(0x1 << INTERLACEENB);

	if(!V4L2_FIELD_HAS_BOTH(mf.field))
	{
		pcdev->interlaced = false;
	}
	else
	{
		/* TODO: add interlace support */
		pcdev->interlaced = true;
		tmp |= 0x1 << INTERLACEENB;
	}

	writel(tmp, pcdev->base + VIP_SCANMODE);

	icd->current_fmt	= xlate;

	/*
	 * Setup image dimensions.  mf.width and mf.height are what the sensor
	 * outputs.  pix->width and pix->height are what the user requested.
	 *
	 * pix has already been sanitized by try_fmt().
	 */
	tmp = mf.width;
	if(pcdev->pdata->flags & LF2000_VIP_ITU656)
		tmp += 2;

	writel(tmp, pcdev->base + VIP_IMGWIDTH);

	tmp = mf.height;

#if 0
	/* BUG: Erratum?  ODINT won't work unless we subtract 1 (matches VCOUNT) */
	tmp -= 1;
#endif

	writel(tmp, pcdev->base + VIP_IMGHEIGHT);

	pcdev->sensor_format.width = mf.width;
	pcdev->sensor_format.height = mf.height;

	pcdev->clip_rect.left = pcdev->clip_rect.top = 0;
	pcdev->clip_rect.width = pix->width;
	pcdev->clip_rect.height = pix->height;

	dev_dbg(icd->parent, "%s: pix=%dx%d, mf=%dx%d\n", __func__, pix->width, pix->height, mf.width, mf.height);

	pcdev->clip_rect.left = (mf.width - pix->width) / 2;
	pcdev->clip_rect.top = (mf.height - pix->height) / 2;

	/* Always set clipper in case it's used in pass-through mode*/
	switch(pipe_idx) {
	case VIP_OUTPUT_PIPE_DECIMATOR:
		pcdev->clip_rect.left = pcdev->clip_rect.top = 0;
		pcdev->clip_rect.width = mf.width;
		pcdev->clip_rect.height = mf.height;
		set_decimator(pcdev, pix->width, pix->height);
		/* Fall-through: must set source */
	case VIP_OUTPUT_PIPE_CLIPPER:
		set_clipper(pcdev, &pcdev->clip_rect);
		break;
	}

	/* swizzle buffer offsets for rotated aspect ratio */
	if (rotated) {
		pix->width  = rot_width;
		pix->height = rot_height;
	}

	/* Address will be set by subsequent buf_queue() */
	pcdev->target_height[LF2000_VIP_PIPE_CAPTURE] = pix->height;
	ret = setup_pipe(pcdev, (dma_addr_t)NULL, pix, LF2000_VIP_PIPE_CAPTURE);

out:
	return ret;
}

/*****************************************************************************
 * soc_camera_host_ops.try_fmt()
 *
 * Provided for userspace to determine hardware limitations.  Does not change
 * driver state.  Can be called at any time; may not return -EBUSY.
 *
 * TODO: see function body
 ****************************************************************************/
static int lf2000_camera_try_fmt(struct soc_camera_device *icd,
					struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	const struct soc_camera_format_xlate *xlate;
	const struct soc_mbus_pixelfmt *host_fmt;
 	int ret = 0;
 
	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	/* confirm presence of requested fourcc in lf2000_camera_fmt */
	host_fmt = lf2000_camera_format(pix);
	if(host_fmt == NULL)
	{
		/* Requested format not supported - suggest alternative */
		pix->pixelformat = V4L2_PIX_FMT_YUV420;
	}
	if(pix->pixelformat == V4L2_PIX_FMT_YUYV)
	{
		pix->priv &= ~(PIPE_TO_FMT_PRIV(VIP_OUTPUT_PIPE_DECIMATOR));
		pix->priv |= PIPE_TO_FMT_PRIV(VIP_OUTPUT_PIPE_CLIPPER);
	}

	/* TODO: colorspace is dependent upon sensor */
	switch(pix->colorspace) {
	case V4L2_COLORSPACE_SMPTE170M:
	case V4L2_COLORSPACE_JPEG:
		break;
	default:
		/* Requested colorspace not supported - suggest full range */ 
		pix->colorspace = V4L2_COLORSPACE_JPEG;
	}

	/* Limit request to sensor capabilities */
	mf.width	= min_t(__u32, pix->width, VIP_MAX_WIDTH);
	mf.height	= min_t(__u32, pix->height, VIP_MAX_HEIGHT);
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;

	/*
	 * Returns the first MBUS_FMT with a matching PIX_FMT (fourcc).
	 * Valid combinations (overlaping MBUS_FMTs between LF2000 and
	 * sensor) are filtered by .get_formats()
	 */
	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	/*
	 * xlate must be non-NULL here.  The soc framework fails initialization
	 * when .get_formats() provides nothing.
	 */

	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if(ret < 0)
		goto out;

	switch(pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
		pix->bytesperline = 2*mf.width;
		break;
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV420:
		pix->bytesperline = 4096;
		break;
	}

	/* We can clip / decimate sensor output */
	if((pix->width < mf.width) && (pix->height < mf.height))
	{
		pix->width	= pix->width & ~(0xF);
		pix->height	= pix->height & ~(0x1);
	}
	else
	{
		pix->width	= mf.width & ~(0xF);
		pix->height	= mf.height & ~(0x1);
	}
	pix->field	= mf.field;
	pix->colorspace	= mf.colorspace;

out:
	return ret;
}

/*****************************************************************************
 * soc_camera_host_ops.get_formats()
 *
 ****************************************************************************/
static int lf2000_camera_get_formats(struct soc_camera_device *icd,
		unsigned int idx, struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	enum v4l2_mbus_pixelcode code;
	int formats = 0, ret, limit;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if(ret < 0)
	{
		/* no more formats */
		goto out;
	}

	/* Supported mbus formats */
	switch(code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
	case V4L2_MBUS_FMT_YVYU8_2X8:
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
		/*
		 * MBUS_FMT and PIX_FMT are independent.
		 */

		limit = ARRAY_SIZE(lf2000_camera_fmt);

		ret = ((0x1 << LF2000_VIP_ROTATE) |
		       (0x1 << LF2000_VIP_HFLIP) |
		       (0x1 << LF2000_VIP_VFLIP));

		/*
		 * Do not publish packed YUYV if rotation is requested.
		 */
		if(pcdev->pdata->flags & ret)
		{
			--limit;
		}

		for(ret = 0; ret < limit; ret++)
		{
			if(xlate)
			{
				xlate->host_fmt = &lf2000_camera_fmt[ret];
				xlate->code = code;
				xlate++;
			}
			formats++;
		}
		break;
	default:
		break;
	}

	/* TODO: raw pass-through? */	

out:
	return formats;
}

/*****************************************************************************
 * soc_camera_host_ops.init_videobuf()
 *
 * Provided for soc_camera framework to initialize video buffers upon device
 * open.
 *
 * TODO: upgrade to videobuf2 in 2.6.39+
 ****************************************************************************/
static void lf2000_camera_init_videobuf(struct videobuf_queue *q,
					struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	enum v4l2_field field;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	if(pcdev->interlaced)
		field = V4L2_FIELD_INTERLACED;
	else
		field = V4L2_FIELD_NONE;

	videobuf_queue_dma_contig_init(q, &lf2000_videobuf_ops,
					icd->parent, &pcdev->lock,
					V4L2_BUF_TYPE_VIDEO_CAPTURE,
					field,
					sizeof(struct lf2000_buffer), icd,
					NULL);
	return;
}

/*****************************************************************************
 * soc_camera_host_ops.reqbufs()
 *
 * Provided for userspace to initiate memory mapped or user-pointer I/O.
 ****************************************************************************/
static int lf2000_camera_reqbufs(struct soc_camera_device *icd,
				struct v4l2_requestbuffers *p)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_pix_format	*fmt = &pcdev->pipe_format[LF2000_VIP_PIPE_CAPTURE];
	int i;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	pcdev->buf_type = p->memory;

	for(i = 0; i < p->count; i++)
	{
		struct videobuf_buffer	*vb = icd->vb_vidq.bufs[i];
		struct lf2000_buffer *buf = container_of(icd->vb_vidq.bufs[i],
						struct lf2000_buffer, vb);

		vb->bytesperline	= fmt->bytesperline;
		vb->width		= fmt->width;
		vb->height		= fmt->height;
		vb->field		= fmt->field;

		INIT_LIST_HEAD(&buf->vb.queue);
	}

	return 0;
}

/*****************************************************************************
 * soc_camera_host_ops.poll()
 *
 * Provided for userspace to suspend execution until the driver has captured
 * data.
 ****************************************************************************/
static unsigned int lf2000_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;
	struct lf2000_buffer *buf;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	buf = list_entry(icd->vb_vidq.stream.next, struct lf2000_buffer,
			 vb.stream);

	poll_wait(file, &buf->vb.done, pt);

	if(buf->vb.state == VIDEOBUF_DONE || buf->vb.state == VIDEOBUF_ERROR)
		return POLLIN | POLLRDNORM;

	return 0;
}

/*****************************************************************************
 * soc_camera_host_ops.querycap()
 *
 * Provided for userspace to determine driver capabilities.  Does not change
 * driver state.
 ****************************************************************************/
static int lf2000_camera_querycap(struct soc_camera_host *ici,
					struct v4l2_capability *cap)
{
	struct lf2000_vip_dev *pcdev = ici->priv;

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

	strlcpy(cap->card, "LF2000 Camera", sizeof(cap->card));
	cap->version = KERNEL_VERSION(0,0,1);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OVERLAY |
		V4L2_CAP_STREAMING;

	return 0;
}


/*****************************************************************************
 * soc_camera_host_ops.overlay()
 *
 ****************************************************************************/
static int lf2000_camera_overlay(struct soc_camera_host *ici, const unsigned int enable)
{
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(pcdev->icd);
	struct v4l2_pix_format	*fmt = &pcdev->pipe_format[LF2000_VIP_PIPE_OVERLAY];
	int pipe_idx = FMT_PRIV_TO_PIPE(fmt->priv);
	int ret, cap_idx;

	unsigned long flags;

	spin_lock_irqsave(&pcdev->lock, flags);

	ret = enable_pipe(pcdev, LF2000_VIP_PIPE_OVERLAY, enable);

	spin_unlock_irqrestore(&pcdev->lock, flags);

	if(pipe_idx == VIP_OUTPUT_PIPE_CLIPPER)
		cap_idx = VIP_OUTPUT_PIPE_DECIMATOR;
	else
		cap_idx = VIP_OUTPUT_PIPE_CLIPPER;

	/* soc_camera calls s_stream for capturing */
	if(pcdev->capture == false)
		ret = v4l2_subdev_call(sd, video, s_stream, enable);

	return ret;
}

/*****************************************************************************
 * soc_camera_host_ops.get_fbuf()
 *
 ****************************************************************************/
static int lf2000_camera_get_fbuf(struct soc_camera_host *ici,
					struct v4l2_framebuffer *fb)
{
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_pix_format	*fmt = &pcdev->pipe_format[LF2000_VIP_PIPE_OVERLAY];

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

	fb->capability = 0; /* TODO: V4L2_FBUF_CAP_GLOBAL_ALPHA? */
	fb->flags = V4L2_FBUF_FLAG_OVERLAY;

	fb->fmt = *fmt;

	return 0;
}

/*****************************************************************************
 * soc_camera_host_ops.set_fbuf()
 *
 ****************************************************************************/
static int lf2000_camera_set_fbuf(struct soc_camera_host *ici,
					struct v4l2_framebuffer *fb)
{
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_rect clip;
	int ret = 0, pipe_idx;
	unsigned long div, ar_w, ar_h;
	u16 w, h, src_w, src_h;

	dev_dbg(&pcdev->pdev->dev, "%s: %x\n", __FUNCTION__, fb->fmt.priv);

	/* Check if the requested output pipe is already active */
	pipe_idx = FMT_PRIV_TO_PIPE(fb->fmt.priv);
	if(pcdev->output_pipe[pipe_idx] != LF2000_VIP_PIPE_IDLE)
	{
		ret = -EBUSY;
		goto out;
	}

	if(pipe_idx == VIP_OUTPUT_PIPE_CLIPPER)
	{
		src_w = pcdev->sensor_format.width;
		src_h = pcdev->sensor_format.height;
	}
	else
	{
		src_w = pcdev->clip_rect.width;
		src_h = pcdev->clip_rect.height;
	}

	dev_dbg(&pcdev->pdev->dev, "%s: sens=%dx%d, clip=%dx%d, fb=%dx%d\n", __func__,
			pcdev->sensor_format.width, pcdev->sensor_format.height,
			pcdev->clip_rect.width, pcdev->clip_rect.height,
			fb->fmt.width, fb->fmt.height);

	/* Can't decimate/clip a smaller source image */
	if((src_w < fb->fmt.width) || (src_h < fb->fmt.height))
	{
		dev_err(&pcdev->pdev->dev, "%s: bad src size\n", __FUNCTION__);
		ret = -EINVAL;
		goto out;
	}

	/* width must be divisible by 16, height by 2 */
	if((fb->fmt.width & 0xF) || (fb->fmt.height & 0x1))
	{
		/* use requested framebuffer as-is */
		if(FMT_PRIV_TO_DIM(fb->fmt.priv))
		{
			dev_err(&pcdev->pdev->dev, "%s: bad fb fmt\n", __FUNCTION__);
			ret = -EINVAL;
			goto out;
		}
		dev_warn(&pcdev->pdev->dev,
			"%s: overlay will be adjusted\n", __FUNCTION__);
	}

	/* find input aspect ratio.  Usually 4:3 */
	div	= gcd(src_w, src_h);
	ar_w	= src_w / div;
	ar_h	= src_h / div;

	/* maintain aspect ratio */
	if(((fb->fmt.width*ar_h != fb->fmt.height*ar_w)) &&
	   FMT_PRIV_TO_AR(fb->fmt.priv))
	{
		if(FMT_PRIV_TO_DIM(fb->fmt.priv))
		{
			dev_err(&pcdev->pdev->dev, "%s: bad fb fmt\n", __FUNCTION__);
			ret = -EINVAL;
			goto out;
		}
		dev_warn(&pcdev->pdev->dev,
			"%s: overlay will be adjusted\n", __FUNCTION__);
	}

	/* TODO: magic number */
	if(FMT_PRIV_TO_FB(fb->fmt.priv) != 2)
	{
		dev_err(&pcdev->pdev->dev,
			"%s: video layer mismatch.\n", __FUNCTION__);
		ret = -EINVAL;
		goto out;
	}

	/* target dimensions must match display window */
	pcdev->fb = registered_fb[FMT_PRIV_TO_FB(fb->fmt.priv)];
	if((fb->fmt.width > pcdev->fb->var.xres) ||
	   (fb->fmt.height > pcdev->fb->var.yres))
	{
		dev_err(&pcdev->pdev->dev, "%s: bad fb res\n", __FUNCTION__);
		ret = -EINVAL;
		goto out;
	}

	pcdev->doublebuf = false;

#ifdef CONFIG_VIDEO_LF2000_DOUBLEBUF
	if(FMT_PRIV_TO_DOUBLE_BUF(fb->fmt.priv) == VIP_OVERLAY_DOUBLE_BUF)
	{
		pcdev->fb = registered_fb[FMT_PRIV_TO_FB(fb->fmt.priv)];
		/* Requires side-by-side buffers */
		if(fb->fmt.width > pcdev->fb->var.xres_virtual/4)
		{
			dev_err(&pcdev->pdev->dev, "%s: bad fb virt\n", __FUNCTION__);
			ret = -EINVAL;
			goto out;
		}
		pcdev->fbbase = (dma_addr_t)fb->base;
		pcdev->doublebuf = true;
		pcdev->fb->var.xoffset = 0;
	}
#endif

	/*
	 * The framebuffer format (dimensions, fourCC) is in fb->fmt.
	 * The camera format (dimensions, fourCC) was set with a previous
	 * call to lf2000_camera_set_fmt().  The fourCCs of the two formats
	 * must match, but the dimensions can differ.
	 *
	 * If fb->fmt is adjusted below, that implies we target only a
	 * subregion of the framebuffer, hence the dev_warn()s above.
	 */

	/* width must be divisible by 16, height by 2 */
	fb->fmt.width &= ~(0xF);
	fb->fmt.height &= ~(0x1);

	/* find largest possible ar_w:ar_h (e.g., 4:3) window within fb->fmt */
	while(FMT_PRIV_TO_AR(fb->fmt.priv) &&
	      (fb->fmt.width*ar_h != fb->fmt.height*ar_w))
	{
		if(((__s32)fb->fmt.width < 16) || ((__s32)fb->fmt.height < 2))
		{
			dev_err(&pcdev->pdev->dev, "%s: bad fmt size\n", __FUNCTION__);
			ret = -EINVAL;
			goto out;
		}

		/* Limited by smaller dimension */
		if(fb->fmt.width*ar_h <= fb->fmt.height*ar_w)
		{
			fb->fmt.height = ar_h*fb->fmt.width/ar_w;
		}
		else
		{
			fb->fmt.width = ar_w*fb->fmt.height/ar_h;
		}

		/* round down */
		fb->fmt.width &= ~(0xF);
		fb->fmt.height &= ~(0x1);
	}

	/* the framebuffer's true y resolution could differ from fb->fmt */
	pcdev->target_height[LF2000_VIP_PIPE_OVERLAY] = fb->fmt.height; //pcdev->fb->var.yres;
	ret = setup_pipe(pcdev, (dma_addr_t)fb->base, &fb->fmt,
					LF2000_VIP_PIPE_OVERLAY);

	if(ret) {
		dev_err(&pcdev->pdev->dev, "%s: setup_pipe failed\n", __FUNCTION__);
		goto out;
	}

	w = fb->fmt.width;
	h = fb->fmt.height;

	switch(pipe_idx) {
	case VIP_OUTPUT_PIPE_CLIPPER:
		/*
		 * Clip to top left corner by default.  Can be adjusted with
		 * VIDIOC_S_CROP.
		 */
		pcdev->clip_rect.left = pcdev->clip_rect.top = 0;
		pcdev->clip_rect.width = w;
		pcdev->clip_rect.height = h;
		set_clipper(pcdev, &pcdev->clip_rect);
		break;

	case VIP_OUTPUT_PIPE_DECIMATOR:
		set_decimator(pcdev, w, h);
		break;
	}

out:
	return ret;
}

static int lf2000_camera_get_ctrl(struct soc_camera_device *icd, struct v4l2_control *ctl)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	switch (ctl->id) {
	case V4L2_CID_HFLIP:
		ctl->value = (pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] & LF2000_VIP_HFLIP) ? 1 : 0;
		break;
	case V4L2_CID_VFLIP:
		ctl->value = (pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] & LF2000_VIP_VFLIP) ? 1 : 0;
		break;
	case V4L2_CID_ROTATE:
		ctl->value = (pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] & LF2000_VIP_ROTATE) ? 90 : 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int lf2000_camera_set_ctrl(struct soc_camera_device *icd, struct v4l2_control *ctl)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	unsigned long flags;

	dev_dbg(icd->parent, "%s\n", __FUNCTION__);

	switch (ctl->id) {
	case V4L2_CID_HFLIP:
		if (ctl->value)
			pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] |= LF2000_VIP_HFLIP;
		else
			pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] &= ~LF2000_VIP_HFLIP;
		spin_lock_irqsave(&pcdev->lock, flags);
		set_flip_rotate(pcdev, pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE], LF2000_VIP_PIPE_CAPTURE);
		spin_unlock_irqrestore(&pcdev->lock, flags);
		break;
	case V4L2_CID_VFLIP:
		if (ctl->value)
			pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] |= LF2000_VIP_VFLIP;
		else
			pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] &= ~LF2000_VIP_VFLIP;
		spin_lock_irqsave(&pcdev->lock, flags);
		set_flip_rotate(pcdev, pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE], LF2000_VIP_PIPE_CAPTURE);
		spin_unlock_irqrestore(&pcdev->lock, flags);
		break;
	case V4L2_CID_ROTATE:
		if (ctl->value)
			pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] |= LF2000_VIP_ROTATE;
		else
			pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] &= ~LF2000_VIP_ROTATE;
		spin_lock_irqsave(&pcdev->lock, flags);
		set_flip_rotate(pcdev, pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE], LF2000_VIP_PIPE_CAPTURE);
		spin_unlock_irqrestore(&pcdev->lock, flags);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int lf2000_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct soc_camera_device *icd = container_of(ctrl->handler, struct soc_camera_device, ctrl_handler);
	struct v4l2_control ctl;

	if (!ctrl)
		return -EINVAL;

	ctl.id = ctrl->id;
	ctl.value = ctrl->val;
	printk(KERN_INFO "lf2000_s_ctrl: %d, %d\n", ctl.id, ctl.value);
	return lf2000_camera_set_ctrl(icd, &ctl);
}

static const struct v4l2_queryctrl lf2000_camera_controls[] = {
	{
		.id            = V4L2_CID_HFLIP,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "Mirror Horizontally",
		.minimum       = 0,
		.maximum       = 1,
		.step          = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_VFLIP,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
		.name          = "Flip Vertically",
		.minimum       = 0,
		.maximum       = 1,
		.step          = 1,
		.default_value = 0,
	},
	{
		.id            = V4L2_CID_ROTATE,
		.type          = V4L2_CTRL_TYPE_INTEGER,
		.name          = "Rotation Clockwise",
		.minimum       = 0,
		.maximum       = 90,
		.step          = 90,
		.default_value = 0,
	},
};

static struct soc_camera_host_ops lf2000_host_ops = {
	.owner		= THIS_MODULE,
	.add		= lf2000_camera_add_device,
	.remove		= lf2000_camera_remove_device,
	.set_fmt	= lf2000_camera_set_fmt,
	.try_fmt	= lf2000_camera_try_fmt,
	.get_formats	= lf2000_camera_get_formats,
	.init_videobuf	= lf2000_camera_init_videobuf,
	.reqbufs	= lf2000_camera_reqbufs,
	.querycap	= lf2000_camera_querycap,
	.set_bus_param	= lf2000_camera_set_bus_param,
	.poll		= lf2000_camera_poll,
#if 1 // FIXME
	.overlay	= lf2000_camera_overlay,
	.get_fbuf	= lf2000_camera_get_fbuf,
	.set_fbuf	= lf2000_camera_set_fbuf,
	.get_ctrl	= lf2000_camera_get_ctrl,
	.set_ctrl	= lf2000_camera_set_ctrl,
	.controls	= lf2000_camera_controls,
	.num_controls	= ARRAY_SIZE(lf2000_camera_controls),
#endif
};

static const struct v4l2_ctrl_ops lf2000_ctrl_ops = {
	.s_ctrl = lf2000_s_ctrl,
};

static int lf2000_camera_add_controls(struct soc_camera_device *icd)
{
	int i;
	static bool once = false;

	if (once)
		return 0;
	once = true;

	for (i = 0; i < ARRAY_SIZE(lf2000_camera_controls); i++) {
		printk(KERN_INFO "adding control %d: %d\n", i, lf2000_camera_controls[i].id);
		v4l2_ctrl_new_std(&icd->ctrl_handler, &lf2000_ctrl_ops,
				lf2000_camera_controls[i].id,
				lf2000_camera_controls[i].minimum,
				lf2000_camera_controls[i].maximum,
				lf2000_camera_controls[i].step,
				lf2000_camera_controls[i].default_value);
	}
	return icd->ctrl_handler.error;
}

static irqreturn_t vip_isr(int irq, void *data)
{
	struct lf2000_vip_dev *pcdev = data;
	struct videobuf_buffer *vb;
	struct v4l2_pix_format *pix;
	struct fb_var_screeninfo *var;
	int pipe_idx;
	dma_addr_t addr;
	U16 enb, config, intc, odint;
	unsigned long flags;

	intc	= readl(pcdev->base + VIP_INTCTRL);
	odint	= readl(pcdev->base + VIP_ODINT);

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

	if (intc & (0x1 << HSINTENB))
		goto out;

	if(pcdev->bad_frame < pcdev->skip_frames)
	{
		pcdev->bad_frame++;
		pcdev->unblank = false;

		goto out;
	}
	if(pcdev->bad_frame == pcdev->skip_frames)
	{
		pcdev->bad_frame++;
		pcdev->unblank = true;
		/*
		 * Bad frames have been dropped - switch from VS to OD
		 * and enable writing to memory.
		 */
		intc	&= ~(0x1 << VSINTENB);
		odint	|= (0x1 << ODINTENB);
		writel(pcdev->cd_enb, pcdev->base + VIP_CDENB);

		goto out;
	}

	if(!(odint & (0x1 << ODINTPEND)))
	{
		goto out;
	}

#ifdef CONFIG_VIDEO_LF2000_SYSFS
	write_lock_irqsave(&pcdev->sysfs_lock, flags);
	pcdev->frame_count++;
	write_unlock_irqrestore(&pcdev->sysfs_lock, flags);
#endif

	spin_lock_irqsave(&pcdev->lock, flags);

	/* Handle capture */
	if(pcdev->active)
	{
		/* Signal completed capture */		
		vb = &pcdev->active->vb;
		WARN_ON(list_empty(&vb->queue));

		list_del_init(&vb->queue);
		vb->state = VIDEOBUF_DONE;
		do_gettimeofday(&vb->ts);
		vb->field_count++;

		/* Flush write buffer before userspace tries to read */
		mb();

		wake_up(&vb->done);

		pix = &pcdev->pipe_format[LF2000_VIP_PIPE_CAPTURE];

		if(list_empty(&pcdev->cap_list))
		{
			/* Queue depleted */

			enb	= readl(pcdev->base + VIP_CDENB);
			config	= readl(pcdev->base + VIP_CONFIG);

			pcdev->active = NULL;
			pipe_idx = FMT_PRIV_TO_PIPE(pix->priv);

			if(pipe_idx == VIP_OUTPUT_PIPE_CLIPPER)
				enb &= ~(0x1 << CLIPENB);
			else
				enb &= ~(0x1 << DECIENB);

			if(pcdev->overlay == false)
			{
				enb	&= ~(0x1 << SEPENB);
				config	&= ~(0x1 << VIPENB);
				odint	&= ~(0x1 << ODINTENB);
				/*
				 * All interrupts now off. Will be re-enabled
				 * via lf2000_videobuf_queue()->enable_pipe()
				 */
				writel(config, pcdev->base + VIP_CONFIG);
			}

			writel(enb, pcdev->base + VIP_CDENB);
		}
		else
		{
			/* Grab next buffer and point HW to it */
			pcdev->active = list_entry(pcdev->cap_list.next,
						struct lf2000_buffer, vb.queue);

			pcdev->active->vb.state = VIDEOBUF_ACTIVE;

			setup_pipe(pcdev, pcdev->active->addr, pix, LF2000_VIP_PIPE_CAPTURE);
		}
	}

	spin_unlock_irqrestore(&pcdev->lock, flags);

	/* Handle overlay (overlay is automatic except for double buffering) */
	if(pcdev->doublebuf)
	{
		pix = &pcdev->pipe_format[LF2000_VIP_PIPE_OVERLAY];
		var = &pcdev->fb->var;

		/* Show completed image */
		fb_pan_display(pcdev->fb, var);

		if (pcdev->unblank) {
			// FIXME: soc_dpc_set_vid_enable(0, 1);
			nxp_soc_disp_video_set_enable(0, 1);
			pcdev->unblank = false;
		}

		var->xoffset ^= pix->width;
		addr = pcdev->fbbase + var->xoffset;

		/* Swap to other buffer */
		setup_pipe(pcdev, addr, pix, LF2000_VIP_PIPE_OVERLAY);
	}

out:
	writel(odint, pcdev->base + VIP_ODINT);
	writel(intc, pcdev->base + VIP_INTCTRL);

	return IRQ_HANDLED;
}

/* FIXME: This function might be redundant except setting up default port */
static void vip_setup_pins(struct lf2000_vip_dev *pcdev)
{
	int i, len;
	u_int pad, fn;
	u_int (*pads)[2];
	u16 tmp;

#if 0	// device.c callback
	/* TODO: refactor Nexell GPIO stuff */

	/* VCLK27 is controlled by soc_camera_link.power() */

	/* VIP0:0 = VCLK, VID[0 ~ 7] */
	const u_int port_0[][2] = {
		/* VCLK */
		{ PAD_GPIO_C + 0, NX_GPIO_PADFUNC_2 },
		/* DATA */
		{ PAD_GPIO_C + 8, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 9, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 10, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 11, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 12, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 13, NX_GPIO_PADFUNC_2 },
	};

	/* VIP0:1 =  VCLK_B, VID_B[0 ~ 7], VHSYNC_B, VVSYNC_B, VDVALID_B  */
	const u_int port_1[][2] = {
		/* VCLK_B */
		{ PAD_GPIO_C + 22, NX_GPIO_PADFUNC_2 },
		/* DATA_B */
		{ PAD_GPIO_C + 25, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 26, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 27, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 28, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 29, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 30, NX_GPIO_PADFUNC_1 },
		{ PAD_GPIO_C + 31, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_D + 0, NX_GPIO_PADFUNC_2 },
		/* HSYNC, VSYNC */
		{ PAD_GPIO_C + 24, NX_GPIO_PADFUNC_2 },
		{ PAD_GPIO_C + 21, NX_GPIO_PADFUNC_2 },
		/* DVALID */
		{ PAD_GPIO_C + 23, NX_GPIO_PADFUNC_2 },
	};

	dev_info(&pcdev->pdev->dev, "%s\n", __FUNCTION__);
	
	tmp = readl(pcdev->base + VIP_VIP1);

	if(pcdev->pdata->flags & LF2000_VIP_PORT)
	{
		dev_info(&pcdev->pdev->dev, "Selecting port 1 : %d\n", pcdev->pdata->flags);
		pads = port_1;
		len = ARRAY_SIZE(port_1);
		tmp |= (1 << VIPPORTSEL);
	}
	else
	{
		dev_info(&pcdev->pdev->dev, "Selecting port 0 : %d\n", pcdev->pdata->flags);
		pads = port_0;
		len = ARRAY_SIZE(port_0);
		tmp &= ~(1 << VIPPORTSEL);
	}
	writel(tmp, pcdev->base + VIP_VIP1);

	for (i = 0; i < len; i++)
	{
		pad = pads[i][0];	
		fn = pads[i][1];
		// FIXME: soc_gpio_set_io_func(pad, fn);
		nxp_soc_gpio_set_io_func(pad, fn);
	}
#endif
}

static void vip_setup_clocks(struct lf2000_vip_dev *pcdev)
{
	U32 tmp;
	int module = VIPX;
	int port = 0;
	int extsync = (pcdev->pdata->flags & LF2000_VIP_ITU656) ? CFALSE : CTRUE;
	int i;

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

#if 0	// not LF3000
	tmp = readl(pcdev->base + VIP_CLKENB);
	tmp &= ~(0x3 << BCLKMODE);

	tmp |= 0x1 << PCLKMODE;	/* PCLK always on */
	tmp |= 0x2 << BCLKMODE;	/* BCLK dynamic */
	writel(tmp, pcdev->base + VIP_CLKENB);


	tmp = readl(pcdev->base + VIP_CLKGENL);
	tmp &= ~((0x7 << CLKSRCSEL) | (0x7F << CLKDIV0));

	tmp |= 0x3 << CLKSRCSEL;	/* ICLK */
	tmp |= 0 << CLKDIV0;	/* divisor = 1 */
	writel(tmp, pcdev->base + VIP_CLKGENL);


	tmp = readl(pcdev->base + VIP_CLKENB);

	tmp |= 0x1 << CLKGENENB;	/* Enable clock generation */
	writel(tmp, pcdev->base + VIP_CLKENB);
#endif

	for (i = 0; i < 2; i++)
	{
	module = i;
	port = !i; // VIP0/VID2 or VIP1/VID0

	// VIP clock setup from Nexell V4L driver
	printk("%s: vip %d: clk idx = %d @ %08x\n", __func__, module, NX_VIP_GetClockNumber(module), NX_CLKGEN_GetPhysicalAddress(NX_VIP_GetClockNumber(module)));
    NX_CLKGEN_SetBaseAddress(NX_VIP_GetClockNumber(module), NX_CLKGEN_GetPhysicalAddress(NX_VIP_GetClockNumber(module)));
    NX_CLKGEN_SetClockDivisorEnable(NX_VIP_GetClockNumber(module), CTRUE);
    NX_CLKGEN_SetClockBClkMode(NX_VIP_GetClockNumber(module), NX_BCLKMODE_DYNAMIC);
//    NX_CLKGEN_SetClockPClkMode(NX_VIP_GetClockNumber(module), NX_PCLKMODE_ALWAYS);
    NX_RSTCON_SetnRST(NX_VIP_GetResetNumber(module), RSTCON_nDISABLE);
    NX_RSTCON_SetnRST(NX_VIP_GetResetNumber(module), RSTCON_nENABLE);
    NX_CLKGEN_SetClockSource(NX_VIP_GetClockNumber(module), 0, 4 + port); /* external PCLK 0 or 1 */
    NX_CLKGEN_SetClockDivisor(NX_VIP_GetClockNumber(module), 0, 1);
//    NX_CLKGEN_SetClockOutInv(NX_VIP_GetClockNumber(module), 0, 1);
//	NX_CLKGEN_SetClockSource(NX_VIP_GetClockNumber(module), 0, 0); // PLL0
//	NX_CLKGEN_SetClockDivisor(NX_VIP_GetClockNumber(module), 0, 16); // 800MHz / 16
    NX_CLKGEN_SetClockDivisorEnable(NX_VIP_GetClockNumber(module), CTRUE);
    volatile u32 *clkgen_base = (volatile u32 *)IO_ADDRESS(NX_CLKGEN_GetPhysicalAddress(NX_VIP_GetClockNumber(module)));
    *clkgen_base |= 1 << 4;
    *clkgen_base |= 1 << 2;

    // VIP data format setup for embedded sync input
    NX_VIP_SetHVSync(module, extsync, 2*pcdev->sensor_format.width, pcdev->sensor_format.height,
    		2*pcdev->pdata->hsw, 2*pcdev->pdata->hfp, 2*pcdev->pdata->hbp,
    		pcdev->pdata->vsw, pcdev->pdata->vfp, pcdev->pdata->vbp); // why horizontal*2?
    NX_VIP_SetDataMode(module, NX_VIP_DATAORDER_CBY0CRY1, 8);
    NX_VIP_SetFieldMode(module, extsync, 0, CFALSE, CFALSE);
    NX_VIP_SetDValidMode(module, CFALSE, CFALSE, CFALSE);
    NX_VIP_SetFIFOResetMode(module, NX_VIP_FIFORESET_FRAMEEND);
    NX_VIP_SetInputPort(module, NX_VIP_INPUTPORT_A + port);

    NX_VIP_SetInterruptEnableAll(module, CFALSE);
    NX_VIP_ClearInterruptPendingAll(module);

	}
}

static int __init lf2000_vip_probe(struct platform_device *pdev)
{
	struct lf2000_vip_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	unsigned int irq;
	int err = 0, i;

	printk(KERN_INFO "%s\n", __FUNCTION__);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if(!res || (int)irq < 0)
	{
		err = -ENODEV;
		goto exit;
	}

	NX_VIP_Initialize();
	NX_VIP_SetBaseAddress(0, NX_VIP_GetPhysicalAddress(0));
	NX_VIP_SetBaseAddress(1, NX_VIP_GetPhysicalAddress(1));

	printk("%s: vip res: mem=%08x, irq=%d\n", __func__, res->start, irq);
	printk("%s: vip vtk: mem=%08x, irq=%d\n", __func__, NX_VIP_GetPhysicalAddress(0), NX_VIP_GetInterruptNumber(0));
	printk("%s: vip vtk: mem=%08x, irq=%d\n", __func__, NX_VIP_GetPhysicalAddress(1), NX_VIP_GetInterruptNumber(1));

	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if(!pcdev)
	{
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	pcdev->pdev = pdev;
	pcdev->pdata = pdev->dev.platform_data;
	pcdev->res = res;

	INIT_LIST_HEAD(&pcdev->cap_list);
	spin_lock_init(&pcdev->lock);

	/*
	 * Request the region.
	 */
	if(!request_mem_region(res->start, resource_size(res), res->name))
	{
		err = -EBUSY;
		goto exit_kfree;
	}

#if 0
	base = ioremap(res->start, resource_size(res));
	if(!base)
	{
		err = -ENOMEM;
		goto exit_release;
	}
#else
	base = NX_VIP_GetPhysicalAddress(VIPX);
	irq  = NX_VIP_GetInterruptNumber(VIPX);
#endif
	pcdev->irq = irq;
	pcdev->base = base;

	snprintf(pcdev->name, sizeof(pcdev->name), "%s%u", pdev->name, pdev->id);
	err = request_irq(pcdev->irq, vip_isr, 0, pcdev->name, pcdev);
	if(err)
	{
		dev_err(&pdev->dev, "VIP interrupt register failed\n");
		goto exit_iounmap;
	}

	vip_setup_pins(pcdev);

	vip_setup_clocks(pcdev);

	pcdev->soc_host.drv_name	= dev_name(&pdev->dev);
	pcdev->soc_host.ops		= &lf2000_host_ops;
	pcdev->soc_host.priv		= pcdev;
	pcdev->soc_host.v4l2_dev.dev	= &pdev->dev;
	pcdev->soc_host.nr		= pdev->id;

	for(i = 0; i < ARRAY_SIZE(pcdev->output_pipe); i++)
	{
		pcdev->output_pipe[i] = LF2000_VIP_PIPE_IDLE;
	}
	pcdev->capture = false;
	pcdev->overlay = false;

	pcdev->skip_frames = DEFAULT_LF2000_SKIP_FRAMES;

	/* separate flip/rotate flags for capture vs overlay */
	pcdev->flip_flags[LF2000_VIP_PIPE_CAPTURE] = pcdev->pdata->flags;
	pcdev->flip_flags[LF2000_VIP_PIPE_OVERLAY] = pcdev->pdata->flags;

	/* TODO: set default pipe_format[] */

	err = soc_camera_host_register(&pcdev->soc_host);
	if(err)
		goto exit_free_irq;

	/* register control handler */
	v4l2_ctrl_handler_init(&pcdev->ctrl_handler, ARRAY_SIZE(lf2000_camera_controls)); //lf2000_host_ops.num_controls);
	for (i = 0; i < ARRAY_SIZE(lf2000_camera_controls); i++) {
		printk(KERN_INFO "adding control %d: %d\n", i, lf2000_camera_controls[i].id);
		v4l2_ctrl_new_std(&pcdev->ctrl_handler, &lf2000_ctrl_ops,
				lf2000_camera_controls[i].id,
				lf2000_camera_controls[i].minimum,
				lf2000_camera_controls[i].maximum,
				lf2000_camera_controls[i].step,
				lf2000_camera_controls[i].default_value);
	}
	if (pcdev->ctrl_handler.error) {
		err = pcdev->ctrl_handler.error;
		goto exit_ctrl;
	}

#ifdef CONFIG_VIDEO_LF2000_DEBUGFS
	vip_init_debugfs(pcdev);
#endif

#ifdef CONFIG_VIDEO_LF2000_SYSFS
	rwlock_init(&pcdev->sysfs_lock);
	sysfs_create_group(&pdev->dev.kobj, &vip_attr_group);
#endif
	init_rwsem(&pcdev->skip_mutex);
	
	dev_info(&pdev->dev, "LF2000 Video Input Port driver loaded\n");

	return 0;

exit_ctrl:
	v4l2_ctrl_handler_free(&pcdev->ctrl_handler);
exit_free_irq:
	free_irq(pcdev->irq, pcdev);
exit_iounmap:
	iounmap(base);
exit_release:
	release_mem_region(res->start, resource_size(res));
exit_kfree:
	kfree(pcdev);
exit:
	return err;
}

static int __exit lf2000_vip_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);

	struct lf2000_vip_dev *pcdev = container_of(soc_host,
					struct lf2000_vip_dev, soc_host);
	struct resource *res;

	v4l2_ctrl_handler_free(&pcdev->ctrl_handler);

	free_irq(pcdev->irq, pcdev);

	soc_camera_host_unregister(soc_host);

#ifdef CONFIG_VIDEO_LF2000_SYSFS
	sysfs_remove_group(&pdev->dev.kobj, &vip_attr_group);
#endif

#ifdef CONFIG_VIDEO_LF2000_DEBUGFS
	if(pcdev->debug)
		debugfs_remove_recursive(pcdev->debug);
#endif

	iounmap(pcdev->base);

	res = pcdev->res;
	release_mem_region(res->start, resource_size(res));

	kfree(pcdev);

	dev_info(&pdev->dev, "LF2000 Video Input Port driver unloaded\n");

	return 0;
}

static struct platform_driver lf2000_vip_driver __refdata = {
	.driver		= {
		.name	= VIP_DEV_NAME,
	},
	.probe		= lf2000_vip_probe,
	.remove		= __exit_p(lf2000_vip_remove),
};

static int __init lf2000_vip_init(void)
{
	return platform_driver_register(&lf2000_vip_driver);
}
module_init(lf2000_vip_init);

static void __exit lf2000_vip_exit(void)
{
	platform_driver_unregister(&lf2000_vip_driver);
}
module_exit(lf2000_vip_exit);


MODULE_DESCRIPTION("LF2000 Video Input Port (VIP) driver");
MODULE_LICENSE("GPL v2");
