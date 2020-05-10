/*
 * EXPERIMENTAL DRIVER
 *
 * V4L2 soc_camera_host driver for LF2000 Video Input Port
 *
 * Copyright (C) 2011-2012, LeapFrog Enterprises, Inc.
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
#include <linux/gcd.h>

#include <linux/lf2000/lf2000vip.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/videobuf-dma-contig.h>

#include <asm/io.h>

#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>
#include <mach/lf2000_vip.h>

#include "lf2000_vip.h"

#include <linux/lf1000/lf1000fb.h>
/* For optional double-buffering of overlay */
#include <linux/fb.h>

struct lf2000_buffer {
	struct videobuf_buffer		vb;
	dma_addr_t			addr;	/* cached - expensive lookup */
	enum v4l2_mbus_pixelcode	code;
	int				offset;
};

enum lf2000_vip_pipe {
	LF2000_VIP_PIPE_IDLE	= 0x0,	/* inactive */
	LF2000_VIP_PIPE_CAPTURE	= 0x1,	/* active with capture */
	LF2000_VIP_PIPE_OVERLAY	= 0x2,	/* active with overlay */
	LF2000_VIP_PIPE_BOTH	= 0x3,	/* active with both */
};

struct lf2000_vip_dev {
	struct platform_device		*pdev;
	struct lf2000_vip_platform_data	*pdata;

	struct soc_camera_host		soc_host;
	struct soc_camera_device	*icd;

	unsigned int			irq;
	void __iomem			*base;
	struct resource			*res;

	struct lf2000_buffer		*active;

	struct list_head		capture;

	enum v4l2_memory		buf_type;

	int				(*mmap_mapper)(struct videobuf_queue *q,
						struct videobuf_buffer *buf,
						struct vm_area_struct *vma);

	char				name[10];

	spinlock_t			lock;

	bool				interlaced;

	enum lf2000_vip_pipe		clipper_status;
	struct v4l2_pix_format		sensor_format;
	struct v4l2_pix_format		clipper_format;

	struct v4l2_rect		clip_rect;

#ifdef CONFIG_VIDEO_LF2000_DEBUGFS
	struct dentry			*debug;
	struct dentry			*debug_registers;
	struct dentry			*debug_clip_registers;
	struct dentry			*debug_deci_registers;
#endif

#ifdef CONFIG_VIDEO_LF2000_SYSFS
	rwlock_t			sysfs_lock;
	unsigned int			frame_count;
#endif
};

struct planar_coords {
	u16			seg[3];
	struct v4l2_rect	rect[3];
};

/* Local-scope prototypes */
static void update_sync(struct lf2000_vip_dev *pcdev, bool itu656);
static int setup_clipper(struct lf2000_vip_dev *pcdev, const dma_addr_t base, 
	const struct v4l2_pix_format *pix, enum lf2000_vip_pipe pipe_func);
static int enable_clipper(struct lf2000_vip_dev *pcdev,
		const enum lf2000_vip_pipe pipe_func, const bool enable);

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
	struct soc_camera_host	*ici = to_soc_camera_host(icd->dev.parent);
	struct lf2000_vip_dev	*pcdev = ici->priv;
	int			ret = 0;

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

	*size = pcdev->clipper_format.bytesperline * pcdev->clipper_format.height;

	if(!*count)
		*count = 2;

	dev_dbg(icd->dev.parent, "count=%d, size=%d\n", *count, *size);

	return ret;
}

/*****************************************************************************
 * videobuf helper
 *
 ****************************************************************************/
static void free_buffer(struct videobuf_queue *vq, struct lf2000_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host	*ici = to_soc_camera_host(icd->dev.parent);
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
	struct soc_camera_host	*ici = to_soc_camera_host(icd->dev.parent);
	struct lf2000_vip_dev	*pcdev = ici->priv;
	struct v4l2_pix_format	*fmt = &pcdev->clipper_format;

	struct lf2000_buffer *buf = container_of(vb, struct lf2000_buffer, vb);
	int ret = 0;

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

	/* Added list head initialization on alloc */
	WARN_ON(!list_empty(&vb->queue));

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
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;

		vb->state = VIDEOBUF_PREPARED;
		buf->addr = videobuf_to_dma_contig(vb);
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
 * Queue a buffer for I/O.
 ****************************************************************************/
static void lf2000_videobuf_queue(struct videobuf_queue *vq,
					struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct lf2000_buffer *buf = container_of(vb, struct lf2000_buffer, vb);
	struct v4l2_pix_format	*pix = &pcdev->clipper_format;
	dma_addr_t addr;

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

	list_add_tail(&vb->queue, &pcdev->capture);

	vb->state = VIDEOBUF_QUEUED;

	/* First buffer queued. */
	if(!pcdev->active)
	{
		vb->state = VIDEOBUF_ACTIVE;

		pcdev->active = buf;

		/* Load proper registers with buffer address */
		setup_clipper(pcdev, buf->addr, pix, LF2000_VIP_PIPE_CAPTURE);

		/* Start streaming */
		enable_clipper(pcdev, LF2000_VIP_PIPE_CAPTURE, true);
	}
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
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct lf2000_buffer *buf = container_of(vb, struct lf2000_buffer, vb);
	struct device *dev = icd->dev.parent;
	unsigned long flags;

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

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
	if(pcdev->active == NULL)
	{	
		enable_clipper(pcdev, LF2000_VIP_PIPE_CAPTURE, false);
	}
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

static void vip_set_planar(void __iomem *base, 
	const unsigned char fmt, const struct planar_coords *coords)
{
	/*
	 * Clipper and Decimator have identical planar registers -
	 * use decimator offsets to calculate both.
	 */
	u16 tmp = readw(base + VIP_DECI_FORMAT - VIP_DECI_LUSEG);

	tmp &= ~(0x3 << CLIP_FORMATSEL);
	tmp |= fmt;

	writew(tmp, base + VIP_DECI_FORMAT - VIP_DECI_LUSEG);

	writew(coords->seg[0], base + VIP_DECI_LUSEG - VIP_DECI_LUSEG);
	writew(coords->rect[0].left, base + VIP_DECI_LULEFT - VIP_DECI_LUSEG);
	writew(coords->rect[0].top, base + VIP_DECI_LUTOP - VIP_DECI_LUSEG);
	writew(coords->rect[0].width, base + VIP_DECI_LURIGHT - VIP_DECI_LUSEG);
	writew(coords->rect[0].height, base + VIP_DECI_LUBOTTOM - VIP_DECI_LUSEG);

	writew(coords->seg[1], base + VIP_DECI_CBSEG - VIP_DECI_LUSEG);
	writew(coords->rect[1].left, base + VIP_DECI_CBLEFT - VIP_DECI_LUSEG);
	writew(coords->rect[1].top, base + VIP_DECI_CBTOP - VIP_DECI_LUSEG);
	writew(coords->rect[1].width, base + VIP_DECI_CBRIGHT - VIP_DECI_LUSEG);
	writew(coords->rect[1].height, base + VIP_DECI_CBBOTTOM - VIP_DECI_LUSEG);

	writew(coords->seg[2], base + VIP_DECI_CRSEG - VIP_DECI_LUSEG);
	writew(coords->rect[2].left, base + VIP_DECI_CRLEFT - VIP_DECI_LUSEG);
	writew(coords->rect[2].top, base + VIP_DECI_CRTOP - VIP_DECI_LUSEG);
	writew(coords->rect[2].width, base + VIP_DECI_CRRIGHT - VIP_DECI_LUSEG);
	writew(coords->rect[2].height, base + VIP_DECI_CRBOTTOM - VIP_DECI_LUSEG);
}

static void set_clipper(struct lf2000_vip_dev *pcdev,
				const struct v4l2_rect *rect)
{
	u16 tmp;

	tmp = rect->left;
	writew(tmp, pcdev->base + VIP_CLIP_LEFT);
	tmp += rect->width;
	writew(tmp, pcdev->base + VIP_CLIP_RIGHT);

	tmp = rect->top;
	writew(tmp, pcdev->base + VIP_CLIP_TOP);
	tmp += rect->height;
	writew(tmp, pcdev->base + VIP_CLIP_BOTTOM);
}

static void update_sync(struct lf2000_vip_dev *pcdev, bool itu656)
{
	u16 tmp;

	if(itu656)
	{
		tmp = pcdev->pdata->vfp + 1;
		writew(tmp, pcdev->base + VIP_VBEGIN);
		tmp += pcdev->pdata->vsw;
		writew(tmp, pcdev->base + VIP_VEND);

		tmp = pcdev->pdata->hfp - 7;
		writew(tmp, pcdev->base + VIP_HBEGIN);
		tmp += pcdev->pdata->hsw;
		writew(tmp, pcdev->base + VIP_HEND);
	}
	else
	{
		tmp = pcdev->pdata->vbp - 1;
		writew(tmp, pcdev->base + VIP_VBEGIN);
		tmp += pcdev->sensor_format.height;
		writew(tmp, pcdev->base + VIP_VEND);

		tmp = pcdev->pdata->hbp - 1;
		writew(tmp, pcdev->base + VIP_HBEGIN);
		tmp += pcdev->sensor_format.width;
		writew(tmp, pcdev->base + VIP_HEND);
	}
}

static void vip_calculate_planar_addr(struct planar_coords *coords,
		const dma_addr_t ptr, const struct v4l2_pix_format *fmt)
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
	base = (uintptr_t)ptr + pitch/2 + pitch * fmt->height/2;
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

static int setup_clipper(struct lf2000_vip_dev *pcdev, const dma_addr_t base, 
	const struct v4l2_pix_format *pix, enum lf2000_vip_pipe pipe_func)
{
	struct planar_coords coords;
	unsigned char fmt;
	int ret = 0;
	u16 tmp;

	fmt = (pix->pixelformat == V4L2_PIX_FMT_YUV422P ) ?
			VIP_YUV422 : VIP_YUV420;

	vip_calculate_planar_addr(&coords, base, pix);
	vip_set_planar(pcdev->base + VIP_CLIP_LUSEG, fmt, &coords);

	/* Enable interrupts */
	/* TODO: why doesn't ODINT work w/VEND set? */
	tmp = readw(pcdev->base + VIP_INTCTRL);
	tmp |= (0x1 << VSINTENB);
	writew(tmp, pcdev->base + VIP_INTCTRL);

	/* Keep track of current format for capture/overlay */
	pcdev->clipper_format = *pix;

	return ret;
}

static int enable_clipper(struct lf2000_vip_dev *pcdev,
		const enum lf2000_vip_pipe pipe_func, const bool enable)
{
	u16 tmp, bit = 0;
	int ret = 0;

	dev_dbg(&pcdev->pdev->dev, "%s: %d\n", __FUNCTION__, enable);

	bit = (0x1 << CLIPENB);

	tmp = readw(pcdev->base + VIP_CDENB);

	if(enable)
	{
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
		tmp |= bit;
		pcdev->clipper_status |= pipe_func;
	}
	else
	{
		tmp &= ~bit;
		pcdev->clipper_status &= ~(pipe_func);

		switch(pipe_func) {
		case LF2000_VIP_PIPE_CAPTURE:
			if(pcdev->active)
			{
			/* TODO: dirty shutdown - closing while stream is active */
			}
			break;
		case LF2000_VIP_PIPE_OVERLAY:
			break;
		}
	}

	writew(tmp, pcdev->base + VIP_CDENB);

out:
	return ret;
}


/*****************************************************************************
 * soc_camera_host_ops
 *****************************************************************************/

/*****************************************************************************
 * soc_camera_host_ops.add_device()
 *
 * Provided for soc_camera framework to associate a camera to a camera host.
 *
 * TODO: activate clocks?
 ****************************************************************************/
static int lf2000_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
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
		 "LF2000 VIP driver attached to camera %d\n", icd->devnum);

	pcdev->icd = icd;

	config = rotate = 0x0;

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

	if(pcdev->pdata->flags & LF2000_VIP_ROTATE)
	{
		rotate |= (0x1 << CLIP_ROTATION);
	}
	if(pcdev->pdata->flags & LF2000_VIP_HFLIP)
	{
		rotate |= (0x1 << CLIP_HFLIP);
	}
	if(pcdev->pdata->flags & LF2000_VIP_VFLIP)
	{
		rotate |= (0x1 << CLIP_VFLIP);
	}

	tmp = readw(pcdev->base + VIP_CLIP_ROTFLIP);
	tmp &= ~(0x7);
	tmp |= rotate;
	writew(tmp, pcdev->base + VIP_CLIP_ROTFLIP);

	tmp = readw(pcdev->base + VIP_DECI_ROTFLIP);
	tmp &= ~(0x7);
	tmp |= rotate;
	writew(tmp, pcdev->base + VIP_DECI_ROTFLIP);

	/* Enable separator */
	tmp = readw(pcdev->base + VIP_CDENB);
	tmp |= (0x1 << SEPENB);
	writew(tmp, pcdev->base + VIP_CDENB);

	/* Enable VIP */
	tmp = config | (0x1 << VIPENB) | (0x1 << DWIDTH);

	writew(tmp, pcdev->base + VIP_CONFIG);

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
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	U16 tmp;

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

	BUG_ON(icd != pcdev->icd);

	enable_clipper(pcdev, LF2000_VIP_PIPE_BOTH, false);

	/* This VIDIOC_STREAMOFF is redundant for well-behaved applications */
	v4l2_subdev_call(sd, video, s_stream, 0);

	/* Disable interrupt sources */
	tmp = readw(pcdev->base + VIP_ODINT);
	tmp &= ~(0x1 << ODINTENB);
	writew(tmp, pcdev->base + VIP_ODINT);

	tmp = readw(pcdev->base + VIP_INTCTRL);
	tmp &= ~((0x1 << HSINTENB) | (0x1 << VSINTENB));
	writew(tmp, pcdev->base + VIP_INTCTRL);

	/* Disable separator */
	tmp = readw(pcdev->base + VIP_CDENB);
	tmp &= ~(0x1 << SEPENB);
	writew(tmp, pcdev->base + VIP_CDENB);

	/* Disable VIP */
	tmp = readw(pcdev->base + VIP_CONFIG);
	tmp &= ~(0x1 << VIPENB);
	writew(tmp, pcdev->base + VIP_CONFIG);

	dev_info(&pcdev->pdev->dev,
		 "LF2000 VIP driver detached from camera %d\n", icd->devnum);

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
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	unsigned long camera_flags, common_flags;
	int ret;
	U32 tmp;

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

	/* Find what bus settings sensor supports */
	camera_flags = icd->ops->query_bus_param(icd);

	/* See what overlaps */
	common_flags = soc_camera_bus_param_compatible(camera_flags,
							VIP_BUS_FLAGS);

	if(!common_flags)
		return -EINVAL;

	if(common_flags & SOCAM_PCLK_SAMPLE_RISING)
		common_flags &= ~SOCAM_PCLK_SAMPLE_FALLING;
	else
		common_flags &= ~SOCAM_PCLK_SAMPLE_RISING;

	/* Apply shared bus settings to sensor */
	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0)
		return ret;

	/* Apply to VIP */
	tmp = readl(pcdev->base + VIP_CLKGENL);
	tmp &= ~( 0x7 << CLKSRCSEL);

	if(common_flags & SOCAM_PCLK_SAMPLE_RISING)
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
static const struct soc_mbus_pixelfmt lf2000_camera_fmt = {
	.fourcc			= V4L2_PIX_FMT_YUV420,
	.name			= "Planar YUV 420",
	.bits_per_sample	= 8,
	.packing		= SOC_MBUS_PACKING_NONE,
	.order			= SOC_MBUS_ORDER_LE,
};

static const struct soc_mbus_pixelfmt *lf2000_camera_format(const struct v4l2_pix_format *pix)
{

	if(lf2000_camera_fmt.fourcc == pix->pixelformat)
		return &lf2000_camera_fmt;

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
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	const struct soc_camera_format_xlate *xlate;
	int ret = 0;
	u16 tmp;

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

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
	tmp = readw(pcdev->base + VIP_CONFIG);
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
		dev_warn(icd->dev.parent, "No mbus format supported\n");
		ret = -EINVAL;
		goto out;
	}

	writew(tmp, pcdev->base + VIP_CONFIG);

	/* interlacing */
	tmp = readw(pcdev->base + VIP_SCANMODE);
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

	writew(tmp, pcdev->base + VIP_SCANMODE);

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

	writew(tmp, pcdev->base + VIP_IMGWIDTH);

	tmp = mf.height;
	writew(tmp, pcdev->base + VIP_IMGHEIGHT);

	pcdev->sensor_format.width = mf.width;
	pcdev->sensor_format.height = mf.height;

	pcdev->clip_rect.left = pcdev->clip_rect.top = 0;
	pcdev->clip_rect.width = pix->width;
	pcdev->clip_rect.height = pix->height;

	set_clipper(pcdev, &pcdev->clip_rect);

	/* Address will be set by subsequent buf_queue() */
	ret = setup_clipper(pcdev, (dma_addr_t)NULL, pix, LF2000_VIP_PIPE_CAPTURE);

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
 	int ret = 0;
 
	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

	pix->pixelformat = V4L2_PIX_FMT_YUV420;

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

	pix->bytesperline = 4096;

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
	enum v4l2_mbus_pixelcode code;
	int formats = 0, ret, limit;

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

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

		if(xlate)
		{
			xlate->host_fmt = &lf2000_camera_fmt;
			xlate->code = code;
			xlate++;
		}
		formats++;
		break;
	default:
		break;
	}

	/* TODO: raw pass-through? */	

out:
	return formats;
}

/*****************************************************************************
 * Trick copied from omap1_camera.c:  Local mmap_mapper wrapper:
 * use videobuf_dma_contig if necessary, but try to squeeze in "neighbors"
 ****************************************************************************/
struct videobuf_dma_contig_memory {
	u32 magic;
	void *vaddr;
	dma_addr_t dma_handle;
	unsigned long size;
};

static int lf2000_vip_mmap_mapper(struct videobuf_queue *q,
		struct videobuf_buffer *buf, struct vm_area_struct *vma)
{

	struct lf2000_vip_dev *pcdev = q->priv_data;
#if 0
	struct lf2000_buffer *lbuf = container_of(buf, struct lf2000_buffer, vb);
	struct lf2000_buffer *prev_lbuf;
	struct videobuf_buffer	*prev_buf;
	int bufs_per_alloc, prev_idx, ret = 0;
	struct v4l2_pix_format	*fmt = &pcdev->clipper_format;

	struct videobuf_dma_contig_memory *mem;
	struct videobuf_dma_contig_memory *prev_mem;
	struct videobuf_mapping *map;
	unsigned long size;

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

	/*
	 * When dealing with planar formats, we don't want the allocator
	 * (dma_alloc_coherent()) to be invoked for every requested buffer.
	 * Planar allocations have a fixed 4k stride, so we can potentially
	 * take advantage of that and fit several adjacent buffers
	 * ("neighbors") within one allocation.
	 */

	prev_idx = buf->i - 1;
printk(KERN_EMERG "%s: stride: %d width: %d\n", __FUNCTION__, buf->bytesperline, buf->width);

	/* Valid for packed and 422P, 420P, but not 444P */
	bufs_per_alloc = fmt->bytesperline / 2 / fmt->width;

	/* Packed, big planar, or first allocation */
	if((bufs_per_alloc == 1) || (prev_idx < 0))
		goto out;
printk(KERN_EMERG "%s: idx: %d\n", __FUNCTION__, prev_idx);

	/* Neighbor candidate */
	prev_buf = q->bufs[prev_idx];	
	prev_lbuf = container_of(prev_buf, struct lf2000_buffer, vb);

	/* This buffer won't fit in the existing allocation */
	if(prev_lbuf->offset + 2 * fmt->width > fmt->bytesperline / 2)
		goto out;

	prev_mem = prev_buf->priv;

	/* Mirror dma_contig mapper functionality */
	lbuf->offset = prev_lbuf->offset + fmt->width;
printk(KERN_EMERG "%s: offset: %d\n", __FUNCTION__, lbuf->offset);
	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if(!map)
		return -ENOMEM;

	buf->map = map;
	map->q = q;

	buf->baddr = vma->vm_start;

	mem = buf->priv;
	BUG_ON(!mem);

//	MAGIC_CHECK(mem->magic, MAGIC_DC_MEM);

	mem->size = PAGE_ALIGN(buf->bsize);

	/* Memory has already been allocated w/dma_alloc_coherent() */
	mem->vaddr = prev_mem->vaddr + fmt->width;
	mem->dma_handle = prev_mem->dma_handle + fmt->width;

#if 1
	/* Memory has already been mapped, but map it again to this vma */
	size = vma->vm_end - vma->vm_start - lbuf->offset;
	size = (size < mem->size) ? size : mem->size;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	ret = remap_pfn_range(vma, vma->vm_start,
				mem->dma_handle >> PAGE_SHIFT,
				size, vma->vm_page_prot);

	vma->vm_flags		|= VM_DONTEXPAND;
	vma->vm_private_data	= map;

	return ret;
#else
	/* TODO: re-use existing PTEs */
	/*
	 * Memory has already been mapped, so the PTEs already exist.
	 * The VM subsystem already prepared the new mapping
	 * (our call stack is mmap_pgoff()->do_mmap_pg_off()->mmap_region()),
	 * so we have to clean it up in order to reuse the old mapping.
	 */
	ret = do_munmap(vma->mm, vma->vm_start, (vma->vm_end - vma->vm_start_;
printk(KERN_EMERG "%s: do_unmap: %d\n", __FUNCTION__, ret);

	return ret;
#endif

out:
#endif
	return pcdev->mmap_mapper(q, buf, vma);
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
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	enum v4l2_field field;

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

	if(pcdev->interlaced)
		field = V4L2_FIELD_INTERLACED;
	else
		field = V4L2_FIELD_NONE;

	videobuf_queue_dma_contig_init(q, &lf2000_videobuf_ops,
					icd->dev.parent, &pcdev->lock,
					V4L2_BUF_TYPE_VIDEO_CAPTURE,
					field,
					sizeof(struct lf2000_buffer), icd,
					NULL);

	if(q->int_ops->mmap_mapper != lf2000_vip_mmap_mapper)
	{
		pcdev->mmap_mapper	= q->int_ops->mmap_mapper;
		q->int_ops->mmap_mapper	= lf2000_vip_mmap_mapper;
	}

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
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct lf2000_vip_dev *pcdev = ici->priv;
	struct v4l2_pix_format	*fmt = &pcdev->clipper_format;
	int i;

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

	pcdev->buf_type = p->memory;

	for(i = 0; i < p->count; i++)
	{
		struct videobuf_buffer	*vb = icd->vb_vidq.bufs[i];
		struct lf2000_buffer	*buf = container_of(vb,
						struct lf2000_buffer, vb);
		buf->offset = 0;
printk(KERN_EMERG "%s: %d, %d\n", __FUNCTION__, vb->bytesperline, fmt->bytesperline);
		vb->bytesperline	= fmt->bytesperline;
printk(KERN_EMERG "%s: %d, %d\n", __FUNCTION__, vb->width, fmt->width);
		vb->width		= fmt->width;
printk(KERN_EMERG "%s: %d, %d\n", __FUNCTION__, vb->height, fmt->height);
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

	dev_dbg(icd->dev.parent, "%s\n", __FUNCTION__);

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
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}


static const struct v4l2_queryctrl lf2000_camera_controls[] = {
	/* TODO: V4L2_CID_HFLIP V4L2_CID_VFLIP V4L2_CID_ROTATE */
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
	.controls	= lf2000_camera_controls,
	.num_controls	= ARRAY_SIZE(lf2000_camera_controls),
};

static irqreturn_t vip_isr(int irq, void *data)
{
	struct lf2000_vip_dev *pcdev = data;
	struct lf2000_buffer *buf;
	struct videobuf_buffer *vb;
	struct v4l2_pix_format *pix;
	dma_addr_t addr;
	U16 enb, ctrl;
	unsigned long flags;

	enb = readw(pcdev->base + VIP_CDENB);
	writew(0, pcdev->base + VIP_CDENB);

//	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

#ifdef CONFIG_VIDEO_LF2000_SYSFS
	write_lock_irqsave(&pcdev->sysfs_lock, flags);
	pcdev->frame_count++;
	write_unlock_irqrestore(&pcdev->sysfs_lock, flags);
#endif

//	if(pcdev->bad_frame++ < 2)
//	{
//		goto out;
//	}

//	pcdev->bad_frame = 0;
	/* Handle capture */
	if(pcdev->active)
	{
		spin_lock_irqsave(&pcdev->lock, flags);

		vb = &pcdev->active->vb;
		buf = container_of(vb, struct lf2000_buffer, vb);
		WARN_ON(list_empty(&vb->queue));

		list_del_init(&vb->queue);
		vb->state = VIDEOBUF_DONE;
		do_gettimeofday(&vb->ts);
		vb->field_count++;

		wake_up(&vb->done);

		if(list_empty(&pcdev->capture))
		{
			/* Queue depleted */
			pcdev->active = NULL;
		}
		else
		{
			/* Grab next buffer and point HW to it */
			pix = &pcdev->clipper_format;
			pcdev->active = list_entry(pcdev->capture.next,
						struct lf2000_buffer, vb.queue);

			vb = &pcdev->active->vb;
			vb->state = VIDEOBUF_ACTIVE;

			setup_clipper(pcdev, pcdev->active->addr, pix, LF2000_VIP_PIPE_CAPTURE);
		}

		spin_unlock_irqrestore(&pcdev->lock, flags);
	}

out:
	ctrl = readw(pcdev->base + VIP_INTCTRL);
	ctrl |= (0x1 << VSINTPEND);
	writew(ctrl, pcdev->base + VIP_INTCTRL);

	writew(enb, pcdev->base + VIP_CDENB);

	return IRQ_HANDLED;
}

static void vip_setup_pins(struct lf2000_vip_dev *pcdev)
{
	int i, len;
	u_int pad, fn;
	u_int (*pads)[2];
	u16 tmp;

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

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

	tmp = readw(pcdev->base + VIP_VIP1);

	if(pcdev->pdata->flags & LF2000_VIP_PORT)
	{
		pads = port_1;
		len = ARRAY_SIZE(port_1);
		tmp |= (1 << VIPPORTSEL);
	}
	else
	{
		pads = port_0;
		len = ARRAY_SIZE(port_0);
		tmp &= ~(1 << VIPPORTSEL);
	}
	writew(tmp, pcdev->base + VIP_VIP1);

	for (i = 0; i < len; i++)
	{
		pad = pads[i][0];	
		fn = pads[i][1];
		soc_gpio_set_io_func(pad, fn);
	}
}

static void vip_setup_clocks(struct lf2000_vip_dev *pcdev)
{
	U32 tmp;

	dev_dbg(&pcdev->pdev->dev, "%s\n", __FUNCTION__);

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
}

static int __init lf2000_vip_probe(struct platform_device *pdev)
{
	struct lf2000_vip_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	unsigned int irq;
	int err = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if(!res || (int)irq < 0)
	{
		err = -ENODEV;
		goto exit;
	}

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

	INIT_LIST_HEAD(&pcdev->capture);
	spin_lock_init(&pcdev->lock);

	/*
	 * Request the region.
	 */
	if(!request_mem_region(res->start, resource_size(res), res->name))
	{
		err = -EBUSY;
		goto exit_kfree;
	}

	base = ioremap_nocache(res->start, resource_size(res));
	if(!base)
	{
		err = -ENOMEM;
		goto exit_release;
	}
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

	pcdev->clipper_status		= LF2000_VIP_PIPE_IDLE;

	err = soc_camera_host_register(&pcdev->soc_host);
	if(err)
		goto exit_free_irq;

#ifdef CONFIG_VIDEO_LF2000_DEBUGFS
	vip_init_debugfs(pcdev);
#endif

#ifdef CONFIG_VIDEO_LF2000_SYSFS
	rwlock_init(&pcdev->sysfs_lock);
	sysfs_create_group(&pdev->dev.kobj, &vip_attr_group);
#endif

	dev_info(&pdev->dev, "LF2000 Video Input Port driver loaded\n");

	return 0;

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
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);

	struct lf2000_vip_dev *pcdev = container_of(soc_host,
					struct lf2000_vip_dev, soc_host);
	struct resource *res;

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


MODULE_DESCRIPTION("Alternate LF2000 Video Input Port (VIP) driver");
MODULE_LICENSE("GPL v2");
