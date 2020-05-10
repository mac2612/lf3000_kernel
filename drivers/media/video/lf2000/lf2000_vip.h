/*
 * Video Input Port driver for LF2000
 *
 * Copyright (c) 2011 Leapfrog Enterprises Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LF2000_VIP_H__
#define __LF2000_VIP_H__

#define VIP_MAX_WIDTH	4096
#define VIP_MAX_HEIGHT	2048

/* For CLIP_FORMATSEL & DECI_FORMATSEL */
#define VIP_YUV444	0x2
#define VIP_YUV422	0x1
#define VIP_YUV420	0x0

/* PCLK here is pixel clock, i.e., LF2000 ICLK, not LF2000 PCLK */
/* TODO: SOCAM_DATA_ACTIVE_? */
#define VIP_BUS_FLAGS	(V4L2_MBUS_MASTER | V4L2_MBUS_HSYNC_ACTIVE_HIGH | \
			V4L2_MBUS_VSYNC_ACTIVE_HIGH | SOCAM_DATAWIDTH_8 | \
			V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_PCLK_SAMPLE_FALLING | \
			V4L2_MBUS_DATA_ACTIVE_HIGH)

/* Registers */
#define VIP_CONFIG	0x000
#define VIP_INTCTRL	0x004
#define VIP_SYNCCTRL	0x008
#define	VIP_SYNCMON	0x00C
#define VIP_VBEGIN	0x010
#define VIP_VEND	0x014
#define VIP_HBEGIN	0x018
#define VIP_HEND	0x01C
#define VIP_FIFOCTRL	0x020
#define VIP_HCOUNT	0x024
#define	VIP_VCOUNT	0x028
#define VIP_VCLKOUTSEL	0x02C // FIXME: not LF3000

/* Registers - clipper & decimator */
#define VIP_CDENB	0x200
#define VIP_ODINT	0x204
#define VIP_IMGWIDTH	0x208
#define VIP_IMGHEIGHT	0x20C

#define VIP_CLIP_LEFT	0x210
#define VIP_CLIP_RIGHT	0x214
#define VIP_CLIP_TOP	0x218
#define VIP_CLIP_BOTTOM	0x21C

#define VIP_DECI_TARGETW	0x220
#define VIP_DECI_TARGETH	0x224
#define VIP_DECI_DELTAW		0x228
#define VIP_DECI_DELTAH		0x22C
#define VIP_DECI_CLEARW		0x230
#define VIP_DECI_CLEARH		0x234

#define VIP_DECI_LUSEG		0x238
#define VIP_DECI_CRSEG		0x23C
#define VIP_DECI_CBSEG		0x240
#define VIP_DECI_FORMAT		0x244
#define VIP_DECI_ROTFLIP	0x248
#define VIP_DECI_LULEFT		0x24C
#define VIP_DECI_CRLEFT		0x250
#define VIP_DECI_CBLEFT		0x254
#define VIP_DECI_LURIGHT	0x258
#define VIP_DECI_CRRIGHT	0x25C	// FIXME: datasheet typo
#define VIP_DECI_CBRIGHT	0x260
#define VIP_DECI_LUTOP		0x264
#define VIP_DECI_CRTOP		0x268
#define VIP_DECI_CBTOP		0x26C
#define VIP_DECI_LUBOTTOM	0x270
#define VIP_DECI_CRBOTTOM	0x274
#define VIP_DECI_CBBOTTOM	0x278

#define VIP_CLIP_LUSEG		0x27C
#define VIP_CLIP_CRSEG		0x280
#define VIP_CLIP_CBSEG		0x284
#define VIP_CLIP_FORMAT		0x288
#define VIP_CLIP_ROTFLIP	0x28C
#define VIP_CLIP_LULEFT		0x290
#define VIP_CLIP_CRLEFT		0x294
#define VIP_CLIP_CBLEFT		0x298
#define VIP_CLIP_LURIGHT	0x29C
#define VIP_CLIP_CRRIGHT	0x2A0
#define VIP_CLIP_CBRIGHT	0x2A4
#define VIP_CLIP_LUTOP		0x2A8
#define VIP_CLIP_CRTOP		0x2AC
#define VIP_CLIP_CBTOP		0x2B0
#define VIP_CLIP_LUBOTTOM	0x2B4
#define VIP_CLIP_CRBOTTOM	0x2B8
#define VIP_CLIP_CBBOTTOM	0x2BC

#define VIP_SCANMODE		0x2C0

#define VIP_CLIP_YUYVENB	0x2C4
#define VIP_CLIP_BASEADDRH	0x2C8
#define VIP_CLIP_BASEADDRL	0x2CC
#define VIP_CLIP_STRIDEH	0x2D0
#define VIP_CLIP_STRIDEL	0x2D4

#define VIP_VIP1		0x2D8	 // FIXME: not LF3000

#define	VIP_CLKENB	0x7C0
#define VIP_CLKGENL	0x7C4
#define VIP_CLKGENH	0x7C8


/* VIP Configuration Register (VIP_CONFIG) */
#define DRANGE		9 /* Reserved - Nexell driver DRANGE */
#define EXTSYNCENB	8
#define DORDER		2 /* bits 3:2 */
#define DWIDTH		1 /* TODO: 16 bits? */
#define VIPENB		0

/* VIP Interrupt Control Register (VIP_INTCTRL) */
#define HSINTENB	9
#define VSINTENB	8
#define HSINTPEND	1
#define VSINTPEND	0

/* VIP Synchronization Control Register (VIP_SYNCCTRL) */
#define SYNCCTRL_RES	8 /* bits 9:8 - Nexell driver EXTHSPOL, EXTVSPOL */
#define LASTFIELD	5
#define DVALIDPOL	4
#define EXTFIELDENB	3
#define EXTDVENB	2
#define FIELDSEL	0 /* bits 1:0 */

/* VIP FIFO Control Register (VIP_FIFOCTRL) */
#define FIFOWRENB	11
#define FIFORDENB	10
#define FIFOEMPTY	9
#define FIFOFULL	8
#define RESETFIFOSEL	1 /* bits 2:1 */
#define RESETFIFO	0

/* VIP Clock 0utput Select Register (VIP_VCLKOUTSEL) */
#define VCLKOUTSEL	0

/* VIP Clipper & Decimator Enable Register (VIP_CDENB) */
#define SEPENB		8
#define CLIPENB		1
#define DECIENB		0

/* VIP Operation Done Interrupt Register (VIP_ODINT) */
#define ODINTENB	8
#define ODINTPEND	0

/* VIP Clipper Format Register (VIP_CLIP_FORMAT) */
#define CLIP_FORMATSEL	0 /* bits 1:0 */

/* VIP Clipper Rotation & Flip Register (VIP_CLIP_ROTFLIP) */
#define CLIP_ROTATION	2
#define CLIP_VFLIP	1
#define CLIP_HFLIP	0

/* VIP Scan Mode Register (VIP_SCANMODE) */
#define INTERLACEENB	1
#define FIELDINV	0

/* VIP Clipper Linear YUYV Enable Register (VIP_CLIP_YUYVENB) */
#define YUYVENB		0

/* VIP VIP1 Register (VIP_VIP1) */
#define VIPPORTSEL	0

/* VIP Clock Generation Enable Register (VIP_CLKENB) */
#define PCLKMODE	3
#define CLKGENENB	2
#define BCLKMODE	0 /* bits 1:0 */

/* VIP Clock Generation Control Register LOW (VIP_CLKGENL) */
#define OUTCLKENB	15
#define CLKDIV0		5 /* bits 12:5 */
#define CLKSRCSEL	2 /* bits 4:2 */
#define OUTCLKINV0	1

#endif /* __LF2000_VIP_H__ */
