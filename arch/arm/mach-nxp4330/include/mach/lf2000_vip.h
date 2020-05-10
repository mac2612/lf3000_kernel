/*
 * LF2000 VIP driver
 *
 * Copyright (C) 2011 LeapFrog Enterprises, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_VIP_H__
#define __MACH_VIP_H__

#define	VIP_DEV_NAME			"vip"

/* flags */
//#define LF2000_VIP_PORT0	(0 << 1) // 	/* VCLK, VID[0 ~ 7] */
#define LF2000_VIP_PORT		(1 << 0) // 1	/* VCLK_B, VID_B[0 ~ 7], SYNC */
#define LF2000_VIP_ROTATE	(1 << 2) // 4
#define LF2000_VIP_HFLIP	(1 << 3) // 8 
#define LF2000_VIP_VFLIP	(1 << 4) // 16 
#define LF2000_VIP_ITU656	(1 << 5) // 32

struct lf2000_vip_platform_data {
	unsigned short	hfp;	/* Horizontal front porch */
	unsigned short	hsw;	/* Horizontal sync width */
	unsigned short	hbp;	/* Horizontal back porch */
	unsigned short	vfp;	/* Vertical ... */
	unsigned short	vsw;
	unsigned short	vbp;

	unsigned long	flags;
};

extern void lf2000_set_vip_info(const struct lf2000_vip_platform_data *);

#endif /* __MACH_VIP_H__ */
