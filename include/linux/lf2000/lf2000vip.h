/* 
 * Additional commands for the LF2000/Nexell VIP interface.
 *
 * Copyright (c) 2011 LeapFrog Enterprises, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_LF2000_VIP_H__
#define __ASM_LF2000_VIP_H__

/*
 * We use the v4l2_pix_format.priv field to describe output pipe (i.e., clipper
 * or decimator). It's also used to select double buffering of the overlay.
 * We must assume a reasonable default if priv is unset.
 */

#define LF2000_FMT_PRIV_FB		25
#define LF2000_FMT_PRIV_FB_MASK		0x1F

#define LF2000_FMT_PRIV_DOUBLE_BUF	24
#define LF2000_FMT_PRIV_DOUBLE_BUF_MASK	1

#define LF2000_FMT_PRIV_PIPE		23
#define LF2000_FMT_PRIV_PIPE_MASK	1

#define LF2000_FMT_PRIV_ASPECT_RATIO		22
#define LF2000_FMT_PRIV_ASPECT_RATIO_MASK	1

#define LF2000_FMT_PRIV_MATCH_DIM		21
#define LF2000_FMT_PRIV_MATCH_DIM_MASK		1

#define FMT_PRIV_TO_FB(x)	(((x) >> LF2000_FMT_PRIV_FB) & \
				LF2000_FMT_PRIV_FB_MASK)

#define FB_TO_FMT_PRIV(x)	(((x) & LF2000_FMT_PRIV_FB_MASK) \
				 << LF2000_FMT_PRIV_FB)

#define FMT_PRIV_TO_DOUBLE_BUF(x) (((x) >> LF2000_FMT_PRIV_DOUBLE_BUF) & \
				LF2000_FMT_PRIV_DOUBLE_BUF_MASK)

#define DOUBLE_BUF_TO_FMT_PRIV(x) (((x) & LF2000_FMT_PRIV_DOUBLE_BUF_MASK) \
				 << LF2000_FMT_PRIV_DOUBLE_BUF)

#define FMT_PRIV_TO_PIPE(x)	(((x) >> LF2000_FMT_PRIV_PIPE) & \
				LF2000_FMT_PRIV_PIPE_MASK)

#define PIPE_TO_FMT_PRIV(x)	(((x) & LF2000_FMT_PRIV_PIPE_MASK) \
				 << LF2000_FMT_PRIV_PIPE)

#define FMT_PRIV_TO_AR(x)	(((x) >> LF2000_FMT_PRIV_ASPECT_RATIO) & \
				LF2000_FMT_PRIV_ASPECT_RATIO_MASK)

#define AR_TO_FMT_PRIV(x)	(((x) & LF2000_FMT_PRIV_ASPECT_RATIO_MASK) \
				 << LF2000_FMT_PRIV_ASPECT_RATIO)

#define FMT_PRIV_TO_DIM(x)	(((x) >> LF2000_FMT_PRIV_MATCH_DIM) & \
				LF2000_FMT_PRIV_MATCH_DIM_MASK)

#define DIM_TO_FMT_PRIV(x)	(((x) & LF2000_FMT_PRIV_MATCH_DIM_MASK) \
				 << LF2000_FMT_PRIV_MATCH_DIM)

enum {
	VIP_OVERLAY_SINGLE_BUF	= 0,
	VIP_OVERLAY_DOUBLE_BUF	= 1,
};

enum {
	VIP_OUTPUT_PIPE_CLIPPER		= 0,
	VIP_OUTPUT_PIPE_DECIMATOR	= 1,
};

enum {
	VIP_OVERLAY_IGNORE_AR	= 0,
	VIP_OVERLAY_PRESERVE_AR	= 1,
};

enum {
	VIP_OVERLAY_ADJUST_DIM	= 0,
	VIP_OVERLAY_MATCH_DIM	= 1,
};

#endif /* __ASM_LF2000_VIP_H__ */
