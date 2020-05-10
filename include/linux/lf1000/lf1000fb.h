/* 
 * Additional commands for the LF1000 frame buffer interface.
 *
 * Copyright (c) 2010 LeapFrog Enterprises Inc.
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

#ifndef __ASM_LF1000_FB_H__
#define __ASM_LF1000_FB_H__

#include <linux/types.h>
#include <linux/ioctl.h>

/* We use the nonstd field to pack in some layer information.  The contents
 * need to be defined such that nothing unusual happens if nonstd is zero. */

#define LF1000_NONSTD_PLANAR		23
#define LF1000_NONSTD_PLANAR_MASK	1
#define LF1000_NONSTD_PRIORITY		24
#define LF1000_NONSTD_PRIORITY_MASK	0x3
#define LF1000_NONSTD_FORMAT		20
#define LF1000_NONSTD_FORMAT_MASK	0x7

#define NONSTD_TO_PLANAR(x)	(((x) >> LF1000_NONSTD_PLANAR) & \
				LF1000_NONSTD_PLANAR_MASK)

#define NONSTD_TO_PFOR(x)       (((x) >> LF1000_NONSTD_FORMAT) & \
				LF1000_NONSTD_FORMAT_MASK)
#define NONSTD_TO_POS(x)        (((x) >> LF1000_NONSTD_PRIORITY) & \
				LF1000_NONSTD_PRIORITY_MASK)

enum {
	LAYER_FORMAT_RGB        = 0,
	LAYER_FORMAT_YUV420     = 1,
	LAYER_FORMAT_YUV422     = 2,
};

/* lf1000fb_blend_cmd: get/set full-layer blending. */
struct lf1000fb_blend_cmd {
	unsigned char	enable;
	unsigned char	alpha;
};

/* lf1000fb_position_cmd: get/set the location of the upper-left corner of the
 * layer. */
struct lf1000fb_position_cmd {
	int		left;
	int		top;
	unsigned	apply : 1;	/* on set: apply right away */
	int		right;
	int		bottom;
};

/* lf1000fb_vidscale_cmd: get/set the origin size for the video scaler.  Only
 * valid for the YUV layer. */
struct lf1000fb_vidscale_cmd {
	unsigned int	sizex;
	unsigned int	sizey;
	unsigned	apply : 1;	/* on set: apply right away */
};

union lf1000fb_cmd {
	struct lf1000fb_blend_cmd	blend;
	struct lf1000fb_position_cmd	position;
	struct lf1000fb_vidscale_cmd	vidscale;
};

#define LF1000FB_IOCSALPHA	_IOW('m', 1, struct lf1000fb_alpha_cmd  *)
#define LF1000FB_IOCGALPHA	_IOR('m', 2, struct lf1000fb_alpha_cmd  *)
#define LF1000FB_IOCSPOSTION	_IOW('m', 3, struct lf1000fb_position_cmd *)
#define LF1000FB_IOCGPOSTION	_IOR('m', 4, struct lf1000fb_position_cmd *)
#define LF1000FB_IOCSVIDSCALE	_IOW('m', 5, struct lf1000fb_vidscale_cmd *)
#define LF1000FB_IOCGVIDSCALE	_IOR('m', 6, struct lf1000fb_vidscale_cmd *)

#ifndef FBIO_WAITFORVSYNC
#define FBIO_WAITFORVSYNC	 _IOW('F', 0x20, __u32)
#endif

#endif /* __ASM_LF1000_FB_H__ */
