/*
 * Hynix YAC-family Cameras
 *
 * Copyright (C) 2011 LeapFrog Enterprises, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __HYNIX_YAC_H__
#define __HYNIX_YAC_H__

#include <media/soc_camera.h>

/* for flags */
#define HYNIX_YAC_ITU656	(1 << 0)	/* ITU-R BT.656-5 -like mode */

/*
 * YAC* camera info
 */
struct hynix_yac_camera_info {
	/* Used for printk/debugfs */
	const char 			*name;

	/* MCLK frequency in Hz */
	const unsigned long		mclk_hz;

	/* Maximum host PCLK frequency in Hz */
	const unsigned long		host_max_pclk_hz;

	/* VSYNC duration in lines */
	const unsigned short		vsync;

	const unsigned long		flags;

	/* ITU656 markers */
	const unsigned char		sof;
	const unsigned char		sol;
	const unsigned char		eof;
	const unsigned char		eol;

	/* For tracking power state */
	bool				reset;
	bool				attached;

	/* Power sequencing pins */
	int				RESETB;
	int				CHIP_ENABLEB;
	const char 			*resetb_name;
	const char 			*enableb_name;
	unsigned int			enable_delay_ms;
    int             CLOCK_ENABLEB;
    const char		*clock_enableb_name;     
};

/*
 * v4l2_captureparm.extendedmode
 */
#define HYNIX_YAC_VAR_FRAME_RATE		0	/* bit 0 */
#define HYNIX_YAC_VAR_FRAME_RATE_MASK		1	/* width 1 */

#define PARM_EXTMODE_TO_FRAME_RATE(x)	\
	(((x) >> HYNIX_YAC_VAR_FRAME_RATE ) &	\
	HYNIX_YAC_VAR_FRAME_RATE_MASK)

#define FRAME_RATE_TO_PARM_EXTMODE(x)	\
	(((x) & HYNIX_YAC_VAR_FRAME_RATE_MASK)	\
	<< HYNIX_YAC_VAR_FRAME_RATE)

enum {
	HYNIX_YAC_FIXED_FRAME_RATE	= 0,
	HYNIX_YAC_VARIABLE_FRAME_RATE	= 1
};

#endif /* __HYNIX_YAC_H__ */
