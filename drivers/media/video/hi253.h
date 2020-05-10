#ifndef __HI253_H__
#define __HI253_H__

#include <linux/v4l2-mediabus.h>

static inline void hi253_clamp_format_size(struct v4l2_mbus_framefmt *_fmt)
{
	// Clamp format size to one of native sensor sizes
	if (_fmt->width <= 400 && _fmt->height <= 300) {
		_fmt->width = 400;
		_fmt->height = 300;
	}
	else if (_fmt->width <= 800 && _fmt->height <= 600) {
		_fmt->width = 800;
		_fmt->height = 600;
	}
	else {
		_fmt->width = 1600;
		_fmt->height = 1200;
	}
}

#endif // __HI253_H__
