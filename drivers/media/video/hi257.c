/* linux/drivers/media/video/hi257.c
 *
 * Copyright 2013 LeapFrog Enterprises Firmware Engineering
 * Based on Samsung driver for V4L2 Media API (Nexell compatible)
 *
 * Driver for Hynix HI257 (UXGA camera)
 * 2.0Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG 1

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/v4l2-subdev.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <media/v4l2-ctrls.h>

#if defined(CONFIG_PLAT_NXP4330_CABO)
#define PLLX(x)		(x | 0x03)
#else
#define PLLX(x)		(x | 0x03)
#endif

#define FLIP(x)		(x | 0x03)
#include "hi257_regs.h"
#include "hi257_default.h"
#include "lf2000/hi257_capture.h"
#include "hi253.h"


#define ADDR_HI257 	0x20

#define HI257_DRIVER_NAME	"HI257"

#define NUM_CTRLS           11

#define PREVIEW_MODE        0
#define CAPTURE_MODE        1

#define MAX_WIDTH          1600 // 1604
#define MAX_HEIGHT         1200 // 1204

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

#define USE_READBACK		1
#define MAX_RETRY   		5 //10

static const unsigned char REG_MAP[NO_SUCH_PAGE] = {
	[WINDOW_PAGE]		= 0x00,
	[FORMAT_PAGE]		= 0x10,
	[NOISE1_PAGE]		= 0x11,
	[NOISE2_PAGE]		= 0x12,
	[EDGE_PAGE]			= 0x13,
	[LENS_PAGE]			= 0x14,
	[COLOR_PAGE]		= 0x15,
	[GAMMA_PAGE]		= 0x16,
	[FLICKER_PAGE]		= 0x17,
	[SCALING_PAGE]		= 0x18,
	[MULTICOLOR_PAGE]	= 0x19,
	[AUTOEXP_PAGE]		= 0x20,
	[AEWGTCF_PAGE]		= 0x21,
	[AUTOWB_PAGE]		= 0x22,
	[UNDOC_0x02_PAGE]	= 0x02,
};


static int read_device_id_for_hi257(struct i2c_client *client);

/*
 * Specification
 * Parallel : ITU-656 YUV422
 * Resolution : 1600 (H) x 1200 (V)
 * FPS : 15fps @UXGA, 30fps @SVGA, QVGA
 */

struct hi257_state {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct v4l2_ctrl_handler handler;
    /* standard */
    struct v4l2_ctrl *focus;
    struct v4l2_ctrl *wb;
    struct v4l2_ctrl *color_effect;
    struct v4l2_ctrl *exposure;
    /* custom */
    struct v4l2_ctrl *scene_mode;
    struct v4l2_ctrl *anti_shake;
    struct v4l2_ctrl *mode_change;

    bool inited;
    int width;
    int height;
    int mode; // PREVIEW or CAPTURE

    /* for zoom */
    struct v4l2_rect crop;

    int readback_errors;
    int retry_errors;
};

static inline struct hi257_state *to_state(struct v4l2_subdev *sd)
{
    return container_of(sd, struct hi257_state, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
    return &container_of(ctrl->handler, struct hi257_state, handler)->sd;
}

static inline struct hi257_state *client_to_priv(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct hi257_state, sd);
}

static unsigned char i2c_read_reg(struct i2c_client *client, unsigned char reg_h, unsigned char reg)
{
    int ret;
    unsigned char i2c_data[10];
    (void)reg_h;

    i2c_data[0] = reg;
    ret = i2c_master_send(client, i2c_data, 1);
    if (ret != 1)
    	return ret;

    ret = i2c_master_recv(client, &i2c_data[1], 1);
    if (ret != 1)
    	return ret;
    return i2c_data[1];
}

static inline int hi257_i2c_write(struct i2c_client *client, unsigned char buf[], int length)
{
    struct i2c_msg msg = {client->addr, 0, length, buf};
    return i2c_transfer(client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int hi257_write_regs(struct i2c_client *client, unsigned char (*regvals)[3], int size)
{
    unsigned char tmp[4];
    int i, err = 0;
    static unsigned char page = NO_SUCH_PAGE;
    int retry = 0;
    struct hi257_state* state = client_to_priv(client);

    for (i = 0; i < size ; i++) {
           if (page != REG_MAP[regvals[i][0]]) {
           		page  = REG_MAP[regvals[i][0]];
           		tmp[0] =(unsigned char)( PAGEMODE );
           		tmp[1] =(unsigned char)( page );

				do {
					err = hi257_i2c_write(client, tmp , 2);
					//printk("%s: %02x: %02x\n", __func__, tmp[0], tmp[1]);
#if USE_READBACK
					tmp[2] = i2c_read_reg(client, 0, tmp[0]);
					if (tmp[2] != tmp[1]) {
						pr_err("%s: register set failed for %02x: %02x: %02x, read back %02x\n", __func__, page, tmp[0], tmp[1], tmp[2]);
						err = -EAGAIN; 
					}
#endif
				} while (err < 0 && ++retry < MAX_RETRY);

				if (retry) {
					state->readback_errors++;
					state->retry_errors += retry;
					retry = 0;
				}
			}

            tmp[0] =(unsigned char)( regvals[i][1] );
            tmp[1] =(unsigned char)( regvals[i][2] );

            do {
				err = hi257_i2c_write(client, tmp , 2);
				//printk("%s: %02x: %02x\n", __func__, tmp[0], tmp[1]);

				if (err < 0)
					pr_err("%s: register set failed for %02x: %02x: %02x, error = %d\n", __func__, page, tmp[0], tmp[1], err);
#if USE_READBACK
				tmp[2] = i2c_read_reg(client, 0, tmp[0]);
				if (tmp[2] != tmp[1]) {
					pr_err("%s: register set failed for %02x: %02x: %02x, read back %02x\n", __func__, page, tmp[0], tmp[1], tmp[2]);
					//err = -EAGAIN;
				}
#endif
            } while (err < 0 && ++retry < MAX_RETRY);

            if (retry) {
				state->readback_errors++;
				state->retry_errors += retry;
				retry = 0;
			}
    }

    return err;
}

static int hi257_write(struct i2c_client *client, unsigned char page, unsigned char reg, unsigned char val)
{
    unsigned char tmp[3];

    tmp[0] = page;
    tmp[1] = reg;
    tmp[2] = val;

    return hi257_write_regs(client, &tmp, 1); // DOH!
}

static int hi257_modify(struct i2c_client *client, unsigned char page, unsigned char reg, unsigned char set, unsigned char clear)
{
	unsigned char tmp;

	// FIXME: track page selection across reads & writes
	tmp = i2c_read_reg(client, 0, PAGEMODE);

	if (tmp != page)
		hi257_write(client, tmp, PAGEMODE, page);

	tmp = i2c_read_reg(client, 0, reg);

	tmp &= ~clear;
	tmp |= set;

	return hi257_write(client, page, reg, tmp);
}

static struct v4l2_rect *
_get_pad_crop(struct hi257_state *me, struct v4l2_subdev_fh *fh,
        unsigned int pad, enum v4l2_subdev_format_whence which)
{
    switch (which) {
    case V4L2_SUBDEV_FORMAT_TRY:
        return v4l2_subdev_get_try_crop(fh, pad);
    case V4L2_SUBDEV_FORMAT_ACTIVE:
        return &me->crop;
    default:
        pr_err("%s: invalid which %d\n", __func__, which);
        return &me->crop;
    }
}

static int hi257_set_frame_size(struct v4l2_subdev *sd, int mode, int width, int height)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    printk("%s\n", __func__);

    if (width > 800 || height > 600)
        return hi257_write_regs(client, (unsigned char(*)[3])HI257_Capture_UXGA, ARRAY_SIZE(HI257_Capture_UXGA));
    else if (width > 400 || height > 300)
        return hi257_write_regs(client, (unsigned char(*)[3])HI257_Capture_SVGA, ARRAY_SIZE(HI257_Capture_SVGA));
    return hi257_write_regs(client, (unsigned char(*)[3])HI257_Capture_QSVGA, ARRAY_SIZE(HI257_Capture_QSVGA));
}

static int hi257_s_crop(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_crop *crop)
{
    struct hi257_state *state = to_state(sd);
    struct v4l2_rect *__crop = _get_pad_crop(state, fh, crop->pad, crop->which);

    if ((crop->rect.left + crop->rect.width) > MAX_WIDTH ||
        (crop->rect.top + crop->rect.height) > MAX_HEIGHT) {
        pr_err("%s: invalid crop rect(left %d, width %d, max width %d, top %d, height %d, max height %d\n)",
                __func__, crop->rect.left, crop->rect.width, MAX_WIDTH,
                crop->rect.top, crop->rect.height, MAX_HEIGHT);
        return -EINVAL;
    }

    // TODO

    *__crop = crop->rect;
    return 0;
}

static int hi257_g_crop(struct v4l2_subdev *sd,
        struct v4l2_subdev_fh *fh, struct v4l2_subdev_crop *crop)
{
    struct hi257_state *state = to_state(sd);
    struct v4l2_rect *__crop = _get_pad_crop(state, fh, crop->pad, crop->which);
    crop->rect = *__crop;
    return 0;
}

static int _hi257_get_default(const char key, char* value)
{
   int i;
   size_t defaultsSize = ARRAY_SIZE(HI257_Default);
   
   //err = hi257_write_regs(client, HI257_Default, 
   for(i = 0; i < defaultsSize; ++i) {
      if(HI257_Default[i][1] == key) {
	 *value = HI257_Default[i][2];
	 return 0;
      }
   }
   
   return -1;
}

static int hi257_s_fmt(struct v4l2_subdev *sd,
        struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *fmt)
{
    int err = 0;
    struct v4l2_mbus_framefmt *_fmt = &fmt->format;
    struct hi257_state *state = to_state(sd);
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    char default_vdoctl2;
    
    printk("==========> %s\n", __func__);
    if (!state->inited) {
        pr_err("%s: device is not initialized!!!\n", __func__);
        return -EINVAL;
    }

    if(_hi257_get_default(VDOCTL2, &default_vdoctl2) < 0) {
       pr_err("%s: no default VDOCTL2 value\n", __func__);
       return -EINVAL;
    }
    
    printk("%s: vidoctl2 default = 0x%02x\n", __func__, default_vdoctl2);
    
	// Clamp format size to one of native sensor sizes
    hi253_clamp_format_size(_fmt);

    pr_debug("%s: %dx%d\n", __func__, _fmt->width, _fmt->height);
    state->width = _fmt->width;
    state->height = _fmt->height;
    printk("%s: mode %d, %dx%d\n", __func__, state->mode, state->width, state->height);
    err = hi257_set_frame_size(sd, state->mode, state->width, state->height);

    printk("%s: %s\n", __func__, sd->name);
    
    if (strstr(sd->name, "0-0020"))
    	hi257_modify(client, WINDOW_PAGE, VDOCTL2, default_vdoctl2, 0x00);
    else
    	hi257_modify(client, WINDOW_PAGE, VDOCTL2, 0x00, default_vdoctl2 & 0x02);
    return err;
}

static int hi257_set_af(struct v4l2_subdev *sd, int value)
{
#if 0
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int err = 0;

    // TODO
#endif
    return 0;
}

enum {
    WB_AUTO = 0,
    WB_DAYLIGHT,
    WB_CLOUDY,
    WB_FLUORESCENT,
    WB_INCANDESCENT,
    WB_MAX
};

static int hi257_set_wb(struct v4l2_subdev *sd, int value)
{
#if 0
    int err;
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct reg_val *reg_val;
    int size;

    // TODO
#endif
    return 0;
}

enum {
    COLORFX_NONE = 0,
    COLORFX_SEPIA,
    COLORFX_AQUA,
    COLORFX_MONO,
    COLORFX_NEGATIVE,
    COLORFX_SKETCH,
    COLORFX_MAX
};

static int hi257_set_colorfx(struct v4l2_subdev *sd, int value)
{
#if 0
	int err;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    // TODO
#endif
    return 0;
}

#define MIN_EXPOSURE    -4
#define MAX_EXPOSURE     4
static int hi257_set_exposure(struct v4l2_subdev *sd, int value)
{
#if 0
	int err;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    // TODO
#endif
    return 0;
}

enum {
    SCENE_OFF = 0,
    SCENE_PORTRAIT,
    SCENE_LANDSCAPE,
    SCENE_SPORTS,
    SCENE_NIGHTSHOT,
    SCENE_MAX
};

static int hi257_set_scene(struct v4l2_subdev *sd, int value)
{
#if 0
    int err;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    // TODO
#endif
    return 0;
}

enum {
    ANTI_SHAKE_OFF = 0,
    ANTI_SHAKE_50Hz,
    ANTI_SHAKE_60Hz,
    ANTI_SHAKE_MAX
};

static int hi257_set_antishake(struct v4l2_subdev *sd, int value)
{
#if 0
    int err;
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    // TODO
#endif
    return 0;
}

static int hi257_mode_change(struct v4l2_subdev *sd, int value)
{
    struct hi257_state *state = to_state(sd);
    int err = 0;

    printk("%s: mode %d\n", __func__, value);

    if (unlikely(value < PREVIEW_MODE || value > CAPTURE_MODE)) {
        pr_err("%s: invalid value(%d)\n", __func__, value);
        return -EINVAL;
    }

    if (state->mode != value) {
        state->mode = value;
        err = hi257_set_frame_size(sd, value, state->width, state->height);
    }

    return err;
}

#define V4L2_CID_CAMERA_SCENE_MODE		(V4L2_CTRL_CLASS_CAMERA | 0x1001)
#define V4L2_CID_CAMERA_ANTI_SHAKE		(V4L2_CTRL_CLASS_CAMERA | 0x1002)
#define V4L2_CID_CAMERA_MODE_CHANGE		(V4L2_CTRL_CLASS_CAMERA | 0x1003)

#define ADJ_SATR(x)	((x + 0x10 < 0xFF) ? x + 0x10 : 0xFF)

static int hi257_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int value = ctrl->val;
    int err = 0;

    printk("%s: id(0x%x), value(%d)\n", __func__, ctrl->id, value);

    switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			hi257_modify(client, FORMAT_PAGE, ISPCTL3, 0x10, 0x00);
			if (ctrl->val >= 0x80)
				hi257_write(client, FORMAT_PAGE, YOFS, ctrl->val - 0x80);
			else
				hi257_write(client, FORMAT_PAGE, YOFS, 0xFF - ctrl->val);
			break;

		case V4L2_CID_CONTRAST:
			hi257_modify(client, FORMAT_PAGE, ISPCTL4, 0x02, 0x00);
			hi257_write(client, FORMAT_PAGE, CONTRAST, ctrl->val);
			break;

		case V4L2_CID_SATURATION:
			hi257_modify(client, FORMAT_PAGE, SATCTL, 0x01, 0x00);
			hi257_write(client, FORMAT_PAGE, SATB, ctrl->val);
			hi257_write(client, FORMAT_PAGE, SATR, ADJ_SATR(ctrl->val));
			break;

		case V4L2_CID_AUTO_WHITE_BALANCE:
			if (ctrl->val)
				hi257_modify(client, AUTOWB_PAGE, AWBCTL1, 0x80, 0x00);
			else
				hi257_modify(client, AUTOWB_PAGE, AWBCTL1, 0x00, 0x80);
			break;

        case V4L2_CID_FOCUS_AUTO:
            err = hi257_set_af(sd, value);
            break;

        case V4L2_CID_DO_WHITE_BALANCE:
            err = hi257_set_wb(sd, value);
            break;

        case V4L2_CID_COLORFX:
            err = hi257_set_colorfx(sd, value);
            break;

        case V4L2_CID_EXPOSURE:
            err = hi257_set_exposure(sd, value);
            break;

        /* custom */
        case V4L2_CID_CAMERA_SCENE_MODE:
            err = hi257_set_scene(sd, value);
            break;

        case V4L2_CID_CAMERA_ANTI_SHAKE:
            err = hi257_set_antishake(sd, value);
            break;

        case V4L2_CID_CAMERA_MODE_CHANGE:
            err = hi257_mode_change(sd, value);
            break;

        default:
            pr_err("%s: no such control\n", __func__);
            break;
    }

    return err;
}

static const struct v4l2_ctrl_ops hi257_ctrl_ops = {
    .s_ctrl = hi257_s_ctrl,
};

static const struct v4l2_ctrl_config hi257_custom_ctrls[] = {
    {
        .ops    = &hi257_ctrl_ops,
        .id     = V4L2_CID_CAMERA_SCENE_MODE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "SceneMode",
        .min    = 0,
        .max    = SCENE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &hi257_ctrl_ops,
        .id     = V4L2_CID_CAMERA_ANTI_SHAKE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "AntiShake",
        .min    = 0,
        .max    = ANTI_SHAKE_MAX - 1,
        .def    = 0,
        .step   = 1,
    }, {
        .ops    = &hi257_ctrl_ops,
        .id     = V4L2_CID_CAMERA_MODE_CHANGE,
        .type   = V4L2_CTRL_TYPE_INTEGER,
        .name   = "ModeChange",
        .min    = 0,
        .max    = 1,
        .def    = 0,
        .step   = 1,
    },
};

static int hi257_initialize_ctrls(struct hi257_state *state)
{
    v4l2_ctrl_handler_init(&state->handler, NUM_CTRLS);

    /* standard */
    state->focus = v4l2_ctrl_new_std(&state->handler, &hi257_ctrl_ops,
            V4L2_CID_FOCUS_AUTO, 0, 1, 1, 0);
    if (!state->focus) {
        pr_err("%s: failed to create focus ctrl\n", __func__);
        return -1;
    }
    state->wb = v4l2_ctrl_new_std(&state->handler, &hi257_ctrl_ops,
            V4L2_CID_DO_WHITE_BALANCE, WB_AUTO, WB_MAX - 1, 1, WB_AUTO);
    if (!state->wb) {
        pr_err("%s: failed to create wb ctrl\n", __func__);
        return -1;
    }
    state->color_effect = v4l2_ctrl_new_std_menu(&state->handler, &hi257_ctrl_ops,
            V4L2_CID_COLORFX, COLORFX_MAX - 1, 0, COLORFX_NONE);
    if (!state->color_effect) {
        pr_err("%s: failed to create color_effect ctrl\n", __func__);
        return -1;
    }
    state->exposure = v4l2_ctrl_new_std_menu(&state->handler, &hi257_ctrl_ops,
            V4L2_CID_EXPOSURE, MAX_EXPOSURE, 1, 0);
    if (!state->exposure) {
        pr_err("%s: failed to create exposure ctrl\n", __func__);
        return -1;
    }

    /* custom */
    state->scene_mode = v4l2_ctrl_new_custom(&state->handler, &hi257_custom_ctrls[0], NULL);
    if (!state->scene_mode) {
        pr_err("%s: failed to create scene_mode ctrl\n", __func__);
        return -1;
    }
    state->anti_shake = v4l2_ctrl_new_custom(&state->handler, &hi257_custom_ctrls[1], NULL);
    if (!state->anti_shake) {
        pr_err("%s: failed to create anti_shake ctrl\n", __func__);
        return -1;
    }
    state->mode_change = v4l2_ctrl_new_custom(&state->handler, &hi257_custom_ctrls[2], NULL);
    if (!state->mode_change) {
        pr_err("%s: failed to create mode_change ctrl\n", __func__);
        return -1;
    }

    // HI257 controls
/*
    v4l2_ctrl_new_std(&state->handler, &hi257_ctrl_ops, V4L2_CID_BRIGHTNESS, 0x00, 0xFF, 1, 0x7B);
    v4l2_ctrl_new_std(&state->handler, &hi257_ctrl_ops, V4L2_CID_CONTRAST  , 0x00, 0xFF, 1, 0x84);
    v4l2_ctrl_new_std(&state->handler, &hi257_ctrl_ops, V4L2_CID_SATURATION, 0x00, 0xFF, 1, 0x90);
    v4l2_ctrl_new_std(&state->handler, &hi257_ctrl_ops, V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);
*/

    state->sd.ctrl_handler = &state->handler;
    if (state->handler.error) {
        printk("%s: ctrl handler error(%d)\n", __func__, state->handler.error);
        v4l2_ctrl_handler_free(&state->handler);
        return -1;
    }

    return 0;
}


static int hi257_connected_check(struct i2c_client *client)
{
	if (read_device_id_for_hi257(client) == HI257_ID)
        return 0;
    return -1;
}

static int hi257_init(struct v4l2_subdev *sd, u32 val)
{
    int err = 0;
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct hi257_state *state = to_state(sd);

    if (!val) {
        if (!state->inited)
            return 0;
        hi257_modify(client, WINDOW_PAGE, PWRCTL, 0x01, 0x00); // soft sleep
        printk("hi257_init exit: %s, readback errors = %d, retry attempts = %d\n", sd->name, state->readback_errors, state->retry_errors);
        state->readback_errors = 0;
        state->retry_errors = 0;
    	state->inited = false;
    	return 0;
    }

    if (!state->inited) {
        printk("hi257_init: %s\n", sd->name);
        hi257_modify(client, WINDOW_PAGE, PWRCTL, 0x02, 0x00); // soft reset
        hi257_modify(client, WINDOW_PAGE, PWRCTL, 0x00, 0x02);
        if(hi257_connected_check(client) < 0) {
            v4l_info(client, "%s: camera not connected..\n", __func__);
            return -1;
        }
        err = hi257_write_regs(client, (unsigned char(*)[3])HI257_Default, ARRAY_SIZE(HI257_Default));
        if (err < 0) {
            pr_err("%s: write reg error(err: %d)\n", __func__, err);
            return err;
        }
        state->inited = true;
    }

    return 0;
}

static int hi257_s_power(struct v4l2_subdev *sd, int on)
{
    printk("hi257_s_power: %s: %d\n", sd->name, on);
    hi257_init(sd, on);
    return 0;
}

static const struct v4l2_subdev_core_ops hi257_core_ops = {
    .s_power = hi257_s_power,
    .s_ctrl = v4l2_subdev_s_ctrl,
};

static const struct v4l2_subdev_pad_ops hi257_pad_ops = {
    .set_fmt  = hi257_s_fmt,
    .set_crop = hi257_s_crop,
    .get_crop = hi257_g_crop,
};

static const struct v4l2_subdev_ops hi257_ops = {
    .core = &hi257_core_ops,
    .pad = &hi257_pad_ops,
};


/**
 * media_entity_operations
 */
static int _link_setup(struct media_entity *entity,
        const struct media_pad *local,
        const struct media_pad *remote, u32 flags)
{
    printk("%s: entered\n", __func__);
    return 0;
}

static const struct media_entity_operations hi257_media_ops = {
    .link_setup = _link_setup,
};
/*
 * hi257_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int hi257_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct hi257_state *state;
    struct v4l2_subdev *sd;
    int ret;

    state = kzalloc(sizeof(struct hi257_state), GFP_KERNEL);
    if (state == NULL)
        return -ENOMEM;

    sd = &state->sd;
    strcpy(sd->name, HI257_DRIVER_NAME);

    /* Registering subdev */
    v4l2_i2c_subdev_init(sd, client, &hi257_ops);

    sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    state->pad.flags = MEDIA_PAD_FL_SOURCE;
    sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
    sd->entity.ops = &hi257_media_ops;
    if (media_entity_init(&sd->entity, 1, &state->pad, 0)) {
        dev_err(&client->dev, "%s: failed to media_entity_init()\n", __func__);
        kfree(state);
        return -ENOENT;
    }

    ret = hi257_initialize_ctrls(state);
    if (ret < 0) {
        pr_err("%s: failed to initialize controls\n", __func__);
        return ret;
    }

    dev_info(&client->dev, "hi257 has been probed\n");

    return 0;
}


static int hi257_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);

    v4l2_device_unregister_subdev(sd);
    v4l2_ctrl_handler_free(sd->ctrl_handler);
    media_entity_cleanup(&sd->entity);
    kfree(to_state(sd));
    return 0;
}

static const struct i2c_device_id hi257_id[] = {
    { HI257_DRIVER_NAME, 0 },
    { },
};

static int read_device_id_for_hi257(struct i2c_client *client)
{
    int id;
    client->addr = ADDR_HI257;
    id = i2c_read_reg(client, 0, DEVID);
    v4l_info(client,"Check for **** Hynix HI257 **** \n");
    v4l_info(client,"Chip ID 0x%02x :0x%02x \n",  DEVID, id);

    return id;
}


MODULE_DEVICE_TABLE(i2c, hi257_id);

static struct i2c_driver _i2c_driver = {
    .driver = {
        .name = HI257_DRIVER_NAME,
    },
    .probe    = hi257_probe,
    .remove   = hi257_remove,
    .id_table = hi257_id,
};

module_i2c_driver(_i2c_driver);

MODULE_DESCRIPTION("HI257 UXGA camera driver");
MODULE_AUTHOR("Dave Milici <dmilici@leapfrog.com>");
MODULE_LICENSE("GPL");
