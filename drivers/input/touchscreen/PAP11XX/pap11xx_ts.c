/*
 * drivers/input/touchscreen/pap11xx_ts.c
 *
 * PixArt pap11xx TouchScreen driver.
 *
 * Copyright (c) 2013 PixArt Imaging Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Explanation of sysfs debug interface
 *
 * With root privilege, use adb and type
 * cd /sys/devices/platform/nxp-i2c.2/i2c-2/2-0033/m
 * From there you can do several things
 *
 *  Register dump:
 *  echo "r 00 80" > m
 *  cat m
 *  (the first parameter is a starting address (hex) from 0 to 7f
 *   and the 2nd parameter is a length (hex) from 1 to 80
 *
 * Write to a register (one byte):
 *  echo "w 22 95" > m
 *  (the first parameter is the address (hex) from 0 to 7f
 *   the 2nd parameter is the value (hex) from 0 to ff
 * Write to multiple registers:
 *  echo "w a1 d1 a2 d2 a3 d3"   > m
 *  (up to 64 pairs of address, value both in hex)
 *
 * Burst Write up to 128 bytes to register 0x0b
 *  echo "b 0b 55 56  a6.... aa 55" > m
 *
 * Disable navigation
 *  echo "d" > m
 *
 * Enable navigation
 *  echo "e" > m
 *
 * Find driver version string
 *  echo "v" > m
 *  cat m
 *
 * Reload firmware
 *  echo "f" > m
 *
 * Kernel debug message level
 *  echo "k 1" > m
 *
 * Erase PAP1110 customr flash
 *  echo "s" > m
 *
 * Run customer flash config for PAP1110
 *  echo "i" > m
 *
 * Download FW for PAP1110
 *  echo "f" > m
 *
 */

/*
 * Version History
 *	v1.00 - First released - Bert
 *	v1.01 - Fix the issue of points tracking - Bert
 *	v1.02 - Create ENABLE_FW_DOWNLOAD option - Bert
 *	v1.02 - Create ENABLE_FW_FLASH_UPDATE option - copied from 1.06/1.07
 *	v1.2.1 SZ: Looping false touch after FW programming.
 *			Add RESET temperarily before Pixart correct soft/hd reset.
*	v1.03 - Change resolution to 480/272 for 5' panel of LF
 *			at PreEP stage. FW version = 0x31 - Bert
 *  v1.04 - Enhance the hedge checking for motion report, FW version = 0x36
 *  v1.05 - FW version 0x3C - Tune touch panel noise performance.
 *    move board/CTP setup code snippets to platform dependent files.
 *  v1.06 - FW 0x3F release
 *  v1.07 - Only update FW when version > 0x33
 *  v1.08- -Only update FW when version >= 0x3A // revoked.
 *  v1.08 - Use the criteria of 1.07. Only update FW when version > 0x33
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/input/mt.h>
#include <mach/soc.h>

//#include <mach/gpio.h> // ptr to arch/arm/plat-mxc/include/mach/gpio.h which seemed odd
//#include <plat/gpio-cfg.h>	// ptr to arch/arm/plat-samsung/include/plat/gpio-cfg.h

// #include <plat/ctouch.h>	// should ptr to the same folder as above but non-exist
// #include "ctouch.h"
//or
//#include <mach/ctouch.h>
#include <plat/pap11xx_touch.h>	// should ptr to the same folder as above but non-exist
//#include <mach/pap11xx_touch.h>
// #include "pap11xx_touch.h" // this is also used in device.c
#include "pap11xx_ts.h"
#include "pap1110_fw.h"

static int revert_x = 0;
static int revert_y = 0;
static short report_pressure = 40; // 40 = default pressure to report
static char *fw_version = "0x0";

module_param(revert_x, bool, 0644);
MODULE_PARM_DESC(revert_x, "Revert x-axis. (0=disable, 1=enable)");
module_param(revert_y, bool, 0644);
MODULE_PARM_DESC(revert_y, "Revert y-axis. (0=disable, 1=enable)");
module_param(report_pressure, short, 0644);
MODULE_PARM_DESC(report_pressure, "Pressure value to report to input-event system. Range: 0-255, 0=raw, default=40");
module_param(fw_version, charp, 0444);
MODULE_PARM_DESC(fw_version, "Displays firmware version");

/**
 *  Macro definition
 */
/* PIXART_DEV_BOARD_SETUP is only used by PixArt dev board */
#define PIXART_DEV_BOARD_SETUP			0


/**
 *  Enable customer settings
 */
#define ENABLE_FW_DOWNLOAD				1
/* FORCE_FW_DOWNLOAD: Download fw from driver each time at boot-up stage */
#define FORCE_FW_DOWNLOAD				0


#define MODULE_NAME					"PAP11XX: "
#define MODULE_MOD_TIME				"2013/01/10"
#define DRIVER_VERSION				"PixArt PAP11XX - LF v1.08"
#define PAP11XX_I2C_NAME			"pap11xx_ts"
#define PAP11XX_DEV_NAME 			PAP11XX_I2C_NAME

#define pr_func() pr_info(MODULE_NAME "%s\n", __func__)
#define DBG_I2C(x...) \
	if (IS_BIT_SET(drv_debug_mask, DEBUG_MASK_I2C_RW)) pr_info(x)
#define DBG_CMD(x...) \
	if (IS_BIT_SET(drv_debug_mask, DEBUG_MASK_COMMAND)) pr_info(x)
#define DBG_STATUS(x...) \
	if (IS_BIT_SET(drv_debug_mask, DEBUG_MASK_PT_STATUS)) pr_info(x)
#define DBG_RPT(x...) \
	if (IS_BIT_SET(drv_debug_mask, DEBUG_MASK_MOTION_REPORT)) pr_info(x)


/**
 *  Variable
 */
static struct i2c_client *drv_client;
static uint8_t finger_status[PAP11XX_ID_MAX] = {FINGER_NO_TOUCH};
static uint8_t pre_finger_status[PAP11XX_ID_MAX] = {FINGER_NO_TOUCH};
static uint32_t drv_debug_mask, touch_times;
static struct sysfs_data user_cmd;

// float	pressure_scale = 100/5000.0;
static int	pressure_scale = 50; // as a divider
/**
 *  Function Prototype
 */
static int pap11xx_init_panel(struct pap11xx_data *ts);
static int pap11xx_reset(uint8_t);
static int pap11xx_wait_boot(uint8_t);
static void pap1110_flash_config(struct pap11xx_data *ts);

#if !PIXART_DEV_BOARD_SETUP
static void reset_panel(struct pap11xx_i2c_platform_data *pdata);
#endif

/**
 *  Read register from Chip.
 *
 *  @reg - the target register
 *  @data - (ouput) the space to store the output
 */
static int reg_read(uint8_t reg, uint8_t *data)
{
	int ret;

	if (!drv_client->adapter) {
		WARN_ON(!drv_client->adapter);
		return -ENODEV;
	}

	ret = i2c_smbus_read_i2c_block_data(drv_client, reg, 1, (uint8_t *)data);
	if (ret != 1)
		DBG_I2C(MODULE_NAME "%s: warn! ret = %d\n", __func__, ret);

	return ret;
}

/**
 *  Read multiple registers from Chip.
 *
 *  @reg - the target register
 *  @pdata - (ouput) the space to store the output
 *  @len - the length to read
 *
 *  @Note: should not read more than 32 bytes at one time
 */
static int reg_arr_read(uint8_t reg, uint8_t *pdata, uint8_t len)
{
	int ret;

	if (!drv_client->adapter) {
		WARN_ON(!drv_client->adapter);
		return -ENODEV;
	}

	ret = i2c_smbus_read_i2c_block_data(drv_client, reg, len, pdata);
	if (ret != len)
		DBG_I2C(MODULE_NAME "%s: warn! ret = %d\n", __func__, ret);

	return ret;
}

/**
 *  Write register to Chip.
 *
 *  @reg: the target register
 *  @val: the value we'd like to write
 */
static int reg_write(uint8_t reg, uint8_t val)
{
	int ret;

	if (!drv_client->adapter) {
		WARN_ON(!drv_client->adapter);
		return -ENODEV;
	}

	ret = i2c_smbus_write_i2c_block_data(drv_client, reg, 1, &val);
	if (ret < 0)
		DBG_I2C(MODULE_NAME "%s: warn! ret = %d\n", __func__, ret);

	udelay(PAP11XX_DELAY_US_BETWEEN_I2C_WRITES);
	return ret;
}

/**
 *	Set bits specified in "mask" argument.
 *
 *  @reg - the register we'd like to write
 *  @mask - the bits we'd like to set
 */
static int reg_set_mask(uint8_t reg, uint8_t mask)
{
	uint8_t val;

	reg_read(reg, &val);
	BIT_SET(val, mask);
	return reg_write(reg, val);
}

/**
 *	Reset bits specified in "mask" argument.
 *
 *  @reg - the register we'd like to write
 *  @mask - the bits we'd like to clear
 */
static int reg_clear_mask(uint8_t reg, uint8_t mask)
{
	uint8_t val;

	reg_read(reg, &val);
	BIT_CLEAR(val, mask);
	return reg_write(reg, val);
}

/**
 *  Disable flash before programming
 */
static void pap11xx_disable_flash(void)
{
	reg_write(PAP1110_REG_FLASH_ENABLE, 0);
}

/**
 *  Enable flash before programming
 */
static void pap11xx_enable_flash(void)
{
	reg_write(PAP1110_REG_FLASH_ENABLE, PAP1110_ENABLE_FLASH);
	mdelay(1);
}

/**
 *  Read osc_trim regiom information block
 *
 *  @osc_fine: (output) osc trim reg - high byte
 *  @osc_coarse: (output) osc trim reg - low byte
 */
static uint16_t pap1110_osc_reg_read(uint8_t *osc_fine,
		uint8_t *osc_coarse)
{
	uint8_t osc_f, osc_c;

	reg_write(0x7f, 0x07);
	reg_write(PAP1110_REG_WD, PAP1110_WD_DISABLE);
	reg_write(0x7f, 0x48);
	reg_write(0x7f, 0x13);
	reg_write(0x7f, 0x00);
	reg_write(0x30, 0x01);
	reg_write(0x7f, 0x01);
	reg_write(0x01, 0x00);
	reg_write(0x00, 0x00);
	reg_write(0x04, 0x80);
	reg_write(0x05, 0x00);
	reg_write(0x06, 0x00);
	reg_write(0x07, 0x00);
	reg_write(0x00, 0xa1);
	reg_write(0x01, 0x01);
	reg_read(0x03, &osc_f);
	reg_write(0x04, 0x81);
	reg_read(0x03, &osc_c);
	reg_write(0x7f, 0x07);

	if (osc_fine != NULL)
		*osc_fine = osc_f;
	if (osc_coarse != NULL)
		*osc_coarse = osc_c;

	/* must do reset here */
	pap11xx_reset(POWER_HARD_RESET);
	reg_write(PAP1110_REG_WD, PAP1110_WD_ENABLE);

	return (osc_f << 8) | osc_c;
}

/**
 *  Check OSC trim regsiter.
 *          return 0 if osc regs are correct. -EIO otherwise.
 */
static int pap1110_osc_trim_reg_check(void)
{
	uint8_t osc_fine, osc_coarse;
	uint8_t reg_osc_fine, reg_osc_coarse;
	int ret;

	pap1110_osc_reg_read(&osc_fine, &osc_coarse);
	pr_info(MODULE_NAME "osc flash read: %02x%02x\n", osc_coarse, osc_fine);

	reg_write(0x7f, 0x48);
	reg_write(0x7f, 0x13);
	reg_write(0x7f, 0x04);
	reg_read(0x20, &reg_osc_fine);
	reg_read(0x21, &reg_osc_coarse);
	pr_info(MODULE_NAME "osc reg read: %02x%02x\n",
		reg_osc_coarse, reg_osc_fine);

	if (osc_fine == reg_osc_fine && osc_coarse == reg_osc_coarse) {
		ret = 0;
		pr_info(MODULE_NAME "osc checked.\n");
	} else {
		reg_write(0x20, osc_fine);
		reg_write(0x21, osc_coarse);
		pr_info(MODULE_NAME "osc updated - Done (%02x%02x)\n",
			osc_coarse, osc_fine);
		ret = -EIO;
	}

	reg_write(0x7f, 0x07);

	return ret;
}


/**
 *  Erase flash pages for data blocks
 */
static void pap1110_erase_flash_page(void)
{
	uint8_t val;
	int i;

	pr_func();

	pap11xx_reset(POWER_PARTIAL_RESET);
	pap11xx_enable_flash();
	reg_write(PAP1110_REG_WD, PAP1110_WD_DISABLE);

	reg_write(0x7f, 0x48);
	reg_write(0x7f, 0x13);
	reg_write(0x7f, 0x01);

	for (i = 0; i < 4; i++) {
		reg_write(0x04, (uint8_t)(0x5c + i));
		reg_write(0x05, 0x00);
		reg_write(0x06, 0x00);
		reg_write(0x07, 0x00);
		reg_write(0x00, 0x08);
		reg_write(0x01, 0x01);
		val = 0x0;
		while (IS_BIT_CLEAR(val, 0x1)) {
			reg_read(0x2, &val);
			mdelay(1);
		}
		reg_write(0x01, 0x00);
	}
	reg_write(0x00, 0x00);
	reg_write(0x04, 0x00);
	reg_write(0x7f, 0x07);

	pap11xx_reset(POWER_HARD_RESET);
	pap11xx_disable_flash();
	reg_write(PAP1110_REG_WD, PAP1110_WD_ENABLE);
}

/**
 *  flash programming - calibration data block
 */
static int pap1110_update_cal_block(void)
{
	int retry;
	uint8_t val;

	pr_func();

	/* update trim reg - since we just reboot system */
	pap1110_osc_trim_reg_check();

	reg_write(PAP1110_REG_WD, PAP1110_WD_DISABLE);
	pap11xx_enable_flash();

	reg_write(PAP1110_REG_FLASH_CTL, PAP1110_FLASH_SAVE_CAL);
	mdelay(100);
	retry = 25;
	do {
		reg_read(PAP1110_REG_FLASH_CTL, &val);
		if (IS_BIT_CLEAR(val, PAP1110_FLASH_BUSY_BIT));
			break;
		mdelay(20);
	} while (retry-- > 0);
	if (!retry) {
		pr_err(MODULE_NAME "%s flash busy! (%d)\n", __func__, __LINE__);
		return -EBUSY;
	}

	reg_write(PAP1110_REG_WD, PAP1110_WD_ENABLE);
	pap11xx_disable_flash();
	return 0;
}

/**
 *  flash programming - customer register block
 */
#if 0
static int pap1110_update_cr_block(void)
{
	int retry;
	uint8_t val;

	pr_func();

	/* download procedure - prestart */
	reg_write(0x7f, 0x07);
	pap11xx_enable_flash();
	reg_write(PAP1110_REG_WD, PAP1110_WD_DISABLE);

	reg_write(PAP1110_REG_FLASH_CTL, PAP1110_FLASH_START_CUST_REGS);
	mdelay(1);

	/* the registers and values we'd like to save into flash */
	reg_set_mask(PAP1110_REG_ORIENTATION, PAP1110_ORIENT_FLIP_X_AXIS);
	reg_write(PAP1110_REG_FORCE_RUN_MODE, PAP1110_FORCE_RUN);
	reg_write(PAP1110_REG_HEIGHT_HI, WORD_HI_BYTE(PAP11XX_Y_MAX));
	reg_write(PAP1110_REG_HEIGHT_LO, WORD_LO_BYTE(PAP11XX_Y_MAX));
	reg_write(PAP1110_REG_WIDTH_HI, WORD_HI_BYTE(PAP11XX_X_MAX));
	reg_write(PAP1110_REG_WIDTH_LO, WORD_LO_BYTE(PAP11XX_X_MAX));
	mdelay(1);

	reg_write(PAP1110_REG_FLASH_CTL, PAP1110_FLASH_SAVE_CUST_REGS);
	mdelay(100);
	retry = 25;
	do {
		reg_read(PAP1110_REG_FLASH_CTL, &val);
		if (IS_BIT_CLEAR(val, PAP1110_FLASH_BUSY_BIT));
			break;
		msleep(20);
	} while (retry-- > 0);
	if (!retry) {
		pr_err(MODULE_NAME "%s flash busy! (%d)\n", __func__, __LINE__);
		return -EBUSY;
	}

	pap11xx_disable_flash();
	reg_write(PAP1110_REG_WD, PAP1110_WD_ENABLE);
	return 0;
}
#endif

/**
 *  flash programming - drive/sense block
 */
static int pap1110_update_ds_block(void)
{
	int retry;
	uint8_t fw_id;
	uint8_t i, val;
	uint8_t sense[] = {
		0x9a, 0x78, 0x56, 0x34, 0x12, 0xb0, 0xdc,
	};
	uint8_t drive_v4[] = {
		0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02,
		0x01, 0x00, 0x17, 0x16, 0x0a, 0x0b, 0x0c, 0x0d,
		0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
	};
	uint8_t drive_v2[] = {
		0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03,
		0x02, 0x01, 0x00, 0x17, 0x0b, 0x0c, 0x0d, 0x0e,
		0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
	};


	pr_func();
	reg_read(PAP11XX_REG_FW_REV_ID, &fw_id);
	if (fw_id < 0x31 || fw_id > 0x34) { //SZSZ Need review versioning conflict
		pr_info(MODULE_NAME "FW ID: 0x%02X - DS programming is not needed.\n",
			fw_id);
		return 0;
	}

	/* download procedure - prestart */
	pap11xx_enable_flash();
	reg_write(PAP1110_REG_WD, PAP1110_WD_DISABLE);
	reg_write(PAP1110_REG_IODL_CTL, PAP1110_ENABLE_DL_DS);
	mdelay(1);
	reg_write(PAP1110_REG_IODL_DATA, 0x10);

	/* row/col */
	reg_write(PAP1110_REG_IODL_DATA, 0x0b);
	reg_write(PAP1110_REG_IODL_DATA, 0x14);


	/* sense */
	for (i = 0; i < 7; i++)
		reg_write(PAP1110_REG_IODL_DATA, sense[i]);

	/* drive */
	if (fw_id == 0x34) {
		pr_info(MODULE_NAME "FW ID: %02x - write D/S table of version 4\n",
				fw_id);
		for (i = 0; i < 24; i++)
			reg_write(PAP1110_REG_IODL_DATA, drive_v4[i]);
	} else {
		pr_info(MODULE_NAME "FW ID: %02x - write D/S table of version 2\n",
				fw_id);
		for (i = 0; i < 24; i++)
			reg_write(PAP1110_REG_IODL_DATA, drive_v2[i]);
	}

	reg_write(PAP1110_REG_FLASH_CTL, PAP1110_FLASH_SAVE_DS);
	mdelay(100);
	retry = 25;
	do {
		reg_read(PAP1110_REG_FLASH_CTL, &val);
		if (IS_BIT_CLEAR(val, PAP1110_FLASH_BUSY_BIT));
			break;
		mdelay(20);
	} while (retry-- > 0);
	if (!retry) {
		pr_err(MODULE_NAME "%s flash busy! (%d)\n", __func__, __LINE__);
		return -EBUSY;
	}

	pap11xx_disable_flash();
	reg_write(PAP1110_REG_WD, PAP1110_WD_ENABLE);
	return 0;
}

/**
 *  Program data to flash data pages
 */
static void pap1110_update_flash_page(void)
{
	/* erase whole page */
	pap1110_erase_flash_page();

	/* update calibration data */
	pap1110_update_cal_block();

	/* update drive/sense mapping - only needed for FW 0x31~0x34 */
	pap1110_update_ds_block();

	pap11xx_reset(POWER_HARD_RESET);
}

/**
 *  Load FW to PAP1110
 *
 *  @ts - driver data structure
 *  @force_download - dont check fw version, download it anyway
 */
static int pap1110_load_fw(struct pap11xx_data *ts, int force_download)
{
	uint8_t fw_id, val;
	int i, retry, len = sizeof(fw_pap1110);
	uint8_t *pfw = fw_pap1110;
	uint8_t desired_fw_id = PAP1110_FW_VERSION;

	pr_func();

	/* reset fw update flag */
	ts->fw_update = 0;

	/* check fw id */
	reg_read(PAP11XX_REG_FW_REV_ID, &fw_id);
	if (fw_id == 0) {
		pap11xx_reset(POWER_HARD_RESET);
		reg_read(PAP11XX_REG_FW_REV_ID, &fw_id);
	}

	pr_info(MODULE_NAME "FW on panel: 0x%x; FW with driver: 0x%x; force_download: %d. "
		"\n", fw_id, desired_fw_id, force_download);

	/* SZ PixArt Panels changed mappings between rev3 and rev4
	 * 	FW version before 0x34 (not 0x3A) are for rev3, those after 0x34 (not 0x3A) inclusive
	 * 	are for rev4.
	 */
	// if (!force_download && fw_id >= desired_fw_id && fw_id != 0xff) {

    // check to see whether we should NOT update FW
	if (!force_download && fw_id != 0xff && fw_id >= desired_fw_id) {
		pr_info(MODULE_NAME "Acceptable FW version already present "
			"(rev 0x%x). No reflash needed.\n", fw_id);
		return 0;
	}

	/* enable flash */
	pap11xx_enable_flash();

	/* disable re-storing settings from flash after the next reset,
		wait at most 500ms */
	reg_write(PAP1110_REG_FLASH_CTL, PAP1110_FLASH_NO_SAVE_CUST_REGS);
	mdelay(100);
	retry = 25;
	do {
		reg_read(PAP1110_REG_FLASH_CTL, &val);
		if (IS_BIT_CLEAR(val, PAP1110_FLASH_BUSY_BIT));
			break;
		msleep(20);
	} while (retry-- > 0);
	if (!retry) {
		pr_err(MODULE_NAME "%s flash busy! (%d)\n", __func__, __LINE__);
		return -EBUSY;
	}

	/* partial reset */
	pap11xx_reset(POWER_PARTIAL_RESET);
	reg_read(PAP11XX_REG_FW_REV_ID, &fw_id);
	if (fw_id) {
		pr_err(MODULE_NAME "%s partial reset fail! fw_id=%02X\n",
			__func__, fw_id);
		return -EIO;
	}

	/* download procedure - prestart */
	reg_write(PAP1110_REG_WD, PAP1110_WD_DISABLE);
	pap11xx_enable_flash();
	reg_write(PAP11XX_REG_BOOT_STAT, 0);
	reg_write(PAP1110_ROM_REG_FLASH_CTL, PAP1110_FLASH_ERASE);
	reg_write(PAP1110_REG_IODL_CTL, PAP1110_ENABLE_DL_FW);
	mdelay(50);

	/* dowload busy checking - wait at most 2 seconds,
		busy bit = 1 means busy */
	retry = 200;
	do {
		reg_read(PAP1110_REG_IODL_CTL, &val);
		if (IS_BIT_CLEAR(val, PAP1110_DL_BUSY_BIT));
			break;
		msleep(10);
	} while (retry-- > 0);
	if (!retry) {
		reg_read(PAP1110_REG_ERROR, &val);
		pr_err(MODULE_NAME "%s flash IODL busy! (%d) - error=0x%02x\n",
			__func__, __LINE__, val);
		return -EBUSY;
	}
	reg_read(PAP1110_REG_ERROR, &val);

	/* download start */
	i = 0;
	while (i < len) {
		reg_write(PAP1110_REG_IODL_DATA, *(pfw + i));
		udelay(PAP11XX_DELAY_US_BETWEEN_FLASH_WRITES);
		i++;
	}

	/* wait for reset complete - retry 1 second */
	retry = 100;
	do {
		reg_read(PAP11XX_REG_BOOT_STAT, &val);
		if (IS_BIT_SET(val, PAP11XX_BOOT_DL_FAILURE)) {
			reg_read(PAP1110_REG_ERROR, &val);
			pr_err(MODULE_NAME "%s flash IODL fail! (%d) - error=0x%02x\n",
				__func__, __LINE__, val);
			break;
		}
		if (IS_BIT_SET(val, PAP11XX_BOOT_DL_DONE))
			break;
		msleep(10);
	} while (retry-- > 0);

	reg_write(PAP1110_REG_WD, PAP1110_WD_ENABLE);

	/* check fw id */
	reg_read(PAP11XX_REG_FW_REV_ID, &fw_id);
	if (fw_id != desired_fw_id) {
		reg_read(PAP1110_REG_ERROR, &val);
		pr_err(MODULE_NAME "%s FW DL fail! (%d) - error=0x%02x\n",
			__func__, __LINE__, val);
		return -EIO;
	} else
		pr_info(MODULE_NAME "New FW_VER 0x%02x\n", fw_id);

	/* enable flash */
	pap11xx_enable_flash();

	/* disable re-storing settings from flash after the next reset,
		wait at most 500ms */
	reg_write(PAP1110_REG_FLASH_CTL, PAP1110_FLASH_NO_SAVE_CUST_REGS);
	mdelay(100);
	retry = 25;
	do {
		reg_read(PAP1110_REG_FLASH_CTL, &val);
		if (IS_BIT_CLEAR(val, PAP1110_FLASH_BUSY_BIT));
			break;
		msleep(20);
	} while (retry-- > 0);
	if (!retry) {
		pr_err(MODULE_NAME "%s flash busy! (%d)\n", __func__, __LINE__);
		return -EBUSY;
	}

	pap11xx_reset(POWER_HARD_RESET);
	if (pap11xx_wait_boot(BOOT_NAV_READY) < 0) {
		pr_err(MODULE_NAME "%s flash nav busy! (%d)\n", __func__, __LINE__);
		return -EBUSY;
	}

	/* A hardware reset by reset pin is recommended here. */


	reg_write(PAP1110_REG_WD, PAP1110_WD_ENABLE);
	pap11xx_disable_flash();

	/* set fw update flag */
	ts->fw_update = 1;
	pr_info(MODULE_NAME "%s Done!\n", __func__);
	return 0;
}

/**
 *  Flash CRC Check
 *
 *  @block: read block crc from a specific block
 */
static int pap1110_crc_check(uint8_t block)
{
	uint8_t crc_hi, crc_lo, val;
	uint8_t tar_crc_hi, tar_crc_lo; /* target crc - the correct crc */
	uint8_t fw_id;
	int retry;

	switch (block) {
	case FLASH_BLOCK_DS:
		reg_write(PAP1110_REG_FLASH_CRC_CTL, PAP1110_FLASH_CRC_CTL_DS);
		reg_read(PAP11XX_REG_FW_REV_ID, &fw_id);
		if (fw_id == 0x34) {
			tar_crc_hi = PAP1110_DS_CRC_V4_HI;
			tar_crc_lo = PAP1110_DS_CRC_V4_LO;
		} else {
			tar_crc_hi = PAP1110_DS_CRC_V2_HI;
			tar_crc_lo = PAP1110_DS_CRC_V2_LO;
		}
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	mdelay(100);

	/* wait until done, delay 100ms at most */
	retry = 100;
	do {
		reg_read(PAP1110_REG_FLASH_CRC_CTL, &val);
		if (IS_BIT_CLEAR(val, PAP1110_FLASH_CRC_CTL_BUSY))
			break;
		mdelay(1);
	} while (retry-- > 0);
	if (retry <= 0)
		goto load_crc_fail;

	reg_read(PAP1110_REG_CRC_HI, &crc_hi);
	reg_read(PAP1110_REG_CRC_LO, &crc_lo);

	if (crc_hi == tar_crc_hi && crc_lo == tar_crc_lo) {
		pr_info(MODULE_NAME "CRC-%d read: %02x%02x target:%02x%02x - Done!\n",
			block, crc_hi, crc_lo, tar_crc_hi, tar_crc_lo);
		return 0;
	} else {
		pr_info(MODULE_NAME "CRC-%d read: %02x%02x target:%02x%02x\n",
			block, crc_hi, crc_lo, tar_crc_hi, tar_crc_lo);
		return 1;
	}

load_crc_fail:
	pr_err(MODULE_NAME "%s - (%d): flash busy\n", __func__, block);
	return -EBUSY;
}

/**
 *  Disable interrupt pin of pap11xx.
 *
 *  @disable - disable interrupt if disable is true, false otherwise.
 */
static void pap11xx_disable_interrupt(bool disable)
{
	static uint8_t saved_reg = 0;
	uint8_t reg = 0;

	if (disable) {
		reg_read(PAP11XX_REG_INT_MASK, &reg);
		reg_write(PAP11XX_REG_INT_MASK, 0);
		if (reg != 0x0)
			saved_reg = reg;
		pr_debug("%s save %02x\n", __func__, saved_reg);
	} else {
		if (saved_reg != 0) {
			reg_write(PAP11XX_REG_INT_MASK, saved_reg);
			pr_debug("%s store %02x\n", __func__, saved_reg);
		} else {
			WARN_ON(1);
		}
	}
}

/**
 *  sysfs allocates a buffer of size (PAGE_SIZE) and passes it to the
 *  method. Sysfs will call the method exactly once for each read or
 *  write. This forces the following behavior on the method
 */
static ssize_t pap11xx_user_result(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i = 0;
	uint8_t val;
	uint16_t len = (uint16_t) user_cmd.number;
	static uint8_t data[128];
	int ret = 0;

	struct pap11xx_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	flush_workqueue(ts->pap11xx_wq);
	switch (user_cmd.command) {
	case SYSFS_READ:
		/**
		 *      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
		 * 00: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
		 * 10: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
		 * 20: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
		 * 30: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
		 * 40: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
		 * 50: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
		 * 60: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
		 * 70: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
		 */
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "    ");
		for (i = 0; i < 16; i++)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%2x ", i);

		i = user_cmd.reg[0] & 0xf0;
		for (; i < ((user_cmd.reg[0] + len + 15) & 0xf0); i++) {
			/* new line: show reg index */
			if (!(i & 0xf))
				ret += scnprintf(buf + ret, PAGE_SIZE - ret,
					"\n%02x: ", i);

			if (i >= user_cmd.reg[0] && i < user_cmd.reg[0] + len) {
				reg_read(i, &val);
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%02x ", val);
			}
			else
				ret += scnprintf(buf + ret, PAGE_SIZE - ret, "   ");
		}
		break;

	case SYSFS_READ3N:
		for (i = 0; i < len; i++) {
			reg_read(user_cmd.reg[0], &data[0]);
			reg_read(user_cmd.reg[1], &data[1]);
			reg_read(user_cmd.reg[2], &data[2]);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%02x %02x %02x\n",
				data[0], data[1], data[2]);
			DBG_CMD(MODULE_NAME "double read: %02x %02x %02x\n",
				data[0], data[1], data[2]);
		}
		break;

	case SYSFS_READ2N:
		for (i = 0; i < len; i++) {
			reg_read(user_cmd.reg[0], &data[0]);
			reg_read(user_cmd.reg[1], &data[1]);

			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%02x %02x\n",
				data[0], data[1]);
			DBG_CMD(MODULE_NAME "double read: %02x %02x\n", data[0], data[1]);
		}
		break;

	case SYSFS_WRITE_READ:
		reg_write(user_cmd.reg[0], user_cmd.val0);
		reg_write(user_cmd.reg[1], user_cmd.val1);
		for (i = 0; i < len; i++) {
			reg_read(user_cmd.reg[2], &val);
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%02x ", val);
		}
		break;

	case SYSFS_VERSION:
		ret = scnprintf(buf, PAGE_SIZE, "%s\n", DRIVER_VERSION);
		break;

	case SYSFS_FIRMWARE:
		ret = scnprintf(buf, PAGE_SIZE, "%02x\n", user_cmd.val0);
		break;

	case SYSFS_KDBGLEVEL:
		ret = scnprintf(buf, PAGE_SIZE, "drv_debug_mask = %02x\n",
				drv_debug_mask);
		break;

	default:
		break;
	}

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	return (ssize_t)ret;
}

/**
 *  Chip control via user commands.
 */
static ssize_t pap11xx_user_cmd(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int offset;
	const char *p;
	uint32_t param[6];

	struct pap11xx_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	flush_workqueue(ts->pap11xx_wq);
	switch (buf[0]) {
	case 'r':
		/* ex: r 0 80 */
		user_cmd.command = SYSFS_READ;
		p = buf + 1;
		sscanf(p, "%x %x", param, param + 1);
		user_cmd.reg[0] = (uint8_t) param[0];
		user_cmd.number = (uint16_t) param[1];
		DBG_CMD(MODULE_NAME "read mode: %02x %02x \n", user_cmd.reg[0],
			user_cmd.number);
		break;

	case 'w':
		/* ex: w 74 02 75 03 */
		user_cmd.command = SYSFS_WRITE;
		p = buf + 1;
		while (sscanf(p, "%x %x%n", param, param + 1, &offset) == 2) {
			user_cmd.reg[0] = (uint8_t) param[0];
			user_cmd.data   = (uint8_t) param[1];
			p += offset;
			reg_write(user_cmd.reg[0], user_cmd.data);
			DBG_CMD(MODULE_NAME "write mode: %02x %02x \n", user_cmd.reg[0],
				user_cmd.data);
		}
		break;

	case 'b': {
			char ch;
			int size = 0;
			user_cmd.command = SYSFS_BURST;
			p = buf + 1;
			/* find out what address to write to */
			if (sscanf(p, "%x%n", param, &offset) != 1) {
				user_cmd.command = SYSFS_NULL;
				break;
			}
			user_cmd.reg[0] = (uint8_t) param[0];
			p += offset;
			/* write the data bytes which follow */
			while (sscanf(p, "%x%n", param, &offset) == 1) {
				ch = (char) param[0];
				p += offset;
				reg_write(user_cmd.reg[0], ch);
				size += 1;
				/* DBG_CMD(MODULE_NAME "burst write %02x %02x \n",
					user_cmd.reg[0], ch); */
			}
			DBG_CMD(MODULE_NAME "burst write %d bytes.\n", size);
		}
		break;

	case 'd':
		user_cmd.command = SYSFS_DISABLE;
		DBG_CMD(MODULE_NAME "navigation disabled\n");
		pap11xx_disable_interrupt(true);
		break;

	case 'e':
		user_cmd.command = SYSFS_ENABLE;
		DBG_CMD(MODULE_NAME "navigation enabled\n");
		pap11xx_disable_interrupt(false);
		break;

	case 't':
		user_cmd.command = SYSFS_READ3N;
		p = buf + 1;
		sscanf(p, "%x %x %x %x", param, param + 1, param + 2, param + 3);
		user_cmd.reg[0] = (uint8_t) param[0];
		user_cmd.reg[1] = (uint8_t) param[1];
		user_cmd.reg[2] = (uint8_t) param[2];
		user_cmd.number = (uint16_t) param[3];
		DBG_CMD(MODULE_NAME "triple read mode: 0x%02x 0x%02x 0x%02x x "
			"0x%02x times\n", user_cmd.reg[0], user_cmd.reg[1], user_cmd.reg[2],
			user_cmd.number);
		break;

	case 'u':
		user_cmd.command = SYSFS_READ2N;
		p = buf + 1;
		sscanf(p, "%x %x %x", param, param + 1, param + 2);
		user_cmd.reg[0] = (uint8_t) param[0];
		user_cmd.reg[1] = (uint8_t) param[1];
		user_cmd.number = (uint16_t) param[2];
		DBG_CMD(MODULE_NAME "double read mode: 0x%02x 0x%02x x "
			"0x%02x times\n", user_cmd.reg[0], user_cmd.reg[1],
			user_cmd.number);
		break;

	case 'm':
		user_cmd.command = SYSFS_WRITE_READ;
		p = buf + 1;
		sscanf(p, "%x %x %x %x %x %x", param, param + 1, param + 2,
				param + 3, param + 4, param + 5);
		user_cmd.reg[0] = (uint8_t) param[0];
		user_cmd.val0 = (uint8_t) param[1];
		user_cmd.reg[1] = (uint8_t) param[2];
		user_cmd.val1 = (uint8_t) param[3];
		user_cmd.reg[2] = (uint8_t) param[4];
		user_cmd.number = (uint16_t) param[5];
		DBG_CMD(MODULE_NAME "write 2, read 1 values set %x %x %x %x %x %x\n",
			user_cmd.reg[0], user_cmd.val0, user_cmd.reg[1], user_cmd.val1,
			user_cmd.reg[2], user_cmd.number);
		break;

	case 'v':
		user_cmd.command = SYSFS_VERSION;
		pr_info(MODULE_NAME "Ver: %s\n", DRIVER_VERSION);
		break;

	case 'f': {
			struct pap11xx_data *ts = i2c_get_clientdata(to_i2c_client(dev));
			user_cmd.command = SYSFS_FIRMWARE;
			user_cmd.val0 = pap1110_load_fw(ts, 1);
		}
		break;

	case 'i':
		pap1110_flash_config(ts);
		break;

	case 'k':
		user_cmd.command = SYSFS_KDBGLEVEL;
		p = buf + 1;
		sscanf(p, "%x ", &drv_debug_mask);
		pr_info(MODULE_NAME "kernel debug level changed to: 0x%02x\n",
			drv_debug_mask);
		break;

	case 's':
		pap1110_erase_flash_page();
		break;

	default:
		break;
	}

	return count;
}

static DEVICE_ATTR(m, S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP | S_IROTH,
		pap11xx_user_result, pap11xx_user_cmd);

static struct attribute *tp_attributes[] = {
	&dev_attr_m.attr,
	NULL
};

static const struct attribute_group tp_attr_group = {
	.attrs = tp_attributes,
};

/**
 *  Sysfs init.
 */
static int pap11xx_sysfs_create(struct pap11xx_data *ts)
{
	pr_func();

	/* Register sysfs hooks */
	if (sysfs_create_group(&drv_client->dev.kobj, &tp_attr_group)) {
		pr_err(MODULE_NAME "%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}

	return 0;
}

static int pap11xx_sysfs_remove(struct pap11xx_data *ts)
{
	pr_func();

	/* Free sysfs hooks */
	sysfs_remove_group(&drv_client->dev.kobj, &tp_attr_group);
	return 0;
}

/**
 *  Load FW for different chips
 *
 *  @ts - driver data structure
 */
#if ENABLE_FW_DOWNLOAD
static int pap11xx_load_fw(struct pap11xx_data *ts)
{
	uint8_t hw_id;
	uint8_t force_download = 0;

	pr_func();

	switch (ts->pid) {
	case PID_PAP1110:
		reg_read(PAP11XX_REG_HW_REV_ID, &hw_id);
		switch (hw_id) {
		case HW_ID_ROM10:
			pr_info(MODULE_NAME "%s: hw_id=%02X\n", __func__, hw_id);

#if FORCE_FW_DOWNLOAD
			force_download = 1;
#endif
			return pap1110_load_fw(ts, force_download);
		default:
			pr_err(MODULE_NAME "%s: unsupported hw id! hw_id=%02X\n",
				__func__, hw_id);
			return -ENODEV;
		}
		break;
	default:
		pr_err(MODULE_NAME "%s: unsupported device! pid=%02X\n",
			__func__, ts->pid);
		return -ENODEV;
	}

	/* should not possible to come here */
	pr_err(MODULE_NAME "%s: unsupported device! pid=%02X\n",
		__func__, ts->pid);
	WARN_ON(1);
	return -EIO;
}
#endif

/**
 *  Panel initization - modify register settings here
 */
static void pap1110_flash_config(struct pap11xx_data *ts)
{
	uint8_t update_page = 0;
	uint8_t fw_id;

	reg_read(PAP11XX_REG_FW_REV_ID, &fw_id);
	pr_info(MODULE_NAME "%s (X, Y) = (%d/%d)\n",
		__func__, PAP11XX_X_MAX, PAP11XX_Y_MAX);

	/* osc reg check */
	if (pap1110_osc_trim_reg_check()) {
		pr_info(MODULE_NAME "%s update calibration data.\n", __func__);
		BIT_SET(update_page, UPDATE_CALIBRATION);
	}

	if (fw_id >= 0x31 && fw_id <= 0x34) { //SZSZ needs review
		if (pap1110_crc_check(FLASH_BLOCK_DS)) {
			pr_info(MODULE_NAME "%s update drive/sense block \n", __func__);
			BIT_SET(update_page, UPDATE_DRIVE_SENSE);
		}
	}

	if (update_page)
		pap1110_update_flash_page();
	else
		pr_info(MODULE_NAME "%s flash checked, no update needed.\n",
			__func__);
}

/**
 *  Panel initization - modify register settings here
 *
 *  @ts - driver data structure
 */
static int pap11xx_init_panel(struct pap11xx_data *ts)
{
	uint8_t ret = 0;
	uint8_t fw_id;

	pr_func();

#if ENABLE_FW_DOWNLOAD
	reg_read(PAP11XX_REG_FW_REV_ID, &fw_id);
	pr_info(MODULE_NAME "PXI FW Version: 0x%02X\n", fw_id);
	/* lock fw version in case the device is 0x31~0x33
		for different HW version */
	if (fw_id > 0x33 || fw_id < 0x31) {
		ret = pap11xx_load_fw(ts);
		if (ret)
			return ret;
	}
#endif

	/* store fw version in module param */
	reg_read(PAP11XX_REG_FW_REV_ID, &fw_id);
	sprintf(fw_version, "0x%x", fw_id);

	switch (ts->pid) {
	case PID_PAP1110:
		pap1110_flash_config(ts);
		break;
	default:
		pr_err(MODULE_NAME "%s: unsupported device! pid=%02X\n",
			__func__, ts->pid);
		return -ENODEV;
	}

	return ret;
}

/**
 *  Get motion report from chip
 *
 *  @ts - driver data structure
 *  @report - (output) the location of the current motion report
 */
static int pap11xx_get_report(struct pap11xx_data *ts,
		struct touch_report *report)
{
	int i;
	uint8_t data[16];

	/* enable motion report read */
	reg_set_mask(PAP1110_REG_MOTIOM_REPORT_CTL,
		PAP1110_MOTION_ACCESS_REPORT);
	udelay(200);

	/* motion status, number of points,id0,x0 */
	reg_arr_read(PAP1110_REG_MOTION_REPORT_DATA, data, 2);
	if (data[0] == 0xff) {
		DBG_STATUS(MODULE_NAME "ignoring report w/ status = 0xff.\n");
		/* ignore invalid motion reports */
		return -EINVAL;
	}
	report->status = data[0];
	report->total_touch = data[1];

	/* sanity checking */
	if (report->total_touch > PAP11XX_MAX_FINGER) {
		DBG_STATUS(MODULE_NAME "ignoring report w/ finger = %d.\n",
			report->total_touch);
		return -EINVAL;
	}

	/* read touch points */
	for (i = 0; i < report->total_touch; i++) {
		reg_arr_read(PAP1110_REG_MOTION_REPORT_DATA, data,
			sizeof(struct point_data));
		report->point_data[i].id = data[0] & 0x7f;
		report->point_data[i].x = (data[2] << 8) | data[1];
		report->point_data[i].y = (data[4] << 8) | data[3];
		report->point_data[i].force = (data[6] << 8) | data[5];
		report->point_data[i].area = (data[8] << 8) | data[7];

		// Orientation swap, may be other method using regs.
		if (revert_x)
			report->point_data[i].x = PAP11XX_X_MAX - report->point_data[i].x;
		if (revert_y)
			report->point_data[i].y = PAP11XX_Y_MAX - report->point_data[i].y;		

		/* sanity check before passing on */
		if (report->point_data[i].x > PAP11XX_X_MAX) {
			DBG_STATUS(MODULE_NAME "ignored illegal x value - %d\n",
					report->point_data[i].x);
			goto err_get_report;
		} else if (report->point_data[i].y > PAP11XX_Y_MAX) {
			DBG_STATUS(MODULE_NAME "ignored illegal y value - %d\n",
					report->point_data[i].y);
			goto err_get_report;
		} else if (report->point_data[i].force > PAP11XX_FORCE_MAX
				|| report->point_data[i].force < PAP11XX_FORCE_MIN) {
			DBG_STATUS(MODULE_NAME "ignored illegal z value - %d\n",
					report->point_data[i].force);
			goto err_get_report;
		} else if (report->point_data[i].area > PAP11XX_AREA_MAX
				|| report->point_data[i].area < PAP11XX_AREA_MIN) {
			DBG_STATUS(MODULE_NAME "ignored illegal area value - %d\n",
					report->point_data[i].area);
			goto err_get_report;
		} else if (report->point_data[i].id > PAP11XX_ID_MAX - 1) {
			DBG_STATUS(MODULE_NAME "ignored illegal id value - %d\n",
					report->point_data[i].id);
			goto err_get_report;
		}
	}

	/* reset buffer in case of no touch */
	if (report->total_touch == 0) {
		pr_debug(MODULE_NAME "reset buffer\n");
		memset((void *) &(ts->pre_touch_report), 0x00,
			sizeof (ts->pre_touch_report));
	}

	return 0;

err_get_report:
	return -EINVAL;
}

/**
 *  Tracking finger status.
 *
 *  @report - current motion report
 *  @ts - driver data structure
 */
static void pap11xx_finger_status_update(struct touch_report *report,
		struct pap11xx_data *ts)
{
	struct input_dev *input_dev = ts->input;

	int i = 0, id;

	/* clear current finger status */
	for (i = 0; i < PAP11XX_ID_MAX; i++)
		finger_status[i] = FINGER_NO_TOUCH;

	/* fill in finger status */
	for (i = 0; i < report->total_touch; i++) {
		id = report->point_data[i].id;
		finger_status[id] = FINGER_TOUCH;
	}

	/* compare finger status */
	for (i = 0; i < PAP11XX_ID_MAX; i++) {
		if (!pre_finger_status[i] && !finger_status[i])
			finger_status[i] = FINGER_NO_TOUCH;
		else if (!pre_finger_status[i] && finger_status[i])
			finger_status[i] = FINGER_RELEASE_TO_TOUCH;
		else if (pre_finger_status[i] && finger_status[i])
			finger_status[i] = FINGER_TOUCH_TO_TOUCH;
		else {
			/* release a touch */
			finger_status[i] = FINGER_TOUCH_TO_RELEASE;
      // multi-touch
      input_mt_slot(input_dev, i);
      input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
      input_report_abs(input_dev, ABS_MT_PRESSURE, 0);
      // single touch
      input_mt_report_pointer_emulation(input_dev, 0);
	  }
	}

	/* save finger status */
	for (i = 0; i < PAP11XX_ID_MAX; i++)
		pre_finger_status[i] = finger_status[i];
}

/**
 *  Print out report
 *
 *  @report - current motion report
 *  @status - the status of this chip
 */
static void pap11xx_show_report(struct touch_report *report, uint8_t status)
{
	int i;

	touch_times += 1;
	DBG_RPT(MODULE_NAME "%d pt_status=%02x motion_status=%02x, total=%x, \n",
		touch_times, status, report->status, report->total_touch);
	for (i = 0; i < report->total_touch; i++) {
		DBG_RPT(MODULE_NAME "id=%d, x=%d, y=%d, force=%d, width=%d \n",
			report->point_data[i].id,
			report->point_data[i].x,
			report->point_data[i].y,
			report->point_data[i].force,
			report->point_data[i].area);
	}
	DBG_RPT(MODULE_NAME "\n");
}

/**
 *  Worker function.
 *
 *  @Note: Read motion report from chip and send out a input event.
 */
static void pap11xx_work_func(struct work_struct *irq_work)
{
	int i, report_available, id;
	uint8_t status, reg_err;
	struct touch_report touch_report;
	struct pap11xx_data *ts = container_of(irq_work, struct pap11xx_data, irq_work);
	struct input_dev *input_dev = ts->input;

	if (ts->power >= TOUCH_POWEROFF)
		goto end_work_disabled;

	/* read status and see if this report is valid or not */
	reg_read(PAP11XX_REG_STATUS, &status);
	if (IS_BIT_SET(status, PAP11XX_STATUS_TOUCH_CHANGE)
			|| IS_BIT_SET(status, PAP11XX_STATUS_TOUCH))
		report_available = 1;
	else {
		memset((void *) &(ts->pre_touch_report), 0x00,
			sizeof (ts->pre_touch_report));
	}
	DBG_STATUS(MODULE_NAME "status (0x04) =0x%02x\n", status);

	if (IS_BIT_SET(status, PAP11XX_STATUS_ERROR)) {
		reg_read(PAP1110_REG_ERROR, &reg_err);
		DBG_STATUS(MODULE_NAME "error register (0x0d) =0x%02x\n", reg_err);
	}

	if (report_available) {
		if (pap11xx_get_report(ts, &touch_report)) {
			DBG_STATUS(MODULE_NAME "pap11xx_get_report error\n");
			goto end_of_work_func;
		}
		else if (touch_report.status == 0 && status == 0) {
			DBG_STATUS(MODULE_NAME "touch_report.status error\n");
			goto end_of_work_func;
		}

		if (IS_BIT_SET(status, PAP11XX_STATUS_TOUCH_CHANGE)
				&& touch_report.status == 0)
			touch_report.total_touch = 0;

		/* show report */
		pap11xx_show_report(&touch_report, status);

		pap11xx_finger_status_update(&touch_report, ts);

    //printk( MODULE_NAME "total touch: %d\n", touch_report.total_touch );
		//input_mt_report_finger_count(input_dev, touch_report.total_touch);

		if (touch_report.total_touch) {
			for (i = 0; i < touch_report.total_touch; i++) {
				id = touch_report.point_data[i].id;
				switch (finger_status[id]) {
				case FINGER_RELEASE_TO_TOUCH:
				case FINGER_TOUCH_TO_TOUCH:
          // multi-touch
					input_mt_slot(input_dev, id);
					input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
          input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, touch_report.point_data[i].area);
					input_report_abs(input_dev, ABS_MT_POSITION_X, touch_report.point_data[i].x);
					input_report_abs(input_dev, ABS_MT_POSITION_Y, touch_report.point_data[i].y);
					if (report_pressure == 0)
					  input_report_abs(input_dev, ABS_MT_PRESSURE,
					      (touch_report.point_data[i].force / pressure_scale) | 0x01);
          else
            input_report_abs(input_dev, ABS_MT_PRESSURE, report_pressure);
          // single touch
          input_mt_report_pointer_emulation(input_dev, 0);
					break;
				}
			}
		}
		input_sync(input_dev);
	}

end_of_work_func:
	/* disable motion repot read */
	reg_clear_mask(PAP1110_REG_MOTIOM_REPORT_CTL,
		PAP1110_MOTION_ACCESS_REPORT);

end_work_disabled:
    enable_irq(ts->client->irq);
}

/**
 *  Interrupt handler, queue a work and read motion report later.
 */
static irqreturn_t pap11xx_irq_handler(int irq, void *dev_id)
{
	struct pap11xx_data *ts = dev_id;

	if (!ts->hw_check)
		return IRQ_HANDLED;

	disable_irq_nosync(drv_client->irq);
	if (!work_pending(&ts->irq_work))
		queue_work(ts->pap11xx_wq, &ts->irq_work);

	return IRQ_HANDLED;
}

/**
 *  Wait until chip boot success.
 *
 *  @type -
 *      BOOT_COMPLETE: boot up success
 *      BOOT_NAV_READY: navigation ready
 */
static int pap11xx_wait_boot(uint8_t type)
{
	int retry;
	uint8_t val, bit_event;

	switch (type) {
	case BOOT_COMPLETE:
		bit_event = PAP11XX_BOOT_COMPLETED;
		break;
	case BOOT_NAV_READY:
		bit_event = PAP11XX_BOOT_NAV_READY;
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	/* wait for reset complete - retry 1 second */
	retry = 100;
	do {
		reg_read(PAP11XX_REG_BOOT_STAT, &val);
		if (IS_BIT_SET(val, bit_event))
			break;
		msleep(15);
	} while (retry-- > 0);
	if (!retry)
		goto err_boot_fail;

	return 0;

err_boot_fail:
	pr_err(MODULE_NAME "%s: boot stat reg=0x%02x\n", __func__, val);

	return -EBUSY;
}

/**
 *  Chip reset.
 *
 *  @type -
 *      POWER_HARD_RESET: Hard reset
 *      POWER_PARTIAL_RESET: Reset and run on ROM mode
 */
static int pap11xx_reset(uint8_t type)
{
	uint8_t reset;

	if (reg_write(PAP11XX_REG_SHUTDOWN, PAP11XX_SHUTDOWN) < 0)
		goto err_write_fail;

	udelay(PAP11XX_SHUTDOWN_DELAY_US);

	switch (type) {
	case POWER_HARD_RESET:
		reset = PAP11XX_HARD_RESET;
		break;
	case POWER_PARTIAL_RESET:
		reset = PAP11XX_PARTIAL_RESET;
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	if (reg_write(PAP11XX_REG_SHUTDOWN, reset) < 0)
		goto err_write_fail;

	/* wait for reset complete - retry 1 second */
	if (pap11xx_wait_boot(BOOT_COMPLETE) < 0)
		return -EBUSY;

	return 0;

err_write_fail:
	pr_err(MODULE_NAME "%s: write reg failed\n", __func__);

	return -ENODEV;
}

#if !PIXART_DEV_BOARD_SETUP

static void reset_panel(struct pap11xx_i2c_platform_data *pdata)
{
	nxp_soc_gpio_set_out_value(pdata->gpio_reset,  (pdata->reset_cfg)); // reset
	// msleep (160); //quadruple x 10
	msleep (16); //quadruple
	nxp_soc_gpio_set_out_value(pdata->gpio_reset, !(pdata->reset_cfg)); // unreset/ena
	msleep (200); // waiting for PixArt's spec
	return;
}

#endif

static int __devinit pap11xx_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct pap11xx_i2c_platform_data *pdata;
	struct pap11xx_data *ts;
	struct input_dev *input_dev;
	int err = 0, ret;

	pr_info(MODULE_NAME "%s: init i2c. ver: %s\n", __func__, DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err(MODULE_NAME "%s: need I2C_FUNC_I2C\n", __func__);
		err = -ENODEV;
		goto err_check_func;
	}

	ts = kzalloc(sizeof (struct pap11xx_data), GFP_KERNEL);
	if (!ts) {
		err = -ENOMEM;
		pr_err(MODULE_NAME "%s: out of mem\n", __func__);
		goto err_alocate_fail;
	}
	ts->hw_check = 0;

	pdata = client->dev.platform_data;
	if (!pdata) {
		err = -EINVAL;
		dev_err(&client->dev, "failed to get platform data\n");
		goto err_no_pdata;
	}

	INIT_WORK(&ts->irq_work, pap11xx_work_func);
	i2c_set_clientdata(client, ts);

	ts->pap11xx_wq = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ts->pap11xx_wq) {
		err = -ESRCH;
		pr_err(MODULE_NAME "%s: create workqueue fail! - out of mem!?\n",
			__func__);
		goto err_create_wq;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto err_input_dev_alloc_fail;
	}

#if PIXART_DEV_BOARD_SETUP
	ctp_id = tiny4412_get_ctp();
	if (ctp_id != CTP_PAP11XX && ctp_id != CTP_AUTO) {
		return -ENODEV;
	}
	tiny4412_set_ctp(CTP_PAP11XX);

	ts->gpio_irq = pdata->gpio_irq;
	if (ts->gpio_irq != -EINVAL)
		client->irq = gpio_to_irq(ts->gpio_irq);
	else
		goto err_no_pdata;

	if (pdata->irq_cfg) {
		s3c_gpio_cfgpin(ts->gpio_irq, pdata->irq_cfg);
		s3c_gpio_setpull(ts->gpio_irq, S3C_GPIO_PULL_NONE);
	}
#else
	//// Do LF stuff
	// Is PixArt CTP? Return if not
	// Reset the CTP device
	//void nxp_soc_gpio_set_out_value(unsigned int io, int high)
	//pdata -> gpio_reset
	reset_panel(pdata);
	// Set IRQ
	// Set GPIO?

#endif

	drv_client = client;
	ts->client = client;
	ts->input = input_dev;
	ts->pid = 0;
	ts->fw_update = 0;
	ts->prod_type = PRODUCT_TYPE_MULTITOUCH;
	ts->power = TOUCH_POWERON;

//SZSZSZ	input_dev->name = PAP11XX_DEV_NAME;
	input_dev->name = "touchscreen interface";
	input_dev->dev.parent = &client->dev;
	input_dev->id.bustype = BUS_I2C;
	input_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS)
			| BIT_MASK(EV_SYN);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);

	switch (ts->prod_type) {
	case PRODUCT_TYPE_TOUCHPAD:
		break;
	case PRODUCT_TYPE_MULTITOUCH:
	default:
		__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
		break;
	}

	/* multitouch parameters, calling out generic mt func */
	input_mt_init_slots(input_dev, PAP11XX_ID_MAX);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
		0, PAP11XX_X_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
		0, PAP11XX_Y_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
		0, PAP11XX_FORCE_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
		0, PAP11XX_AREA_MAX, 0, 0);
//SZSZSZ
	input_set_abs_params(input_dev, ABS_X,
		0, PAP11XX_X_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
		0, PAP11XX_Y_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
		0, PAP11XX_FORCE_MAX, 0, 0);


	//SZSZ we may need to invent/translate request_irq(), disable_irq()
	pr_info(MODULE_NAME "%s: client irq %d", __func__, client->irq);
	if (request_irq(client->irq, pap11xx_irq_handler, IRQF_TRIGGER_RISING,
			client->dev.driver->name, ts)) {
		err = -EBUSY;
		dev_err(&client->dev, "irq %d busy? (%s)\n", client->irq,
			client->dev.driver->name);
		goto err_free_input_dev;
	}
	dev_info(&client->dev, MODULE_NAME "touchscreen, irq %d\n", client->irq);
	disable_irq(client->irq);

	ret = input_register_device(input_dev);
	if (ret) {
		err = -ENODEV;
		pr_err(MODULE_NAME "%s: input_register_device ", __func__);
		goto err_free_irq;
	}

	/* hard reset pap11xx controller */
	ret = pap11xx_reset(POWER_HARD_RESET);
	if (ret < 0) {
		err = -ENODEV;
		pr_err(MODULE_NAME "%s: hard reset fail!\n", __func__);
		goto err_free_input_dev;
	}

	/* read the id to check hardware */
	reg_read(PAP11XX_REG_PID, &ts->pid);
	if (ts->pid < 0) {
		err = -ENODEV;
		pr_err(MODULE_NAME "%s: PID check fail!\n", __func__);
		goto err_free_input_dev;
	}

	/* product verification - must be 0x8A for PAP1110 */
	switch (ts->pid) {
	case PID_PAP1110:
		pr_info(MODULE_NAME "%s: Product Major Version %x\n",
			__func__, ts->pid);
		ts->hw_check = 1;
		break;
	default:
		err = -ENODEV;
		pr_err(MODULE_NAME "%s: failed. PID reported was "
			"0x%x\n", __func__, ts->pid);
		goto err_free_input_dev;
	}

	ret = pap11xx_init_panel(ts);
	if (ret != 0) {
		err = -EINVAL;
		pr_err(MODULE_NAME "%s: init panel failed.\n", __func__);
		goto err_free_input_dev;
	}

	dev_info(&client->dev, "PixArt pap11xx TouchScreen initialized\n");
	enable_irq(client->irq);
	pap11xx_sysfs_create(ts);
	return 0;

err_free_irq:
	free_irq(client->irq, ts);

err_free_input_dev:
	input_free_device(input_dev);

err_input_dev_alloc_fail:
	cancel_work_sync(&ts->irq_work);
	destroy_workqueue(ts->pap11xx_wq);

err_create_wq:
	i2c_set_clientdata(client, NULL);

err_no_pdata:
	kfree(ts);

err_alocate_fail:
err_check_func:
	dev_err(&client->dev, "probe pap11xx TouchScreen failed, %d\n", err);

	return err;
}

static int pap11xx_ts_suspend(struct i2c_client *client, pm_message_t state)
{
	struct pap11xx_data *ts = i2c_get_clientdata(client);

	pr_func();

	disable_irq(ts->client->irq);
	ts->power = TOUCH_POWEROFF;
	cancel_work_sync(&ts->irq_work);

	/* Forces lowest power mode */
	switch (ts->pid) {
	case PID_PAP1110:
		reg_write(PAP11XX_REG_SHUTDOWN, PAP11XX_SHUTDOWN);
		break;
	default:
		pr_err(MODULE_NAME "%s: unknown device.\n", __func__);
		break;
	}

	return 0;
}

static int pap11xx_ts_resume(struct i2c_client *client)
{
	struct pap11xx_data *ts = i2c_get_clientdata(client);

	pr_func();

    /* Reset, same effect as de-asserting shutdown pin. */
	switch (ts->pid) {
	case PID_PAP1110:
		reg_write(PAP11XX_REG_SHUTDOWN, PAP11XX_RESUME);
		break;
	default:
		pr_err(MODULE_NAME "%s: unknown device.\n", __func__);
		break;
	}

    /* consider waiting for BOOT_STAT complete instead of fixed time */
    msleep(750);

    cancel_work_sync(&ts->irq_work);
    ts->power = TOUCH_POWERON;
    enable_irq(ts->client->irq);

    return 0;
}

static int pap11xx_ts_remove(struct i2c_client *client)
{
	struct pap11xx_data *ts = i2c_get_clientdata(client);

	pr_func();

	cancel_work_sync(&ts->irq_work);
	destroy_workqueue(ts->pap11xx_wq);

	if (client->irq) {
		disable_irq_nosync(client->irq);
		free_irq(client->irq, ts);
	}

	i2c_set_clientdata(client, NULL);

	input_unregister_device(ts->input);
  input_free_device(ts->input);

	pap11xx_sysfs_remove(ts);

	kfree(ts);
	return 0;
}

static const struct i2c_device_id pap11xx_ts_id[] = {
	{ PAP11XX_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pap11xx_ts_id);

static struct i2c_driver pap11xx_ts_driver = {
	.probe = pap11xx_ts_probe,
	.remove = __devexit_p(pap11xx_ts_remove),
	.id_table = pap11xx_ts_id,
	.suspend = pap11xx_ts_suspend,
	.resume = pap11xx_ts_resume,
	.driver = {
		.name = PAP11XX_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

/**
 *  Module Init.
 *
 *  Set debug level.
 *  Register driver.
 *
 */
static int __init pap11xx_ts_init(void)
{
	int ret;

	pr_func();

	drv_debug_mask = 0;
	/* BIT_SET(drv_debug_mask, DEBUG_MASK_COMMAND); */

	ret = i2c_add_driver(&pap11xx_ts_driver);
	if (ret)
		pr_err(MODULE_NAME "%s: i2c driver register fail! (%d)\n",
			__func__, ret);

	return ret;
}

/**
 *  Module Exit.
 *
 *  Unregister driver.
 *
 */
static void __exit pap11xx_ts_exit(void)
{
	pr_func();

	i2c_del_driver(&pap11xx_ts_driver);
}

module_init(pap11xx_ts_init);
module_exit(pap11xx_ts_exit);

MODULE_AUTHOR("Bert Lin <bert_lin@pixart.com>");
MODULE_DESCRIPTION("PixArt PAP11XX TouchScreen driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

