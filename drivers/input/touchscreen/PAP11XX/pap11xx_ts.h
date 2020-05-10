/*
 * include/linux/input/pap11xx_ts.h
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

#ifndef __LINUX_INPUT_PAP11XX_TS_H__
#define __LINUX_INPUT_PAP11XX_TS_H__

/* bit operation - local */
#define BIT_CLEAR(REG,MASK)			((REG) &= ~(MASK))
#define BIT_SET(REG,MASK)			((REG) |=  (MASK))
#define IS_BIT_CLEAR(REG,MASK)		(((REG) & (MASK)) == 0x00)
#define IS_BIT_SET(REG,MASK)		(((REG) & (MASK)) == (MASK))

/* word transaction */
#define WORD_HI_BYTE(x)				((uint8_t)(((x) & 0xff00) >> 8))
#define WORD_LO_BYTE(x)				((uint8_t)((x) & 0x00ff))

/* debug mask */
#define DEBUG_MASK_I2C_RW			(1<<3)
#define DEBUG_MASK_COMMAND			(1<<2)
#define DEBUG_MASK_PT_STATUS		(1<<1)
#define DEBUG_MASK_MOTION_REPORT	(1<<0)

/* Device configuration */
#define PAP11XX_X_MAX				480
#define PAP11XX_Y_MAX				272
#define PAP11XX_FORCE_MAX			5000
//#define PAP11XX_FORCE_MAX			100
#define PAP11XX_AREA_MAX			3000
#define PAP11XX_FORCE_MIN			200
#define PAP11XX_AREA_MIN			60
#define PAP11XX_ID_MIN				1
#define PAP11XX_ID_MAX				(20 + 1)
#define PAP11XX_MAX_FINGER			10

/* delay time */
#define PAP11XX_DELAY_US_BETWEEN_I2C_WRITES		15
#define PAP11XX_DELAY_US_BETWEEN_FLASH_WRITES	20
#define PAP11XX_SHUTDOWN_DELAY_US				15

/* flash control */
#define UPDATE_DRIVE_SENSE	(1 << 0)
#define UPDATE_CUSTOMER_REG	(1 << 1)
#define UPDATE_CALIBRATION	(1 << 2)
#define UPDATE_OS			(1 << 3)

/* pap11xx register table */
#define PAP11XX_REG_PID						0x00
	#define PID_PAP1110							0x8A
#define PAP11XX_REG_HW_REV_ID				0x01
	#define HW_ID_ROM10							0x10
#define PAP11XX_REG_FW_REV_ID				0x02
#define PAP11XX_REG_BOOT_STAT				0x03
	#define PAP11XX_BOOT_NAV_READY				(1<<7)
	#define PAP11XX_BOOT_DL_DONE				(1<<6)
	#define PAP11XX_BOOT_DL_FAILURE				(1<<2)
	#define PAP11XX_BOOT_COMPLETED				(1<<0)
#define PAP11XX_REG_STATUS					0x04
	#define PAP11XX_STATUS_WATCHDOG				(1<<7)
	#define PAP11XX_STATUS_CHEEK				(1<<5)
	#define PAP11XX_STATUS_REPORT_READY			(1<<4)
	#define PAP11XX_STATUS_TOUCH_CHANGE			(1<<2)
	#define PAP11XX_STATUS_TOUCH				(1<<1)
	#define PAP11XX_STATUS_ERROR				(1<<0)
#define PAP11XX_REG_INT_MASK				0x05
#define PAP11XX_REG_SHUTDOWN				0x7A
	#define PAP11XX_SHUTDOWN					0xAA
	#define PAP11XX_HARD_RESET					0xBB
	#define PAP11XX_PARTIAL_RESET				0xCC
	#define PAP11XX_RESUME						0xDD

/* pap1110 register table */
#define PAP1110_REG_IODL_CTL				0x0A
	#define PAP1110_ENABLE_DL_DS				0x2C
	#define PAP1110_ENABLE_DL_FW				0x2E
	#define PAP1110_DL_BUSY_BIT					(1<<7)
#define PAP1110_REG_IODL_DATA				0x0B
#define PAP1110_REG_ERROR					0x0D
#define PAP1110_REG_ORIENTATION				0x0E
	#define PAP1110_ORIENT_FLIP_Y_AXIS			(1<<2)
	#define PAP1110_ORIENT_FLIP_X_AXIS			(1<<1)
	#define PAP1110_ORIENT_SWAP_X_Y				(1<<0)
#define PAP1110_REG_MOTIOM_REPORT_CTL		0x1E
	#define PAP1110_MOTION_ACCESS_REPORT		(1<<7)
	#define PAP1110_MOTION_DISABLE_REPORT		(1<<3)
	#define PAP1110_MOTION_SORT_BY_FORCE		(1<<0)
#define PAP1110_REG_MOTION_REPORT_DATA		0x1F
#define PAP1110_REG_FORCE_RUN_MODE			0x20
	#define PAP1110_FORCE_RUN					0xa1
#define PAP1110_REG_FLASH_CTL				0x27
	#define PAP1110_FLASH_NO_SAVE_CUST_REGS		0x02
	#define PAP1110_FLASH_START_CUST_REGS		0x42
	#define PAP1110_FLASH_SAVE_CUST_REGS		0x82
	#define PAP1110_FLASH_SAVE_DS				0x84
	#define PAP1110_FLASH_SAVE_CAL				0xc0
	#define PAP1110_FLASH_BUSY_BIT				(1<<0)
#define PAP1110_REG_CRC_HI					0x32
#define PAP1110_REG_CRC_LO					0x33
#define PAP1110_REG_ROW						0x41
#define PAP1110_REG_COL						0x42
#define PAP1110_REG_HEIGHT_LO				0x44
#define PAP1110_REG_HEIGHT_HI				0x45
#define PAP1110_REG_WIDTH_LO				0x46
#define PAP1110_REG_WIDTH_HI				0x47
#define PAP1110_REG_FLASH_CRC_CTL			0x4E
	#define PAP1110_FLASH_CRC_CTL_BUSY			(1<<0)
	#define PAP1110_FLASH_CRC_CTL_CUSTREG		(1<<1)
	#define PAP1110_FLASH_CRC_CTL_DS			(1<<2)
	#define PAP1110_FLASH_CRC_CTL_OS			(1<<4)
	#define PAP1110_FLASH_CRC_CTL_CAL			(1<<6)
#define PAP1110_REG_FLASH_ENABLE			0x7C
	#define PAP1110_ENABLE_FLASH				0xFE
#define PAP1110_REG_WD						0x7D
	#define PAP1110_WD_DISABLE					0xAD
	#define PAP1110_WD_ENABLE					0x00

#define PAP1110_ROM_REG_FLASH_CTL			0x34
	#define PAP1110_FLASH_ERASE					(1<<3)

/* report status */
#define MOTION_REPORT_STATUS		(1<<7)


/* sysfs operations */
enum {
	SYSFS_NULL = 0,
	SYSFS_READ = 1,
	SYSFS_WRITE,
	SYSFS_BURST,
	SYSFS_DISABLE,
	SYSFS_ENABLE,
	SYSFS_READ3N = 6,
	SYSFS_VERSION,
	SYSFS_WRITE_READ,
	SYSFS_FIRMWARE,
	SYSFS_KDBGLEVEL,
	SYSFS_GPIO_CTRL = 11,
	SYSFS_READ2N,
};

/* wait for boot status */
enum {
	BOOT_COMPLETE,
	BOOT_NAV_READY,
};

/* reset event */
enum {
	POWER_HARD_RESET,
	POWER_PARTIAL_RESET,
};

/* power status */
enum {
    TOUCH_POWERON,
    TOUCH_POWEROFF,
    TOUCH_UPDATE
};

/* flash blocks */
enum {
	FLASH_BLOCK_CAL,
	FLASH_BLOCK_DS,
	FLASH_BLOCK_CR,
	FLASH_BLOCK_OS,
};

/* finger tracking */
enum {
	FINGER_NO_TOUCH,
	FINGER_TOUCH,
	FINGER_RELEASE_TO_TOUCH,
	FINGER_TOUCH_TO_TOUCH,
	FINGER_TOUCH_TO_RELEASE,
};

enum prod_type {
	PRODUCT_TYPE_MULTITOUCH,
	PRODUCT_TYPE_TOUCHPAD,
};


struct sysfs_data {
	uint8_t command;
	uint8_t reg[3];
	uint8_t data;
	uint8_t val0;
	uint8_t val1;
	uint16_t number;
};

#pragma pack(1)
struct point_data {
	uint8_t id;
	uint16_t x;
	uint16_t y;
	uint16_t force;
	uint16_t area;
};

struct touch_report {
	uint8_t status;
	uint8_t total_touch;
	struct point_data point_data[PAP11XX_MAX_FINGER];
};
#pragma pack()

/**
 * struct pap11xx_data
 * @gpio_irq: gpio pin control -> this pin is used to control the
 *	interrupt pin
 *
 */
struct pap11xx_data {
	uint8_t prod_type;
	uint8_t fw_update;
	uint8_t hw_check;
	uint8_t power;
	uint8_t pid;
	uint32_t gpio_irq;
	struct workqueue_struct *pap11xx_wq;
	struct i2c_client *client;
    struct input_dev *input;
    struct work_struct irq_work;
    struct touch_report pre_touch_report;
};
#endif

