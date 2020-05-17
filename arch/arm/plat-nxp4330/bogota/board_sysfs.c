/*
 * (C) Copyright 2012 LeapFrog Enterprises, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/device.h>
#include <linux/string.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/stat.h>
//#include <mach/adc.h>
#include <mach/soc.h>

#include <mach/board_revisions.h>
#include <asm/system.h>	/* system_rev global */
#include <mach/platform_id.h>
#include <plat/lf3000_lcd.h>
#include <linux/lf3000/gpio.h>

/* SESTERS, just for testing data dump.  Add Kconfig entry when ready */
#define CONFIG_I2S_DUMP_DATA

#define ADC_LCD_CHAN		0		/* adc channel */
#define ADC_TIMEOUT			1000	/* msec */

struct lcd_entry_t {
	int adc;				/* ADC sense */
	const char* name;		/* ID string */
};

static struct lcd_entry_t lcd_table[] = {
	{	314, 	"SDS-1",	},
	{	352, 	"SDS-2",	},
	{	392, 	"SDS-3",	},
	{	433,	"SDS-4",	},
	{	475,	"K&D-1",	},
	{	512,	"K&D-2",	},
	{	544, 	"K&D-3",	},
	{	592, 	"K&D-4",	},
	{	627,	"Arima-1",	},
	{	661,	"Arima-2",	},
	{	694,	"Arima-3",	},
	{	725,	"Arima-4",	},
	{	759,	"GP-1", 	},
	{	791,	"GP-2", 	},
	{	824,	"GP-3", 	},
	{	853,	"GP-4", 	},
	{	879,	"NVD-1",	},
	{	906,	"NVD-2",	},
	{	929,	"NVD-3",	},
	{	951,	"NVD-4",	}
};

#define NUM_LCD_ENTRIES		(sizeof(lcd_table) / sizeof(lcd_table[0]))
#define ADC_MIN_SENSE  		(lcd_table[0].adc)
#define ADC_MAX_SENSE  		(lcd_table[NUM_LCD_ENTRIES-1].adc)
#define ADC_DELTA_SENSE		((ADC_MAX_SENSE - ADC_MIN_SENSE) / NUM_LCD_ENTRIES)

/******************************************************************************/

/* Protect NOR from accidental erasure:
 * Create an address threshold; addresses LOWER than this address are
 *   blocked from erase or write commands to the nor (will return -EPERM).
 * By default, set the threshold up high, to protect everything in NOR
 */

/******************************************************************************/

u32 nor_write_addr_threshold = 0x7fffffff;

static ssize_t show_nor_write_addr_threshold(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	return sprintf (buf, "0x%08x\n", nor_write_addr_threshold);
}

static ssize_t set_nor_write_addr_threshold(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	if(sscanf(buf, "0x%x", &nor_write_addr_threshold) != 1)
		if(sscanf(buf, "%d", &nor_write_addr_threshold) != 1)
			return -EINVAL;
	return count;
}

static DEVICE_ATTR(nor_write_addr_threshold, S_IRUGO | S_IWUGO, show_nor_write_addr_threshold, set_nor_write_addr_threshold);

/*
 * determine if system can be powered from USB device
 */
bool have_usb_power_option(void)
{
	switch(system_rev) {
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF3000_BOARD_BOGOTA:
		case LF3000_BOARD_BOGOTA_EXP_1:
		case LF3000_BOARD_BOGOTA_EXP_2:
		case LF3000_BOARD_BOGOTA_EXP_3:
		case LF3000_BOARD_BOGOTA_EXP_4:
		case LF3000_BOARD_BOGOTA_EXP_5:
		case LF3000_BOARD_BOGOTA_EXP_6:
		case LF3000_BOARD_CABO:
		case LF3000_BOARD_R3K:
		case LF3000_BOARD_XANADU:
		case LF3000_BOARD_XANADU_TI:
		case LF3000_BOARD_XANADU_TI_SS1:
		case LF3000_BOARD_XANADU_TI_SS2:
			return(1);
			break;

		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_PP:
		case LF2000_BOARD_VTK:
		case LF2000_BOARD_UNKNOWN:
		default:
			break;
	}
	return(0);
}

EXPORT_SYMBOL(have_usb_power_option);

enum lf2000_leapfrog_platform get_leapfrog_platform(void)
{
	switch(system_rev)
	{
		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_PP:
		case LF2000_BOARD_LUCY_CIP:
			return LUCY;
		
		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		// case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF2000_BOARD_VALENCIA_CIP:
			return VALENCIA;

		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
		case LF3000_BOARD_R3K:
			return RIO;

		case LF3000_BOARD_CABO:
			return CABO;

		case LF3000_BOARD_GLASGOW_ALPHA:
		case LF3000_BOARD_GLASGOW_BETA:
		case LF3000_BOARD_GLASGOW_FEP_984_666:
		case LF3000_BOARD_GLASGOW_FEP_984_800:
		case LF3000_BOARD_GLASGOW_TUNE_DDR1:
		case LF3000_BOARD_GLASGOW_TUNE_DDR2:
		case LF3000_BOARD_GLASGOW_TUNE_DDR3:
		case LF3000_BOARD_GLASGOW_TUNE_DDR4:
		case LF3000_BOARD_GLASGOW_TUNE_GPU1:
		case LF3000_BOARD_GLASGOW_TUNE_GPU2:
		case LF3000_BOARD_GLASGOW_TUNE_GPU3:
			return GLASGOW;

		case LF3000_BOARD_XANADU:
		case LF3000_BOARD_XANADU_TI:
		case LF3000_BOARD_XANADU_TI_SS1:
		case LF3000_BOARD_XANADU_TI_SS2:
			return XANADU;
		
		case LF3000_BOARD_BOGOTA:
		case LF3000_BOARD_BOGOTA_EXP_1:
		case LF3000_BOARD_BOGOTA_EXP_2:
		case LF3000_BOARD_BOGOTA_EXP_3:
		case LF3000_BOARD_BOGOTA_EXP_4:
		case LF3000_BOARD_BOGOTA_EXP_5:
		case LF3000_BOARD_BOGOTA_EXP_6:
			return BOGOTA;

		default:
#if   defined(CONFIG_PLAT_NXP4330_BOGOTA)
			return BOGOTA;
#elif defined(CONFIG_PLAT_NXP4330_CABO)
			return CABO;
#elif defined(CONFIG_PLAT_NXP4330_R3K)
			return RIO;
#elif defined(CONFIG_PLAT_NXP4330_XANADU)
			return XANADU;
#else
			return UNKNOWN;
#endif
	}
	return UNKNOWN;
}
EXPORT_SYMBOL(get_leapfrog_platform);

bool is_board_lucy(void)
{
	switch(system_rev)
	{
		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_PP:
		case LF2000_BOARD_LUCY_CIP:
			return 1;

		default:
			break;
	}
	return 0;
}
EXPORT_SYMBOL(is_board_lucy);

static ssize_t sysfs_show_board_id(struct device *dev,
                                   struct device_attribute* attr,
                                   char* buf)
{
	return sprintf(buf, "0x%X\n", system_rev);
}

static DEVICE_ATTR(system_rev, S_IRUGO, sysfs_show_board_id, NULL);

/*
 * Return screen type
 *  Note revision suffix in name
 */
inline int get_median3(int a, int b, int c)
{
	if (a<b) {
		// a X Y   or   c a b
		if (a<c)
			return b<c ? b : c;
		else
			// 2 0 1
			return a;
	}
	else {
		// b X Y   or   c b a
		if (b<c)
			return a<c ? a : c;
		else
			return b;
	}
}

enum camera_type_t get_rear_camera_type(void)
{
	enum camera_type_t type = CAMERA_NONE;

	switch(system_rev)
	{
		case LF2000_BOARD_VTK:
			type = CAMERA_SR300PC10;
			break;

		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_PP:
		case LF2000_BOARD_LUCY_CIP:
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_CIP:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
			type = CAMERA_HI253;
			break;
	}
	return type;
}
EXPORT_SYMBOL(get_rear_camera_type);

int get_rear_camera_i2c_adapter_id(void)
{
	int id = -1;

	switch(system_rev)
	{
		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_PP:
			id = 0;
			break;
		case LF2000_BOARD_VTK:
		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_CIP:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
			id = 1;
			break;
	}
	return id;
}
EXPORT_SYMBOL(get_rear_camera_i2c_adapter_id);

enum camera_type_t get_front_camera_type(void)
{
	enum camera_type_t type = CAMERA_NONE;

	switch(system_rev)
	{
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_CIP:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
			type = CAMERA_HI253;
			break;
	}
	return type;
}
EXPORT_SYMBOL(get_front_camera_type);

int get_front_camera_i2c_adapter_id(void)
{
	int id = -1;

	switch(system_rev)
	{
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
			id = 1;
			break;
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_CIP:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
			id = 0;
			break;
	}
	return id;
}
EXPORT_SYMBOL(get_front_camera_i2c_adapter_id);

unsigned int get_camera_enable_delay_ms(void)
{
	unsigned int delay = 10;

	/*
	 * Datasheet calls for 10 ms.  Valencia FEP has an addtional
	 * hardware delay of 50 ms on CHIP_ENABLEB.
	 */
	switch(system_rev)
	{
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_CIP:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
			delay = 60;
			break;
	}
	return delay;
}
EXPORT_SYMBOL(get_camera_enable_delay_ms);

static const char* lcd_mfg = NULL;
const char* get_lcd_mfg(void)
{
	unsigned short lcd_sense[4] = {0, 0, 0, 0};
	unsigned short lcd_index = 0;
	unsigned short lcd_start = 0, i;

#if 1	//FIXME: sesters, leave out until ADC implemented
	soc_adc_attach();
	lcd_sense[0] = soc_adc_read(ADC_LCD_CHAN, ADC_TIMEOUT);
	lcd_sense[1] = soc_adc_read(ADC_LCD_CHAN, ADC_TIMEOUT);
	lcd_sense[2] = soc_adc_read(ADC_LCD_CHAN, ADC_TIMEOUT);
	lcd_sense[3] = soc_adc_read(ADC_LCD_CHAN, ADC_TIMEOUT);
	lcd_sense[0] = get_median3(lcd_sense[1], lcd_sense[2], lcd_sense[3]);
	soc_adc_detach();
#endif

	/* ADC table entries are 10-bit per spec */
	lcd_sense[0] >>= 2;

	lcd_start = lcd_sense[0] / ADC_DELTA_SENSE;
	/* Take care of the condition when start slot is not close to zero (truncated at front) */
	if (lcd_start > NUM_LCD_ENTRIES-1) {
		lcd_start = NUM_LCD_ENTRIES-1;
	}
	/* scan ADC table from approximate start index */
	if (lcd_sense[0] > lcd_table[lcd_start].adc) {
		for (i = lcd_start; i < NUM_LCD_ENTRIES; i++) {
			int adc_delta_sense = (i < NUM_LCD_ENTRIES-1) ? lcd_table[i+1].adc - lcd_table[i].adc : ADC_DELTA_SENSE;
			if (lcd_sense[0] <= lcd_table[i].adc + (adc_delta_sense >> 1))
				break;
		}
	} else {
		for (i = lcd_start; i > 0; i--) {
			int adc_delta_sense = (i > 0) ? lcd_table[i].adc - lcd_table[i-1].adc : ADC_DELTA_SENSE;
			if (lcd_sense[0] >= lcd_table[i].adc - (adc_delta_sense >> 1))
				break;
		}
	}
	lcd_index = i;
	if (lcd_index < NUM_LCD_ENTRIES)
		return lcd_mfg = lcd_table[lcd_index].name;
	return lcd_mfg = "Unknown";
}
EXPORT_SYMBOL(get_lcd_mfg);

static ssize_t sysfs_show_lcd_mfg(struct device *dev,
				  struct device_attribute *attr,
				  char* buf)
{
	if (lcd_mfg == NULL)
		lcd_mfg = get_lcd_mfg();
	return sprintf(buf, "%s\n", lcd_mfg);
}

static DEVICE_ATTR(lcd_mfg, S_IRUGO, sysfs_show_lcd_mfg, NULL);

static ssize_t sysfs_show_lcd_mfg_get(struct device *dev,
				  struct device_attribute *attr,
				  char* buf)
{
	return sprintf(buf, "%s\n", get_lcd_mfg());
}

static DEVICE_ATTR(lcd_mfg_get, S_IRUGO, sysfs_show_lcd_mfg_get, NULL);

enum lf2000_lcd_size get_lcd_size(void)
{
	switch(system_rev)
	{
		case LF2000_BOARD_LUCY:
		case LF2000_BOARD_LUCY_PP:
		case LF2000_BOARD_LUCY_CIP:
			return LCD_320_240;

		case LF2000_BOARD_VALENCIA:
		case LF2000_BOARD_VALENCIA_EP:
		case LF2000_BOARD_VALENCIA_EP_8:
		case LF2000_BOARD_VALENCIA_FEP:
		case LF2000_BOARD_VALENCIA_FEP_8:
		case LF2000_BOARD_VALENCIA_CIP:
		case LF3000_BOARD_CABO:
			return LCD_480_272;

		case LF2000_BOARD_RIO_KND_800_480:
		case LF2000_BOARD_RIO_BETA_800_480:
		case LF2000_BOARD_RIO_BETA_800_480_700_400:
		case LF2000_BOARD_VALENCIA_EP_800_480:
		case LF2000_BOARD_VALENCIA_EP_800_480_8:
		case LF2000_BOARD_VALENCIA_FEP_800_480:
		case LF2000_BOARD_VALENCIA_FEP_800_480_8:
		case LF2000_BOARD_VALENCIA_KND_800_480:
		case LF2000_BOARD_VALENCIA_KND_800_480_8:
		case LF2000_BOARD_VTK:
			return LCD_800_480;

		case LF2000_BOARD_RIO:
		case LF2000_BOARD_RIO_BETA_1024_600:
		case LF2000_BOARD_RIO_BETA_1024_600_700_400:
		case LF2000_BOARD_RIO_BETA_1024_600_550_275:
		case LF2000_BOARD_RIO_EP_550_275:
		case LF2000_BOARD_RIO_EP_666_333:
		case LF2000_BOARD_RIO_EP_800_333:
		case LF2000_BOARD_RIO_EP_700_400:
		case LF2000_BOARD_RIO_EP_800_400:
		case LF2000_BOARD_RIO_FEP_800_327P67:
		case LF2000_BOARD_RIO_FEP_800_327P666:
		case LF2000_BOARD_VALENCIA_KND_1024_600:
		case LF2000_BOARD_VALENCIA_KND_1024_600_8:
		case LF3000_BOARD_R3K:
		case LF3000_BOARD_XANADU:
		case LF3000_BOARD_XANADU_TI:
		case LF3000_BOARD_XANADU_TI_SS1:
		case LF3000_BOARD_XANADU_TI_SS2:
		case LF3000_BOARD_BOGOTA:
		case LF3000_BOARD_BOGOTA_EXP_1:
		case LF3000_BOARD_BOGOTA_EXP_2:
		case LF3000_BOARD_BOGOTA_EXP_3:
		case LF3000_BOARD_BOGOTA_EXP_4:
		case LF3000_BOARD_BOGOTA_EXP_5:
		case LF3000_BOARD_BOGOTA_EXP_6:
			return LCD_1024_600;

		default:
#if   defined(CONFIG_PLAT_NXP4330_BOGOTA)
			return LCD_1024_600;
#elif defined(CONFIG_PLAT_NXP4330_CABO)
			return LCD_480_272;
#elif defined(CONFIG_PLAT_NXP4330_R3K)
			return LCD_1024_600;
#elif defined(CONFIG_PLAT_NXP4330_XANADU)
			return LCD_1024_600;
#else
			return LCD_UNKNOWN;
#endif
	}
	return LCD_UNKNOWN;
}

EXPORT_SYMBOL(get_lcd_size);

int get_lcd_type(void)
{
	switch (get_leapfrog_platform())
	{
		case LUCY:
			if (strstr(lcd_mfg, "GoWorld") || strstr(lcd_mfg, "PowerTip"))
				return LCD_HX8238;
			else if (strstr(lcd_mfg, "GiantPlus"))
				return LCD_ILI9322;
			break;
		case RIO:
		case LIMA:
			return LCD_ILI6480G2;
		case VALENCIA:
		case CABO:
		case XANADU:
			return LCD_ILI6480G2;
		case GLASGOW:
		case UNKNOWN:
		default:
			break;
	}
	return -1;
}
EXPORT_SYMBOL(get_lcd_type);

static ssize_t sysfs_show_lcd_type(struct device *dev,
				  struct device_attribute *attr,
				  char* buf)
{
	switch(get_lcd_type())
	{
		case LCD_HX8238:
			return sprintf(buf, "HX8238\n");
		case LCD_ILI9322:
			return sprintf(buf, "ILI9322\n");
		case LCD_ILI6480G2:
			return sprintf(buf, "ILI6480G2\n");
	}
	return sprintf(buf, "UNKNOWN\n");
}

static DEVICE_ATTR(lcd_type, S_IRUGO, sysfs_show_lcd_type, NULL);

static ssize_t sysfs_show_lcd_size(struct device *dev,
				  struct device_attribute *attr,
				  char* buf)
{
	switch(get_lcd_size())
	{
		case LCD_320_240:
			return sprintf(buf, "320x240\n");
		case LCD_480_272:
			return sprintf(buf, "480x272\n");
		case LCD_800_480:
			return sprintf(buf, "800x480\n");
		case LCD_1024_600:
			return sprintf(buf, "1024x600\n");
		case LCD_UNKNOWN:
			break;
	}
	if (get_leapfrog_platform() == GLASGOW)
		return sprintf(buf, "1280x720\n");
	return sprintf(buf, "UNKNOWN\n");
}

static DEVICE_ATTR(lcd_size, S_IRUGO, sysfs_show_lcd_size, NULL);

static ssize_t sysfs_show_platform(struct device *dev,
                                   struct device_attribute *attr,
                                   char* buf)
{
	switch (get_leapfrog_platform()) {
	case CABO:
		return sprintf(buf, "CABO\n");
	case GLASGOW:
		return sprintf(buf, "GLASGOW\n");
	case LIMA:
		return sprintf(buf, "LIMA\n");
	case LUCY:
		return sprintf(buf, "LUCY\n");
	case RIO:
		return sprintf(buf, "RIO\n");
	case VALENCIA:
		return sprintf(buf, "VALENCIA\n");
	case BOGOTA:
		return sprintf(buf, "BOGOTA\n");
	case XANADU:
		return sprintf(buf, "XANADU\n");
	case UNKNOWN:
	default:
		break;
	}
	return sprintf(buf, "UNKNOWN\n");
}

static DEVICE_ATTR(platform, S_IRUGO, sysfs_show_platform, NULL);

static ssize_t sysfs_show_platform_family(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
	switch(get_leapfrog_platform())
	{
		case LUCY:
			return sprintf(buf, "LEX\n");\
		case BOGOTA:
		case CABO:
		case VALENCIA:
		case XANADU:
			return sprintf(buf, "LPAD\n");
		case RIO:
		case LIMA:
			return sprintf(buf, "RIO\n");
		case GLASGOW:
			return sprintf(buf, "GLASGOW\n");
		case UNKNOWN:
		default:
			break;
	}
	return sprintf(buf, "UNKNOWN\n");
}

static DEVICE_ATTR(platform_family, S_IRUGO, sysfs_show_platform_family, NULL);

//FIXME (FMirani) : Although the sysfs options are defined and populated, we really 
// 		    are not able to dynamically change the power (WIFI_RESET) since the 
//		    WIFI module does not respond if WIFI pin is toggled dynamically. It
//		    is set to high when the GPIOs are initialized and we are not turning 
//		    it low any time

#if defined(CONFIG_BCM43143)
void bcm_wlan_power_off(int param);
void bcm_wlan_power_on(int param);
static ssize_t sysfs_show_wifi_power(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(WIFI_RESET));
}

static ssize_t sysfs_store_wifi_power(struct device *dev,
                                      struct device_attribute *attr,
			              const char *buf,
			              size_t count)
{
	int powered;
	if(sscanf(buf, "%d", &powered) != 1)
		return -EINVAL;
	if(powered)
		bcm_wlan_power_on(1);
	else
		bcm_wlan_power_off(1);
	return count;
}

static DEVICE_ATTR(wifi_power, S_IRUGO | S_IWUGO, sysfs_show_wifi_power, sysfs_store_wifi_power);
#endif //CONFIG_BCM43143


#if defined(CONFIG_I2S_DUMP_DATA)
int i2s_get_dump_data_remaining(void);
void i2s_set_dump_data_count(int numbytes);
static ssize_t sysfs_show_i2s_dump_data(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
	return sprintf(buf, "%u\n", i2s_get_dump_data_remaining());
}

static ssize_t sysfs_store_i2s_dump_data(struct device *dev,
                                      struct device_attribute *attr,
			              const char *buf,
			              size_t count)
{
	int numbytes;
	if(sscanf(buf, "%d", &numbytes) != 1)
		return -EINVAL;
	i2s_set_dump_data_count(numbytes);
	return count;
}

static DEVICE_ATTR(i2s_dump_data, S_IRUGO | S_IWUGO, sysfs_show_i2s_dump_data, sysfs_store_i2s_dump_data);

#define MAX_NUM_BYTES	(1024 * 8)
char * i2s_get_output_filename(void);
int i2s_set_output_filename(const char * szFileName);

static ssize_t sysfs_show_i2s_filename(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
	return sprintf(buf, "%s\n", i2s_get_output_filename());
}

static ssize_t sysfs_store_i2s_filename(struct device *dev,
                                      struct device_attribute *attr,
			              const char *buf,
			              size_t count)
{
	char str[count];
	sscanf(buf, "%s", str);
	i2s_set_output_filename(str);
	return count;
}

static DEVICE_ATTR(i2s_filename, S_IRUGO | S_IWUGO, sysfs_show_i2s_filename, sysfs_store_i2s_filename);

#endif //CONFIG_I2S_DUMP_DATA

#if defined(CONFIG_SND_CODEC_TC94B26)

int tc94b26_set_locale(const char * locale);
char * tc94b26_get_locale(void);
static ssize_t sysfs_show_tc94b26_locale(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
	return sprintf(buf, "%s\n", tc94b26_get_locale());
}

static ssize_t sysfs_store_tc94b26_locale(struct device *dev,
                                      struct device_attribute *attr,
			              const char *buf,
			              size_t count)
{
	char str[count];
	sscanf(buf, "%s", str);
	tc94b26_set_locale(str);
	return count;
}

static DEVICE_ATTR(tc94b26_locale, S_IRUGO | S_IWUGO, sysfs_show_tc94b26_locale, sysfs_store_tc94b26_locale);

#endif //CONFIG_SND_CODEC_TC94B26

static struct bus_type board_subsys = {
	.name		= "board",
	.dev_name	= "board",
};

static int __init init_board_sysfs(void)
{
	int error = subsys_system_register(&board_subsys, NULL);
	if(!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_nor_write_addr_threshold);
	if(!error)
	error	  = device_create_file(board_subsys.dev_root, &dev_attr_lcd_mfg);
	if(!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_lcd_mfg_get);
	if(!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_lcd_type);
	if(!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_lcd_size);
	if(!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_platform);
	if(!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_platform_family);
	if(!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_system_rev);
#if defined(CONFIG_BCM43143)
	if(!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_wifi_power);
#endif //CONFIG_BCM43143
#if defined(CONFIG_I2S_DUMP_DATA)
	if (!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_i2s_dump_data);
	if (!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_i2s_filename);
#endif
#if defined(CONFIG_SND_CODEC_TC94B26)
	if (!error)
		error = device_create_file(board_subsys.dev_root, &dev_attr_tc94b26_locale);
#endif
	return error;
}

device_initcall(init_board_sysfs);

