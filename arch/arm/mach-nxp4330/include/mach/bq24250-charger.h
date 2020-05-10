/*
 * BQ24250 Battery Charger Driver
 *
 * Copyright (C) 2012, TI Corporation
 *
 * Author: M R Swami Reddy <mr.swami.reddy@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef BQ24250_CHARGER_H
#define BQ24250_CHARGER_H

#include <linux/types.h>
#include <linux/power_supply.h>


/* Use default factory programmed value for hard/soft temperature limit */
#define BQ24250_TEMP_USE_DEFAULT		-273

/*
 * Charging enable can be controlled by software (via i2c) by
 * bq24250-charger driver or by EN pin (active low/high).
 */
enum bq24250_chg_enable {
	BQ24250_CHG_ENABLE_SW,
	BQ24250_CHG_ENABLE_PIN_ACTIVE_LOW,
	BQ24250_CHG_ENABLE_PIN_ACTIVE_HIGH,
};

/**
 * struct bq24250_charger_platform_data - platform data for BQ24250 charger
 * @battery_info: Information about the battery
 * @max_charge_current: maximum current (in uA) the battery can be charged
 * @max_charge_voltage: maximum voltage (in uV) the battery can be charged
 * @pre_charge_current: current (in uA) to use in pre-charging phase
 * @termination_current: current (in uA) used to determine when the
 *			 charging cycle terminates
 * @pre_to_fast_voltage: voltage (in uV) treshold used for transitioning to
 *			 pre-charge to fast charge mode
 * @mains_current_limit: maximum input current drawn from AC/DC input (in uA)
 * @usb_hc_current_limit: maximum input high current (in uA) drawn from USB
 *			  input
 * @chip_temp_threshold: die temperature where device starts limiting charge
 *			 current [%100 - %130] (in degree C)
 * @soft_cold_temp_limit: soft cold temperature limit [%0 - %15] (in degree C),
 *			  granularity is 5 deg C.
 * @soft_hot_temp_limit: soft hot temperature limit [%40 - %55] (in degree  C),
 *			 granularity is 5 deg C.
 * @hard_cold_temp_limit: hard cold temperature limit [%-5 - %10] (in degree C),
 *			  granularity is 5 deg C.
 * @hard_hot_temp_limit: hard hot temperature limit [%50 - %65] (in degree C),
 *			 granularity is 5 deg C.
 * @suspend_on_hard_temp_limit: suspend charging when hard limit is hit
 * @soft_temp_limit_compensation: compensation method when soft temperature
 *				  limit is hit
 * @charge_current_compensation: current (in uA) for charging compensation
 *				 current when temperature hits soft limits
 * @use_mains: AC/DC input can be used
 * @use_usb: USB input can be used
 * @use_usb_otg: USB OTG output can be used (not implemented yet)
 * @irq_gpio; GPIO number used for interrupts (%-1 if not used)
 * @enable_control: how charging enable/disable is controlled
 *		    (driver/pin controls)
 *
 * @use_main, @use_usb, and @use_usb_otg are means to enable/disable
 * hardware support for these. This is useful when we want to have for
 * example OTG charging controlled via OTG transceiver driver and not by
 * the BQ24250 hardware.
 *
 * Hard and soft temperature limit values are given as described in the
 * device data sheet and assuming NTC beta value is %3750. Even if this is
 * not the case, these values should be used. They can be mapped to the
 * corresponding NTC beta values with the help of table %2 in the data
 * sheet. So for example if NTC beta is %3375 and we want to program hard
 * hot limit to be %53 deg C, @hard_hot_temp_limit should be set to %50.
 *
 * If zero value is given in any of the current and voltage values, the
 * factory programmed default will be used. For soft/hard temperature
 * values, pass in %BQ24250_TEMP_USE_DEFAULT instead.
 */
struct bq24250_charger_platform_data {
	struct power_supply_info battery_info;
	unsigned int	max_charge_current;
	unsigned int	max_charge_voltage;
	unsigned int	pre_charge_current;
	unsigned int	termination_current;
	unsigned int	pre_to_fast_voltage;
	unsigned int	mains_current_limit;
	unsigned int	usb_hc_current_limit;
	unsigned int	chip_temp_threshold;
/*	int		soft_cold_temp_limit; */
/*	int		soft_hot_temp_limit; */
/*	int		hard_cold_temp_limit; */
/*	int		hard_hot_temp_limit; */
/*	bool		suspend_on_hard_temp_limit; */
/*	unsigned int	soft_temp_limit_compensation; */
/*	unsigned int	charge_current_compensation; */
	bool		use_mains;
	bool		use_usb;
/*	bool		use_usb_otg; */
	int             irq_gpio;
	char            name[25];
	enum bq24250_chg_enable enable_control;
};

unsigned int bq24250_get_usb_type(void);
//unsigned int bq24250_get_charge_current(void);
unsigned int bq24250_get_charging_status(void);
unsigned int bq24250_charge_enable_disable_set(bool);
unsigned int bq24250_charge_enable_disable_get(void);
#endif /* BQ24250_CHARGER_H */
