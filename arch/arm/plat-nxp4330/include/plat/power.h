#ifndef _LF1000_POWER_H_
#define _LF1000_POWER_H_

#if defined(CONFIG_PLAT_NXP3200_RIO) || defined(CONFIG_PLAT_NXP4330)
/* default values */
/* SP 04-16-13: If you are changing low battery threshold here, 
 * make sure you also change it in these two files to maintain it 
 * across boot, kernel and battery voltage logging.
 * Battery logger - LinuxDist_LF2000/packages/optimization/bat-logger.sh
 * Boot - nxp3200_bsp/bootloader/u-boot-2010.06/board/nxp3200/common/board.c
 */
#define MAX_BATTERY_MV	4400		/* max expected battery value	*/
#define LOW_BATTERY_MV	3600		/* low battery			*/
#define LOW_BATTERY_REPEAT_MV 25	/* repeat every 100mv drop	*/

/* Hysteresis low to normal Battery */
#define NORMAL_BATTERY_MV   (LOW_BATTERY_MV + 200)

#ifdef CONFIG_PLAT_NXP4330_BOGOTA //Bogota
//Math - values used for battery percentage calculation
#define ALPHA_DIV	32 //96 samples for 95% convergence: a 0.1 Hz, 16 minutes.
#define SCALE	128

#define	BATTERY_0	3554
#define	BATTERY_25	3687
#define	BATTERY_50	3750
#define	BATTERY_75	4015
#endif

#else
/* default values */
#define MAX_BATTERY_MV	8000		/* max expected battery value	*/
#define LOW_BATTERY_MV	4200		/* low battery			*/
#define LOW_BATTERY_REPEAT_MV 100	/* repeat every 100mv drop	*/

/* Hysteresis low to normal Battery */
#define NORMAL_BATTERY_MV   (LOW_BATTERY_MV + 400)

#endif // Rio or non-Rio

/*
 * Lower critical battery level below hardware shutoff,
 * allowing play until you die.  Can still adjust level via
 * /sys/devices/platform/xxx/critical_battery_mv interface
 */
#define CRITICAL_BATTERY_MV 0	/* critical low battery		*/

enum lf1000_power_status {
	LF1000_UNKNOWN 		= 0,
	EXTERNAL		= 1,	/* on external power */
	BATTERY			= 2,	/* on battery power */
	LOW_BATTERY		= 3,
	CRITICAL_BATTERY	= 4,
	NIMH			= 5,	/* on NIMH battery power   */
	NIMH_CHARGER		= 6,	/* in NiMH battery charger */
};

enum lf1000_power_source {
	POWER_UNKNOWN		= 0,	/* Unknown power source	*/
	POWER_OTHER		= 1,	/* Unknown power source	*/
	POWER_NIMH		= 2,	/* using NiMH battery */
	POWER_NIMH_CHARGER	= 3,	/* in NiMH battery charger */
	POWER_NIMH_EXTERNAL	= 4,	/* using NiMH with external power */
	POWER_BATTERY		= 5,	/* standard battery */
	POWER_EXTERNAL		= 6,	/* external power source */
};

/* LF2000 USB power levels */
#define LF_USB_GADGET_VBUS_NO_POWER     0       // No USB Power
#define LF_USB_GADGET_VBUS_POWER        500     // USB Power requested

#endif
