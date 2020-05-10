/*
 *
 * tc7734.h -- TC7734 chip support
 *     access the tc7734 via getter / setter functions.
 *
 * Scott Esters <sesters@leapfrog.com>
 *
 */

#ifndef TC7734_H
#define TC7734_H

#define TC7734_NAME		"tc7734-chip"

#define TC7734_ADDR		0x4E

#define DEBUG_I2C	0	// 0 = none, 1 = tracers, 2 = no reg caching
#define TC7734_POWER_EN		0x00
#define TC7734_STATE1		0x01
#define TC7734_STATE2		0x02
#define TC7734_DEFLDO12		0x03
#define TC7734_DEFDCDC12	0x04
#define	TC7734_DEFDCDC34	0x05
#define TC7734_SEQDLY1		0x06
#define TC7734_SEQDLY2		0x07
#define TC7734_DDIM		0x08
#define TC7734_CHGCNF1		0x09
#define TC7734_CHGCNF2		0x0A
#define TC7734_CHGCNF3		0x0B
#define TC7734_CHGCNF4		0x0C
#define TC7734_CHGCNF5		0x0D
#define TC7734_CHGCNF6		0x0E
#define TC7734_STATE_CONFIG	0x0F
#define TC7734_INTMASK		0x10
#define TC7734_SYS_ERR_MASK	0x11
#define TC7734_PW_ERR_MASK	0x12
#define TC7734_LEDD_ERR_MASK	0x13
#define TC7734_PG_MASK		0x14
#define TC7734_PASSWD		0x15
#define TC7734_INT_STAT		0x20
#define TC7734_STAT1		0x21
#define TC7734_STAT2		0x22
#define TC7734_STAT3		0x23
#define TC7734_STAT4		0x24
#define TC7734_STAT5		0x25
#define TC7734_STAT6		0x26
#define TC7734_PG_MON		0x27
#define TC7734_PRODUCT_ID	0x28
#define TC7734_VALUATION_ID	0X29

#define REG_HAS_PASSWORD 0x01	/* register has password */
#define REG_IS_VOLATILE	 0x02	/* register value is volatile  */

#define TC7734_LASTREG		TC7734_VALUATION_ID	
#define TC7734_FIRSTREG		TC7734_POWER_EN
#define	TC7734_NUMREGS	(TC7734_LASTREG - TC7734_FIRSTREG + 1)

#define TC7734_I2C_ADAPTER_0	0
#define TC7734_I2C_ADAPTER_1	1
#define PASSWD_VAL 0xAB

#define TC7734_PWREN_LEDD_EN	(1 << 7)

#define TC7734_INT_PWFAULT	0x01
#define TC7734_INT_USBAC	0x02
#define TC7734_INT_PB		0x04
#define TC7734_INT_SYSFAULT	0x08
#define TC7734_INT_CHGCMP	0x10
#define TC7734_INT_CHGER	0x20
#define TC7734_INT_CHG		0x40
#define TC7734_INT_ATIL		0x80

#define TC7734_STATE2_CHG_MASK	(3 << 4)
#define TC7734_STATE2_CHG		(1 << 4)
#define TC7734_STATE2_DISCHG	(1 << 5)
#define TC7734_STATE2_SFTRST	(1 << 7)

#define TC7734_STATE2_DCP_EN	0x01
#define TC7734_STATE2_CDP_EN	0x02
#define TC7734_STATE2_SDP_EN	0x04
#define TC7734_STATE2_CHG_EN	0x08

#define TC7734_STAT3_NO_TOUT 		0<<6
#define TC7734_STAT3_PRECHG_TOUT 	1<<6
#define TC7734_STAT3_CHG_TOUT 		2<<6
#define TC7734_STAT3_CHG_WAITING	3<<6
#define TC7734_STAT3_TOUT_MASK 		0x03<<6
#define TC7734_STAT3_CGED1 			1<<5
#define TC7734_STAT3_CHARGE_COMPLETE 0x0e
#define TC7734_STAT3_CGMD_MASK		3<<1

#define TC7734_CHGCFG5_USBILIM_MASK	0xF
#define TC7734_CHGCFG5_USBILIM_DEFAULT	0
#define TC7734_CHGCFG5_USBILIM_100MA	1
#define TC7734_CHGCFG5_USBILIM_300MA	2
#define TC7734_CHGCFG5_USBILIM_400MA	3
#define TC7734_CHGCFG5_USBILIM_500MA	4
#define TC7734_CHGCFG5_USBILIM_700MA	5
#define TC7734_CHGCFG5_USBILIM_1000MA	6
#define TC7734_CHGCFG5_USBILIM_1200MA	7
#define TC7734_CHGCFG5_USBILIM_1400MA	8
#define TC7734_CHGCFG5_USBILIM_1500MA	9

#define TC7734_CHIPID_CHIP_MASK		0xF0
#define TC7734_CHIPID_REV_MASK		0x0F
#define TC7734_CHIPID_CHIP_TC7734	0x00

#define TC7734_STAT1_PB			(1 << 0)
#define TC7734_STAT1_STYP_MASK		(3 << 1)
#define TC7734_STAT1_PSDST		(1 << 3)
#define TC7734_STAT1_DTBSY		(1 << 4)
#define TC7734_STAT1_USBAC		(1 << 5)
#define TC7734_STATUS1_PB		0x01

#define TC7734_STATE_SW_STANDBY		(1 << 1)
#define TC7734_STATE_OFF		(1 << 2)

#define TC7734_ST_DISBAT	(1 << 7)

#define TC7734_BACKLIGHT_CONFIG_MASK	0x3f

/* FIXME set this value to 1 or 0 based on EE feedback 0: same phase with DCDC1 1:different phase with DCDC1*/
#define TC7734_LEDD_PS      (1<<7)
 
/* GPIO configuration */

/* Charger type based default charging current values */
enum charging_currents {
	SDP_CHARGING_CURRENT = 0, //500mA - charging is disabled for this case.
	CDP_CHARGING_CURRENT = 1, //1000mA
	DCP_CHARGING_CURRENT = 2, //1500mA
	BATTERY_NO_CURRENT = 3, // Unit is on battery
	NO_CHARGING_CURRENT = 4, //Charging is disabled
};

struct tc7734_chip {
	wait_queue_head_t wait;
	spinlock_t lock;
	struct work_struct tc7734_work;		/* task			*/
	struct workqueue_struct *tc7734_tasks;	/* workqueue		*/
	bool	have_tc7734;			/* 1 = found chip	*/
	bool	busy;				/* 1 = chip is busy	*/
	bool	have_new_vbus_draw;		/* update vbus draw */
	bool	have_new_ichrg_value;		/* update ichrg_value */
	bool    charger_status;			/* charging started or stopped */
	bool    charging_complete;		/* charging has completed */
	u8	reg_cache[TC7734_NUMREGS];	/* saved reg values	*/
	u8	reg_properties[TC7734_NUMREGS];	/* register properties	*/
	void	(*power_button_callback)(void);
	u8	iusb_value;			/* new vbus draw value  */
	u8	ichrg_value;			/* new battery charge value */
	int	maximum_register;		/* maximum register value */
	u8 tc7734_rev;              /* pmic version 1P0 or 1P1 */
	unsigned int usb_powered;   /* track USB plugged in w/ or w/o push button */
	int dcin_cnt;
};

int init_tc7734(void);
int  tc7734_get_power_button(void);
int  tc7734_have_tc7734(void);
int  tc7734_is_battery(void);
int  tc7734_is_usb_present(void);
int  tc7734_is_charging_active(void);
int  tc7734_read_reg(unsigned int reg);
void tc7734_set_power_button_callback(void (*callback)(void));
void tc7734_set_power_standby(void);
void tc7734_set_power_off(void);
int  tc7734_write_reg(unsigned int reg, unsigned int value);
void tc7734_monitor_power_button(void);
void tc7734_wled_set(unsigned int wled);
int  tc7734_set_vbus_draw(unsigned int mA);
int  tc7734_set_charger_current(unsigned int mA);
void tc7734_standby(void);
int  tc7734_get_num_registers(void);
int tc7734_is_usb_powered(void);
void tc7734_backlight_off(void);
unsigned char tc7734_get_pmic_rev(void);
enum charging_currents tc7734_get_default_charger_current(void);

#endif /* TC7734_H */
