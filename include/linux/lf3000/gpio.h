#ifndef _LF3000_GPIO_H_
#define _LF3000_GPIO_H_

//Logical Specifiers
enum gpio_resource {
	HEADPHONE_JACK		= 0,
	LED_ENA			= 1,
	LCD_RESET		= 2,
	AUDIO_POWER		= 3,
	DPAD_UP			= 4,
	DPAD_DOWN		= 5,
	DPAD_RIGHT		= 6,
	DPAD_LEFT		= 7,
	BUTTON_A		= 8,
	BUTTON_B		= 9,

	SHOULDER_LEFT		= 10,
	SHOULDER_RIGHT		= 11,
	BUTTON_HOME		= 12,
	BUTTON_HINT		= 13,
	BUTTON_PAUSE		= 14,
	BUTTON_BRIGHTNESS	= 15,
	BUTTON_VOLUMEUP		= 16,
	BUTTON_VOLUMEDOWN	= 17,
	CARTRIDGE_DETECT	= 18,
	TOUCHSCREEN_X1		= 19,

	TOUCHSCREEN_Y1		= 20,
	TOUCHSCREEN_X2		= 21,
	TOUCHSCREEN_Y2		= 22,
	BUTTON_RED		= 23,
	EXT_POWER		= 24,
	BUTTON_ESC		= 25,
	DOCK_POWER		= 26,
	BATTERY_PACK		= 27,
	SD1_POWER		= 28,
	LFP100_INT		= 29, // Use this for TI PMIC interrupt

	REAR_CAM_ENABLE_L	= 30,
	ACCEL_INT		= 31,
	REAR_CAM_RESET_L	= 32,
	CHG_DISABLE		= 33,
	DCIN_OVP		= 34,
	USBD_ID_SENSE		= 35,
	RT_BATTPACK_DET_L	= 36,
	POWER_ON		= 37,
	PWR_LED			= 38,
	VIBRATE_PWM		= 39,
	
	TX3			= 40,
	RX3			= 41,
	SD0_CLK			= 42,
	SD0_CMD			= 43,
	SD0_D0			= 44,
	SD0_D1			= 45,
	SD0_D2			= 46,
	SD0_D3			= 47,
	SD0_WP			= 48,
	SD0_CD			= 49,

	USB_OVP			= 50,
	WIFI_RESET		= 51,
	FRONT_CAM_ENABLE_L	= 52,
	FRONT_CAM_RESET_L	= 53,
	LCD_SPI			= 54,
	FCAM_CLK_ENA_L		= 55,
	RCAM_CLK_ENA_L		= 56,
	I2C_SCL0		= 57,
	I2C_SDA0		= 58,
	I2C_SCL1		= 59,

	I2C_SDA1		= 60,
	SDHC_DETECT0		= 61,
	SDHC_WP0		= 62,
	SDHC_DETECT1		= 63,
	SDHC_WP1		= 64,
	PENDOWN_DETECT		= 65,	// FIXME: sesters maybe these VTK touchscreen GPIOs match LUCY values above
	PENDOWN_CON		= 66,
	YMON_N			= 67,
	YMON_P			= 68,
	XMON_N			= 69,

	XMON_P			= 70,
	LCD_PCI_ENB		= 71,
	MAGNETO_DRDY		= 72,
	DPAD_LED		= 73,
	HOME_LED		= 74,
	TP_INT			= 75,
	USB_POWER_FLT_L		= 76,
	USB_OTG_PWR_ENA		= 77,
	DOCK_DETECT_L		= 78,
	USB_CHG_DETECT		= 79,

	MAGNETO_INT		= 80,
	TP_DEBUG_NEONODE 	= 81,
	NAND_CHIP_SELECT 	= 82,
	NEONODE_BSL_RX		= 83,
	NEONODE_BSL_TX		= 84,
	NEONODE_TEST		= 85,
	NEONODE_RST		= 86,
	CHG_FLT			= 87,
	USB_MUX_SEL_R		= 88,
	ETHERNET_CS		= 89,

	WIFI_HOST_WAKE		= 90,
	SYNC_BUTTON		= 91,
	BT_RESET_L		= 92,
	BT_LINK			= 93,
	CHG_INT			= 94, // Use this for charger chip interrupt, if there is a separate charger
	TC7734_INT		= 95, // Use this for Toshiba PMIC interrupt
	I2SDOUT0			= 96,
	I2SBCLK0 			= 97,
	I2SDIN0 			= 98,
	I2SLRCK0 			= 99,

	I2SMCLK0 			= 100,
	I2C_SCL2		= 101,
	I2C_SDA2		= 102,
	SPI0_CLOCK		= 103,
	SPI0_FRAME		= 104,
	SPI0_TX			= 105,
	SPI0_RX			= 106,
	I2S_SEL_BT		= 107,
	VOUT_DE_R		= 108,
	AUDIO_INT		= 109,

	AUDIO_MUTE		= 110,
	AUDIO_RST_L		= 111,
	CHG_GRN_DISABLE = 112,
	GPIO_NUMBER_VALUES	=113,
};

//Physical Specifier Helpers
#define LF3000_GPIO_PHYS	0x800
#define LF3000_GPIO_PORT_MASK	0xE0
#define LF3000_GPIO_PIN_MASK	0x1F
#define LF3000_GPIO_PHYS_PORT(x) ( (x & LF3000_GPIO_PORT_MASK) >> 5 )

#define LF3000_GPIO_PORT_NONE	(-1)
#define LF3000_GPIO_PORT_A	(LF3000_GPIO_PHYS | 0x00)
#define LF3000_GPIO_PORT_B	(LF3000_GPIO_PHYS | 0x20)
#define LF3000_GPIO_PORT_C	(LF3000_GPIO_PHYS | 0x40)
#define LF3000_GPIO_PORT_D	(LF3000_GPIO_PHYS | 0x60)
#define LF3000_GPIO_PORT_E	(LF3000_GPIO_PHYS | 0x80)

/* GPIO Functions */
#define GPIO_GPIOFN	0
#define GPIO_ALT1	1
#define GPIO_ALT2	2
#define GPIO_RESERVED	3

/* GPIO Current */
#define GPIO_CURRENT_2MA	0x0
#define GPIO_CURRENT_4MA	0x1
#define GPIO_CURRENT_6MA	0x2
#define GPIO_CURRENT_8MA	0x3

#endif //_LF3000_GPIO_H_
