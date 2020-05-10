#ifndef _LINUX_TU_I2C_MTOUCH_H
#define _LINUX_TU_I2C_MTOUCH_H

//Device Parameter Setting
#define TU_I2C_NAME	"bcd_ap386x"

//#define CONFIG_PM				//Power Management( Device Sleep/Resume Mode )

#define DEV_I2C_ADDRESS 0x5F	//I2C Address
#define CHANNEL_X_SIZE 16		//X Channel Count, drive lines. BCD confirmed to use 16  
//#define CHANNEL_X_SIZE 17		//X Channel Count-ref AP386X P1.0 datasheet
#define CHANNEL_Y_SIZE 10		//Y Channel Count, sense lines
#define MAX_POINT_SIZE 5		//Maximum Touch Count


//Auto Gen. Parameters
//#define REPORT_BUF_SIZE	(3+(MAX_POINT_SIZE*5))
#define REPORT_BUF_SIZE		64
#define AA_X_SIZE 		((CHANNEL_X_SIZE-1)<<6)			//Touch Resolution
#define AA_Y_SIZE 		((CHANNEL_Y_SIZE-1)<<6)			//Touch Resolution
#define AA_MAX_X		AA_X_SIZE-1	//Touch Max X
#define AA_MAX_Y		AA_Y_SIZE-1	//Touch Max Y

/*SZ org
#define AA_X_SIZE 		(480)			//Touch Resolution
#define AA_Y_SIZE 		(272)			//Touch Resolution
#define AA_MAX_X		AA_X_SIZE-1	    //Touch Max X
#define AA_MAX_Y		AA_Y_SIZE-1	    //Touch Max Y
*/

/* Conflict with mod_devicetable.h
 #define I2C_NAME_SIZE   sizeof(TU_I2C_NAME)
*/

#define POINT_STRUCT_SIZE 5			//Point Structure Size - Single




#define TU_X_AXIS 			AA_X_SIZE
#define TU_Y_AXIS 			AA_Y_SIZE



enum tu_registers {

	TU_RMOD = 0x0,		//0xb1
	TU_KEY_CODE,		//0x00
	TU_POINTS,			//Number of touch points

	TU_1_POS_X_LOW,
	TU_1_POS_X_HI,
	TU_1_POS_Y_LOW,
	TU_1_POS_Y_HI,
	TU_1_ID_STATUS,		//ID/Status

	TU_2_POS_X_LOW,
	TU_2_POS_X_HI,
	TU_2_POS_Y_LOW,
	TU_2_POS_Y_HI,
	TU_2_ID_STATUS,

	TU_3_POS_X_LOW,
	TU_3_POS_X_HI,
	TU_3_POS_Y_LOW,
	TU_3_POS_Y_HI,
	TU_3_ID_STATUS,

	TU_4_POS_X_LOW,
	TU_4_POS_X_HI,
	TU_4_POS_Y_LOW,
	TU_4_POS_Y_HI,
	TU_4_ID_STATUS,

	TU_5_POS_X_LOW,
	TU_5_POS_X_HI,
	TU_5_POS_Y_LOW,
	TU_5_POS_Y_HI,
	TU_5_ID_STATUS,

	TU_6_POS_X_LOW,
	TU_6_POS_X_HI,
	TU_6_POS_Y_LOW,
	TU_6_POS_Y_HI,
	TU_6_ID_STATUS,

	TU_7_POS_X_LOW,
	TU_7_POS_X_HI,
	TU_7_POS_Y_LOW,
	TU_7_POS_Y_HI,
	TU_7_ID_STATUS,

	TU_8_POS_X_LOW,
	TU_8_POS_X_HI,
	TU_8_POS_Y_LOW,
	TU_8_POS_Y_HI,
	TU_8_ID_STATUS,

	TU_9_POS_X_LOW,
	TU_9_POS_X_HI,
	TU_9_POS_Y_LOW,
	TU_9_POS_Y_HI,
	TU_9_ID_STATUS,

	TU_10_POS_X_LOW,
	TU_10_POS_X_HI,
	TU_10_POS_Y_LOW,
	TU_10_POS_Y_HI,
	TU_10_ID_STATUS,

	TU_DATA_SIZE
};

/*
enum tu_key_code {
	TU_ANDROID_REL = 0x0,
	TU_ANDROID_HOME,
	TU_ANDROID_BACK,
	TU_ANDROID_MENU,
	TU_ANDROID_4_RESERVED,
	TU_ANDROID_CALL,
	TU_ANDROID_6_RESERVED,
	TU_ANDROID_VOL_UP,
	TU_ANDROID_VOL_DOWN,
};
*/

enum tu_pwr_mode {

	TU_PWR_ACT = 0x0,
	TU_PWR_SLEEP,
	TU_PWR_DSLEEP,
	TU_PWR_FREEZE,
};

enum tu_int_mode {

	TU_INT_PERIOD = 0x0,
	TU_INT_FMOV,
	TU_INT_FTOUCH,
};

enum tu_int_trig {

	TU_INT_TRIG_LOW = 0x0,
	TU_INT_TRIG_HI,
};

struct tu_platform_data {
	/* add platform dependent data here */
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int reset_gpio;

	int gpio_reset;
	// .reset_polarity	= 0,
	int reset_cfg;	// Active low

};


enum tu_key_code {

	TOUCH_KEY_REL = 0x0,
	TOUCH_KEY_HOME,
	TOUCH_KEY_BACK,
	TOUCH_KEY_MENU,
	TU_ANDROID_4_RESERVED,
	TU_ANDROID_CALL,
	TU_ANDROID_6_RESERVED,
	TU_ANDROID_VOL_UP,
	TU_ANDROID_VOL_DOWN,
};

#define	COMMAND_COUNT		5
#define	NORM_CMD_LENG		4

#ifdef CONFIG_PM
static u_int8_t command_list[COMMAND_COUNT][NORM_CMD_LENG] = 
{
	{0x0E, 0x13, 0x00, 0x00},	//sleep
	{0x0E, 0x01, 0x00, 0x00},	//Resume
	{0x0E, 0x03, 0x00, 0x00},	//Disable Touch
	{0x0E, 0x01, 0x00, 0x00},	//Enable Touch
	{0x0E, 0x12, 0x00, 0x00}	//Chip Reset
};
#endif

#endif 	/* _LINUX_TU_I2C_MTOUCH_H */
