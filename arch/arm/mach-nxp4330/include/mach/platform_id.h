
#ifndef NXP3200_PLATFORM_ID_H_
#define NXP3200_PLATFORM_ID_H_

enum lf2000_leapfrog_platform
{
	UNKNOWN,
	LUCY,
	VALENCIA,
	RIO,
	CABO,
	LIMA,
	GLASGOW,
	XANADU,
	BOGOTA,
	QUITO,
};

bool is_board_lucy(void);

bool have_usb_power_option(void);

enum lf2000_leapfrog_platform get_leapfrog_platform(void);

const char* get_lcd_mfg(void);

enum lcd_type_t {
	LCD_HX8238,
	LCD_ILI9322,
	LCD_ILI6480G2,
};

int get_lcd_type(void);

enum camera_type_t {
	CAMERA_NONE = 0,
	CAMERA_SR300PC10,
	CAMERA_HI253,
};

enum camera_type_t get_rear_camera_type(void);
enum camera_type_t get_front_camera_type(void);
int get_rear_camera_i2c_adapter_id(void);
int get_front_camera_i2c_adapter_id(void);
unsigned int get_camera_enable_delay_ms(void);

#endif

