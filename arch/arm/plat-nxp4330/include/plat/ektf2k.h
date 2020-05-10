#ifndef _LINUX_ELAN_KTF2K_H
#define _LINUX_ELAN_KTF2K_H

#undef ELAN_X_MAX
#undef ELAN_Y_MAX
#define ELAN_X_MAX      1024
#define ELAN_Y_MAX      1792

#define ELAN_KTF2K_NAME "elan-ktf2k"

struct elan_ktf2k_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int reset_gpio;
    int mode_check_gpio;
	int (*power)(int on);
};

#endif /* _LINUX_ELAN_KTF2K_H */
