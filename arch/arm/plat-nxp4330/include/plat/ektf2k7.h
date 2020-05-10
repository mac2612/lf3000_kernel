#ifndef _LINUX_ELAN_KTF2K7_H
#define _LINUX_ELAN_KTF2K7_H

/* Porting notes: 
    Even though data structure for 5" and 7" are the same, the X, Y
    are different between the two. To avoid problems caused by this
    inconsistency, use different file name for 7"
*/

#ifndef _LINUX_ELAN_KTF2K_H
/* Tring to make sure only one elan_ktf2k_i2c_platform_data is defined */
#include <plat/ektf2k.h>
#endif

#undef ELAN_X_MAX
#undef ELAN_Y_MAX
#define ELAN_X_MAX      960
#define ELAN_Y_MAX      1728

#define ELAN_KTF2K_NAME7 "elan-ktf2k7"

/*
struct elan_ktf2k_i2c_platform_data7 {
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
*/

#endif /* _LINUX_ELAN_KTF2K7_H */
