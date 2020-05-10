#ifndef GPIO_IOCTL_H
#define GPIO_IOCTL_H

#include <linux/lf3000/gpio.h>

#define GPIO_IOC_MAGIC   'g'

/* port can be 0 for A, 1 for B, 2 for C */
struct outvalue_cmd {
	unsigned int gpio;
	unsigned char value;
};

struct outenb_cmd {
	unsigned int gpio;
	unsigned char value;
};

struct invalue_cmd {
	unsigned int gpio;
	unsigned int value;
};

/* func can be 0 for gpio, 1 for alt1, or 2 for alt2 */
struct func_cmd {
	unsigned int gpio;
	unsigned int func;
};

union gpio_cmd {
	struct outvalue_cmd outvalue;
	struct outenb_cmd outenb;
	struct invalue_cmd invalue;
	struct func_cmd func;
};

/* supported ioctls */

/* set pin output value */
#define GPIO_IOCSOUTVAL		_IOW(GPIO_IOC_MAGIC,  0, struct outvalue_cmd *)
/* set pin output enable */
#define GPIO_IOCSOUTENB		_IOW(GPIO_IOC_MAGIC,  1, struct outenb_cmd *)
/* get pin input value */
#define GPIO_IOCXINVAL		_IOWR(GPIO_IOC_MAGIC, 2, struct invalue_cmd *)
/* set pin function */
#define GPIO_IOCSFUNC		_IOW(GPIO_IOC_MAGIC,  3, struct func_cmd *)
/* get pin function */
#define GPIO_IOCXFUNC		_IOWR(GPIO_IOC_MAGIC, 4, struct func_cmd *)
/* set pin DRIVE */
#define GPIO_IOCSDRIVE		_IOW(GPIO_IOC_MAGIC,  5, struct func_cmd *)
/* get pin DRIVE */
#define GPIO_IOCXDRIVE		_IOWR(GPIO_IOC_MAGIC, 6, struct func_cmd *)
/* set pin PULLUP */
#define GPIO_IOCSPULLUP		_IOW(GPIO_IOC_MAGIC,  7, struct func_cmd *)
/* get pin DRIVE */
#define GPIO_IOCXPULLUP		_IOWR(GPIO_IOC_MAGIC, 8, struct func_cmd *)
/* get pin output enable */
#define GPIO_IOCXOUTENB		_IOW(GPIO_IOC_MAGIC,  9, struct outenb_cmd *)

#endif
