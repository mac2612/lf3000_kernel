/*
 * Copyright (C) 2013 Nexell Co.Ltd
 * Author: BongKwan Kook <kook@nexell.co.kr>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MACH_NXP4330_OHCI_H
#define __MACH_NXP4330_OHCI_H

struct nxp4330_ohci_platdata {
	int (*phy_init)(struct platform_device *pdev, int type);
	int (*phy_exit)(struct platform_device *pdev, int type);
};

//extern void nxp4330_ohci_set_platdata(struct nxp4330_ohci_platdata *pd);

#endif /* __MACH_NXP4330_OHCI_H */
