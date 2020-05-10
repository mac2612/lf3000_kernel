/*
 * Copyright (C) 2013 Nexell Co.Ltd
 * Author: BongKwan Kook <kook@nexell.co.kr>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MACH_NXP4330_EHCI_H
#define __MACH_NXP4330_EHCI_H

struct nxp4330_ehci_platdata {
	int (*phy_init)(struct platform_device *pdev, int type);
	int (*phy_exit)(struct platform_device *pdev, int type);
	int (*hsic_phy_pwr_on)(struct platform_device *pdev, bool on);
	int resume_delay_time;	/* unit ms, more than 100 ms */
};


#endif /* __MACH_NXP4330_EHCI_H */
