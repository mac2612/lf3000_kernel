/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/memblock.h>
#include <linux/interrupt.h>

#if defined(CONFIG_PM)
#include <mach/platform.h>
#include <mach/pm.h>

/*
 * 	PM ops
 */
#define	POLY	(0x04C11DB7L)
static inline U32 iget_fcs(U32 fcs, U32 data)
{
	register int i;
	fcs ^= data;
	for(i=0; i<32; i++)	{
		if(fcs & 0x01)
			fcs ^= POLY;
		fcs >>= 1;
	}
	return fcs;
}

#define	PM_SAVE_ADDR 	(PHYS_OFFSET + 0x1000)	/* 0x40001000  */
#define	PM_SAVE_SIZE 	(0x1000)	/* 32 K = 0x40002000 : 0x40008000 */

static unsigned int *pm_save_virt = NULL;
static unsigned int  pm_save_size = PM_SAVE_SIZE;

static void board_suspend_mark(struct suspend_mark_up *mark, int suspend)
{
	unsigned int pm_save_phys = PM_SAVE_ADDR;
	if (suspend && pm_save_virt) {
		unsigned int *virt = (unsigned int *)pm_save_virt;
		int size = sizeof(*mark)/4;
		unsigned int crc_ret = 0, i;

		virt[0] = mark->signature;
		virt[1] = mark->resume_fn;
		virt[2] = mark->save_phy_addr;
		virt[3] = mark->save_crc_ret;
		virt[4] = mark->save_phy_len;

		for(i=0; size > i; i++)
			crc_ret = iget_fcs(crc_ret, virt[i]);

		virt[5] = crc_ret;
		lldebugout("Suspend BOARD CRC [phy=0x%08x, calc=0x%08x, len=%d]\n",
			pm_save_phys, crc_ret, size);
	}
}

/* register pm ops function */
static int __init board_suspend_init(void)
{
	unsigned int pm_save_phys = PM_SAVE_ADDR;
	int ret;

	nxp_board_suspend_mark = board_suspend_mark;

	if (pm_save_size > 0x1000) {
		printk("*** FAIL: over reseve pm region max size %d (32K) ***\n", pm_save_size);
		return -EINVAL;
	}

	ret = memblock_is_region_reserved(pm_save_phys, pm_save_size);
	if (!ret)
		memblock_reserve(pm_save_phys, pm_save_size);

	ret = memblock_is_region_reserved(pm_save_phys, pm_save_size);
	if (!ret) {
		printk("*** FAIL: pm region 0x%x(%d) is not reserved ***\n",
			pm_save_phys, pm_save_size);
		return -1;
	}
 	pm_save_virt = (uint*)__phys_to_virt(pm_save_phys);
	printk("PM : Reserve phys 0x%x to virt 0x%p size %d\n",
		pm_save_phys, pm_save_virt, pm_save_size);

	return 0;
}
postcore_initcall(board_suspend_init);



#endif	/* CONFIG_PM */
