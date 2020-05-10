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
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#if defined(CONFIG_PM)
#include <mach/platform.h>
#include <mach/pm.h>

#if (1)
#define DBGOUT(msg...)		{ printk(KERN_INFO "board-pm: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif
/*
 * 	PM ops
 */
static void board_suspend_poweron(void)
{
	PM_DBGOUT("+%s\n", __func__);
	nxp_board_base_init();
	PM_DBGOUT("-%s\n", __func__);
}

static struct board_suspend_ops board_ops = {
	.poweron	= board_suspend_poweron,
};

/* register pm ops function */
static int __init board_suspend_init(void)
{
	DBGOUT("%s\n", __func__);

	nxp_board_suspend_register(&board_ops);
	return 0;
}
postcore_initcall(board_suspend_init);

#endif	/* CONFIG_PM */
