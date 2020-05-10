/*
 *
 * Describe platform screen modules and provide information on the system's
 * screen.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __LF1000_SCREEN_H__
#define __LF1000_SCREEN_H__

struct lf1000_screen_info {
	const char *name;
	u16 xres;
	u16 yres;

	u32 clk_hz;

	/* horizonal sync */
	u16 hsw;	/* sync width */
	u16 hfp;	/* front porch */
	u16 hbp;	/* back porch */

	/* vertical sync */
	u16 vsw;	/* sync width */
	u16 vfp;	/* front porch */
	u16 vbp;	/* back porch */
};

struct lf1000_screen_info *lf1000_get_screen_info(void);
void lf1000_dpc_enable_int(bool en);
bool lf1000_dpc_int_pending(void);
void lf1000_dpc_clear_int(void);

#endif /* __LF1000_SCREEN_H__ */
