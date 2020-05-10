/*
 * Definitions for TC94B26 codec driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* TC94B26 configuration options */
struct tc94b26_pdata {
	int hp_detect_pol;			/* 1=headphone high impedance, debounce time set to 128 ms */
	int hp_spk_autosw_dis;		/* Unimplemented: autoswitch speaker / headphone */
};
