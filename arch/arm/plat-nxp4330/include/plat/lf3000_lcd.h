/*
 * lf2000_lcd.h
 *
 * Copyright (C) LeapFrog Enterprises
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

#ifndef LF2000_LCD_H_
#define LF2000_LCD_H_

enum lf2000_lcd_size
{
	LCD_UNKNOWN,
	LCD_320_240,
	LCD_480_272,
	LCD_800_480,
	LCD_1024_600,
};

enum lf2000_lcd_size get_lcd_size(void);

#endif /* LF2000_LCD_H_ */

