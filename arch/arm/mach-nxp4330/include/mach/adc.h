/*
 * (C) Copyright 2010
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
#ifndef __ADC_H__
#define __ADC_H__

/*------------------------------------------------------------------------------
 * 	data
 */
struct adc_info{
	unsigned char 	ch;			/* 0 ~ 5  */
	unsigned short 	level;		/* 10 bits */
};

/*------------------------------------------------------------------------------
 * 	ioctl
 */
#include "ioc_magic.h"

#define IOCTL_ADC_GET_LEVEL		_IO(IOC_NX_MAGIC, 1)


#endif /* __ADC_H__ */
