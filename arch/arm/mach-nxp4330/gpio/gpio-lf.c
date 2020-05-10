/*
 * (C) Copyright 2011
 * Daniel Lazzari Jr, LeapFrog Inc, <dlazzari@leapfrog.com>
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
 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/lf3000/gpio.h>

//For now, use the Nexell supplied functions to hook up
//the framework functionality
#include "../prototype/module/nx_gpio.h"
#include <mach/iomap.h>

static DEFINE_SPINLOCK(gpio_lock);

extern unsigned lf3000_gpio_l2p(struct gpio_chip* chip, unsigned offset);

static int lf3000_gpio_get_value (struct gpio_chip *chip, unsigned int offset)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	return (int)NX_GPIO_GetInputValue(mod, pin);
}

static void lf3000_gpio_set_value (struct gpio_chip *chip,
                                   unsigned int offset, int value)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	spin_lock(&gpio_lock);
	NX_GPIO_SetOutputValue(mod, pin, value);
	spin_unlock(&gpio_lock);
}

static int lf3000_gpio_direction_input (struct gpio_chip *chip, unsigned int offset)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	spin_lock(&gpio_lock);
	NX_GPIO_SetOutputEnable(mod, pin, 0);
	spin_unlock(&gpio_lock);
	return 0;
}

static int lf3000_gpio_direction_output (struct gpio_chip *chip,
                                         unsigned int offset, int value)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	spin_lock(&gpio_lock);
	NX_GPIO_SetOutputEnable(mod, pin, 1);
	NX_GPIO_SetOutputValue(mod, pin, value);
	spin_unlock(&gpio_lock);
	return 0;
}

static int lf3000_gpio_to_irq (struct gpio_chip *chip, unsigned int offset)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	return (pin + IRQ_GPIO_START + (mod<<5)); //Must match gpio_irq_handler
}

/*******************************************************************************
 * GPIO Framework extension
 */

static int lf3000_gpio_get_fn(struct gpio_chip* chip, unsigned offset)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	return NX_GPIO_GetPadFunction(mod, pin);
}

static int lf3000_gpio_set_fn(struct gpio_chip* chip, unsigned offset, unsigned function)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	NX_GPIO_SetPadFunction(mod, pin, function);
	return 0;
}

static NX_GPIO_PADPULL lf3000_gpio_get_pull(struct gpio_chip* chip, unsigned offset)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	return NX_GPIO_GetPullMode(mod, pin);
}

static int lf3000_gpio_set_pull(struct gpio_chip* chip, unsigned offset, NX_GPIO_PADPULL value)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	NX_GPIO_SetPullMode(mod, pin, value);
	return 0;
}

#define LF3000_GPIO_CURRENT_BASE 0xC001E100

static int lf3000_gpio_get_cur(struct gpio_chip* chip, unsigned offset)
{
	unsigned int mod, pin;
	mod = LF3000_GPIO_PHYS_PORT(offset);
	pin = offset & LF3000_GPIO_PIN_MASK;
	
	int drv1=0, drv0=0;
	int drv1_value, drv0_value;

	drv1_value = NX_GPIO_GetDRV1(mod);
	drv0_value = NX_GPIO_GetDRV0(mod);
	drv1 = (drv1_value >> pin) & 1;
	drv0 = (drv0_value >> pin) & 1;

	/* drv0 is MSB, drv1 is LSB */
	return (drv0 << 1 | drv1);
}

static int lf3000_gpio_set_cur(struct gpio_chip* chip, unsigned offset, unsigned curr)
{
	unsigned int mod, pin, reg, val, mask;

	mod  = LF3000_GPIO_PHYS_PORT(offset);
	pin  = offset & LF3000_GPIO_PIN_MASK;

	int drv1=0, drv0=0;
	int drv1_value, drv0_value;

	switch( curr )
	{
		case 0: drv0 = 0, drv1 = 0; break;
		case 1: drv0 = 0, drv1 = 1; break;
		case 2: drv0 = 1, drv1 = 0; break;
		case 3: drv0 = 1, drv1 = 1; break;
		default : drv0 =0; drv1=0;  break;
	}

	drv0_value  = NX_GPIO_GetDRV0(mod);
	drv0_value &= ~( (U32)    1 << pin);
	drv0_value |=  ( (U32) drv0 << pin);

	drv1_value  = NX_GPIO_GetDRV1(mod);
	drv1_value &= ~( (U32)    1 << pin);
	drv1_value |=  ( (U32) drv1 << pin);

	NX_GPIO_SetDRV0 ( mod, drv0_value );
	NX_GPIO_SetDRV1 ( mod, drv1_value );

	return 0;
}

/* We use 2 chips so that we can have lots of room for logical pins
 * and not reserve pins we aren't actually using. Note that the
 * logical chip is just a virtual mapping of the physical pins
 */
static struct gpio_chip lf3000_virtual_gpiochip = {
	.label			= "lf3000_virtual_gpio",
	.to_phys		= lf3000_gpio_l2p,
	.base			= 0,
	.ngpio			= GPIO_NUMBER_VALUES,
	.is_virtual		= 1,
};

static struct gpio_chip lf3000_physical_gpiochip = {
	.label			= "lf3000_physical_gpio",
	.direction_input	= lf3000_gpio_direction_input,
	.get			= lf3000_gpio_get_value,
	.direction_output	= lf3000_gpio_direction_output,
	.set			= lf3000_gpio_set_value,
	.to_irq			= lf3000_gpio_to_irq,
	.set_function		= lf3000_gpio_set_fn,
	.get_function		= lf3000_gpio_get_fn,
	.set_pull		= lf3000_gpio_set_pull,
	.get_pull		= lf3000_gpio_get_pull,
	.set_current		= lf3000_gpio_set_cur,
	.get_current		= lf3000_gpio_get_cur,
	.base			= LF3000_GPIO_PHYS,
	.ngpio			= 160,
	.can_sleep		= 0,
};

extern void lf3000_gpio_init_map(void);
void __init lf3000_gpio_init(void)
{
	spin_lock_init(&gpio_lock);
	lf3000_gpio_init_map();
	gpiochip_add(&lf3000_virtual_gpiochip);
	gpiochip_add(&lf3000_physical_gpiochip);
}



