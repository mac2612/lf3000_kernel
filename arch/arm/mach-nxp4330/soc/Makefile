#
# Copyright 2009 Nexell Co.
#
# Makefile for the NXP CPU

obj-y += rsc.o gpio.o
obj-$(CONFIG_ADC_NEXELL)			+= adc.o
obj-$(CONFIG_NEXELL_SOC_PWM)		+= pwm.o
obj-$(CONFIG_NEXELL_DISPLAY)		+= display.o
obj-$(CONFIG_NEXELL_DISPLAY_LCD)	+= display_lcd.o
obj-$(CONFIG_NEXELL_DISPLAY_LVDS)	+= display_lvds.o
obj-$(CONFIG_NEXELL_DISPLAY_HDMI)	+= display_hdmi.o hdmi/
obj-$(CONFIG_NEXELL_DISPLAY_MIPI)	+= display_mipi.o
obj-$(CONFIG_NEXELL_DISPLAY_RESC)	+= display_resc.o
obj-$(CONFIG_USB_EHCI_NXP4330)		+= setup-usb-phy.o
obj-$(CONFIG_S3C_DEV_USB_HSOTG)		+= setup-usb-phy.o
obj-$(CONFIG_NXP_OUT_HDMI)			+= hdmi/
obj-$(CONFIG_NEXELL_DISPLAY_LCD)	+= spi_lcd.o
obj-$(CONFIG_NEXELL_DISPLAY_LVDS)	+= spi_lcd.o

