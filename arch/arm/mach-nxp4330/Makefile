#
# Copyright 2009 Nexell Co.
#
# Makefile for the NXP4330

obj-$(CONFIG_ARCH_NXP4330) := cpu.o nxp4330.o timer.o irq.o clock.o	devices.o

obj-$(CONFIG_ARCH_NXP4330) 			+= soc/
obj-$(CONFIG_HOTPLUG_CPU)			+= hotplug.o
obj-$(CONFIG_SMP)					+= platsmp.o headsmp.o
obj-$(CONFIG_PM)					+= pm.o lldebug.o
obj-$(CONFIG_ARM_NXP4330_CPUFREQ)   += cpufreq.o cpufreq-limit.o
obj-$(CONFIG_ARM_NXP4330_CPUFREQ_BY_RESOURCE)   += cpufreq_resource.o
obj-$(CONFIG_MMC_DW)				+= dev-mmc.o
obj-$(CONFIG_HAVE_PWM)				+= pwm.o
obj-$(CONFIG_CMA)					+= cma_reserve_mem.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE)	+= ram_con_dev.o
obj-y								+= nxp-dfs-bclk.o

obj-$(CONFIG_KEYBOARD_GPIO)			+=  keyboard.o
obj-$(CONFIG_GPIO_LEAPFROG)			+= gpio/
obj-$(CONFIG_SOC_LFP100)		+= lfp100.o
obj-$(CONFIG_BQ24250_CHARGER)		+= bq24250_charger.o
obj-$(CONFIG_TC7734_PMIC)			+= tc7734.o

#===============================================================================
# Support prototype
#===============================================================================
PROTOTYPE := prototype
MODULES	  := module
BASEDIR	  := base

include $(srctree)/arch/arm/mach-nxp4330/$(PROTOTYPE).mk

KBUILD_CFLAGS += -I$(srctree)/$(MACHINE)$(PROTOTYPE)/$(BASEDIR)	\
				 -I$(srctree)/$(MACHINE)$(PROTOTYPE)/$(MODULES)	\
				 -I$(srctree)/$(MACHINE)$(PROTOTYPE)/$(MODULES)

ifeq ($(CONFIG_PROTOTYPE_RELEASE),y)
KBUILD_CFLAGS += -D__LINUX__ -D__PRINTK__ -DNX_RELEASE
else
KBUILD_CFLAGS += -D__LINUX__ -D__PRINTK__ -DNX_DEBUG
endif

#===============================================================================
#	Build options for nexell soc
#===============================================================================
KBUILD_CFLAGS += -mtune=cortex-a9 -mfpu=vfp -mfloat-abi=softfp
# -mfpu=vfpv3
