#
# Copyright 2009 Nexell Co.
#
# Makefile for the Nexell platform board

ifeq ($(CONFIG_PLAT_NXP4330_LYNX),y)
PLATBOARD :=lynx
endif

ifeq ($(CONFIG_PLAT_NXP4330_VTK),y)
PLATBOARD :=vtk
endif

ifeq ($(CONFIG_PLAT_NXP4330_SMARTBOX),y)
PLATBOARD :=smartbox
endif

ifeq ($(CONFIG_PLAT_NXP4330_PYXIS),y)
PLATBOARD :=pyxis
endif

ifeq ($(CONFIG_PLAT_NXP4330_W7),y)
PLATBOARD :=w7
endif

ifeq ($(CONFIG_PLAT_NXP4330_NBOX),y)
PLATBOARD :=NBOX
endif

ifeq ($(CONFIG_PLAT_NXP4330_MORPHEUS),y)
PLATBOARD :=morpheus
endif

ifeq ($(CONFIG_PLAT_NXP4330_SECRET),y)
PLATBOARD :=secret
endif

ifeq ($(CONFIG_PLAT_NXP4330_BLACKBOX),y)
PLATBOARD :=blackbox
endif

ifeq ($(CONFIG_PLAT_NXP4330_DRONE),y)
PLATBOARD :=drone
endif

ifeq ($(CONFIG_PLAT_NXP4330_KAR),y)
PLATBOARD :=kar
endif

ifeq ($(CONFIG_PLAT_NXP4330_MOON),y)
PLATBOARD :=moon
endif

ifeq ($(CONFIG_PLAT_NXP4330_BOGOTA),y)
PLATBOARD :=bogota
endif

ifeq ($(CONFIG_PLAT_NXP4330_XANADU),y)
PLATBOARD :=xanadu
endif

ifeq ($(CONFIG_PLAT_NXP4330_GLASGOW),y)
PLATBOARD :=glasgow
endif

ifeq ($(CONFIG_PLAT_NXP4330_CABO),y)
PLATBOARD :=cabo
endif

ifeq ($(CONFIG_PLAT_NXP4330_QUITO),y)
PLATBOARD :=quito
endif

#
# Build options
#

obj-y	:= $(PLATBOARD)/

KBUILD_AFLAGS 	+= -I$(srctree)/$(platdirs)common 	\
				   -I$(srctree)/$(platdirs)$(PLATBOARD)/include
KBUILD_CFLAGS 	+= -I$(srctree)/$(platdirs)common	\
				   -I$(srctree)/$(platdirs)$(PLATBOARD)/include
KBUILD_CPPFLAGS += -I$(srctree)/$(platdirs)common	\
				   -I$(srctree)/$(platdirs)$(PLATBOARD)/include



