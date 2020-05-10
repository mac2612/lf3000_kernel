#
# linux/drivers/nand/nexell/Makefile
#
# rev366
# obj-$(CONFIG_MTD_NAND_NEXELL)	+= nand.o
# obj-$(CONFIG_MTD_NAND_ECC_HW)	+= nand_ecc.o
#
# pm's sandbox
obj-$(CONFIG_MTD_NAND_NEXELL)		+= nx-nand.o
nx-nand-objs				:= nand.o 
obj-$(CONFIG_MTD_NAND_LFCART)		+= lfcart.o

ifeq ($(CONFIG_MTD_NAND_MANUAL_ELP_PATH),y)
obj-$(CONFIG_MTD_NAND_ECC_HW)	+= nand_ecc_elp.o
else
obj-$(CONFIG_MTD_NAND_ECC_HW)	+= nand_ecc.o
endif
