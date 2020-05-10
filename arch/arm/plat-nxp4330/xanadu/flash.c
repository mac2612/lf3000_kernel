
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/io.h>
#include <linux/crc32.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/amba/pl022.h>
#include <asm/system_info.h>

#include <cfg_main.h>

#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/devices.h>
#include <mach/nxp4330.h>
#include <mach/soc.h>
#include <mach/platform.h>

/*------------------------------------------------------------------------------
 * NOR Information
 */
#define	LF2000_NOR_FLASH_BASE_LOW	0x00000000
#define	LF2000_NOR_FLASH_BASE_HIGH0	0x34000000
#define	LF2000_NOR_FLASH_BASE_HIGH1	0x04000000

#define	LF2000_NOR_FLASH_SIZE			( 512 * 1024 )
#define	LF2000_NOR_MFGDATA_SIZE		( 4096 )
#define	LF2000_NOR_BOOT_SIZE			( LF2000_NOR_FLASH_SIZE - 2*LF2000_NOR_MFGDATA_SIZE )

static struct mtd_partition flash_parts[] = {
	{
		.name           = "NOR_Boot",
		.offset         = 0x00000000,
		.size           = LF2000_NOR_BOOT_SIZE,
	},
	{
		.name           = "MfgData0",
		.offset         = LF2000_NOR_BOOT_SIZE,
		.size           = LF2000_NOR_MFGDATA_SIZE,
	},
	{
		.name           = "MfgData1",
		.offset         = LF2000_NOR_BOOT_SIZE + LF2000_NOR_MFGDATA_SIZE,
		.size           = LF2000_NOR_MFGDATA_SIZE,
	},
};

static struct physmap_flash_data flash_data = {
	.width		= 1,		/* 8-bit */
	.nr_parts		= ARRAY_SIZE(flash_parts),
	.parts		= flash_parts,
};

static struct resource nor_flash_resources[] = {
	{
		.start		= LF2000_NOR_FLASH_BASE_LOW,
		.end			= LF2000_NOR_FLASH_BASE_LOW + LF2000_NOR_FLASH_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= LF2000_NOR_FLASH_BASE_HIGH0,
		.end			= LF2000_NOR_FLASH_BASE_HIGH0 + LF2000_NOR_FLASH_SIZE - 1,
		.flags		= IORESOURCE_MEM,
	}
};

static struct platform_device nor_flash_device = {
	.name		= "physmap-flash",
	.id		= 1,
	.resource	= nor_flash_resources,
	.num_resources	= 1,
	.dev		= {
		.platform_data = &flash_data,
	}
};

static struct platform_device nor_flash_device2 = {
	.name		= "physmap-flash",
	.id		= 2,
	.resource	= &(nor_flash_resources[1]),
	.num_resources	= 1,
	.dev		= {
		.platform_data = &flash_data,
	}
};

#if defined(CONFIG_MTD_M25P80)

static struct flash_platform_data nxp4330_spi_slave_data = {
	.type		= "pm25ld040",
	.nr_parts	= ARRAY_SIZE(flash_parts),
	.parts		= flash_parts,
};

static void spi0_cs(u32 chipselect)
{
	#if (CFG_SPI0_CS_GPIO_MODE)
    if(nxp_soc_gpio_get_io_func( CFG_SPI0_CS )!= nxp_soc_gpio_get_altnum( CFG_SPI0_CS)) {
        nxp_soc_gpio_set_io_func( CFG_SPI0_CS, nxp_soc_gpio_get_altnum( CFG_SPI0_CS));
        nxp_soc_gpio_set_io_dir( CFG_SPI0_CS,1);
    }
    nxp_soc_gpio_set_out_value(  CFG_SPI0_CS , chipselect);
#else
    ;
#endif
}
static struct pl022_config_chip spi0_flash_info = {
    /* available POLLING_TRANSFER, INTERRUPT_TRANSFER, DMA_TRANSFER */
    .com_mode = POLLING_TRANSFER,
    .iface = SSP_INTERFACE_MOTOROLA_SPI,
    /* We can only act as master but SSP_SLAVE is possible in theory */
    .hierarchy = SSP_MASTER,
    /* 0 = drive TX even as slave, 1 = do not drive TX as slave */
    .slave_tx_disable = 1,
    .rx_lev_trig = SSP_RX_4_OR_MORE_ELEM,
    .tx_lev_trig = SSP_TX_4_OR_MORE_EMPTY_LOC,
    .ctrl_len = SSP_BITS_8,
    .wait_state = SSP_MWIRE_WAIT_ZERO,
    .duplex = SSP_MICROWIRE_CHANNEL_FULL_DUPLEX,
    /*
     * This is where you insert a call to a function to enable CS
     * (usually GPIO) for a certain chip.
     */
#if (CFG_SPI0_CS_GPIO_MODE)
    .cs_control = spi0_cs,
#endif
    .clkdelay = SSP_FEEDBACK_CLK_DELAY_1T,

};

static struct spi_board_info spi_flash_board_info[] __initdata = {
	{
		.modalias	= "m25p80",
		.platform_data	= &nxp4330_spi_slave_data,
		.max_speed_hz	= 20000000,
		.irq		= IRQ_PHY_SSP0,
		.bus_num	= 0,
		.chip_select	= 1,
        .controller_data = &spi0_flash_info,
        .mode            = SPI_MODE_0,
	},
};

#endif

static int __init lf2000_flash_init(void)
{
		printk(KERN_CRIT "Initializing NOR\n");
#if 0	// FIXME: sesters, removed parallel nor test for testing
		platform_device_register(&nor_flash_device);
		platform_device_register(&nor_flash_device2);
#endif

#if defined(CONFIG_MTD_M25P80)
	spi_register_board_info(spi_flash_board_info, ARRAY_SIZE(spi_flash_board_info));
	printk(KERN_INFO "plat: register pflash\n");
#endif

	return 0;
}

device_initcall(lf2000_flash_init);

