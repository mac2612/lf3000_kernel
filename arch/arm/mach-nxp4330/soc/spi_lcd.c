#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <mach/platform.h>
#include <linux/platform_device.h>

#include <mach/devices.h>
#include <mach/soc.h>
#include "display_4330.h"

#include <linux/delay.h>
#include <nx_ssp.h>

#define DEBUG_LOOPBACK 	0	// for debugging in loopback mode
#define DEBUG_READBACK  0   // for debugging register readback
#define DEBUG_BITBANG   1	// for gpio bitbang mode

static int spi_channel = 1;	// FIXME
static int gpio_port  = 4;	// FIXME
static int gpio_pin	= 15;	// FIXME

#if 0	/* 10apr14 */
static void print_0xc005c000(void)
{
	U32 * p = (U32 *)0xc005c000;
	int i;

	printk(KERN_INFO "Memory at 0xC005C000\n");
	for (i = 0; i < 3; ++i) {
		printk(KERN_INFO "%08x  %08x  %08x  %08x\n",
				*p, *(p+1), *(p+2), *(p+3));
		p += 4;
	}
}
#endif	/* 10apr14 */

void spi_lcd_init(void)
{
	printk(KERN_INFO "%s: spi_channel: %d, gpio_port:%d, gpio_pin:%d\n",
		__func__, spi_channel, gpio_port, gpio_pin);
#if 0	/* 10apr14 */
	printk(KERN_INFO "%s: Channel %d physaddr: 0x%x, ioaddr 0x%x\n",
		__func__, spi_channel, 
		(unsigned int)NX_SSP_GetPhysicalAddress(spi_channel),
		(U32)IO_ADDRESS(NX_SSP_GetPhysicalAddress(spi_channel)));
	print_0xc005c000();
#endif	/* 10apr14 */

	// SPI init module
	NX_SSP_Initialize();
	NX_SSP_SetBaseAddress(spi_channel,
			(U32)IO_ADDRESS(NX_SSP_GetPhysicalAddress(spi_channel)));
	NX_SSP_OpenModule(spi_channel);
	printk(KERN_INFO "%s: spi_channel: %d, phys=%08x, io=%08x\n",
		__func__, spi_channel, NX_SSP_GetPhysicalAddress(spi_channel), NX_SSP_GetBaseAddress(spi_channel));

//	NX_CLKGEN_Initialize();
    NX_CLKGEN_SetBaseAddress(NX_SSP_GetClockNumber(spi_channel),
    		(U32)IO_ADDRESS(NX_CLKGEN_GetPhysicalAddress(NX_SSP_GetClockNumber(spi_channel))));
	printk(KERN_INFO "%s: spi_clkgen: %d, phys=%08x, io=%08x\n",
		__func__, NX_SSP_GetClockNumber(spi_channel), NX_CLKGEN_GetPhysicalAddress(NX_SSP_GetClockNumber(spi_channel)), NX_CLKGEN_GetBaseAddress(NX_SSP_GetClockNumber(spi_channel)));
    NX_CLKGEN_SetClockBClkMode(NX_SSP_GetClockNumber(spi_channel), NX_BCLKMODE_DYNAMIC);
    NX_CLKGEN_SetClockPClkMode(NX_SSP_GetClockNumber(spi_channel), NX_PCLKMODE_ALWAYS);
    NX_CLKGEN_SetClockSource(NX_SSP_GetClockNumber(spi_channel), 0, 2);		// PLL2
    NX_CLKGEN_SetClockDivisor(NX_SSP_GetClockNumber(spi_channel), 0, 300/20); 	// FIXME: 275MHz/14 = 20MHz
    NX_CLKGEN_SetClockDivisorEnable(NX_SSP_GetClockNumber(spi_channel), CTRUE);
    NX_RSTCON_SetnRST(NX_SSP_GetResetNumber(spi_channel, 0), RSTCON_nDISABLE);
    NX_RSTCON_SetnRST(NX_SSP_GetResetNumber(spi_channel, 1), RSTCON_nDISABLE);
    NX_RSTCON_SetnRST(NX_SSP_GetResetNumber(spi_channel, 0), RSTCON_nENABLE);
    NX_RSTCON_SetnRST(NX_SSP_GetResetNumber(spi_channel, 1), RSTCON_nENABLE);

	// SPI CS auto assertion
//	NX_GPIO_SetPadFunction (gpio_port, gpio_pin, NX_GPIO_PADFUNC_2);	// FIXME
	NX_GPIO_SetPadFunction (gpio_port, gpio_pin, NX_GPIO_PADFUNC_0);	// FIXME
	NX_GPIO_SetOutputEnable(gpio_port, gpio_pin, 1);
	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 1);

	// SPI init setup
	NX_SSP_SetClockPrescaler(spi_channel, 4, 1);	// FIXME: 275MHz/14 = 20MHz?
	NX_SSP_SetDMATransmitMode(spi_channel, CFALSE); // no DMA
	NX_SSP_SetBitWidth(spi_channel, 16);			// 16-bit
	NX_SSP_SetProtocol(spi_channel, NX_SSP_PROTOCOL_SPI);// SPI
	NX_SSP_SetSPIFormat(spi_channel, NX_SSP_FORMAT_A);// format
	NX_SSP_SetSlaveMode(spi_channel, CFALSE);		// master
	NX_SSP_SetClockPolarityInvert(spi_channel, CFALSE);	// normal polarity

	// SPI enable
	NX_SSP_SetInterruptEnable(spi_channel, 0, CFALSE);
	NX_SSP_SetInterruptEnable(spi_channel, 1, CFALSE);
	NX_SSP_SetInterruptEnable(spi_channel, 2, CFALSE);
	NX_SSP_SetInterruptEnable(spi_channel, 3, CFALSE);
	NX_SSP_ClearInterruptPendingAll(spi_channel);
    NX_SSP_SetEnable(spi_channel, CFALSE);

#if DEBUG_LOOPBACK
    NX_SSP_Set_SSPCR1(spi_channel, NX_SSP_Get_SSPCR1(spi_channel) | (1<<0)); // loopback
#else
    NX_SSP_Set_SSPCR1(spi_channel, NX_SSP_Get_SSPCR1(spi_channel) & ~(1<<0)); // loopback
#endif

    printk(KERN_INFO "%s: CR0=%08x\n", __func__, NX_SSP_Get_SSPCR0(spi_channel));
    printk(KERN_INFO "%s: CR1=%08x\n", __func__, NX_SSP_Get_SSPCR1(spi_channel));
    printk(KERN_INFO "%s: SR =%08x\n", __func__, NX_SSP_Get_SSPSR(spi_channel));
}

static void spi_lcd_bitbang(u32 val, int len)
{
	int i;
	int gpio_clk = 14;	// FIXME
	int gpio_frm = 15;	// FIXME
	int gpio_txd = 19;	// FIXME

	NX_GPIO_SetOutputValue(gpio_port, gpio_frm, 0);

	for (i = 0; i < len; i++) {
		NX_GPIO_SetOutputValue(gpio_port, gpio_txd, (val >> (len-1-i)) & 1);
		NX_GPIO_SetOutputValue(gpio_port, gpio_clk, 0);
		udelay(1);
		NX_GPIO_SetOutputValue(gpio_port, gpio_clk, 1);
		udelay(1);
	}

	NX_GPIO_SetOutputValue(gpio_port, gpio_frm, 1);
}

static void spi_lcd_write(u8 reg, u8 val)
{
#if DEBUG_BITBANG
	spi_lcd_bitbang((reg<<10) | (0<<9) | val, 16);
	return;
#else
	int timeout = 100;

	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 0);
	NX_SSP_SetProtocol(spi_channel, NX_SSP_PROTOCOL_SPI);
	NX_SSP_SetBitWidth(spi_channel, 16);
	NX_SSP_SetEnable(spi_channel, CTRUE);

	// SPI LCD 16-bit register write for ILI6480G2
	NX_SSP_PutHalfWord(spi_channel, (reg<<10) | (0<<9) | val);
	while (!NX_SSP_IsTxFIFOEmpty(spi_channel) && --timeout)
		udelay(1);
	printk(KERN_INFO "%s: Tx timeout=%d\n", __func__, timeout);
#if DEBUG_LOOPBACK
	timeout = 1000;
	while (!NX_SSP_IsRxFIFOEmpty(spi_channel) && --timeout)
		udelay(1);
	printk(KERN_INFO "%s: Rx timeout=%d\n", __func__, timeout);
	printk(KERN_INFO "%s: Rx readword=%04x\n", __func__, NX_SSP_GetHalfWord(spi_channel));
#endif

	NX_SSP_SetEnable(spi_channel, CFALSE);
	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 1);
#endif
}

static void spi_lcd_write24(u8 reg, u16 val)
{
#if DEBUG_BITBANG
	spi_lcd_bitbang((0x70 << 16) | reg, 24);
	spi_lcd_bitbang((0x72 << 16) | val, 24);
	return;
#else
	int timeout = 100;

	printk(KERN_INFO "%s: %02X, %04X\n", __func__, reg, val);

	// SPI LCD 24-bit register write for HX8238
	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 0);

	NX_SSP_SetProtocol(spi_channel, NX_SSP_PROTOCOL_SPI);
	NX_SSP_SetBitWidth(spi_channel, 8);
	NX_SSP_SetEnable(spi_channel, CTRUE);

	NX_SSP_PutByte(spi_channel, 0x70);
	NX_SSP_PutByte(spi_channel, 0x00);
	NX_SSP_PutByte(spi_channel, reg);
	while (!NX_SSP_IsTxFIFOEmpty(spi_channel) && --timeout)
		udelay(0);

	printk(KERN_INFO "%s: %02X, %04X, timeout=%d\n", __func__, reg, val, timeout);
	timeout = 100;

	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 1);
	udelay(0);
	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 0);

	NX_SSP_PutByte(spi_channel, 0x72);
	NX_SSP_PutByte(spi_channel, val >> 8);
	NX_SSP_PutByte(spi_channel, val & 0xFF);
	while (!NX_SSP_IsTxFIFOEmpty(spi_channel) && --timeout)
		udelay(0);

	printk(KERN_INFO "%s: %02X, %04X, timeout=%d\n", __func__, reg, val, timeout);

	NX_SSP_SetEnable(spi_channel, CFALSE);
	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 1);
#endif
}

#if DEBUG_READBACK
static u16 spi_lcd_read24(u8 reg)
{
	u16 val = 0;
	int timeout = 1000;

	// SPI LCD 24-bit register write for HX8257
	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 0);

	NX_SSP_SetProtocol(spi_channel, NX_SSP_PROTOCOL_SPI);
	NX_SSP_SetBitWidth(spi_channel, 8);
	NX_SSP_SetEnable(spi_channel, CTRUE);

	NX_SSP_PutByte(spi_channel, 0x70);
	NX_SSP_PutByte(spi_channel, 0x00);
	NX_SSP_PutByte(spi_channel, reg);
	while (!NX_SSP_IsTxFIFOEmpty(spi_channel) && --timeout)
		udelay(0);

	printk(KERN_INFO "%s: %02X, %04X, timeout=%d\n", __func__, reg, val, timeout);

	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 1);
	udelay(0);
	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 0);

//	NX_SSP_SetProtocol(spi_channel, NX_SSP_PROTOCOL_NM);
//	NX_SSP_SetBitWidth(spi_channel, 16);

	// SPI LCD 24-bit register read for HX8257
	NX_SSP_PutByte(spi_channel, 0x73);	// read
	while (!NX_SSP_IsTxFIFOEmpty(spi_channel))
		udelay(0);
	timeout = 1000;
	while (NX_SSP_IsRxFIFOEmpty(spi_channel) && --timeout)
		udelay(0);
	val = NX_SSP_GetByte(spi_channel);
	val <<= 8;
	val |= NX_SSP_GetByte(spi_channel);

	NX_SSP_SetEnable(spi_channel, CFALSE);
	NX_GPIO_SetOutputValue(gpio_port, gpio_pin, 1);

	printk(KERN_INFO "%s: %02X, %04X, timeout=%d\n", __func__, reg, val, timeout);

	return val;
}
#endif


void spi_lcd_setup(void)
{
}

void spi_lcd_flip(int module, int flip)
{
#if defined(CONFIG_PLAT_NXP4330_CABO)
	// ILI6480G2
	spi_lcd_write(0x01, (flip) ? 0x1C : 0x1F);

	// HX8257
	spi_lcd_write24(0x01, (flip) ? 0x7846 : 0x7840);
#endif

#if defined(CONFIG_PLAT_NXP4330_BOGOTA)
	// HX8282
	spi_lcd_write(0x00, (flip) ? 0x3D : 0x0D);
#endif

#if defined(CONFIG_PLAT_NXP4330_QUITO)
	/* FIXME: may need corrections */
	// ILI6480G2
	spi_lcd_write(0x01, (flip) ? 0x1C : 0x1F);
#endif
}
