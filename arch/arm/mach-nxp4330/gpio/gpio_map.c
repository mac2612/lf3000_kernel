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

#include <linux/gpio.h>
#include <linux/lf3000/gpio.h>
#include <asm/system.h>

#include <mach/platform.h>
#include <mach/board_revisions.h>

int gpio_map[GPIO_NUMBER_VALUES];

/*
 * GPIO entry map
 * Sort entries by GPIO port / pin, accounting for all GPIO pins
 * Mark unused GPIO pins with LF3000_GPIO_PORT_NONE for housekeeping
 */
 //Placeholders

static void init_bogota(void)
{
	printk(KERN_WARNING "%s: Bogota GPIO mapping\n", __func__);

	/* GPIO Port A */
	/* NC             - pin  A0 */
	/* PVD0           - pin  A1 */
	/* PVD1           - pin  A2 */
	/* PVD2           - pin  A3 */
	/* PVD3           - pin  A4 */
	/* PVD4           - pin  A5 */
	/* PVD5           - pin  A6 */
	/* PVD6           - pin  A7 */

	/* PVD7           - pin  A8 */
	/* NC             - pin  A9 */
	/* NC             - pin A10 */
	/* NC             - pin A11 */
	/* NC             - pin A12 */
	gpio_map[I2S_SEL_BT] 		= LF3000_GPIO_PORT_A | 13;
	/* NC             - pin A14 */
	/* NC             - pin A15 */

	/* NC             - pin A16 */
	gpio_map[CHG_GRN_DISABLE] 	= LF3000_GPIO_PORT_A | 17;
	/* NC             - pin A18 */
	/* NC             - pin A19 */
	/* NC             - pin A20 */
	/* NC             - pin A21 */
	/* NC             - pin A22 */
	/* NC             - pin A23 */

	/* NC             - pin A24 */
	/* NC             - pin A25 */
	/* NC             - pin A26 */
	gpio_map[VOUT_DE_R] 		= LF3000_GPIO_PORT_A | 27;
	/* VID1_PCLK      - pin A28 */
	/* SDCLK0_R       - pin A29 */
	/* VID1_0         - pin A30 */
	/* SDCMD0_R       - pin A31 */

	/* GPIO Port B */
	/* VID1_1         - pin  B0 */
	/* SDDAT00_R      - pin  B1 */
	/* VID1_2         - pin  B2 */
	/* SDDAT01_R      - pin  B3 */
	/* VID1_3         - pin  B4 */
	/* SDDAT02_R      - pin  B5 */
	/* VID1_4         - pin  B6 */
	/* SDDAT03_R      - pin  B7 */

	/* VID1_5         - pin  B8 */
	/* VID1_6         - pin  B9 */
	/* VID1_7         - pin B10 */
	/* NAND_CLE_R     - pin B11 */
	/* NAND_ALE_R     - pin B12 */
	/* SD0            - pin B13 */
	/* NAND_RNB       - pin B14 */
	/* SD1            - pin B15 */
	
	/* NAND_NFOE_R_L  - pin B16 */
	/* SD2            - pin B17 */
	/* NAND_NFWE_R_L  - pin B18 */
	/* SD3            - pin B19 */
	/* SD4            - pin B20 */
	/* SD5            - pin B21 */
	/* SD6            - pin B22 */
	/* SD7            - pin B23 */

	gpio_map[TOUCHSCREEN_Y2]	= LF3000_GPIO_PORT_B | 24;	//FIXME CTP_RST_RTP_Y2_R
	gpio_map[TOUCHSCREEN_X2]	= LF3000_GPIO_PORT_B | 25;	//FIXME CTP_INT_RTP_X2_R
	/* NC             - pin B26 */
	/* NC             - pin B27 */
	/* NC             - pin B28 */
	gpio_map[BUTTON_VOLUMEUP]	= LF3000_GPIO_PORT_B | 29;
	gpio_map[BUTTON_VOLUMEDOWN]	= LF3000_GPIO_PORT_B | 30;
	/* NC             - pin B31 */

	/* GPIO Port C */
	/* NC             - pin  C0 */
	gpio_map[CARTRIDGE_DETECT]	= LF3000_GPIO_PORT_C | 1; //CIP_L as per the excel sheet
	/* NC             - pin  C2 */
	/* NC  	          - pin  C3 */
#if defined(CONFIG_SOC_LFP100)
	gpio_map[LFP100_INT]		= LF3000_GPIO_PORT_C | 4;
#endif
#if defined(CONFIG_TC7734_PMIC)
	gpio_map[TC7734_INT]		= LF3000_GPIO_PORT_C | 4; //PMIC_INT_L
#endif
	/* BT_HOST_CTS    - pin  C5 */
	/* BT_HOST_RTS    - pin  C6 */
	gpio_map[WIFI_RESET]		= LF3000_GPIO_PORT_C | 7;
	
	gpio_map[ACCEL_INT]		= LF3000_GPIO_PORT_C | 8;
	gpio_map[BT_RESET_L] 		= LF3000_GPIO_PORT_C | 9;
	gpio_map[WIFI_HOST_WAKE] 	= LF3000_GPIO_PORT_C | 10;
	/* NC	          - pin C11 */
	gpio_map[BT_LINK] 		= LF3000_GPIO_PORT_C | 12;
	/* NC             - pin C13 */
	gpio_map[REAR_CAM_RESET_L]	= LF3000_GPIO_PORT_C | 14; //VID0_RESET_L
	gpio_map[FRONT_CAM_ENABLE_L]	= LF3000_GPIO_PORT_C | 15; //VID1_ENA_L
	
	gpio_map[FRONT_CAM_RESET_L]	= LF3000_GPIO_PORT_C | 16; //VID1_RESET_L
	/* NC             - pin C17 */
	/* SDCLK2         - pin C18 */
	/* SDCMD2         - pin C19 */
	/* SDDAT20        - pin C20 */
	/* SDDAT21        - pin C21 */
	/* SDDAT22        - pin C22 */
	/* SDDAT23        - pin C23 */

	gpio_map[REAR_CAM_ENABLE_L] = LF3000_GPIO_PORT_C | 24; //VID0_ENA_L
	/* nSWAIT         - pin C25 */
	/* NC             - pin C26 */
	/* NC             - pin C27 */
	/* NC             - pin C28 */
	gpio_map[SPI0_CLOCK]		= LF3000_GPIO_PORT_C | 29;
	gpio_map[SPI0_FRAME]		= LF3000_GPIO_PORT_C | 30;
	gpio_map[SPI0_TX]		= LF3000_GPIO_PORT_C | 31;


	/* GPIO Port D */

	gpio_map[SPI0_RX]		= LF3000_GPIO_PORT_D | 0;
	/* MCU_PWM0       - pin  D1 */
	gpio_map[I2C_SCL0]		= LF3000_GPIO_PORT_D | 2;
	gpio_map[I2C_SDA0]		= LF3000_GPIO_PORT_D | 3;
	gpio_map[I2C_SCL1]		= LF3000_GPIO_PORT_D | 4;
	gpio_map[I2C_SDA1]		= LF3000_GPIO_PORT_D | 5;
	gpio_map[I2C_SCL2]		= LF3000_GPIO_PORT_D | 6;
	gpio_map[I2C_SDA2]		= LF3000_GPIO_PORT_D | 7;

	/* NC             - pin  D8 */
	/* I2SDOUT        - pin  D9 */
	/* I2SBCLK        - pin D10 */
	/* I2SDIN         - pin D11 */
	/* I2SSYNC        - pin D12 */
	/* I2SMCLK        - pin D13 */
	/* RX0            - pin D14 */
	/* BT_HOST_RX0    - pin D15 */

	/* NC	          - pin D16 */
	/* NC             - pin D17 */
	/* TX0            - pin D18 */
	/* BT_HOST_TX0    - pin D19 */
	gpio_map[CHG_FLT]		= LF3000_GPIO_PORT_D | 20;
	/* NC             - pin D21 */
	/* SDCLK1_R       - pin D22 */
	/* SDCMD1_R       - pin D23 */

	/* SDDAT10_R      - pin D24 */
	/* SDDAT11_R      - pin D25 */
	/* SDDAT12_R      - pin D26 */
	/* SDDAT13_R      - pin D27 */
	/* VID0_0         - pin D28 */
	/* VID0_1         - pin D29 */
	/* VID0_2         - pin D30 */
	/* VID0_3         - pin D31 */

	/* GPIO PORT E */
	/* VID0_4         - pin  E0 */
	/* VID0_5         - pin  E1 */
	/* VID0_6         - pin  E2 */
	/* VID0_7         - pin  E3 */
	/* VID0_PCLK      - pin  E4 */
	/* VID0_HSYNC     - pin  E5 */
	/* VID0_VSYNC     - pin  E6 */
	/* VID1_VSYNC     - pin  E7 */

	/* NC	          - pin  E8 */
	gpio_map[DPAD_LEFT]		= LF3000_GPIO_PORT_E | 9;
	gpio_map[HEADPHONE_JACK]	= LF3000_GPIO_PORT_E | 10;
	/* NC             - pin E11 */
	gpio_map[AUDIO_INT]		= LF3000_GPIO_PORT_E | 12;
	/* VID1_HSYNC     - pin E13 */
	/* SCK            - pin E14 */
	gpio_map[LCD_SPI]		= LF3000_GPIO_PORT_E | 15; //LCD_SCS_L

	gpio_map[AUDIO_MUTE]		= LF3000_GPIO_PORT_E | 16;
	gpio_map[AUDIO_RST_L]		= LF3000_GPIO_PORT_E | 17;
	/* NC             - pin E18 */
	/* SDI            - pin E19 */
	gpio_map[BUTTON_HOME]		= LF3000_GPIO_PORT_E | 20;
	gpio_map[DPAD_DOWN]		= LF3000_GPIO_PORT_E | 21;
	gpio_map[LED_ENA]		= LF3000_GPIO_PORT_E | 22; //LCD_LED_ENA
	gpio_map[DPAD_RIGHT]		= LF3000_GPIO_PORT_E | 23;
	
	gpio_map[DPAD_UP]		= LF3000_GPIO_PORT_E | 24;
	/* JTAG_TRST_L    - pin E25 */
	/* JTAG_TMS       - pin E26 */
	/* JTAG_TDI       - pin E27 */
	/* JTAG_TCK       - pin E28 */
	/* JTAG_TDO       - pin E29 */
	gpio_map[USBD_ID_SENSE]		= LF3000_GPIO_PORT_E | 30; //USB_OTG_ID
	gpio_map[USB_POWER_FLT_L]	= LF3000_GPIO_PORT_E | 31; //USB_PWR_OK_L

	/* GPIO Port ALIVE */
	/* N/C             - pin  0 */
	/* N/C             - pin  1 */
	/* N/C             - pin  2 */
	/* N/C             - pin  3 */
	/* N/C             - pin  4 */
	/* N/C             - pin  5 */

} /* init_bogota() */

static void init_cabo(void)
{
	printk(KERN_WARNING "%s: CABO (Alpha) GPIO mapping\n", __func__);

        /* UNRESOLVED: FCAM_CLK_ENA_L, USB_CHG_DETECT */

        /* GPIO Port A */
        /* VOUT_PCLK_R  - pin  A0 */
        /* PVD0      - pin  A1 */
        /* PVD1      - pin  A2 */
        /* PVD2      - pin  A3 */
        /* PVD3      - pin  A4 */
        /* PVD4      - pin  A5 */
        /* PVD5      - pin  A6 */
        /* PVD6      - pin  A7 */

        /* PVD7      - pin  A8 */
        /* PVD8      - pin  A9 */
        /* PVD9      - pin A10 */
        /* PVD10     - pin A11 */
        /* PVD11     - pin A12 */
        /* PVD12     - pin A13 */
        /* PVD13     - pin A14 */
        /* PVD14     - pin A15 */

        /* PVD15     - pin A16 */
        /* PVD16     - pin A17 */
        /* PVD17     - pin A18 */
        /* PVD18     - pin A19 */
        /* PVD19     - pin A20 */
        /* PVD20     - pin A21 */
        /* PVD21     - pin A22 */
        /* PVD22     - pin A23 */

        /* PVD23         - pin A24 */
        /* VOUT_PVSYNC_R - pin A25 */
        /* VOUT_PHSYNC_R - pin A26 */
        /* VOUT_DE_R     - pin A27 */
        /* VID1_PCLK     - pin A28 */
        /* SDCLK0_R      - pin A29 */
        /* VID1_0        - pin A30 */
        /* SDCMD0_R      - pin A31 */	

        /* GPIO Port B */
        /* VID1_1      - pin  B0 */
        /* SDDAT00_R   - pin  B1 */
        /* VID1_2      - pin  B2 */
        /* SDDAT01_R   - pin  B3 */
        /* VID1_3      - pin  B4 */
        /* SDDAT02_R   - pin  B5 */
        /* VID1_4      - pin  B6 */
        /* SDDAT03_R   - pin  B7 */

        /* VID1_5     - pin  B8 */
        /* VID1_6     - pin  B9 */
        /* VID1_7     - pin B10 */
        /* NAND_CLE_R - pin B11 */
        /* NAND_ALE_R - pin B12 */
        /* SD0        - pin B13 */
        /* NAND_RNB   - pin B14 */
#if 0	/* 4nov13  Added, then disabled after seeing that NAND_RBN isn't defined */
	gpio_map[NAND_RNB]		= LF3000_GPIO_PORT_B | 14;
#endif	/* 4nov13 */
        /* SD1        - pin B15 */

        /* NAND_NFOE_R_L  - pin B16 */
        /* SD2            - pin B17 */
        /* NAND_NFWE_R_L  - pin B18 */
        /* SD3            - pin B19 */
        /* SD4            - pin B20 */
        /* SD5            - pin B21 */
        /* SD6            - pin B22 */
        /* SD7            - pin B23 */

	gpio_map[TOUCHSCREEN_Y2]	= LF3000_GPIO_PORT_B | 24;	//FIXME CTP_RST_RTP_Y2_R
	gpio_map[TOUCHSCREEN_X2]	= LF3000_GPIO_PORT_B | 25;	//FIXME CTP_INT_RTP_X2_R
	/* N/C		  - pin B26 */
	/* N/C            - pin B27 */
	/* N/C            - pin B28 */
	gpio_map[BUTTON_VOLUMEUP]       = LF3000_GPIO_PORT_B | 29;	
	gpio_map[BUTTON_VOLUMEDOWN]     = LF3000_GPIO_PORT_B | 30;
	/* N/C            - pin B31 */

        /* GPIO Port C */
	/* N/C           - pin C0  */
        /* N/C           - pin C1  */
        /* N/C           - pin C2  */
        /* N/C  	 - pin C3  */
	gpio_map[LFP100_INT]		= LF3000_GPIO_PORT_C | 4;
        /* CIP_L         - pin C5  */
	gpio_map[CARTRIDGE_DETECT]	= LF3000_GPIO_PORT_C | 5;
	gpio_map[ACCEL_INT]             = LF3000_GPIO_PORT_C | 6;
        /* N/C	         - pin C7  */
	
	/* N/C        - pin C8  */
        /* SSPCLK2    - pin C9  */
        /* SSPFRM2    - pin C10 */
        /* SSPRXD2    - pin C11 */
        /* SSPTXD2    - pin C12 */
        /* N/C        - pin C13 */
        /* N/C        - pin C14 */
	gpio_map[FRONT_CAM_ENABLE_L]    = LF3000_GPIO_PORT_C | 15;	//FIXME VID2_ENA_L

	gpio_map[FRONT_CAM_RESET_L]     = LF3000_GPIO_PORT_C | 16;	//FIXME VID2_RESET_L
        /* N/C	      - pin C17 */
        /* SDCLK2     - pin C18 */
        /* SDCMD2     - pin C19 */
        /* SDDAT20    - pin C20 */	//FIXME - need to check if these pins (c20 - c23) are same as SD0_D0 - SD0_D3
        /* SDDAT21    - pin C21 */
        /* SDDAT22    - pin C22 */
        /* SDDAT23    - pin C23 */

        /* N/C 	      - pin C24 */
        /* nSWAIT     - pin C25 */	//FIXME - do we need to map this ? 
        /* N/C        - pin C26 */
        /* N/C        - pin C27 */
        /* N/C 	      - pin C28 */
        /* SSPCLK0_R  - pin C29 */
        /* SSPFRM0    - pin C30 */                                    
        /* SSPTXD0    - pin C31 */

        /* GPIO Port D */

        /* SSPRXD0    - pin  D0 */
        /* MCU_PWM0   - pin  D1 */
	gpio_map[I2C_SCL0]              = LF3000_GPIO_PORT_D | 2;
	gpio_map[I2C_SDA0]              = LF3000_GPIO_PORT_D | 3;
	gpio_map[I2C_SCL1]              = LF3000_GPIO_PORT_D | 4;
	gpio_map[I2C_SDA1]              = LF3000_GPIO_PORT_D | 5;
	gpio_map[I2C_SCL2]	            = LF3000_GPIO_PORT_D | 6;
	gpio_map[I2C_SDA2]              = LF3000_GPIO_PORT_D | 7;

        /* N/C          - pin  D8  */
        /* I2SDOUT_OUT  - pin  D9  */
        /* I2SBCLK	- pin  D10 */
        /* I2SDIN       - pin  D11 */
        /* I2SSYNC	- pin  D12 */	
        /* I2SMCLK  	- pin  D13 */
        /* RX0          - pin  D14 */	
        /* N/C	        - pin  D15 */

        /* N/C	        - pin D16 */
        /* N/C 		- pin D17 */
        /* TX0          - pin D18 */	
	gpio_map[WIFI_RESET]		= LF3000_GPIO_PORT_D | 19;
	gpio_map[CHG_FLT]               = LF3000_GPIO_PORT_D | 20;
        /* N/C		 - pin D21 */
        /* SDCLK1_R      - pin D22 */	
        /* SDCMD1_R      - pin D23 */

        /* SDDAT10_R     - pin D24 */
        /* SDDAT11_R     - pin D25 */
        /* SDDAT12_R     - pin D26 */
        /* SDDAT13_R     - pin D27 */
        /* VID0_0        - pin D28 */
        /* VID0_1        - pin D29 */
        /* VID0_2        - pin D30 */
        /* VID0_3        - pin D31 */

        /* GPIO PORT E */
        /* VID0_4        - pin E0  */
        /* VID0_5        - pin E1  */
        /* VID0_6        - pin E2  */
        /* VID0_7        - pin E3  */
        /* VID0_PCLK     - pin E4  */
	gpio_map[REAR_CAM_ENABLE_L]	= LF3000_GPIO_PORT_E | 5;
	gpio_map[REAR_CAM_RESET_L]	= LF3000_GPIO_PORT_E | 6;
       	/* N/C           - pin E7  */ 

        /* N/C 		 - pin E8  */
	gpio_map[DPAD_LEFT]		= LF3000_GPIO_PORT_E | 9;
	gpio_map[HEADPHONE_JACK] = LF3000_GPIO_PORT_E | 10;
        gpio_map[CHG_INT]        = LF3000_GPIO_PORT_E | 11;	
        /* N/C           - pin E12 */	
        /* N/C           - pin E13 */
	/* SCK_R         - pin E14 */		
	gpio_map[LCD_SPI]               = LF3000_GPIO_PORT_E | 15;//FIXME it is supposed to be LCD_SCS_L(same?)

        /* N/C           - pin E16 */
        /* N/C           - pin E17 */
        /* N/C           - pin E18 */
        /* SDI           - pin E19 */
	gpio_map[BUTTON_ESC]             = LF3000_GPIO_PORT_E | 20;
	gpio_map[DPAD_DOWN]               = LF3000_GPIO_PORT_E | 21;
	gpio_map[LED_ENA]                 = LF3000_GPIO_PORT_E | 22;
	gpio_map[DPAD_RIGHT]              = LF3000_GPIO_PORT_E | 23;
	gpio_map[DPAD_UP]                 = LF3000_GPIO_PORT_E | 24;	
        /* JTAG_TRST_L   - pin E25 */
        /* JTAG_TMS      - pin E26 */
        /* JTAG_TDI      - pin E27 */
        /* JTAG_TCK      - pin E28 */
        /* JTAG_TDO      - pin E29 */

	gpio_map[USBD_ID_SENSE]             = LF3000_GPIO_PORT_E | 30; //FIXME-it is supposed tobe USB_OTG_ID .. verify
	gpio_map[USB_POWER_FLT_L]           = LF3000_GPIO_PORT_E | 31; //FIXME-it s supposed to be USB_PWR_OK_L.. verify

        /* GPIO Port ALIVE */
        /* N/C        -  pin 0 */
        /* N/C        -  pin 1 */
        /* N/C        -  pin 2 */
        /* N/C        -  pin 3 */
        /* N/C        -  pin 4 */
        /* N/C        -  pin 5 */
        /* N/C        -  pin 6 */
        /* N/C        -  pin 7 */
}	/* init_cabo() */

static void init_glasgow(void)
{
	printk(KERN_WARNING "%s: GLASGOW (Alpha) GPIO mapping\n", __func__);

        /* UNRESOLVED: FCAM_CLK_ENA_L, USB_CHG_DETECT */

        /* GPIO Port A */
        /* N/C       - pin  A0 */
        /* DISD0     - pin  A1 */
        /* DISD1     - pin  A2 */
        /* DISD2     - pin  A3 */
        /* DISD3     - pin  A4 */
        /* DISD4     - pin  A5 */
        /* DISD5     - pin  A6 */
        /* DISD6     - pin  A7 */

        /* DISD7     - pin  A8 */
        /* N/C       - pin  A9 */
        /* N/C       - pin A10 */
        /* N/C       - pin A11 */
        /* N/C       - pin A12 */
	gpio_map[USBD_ID_SENSE]		= LF3000_GPIO_PORT_A | 13;
	gpio_map[USB_POWER_FLT_L]	= LF3000_GPIO_PORT_A | 14;
	gpio_map[USB_MUX_SEL_R]		= LF3000_GPIO_PORT_A | 15;

	/* N/C       - pin A16 */
	/* N/C       - pin A17 */
	/* N/C       - pin A18 */
	/* N/C       - pin A19 */
	/* N/C       - pin A20 */
	/* N/C       - pin A21 */
	/* N/C       - pin A22 */
	/* N/C       - pin A23 */

	/* N/C       - pin A24 */
	/* N/C       - pin A25 */
	/* N/C       - pin A26 */
	/* N/C       - pin A27 */
	/* N/C       - pin A28 */
        /* SDCLK0_R  - pin A29 */
        /* SDEX0     - pin A30 */
        /* SDCMD0_R  - pin A31 */

        /* GPIO Port B */
        /* SDEX1     - pin  B0 */
        /* SDDAT00_R - pin  B1 */
        /* SDEX2     - pin  B2 */
        /* SDDAT01_R - pin  B3 */
        /* SDEX3     - pin  B4 */
        /* SDDAT02_R - pin  B5 */
        /* SDEX4     - pin  B6 */
        /* SDDAT03_R - pin  B7 */

        /* SDEX5     - pin  B8 */
        /* SDEX6     - pin  B9 */
        /* SDEX7     - pin B10 */
	/* N/C       - pin B11 */
	/* N/C       - pin B12 */
	/* N/C       - pin B13 */
	/* N/C       - pin B14 */
	/* N/C       - pin B15 */

	/* N/C       - pin B16 */
	/* N/C       - pin B17 */
	/* N/C       - pin B18 */
	/* N/C       - pin B19 */
	/* N/C       - pin B20 */
	/* N/C       - pin B21 */
	/* N/C       - pin B22 */
	/* N/C       - pin B23 */

	/* N/C       - pin B24 */
        /* N/C       - pin B25 */
	/* N/C	     - pin B26 */
	gpio_map[SYNC_BUTTON]       = LF3000_GPIO_PORT_B | 27;
	/* N/C       - pin B28 */
        /* N/C       - pin B29 */
        /* N/C       - pin B30 */
	/* N/C       - pin B31 */

        /* GPIO Port C */
	/* N/C       - pin  C0 */
        /* N/C       - pin  C1 */
        /* N/C       - pin  C2 */
        /* N/C       - pin  C3 */
        /* N/C       - pin  C4 */
        /* N/C       - pin  C5 */
        /* N/C       - pin  C6 */
        /* N/C       - pin  C7 */

        /* N/C       - pin  C8 */
        /* N/C       - pin  C9 */
        /* N/C       - pin C10 */
	gpio_map[USB_OTG_PWR_ENA]	= LF3000_GPIO_PORT_C | 11;
        /* N/C       - pin C12 */
        /* N/C       - pin C13 */
	gpio_map[WIFI_RESET]        	= LF3000_GPIO_PORT_C | 14;
	gpio_map[BT_RESET_L]        = LF3000_GPIO_PORT_C | 15;

	gpio_map[WIFI_HOST_WAKE]	= LF3000_GPIO_PORT_C | 16;
        /* N/C	     - pin C17 */
        /* SDCLK2    - pin C18 */
        /* SDCMD2    - pin C19 */
        /* SDDAT20   - pin C20 */
        /* SDDAT21   - pin C21 */
        /* SDDAT22   - pin C22 */
        /* SDDAT23   - pin C23 */

    gpio_map[BT_LINK]			= LF3000_GPIO_PORT_C | 24;
        /* nSWAIT    - pin C25 */
        /* N/C       - pin C26 */
        /* N/C       - pin C27 */
	gpio_map[ETHERNET_CS]		= LF3000_GPIO_PORT_C | 28;
        /* SSPCLK0_R - pin C29 */
        /* SSPFRM0   - pin C30 */
        /* SSPTXD0   - pin C31 */

        /* GPIO Port D */

        /* SSPRXD0   - pin  D0 */
        /* N/C       - pin  D1 */
	gpio_map[I2C_SCL0]              = LF3000_GPIO_PORT_D | 2;
	gpio_map[I2C_SDA0]              = LF3000_GPIO_PORT_D | 3;
//	gpio_map[I2C_SCL1]              = LF3000_GPIO_PORT_D | 4;
//	gpio_map[I2C_SDA1]              = LF3000_GPIO_PORT_D | 5;
        /* N/C       - pin  D6 */
        /* N/C       - pin  D7 */

        /* N/C       - pin  D8 */
        /* I2SDOUT_OUT - pin D9 */
        /* I2SBCLK   - pin D10 */
        /* I2SDIN    - pin D11 */
        /* I2SSYNC   - pin D12 */
        /* I2SMCLK   - pin D13 */
        /* RX0       - pin D14 */

        /* RX1	     - pin D15 */

        /* N/C	     - pin D16 */
        /* N/C 	     - pin D17 */
        /* TX0       - pin D18 */
        /* TX1       - pin D19 */
        /* N/C	     - pin D20 */
        /* N/C	     - pin D21 */
        /* SDCLK1_R  - pin D22 */
        /* SDCMD1_R  - pin D23 */

        /* SDDAT10_R - pin D24 */
        /* SDDAT11_R - pin D25 */
        /* SDDAT12_R - pin D26 */
        /* SDDAT13_R - pin D27 */
        /* N/C	     - pin D28 */
        /* N/C	     - pin D29 */
        /* N/C	     - pin D30 */
        /* N/C	     - pin D31 */

        /* GPIO PORT E */
        /* N/C       - pin  E0 */
        /* N/C	     - pin  E1 */
        /* N/C	     - pin  E2 */
        /* N/C	     - pin  E3 */
        /* N/C	     - pin  E4 */
   	/* N/C       - pin  E5 */
   	/* N/C       - pin  E6 */
       	/* N/C       - pin  E7 */

        /* N/C       - pin  E8 */
        /* N/C       - pin  E9 */
        /* N/C       - pin E10 */
        /* N/C       - pin E11 */
        /* N/C       - pin E12 */
        /* N/C       - pin E13 */
        /* N/C       - pin E14 */
        /* N/C       - pin E15 */

        /* N/C       - pin E16 */
        /* N/C       - pin E17 */
        /* N/C       - pin E18 */
        /* N/C       - pin E19 */
        /* N/C       - pin E20 */
        /* N/C       - pin E21 */
        /* N/C       - pin E22 */
        /* N/C       - pin E23 */
        /* N/C       - pin E24 */
        /* N/C       - pin E25 */
        /* N/C       - pin E26 */
        /* N/C       - pin E27 */
        /* N/C       - pin E28 */
        /* N/C       - pin E29 */
        /* N/C       - pin E30 */
        /* N/C       - pin E31 */

        /* GPIO Port ALIVE */
        /* N/C        -  pin 0 */
        /* N/C        -  pin 1 */
        /* N/C        -  pin 2 */
        /* N/C        -  pin 3 */
        /* N/C        -  pin 4 */
        /* N/C        -  pin 5 */
        /* N/C        -  pin 6 */
        /* N/C        -  pin 7 */
}	/* init_glasgow() */

static void init_r3k(void)
{
	printk(KERN_WARNING "%s: R3K GPIO mapping\n", __func__);

	/* UNRESOLVED: FCAM_CLK_ENA_L, USB_CHG_DETECT */

	/* GPIO Port A */
	/* VOUT_PCLK_R  - pin  A0 */
	/* PVD0      - pin  A1 */
	/* PVD1      - pin  A2 */
	/* PVD2      - pin  A3 */
	/* PVD3      - pin  A4 */
	/* PVD4      - pin  A5 */
	/* PVD5      - pin  A6 */
	/* PVD6      - pin  A7 */

	/* PVD7      - pin  A8 */
	/* PVD8      - pin  A9 */
	/* PVD9      - pin A10 */
	/* PVD10     - pin A11 */
	/* PVD11     - pin A12 */
	/* PVD12     - pin A13 */
	/* PVD13     - pin A14 */
	/* PVD14     - pin A15 */

	/* PVD15     - pin A16 */
	/* PVD16     - pin A17 */
	/* PVD17     - pin A18 */
	/* PVD18     - pin A19 */
	/* PVD19     - pin A20 */
	/* PVD20     - pin A21 */
	/* PVD21     - pin A22 */
	/* PVD22     - pin A23 */

	/* PVD23         - pin A24 */
	/* VOUT_PVSYNC_R - pin A25 */
	/* VOUT_PHSYNC_R - pin A26 */
	/* VOUT_DE_R     - pin A27 */
	/* VID1_PCLK     - pin A28 */
	/* SDCLK0_R      - pin A29 */
	/* SDEX0         - pin A30 */
	/* SDCMD0_R      - pin A31 */

	/* GPIO Port B */
	/* SDEX1       - pin  B0 */
	/* SDDAT00_R   - pin  B1 */
	/* SDEX2       - pin  B2 */
	/* SDDAT01_R   - pin  B3 */
	/* SDEX3       - pin  B4 */
	/* SDDAT02_R   - pin  B5 */
	/* SDEX4       - pin  B6 */
	/* SDDAT03_R   - pin  B7 */

	/* SDEX5      - pin  B8 */
	/* SDEX6      - pin  B9 */
	/* SDEX7      - pin B10 */
	/* NAND_CLE_R - pin B11 */
	/* NAND_ALE_R - pin B12 */
	/* SD0        - pin B13 */
	/* NAND_RNB   - pin B14 */
#if 0	/* 4nov13  Added, then disabled after seeing that NAND_RBN isn't defined */
	gpio_map[NAND_RNB]		= LF3000_GPIO_PORT_B | 14;
#endif	/* 4nov13 */
	/* SD1        - pin B15 */

	/* NAND_NFOE_R_L  - pin B16 */
	/* SD2            - pin B17 */
	/* NAND_NFWE_R_L  - pin B18 */
	/* SD3            - pin B19 */
	/* SD4            - pin B20 */
	/* SD5            - pin B21 */
	/* SD6            - pin B22 */
	/* SD7            - pin B23 */

	gpio_map[BUTTON_ESC]		= LF3000_GPIO_PORT_B | 24;
	gpio_map[DPAD_UP]		= LF3000_GPIO_PORT_B | 25;
	gpio_map[DPAD_DOWN]		= LF3000_GPIO_PORT_B | 26;
	gpio_map[DPAD_RIGHT]		= LF3000_GPIO_PORT_B | 27;
	gpio_map[DPAD_LEFT]		= LF3000_GPIO_PORT_B | 28;
	gpio_map[BUTTON_VOLUMEUP]	= LF3000_GPIO_PORT_B | 29;
	gpio_map[BUTTON_VOLUMEDOWN]	= LF3000_GPIO_PORT_B | 30;
	gpio_map[HEADPHONE_JACK]	= LF3000_GPIO_PORT_B | 31;

	/* GPIO Port C */
	gpio_map[WIFI_RESET]		= LF3000_GPIO_PORT_C | 0;
	/* N/C           - pin C1  */
	/* N/C           - pin C2  */
	/* MCU_HDMI_CEC  - pin C3  */
	gpio_map[LFP100_INT]		= LF3000_GPIO_PORT_C | 4;
	/* CIP_L         - pin C5  */
	gpio_map[CARTRIDGE_DETECT]	= LF3000_GPIO_PORT_C | 5;
	gpio_map[ACCEL_INT]		= LF3000_GPIO_PORT_C | 6;
	/* CPT_WAKE      - pin C7  */

	gpio_map[TP_INT]		= LF3000_GPIO_PORT_C | 8;
	/* SSPCLK2    - pin C9  */
	/* SSPFRM2    - pin C10 */
	/* SSPRXD2    - pin C11 */
	/* SSPTXD2    - pin C12 */
	/* N/C        - pin C13 */
	/* VID2_PCLK  - pin C14 */
	gpio_map[FRONT_CAM_ENABLE_L]	= LF3000_GPIO_PORT_C | 15;

	gpio_map[FRONT_CAM_RESET_L]	= LF3000_GPIO_PORT_C | 16;
	/* VID2_0     - pin C17 */
	/* VID2_1     - pin C18 */
	/* VID2_2     - pin C19 */
	/* VID2_3     - pin C20 */
	/* VID2_4     - pin C21 */
	/* VID2_5     - pin C22 */
	/* VID2_6     - pin C23 */

	/* VID2_7     - pin C23 */
	/* TBD        - pin C25 */
	/* N/C        - pin C26 */
	/* N/C        - pin C27 */
	/* MCU_nSCS1  - pin C28 */
	/* SSPCLK0_R  - pin C29 */
	/* SSPFRM0    - pin C30 */
	/* SSPTXD0    - pin C31 */

	/* GPIO Port D */

	/* SSPRXD0    - pin  D0 */
	/* MCU_PWM0   - pin  D1 */
	gpio_map[I2C_SCL0]		= LF3000_GPIO_PORT_D | 2;
	gpio_map[I2C_SDA0]		= LF3000_GPIO_PORT_D | 3;
	gpio_map[I2C_SCL1]		= LF3000_GPIO_PORT_D | 4;
	gpio_map[I2C_SDA1]		= LF3000_GPIO_PORT_D | 5;
	/* SCL2       - pin  D6 */
	/* SDA2       - pin  D7 */

	/* N/C          - pin  D8 */
	/* I2SDOUT_OUT  - pin  D9 */
	/* I2SSYNC_OUT  - pin  D10*/
	/* I2SDIN       - pin  D11*/
	/* I2SMCLK_OUT  - pin  D12*/
	/* I2SBCLK_OUT  - pin  D13*/
	/* RX0          - pin  D14*/
	/* ETHER_INT    - pin  D15*/

	/* CHG_CE_L      - pin D16 */
	/* CHG_INT       - pin D17 */
	/* TX0           - pin D18 */
	/* A100_INT      - pin D19 */
	gpio_map[LED_ENA]		= LF3000_GPIO_PORT_D | 20;
	/* USB_PWR_OK_L  - pin D21 */
	/* SDCLK1_R      - pin D22 */
	/* SDCMD1_R      - pin D23 */

	/* SDDAT10_R     - pin D24 */
	/* SDDAT11_R     - pin D25 */
	/* SDDAT12_R     - pin D26 */
	/* SDDAT13_R     - pin D27 */
	/* VID0_0        - pin D28 */
	/* VID0_1        - pin D29 */
	/* VID0_2        - pin D30 */
	/* VID0_3        - pin D31 */

	/* GPIO PORT E */
	/* VID0_4        - pin E0  */
	/* VID0_5        - pin E1  */
	/* VID0_6        - pin E2  */
	/* VID0_7        - pin E3  */
	/* VID0_PCLK     - pin E4  */
	/* VID0_HSYNC    - pin E5  */
	/* VID0_VSYNC    - pin E6  */
	gpio_map[REAR_CAM_RESET_L]	= LF3000_GPIO_PORT_E | 7;

	/* MCU_SA2       - pin E8  */
	/* N/C           - pin E9  */
	/* N/C           - pin E10 */
	/* N/C           - pin E11 */
	/* N/C           - pin E12 */
	gpio_map[REAR_CAM_ENABLE_L]	= LF3000_GPIO_PORT_E | 13;
	/* SCK_R         - pin E14 */
	gpio_map[LCD_SPI]		= LF3000_GPIO_PORT_E | 15;

	/* N/C           - pin E16 */
	/* N/C           - pin E17 */
	/* N/C           - pin E18 */
	/* SDI           - pin E19 */
	/* N/C           - pin E20 */
	/* N/C           - pin E21 */
	/* N/C           - pin E22 */
	/* N/C           - pin E23 */

	/* CHG_FLT       - pin E24 */
	/* JTAG_TRST_L   - pin E25 */
	/* JTAG_TMS      - pin E26 */
	/* JTAG_TDI      - pin E27 */
	/* JTAG_TCK      - pin E28 */
	/* JTAG_TDO      - pin E29 */
	/* MCU_nSOE      - pin E30 */
	/* MCU_nSWE      - pin E31 */

	/* GPIO Port ALIVE */
	/* N/C        -  pin 0 */
	/* N/C        -  pin 1 */
	/* N/C        -  pin 2 */
	/* N/C        -  pin 3 */
	/* N/C        -  pin 4 */
	/* N/C        -  pin 5 */
	/* N/C        -  pin 6 */
	/* N/C        -  pin 7 */
}	/* init_r3k() */

static void init_vtk(void)
{

//#define LF3000_GPIO_EXTENDER_BASE 0xC00
	printk(KERN_WARNING "%s: VTK GPIO mapping are not availble, this is just a placeholder\n", __func__);
}

static void init_xanadu(void)
{
	printk(KERN_WARNING "%s: XANADU (Alpha) GPIO mapping\n", __func__);

        /* UNRESOLVED: FCAM_CLK_ENA_L, USB_CHG_DETECT */

        /* GPIO Port A */
        /* VOUT_PCLK_R  - pin  A0 */
        /* PVD0      - pin  A1 */
        /* PVD1      - pin  A2 */
        /* PVD2      - pin  A3 */
        /* PVD3      - pin  A4 */
        /* PVD4      - pin  A5 */
        /* PVD5      - pin  A6 */
        /* PVD6      - pin  A7 */

        /* PVD7      - pin  A8 */
        /* PVD8      - pin  A9 */
        /* PVD9      - pin A10 */
        /* PVD10     - pin A11 */
        /* PVD11     - pin A12 */
        /* PVD12     - pin A13 */
        /* PVD13     - pin A14 */
        /* PVD14     - pin A15 */

        /* PVD15     - pin A16 */
        /* PVD16     - pin A17 */
        /* PVD17     - pin A18 */
        /* PVD18     - pin A19 */
        /* PVD19     - pin A20 */
        /* PVD20     - pin A21 */
        /* PVD21     - pin A22 */
        /* PVD22     - pin A23 */

        /* PVD23         - pin A24 */
        /* VOUT_PVSYNC_R - pin A25 */
        /* VOUT_PHSYNC_R - pin A26 */
        /* VOUT_DE_R     - pin A27 */
        /* VID1_PCLK     - pin A28 */
        /* SDCLK0_R      - pin A29 */
        /* VID1_0        - pin A30 */
        /* SDCMD0_R      - pin A31 */

        /* GPIO Port B */
        /* VID1_1      - pin  B0 */
        /* SDDAT00_R   - pin  B1 */
        /* VID1_2      - pin  B2 */
        /* SDDAT01_R   - pin  B3 */
        /* VID1_3      - pin  B4 */
        /* SDDAT02_R   - pin  B5 */
        /* VID1_4      - pin  B6 */
        /* SDDAT03_R   - pin  B7 */

        /* VID1_5     - pin  B8 */
        /* VID1_6     - pin  B9 */
        /* VID1_7     - pin B10 */
        /* NAND_CLE_R - pin B11 */
        /* NAND_ALE_R - pin B12 */
        /* SD0        - pin B13 */
        /* NAND_RNB   - pin B14 */
#if 0	/* 4nov13  Added, then disabled after seeing that NAND_RBN isn't defined */
	gpio_map[NAND_RNB]		= LF3000_GPIO_PORT_B | 14;
#endif	/* 4nov13 */
        /* SD1        - pin B15 */

        /* NAND_NFOE_R_L  - pin B16 */
        /* SD2            - pin B17 */
        /* NAND_NFWE_R_L  - pin B18 */
        /* SD3            - pin B19 */
        /* SD4            - pin B20 */
        /* SD5            - pin B21 */
        /* SD6            - pin B22 */
        /* SD7            - pin B23 */

	gpio_map[TOUCHSCREEN_Y2]	= LF3000_GPIO_PORT_B | 24;	//FIXME CTP_RST_RTP_Y2_R
	gpio_map[TOUCHSCREEN_X2]	= LF3000_GPIO_PORT_B | 25;	//FIXME CTP_INT_RTP_X2_R
	/* N/C		  - pin B26 */
	/* N/C            - pin B27 */
	/* N/C            - pin B28 */
	gpio_map[BUTTON_VOLUMEUP]       = LF3000_GPIO_PORT_B | 29;
	gpio_map[BUTTON_VOLUMEDOWN]     = LF3000_GPIO_PORT_B | 30;
	/* N/C            - pin B31 */

        /* GPIO Port C */
	/* N/C           - pin C0  */
        /* N/C           - pin C1  */
        /* N/C           - pin C2  */
        /* N/C  	 - pin C3  */
#if defined(CONFIG_SOC_LFP100)
	gpio_map[LFP100_INT]		= LF3000_GPIO_PORT_C | 4;
#elif defined(CONFIG_TC7734_PMIC)
	gpio_map[TC7734_INT]		= LF3000_GPIO_PORT_C | 4; //HGP
#endif
	gpio_map[CARTRIDGE_DETECT]	= LF3000_GPIO_PORT_C | 5;
	gpio_map[ACCEL_INT]             = LF3000_GPIO_PORT_C | 6;
        /* N/C	         - pin C7  */

	/* N/C        - pin C8  */
        /* SSPCLK2    - pin C9  */
        /* SSPFRM2    - pin C10 */
        /* SSPRXD2    - pin C11 */
        /* SSPTXD2    - pin C12 */
        /* N/C        - pin C13 */
        /* N/C        - pin C14 */
	gpio_map[FRONT_CAM_ENABLE_L]    = LF3000_GPIO_PORT_C | 15;	//FIXME VID2_ENA_L

	gpio_map[FRONT_CAM_RESET_L]     = LF3000_GPIO_PORT_C | 16;	//FIXME VID2_RESET_L
        /* N/C	      - pin C17 */
        /* SDCLK2     - pin C18 */
        /* SDCMD2     - pin C19 */
        /* SDDAT20    - pin C20 */	//FIXME - need to check if these pins (c20 - c23) are same as SD0_D0 - SD0_D3
        /* SDDAT21    - pin C21 */
        /* SDDAT22    - pin C22 */
        /* SDDAT23    - pin C23 */

        /* N/C 	      - pin C24 */
        /* nSWAIT     - pin C25 */	//FIXME - do we need to map this ?
        /* N/C        - pin C26 */
        /* N/C        - pin C27 */
        /* N/C 	      - pin C28 */
        /* SSPCLK0_R  - pin C29 */
        /* SSPFRM0    - pin C30 */
        /* SSPTXD0    - pin C31 */

        /* GPIO Port D */

        /* SSPRXD0    - pin  D0 */
        /* MCU_PWM0   - pin  D1 */
	gpio_map[I2C_SCL0]              = LF3000_GPIO_PORT_D | 2;
	gpio_map[I2C_SDA0]              = LF3000_GPIO_PORT_D | 3;
	gpio_map[I2C_SCL1]              = LF3000_GPIO_PORT_D | 4;
	gpio_map[I2C_SDA1]              = LF3000_GPIO_PORT_D | 5;
	gpio_map[TOUCHSCREEN_Y1]	= LF3000_GPIO_PORT_D | 6;	//FIXME CTP_SCL_RTP_Y1_R
	gpio_map[TOUCHSCREEN_X1]        = LF3000_GPIO_PORT_D | 7;       //FIXME CTP_SCL_RTP_X1_R

        /* N/C          - pin  D8  */
        /* I2SDOUT_OUT  - pin  D9  */
        /* I2SBCLK	- pin  D10 */
        /* I2SDIN       - pin  D11 */
        /* I2SSYNC	- pin  D12 */
        /* I2SMCLK  	- pin  D13 */
        /* RX0          - pin  D14 */
        /* N/C	        - pin  D15 */

        /* N/C	        - pin D16 */
        /* N/C 		- pin D17 */
        /* TX0          - pin D18 */
	gpio_map[WIFI_RESET]		= LF3000_GPIO_PORT_D | 19;
	gpio_map[CHG_FLT]               = LF3000_GPIO_PORT_D | 20;
        /* N/C		 - pin D21 */
        /* SDCLK1_R      - pin D22 */
        /* SDCMD1_R      - pin D23 */

        /* SDDAT10_R     - pin D24 */
        /* SDDAT11_R     - pin D25 */
        /* SDDAT12_R     - pin D26 */
        /* SDDAT13_R     - pin D27 */
        /* VID0_0        - pin D28 */
        /* VID0_1        - pin D29 */
        /* VID0_2        - pin D30 */
        /* VID0_3        - pin D31 */

        /* GPIO PORT E */
        /* VID0_4        - pin E0  */
        /* VID0_5        - pin E1  */
        /* VID0_6        - pin E2  */
        /* VID0_7        - pin E3  */
        /* VID0_PCLK     - pin E4  */
	gpio_map[REAR_CAM_ENABLE_L]	= LF3000_GPIO_PORT_E | 5;
	gpio_map[REAR_CAM_RESET_L]	= LF3000_GPIO_PORT_E | 6;
       	/* N/C           - pin E7  */

        /* N/C 		 - pin E8  */
	gpio_map[DPAD_LEFT]		= LF3000_GPIO_PORT_E | 9;
	gpio_map[HEADPHONE_JACK] = LF3000_GPIO_PORT_E | 10;
        /* CHG_INT       - pin E11 */
        /* N/C           - pin E12 */
        /* N/C           - pin E13 */
	/* SCK_R         - pin E14 */
	gpio_map[LCD_SPI]               = LF3000_GPIO_PORT_E | 15;//FIXME it is supposed to be LCD_SCS_L(same?)

        /* N/C           - pin E16 */
        /* N/C           - pin E17 */
        /* N/C           - pin E18 */
        /* SDI           - pin E19 */
	gpio_map[BUTTON_ESC]             = LF3000_GPIO_PORT_E | 20;
	gpio_map[DPAD_DOWN]               = LF3000_GPIO_PORT_E | 21;
	gpio_map[LED_ENA]                 = LF3000_GPIO_PORT_E | 22;
	gpio_map[DPAD_RIGHT]              = LF3000_GPIO_PORT_E | 23;
	gpio_map[DPAD_UP]                 = LF3000_GPIO_PORT_E | 24;
        /* JTAG_TRST_L   - pin E25 */
        /* JTAG_TMS      - pin E26 */
        /* JTAG_TDI      - pin E27 */
        /* JTAG_TCK      - pin E28 */
        /* JTAG_TDO      - pin E29 */

	gpio_map[USBD_ID_SENSE]             = LF3000_GPIO_PORT_E | 30; //FIXME-it is supposed tobe USB_OTG_ID .. verify
	gpio_map[USB_POWER_FLT_L]           = LF3000_GPIO_PORT_E | 31; //FIXME-it s supposed to be USB_PWR_OK_L.. verify

        /* GPIO Port ALIVE */
        /* N/C        -  pin 0 */
        /* N/C        -  pin 1 */
        /* N/C        -  pin 2 */
        /* N/C        -  pin 3 */
        /* N/C        -  pin 4 */
        /* N/C        -  pin 5 */
        /* N/C        -  pin 6 */
        /* N/C        -  pin 7 */
}	/* init_xanadu() */


static void init_quito(void)
{
  printk(KERN_WARNING "%s: QUITO (Alpha) GPIO mapping\n", __func__);

        /* UNRESOLVED: FCAM_CLK_ENA_L, USB_CHG_DETECT */

        /* GPIO Port A */
        /* VOUT_PCLK_R  - pin  A0 */
        /* PVD0      - pin  A1 */
        /* PVD1      - pin  A2 */
        /* PVD2      - pin  A3 */
        /* PVD3      - pin  A4 */
        /* PVD4      - pin  A5 */
        /* PVD5      - pin  A6 */
        /* PVD6      - pin  A7 */

        /* PVD7      - pin  A8 */
        /* PVD8      - pin  A9 */
        /* PVD9      - pin A10 */
        /* PVD10     - pin A11 */
        /* PVD11     - pin A12 */
        /* PVD12     - pin A13 */
        /* PVD13     - pin A14 */
        /* PVD14     - pin A15 */

        /* PVD15     - pin A16 */
        /* PVD16     - pin A17 */
        /* PVD17     - pin A18 */
        /* PVD18     - pin A19 */
        /* PVD19     - pin A20 */
        /* PVD20     - pin A21 */
        /* PVD21     - pin A22 */
        /* PVD22     - pin A23 */

        /* PVD23         - pin A24 */
        /* VOUT_PVSYNC_R - pin A25 */
        /* VOUT_PHSYNC_R - pin A26 */
        /* VOUT_DE_R     - pin A27 */
        /* VID1_PCLK     - pin A28 */
        /* SDCLK0_R      - pin A29 */
        /* VID1_0        - pin A30 */
        /* SDCMD0_R      - pin A31 */

        /* GPIO Port B */
        /* VID1_1      - pin  B0 */
        /* SDDAT00_R   - pin  B1 */
        /* VID1_2      - pin  B2 */
        /* SDDAT01_R   - pin  B3 */
        /* VID1_3      - pin  B4 */
        /* SDDAT02_R   - pin  B5 */
        /* VID1_4      - pin  B6 */
        /* SDDAT03_R   - pin  B7 */

        /* VID1_5     - pin  B8 */
        /* VID1_6     - pin  B9 */
        /* VID1_7     - pin B10 */
        /* NAND_CLE_R - pin B11 */
        /* NAND_ALE_R - pin B12 */
        /* SD0        - pin B13 */
        /* NAND_RNB   - pin B14 */
#if 0 /* 4nov13  Added, then disabled after seeing that NAND_RBN isn't defined */
  gpio_map[NAND_RNB]    = LF3000_GPIO_PORT_B | 14;
#endif  /* 4nov13 */
        /* SD1        - pin B15 */

        /* NAND_NFOE_R_L  - pin B16 */
        /* SD2            - pin B17 */
        /* NAND_NFWE_R_L  - pin B18 */
        /* SD3            - pin B19 */
        /* SD4            - pin B20 */
        /* SD5            - pin B21 */
        /* SD6            - pin B22 */
        /* SD7            - pin B23 */

  gpio_map[TOUCHSCREEN_Y2]  = LF3000_GPIO_PORT_B | 24;  //FIXME CTP_RST_RTP_Y2_R
  gpio_map[TOUCHSCREEN_X2]  = LF3000_GPIO_PORT_B | 25;  //FIXME CTP_INT_RTP_X2_R
  /* N/C      - pin B26 */
  /* N/C            - pin B27 */
  /* N/C            - pin B28 */
  gpio_map[BUTTON_VOLUMEUP]       = LF3000_GPIO_PORT_B | 29;
  gpio_map[BUTTON_VOLUMEDOWN]     = LF3000_GPIO_PORT_B | 30;
  /* N/C            - pin B31 */

        /* GPIO Port C */
  /* N/C           - pin C0  */
        /* N/C           - pin C1  */
        /* N/C           - pin C2  */
        /* N/C     - pin C3  */
  gpio_map[LFP100_INT]    = LF3000_GPIO_PORT_C | 4;
        /* CIP_L         - pin C5  */
  gpio_map[CARTRIDGE_DETECT]  = LF3000_GPIO_PORT_C | 5;
  gpio_map[ACCEL_INT]             = LF3000_GPIO_PORT_C | 6;
        /* N/C           - pin C7  */

  /* N/C        - pin C8  */
        /* SSPCLK2    - pin C9  */
        /* SSPFRM2    - pin C10 */
        /* SSPRXD2    - pin C11 */
        /* SSPTXD2    - pin C12 */
        /* N/C        - pin C13 */
        /* N/C        - pin C14 */
  gpio_map[FRONT_CAM_ENABLE_L]    = LF3000_GPIO_PORT_C | 15;  //FIXME VID2_ENA_L

  gpio_map[FRONT_CAM_RESET_L]     = LF3000_GPIO_PORT_C | 16;  //FIXME VID2_RESET_L
        /* N/C        - pin C17 */
        /* SDCLK2     - pin C18 */
        /* SDCMD2     - pin C19 */
        /* SDDAT20    - pin C20 */  //FIXME - need to check if these pins (c20 - c23) are same as SD0_D0 - SD0_D3
        /* SDDAT21    - pin C21 */
        /* SDDAT22    - pin C22 */
        /* SDDAT23    - pin C23 */

        /* N/C        - pin C24 */
        /* nSWAIT     - pin C25 */  //FIXME - do we need to map this ?
        /* N/C        - pin C26 */
        /* N/C        - pin C27 */
        /* N/C        - pin C28 */
        /* SSPCLK0_R  - pin C29 */
        /* SSPFRM0    - pin C30 */
        /* SSPTXD0    - pin C31 */

        /* GPIO Port D */

        /* SSPRXD0    - pin  D0 */
        /* MCU_PWM0   - pin  D1 */
  gpio_map[I2C_SCL0]              = LF3000_GPIO_PORT_D | 2;
  gpio_map[I2C_SDA0]              = LF3000_GPIO_PORT_D | 3;
  gpio_map[I2C_SCL1]              = LF3000_GPIO_PORT_D | 4;
  gpio_map[I2C_SDA1]              = LF3000_GPIO_PORT_D | 5;
  gpio_map[I2C_SCL2]              = LF3000_GPIO_PORT_D | 6;
  gpio_map[I2C_SDA2]              = LF3000_GPIO_PORT_D | 7;

        /* N/C          - pin  D8  */
        /* I2SDOUT_OUT  - pin  D9  */
        /* I2SBCLK  - pin  D10 */
        /* I2SDIN       - pin  D11 */
        /* I2SSYNC  - pin  D12 */
        /* I2SMCLK    - pin  D13 */
        /* RX0          - pin  D14 */
        /* N/C          - pin  D15 */

        /* N/C          - pin D16 */
        /* N/C    - pin D17 */
        /* TX0          - pin D18 */
  gpio_map[WIFI_RESET]    = LF3000_GPIO_PORT_D | 19;
  gpio_map[CHG_FLT]               = LF3000_GPIO_PORT_D | 20;
        /* N/C     - pin D21 */
        /* SDCLK1_R      - pin D22 */
        /* SDCMD1_R      - pin D23 */

        /* SDDAT10_R     - pin D24 */
        /* SDDAT11_R     - pin D25 */
        /* SDDAT12_R     - pin D26 */
        /* SDDAT13_R     - pin D27 */
        /* VID0_0        - pin D28 */
        /* VID0_1        - pin D29 */
        /* VID0_2        - pin D30 */
        /* VID0_3        - pin D31 */

        /* GPIO PORT E */
        /* VID0_4        - pin E0  */
        /* VID0_5        - pin E1  */
        /* VID0_6        - pin E2  */
        /* VID0_7        - pin E3  */
        /* VID0_PCLK     - pin E4  */
  gpio_map[REAR_CAM_ENABLE_L] = LF3000_GPIO_PORT_E | 5;
  gpio_map[REAR_CAM_RESET_L]  = LF3000_GPIO_PORT_E | 6;
        /* N/C           - pin E7  */

        /* N/C     - pin E8  */
  gpio_map[DPAD_LEFT]   = LF3000_GPIO_PORT_E | 9;
  gpio_map[HEADPHONE_JACK] = LF3000_GPIO_PORT_E | 10;
        gpio_map[CHG_INT]        = LF3000_GPIO_PORT_E | 11;
        /* N/C           - pin E12 */
        /* N/C           - pin E13 */
  /* SCK_R         - pin E14 */
  gpio_map[LCD_SPI]               = LF3000_GPIO_PORT_E | 15;//FIXME it is supposed to be LCD_SCS_L(same?)

        /* N/C           - pin E16 */
        /* N/C           - pin E17 */
        /* N/C           - pin E18 */
        /* SDI           - pin E19 */
  gpio_map[BUTTON_ESC]             = LF3000_GPIO_PORT_E | 20;
  gpio_map[DPAD_DOWN]               = LF3000_GPIO_PORT_E | 21;
  gpio_map[LED_ENA]                 = LF3000_GPIO_PORT_E | 22;
  gpio_map[DPAD_RIGHT]              = LF3000_GPIO_PORT_E | 23;
  gpio_map[DPAD_UP]                 = LF3000_GPIO_PORT_E | 24;
        /* JTAG_TRST_L   - pin E25 */
        /* JTAG_TMS      - pin E26 */
        /* JTAG_TDI      - pin E27 */
        /* JTAG_TCK      - pin E28 */
        /* JTAG_TDO      - pin E29 */

  gpio_map[USBD_ID_SENSE]             = LF3000_GPIO_PORT_E | 30; //FIXME-it is supposed tobe USB_OTG_ID .. verify
  gpio_map[USB_POWER_FLT_L]           = LF3000_GPIO_PORT_E | 31; //FIXME-it s supposed to be USB_PWR_OK_L.. verify

        /* GPIO Port ALIVE */
        /* N/C        -  pin 0 */
        /* N/C        -  pin 1 */
        /* N/C        -  pin 2 */
        /* N/C        -  pin 3 */
        /* N/C        -  pin 4 */
        /* N/C        -  pin 5 */
        /* N/C        -  pin 6 */
        /* N/C        -  pin 7 */
} /* init_quito() */


int lf3000_gpio_map_valid = 0;

void lf3000_gpio_init_map(void)
{
	int i;

	/* mark all gpio mappings as invalid */
	for (i=0; i < (sizeof(gpio_map) / sizeof(gpio_map[0])); i++) {
		gpio_map[i] = LF3000_GPIO_PORT_NONE;
	}
	
	/* remap GPIOs based on Board Revision */
	switch(system_rev) {
		case LF3000_BOARD_BOGOTA:
		case LF3000_BOARD_BOGOTA_EXP_1:
		case LF3000_BOARD_BOGOTA_EXP_2:
		case LF3000_BOARD_BOGOTA_EXP_3:
		case LF3000_BOARD_BOGOTA_EXP_4:
		case LF3000_BOARD_BOGOTA_EXP_5:
		case LF3000_BOARD_BOGOTA_EXP_6:
			init_bogota();
			break;
			
		case LF3000_BOARD_CABO:
			init_cabo();
			break;

		case LF3000_BOARD_GLASGOW_ALPHA:
		case LF3000_BOARD_GLASGOW_BETA:
		case LF3000_BOARD_GLASGOW_FEP_984_666:
		case LF3000_BOARD_GLASGOW_FEP_984_800:
		case LF3000_BOARD_GLASGOW_TUNE_DDR1:
		case LF3000_BOARD_GLASGOW_TUNE_DDR2:
		case LF3000_BOARD_GLASGOW_TUNE_DDR3:
		case LF3000_BOARD_GLASGOW_TUNE_DDR4:
		case LF3000_BOARD_GLASGOW_TUNE_GPU1:
		case LF3000_BOARD_GLASGOW_TUNE_GPU2:
		case LF3000_BOARD_GLASGOW_TUNE_GPU3:
			init_glasgow();
			break;

		case LF3000_BOARD_R3K:
			init_r3k();
			break;

		case LF3000_BOARD_VTK:
			init_vtk();
			break;

		case LF3000_BOARD_XANADU:
		case LF3000_BOARD_XANADU_TI:
		case LF3000_BOARD_XANADU_TI_SS1:
		case LF3000_BOARD_XANADU_TI_SS2:
			init_xanadu();
			break;
			
    case LF3000_BOARD_QUITO:
      init_quito();
      break;

 		default:
			printk(KERN_ERR "%s GPIO mapping does not support " \
				"Board Revision = 0x%X\n",
				__func__, system_rev);
#if   defined(CONFIG_PLAT_NXP4330_BOGOTA)
			printk(KERN_WARNING "defaulting to BOGOTA GPIO mapping\n");
			init_bogota();
#elif defined(CONFIG_PLAT_NXP4330_CABO)
			printk(KERN_WARNING "defaulting to CABO GPIO mapping\n");
			init_cabo();
#elif defined(CONFIG_PLAT_NXP4330_GLASGOW)
			printk(KERN_WARNING "defaulting to GLASGOW GPIO mapping\n");
			init_glasgow();
#elif defined(CONFIG_PLAT_NXP4330_R3K)
			printk(KERN_WARNING "defaulting to R3K GPIO mapping\n");
			init_r3k();
#elif defined(CONFIG_PLAT_NXP4330_VTK)
			printk(KERN_WARNING "defaulting to VTK GPIO mapping\n");
			init_vtk();
#elif defined(CONFIG_PLAT_NXP4330_XANADU)
			printk(KERN_WARNING "defaulting to XANADU GPIO mapping\n");
			init_xanadu();
#elif defined(CONFIG_PLAT_NXP4330_QUITO)
      printk(KERN_WARNING "defaulting to QUITO GPIO mapping\n");
      init_quito();
#else
#error CONFIG_PLAT not set
#endif
			break;
	}
	lf3000_gpio_map_valid = 1;	/* initialized gpio_map[] */
}

extern unsigned lf3000_gpio_l2p( struct gpio_chip* chip, unsigned offset )
{
	/* ensure GPIO map initialized */
	if (!lf3000_gpio_map_valid)
		lf3000_gpio_init_map();

	/* ensure index is in range */
	if (offset < ARRAY_SIZE(gpio_map)) {
		return gpio_map[offset];
	} else {
		printk(KERN_WARNING "%s.%s:%d offset (%d) out of range\n",
			__FILE__, __func__, __LINE__, offset);
	}
	return LF3000_GPIO_PORT_NONE;
}

