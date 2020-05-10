/*------------------------------------------------------------------------------
 *
 *	Copyright (C) 2012
 *
 *	NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 *  AND	WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 *  BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 *  FOR A PARTICULAR PURPOSE.
 *
 *	Module     : System memory config
 *	Description:
 *	Author     : Platform Team
 *	Export     :
 *	History    :
 *	   2009/05/13 first implementation
 ------------------------------------------------------------------------------*/
#ifndef __CFG_GPIO_H__
#define __CFG_GPIO_H__

/*------------------------------------------------------------------------------
 *
 * Common GPIO settings
 *
 *	(GROUP_A)
 *
 *    0 bit                            4 bit                           8 bit         12 bit   		        16 bit
 *    | GPIO'A'OUTENB and GPIO'A'ALTFN | GPIO'A'OUT or GPIO'A'DETMODE0 | GPIO'A'PUENB| CLKPWR.PADSTRENGTHGPIO'A'|
 *
 -----------------------------------------------------------------------------*/

#if 0
#define PAD_GPIOA00_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD0 		    ]
#define PAD_GPIOA01_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD1 		    ]
#define PAD_GPIOA02_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD2 		    ]
#define PAD_GPIOA03_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD3 		    ]
#define PAD_GPIOA04_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD4 		    ]
#define PAD_GPIOA05_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD5 		    ]
#define PAD_GPIOA06_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD6 		    ]
#define PAD_GPIOA07_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD7 		    ]
#endif

#define PAD_GPIOA08_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: UART0_TX	    ]

#if 0
#define PAD_GPIOA18_    (PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_UP  | PAD_STRENGTH_2)	// [IN:   CIP_L             ]

#define PAD_GPIOA21_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: I2S_DOUT          ]
#define PAD_GPIOA22_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: I2S_BCLK          ]
#define PAD_GPIOA23_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: I2S_DIN           ]
#define PAD_GPIOA24_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: I2S_LRCK          ]
#define PAD_GPIOA25_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: I2S_MCLK          ]
#define PAD_GPIOA26_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SCL0              ]
#define PAD_GPIOA27_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SDA0              ]
#define PAD_GPIOA28_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SCL1              ]
#define PAD_GPIOA29_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SDA1              ]

/*------------------------------------------------------------------------------
 *	(GROUP_B)
 *
 *	0 bit				 4 bit				 8 bit	       12 bit			  16 bit
 *	| GPIO'B'OUTENB and GPIO'B'ALTFN | GPIO'B'OUT or GPIO'B'DETMODE0 | GPIO'B'PUENB| CLKPWR.PADSTRENGTHGPIO'B'|
 *
 -----------------------------------------------------------------------------*/
#define PAD_GPIOB00_  	(PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD0_CLK 	]
#define PAD_GPIOB01_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD0_CMD 	]
#define PAD_GPIOB02_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD0_DAT0	]
#define PAD_GPIOB03_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD0_DAT1	]
#define PAD_GPIOB04_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD0_DAT2	]
#define PAD_GPIOB05_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD0_DAT3	]
#define PAD_GPIOB06_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD1_CLK 	]
#define PAD_GPIOB07_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD1_CMD 	]
#define PAD_GPIOB08_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD1_DAT0	]
#define PAD_GPIOB09_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD1_DAT1	]
#define PAD_GPIOB10_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD1_DAT2	]
#define PAD_GPIOB11_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD1_DAT3	]
#define PAD_GPIOB12_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SPI0_FRM	]
#define PAD_GPIOB13_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SPI0_CLK	]
#define PAD_GPIOB14_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SPI0_RXD	]
#define PAD_GPIOB15_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SPI0_TXD	]
#define PAD_GPIOB16_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD8    	]
#define PAD_GPIOB17_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD9    	]
#define PAD_GPIOB18_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD10   	]
#define PAD_GPIOB19_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD11   	]
#define PAD_GPIOB20_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD12   	]
#define PAD_GPIOB21_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD13   	]
#define PAD_GPIOB22_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD14   	]
#define PAD_GPIOB23_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD15   	]
#define PAD_GPIOB24_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD16   	]
#define PAD_GPIOB25_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD17   	]
#define PAD_GPIOB26_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD18   	]
#define PAD_GPIOB27_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD19   	]
#define PAD_GPIOB28_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD20   	]
#define PAD_GPIOB29_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD21   	]
#define PAD_GPIOB30_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD22   	]
#define PAD_GPIOB31_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVD23   	]

/*------------------------------------------------------------------------------
 *	(GROUP_C)
 *
 *	0 bit				 4 bit				 8 bit	       12 bit			  16 bit
 *	| GPIO'C'OUTENB and GPIO'C'ALTFN | GPIO'C'OUT or GPIO'C'DETMODE0 | GPIO'C'PUENB| CLKPWR.PADSTRENGTHGPIO'C'|
 *
 -----------------------------------------------------------------------------*/

#define PAD_GPIOC19_    (PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [IN:   N/C               ]
#define PAD_GPIOC20_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: nNCS1             ]
#define PAD_GPIOC21_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VVSYNC_B          ]
#define PAD_GPIOC22_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VCLK_B            ]
#define PAD_GPIOC23_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VDVALID           ]
#define PAD_GPIOC24_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VHSYNC_B          ]
#define PAD_GPIOC25_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VID_B0            ]
#define PAD_GPIOC26_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VID_B1            ]
#define PAD_GPIOC27_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VID_B2            ]
#define PAD_GPIOC28_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VID_B3            ]
#define PAD_GPIOC29_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VID_B4            ]
#define PAD_GPIOC30_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VID_B5            ]
#define PAD_GPIOC31_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VID_B6            ]

/*------------------------------------------------------------------------------
 *	(GROUP_D)
 *
 *	0 bit                            4 bit                           8 bit         12 bit                     16 bit
 *	| GPIO'C'OUTENB and GPIO'C'ALTFN | GPIO'C'OUT or GPIO'C'DETMODE0 | GPIO'C'PUENB| CLKPWR.PADSTRENGTHGPIO'C'|
 *
 -----------------------------------------------------------------------------*/
#define PAD_GPIOD00_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VID_B7            ]
#define PAD_GPIOD01_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD8               ]
#define PAD_GPIOD02_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD9               ]
#define PAD_GPIOD03_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD10              ]
#define PAD_GPIOD04_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD11              ]
#define PAD_GPIOD05_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD12              ]
#define PAD_GPIOD06_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD13              ]
#define PAD_GPIOD07_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD14              ]
#define PAD_GPIOD08_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SD15              ]
#endif

#define PAD_GPIOD09_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA8               ]
#define PAD_GPIOD10_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA9               ]
#define PAD_GPIOD11_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA10              ]
#define PAD_GPIOD12_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA11              ]
#define PAD_GPIOD13_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA12              ]
#define PAD_GPIOD14_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA13              ]
#define PAD_GPIOD15_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA14              ]
#define PAD_GPIOD16_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA15              ]
#define PAD_GPIOD17_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA16              ]
#define PAD_GPIOD18_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA17              ]
#define PAD_GPIOD19_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: SA18              ]

#if 0
#define PAD_GPIOD20_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVCLK             ]
#define PAD_GPIOD21_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PDE               ]
#define PAD_GPIOD22_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PHSYNC            ]
#define PAD_GPIOD23_    (PAD_MODE_ALT1 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT1: PVSYNC            ]
#define PAD_GPIOD24_    (PAD_MODE_ALT2 | PAD_OUT_LOWLEVEL  | PAD_PULL_OFF | PAD_STRENGTH_2)	// [ALT2: VCLK27            ]
#endif

/*------------------------------------------------------------------------------
 *	(GROUPALV)
 *	0		      4				    8	     12
 *	| MODE(IN/OUT/DETECT) | ALIVE OUT or ALIVE DETMODE0 | PullUp |
 *
 -----------------------------------------------------------------------------*/
#define	PAD_GPIOALV0_  	(PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_UP)		// [IN: N/C ]
#define	PAD_GPIOALV1_  	(PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_UP)		// [IN: N/C ]
#define	PAD_GPIOALV2_  	(PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_UP)		// [IN: N/C ]
#define	PAD_GPIOALV3_  	(PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_UP)		// [IN: N/C ]
#define	PAD_GPIOALV4_  	(PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_UP)		// [IN: N/C ]
#define	PAD_GPIOALV5_  	(PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_UP)		// [IN: N/C ]
#define	PAD_GPIOALV6_  	(PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_UP)		// [IN: N/C ]
#define	PAD_GPIOALV7_  	(PAD_MODE_IN   | PAD_OUT_LOWLEVEL  | PAD_PULL_UP)		// [IN: N/C ]

/*------------------------------------------------------------------------------
 *	(BUS signal)		               [BIT]: 0		8	   12
 *					      	      | BUS Pad | Strength |
 *
 -----------------------------------------------------------------------------*/
#define	PAD_BUS_STATIC_CNTL_ 	(NX_CLKPWR_BUSPAD_STATIC_CNTL	| PAD_STRENGTH_4)
#define	PAD_BUS_STATIC_ADDR_ 	(NX_CLKPWR_BUSPAD_STATIC_ADDR	| PAD_STRENGTH_4)
#define	PAD_BUS_STATIC_DATA_ 	(NX_CLKPWR_BUSPAD_STATIC_DATA	| PAD_STRENGTH_4)
#define	PAD_BUS_HSYNC_ 		(NX_CLKPWR_BUSPAD_HSYNC		| PAD_STRENGTH_2)
#define	PAD_BUS_VSYNC_ 		(NX_CLKPWR_BUSPAD_VSYNC		| PAD_STRENGTH_2)
#define	PAD_BUS_DE_ 		(NX_CLKPWR_BUSPAD_DE		| PAD_STRENGTH_2)

/*------------------------------------------------------------------------------
 *	GPIO I2C
 */
#define	CFG_IO_I2C0_SCL		I2C_SCL0		/* GPIO */
#define	CFG_IO_I2C0_SDA		I2C_SDA0		/* GPIO */
#define	CFG_IO_I2C1_SCL		I2C_SCL1		/* GPIO */
#define	CFG_IO_I2C1_SDA		I2C_SDA1		/* GPIO */
#define	CFG_IO_I2C2_SCL		I2C_SCL2		/* GPIO */
#define	CFG_IO_I2C2_SDA		I2C_SDA2		/* GPIO */

/*------------------------------------------------------------------------------
 *	GPIO SPI
 */
#define CFG_PIO_SPI0_FRAME			SPI0_FRAME
#define CFG_PIO_SPI0_CLOCK			SPI0_CLOCK
#define CFG_PIO_SPI0_RX				SPI0_RX
#define CFG_PIO_SPI0_TX				SPI0_TX

/*------------------------------------------------------------------------------
 *      Specific Board Defines
 */

#include <../../bogota/include/cfg_gpio_bogota_alpha.h>
#include <../../cabo/include/cfg_gpio_cabo_alpha.h>
#include <../../glasgow/include/cfg_gpio_glasgow_alpha.h>
#include <../../glasgow/include/cfg_gpio_glasgow_beta.h>
#include <../../r3k/include/cfg_gpio_r3k.h>
#include <../../xanadu/include/cfg_gpio_xanadu_alpha.h>
#include <../../quito/include/cfg_gpio_quito_alpha.h>

#endif	/* __CFG_GPIO_H__ */
