/*------------------------------------------------------------------------------
 *
 *	Copyright (C) 2009 Nexell Co., Ltd All Rights Reserved
 *	Nexell Co. Proprietary & Confidential
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
#ifndef __CFG_MEM_H__
#define __CFG_MEM_H__

/*------------------------------------------------------------------------------
 * 	 System memory map
 */
#define CFG_MEM_NUMBER_OF_BANKS			1	/* memory calculations assume 1 memory bank */
							/* maybe definition of CONFIG_ARM_NR_BANKS should tie in here */

#define	CFG_MEM_PHY_SYSTEM_BASE			SZ_1G	/* System, must be at an even 2MB boundary (head.S) */

/*------------------------------------------------------------------------------
 *   DMA zone, if not defined DMA default size is 2M
 */
#define CFG_MEM_PHY_DMAZONE_SIZE		SZ_16M		/* 16 MB DMA zone */


#if defined(CONFIG_PLAT_NXP4330_BOGOTA)
#define	CFG_MEM_PHY_SYSTEM_SIZE			( SZ_512M - CFG_MEM_PHY_DMAZONE_SIZE )	/* 512 MB - 16 MB DMA */
#elif defined(CONFIG_PLAT_NXP4330_CABO)
#define	CFG_MEM_PHY_SYSTEM_SIZE			( SZ_512M - CFG_MEM_PHY_DMAZONE_SIZE )	/* 512 MB - 16 MB DMA */
#elif defined(CONFIG_PLAT_NXP4330_GLASGOW)
#define	CFG_MEM_PHY_SYSTEM_SIZE			( SZ_1G - CFG_MEM_PHY_DMAZONE_SIZE )	/* 1024 MB - 16 MB DMA */
#elif defined(CONFIG_PLAT_NXP4330_XANADU)
#define	CFG_MEM_PHY_SYSTEM_SIZE			( SZ_1G - CFG_MEM_PHY_DMAZONE_SIZE )	/* 1024 MB - 16 MB DMA */
#elif defined(CONFIG_PLAT_NXP4330_QUITO)
#define CFG_MEM_PHY_SYSTEM_SIZE     ( SZ_512M - CFG_MEM_PHY_DMAZONE_SIZE )  /* 512 MB - 16 MB DMA */
#else
#define	CFG_MEM_PHY_SYSTEM_SIZE			( SZ_256M - CFG_MEM_PHY_DMAZONE_SIZE )	/* small size so system boots */
#endif


#endif /* __CFG_MEM_H__ */
