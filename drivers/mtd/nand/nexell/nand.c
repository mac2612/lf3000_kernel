/*
 * (C) Copyright 2010
 * KOO Bon-Gyu, Nexell Co, <freestyle@nexell.co.kr>
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/mach-types.h>
#include <linux/bitops.h>

#include <linux/sched.h>
#include <asm/stacktrace.h>
#include <asm/traps.h>
#include <asm/unwind.h>

#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>

#include "nand_ecc.h"
#include "../../mtdcore.h"
#include <linux/gpio.h>
#include <linux/lf3000/gpio.h>
#include "../../ubi/ubi-media.h"
#define NAND_CART_DETECT_LEVEL 0

#if	(0)
#define TM_DBGOUT(msg...)		{ printk(KERN_INFO "nand: " msg); }
#else
#define TM_DBGOUT(msg...)		do {} while (0)
#endif

#if	(0)
#define DBGOUT(msg...)		{ printk(KERN_INFO "nand: " msg); }
#else
#define DBGOUT(msg...)		do {} while (0)
#endif

#define ERROUT(msg...)		{ 					\
		printk(KERN_ERR "ERROR: %s, %s line %d: \n",		\
			__FILE__, __FUNCTION__, __LINE__),	\
		printk(KERN_ERR msg); }

#define CLEAR_RnB(r)							\
	r = NX_MCUS_GetInterruptPending(0);			\
	if (r) {									\
		NX_MCUS_ClearInterruptPending(0); 		\
		NX_MCUS_GetInterruptPending(0); 		\
	}

#define CHECK_RnB()	NX_MCUS_GetInterruptPending(0)

#define CLEAR_ECCIRQ(r)							\
	r = NX_MCUS_GetECCInterruptPending(0);			\
	if (r) {									\
		NX_MCUS_ClearECCInterruptPending(0); 		\
		NX_MCUS_GetECCInterruptPending(0); 		\
	}

#define CHECK_ECCIRQ()	NX_MCUS_GetECCInterruptPending(0)

#if !defined (CONFIG_SYS_NAND_MAX_CHIPS)
#define CONFIG_SYS_NAND_MAX_CHIPS   1
#endif

/*------------------------------------------------------------------------------
 * nand interface
 */
static void nand_select_chip(struct mtd_info *mtd, int chipnr)
{
	DBGOUT("%s, chipnr=%d\n", __func__, chipnr);

#if defined(CFG_NAND_OPTIONS)
	struct nand_chip *chip = mtd->priv;
	chip->options |= CFG_NAND_OPTIONS;
#endif

	if (chipnr > 4) {
		ERROUT("not support nand chip index %d\n", chipnr);
		return;
	}

	if (-1 == chipnr) {
		NX_MCUS_SetNFCSEnable(CFALSE);		// nand chip select control disable
	} else {
		NX_MCUS_SetNFBank(chipnr);
		NX_MCUS_SetNFCSEnable(CTRUE);
	}
}

#define MASK_CLE	0x10	/* NFCM   + 2C00_0000 */
#define MASK_ALE	0x18	/* NFADDR + 2C00_0000 */

static void nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	void __iomem* addr = chip->IO_ADDR_W;
	int ret = 0;
	unsigned long flags;
	struct nxp_nand *nxp = mtd_to_nxp(mtd);

	if (cmd == NAND_CMD_NONE)
		return;

	spin_lock_irqsave (&nxp->cmdlock, flags);

	if (ctrl & NAND_CLE)
	{
		if (cmd != NAND_CMD_STATUS &&
			cmd != NAND_CMD_READID &&
			cmd != NAND_CMD_RESET)
			CLEAR_RnB(ret);

		//printk ("  [%s:%d] command: %02x\n", __func__, __LINE__, (unsigned char)cmd);
		writeb(cmd, addr + MASK_CLE);
	}
	else if (ctrl & NAND_ALE)
	{
		//printk ("  [%s:%d] address: %02x\n", __func__, __LINE__, (unsigned char)cmd);
		writeb(cmd, addr + MASK_ALE);
	}

	spin_unlock_irqrestore (&nxp->cmdlock, flags);
}

struct nand_timings {
	uint32_t tACS;			// tACS
	uint32_t tCAH;			// tCAH
	uint32_t tCOS;			// tCOS
	uint32_t tOCH;			// tOCH
	uint32_t tACC;			// tACC
};

struct nand_timing_mode5_params {
	uint32_t tCS;
	uint32_t tCH;
	uint32_t tCLS_tALS;
	uint32_t tCLH_tALH;
	uint32_t tWP;
	uint32_t tWH;
	uint32_t tWC;
	uint32_t tDS;
	uint32_t tDH;
	uint32_t tCEA;		// max
	uint32_t tREA;		// max
	uint32_t tRP;
	uint32_t tREH;
	uint32_t tRC;
	uint32_t tCOH;
} NAND_TM5_PARAM[6] =
{//	tCS tCH tCLS tCLH tWP tWH  tWC tDS tDH tCEA tREA tRP tREH  tRC tCOH
	{70, 20,  50,  20, 50, 30, 100, 40, 20, 100,  40, 50,  30, 100,  0},		// mode 0
	{35, 10,  25,  10, 25, 15,  45, 20, 10,  45,  30, 25,  15,  50, 15},		// mode 1
	{25, 10,  15,  10, 17, 15,  35, 15,  5,  30,  25, 17,  15,  35, 15},		// mode 2
	{25,  5,  10,   5, 15, 10,  30, 10,  5,  25,  20, 15,  10,  30, 15},		// mode 3
	{20,  5,  10,   5, 12, 10,  25, 10,  5,  25,  20, 12,  10,  25, 15},		// mode 4
	{15,  5,  10,   5, 10,  7,  20,  7,  5,  25,  16, 10,   7,  20, 15}			// mode 5
};



/**
 * nand_calc_timing_mode - calculate based on the timing mode
 * @mode: timing mode. (0 ~ 5)
 * @clkhz: BCLK clock rate in hz
 * @timings: value to set the timing register [Returns]
 */
static int nand_calc_timing_mode (uint32_t mode, uint32_t clkhz,
			struct nand_timings *timings)
{
	uint32_t nclk; 
	uint32_t tCS, tCH, tCLS, tCLH, tWP, tWH, tWC, tDS, tDH, tCEA, tREA, tRP, tREH, tRC, tCOH;
	uint32_t tRCS, tWCS0, tWCS1, tRACC, tWACC, tRCH, tWCH0, tWCH1;
	uint32_t tCOS, tACC, tOCH;
	struct nand_timing_mode5_params *pntmp;

	// error check
	if(mode >= 6)
		mode = 0;
	if(clkhz < 1000000)		// BCLK is minimum > 1MHz
		clkhz = 1000000;
	if(!timings)
		return -1;

	// varient convertion
	nclk = 1000000000/(clkhz/1000);	// convert to pico second
	pntmp = &NAND_TM5_PARAM[mode];
	tCS = pntmp->tCS*1000000;
	tCH = pntmp->tCH*1000000;
	tCLS = pntmp->tCLS_tALS*1000000;
	tCLH = pntmp->tCLH_tALH*1000000;
	tWP = pntmp->tWP*1000000;
	tWH = pntmp->tWH*1000000;
	tWC = pntmp->tWC*1000000;
	tDS = pntmp->tDS*1000000;
	tDH = pntmp->tDH*1000000;
	tCEA = pntmp->tCEA*1000000;
	tREA = pntmp->tREA*1000000;
	tRP = pntmp->tRP*1000000;
	tREH = pntmp->tREH*1000000;
	tRC = pntmp->tRC*1000000;
	tCOH = pntmp->tCOH*1000000;

	TM_DBGOUT("nclk: %u, mode: %u\n", nclk, mode);
	TM_DBGOUT("tCS: %u, tCH: %u tCLS: %u, tCLH: %u, tWP: %u, tWH: %u, tWC: %u\n",
		tCS, tCH, tCLS, tCLH, tWP, tWH, tWC);
	TM_DBGOUT("tDS: %u, tDH: %u, tCEA: %u, tREA: %u,\n  tRP: %u, tREH: %u, tRC: %u, tCOH: %u\n", 
		tDS, tDH, tCEA, tREA, tRP, tREH, tRC, tCOH);

	// timing calculation
	tRCS = (tCEA-tREA)/nclk;	//(tCEA-tREA)/nclk
	tWCS0 = (tCS-tWP)/nclk;		//(tCS-tWP)/nclk
	tWCS1 = (tCLS-tWP)/nclk;	//(tCLS-tWP)/nclk
	tRACC = ((tREA+nclk*2000)>tRP?(tREA+nclk*2000):tRP)/nclk;	//MAX(tREA+nclk*2, tRP)/nclk
	tWACC = ((tWP>tDS)?tWP:tDS)/nclk;	//MAX(tWP,tDS)/nclk
	tRCH = ((tRC-tRP)>tREH?(tRC-tRP):tREH)/nclk-tRCS;	//MAX(tRC-tRP,tREH)/nclk-tRCS
	tWCH0 = ((tWC-tWP)>tWH?(tWC-tWP):tWH)/nclk-(tWCS0>tWCS1?tWCS0:tWCS1);//MAX(tWC-tWP, tWH)/nclk - MAX(tWCS0, tWCS1)
	tWCH1 = ((tCH>tCLH?tCH:tCLH)>tDH?(tCH>tCLH?tCH:tCLH):tDH)/nclk;		//MAX(tCH,tCLH,tDH)/nclk

	TM_DBGOUT("tRCS: %u, tWCS0: %u, tWCS1: %u, tRACC: %u, tWACC: %u, tRCH: %u, tWCH0: %u, tWCH1: %u\n",
		tRCS, tWCS0, tWCS1, tRACC, tWACC, tRCH, tWCH0, tWCH1);

	// convertion to clock base asynchronous nand controller state machine
	tCOS = (tRCS>tWCS0?tRCS:tWCS0)>tWCS1?(tRCS>tWCS0?tRCS:tWCS0):tWCS1;//MAX(tRCS, tWCS0, tWCS1);
	tACC = tRACC>tWACC?tRACC:tWACC;		//MAX(tRACC, tWACC);
	tOCH = (tRCH>tWCH0?tRCH:tWCH0)>tWCH1?(tRCH>tWCH0?tRCH:tWCH0):tWCH1;//MAX(tRCH, tWCH0, tWCH1);

	TM_DBGOUT("tCOS: %u, tACC: %u, tOCH: %u\n", tCOS, tACC, tOCH);

	// convert to register value
	tCOS += 999;	// round up tCOS
	tACC += 999;	// round up tACC
	tOCH += 999;	// round up tOCH

	// fillup paramter	
	timings->tACS = 0;
	timings->tCOS = tCOS/1000;
	timings->tACC = tACC/1000;
	timings->tOCH = tOCH/1000;
	timings->tCAH = 0;

	TM_DBGOUT("  fill - tCOS: %u, tACC: %u, tOCH: %u\n", tCOS/1000, tACC/1000, tOCH/1000);

	return 0;
}

static int nand_onfi_timing_set(struct mtd_info *mtd, uint32_t mode)
{
	struct clk *clk;
	uint32_t clkhz;
	struct nand_timings tmgs;
	int ret;


	clk = clk_get (NULL, CORECLK_NAME_BCLK), clkhz = clk_get_rate(clk), clk_put(clk);
	TM_DBGOUT(" BCLK: %u HZ\n", clkhz);

	// setting - nand flash

	// setting - nand controller timming
	NX_MCUS_GetNANDBUSConfig
	(
		0,
		&tmgs.tACS,
		&tmgs.tCAH,
		&tmgs.tCOS,
		&tmgs.tOCH,
		&tmgs.tACC
	);
	TM_DBGOUT("[BEFORE]  tACS: %u, tCAH: %u, tCOS: %u, tOCH: %u, tACC: %u\n", 
		tmgs.tACS, tmgs.tCAH, tmgs.tCOS, tmgs.tOCH, tmgs.tACC);

	ret = nand_calc_timing_mode (mode, clkhz, &tmgs);
	if (ret < 0)
		return -1;

	NX_MCUS_SetNANDBUSConfig
	(
		0,
		tmgs.tACS,
		tmgs.tCAH,
		tmgs.tCOS,
		tmgs.tOCH,
		tmgs.tACC
	);

	NX_MCUS_GetNANDBUSConfig
	(
		0,
		&tmgs.tACS,
		&tmgs.tCAH,
		&tmgs.tCOS,
		&tmgs.tOCH,
		&tmgs.tACC
	);
	TM_DBGOUT("[AFTER]  tACS: %u, tCAH: %u, tCOS: %u, tOCH: %u, tACC: %u\n", 
		tmgs.tACS, tmgs.tCAH, tmgs.tCOS, tmgs.tOCH, tmgs.tACC);


	return 0;
}

/* timing set */
static int nexell_nand_timing_set(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	uint32_t ret, mode;

	ret = onfi_get_async_timing_mode(chip);
	if (ret == ONFI_TIMING_MODE_UNKNOWN)
	{
#ifdef CONFIG_NXP4330_LEAPFROG
		printk(KERN_INFO "nexell_nand_timing_set use default timing\n");
		return 0;
#endif
		NX_MCUS_SetNANDBUSConfig
		(
			 0, /* NF */
			 CFG_SYS_NAND_TACS,              // tACS  ( 0 ~ 3 )
			 CFG_SYS_NAND_TCAH,              // tCAH  ( 0 ~ 3 )
			 CFG_SYS_NAND_TCOS,              // tCOS  ( 0 ~ 3 )
			 CFG_SYS_NAND_TCOH,              // tCOH  ( 0 ~ 3 )
			 CFG_SYS_NAND_TACC               // tACC  ( 1 ~ 16)
		);

		return 0;
	}

	mode = fls(ret) - 1;
	TM_DBGOUT("ONFI TIMING MODE (%d) \n", mode);

	nand_onfi_timing_set (mtd, mode);

	return 0;
}


static int nand_dev_ready(struct mtd_info *mtd)
{
	int ret;
	CLEAR_RnB(ret);
	DBGOUT("[%s, RnB=%d]\n", ret?"READY":"BUSY", NX_MCUS_IsNFReady());
	return ret;
}

static void nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	readsl(chip->IO_ADDR_R, buf, (len >> 2));
	if (len & 3)
		readsb(chip->IO_ADDR_R, buf + (len & ~0x3), (len & 3));
}

static void nand_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	writesl(chip->IO_ADDR_W, buf, (len >> 2));
	if (len & 3)
		writesb(chip->IO_ADDR_W, buf + (len & ~0x3), (len & 3));
}

#if !defined (CONFIG_NXP4330_LEAPFROG)
/*
 * Enable NAND write protect
 */
static void nxp_wp_enable(void)
{
	nxp_soc_gpio_set_out_value(CFG_IO_NAND_nWP, 0);
}

/*
 * Disable NAND write protect
 */
static void nxp_wp_disable(void)
{
	nxp_soc_gpio_set_out_value(CFG_IO_NAND_nWP, 1);
}
#endif

static void nand_dev_init(struct mtd_info *mtd)
{
#if !defined (CONFIG_NXP4330_LEAPFROG)
	unsigned int io = CFG_IO_NAND_nWP;
#else
#warning "NAND WP line disabled in LeapFrog builds"
#endif

	NX_MCUS_SetAutoResetEnable(CTRUE);
	NX_MCUS_ClearInterruptPending(0);
	NX_MCUS_SetInterruptEnable(0, CFALSE); /* polling */
#ifdef CFG_NAND_ECCIRQ_MODE
	NX_MCUS_SetECCInterruptEnable(0, CTRUE);
#else
	NX_MCUS_SetECCInterruptEnable(0, CFALSE);
#endif
	NX_MCUS_SetNFBank(0);
	NX_MCUS_SetNFCSEnable(CFALSE);

#if !defined (CONFIG_NXP4330_LEAPFROG)
	nxp_soc_gpio_set_out_value(io, 0);
	nxp_soc_gpio_set_io_dir(io, 1);
	nxp_soc_gpio_set_io_func(io, nxp_soc_gpio_get_altnum(io));
	nxp_soc_gpio_set_out_value(io, 1);
#endif
}


#if defined (CONFIG_MTD_NAND_ECC_BCH)
static uint8_t *verify_page;
static int nand_bch_write_page(struct mtd_info *mtd, struct nand_chip *chip,
			   const uint8_t *buf, int oob_required, int page, int cached, int raw)
{
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	int ret = 0;
	struct nxp_nand *nxp = mtd_to_nxp(mtd);
#endif
	int status;

	DBGOUT("%s page %d, raw=%d\n", __func__, page, raw);
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

	/* not verify */
	if (raw)
		chip->ecc.write_page_raw(mtd, chip, buf, oob_required);
	else
		chip->ecc.write_page(mtd, chip, buf, oob_required);

	/*
	 * Cached progamming disabled for now, Not sure if its worth the
	 * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
	 */
	cached = 0;

	if (!cached || !(chip->options & NAND_CACHEPRG)) {

		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
		/*
		 * See if operation failed and additional status checks are
		 * available
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_WRITING, status,
					       page);

		if (status & NAND_STATUS_FAIL)
			return -EIO;
	} else {
		chip->cmdfunc(mtd, NAND_CMD_CACHEDPROG, -1, -1);
		status = chip->waitfunc(mtd, chip);
	}

#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	if (raw)
		return 0;

	/* Send command to read back the data */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
	chip->ecc.read_page(mtd, chip, (uint8_t *)nxp->verify_page, oob_required, page);
	if (ret < 0)
		return -EIO; //		return ret;

	if (memcmp (nxp->verify_page, buf, mtd->writesize))
	{
		ERROUT ("%s fail verify %d page\n", __func__, page);
		return -EIO;
	}

	chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
#endif
	return 0; // mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0
}

static int nand_ecc_layout_swbch(struct mtd_info *mtd)
{
	struct nxp_nand *nxp = mtd_to_nxp(mtd);
	struct nand_chip *chip = mtd->priv;
	struct nand_ecclayout *layout = &chip->ecc.layout[0];
	struct nand_oobfree *oobfree  = layout->oobfree;
	int ecctotal = chip->ecc.total;
	int oobsize	 = mtd->oobsize;

	printk(KERN_INFO "sw bch ecc %d bit, oob %2d, bad '0,1', ecc %d~%d (%d), free %d~%d (%d) ",
		ECC_BCH_BITS, oobsize, oobfree->offset+oobfree->length, oobsize-1, ecctotal,
		oobfree->offset, oobfree->length + 1, oobfree->length);

	verify_page = kzalloc(mtd->writesize, GFP_KERNEL);
	if (!verify_page)
		return -ENOMEM;

	return 0;
}
#endif

static int nand_ecc_layout_check(struct mtd_info *mtd)
{
	int ret = 0;
#if defined (CONFIG_MTD_NAND_ECC_HW)
	ret = nand_ecc_layout_hwecc(mtd);
#elif defined (CONFIG_MTD_NAND_ECC_BCH)
	ret = nand_ecc_layout_swbch(mtd);
#endif
	return ret;
}

static int nand_resume(struct platform_device *pdev)
{
	struct nxp_nand  *nxp  = platform_get_drvdata(pdev);
	struct mtd_info  *mtd  = &nxp->mtd;
	struct nand_chip *chip = mtd->priv;

	PM_DBGOUT("+%s\n", __func__);

	/* Select the device */
	nand_dev_init(mtd);
	chip->select_chip(mtd, 0);

#if defined (CONFIG_MTD_NAND_ECC_HW)
	nand_hw_ecc_init_device(mtd, ECC_HW_BITS);
#endif

	nexell_nand_timing_set(mtd);
	/*
	 * Reset the chip, required by some chips (e.g. Micron MT29FxGxxxxx)
	 * after power-up
	 */
	chip->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);
	PM_DBGOUT("-%s\n", __func__);
	return 0;
}

#ifdef CFG_NAND_ECCIRQ_MODE
static irqreturn_t nxp_irq(int irq, void *_nxp)
{
	struct nxp_nand *nxp = _nxp;
	struct nand_chip *nand_chip = &nxp->chip;
	irqreturn_t result = IRQ_NONE;

	int r;

	if (CHECK_RnB() == CTRUE) {
		// printk (" -- RnB\n");
		CLEAR_RnB(r);

		result = IRQ_HANDLED;
	}
	else if (CHECK_ECCIRQ() == CTRUE) {
		// printk (" -- ECC\n");
		CLEAR_ECCIRQ(r);

		result = IRQ_HANDLED;
	}
	else {
		// printk (" ?? WHAT\n");
		result = IRQ_NONE;
	}

	nxp->irqcond = 1;
	wake_up(&nand_chip->controller->wq);

	return result;
}
#endif

#ifdef CFG_NAND_ECCIRQ_MODE
uint32_t wait_for_location_done(struct mtd_info *mtd)
{
	struct nxp_nand *nxp = mtd_to_nxp(mtd);
	struct nand_chip *nand_chip = &nxp->chip;
	unsigned long timeout;

	nxp->irqcond = 0;
	timeout = wait_event_timeout(nand_chip->controller->wq,
			nxp->irqcond, /* !CHECK_ECCIRQ(), */
			msecs_to_jiffies(1000));
		
	if (timeout == 0) {
		/* timeout */
		printk(KERN_ERR "timeout occurred\n");
		return -ETIMEDOUT;
	}
	return 0;
}
#endif

static int nand_remove(struct platform_device *pdev)
{
	struct nxp_nand  *nxp  = platform_get_drvdata(pdev);
	struct mtd_info  *mtd  = &nxp->mtd;
	int ret = 0;

	nand_release(mtd);
	if (nxp->irq)
		free_irq(nxp->irq, nxp);
#ifdef CONFIG_NAND_RANDOMIZER
	if (nxp->randomize_buf)
		kfree (nxp->randomize_buf);
#endif
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	if (nxp->verify_page)
		kfree (nxp->verify_page);
#endif
#if defined (CONFIG_MTD_NAND_ECC_HW)
	ret = nand_hw_ecc_fini_device(mtd);
#endif
	kfree(nxp);

	return 0;
}

/* This routine examines the nand's ID bytes and extracts from them and saves
 * lots of information about the nand's geometry and the amount of error
 * correction that it needs.
 */
static int lf2000_nand_init_size(struct mtd_info *mtd,
				 struct nand_chip *chip,
				 u8 *id_data)
{
	int extid;
	int busw;

	printk(KERN_INFO "%s: id bytes: "
				 "%02x %02x %02x %02x %02x %02x %02x %02x\n",
		__FUNCTION__,
		id_data[0], id_data[1], id_data[2], id_data[3],
		id_data[4], id_data[5], id_data[6], id_data[7]);

		/* The 3rd id byte holds MLC / multichip data */
	chip->cellinfo = id_data[2];
		/* The 4th id byte has sizes of page, oob, and eraseblock */
	extid = id_data[3];

		/* Leapfrog uses at least 2 NAND devices that emit 6 ID bytes
		 * which have non-traditional field definitions.
		 * One of these is a Samsung K9GAG08U0E.
 		 * The other is a Hynix H27UAG8T2B.
		 *
		 * We first check if the 3rd ID byte indicates that a cell contains
		 * more than 1 bit (2 levels), if the 8 ID bytes in id_data[] start to
		 * repeat after 6 bytes (i.e., if id_data[0] == id_data[6] and
		 * id_data[1] == id_data[7]).
		 * If not, or if the manufacturer is not Samsung or Hynix, we interpret
		 * the ID bytes in the traditional way.
		 * If they do, we interpret a Samsung nand's fields one way and Hynix
		 * another way.
		 */
	if (   (chip->cellinfo & NAND_CI_CELLTYPE_MSK)
		&& (id_data[5] != 0x00)
		&& ((id_data[0] == id_data[6]) && (id_data[1] == id_data[7]))
		&& (   (id_data[0] == NAND_MFR_SAMSUNG)
			|| (id_data[0] == NAND_MFR_HYNIX)) )
	{
		int ecc_level;

			/* no subpage writes on these MLC NANDs */
		//chip->options	   |= NAND_NO_SUBPAGE_WRITE;
		//chip->block_bad		= nand_block_bad_first_or_last;
		//chip->block_markbad	= nand_block_markbad_first_and_last;
		//chip->scan_bbt		= nand_big_mlc_bbt;

		/* The 5th ecc byte indicates required ECC level */
		ecc_level = (id_data[4] >> 4) & 0x07;

			/* Calc pagesize */
		mtd->writesize = 2048 << (extid & 0x03);
		if (id_data[0] == NAND_MFR_SAMSUNG)
		{
			/* Calc oobsize */
			/* NOTE: oobsize is indicated by bits 6, 3, and 2
			 * of the 4th ID byte:
			 * bits 7654.3210   HEX	  Description
			 *		x0xx.00xx  0x00 : reserved
			 *		x0xx.01xx  0x04 : 128 bytes
			 *		x0xx.10xx  0x08 : 218 bytes
			 *		x0xx.11xx  0x0C : 400 bytes
			 *		x1xx.00xx  0x40 : 436 bytes
			 *		x1xx.01xx  0x44 : 512 bytes
			 *		x1xx.10xx  0x48 : 640 bytes
			 *		x1xx.11xx  0x4C : reserved
			 *
			 * clear the unused bits, leaving only 6, 3, and 2
			 */
			switch (extid & 0x4C) {
			case 0x04:	mtd->oobsize = 128;	break;
			case 0x08: 	mtd->oobsize = 218;	break;
			case 0x0C:	mtd->oobsize = 400;	break;
			case 0x40:	mtd->oobsize = 436;	break;
			case 0x44:	mtd->oobsize = 512;	break;
			case 0x48:	mtd->oobsize = 640;	break;
			case 0x00:	/* reserved */
			case 0x4C:	/* reserved */
			default:	mtd->oobsize = 0;	break;
			}
				/* Calc blocksize
				 * NOTE: blocksize is indicated by bits 7, 5, and 4
				 * of the 4th ID byte:
				 *				000: 128K
				 *				001: 256K
				 *				010: 512K
				 *				011: 1M
				 * 100,101,110, 111: reserved
				 * This code treats all the reserved values as though their
				 * msb is 0.  Not exactly right, but what else to do?
				 */
			mtd->erasesize = (128 * 1024) << ((extid >> 4) & 0x03);
				/* Get the required ecc strength */
			switch (ecc_level) {
			case 0:		// 1 bit / 512 bytes
// TODO: FIXME: do we need to have a nand_sw_ecc_init(chip) routine
//				to init all the function pointers and other fields of chip?
				chip->ecc.mode = NAND_ECC_SOFT;
				printk(KERN_INFO "NAND ecc: Software \n");
				break;
#if defined (CONFIG_MTD_NAND_ECC_HW)
			case 1:		// 2 bits / 512 bytes
				nand_hw_ecc_init_device(mtd, 4);
				break;
			case 2:		// 4 bits / 512 bytes
				nand_hw_ecc_init_device(mtd, 4);
				break;
			default:
				// TODO: FIXME: decide what to do for default case
				// 8-bit ecc uses 13 ecc bytes per 512 bytes of data.
				// If the spare area has at least 16 bytes per 512
				// bytes of data, the ecc bytes will fit in the oob.
			case 3:		// 8 bits / 512 bytes
				nand_hw_ecc_init_device(mtd, 8);
				break;
			case 4:		// 16 bits / 512 bytes
				nand_hw_ecc_init_device(mtd, 16);
				break;
			case 5:		// 24 bits / 1024 bytes
				nand_hw_ecc_init_device(mtd, 24);
				break;
#else
			default:
				chip->ecc.mode = NAND_ECC_SOFT;
				printk(KERN_INFO "NAND ecc: Software \n");
				break;
#endif
			}
		}
		else {	/* Hynix */
			/* Calc oobsize */
			/* NOTE: oobsize is indicated by bits 6, 3, and 2
			 * of the 4th ID byte:
			 * (see the Hynix H27UAG8T2B datasheet, p20)
			 * bits 7654.3210   HEX	  Description
			 *		x0xx.00xx  0x00 : 128 bytes
			 *		x0xx.01xx  0x04 : 224 bytes
			 *		x0xx.10xx  0x08 : 448 bytes
			 *		x0xx.11xx  0x0C : reserved
			 *		x1xx.00xx  0x40 : reserved
			 *		x1xx.01xx  0x44 : reserved
			 *		x1xx.10xx  0x48 : reserved
			 *		x1xx.11xx  0x4C : reserved
			 *
			 * clear the unused bits, leaving only 6, 3, and 2
			 */
			switch (extid & 0x4C) {
			case 0x00:	mtd->oobsize = 128;	break;
			case 0x04:	mtd->oobsize = 224;	break;
			case 0x08:	mtd->oobsize = 448;	break;
			case 0x0C:	mtd->oobsize = 64;	break;
			case 0x40:	mtd->oobsize = 32;	break;
			case 0x44:	mtd->oobsize = 16;	break;
			case 0x48:	mtd->oobsize = 640;	break;
			case 0x4C:	/* reserved */
			default:	mtd->oobsize = 0;	break;
			}
				/* Calc blocksize */
			/* Mask out all bits except 7, 5, and 4 */
			switch (extid & 0xB0) {
			case 0x00: mtd->erasesize =  128 * 1024; break;
			case 0x10: mtd->erasesize =  256 * 1024; break;
			case 0x20: mtd->erasesize =  512 * 1024; break;
			case 0x30: mtd->erasesize =  768 * 1024; break;
			case 0x80: mtd->erasesize = 1024 * 1024; break;
			case 0x90: mtd->erasesize = 2048 * 1024; break;
			case 0xA0: /* reserved */
			case 0xB0: /* reserved */
				   mtd->erasesize = 0;	break;
			}
				/* Get the required ecc strength */
			switch (ecc_level) {
			case 0:		// 1 bit / 512 bytes
				chip->ecc.mode = NAND_ECC_SOFT;
				printk(KERN_INFO "NAND ecc: Software \n");
				break;
#if defined (CONFIG_MTD_NAND_ECC_HW)
			case 1:		// 2 bits / 512 bytes
				nand_hw_ecc_init_device(mtd, 4);
				break;
			case 2:		// 4 bits / 512 bytes
				nand_hw_ecc_init_device(mtd, 4);
				break;
			case 3:		// 8 bits / 512 bytes
				nand_hw_ecc_init_device(mtd, 8);
				break;
			case 4:		// 16 bits / 512 bytes
				nand_hw_ecc_init_device(mtd, 16);
				break;
			case 5:		// 24 bits / 2048 bytes
			case 6:		// 24 bits / 1024 bytes
			default:	/* reserved; default to 24 bits / 1KB */
				nand_hw_ecc_init_device(mtd, 24);
				printk(KERN_INFO "NAND ecc: 24-bit HW\n");
				break;
#else
			default:
				chip->ecc.mode = NAND_ECC_SOFT;
				printk(KERN_INFO "NAND ecc: Software \n");
				break;
#endif
			}
		}
		busw = 0;
	}
	// NOTE: If we need to deal with other types of non-traditional NANDs,
	// we can insert code here to check for them and to deal with them.

	else {	/* Sometimes we read invalid ID bytes; usually the first one
		 * is not a recognized manufacturer code.  Do the processing
		 * that's in nand_get_flash_type() only if the first ID byte
		 * is a recognzied mfr code.  This might be wrong sometimes,
		 * but is will catch many of the errors.
		 */
		switch (id_data[0]) {
		case NAND_MFR_TOSHIBA:
		case NAND_MFR_SAMSUNG:
		case NAND_MFR_FUJITSU:
		case NAND_MFR_NATIONAL:
		case NAND_MFR_RENESAS:
		case NAND_MFR_STMICRO:
		case NAND_MFR_HYNIX:
		case NAND_MFR_MICRON:
		case NAND_MFR_AMD:
			/* This processing is identical to code in
			 * nand_get_flash_type().
			 * First calc pagesize */
			mtd->writesize = 1024 << (extid & 0x03);
			extid >>= 2;
				/* Calc oobsize */
			mtd->oobsize = (8 << (extid & 0x01)) * (mtd->writesize >> 9);
			extid >>= 2;
				/* Calc blocksize (multiples of 64KiB) */
			mtd->erasesize = (64 * 1024) << (extid & 0x03);
			extid >>= 2;
				/* Get buswidth information */
			busw = (extid & 0x01) ? NAND_BUSWIDTH_16 : 0;
			break;
		default:	/* Force an error for unexpected type of NAND */
			printk(KERN_INFO "Non-Samsung, non-Hynix, non-ONFI unit\n");
			busw = -1; /* indicate an error */
			break;
		}
	}
	return busw;
}

static const char *part_probes[] = { "cmdlinepart", NULL };
static struct mtd_partition partition_info_cart[] = {
	{ .name		= "Cartridge",
	  .offset	= 0,
 	  .size		= MTDPART_SIZ_FULL },
};

static ssize_t get_cart_hotswap_state(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev__ = to_platform_device(dev);
	struct nxp_nand *nxp = platform_get_drvdata(pdev__);

	return sprintf (buf, "%d\t%d\n", nxp->cart_ready, nxp->cart_ubi);
}

static ssize_t set_cart_hotswap_state(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct resource *res;
	int cart_parts_nb = 0;
	int hotswap_state;
	struct platform_device *pdev__ ;
	struct mtd_partition *cart_parts = NULL;
	struct nand_chip * chip;
	struct mtd_info  * mtd;
	size_t nread;
	uint32_t magic;
	ssize_t ret = count;
	int scan;
	int i = 0;
	struct nxp_nand_plat_data *pdata;
	struct nxp_nand  *nxp;

	printk(KERN_INFO "Entered %s\n", __func__);

	if(sscanf(buf, "%d", &hotswap_state) != 1)
		return -EINVAL;
	else printk(KERN_INFO "%s: %d\n", __func__, hotswap_state);

	pdev__= to_platform_device(dev);
	res = platform_get_resource(pdev__, IORESOURCE_MEM, 0);
	if(!res) {
		dev_err(dev, "nand: cart_hotswap() failed to get resource!\n");
		return -ENXIO;
	}

	nxp = platform_get_drvdata(pdev__);
	if (down_interruptible(&nxp->sem_hotswap))
		return -ERESTARTSYS;

	gpio_request(CARTRIDGE_DETECT,"Cartridge Detect");
	gpio_request(NAND_CHIP_SELECT,"Nand Chip Select");

	if ((0 != hotswap_state) && (1 != hotswap_state))
	{
		gpio_set_function(CARTRIDGE_DETECT, 1);
			// configure CARTRIDGE_DETECT as GPIO
		gpio_direction_input(CARTRIDGE_DETECT); // and an input
		if (gpio_get_value(CARTRIDGE_DETECT) != NAND_CART_DETECT_LEVEL)
		{
			hotswap_state = 0;
		}
		else
		{
			hotswap_state = 1;
		}
	}

	if(0 == hotswap_state) {	// cart is removed
		nxp->cart_ready = 0;
		nxp->eccmode = 0;
		nand_release(&nxp->mtd);
		dev_info(dev, "cartridge removed !\n");
	}
	else
	{  // cart is inserted

		// check if a cartridge is inserted
		gpio_set_function(NAND_CHIP_SELECT, 1);
			// configure NAND_CHIP_SELECT as AltFn1
		gpio_set_function(CARTRIDGE_DETECT, 1);
			// configure CARTRIDGE_DETECT as GPIO
		gpio_direction_input(CARTRIDGE_DETECT); // and an input
		if (gpio_get_value(CARTRIDGE_DETECT) != NAND_CART_DETECT_LEVEL)
		{
			dev_err(dev, "cartridge insertion can't be confirmed by driver\n");
			ret = -EAGAIN;
			goto out;
		}
		dev_info(dev, "cartridge inserted\n");

		if(nxp->cart_ready == 1){
			dev_notice(dev, "cartridge driver was ready\n");
			goto out;
		}
		mtd = &nxp->mtd;
		chip = &nxp->chip;
		memset(mtd, 0, sizeof(struct mtd_info));
		memset(chip, 0, sizeof(struct nand_chip));
		mtd->priv = chip;
		mtd->name = DEV_NAME_NAND;
		mtd->owner = THIS_MODULE;
		chip->IO_ADDR_R 	= (void __iomem *)__PB_IO_MAP_NAND_VIRT;
		chip->IO_ADDR_W 	= (void __iomem *)__PB_IO_MAP_NAND_VIRT;
		chip->cmd_ctrl 		= nand_cmd_ctrl;
		chip->dev_ready 	= nand_dev_ready;
		chip->select_chip 	= nand_select_chip;
		chip->init_size		= lf2000_nand_init_size;
		chip->chip_delay 	= 15;
		chip->ecc.mode  = NAND_ECC_SOFT;

		// After unsuccessful scan, delay for an increasing period
		// and then try again
		do {
			scan = nand_scan(mtd, 1);
			if (0 == scan)
				break;
			if (-EPERM == scan) {
				break;	//permanent error, don't retry
			}

			udelay(10000 << i);	// 10 msec, 20 msec, 40 msec
			printk(KERN_INFO "Delayed %d microsec before again"
					 " calling nand_scan\n",
				10000 << i);
		} while (++i < 4);

		if (i > 1)
			dev_info(dev, "tried to scan cartridge %d times\n",i);

		if (scan) {
			nxp->cart_ready = -1;
			dev_err(dev, "cartridge inserted, but NAND not detected !\n");
			nand_release(mtd);
			ret = -EPERM;
			goto out;
		}
		printk(KERN_INFO "cart chip options: 0x%08x\n", chip->options);

		if (nand_ecc_layout_check(mtd)){
			nxp->cart_ready = -1;
			dev_err(dev, "nand_ecc_layout_check() found error!\n");
			nand_release(mtd);
			ret = -ENXIO;
			goto out;
		}

		pdata = dev_get_platdata(&pdev__->dev);
		pdata->nr_parts  = parse_mtd_partitions(mtd, part_probes, &pdata->parts, NULL);
		if (pdata->nr_parts == 0) {
			pdata->parts    = partition_info_cart;
			pdata->nr_parts = ARRAY_SIZE(partition_info_cart);
		}
			/* Register the cartridge partitions, if it exists */
		ret = mtd_device_parse_register(mtd, NULL, 0, pdata->parts, pdata->nr_parts);
		if (ret) {
			nand_release(mtd);
			goto out;
		}

		mtd->_read(mtd, 0, sizeof(uint32_t), &nread, (void *)&magic);

		magic = be32_to_cpu(magic);
		if (magic == UBI_EC_HDR_MAGIC) {
			nxp->cart_ubi=1;
			dev_info(dev, "cartridge has UBI layer, nread=%d\n", nread);
		}
		else {
			nxp->cart_ubi=0;
			dev_info(dev,"cartridge has no UBI, nread=%d\n", nread);
		}
		nxp->cart_ready = 1;
		dev_info(dev, "cart driver ready !\n");
	}
out:
	up(&nxp->sem_hotswap);
	return ret;
}
static DEVICE_ATTR(cart_hotswap,
		   S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
		   get_cart_hotswap_state, set_cart_hotswap_state);
static struct attribute *nand_attributes[] = {
	&dev_attr_cart_hotswap.attr,
	NULL
};

static struct attribute_group nand_attr_group = {
	.attrs = nand_attributes
};

static int nand_probe(struct platform_device *pdev)
{
	struct nxp_nand_plat_data *pdata = dev_get_platdata(&pdev->dev);
	struct nxp_nand  *nxp;
	struct mtd_info  *mtd;
	struct nand_chip *chip;
	int maxchips = CONFIG_SYS_NAND_MAX_CHIPS;
	int chip_delay = !pdata ? 15 : (pdata->chip_delay ? pdata->chip_delay : 15);
#ifdef CFG_NAND_ECCIRQ_MODE
	int irq = 0; /* platform_get_irq(pdev, 0); */
#endif
	int ret = 0;

	if (pdata == NULL)
		dev_warn(&pdev->dev, "NULL platform data!\n");

	nxp = kzalloc(sizeof (*nxp), GFP_KERNEL);
	if (!nxp) {
		printk(KERN_ERR "NAND: failed to allocate device structure.\n");
		ret = -ENOMEM;
		goto err_kzalloc;
	}

	nxp->pdev = pdev;

	platform_set_drvdata(pdev, nxp);
	sema_init(&nxp->sem_hotswap, 1);
	mtd = &nxp->mtd;
	chip = &nxp->chip;
	mtd->priv = chip;
	mtd->name = DEV_NAME_NAND;
	mtd->owner = THIS_MODULE;

	nand_dev_init(mtd);

	/* insert callbacks */
	chip->IO_ADDR_R 	= (void __iomem *)__PB_IO_MAP_NAND_VIRT;
	chip->IO_ADDR_W 	= (void __iomem *)__PB_IO_MAP_NAND_VIRT;
	chip->cmd_ctrl 		= nand_cmd_ctrl;
	chip->dev_ready 	= nand_dev_ready;
	chip->select_chip 	= nand_select_chip;
	chip->chip_delay 	= chip_delay;
	chip->init_size		= lf2000_nand_init_size;
//	chip->read_buf 		= nand_read_buf;
//	chip->write_buf 	= nand_write_buf;
#if defined (CONFIG_MTD_NAND_ECC_BCH)
//	chip->write_page	= nand_bch_write_page;
#endif

#if defined (CONFIG_MTD_NAND_ECC_HW)
	ret = nand_hw_ecc_init_device(mtd, ECC_HW_BITS);
	printk(KERN_INFO "NAND ecc: Hardware (delay %d)\n", chip_delay);
#elif defined (CONFIG_MTD_NAND_ECC_BCH)
	chip->ecc.mode 	 = NAND_ECC_SOFT_BCH;

	/* refer to nand_ecc.c */
	switch (ECC_BCH_BITS) {
	case  4: chip->ecc.bytes =   7; chip->ecc.size  =  512; break;
	case  8: chip->ecc.bytes =  13; chip->ecc.size  =  512; break;
	case 12: chip->ecc.bytes =  20; chip->ecc.size  =  512; break;
	case 16: chip->ecc.bytes =  26; chip->ecc.size  =  512; break;
	case 24: chip->ecc.bytes =  42; chip->ecc.size  = 1024; break;
	case 40: chip->ecc.bytes =  70; chip->ecc.size  = 1024; break;
//	case 60: chip->ecc.bytes = 105; chip->ecc.size  = 1024; break;	/* not test */
	default:
		printk(KERN_ERR "Fail: not supoort bch ecc %d mode !!!\n", ECC_BCH_BITS);
		ret = -1;
		goto err_something;
	}
	printk(KERN_INFO "NAND ecc: Software BCH (delay %d)\n", chip_delay);
#else
	chip->ecc.mode  = NAND_ECC_SOFT;
	printk(KERN_INFO "NAND ecc: Software (delay %d)\n", chip_delay);
#endif

	printk(KERN_NOTICE "Scanning NAND device ...\n");
	if (nand_scan(mtd, maxchips)) {
		ret = -ENXIO;
		goto nocart;
	}

	if (nand_ecc_layout_check(mtd)){
		nxp->cart_ready = -1;
		dev_err(&pdev->dev, "cartridge inserted, but NAND not detected !\n");
		nand_release(mtd);
		ret = -EPERM;
		goto nocart;
	}

#ifdef CFG_NAND_ECCIRQ_MODE
	ret = request_irq(irq, nxp_irq, 0, DEV_NAME_NAND, nxp);
    if (ret < 0) {
        pr_err("%s: failed to request_irq(%d)\n", __func__, 0);
		ret = -ENODEV;
		goto err_something;
    }

	nxp->irq = irq;
#endif

	/* set command partition */
	pdata->nr_parts  = parse_mtd_partitions(mtd, part_probes, &pdata->parts, NULL);
	if (pdata->nr_parts == 0) {
		pdata->parts    = partition_info_cart;
		pdata->nr_parts = ARRAY_SIZE(partition_info_cart);
	}
	ret = mtd_device_parse_register(mtd, NULL, 0, pdata->parts, pdata->nr_parts);
	if (ret) {
		nand_release(mtd);
		goto err_something;
	} else {
//		platform_set_drvdata(pdev, chip);
	}
	nxp->cart_ready = 1;

#ifdef CONFIG_NAND_RANDOMIZER
	nxp->pages_per_block_mask = (mtd->erasesize/mtd->writesize) - 1;
	if (!nxp->randomize_buf)
		nxp->randomize_buf = kzalloc(mtd->writesize, GFP_KERNEL);
	if (!nxp->randomize_buf) {
		ERROUT("randomize buffer alloc failed\n");
		goto err_something;
	}
#endif
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	if (!nxp->verify_page)
		nxp->verify_page = kzalloc(NAND_MAX_PAGESIZE, GFP_KERNEL);
	if (!nxp->verify_page) {
		ERROUT("verify buffer alloc failed\n");
		goto err_something;
	}
#endif

nocart:
	nxp->eccmode = 0;
	sysfs_create_group(&pdev->dev.kobj, &nand_attr_group);
	nexell_nand_timing_set(mtd);

	printk(KERN_NOTICE "%s: Nand partition \n", ret?"FAIL":"DONE");
	return ret;

err_something:
#ifdef CONFIG_NAND_RANDOMIZER
	if (nxp->randomize_buf)
		kfree (nxp->randomize_buf);
#endif
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	if (nxp->verify_page)
		kfree (nxp->verify_page);
#endif
	kfree(nxp);
err_kzalloc:
	return ret;
}

static struct platform_driver nand_driver = {
	.probe		= nand_probe,
	.remove		= nand_remove,
	.resume		= nand_resume,
	.driver		= {
	.name		= DEV_NAME_LF_NAND,
	.owner		= THIS_MODULE,
	},
};
module_platform_driver(nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jhkim <jhkim@nexell.co.kr>");
MODULE_DESCRIPTION("MTD nand driver for the Nexell");
