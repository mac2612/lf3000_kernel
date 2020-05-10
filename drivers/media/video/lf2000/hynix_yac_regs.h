/* 
 * Hynix YAC Camera Driver
 *
 * Copyright (c) 2011 Leapfrog Enterprises Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __HYNIX_YAC_REGS_H__
#define __HYNIX_YAC_REGS_H__

/* Sensor IDs (WINDOW_PAGE:DEVID) */
#define HI253_ID	0x92
#define SR300PC10_ID	0x93
#define HI161_ID	0x94

/* Pages - used as indicies to mapping tables */
enum register_page {
	WINDOW_PAGE = 0,
	FORMAT_PAGE,
	NOISE1_PAGE,
	DEBLUR_PAGE,
	NOISE2_PAGE,
	EDGE_PAGE,
	LENS_PAGE,
	COLOR_PAGE,
	GAMMA_PAGE,
	FLICKER_PAGE,
	SCALING_PAGE,
	AUTOEXP_PAGE,
	AUTOWB_PAGE,
	AUTOFOCUS_PAGE,
	MCU_PAGE,

	/* undocumented */
	UNDOC_0x02_PAGE,	/* Analog control? */

	/* undocumented SR300PC10 */
#if 0
	0x21 - Autoexposure?
	0x50 - ?
	0x51 - ?
	0x70 - Autoexposure?
	0x72 - ARS?
	0x73 - Auto(exposure) Y offset?
	0x80 - CMC?
	0x81 - Color?
	0x82 - Gamma?
#endif

	/* undocumented Hi-253 */
	UNDOC_0x03_PAGE,

	/* delimiter */
	NO_SUCH_PAGE,
};

/* Register addresses (within given page) */
/* Common (All pages) */
#define PAGEMODE	0x03

/* Page 0x0 (WINDOW_PAGE) Common */
#define PWRCTL		0x01
#define DEVID		0x04
#define PLLCTL1		0x0E
#define PLLCTL2		0x0F
#define VDOCTL1		0x10
#define VDOCTL2		0x11
#define SYNCCTL		0x12
#define WINROWH		0x20
#define WINROWL		0x21
#define WINCOLH		0x22
#define WINCOLL		0x23
#define WINHGTH		0x24
#define WINHGTL		0x25
#define WINWIDH		0x26
#define WINWIDL		0x27
#define HBLANKH		0x40
#define HBLANKL		0x41
#define VSYNCH		0x42
#define VSYNCL		0x43
#define VSCLIP		0x44
#define VSCTL1		0x45
#define VSCTL2		0x46
#define VSCTL3		0x47
#define BLCCTL		0x80
#define BLCTIMETHON	0x90
#define BLCTIMETHOFF	0x91
#define BLCAGTHH	0x92
#define BLCAGTHL	0x93
#define BLCDGH		0x94
#define BLCDGL		0x95
#define BLCOFSDB	0xA0
#define BLCOFSDGB	0xA2
#define BLCOFSDR	0xA4
#define BLCOFSDGR	0xA6
#define STRCTL		0xC0
#define STRWID		0xC1
#define STRTIME		0xC2

/* Page 0x0 (WINDOW_PAGE) SR300PC10 */
#define BIN3WIDH	0x36
#define BIN3WIDL	0x37

/* Page 0x0 (WINDOW_PAGE) Hi-253 */
#define HREFCTL		0x13
#define HREF1		0x48
#define HREF2		0x49
#define VSYNCDELAY1	0x4A
#define VSYNCDELAY2	0x4B


/* Page 0x10 (FORMAT_PAGE) Common */
#define ISPCTL1		0x10
#define ISPCTL2		0x11
#define ISPCTL3		0x12
#define ISPCTL4		0x13
#define ISPCTL5		0x14
#define ITUCTL		0x20
#define ITUSOF		0x24
#define ITUSOL		0x25
#define ITUEOF		0x26
#define ITUEOL		0x27
#define YOFS		0x40
#define DYOFS		0x41
#define UOFS		0x42
#define VOFS		0x43
#define UCON		0x44
#define VCON		0x45
#define SOLARI		0x46
#define BINARY		0x47
#define CONTRAST	0x48
#define AGBRT		0x50
#define SATCTL		0x60
#define SATB		0x61
#define SATR		0x62
#define AGSAT		0x63
#define SATTIMETH	0x66
#define SATOUTDEL	0x67
#define UPOSSAT		0x6A
#define UNEGSAT		0x6B
#define VPOSSAT		0x6C
#define VNEGSAT		0x6D

/* Page 0x10 (FORMAT_PAGE) Hi-253 */
#define CBPOSSAT	0x6A
#define CBNEGSAT	0x6B
#define CRPOSSAT	0x6C
#define CRNEGSAT	0x6D
#define LGRATION	0x70
#define LGOFS		0x71


/* Page 0x11 (NOISE1_PAGE) Common */
/* -No common registers- */

/* Page 0x11 (NOISE1_PAGE) SR300PC10 */
#define DLPFCTL		0x10
#define DLPFOPT1	0x11
#define DLPFOPT2	0x12
#define DLPFOPT3	0x13
#define DLPFOPT4	0x14
#define EDGEBP		0x19
#define EDGEMASK1	0x1A
#define EDGEMASK2	0x1B
#define EDGEMASK3	0x1C
#define HVRAT1OUT	0x1D
#define HVRAT1IN	0x1E
#define HVRAT1DAK	0x1F
#define HVRAT2OUT	0x20
#define HVRAT2IN	0x21
#define HVRAT2IN	0x21
#define HVRAT2DAK	0x22
#define DLPFOPT5	0x23
#define DLPFOPT6	0x24
#define DLPFOPT7	0x25
#define DLPFOPT8	0x26
#define DLPFOPT9	0x27
#define DLPFOPT10	0x28
#define DLPFOPT11	0x29
#define OUTRAT		0x30
#define OUTTHH		0x31
#define OUTTHM		0x32
#define OUTTHL		0x33
#define DAKREGOUT	0x34
#define OUTOFSLO	0x35
#define OUTSTDLO	0x36
#define OUTCLIPLO	0x37
#define OUTOFSHR2	0x38
#define OUTOFSMR2	0x39
#define OUTOFSMLR2	0x3A
#define OUTOFSLR2	0x3B
#define OUTSTDHR2	0x3C
#define OUTSTDMR2	0x3D
#define OUTSTDMLR2	0x3E
#define OUTSTDLR2	0x3F
#define OUTCLIPHR2	0x40
#define OUTCLIPMR2	0x41
#define OUTCLIPMLR2	0x42
#define OUTCLIPLR2	0x43
#define OUTOFSHR1	0x44
#define OUTOFSMR1	0x45
#define OUTOFSMLR1	0x46
#define OUTOFSLR1	0x47
#define OUTSTDHR1	0x48
#define OUTSTDMR1	0x49
#define OUTSTDMLR1	0x4A
#define OUTSTDLR1	0x4B
#define OUTCLIPHR1	0x4C
#define OUTCLIPMR1	0x4D
#define OUTCLIPMLR1	0x4E
#define OUTCLIPLR1	0x4F
#define OUTEDGEH	0x50
#define OUTEDGEM	0x51
#define OUTEDGEML	0x52
#define OUTEDGEL	0x53
#define OUTCOLH		0x54
#define OUTCOLM		0x55
#define OUTCOLML	0x56
#define OUTCOLL		0x57
#define OUTOFSLIMH	0x58
#define OUTOFSLIMM	0x59
#define OUTOFSLIMML	0x5A
#define OUTOFSLIML	0x5B
#define OUTDPCTH	0x5C
#define OUTDPCSTD	0x5D
#define INRAT		0x5E
#define INTHH_SR300PC10		0x5F
#define INTHM_SR300PC10		0x60
#define INTHL_SR300PC10		0x61
#define DAKREGIN	0x62
#define INOFSLO		0x63
#define INSTDLO		0x64
#define INCLIPLO	0x65
#define INOFSHR2	0x66
#define INOFSMR2	0x67
#define INOFSMLR2	0x68
#define INOFSLR2	0x69
#define INSTDHR2	0x6A
#define INSTDMR2	0x6B
#define INSTDMLR2	0x6C
#define INSTDLR2	0x6D
#define INCLIPHR2	0x6E
#define INCLIPMR2	0x6F
#define INCLIPMLR2	0x70
#define INCLIPLR2	0x71
#define INOFSHR1	0x72
#define INOFSMR1	0x73
#define INOFSMLR1	0x74
#define INOFSLR1	0x75
#define INSTDHR1	0x76
#define INSTDMR1	0x77
#define INSTDMLR1	0x78
#define INSTDLR1	0x79
#define INCLIPHR1	0x7A
#define INCLIPMR1	0x7B
#define INCLIPMLR1	0x7C
#define INCLIPLR1	0x7D
#define INEDGEH		0x7E
#define INEDGEM		0x7F
#define INEDGEML	0x80
#define INEDGEL		0x81
#define INCOLH		0x82
#define INCOLM		0x83
#define INCOLML		0x84
#define INCOLL		0x85
#define INOFSLIMH	0x86
#define INOFSLIMM	0x87
#define INOFSLIMML	0x88
#define INOFSLIML	0x89
#define INDPCTH		0x8A
#define INDPCSTD	0x8B
#define DAK1RAT		0x8C
#define DAK1THH		0x8D
#define DAK1THM		0x8E
#define DAK1THL		0x8F
#define DAKREGDAK1	0x90
#define DAK1OFSLO	0x91
#define DAK1STDLO	0x92
#define DAK1CLIPLO	0x93
#define DAK1OFSHR2	0x94
#define DAK1OFSMR2	0x95
#define DAK1OFSMLR2	0x96
#define DAK1OFSLR2	0x97
#define DAK1STDHR2	0x98
#define DAK1STDMR2	0x99
#define DAK1STDMLR2	0x9A
#define DAK1STDLR2	0x9B
#define DAK1CLIPHR2	0x9C
#define DAK1CLIPMR2	0x9D
#define DAK1CLIPMLR2	0x9E
#define DAK1CLIPLR2	0x9F
#define DAK1OFSHR1	0xA0
#define DAK1OFSMR1	0xA1
#define DAK1OFSMLR1	0xA2
#define DAK1OFSLR1	0xA3
#define DAK1STDHR1	0xA4
#define DAK1STDMR1	0xA5
#define DAK1STDMLR1	0xA6
#define DAK1STDLR1	0xA7
#define DAK1CLIPHR1	0xA8
#define DAK1CLIPMR1	0xA9
#define DAK1CLIPMLR1	0xAA
#define DAK1CLIPLR1	0xAB
#define DAK1EDGEH	0xAC
#define DAK1EDGEM	0xAD
#define DAK1EDGEML	0xAE
#define DAK1EDGEL	0xAF
#define DAK1COLH	0xB0
#define DAK1COLM	0xB1
#define DAK1COLML	0xB2
#define DAK1COLL	0xB3
#define DAK1OFSLIMH	0xB4
#define DAK1OFSLIMM	0xB5
#define DAK1OFSLIMML	0xB6
#define DAK1OFSLIML	0xB7
#define DAK1DPCTH	0xB8
#define DAK1DPCSTD	0xB9
#define DAK23RAT	0xBA
#define DAK23THH	0xBB
#define DAK23THM	0xBC
#define DAK23THL	0xBD
#define DAKREGDAK23	0xBE
#define DAK23OFSLO	0xBF
#define DAK23STDLO	0xC0
#define DAK23CLIPLO	0xC1
#define DAK23OFSHR2	0xC2
#define DAK23OFSMR2	0xC3
#define DAK23OFSMLR2	0xC4
#define DAK23OFSLR2	0xC5
#define DAK23STDHR2	0xC6
#define DAK23STDMR2	0xC7
#define DAK23STDMLR2	0xC8
#define DAK23STDLR2	0xC9
#define DAK23CLIPHR2	0xCA
#define DAK23CLIPMR2	0xCB
#define DAK23CLIPMLR2	0xCC
#define DAK23CLIPLR2	0xCD
#define DAK23OFSHR1	0xCE
#define DAK23OFSMR1	0xCF
#define DAK23OFSMLR1	0xD0
#define DAK23OFSLR1	0xD1
#define DAK23STDHR1	0xD2
#define DAK23STDMR1	0xD3
#define DAK23STDMLR1	0xD4
#define DAK23STDLR1	0xD5
#define DAK23CLIPHR1	0xD6
#define DAK23CLIPMR1	0xD7
#define DAK23CLIPMLR1	0xD8
#define DAK23CLIPLR1	0xD9
#define DAK23EDGEH	0xDA
#define DAK23EDGEM	0xDB
#define DAK23EDGEML	0xDC
#define DAK23EDGEL	0xDD
#define DAK23COLH	0xDE
#define DAK23COLM	0xDF
#define DAK23COLML	0xE0
#define DAK23COLL	0xE1
#define DAK23OFSLIMH	0xE2
#define DAK23OFSLIMM	0xE3
#define DAK23OFSLIMML	0xE4
#define DAK23OFSLIML	0xE5
#define DAK23DPCTH	0xE6
#define DAK23DPCSTD	0xE7
#define DLPFAUTOCTL1_SR300PC10	0xF0
#define DLPFAUTOCTL2_SR300PC10	0xF1
#define LPFAGTHL_SR300PC10	0xF2
#define LPFAGTHH_SR300PC10	0xF3
#define LPFOUTTHL_SR300PC10	0xF4
#define LPFOUTTHH_SR300PC10	0xF5
#define LPFYMEANTHL_SR300PC10	0xF6
#define LPFYMEANTHH_SR300PC10	0xF7

/* Page 0x11 (NOISE1_PAGE) Hi-253 */
#define DLPFCTL1	0x10
#define DLPFCTL2	0x11
#define DLPFAUTOCTL1_HI253	0x20
#define DLPFAUTOCTL2_HI253	0x21
#define LPFAGTHL_HI253		0x26
#define LPFAGTHH_HI253		0x27
#define LPFOUTTHL_HI253		0x28
#define LPFOUTTHH_HI253		0x29
#define LPFYMEANTHL_HI253	0x2B
#define LPFYMEANTHH_HI253	0x2C
#define OUT2YBOUNDH	0x30
#define OUT2YBOUNDL	0x31
#define OUT2RATIO	0x32
#define OUT2THH		0x33
#define OUT2THM		0x34
#define OUT2THL		0x35
#define OUT1YBOUNDH	0x36
#define OUT1YBOUNDL	0x37
#define OUT1RATIO	0x38
#define OUT1THH		0x39
#define OUT1THM		0x3A
#define OUT1THL		0x3B
#define INYBOUNDH	0x3C
#define INYBOUNDL	0x3D
#define INRATIO		0x3E
#define INTHH_HI253		0x3F
#define INTHM_HI253		0x40
#define INTHL_HI253		0x41
#define DARK1YBNDH	0x42
#define DARK1YBNDL	0x43
#define DARK1RATIO	0x44
#define DARK1THH	0x45
#define DARK1THM	0x46
#define DARK1THL	0x47
#define DARK2YBNDH	0x48
#define DARK2YBNDL	0x49
#define DARK2RATIO	0x4A
#define DARK2THH	0x4B
#define DARK2THM	0x4C
#define DARK2THL	0x4D
#define DARK3YBNDH	0x4E
#define DARK3YBNDL	0x4F
#define DARK3RATIO	0x50
#define DARK3THH	0x51
#define DARK3THM	0x52
#define DARK3THL	0x53


/* Page 0x12 (DEBLUR_PAGE) SR300PC10 */
#define GLPFCTL1	0x50
#define GLPFTHOUT	0x51
#define GLPFTHIN	0x52
#define GLPFTHDAK	0x53
#define GLPFCTL2	0x55
#define GLPFRATOUT	0x56
#define GLPFRATIN	0x57
#define GLPFRATDAK	0x58
#define GLPFBP		0x59
#define LDCTL		0x70
#define LDBCENOUT	0x71
#define LDBLOUT		0x72
#define LDBROUT		0x73
#define LDBOFSOUT	0x74
#define LDCCENOUT	0x75
#define LDCLOUT		0x76
#define LDCROUT		0x77
#define LDCOFSOUT	0x78
#define LDRATOUT	0x79
#define LDHLMTOUT	0x7A
#define LDLLMTOUT	0x7B
#define LDBCENIND	0x7F
#define LDBLIND		0x80
#define LDBRIND		0x81
#define LDBOFSIND	0x82
#define LDCCENIND	0x83
#define LDCLIND		0x84
#define LDCRIND		0x85
#define LDCOFSIND	0x86
#define LDRATIN		0x87
#define LDHLMTIN	0x88
#define LDLLMTIN	0x89
#define LDBCENDAK23	0x8A
#define LDBLDAK23	0x8B
#define LDBRDAK23	0x8C
#define LDBOFSDAK23	0x8D
#define LDCCENDAK23	0x8E
#define LDCLDAK23	0x8F
#define LDCRDAK23	0x90
#define LDCOFSDAK23	0x91
#define LDRATDAK1	0x92
#define LDHLMTDAK1	0x93
#define LDLLMTDAK1	0x94
#define LDRATDAK23	0x95
#define LDHLMTDAK23	0x96
#define LDLLMTDAK23	0x97
#define LDTERMOPT	0x9B
#define LDTERMNEG	0x9C
#define LDTERMPOS	0x9D
#define LDTERMRAT	0x9E
#define LDEDCENOUT	0xB0
#define LDEDLOUT	0xB1
#define LDEDROUT	0xB2
#define LDEDOFSOUT	0xB3
#define LDEDRATOUT	0xB4
#define LDLOWROUT	0xB5
#define LDEDCENIN	0xB6
#define LDEDLIN		0xB7
#define LDEDRIN		0xB8
#define LDEDOFSIN	0xB9
#define LDEDRATIN	0xBA
#define LDLOWRIN	0xBB
#define LDEDCENDAK	0xBC
#define LDEDLDAK	0xBD
#define LDEDRDAK	0xBE
#define LDEDOFSDAK	0xBF
#define LDEDRATDAK	0xC0
#define LDLOWRDAK	0xC1
#define LDSTDHOUT	0xD0
#define LDSTDMOUT	0xD1
#define LDSTDMLOUT	0xD2
#define LDSTDLOUT	0xD3
#define LDSTDDOUT	0xD4
#define LDSTDHIN	0xD5
#define LDSTDMIN	0xD6
#define LDSTDMLIN	0xD7
#define LDSTDLIN	0xD8
#define LDSTDDIN	0xD9
#define LDSTDHDAK	0xDA
#define LDSTDMDAK	0xDB
#define LDSTDMLDAK	0xDC
#define LDSTDLDAK	0xDD
#define LDSTDDDAK	0xDE


/* Page 0x13/0x12 (NOSIE2_PAGE) Common */
#define YCLPFCTL1	0x20
#define YCLPFCTL2	0x21
#define YCLPFCTL4	0x23
#define YCPRWTH		0x30

/* Page 0x13 (NOSIE2_PAGE) SR300PC10 */
#define YCLPFCTL3	0x22
#define YCUNI1THOUT	0x31
#define YCUNI1THIN	0x32
#define YCUNI1THDAK	0x33
#define YCUNI2TH_SR300PC10	0x34
#define YCUNI3TH_SR300PC10	0x35
#define YCNOR1TH_SR300PC10	0x36
#define YCNOR2TH_SR300PC10	0x37
#define YCNOR3TH_SR300PC10	0x38
#define YCBPOUT		0x3B
#define YCBPDAK		0x3D
#define YCDAK1SK	0x3E
#define YCDAK23SK	0x3F
#define YCAVG		0x40
#define YCOUTTHH	0x50
#define YCOUTTHL	0x51
#define YCOUTSTDH	0x52
#define YCOUTSTDM	0x53
#define YCOUTSTDL	0x54
#define YCOUTRAT	0x55
#define YCINTHH_SR300PC10	0x5C
#define YCINTHL_SR300PC10	0x5D
#define YCINSTDH_SR300PC10	0x5E
#define YCINSTDM_SR300PC10	0x5F
#define YCINSTDL_SR300PC10	0x60
#define YCINRAT_SR300PC10	0x61
#define YCDARK1THH_SR300PC10	0x62
#define YCDARK1THL_SR300PC10	0x63
#define YCDARK1STDH_SR300PC10	0x64
#define YCDARK1STDM_SR300PC10	0x65
#define YCDARK1STDL_SR300PC10	0x66
#define YCDARK1RAT_SR300PC10	0x67
#define YCDARK23THH	0x68
#define YCDARK23THL	0x69
#define YCDARK23STDH	0x6A
#define YCDARK23STDM	0x6B
#define YCDARK23STDL	0x6C
#define YCDARK23RAT	0x6D

/* Page 0x12 (NOSIE2_PAGE) Hi-253 */
#define YCUNI1TH	0x31
#define YCUNI2TH_HI253	0x32
#define YCUNI3TH_HI253	0x33
#define YCNOR1TH_HI253	0x34
#define YCNOR2TH_HI253	0x35
#define YCNOR3TH_HI253	0x36
#define YCOUT2THH	0x40
#define YCOUT2THL	0x41
#define YCOUT2STDH	0x42
#define YCOUT2STDM	0x43
#define YCOUT2STDL	0x44
#define YCOUT2RAT	0x45
#define YCOUT1THH	0x46
#define YCOUT1THL	0x47
#define YCOUT1STDH	0x48
#define YCOUT1STDM	0x49
#define YCOUT1STDL	0x4A
#define YCOUT1RAT	0x4B
#define YCINTHH_HI253		0x4C
#define YCINTHL_HI253		0x4D
#define YCINSTDH_HI253		0x4E
#define YCINSTDM_HI253		0x4F
#define YCINSTDL_HI253		0x50
#define YCINRAT_HI253		0x51
#define YCDARK1THH_HI253	0x52
#define YCDARK1THL_HI253	0x53
#define YCDARK1STDH_HI253	0x54
#define YCDARK1STDM_HI253	0x55
#define YCDARK1STDL_HI253	0x56
#define YCDARK1RAT_HI253	0x57
#define YCDARK2THH	0x58
#define YCDARK2THL	0x59
#define YCDARK2STDH	0x5A
#define YCDARK2STDM	0x5B
#define YCDARK2STDL	0x5C
#define YCDARK2RAT	0x5D
#define YCDARK3THH	0x5E
#define YCDARK3THL	0x5F
#define YCDARK3STDH	0x60
#define YCDARK3STDM	0x61
#define YCDARK3STDL	0x62
#define YCDARK3RAT	0x63
#define DPCCTL		0x90


/* Page 0x14/0x13 (EDGE_PAGE) Common */
#define EDGECTL1	0x10
#define EDGECTL2	0x11
#define EDGECTL3	0x12
#define EDGENGAIN	0x20
#define EDGEPGAIN	0x21
#define EDGEHCLIP1TH	0x23
#define EDGEHCLIP2TH	0x24
#define EDGELCLIPTH	0x25
#define EDGELCLIPLMT	0x26
#define EDGE2DCTL1	0x80
#define EDGE2DCTL2	0x81
#define EDGE2DCTL3	0x82
#define EDGE2DCTL4	0x83
#define EDGE2DNGAIN	0x90
#define EDGE2DPGAIN	0x91
#define EDGE2DLCLIPLMT	0x93
#define EDGE2DHCLIP1TH	0x94

/* Page 0x14 (EDGE_PAGE) SR300PC10 */
#define EDGECTL4	0x13
#define EDGECTL5_SR300PC10	0x15
#define EDGECTL6	0x16
#define EDGECTL7	0x17
#define EDGEPLCLP	0x29
#define EDGETIMETH_SR300PC10	0x2A
#define EDGEAGTH_SR300PC10	0x2B
#define EDGE2DCTL5	0x84
#define EDGE2DEFT	0x87
#define EDGE2DSKT	0x88
#define EDGE2DEMB	0x89
#define EDGE2DUCON	0x8A
#define EDGE2DVCON	0x8B
#define EDGE2DKNEETH	0x99
#define EDGE2DLCOUTH	0xA0
#define EDGE2DLCOUTM	0xA1
#define EDGE2DLCOUTL	0xA2
#define EDGE2DLCINH	0xA3
#define EDGE2DLCINM	0xA4
#define EDGE2DLCINL	0xA5
#define EDGE2DLCDAK1H	0xA6
#define EDGE2DLCDAK1M	0xA7
#define EDGE2DLCDAK1L	0xA8
#define EDGE2DLCDAK23H	0xA9
#define EDGE2DLCDAK23M	0xAA
#define EDGE2DLCDAK23L	0xAB

/* Page 0x13 (EDGE_PAGE) Hi-253 */
#define EDGECTL5_HI253		0x14
#define EDGETIMETH_HI253	0x29
#define EDGEAGTH_HI253		0x2A
#define EDGE2DCTL6		0x85
#define EDGE2DHCLIP2TH		0x95
#define EDGE2DLCOUT2N		0xA0
#define EDGE2DLCOUT2P		0xA1
#define EDGE2DLCOUT1N		0xA2
#define EDGE2DLCOUT1P		0xA3
#define EDGE2DLCINN		0xA4
#define EDGE2DLCINP		0xA5
#define EDGE2DLCD1N		0xA6
#define EDGE2DLCD1P		0xA7
#define EDGE2DLCD2N		0xA8
#define EDGE2DLCD2P		0xA9
#define EDGE2DLCD3N		0xAA
#define EDGE2DLCD3P		0xAB


/* Page 0x15/0x14 (LENS_PAGE) Common */
#define LENSCTL1	0x10
#define LENSCTL2	0x11
#define XCEN		0x20
#define YCEN		0x21
#define LENSRP0		0x40
#define LENSGrP0	0x50
#define LENSBP0		0x60
#define LENSGbP0	0x70

/* Page 0x15 (LENS_PAGE) SR300PC10 */
#define LENSRGAIN_SR300PC10	0x18
#define LENSGGAIN_SR300PC10	0x19
#define LENSBGAIN_SR300PC10	0x1A
#define LENSSTEP_SR300PC10	0x1B
#define LAGOFF_SR300PC10	0x2C
#define LAGON_SR300PC10		0x2D
#define LENSRP1		0x41
#define LENSRP2		0x42
#define LENSRP3		0x43
#define LENSRP4		0x44
#define LENSRP5		0x45
#define LENSRP6		0x46
#define LENSRP7		0x47
#define LENSRP8		0x48
#define LENSGrP1	0x51
#define LENSGrP2	0x52
#define LENSGrP3	0x53
#define LENSGrP4	0x54
#define LENSGrP5	0x55
#define LENSGrP6	0x56
#define LENSGrP7	0x57
#define LENSGrP8	0x58
#define LENSBP1		0x61
#define LENSBP2		0x62
#define LENSBP3		0x63
#define LENSBP4		0x64
#define LENSBP5		0x65
#define LENSBP6		0x66
#define LENSBP7		0x67
#define LENSBP8		0x68
#define LENSGbP1	0x71
#define LENSGbP2	0x72
#define LENSGbP3	0x73
#define LENSGbP4	0x74
#define LENSGbP5	0x75
#define LENSGbP6	0x76
#define LENSGbP7	0x77
#define LENSGbP8	0x78

/* Page 0x14 (LENS_PAGE) Hi-253 */
#define Gb_XCEN		0x14
#define Gb_YCEN		0x15
#define R_XCEN		0x16
#define R_YCEN		0x17
#define B_XCEN		0x18
#define B_YCEN		0x19
#define LENSRGAIN_HI253		0x22
#define LENSGGAIN_HI253		0x23
#define LENSBGAIN_HI253		0x24
#define LAGOFF_HI253		0x25
#define LAGON_HI253		0x26
#define LENSSTEP_HI253		0x28


/* Page 0x16/0x15 (COLOR_PAGE) Common */
#define CMCCTL		0x10
#define CMCOFSGH	0x14
#define CMCOFSGM	0x15
#define CMCOFSGL	0x16
#define CMCSIGN		0x17
#define CMC11		0x30
#define CMC12		0x31
#define CMC13		0x32
#define CMC21		0x33
#define CMC22		0x34
#define CMC23		0x35
#define CMC31		0x36
#define CMC32		0x37
#define CMC33		0x38
#define CMCOFSL11	0x40
#define CMCOFSL12	0x41
#define CMCOFSL13	0x42
#define CMCOFSL21	0x43
#define CMCOFSL22	0x44
#define CMCOFSL23	0x45
#define CMCOFSL31	0x46
#define CMCOFSL32	0x47
#define CMCOFSL33	0x48
#define CMCOFSH11	0x50
#define CMCOFSH12	0x51
#define CMCOFSH13	0x52
#define CMCOFSH21	0x53
#define CMCOFSH22	0x54
#define CMCOFSH23	0x55
#define CMCOFSH31	0x56
#define CMCOFSH32	0x57
#define CMCOFSH33	0x58


/* Page 0x17/0x16 (GAMMA_PAGE) Common */
#define GMACTL		0x10
#define GGMA0		0x30
#define GGMA1		0x31
#define GGMA2		0x32
#define GGMA3		0x33
#define GGMA4		0x34
#define GGMA5		0x35
#define GGMA6		0x36
#define GGMA7		0x37
#define GGMA8		0x38
#define GGMA9		0x39
#define GGMA10		0x3A
#define GGMA11		0x3B
#define GGMA12		0x3C
#define GGMA13		0x3D
#define GGMA14		0x3E
#define GGMA15		0x3F
#define GGMA16		0x40
#define GGMA17		0x41
#define GGMA18		0x42
#define RGMA0		0x50
#define RGMA1		0x51
#define RGMA2		0x52
#define RGMA3		0x53
#define RGMA4		0x54
#define RGMA5		0x55
#define RGMA6		0x56
#define RGMA7		0x57
#define RGMA8		0x58
#define RGMA9		0x59
#define RGMA10		0x5A
#define RGMA11		0x5B
#define RGMA12		0x5C
#define RGMA13		0x5D
#define RGMA14		0x5E
#define RGMA15		0x5F
#define RGMA16		0x60
#define RGMA17		0x61
#define RGMA18		0x62
#define BGMA0		0x70
#define BGMA1		0x71
#define BGMA2		0x72
#define BGMA3		0x73
#define BGMA4		0x74
#define BGMA5		0x75
#define BGMA6		0x76
#define BGMA7		0x77
#define BGMA8		0x78
#define BGMA9		0x79
#define BGMA10		0x7A
#define BGMA11		0x7B
#define BGMA12		0x7C
#define BGMA13		0x7D
#define BGMA14		0x7E
#define BGMA15		0x7F
#define BGMA16		0x80
#define BGMA17		0x81
#define BGMA18		0x82


/* Page 0x18/0x17 (FLICKER_PAGE) Common */
#define FLKMODE		0xC0
#define FLK200		0xC4
#define FLK240		0xC5
#define FLKTH1		0xC6
#define FLKTH2		0xC7

/* Page 0x18 (FLICKER_PAGE) SR300PC10 */
#define ISCTL		0x10
#define FLKDET		0xE0


/* Page 0x19/0x18 (SCALING_PAGE) Common */
#define ZOOMCTL1	0x10
#define ZOOMCTL2	0x11
#define ZOUTWIDH	0x20
#define ZOUTWIDL	0x21
#define ZOUTHGTH	0x22
#define ZOUTHGTL	0x23
#define ZWINSTXH	0x24
#define ZWINSTXL	0x25
#define ZWINSTYH	0x26
#define ZWINSTYL	0x27
#define ZWINENXH	0x28
#define ZWINENXL	0x29
#define ZWINENYH	0x2A
#define ZWINENYL	0x2B
#define ZVERSTEPH	0x2C
#define ZVERSTEPL	0x2D
#define ZHORSTEPH	0x2E
#define ZHORSTEPL	0x2F
#define ZFIFODLY	0x30


/* Page 0x20 (AUTOEXP_PAGE) Common */
#define AECTL1		0x10
#define AECTL2		0x11
#define AEFRAMECTL1	0x20
#define AEFINECTL1	0x28
#define AEFINECTL2	0x29
#define AEFINECTL3	0x2A
#define AEFINECTL4	0x2B
#define AEWGT1		0x60
#define AEWGT2		0x61
#define AEWGT3		0x62
#define AEWGT4		0x63
#define AEWGT5		0x64
#define AEWGT6		0x65
#define AEWGT7		0x66
#define AEWGT8		0x67
#define AEWGT9		0x68
#define AEWGT10		0x69
#define AEWGT11		0x6A
#define AEWGT12		0x6B
#define AEWGT13		0x6C
#define AEWGT14		0x6D
#define AEWGT15		0x6E
#define AEWGT16		0x6F
#define YLVL		0x70
#define YTH1		0x78
#define YTH2HI		0x79
#define YTH2LOW		0x7C
#define EXPINTH		0x80
#define EXPINTM		0x81
#define EXPINTL		0x82
#define EXPTIMEH	0x83
#define EXPTIMEM	0x84
#define EXPTIMEL	0x85
#define EXPMINH		0x86
#define EXPMINL		0x87
#define EXPMAXH		0x88
#define EXPMAXM		0x89
#define EXPMAXL		0x8A
#define EXP100H		0x8B
#define EXP100L		0x8C
#define EXP120H		0x8D
#define EXP120L		0x8E
#define EXPFIXH		0x91
#define EXPFIXM		0x92
#define EXPFIXL		0x93
#define EXPOUT1		0x98
#define EXPOUT2		0x99
#define EXPLMTH		0x9C
#define EXPLMTL		0x9D
#define EXPUNITH	0x9E
#define EXPUNITL	0x9F
#define AG		0xB0
#define AGMIN		0xB1
#define AGMAX		0xB2
#define AGLVLH		0xB3
#define AGTH1		0xB4
#define AGTH2		0xB5
#define AGBTH1		0xB6
#define AGBTH2		0xB7
#define AGBTH3		0xB8
#define AGBTH4		0xB9
#define AGBTH5		0xBA
#define AGBTH6		0xBB
#define AGBTH7		0xBC
#define AGBTH8		0xBD
#define DGMAX		0xC8
#define DGMIN		0xC9
#define YAVG		0xD3

/* Page 0x20 (AUTOEXP_PAGE) SR300PC10 */
#define AGBTH9		0xBE
#define AGBTH10		0xBF
#define AGBTH11		0xC0
#define AGBTH12		0xC1
#define AGLVLL_SR300PC10	0xC2
#define AGSKY_SR300PC10		0xC7

/* Page 0x20 (AUTOEXP_PAGE) Hi-253 */
#define AEFINECTL5	0x2C
#define AEFINECTL6	0x2D
#define AEFINECTL7	0x2E
#define AEFINECTL8	0x2F
#define AGSKY_HI253	0xC0
#define AGLVLL_HI253	0xC3
#define AGTIMETH	0xC4


/* Page 0x22 (AUTOWB_PAGE) Common */
#define AWBCTL1		0x10
#define AWBCTL2		0x11
#define ULVL		0x30
#define VLVL		0x31
#define UVTH1		0x38
#define UVTH2		0x39
#define YRANGE		0x40
#define CDIFF		0x41
#define CSUM2		0x42
#define WHTPXLTH	0x46
#define RGAIN		0x80
#define GGAIN		0x81
#define BGAIN		0x82
#define RMAX		0x83
#define RMIN		0x84
#define BMAX		0x85
#define BMIN		0x86
#define RMAXM		0x87
#define RMINM		0x88
#define BMAXM		0x89
#define BMINM		0x8A
#define RMAXB		0x8B
#define RMINB		0x8C
#define BMAXB		0x8D
#define BMINB		0x8E
#define BGAINPARA1	0x8F
#define BGAINPARA2	0x90
#define BGAINPARA3	0x91
#define BGAINPARA4	0x92
#define BGAINPARA5	0x93
#define BGAINPARA6	0x94
#define BGAINPARA7	0x95
#define BGAINPARA8	0x96
#define BGAINPARA9	0x97
#define BGAINPARA10	0x98
#define BGAINPARA11	0x99
#define BGAINPARA12	0x9A
#define BGAINBND1	0x9B
#define BGAINBND2	0x9C
#define RAINTH1		0x9D
#define RAINTH2		0x9E
#define RAINTH3		0x9F
#define RDELTA1		0xA0
#define BDELTA1		0xA1
#define RDELTA2		0xA2
#define BDELTA2		0xA3
#define AWBEXPLMT1	0xA4
#define AWBEXPLMT2	0xA5
#define AWBEXPLMT3	0xA6
#define MRGAIN		0xB2
#define MBGAIN		0xB3
#define UAVG		0xD2
#define VAVG		0xD3

/* Page 0x22 (AUTOWB_PAGE) SR300PC10 */
#define AWBMAP		0x19
#define AWBCTL3		0x25
#define AWBCTL4		0x2E
#define AWBCTL5		0x2F
#define WHTSLOPE1	0x34
#define WHTSLOPE2	0x35
#define CDIFFBND1	0x36
#define CDIFFBND2	0x37
#define YRANGETOT	0x43
#define CDIFFTOT	0x44
#define CSUM2TOT	0x45
#define YRANGEDAK	0x47
#define CDIFFDAK	0x48
#define CSUM2DAK	0x49
#define YRANGEOUT	0x4A
#define CDIFFOUT	0x4B
#define CSUM2OUT	0x4C
#define STBYTH		0x4D
#define CSUM4		0x50
#define CSUM4TOT	0x51
#define CSUM4DAK	0x52
#define CSUM4OUT	0x53
#define AWBOUTTHON	0x5A
#define AWBOUTTHOFF	0x5B
#define HTMPTHOFF	0xAD
#define HTMPTHON	0xAE
#define HTMPOFS		0xB6
#define LTMPOFS		0xB7
#define LTMPTHMIN	0xBC
#define LTMPTHMAX	0xBD


/* Page 0x24 (AUTOFOCUS_PAGE) Common */
#define AFCTL1		0x10
#define AFCTL2		0x12
#define AFCTL3		0x13
#define AFCTL4		0x19
#define AFROWSTR1	0x40
#define AFROWENR1	0x41
#define AFROWSTR2	0x42
#define AFROWENR2	0x43
#define AFROWSTR3	0x44
#define AFROWENR3	0x45
#define AFCOLSTR1	0x48
#define AFCOLENR1	0x49
#define AFCOLSTR2	0x4A
#define AFCOLENR2	0x4B
#define AFCOLSTR3	0x4C
#define AFCOLENR3	0x4D
#define AFCOLSTR4	0x4E
#define AFCOLENR4	0x4F
#define AFROWLSB	0x50
#define AFCOLLSB	0x51
#define AFDATAR1H	0xA0
#define AFDATAR1L	0xA1
#define AFDATAR2H	0xA2
#define AFDATAR2L	0xA3
#define AFDATAR3H	0xA4
#define AFDATAR3L	0xA5
#define AFDATAR4H	0xA6
#define AFDATAR4L	0xA7
#define AFDATAR5H	0xA8
#define AFDATAR5L	0xA9
#define AFDATAR6H	0xAA
#define AFDATAR6L	0xAB
#define AFDATAR7H	0xAC
#define AFDATAR7L	0xAD
#define AFDATAR8H	0xAE
#define AFDATAR8L	0xAF
#define AFDATAR9H	0xB0
#define AFDATAR9L	0xB1
#define AFDATAR10H	0xB2
#define AFDATAR10L	0xB3
#define AFDATAR11H	0xB4
#define AFDATAR11L	0xB5
#define AFDATAR12H	0xB6
#define AFDATAR12L	0xB7
#define ASDATAR1H	0xB8
#define ASDATAR1L	0xB9
#define ASDATAR2H	0xBA
#define ASDATAR2L	0xBB
#define ASDATAR3H	0xBC
#define ASDATAR3L	0xBD
#define ASDATAR4H	0xBE
#define ASDATAR4L	0xBF
#define ASDATAR5H	0xC0
#define ASDATAR5L	0xC1
#define ASDATAR6H	0xC2
#define ASDATAR6L	0xC3
#define ASDATAR7H	0xC4
#define ASDATAR7L	0xC5
#define ASDATAR8H	0xC6
#define ASDATAR8L	0xC7
#define ASDATAR9H	0xC8
#define ASDATAR9L	0xC9
#define ASDATAR10H	0xCA
#define ASDATAR10L	0xCB
#define ASDATAR11H	0xCC
#define ASDATAR11L	0xCD
#define ASDATAR12H	0xCE
#define ASDATAR12L	0xCF

/* Page 0x30 (MCU_PAGE) SR300PC10 */
#define MCUCTL		0x10
#define MCUCFG1		0x18
#define MCUCFG2		0x21
#define MCUCFG3		0x22
#define MCUCFG4		0x23
#define MCUCFG5		0x24
#define BB2MCUCTL	0x30
#define BB2MCUADDRH	0x31
#define BB2MCUADDRL	0x32
#define BB2MCUDINB3	0x33
#define BB2MCUDINB2	0x34
#define BB2MCUDINB1	0x35
#define BB2MCUDINB0	0x36
#define BB2MCUDOUTB3	0x37
#define BB2MCUDOUTB2	0x38
#define BB2MCUDOUTB1	0x39
#define BB2MCUDOUTB0	0x3A
#define SBUSMEMCTL	0x40
#define SBUSMEMDIN	0x42
#define SBUSMEMDOUT	0x43
#define SBUSMEMADDRH	0x44
#define SBUSMEMADDRL	0x45
#define SBUSSEL		0x50
#define PORT0		0x60

#endif /* __HYNIX_YAC_REGS_H__ */
