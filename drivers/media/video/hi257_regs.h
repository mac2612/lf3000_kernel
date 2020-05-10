/*
 * Hynix YAC Camera Driver
 *
 * Copyright (c) 2011 Leapfrog Enterprises Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __HI257_REGS_H__
#define __HI257_REGS_H__

/* Sensor IDs (WINDOW_PAGE:DEVID) */
#define HI257_ID 0xC4

/* Pages - used as indicies to mapping tables */
enum register_page {
	WINDOW_PAGE = 0,
	FORMAT_PAGE ,
	NOISE1_PAGE,
	NOISE2_PAGE,
	EDGE_PAGE,
	LENS_PAGE,
	COLOR_PAGE,
	GAMMA_PAGE,
	FLICKER_PAGE,
	SCALING_PAGE,
	MULTICOLOR_PAGE,
	AUTOEXP_PAGE,
	AEWGTCF_PAGE,
	AUTOWB_PAGE,

	/* undocumented */
	UNDOC_0x02_PAGE,

	/* delimiter */
	NO_SUCH_PAGE,
};

/* Register addresses (within given page) */
/* Common (All pages) */
#define PAGEMODE	0x03

/* Page 0x0 (WINDOW_PAGE) Common */
#define PWRCTL		0x01
#define DEVID		0x04
#define PADCTL1		0x09
#define PADCTL2		0x0A
#define PLLCTL1		0x0E
#define PLLCTL2		0x0F
#define VDOCTL1		0x10
#define VDOCTL2		0x11
#define SYNCCTL		0x12
#define VDOCTL3		0X14
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
#define BLCOFSDB	0xA0

/* Page 0x10 (FORMAT_PAGE) Common */
#define ISPCTL1		0x10
#define ISPCTL2		0x11
#define ISPCTL3		0x12
#define ISPCTL4		0x13
#define YOFS		0x40
#define UOFS		0x42
#define VOFS		0x43
#define UCON		0x44
#define VCON		0x45
#define SOLARI		0x46
#define CONTRAST	0x48
#define SATCTL		0x60
#define SATB		0x61
#define SATR		0x62
#define AGSAT		0x63
#define SATTIMETH	0x66
#define SATOUTDEL	0x67

/* Page 0x11 (NOISE1_PAGE) Hi-257 */
#define GCTL		0x70

/* Page 0x12 (NOSIE2_PAGE) Common */
#define YCLPFCTL1	0x20
#define YCLPFCTL2	0x21
#define YCLPFCTL3	0x22
#define YCOUTDYTH	0X23
#define YCINDDYTH	0X24
#define YCDAKDYTH	0X25
#define YCOUTLUMHI	0X30
#define YCOUTLUMLO	0X31
#define YCOUTSTDHI	0x32
#define YCOUTSTDLO	0x33
#define YCOUTRAT	0x34
#define YCINDLUMHI  0x35
#define YCINDLUMLO	0x36
#define YCINDSTDHI	0x37
#define YCINDSTDLO	0x38
#define YCINDRAT	0x39
#define YCDAKLUMHI	0x3A
#define YCDAKLUMLO	0x3B
#define YCDAKSTDHI	0x3C
#define YCDAKSTDLO	0x3D
#define YCDAKRAT	0x3E
#define YCINTHH		0x4C
#define YCINTHL		0x4D
#define YCDARK1THH	0x52
#define YCDARK1THL	0x53
#define YCCAWMTHO	0x70
#define YCCDFTHO	0x71
#define YCCAWMTHID	0x72
#define YCCDFTHID	0x73
#define YCCAWMTHDK	0x74
#define YCCDFTHDK	0x75

/* Page 0x13 (EDGE_PAGE) Common */
#define EDGE2DCTL1	0x80
#define EDGE2DCTL2	0x81
#define EDGE2DCTL3	0x82
#define EDGE2DCTL4	0x83
#define EDGE2DLCLIPLMT	0x93
#define EDGE2DHCLIP1TH	0x94
#define EDGE2DHCLIP2TH	0x95
#define EDGE2DLCOUT1N	0xA2
#define EDGE2DLCOUT1P	0xA3
#define EDGE2DLCINN	0xA4
#define EDGE2DLCINP	0xA5
#define EDGE2DLCD1N	0xA6
#define EDGE2DLCD1P	0xA7

/* Page 0x14 (LENS_PAGE) Common */
#define LENSCTL1	0x10
#define XCEN		0x20
#define YCEN		0x21
#define LENSRGAIN	0x22
#define LENSGGAIN	0x23
#define LENSBGAIN	0x24

/* Page 0x15 (COLOR_PAGE) Common */
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


/* Page 0x16 (GAMMA_PAGE) Common */
#define GMACTL		0x10
#define GGMA0		0x30 /*GGMA0-GGMA18 == INDGMA0-INDGMA18, AS PER THE DATASHEET */
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
#define RGMA0		0x50 /*RGMA0-RGMA18 == OUTGMA0-OUTGMA18, AS PER THE DATASHEET */
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
#define BGMA0		0x70 /*BGMA0-BGMA18 == DAKGMA0-DAKGMA18, AS PER THE DATASHEET */
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


/* Page 0x17 (FLICKER_PAGE) Common */
#define FLKMODE		0xC0
#define FLK200		0xC4
#define FLK240		0xC5
#define FLKTH1		0xC6
#define FLKTH2		0xC7


/* Page 0x18 (SCALING_PAGE) Common */
#define MODEZOOM1	0x10
#define ZOOMCONFIG	0x12
#define ZOOMCFG2	0x13
#define ZOOMINTPOL	0x14
#define ZOOMUPDATE	0x15
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


/* Page 0x19 (MULTICOLOR_PAGE) */
#define MCMCCTL1	0x10
#define MCMCCTL2	0x11
#define DELTA1	0x12
#define CENTER1	0x13
#define DELTA2	0x14
#define CENTER2	0x15
#define DELTA3	0x16
#define CENTER3	0x17
#define DELTA4	0x18
#define CENTER4	0x19
#define DELTA5	0x1A
#define CENTER5	0x1B
#define DELTA6	0x1C
#define CENTER6	0x1D
#define SATGAIN1	0x1E
#define SATGAIN2	0x1F
#define SATGAIN3	0x20
#define SATGAIN4	0x21
#define SATGAIN5	0x22
#define SATGAIN6	0x23
#define HUEANGLE1	0x24
#define HUEANGLE2	0x25
#define HUEANGLE3	0x26
#define HUEANGLE4	0x27
#define HUEANGLE5	0x28
#define HUEANGLE6	0x29
#define MCMCCTL3	0x53
#define MCMCLUMCTL1	0x6C
#define MCMCLUMCTL2	0x6D
#define MCMCLUMCTL3	0X6E
#define RG1LUMGAIN1	0x71
#define RG1LUMGAIN2 0x72
#define RG1LUMGAIN3	0x73
#define RG1LUMGAIN4	0x74
#define RG1LUM1	0x75
#define RG1LUM2	0x76
#define RG1LUM3	0x77
#define RG1LUM4	0x78
#define RG2LUMGAIN1	0x79
#define RG2LUMGAIN2 0x7A
#define RG2LUMGAIN3	0x7B
#define RG2LUMGAIN4	0x7C
#define RG2LUM1	0x7D
#define RG2LUM2	0x7E
#define RG2LUM3	0x7F
#define RG2LUM4	0x80
#define RG3LUMGAIN1 0x81
#define RG3LUMGAIN2 0x82
#define RG3LUMGAIN3	0x83
#define RG3LUMGAIN4	0x84
#define RG3LUM1 0x85
#define RG3LUM2	0x86
#define RG3LUM3	0x87
#define RG3LUM4	0x88
#define RG4LUMGAIN1	0x89
#define RG4LUMGAIN2	0x8A
#define RG4LUMGAIN3	0x8B
#define RG4LUMGAIN4	0x8C
#define RG4LUM1	0x8D
#define RG4LUM2	0x8E
#define RG4LUM3	0x8F
#define RG4LUM4	0x90
#define RG5LUMGAIN1	0x91
#define RG5LUMGAIN2	0x92
#define RG5LUMGAIN3	0x93
#define RG5LUMGAIN4	0x94
#define RG5LUM1	0x95
#define RG5LUM2	0x96
#define RG5LUM3	0x97
#define RG5LUM4	0x98
#define RG6LUMGAIN1	0x99
#define RG6LUMGAIN2	0x9A
#define RG6LUMGAIN3	0x9B
#define RG6LUMGAIN4	0x9C
#define RG6LUM1	0x9D
#define RG6LUM2	0x9E
#define RG6LUM3	0x9F
#define RG6LUM4	0xA0
#define ALLGAINCTL1	0xA1


/* Page 0x20 (AUTOEXP_PAGE) Common */
#define AECTL1		0x10
#define AECTL2		0x11
#define AEFRAMECTL1	0x20
#define AEFINECTL1	0x28
#define AEFINECTL2	0x29
#define AEFINECTL3	0x2A
#define AEFINECTL4	0x2B
#define BLTHST	0x56
#define BLTHEN	0x57
#define BLTVST	0x58
#define BLTVED	0x59
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
#define EXPMAXH		0x88 //Datasheet name - EXPMAX120H
#define EXPMAXM		0x89 //Datasheet name - EXPMAX120M
#define EXPMAXL		0x8A //Datasheet name - EXPMAX120L
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
#define EXPMAX100H	0xA5
#define EXPMAX100M	0xA6
#define	EXPMAX100L	0xA7
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
#define AGSKY	0xC0
#define AGLVLL	0xC2
#define YAVG		0xD3

/* Page 0x21 AEWGTCF_PAGE */
#define AEWGT1		0x20
#define AEWGT2		0x21
#define AEWGT3		0x22
#define AEWGT4		0x23
#define AEWGT5		0x24
#define AEWGT6		0x25
#define AEWGT7		0x26
#define AEWGT8		0x27
#define AEWGT9		0x28
#define AEWGT10		0x29
#define AEWGT11		0x2A
#define AEWGT12		0x2B
#define AEWGT13		0x2C
#define AEWGT14		0x2D
#define AEWGT15		0x2E
#define AEWGT16		0x2F
#define AEWGT17		0x30
#define AEWGT18		0x31
#define AEWGT19		0x32
#define AEWGT20		0x33
#define AEWGT21		0x34
#define AEWGT22		0x35
#define AEWGT23		0x36
#define AEWGT24		0x37
#define AEWGT25		0x38
#define AEWGT26		0x39
#define AEWGT27		0x3A
#define AEWGT28		0x3B
#define AEWGT29		0x3C
#define AEWGT30		0x3D
#define AEWGT31		0x3E
#define AEWGT32		0x3F

/* Page 0x22 (AUTOWB_PAGE) Common */
#define AWBCTL1		0x10
#define AWBCTL2		0x11
#define AWBMAP		0x19
#define ULVL		0x30
#define VLVL		0x31
#define UVTH1		0x38
#define UVTH2		0x39
#define YRANGE		0x40
#define CDIFF		0x41
#define CSUM2		0x42
#define YRANGETOT	0x43
#define CDIFFTOT	0x44
#define CSUM2TOT	0x45
#define WHTPXLTH	0x46
#define STBYTH		0x47
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
#define HTMPTHOFF	0xAD
#define HTMPTHON	0xAE
#define LTMPTHMIN	0xAF
#define LTMPTHMAX	0xB0
#define MRGAIN		0xB2
#define MBGAIN		0xB3
#define LTMPOFS		0xB8
#define HTMPOFS		0xB9
#define UAVG		0xD2
#define VAVG		0xD3

#endif /* __HI257_REGS_H__ */
