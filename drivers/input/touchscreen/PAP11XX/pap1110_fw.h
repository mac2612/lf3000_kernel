/* drivers/input/touchscreen/pap1110_fw.h
 *
 * Copyright (C) 2013 Pixart Imaging Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * NOTES
 *  2/23/14 Update FW to 0x42. Asia team tried it a week ago.
 */

// #define PAP1110_FW_VERSION	0x3F
// #define PAP1110_FW_VERSION	0x42
#define PAP1110_FW_VERSION	0x45

#define PAP1110_DS_CRC_V4_HI	0x6c
#define PAP1110_DS_CRC_V4_LO	0x6a
#define PAP1110_DS_CRC_V2_HI	0xf2
#define PAP1110_DS_CRC_V2_LO	0x14

#define PAP1110_CR_CRC_HI	0xfc
#define PAP1110_CR_CRC_LO	0xc5

static unsigned char fw_pap1110[] = {
// #include "flashcode_rev_0x3F_PAP1110_ID_0x8A_ROM_0x10_CRC_FC83.inc"
// #include "flashcode_rev_0x42_PAP1110_ID_0x8A_ROM_0x10_CRC_54A2.inc"
#include "flashcode_rev_0x45_PAP1110_ID_0x8A_ROM_0x10_CRC_07A2.inc"

};

