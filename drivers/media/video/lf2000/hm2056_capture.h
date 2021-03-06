static const char HM2056_Capture_UXGA[][3] = {
{0x00,0x05,0x00},
{0x00,0x06,0x03},
{0x00,0x0D,0x00},
{0x00,0x0E,0x00},
{0x00,0x12,0x04},
{0x00,0x13,0x00},
{0x00,0x2A,0x1F},	//modified by Terence for Frame rate 10 fps
{0x05,0x41,0x9B},
{0x05,0x43,0xBA},

{0x00,0x71,0xAB},
{0x00,0x82,0xE2},
{0x01,0x1F,0xFF},
{0x01,0x25,0xDF},
{0x01,0x26,0x70},
{0x01,0x31,0xBC},
{0x01,0x44,0x04},
{0x01,0x90,0x87},
{0x01,0x92,0x50},
{0x03,0x8F,0x04},
{0x03,0x90,0xD8},
{0x03,0xB7,0xEA},
{0x03,0xB8,0x00},
{0x03,0xB9,0x3A},
{0x03,0xBA,0x01},
{0x05,0x41,0x67},		//modified by terence flicker
{0x05,0x43,0x7C},		//modified by terence flicker
{0x05,0xE4,0x0A},
{0x05,0xE5,0x00},
{0x05,0xE6,0x49},
{0x05,0xE7,0x06},
{0x05,0xE8,0x0A},
{0x05,0xE9,0x00},
{0x05,0xEA,0xB9},
{0x05,0xEB,0x04},
{0x00,0x7C,0x04},
{0x00,0x00,0x01},
{0x01,0x00,0x01},
{0x01,0x01,0x01},
{0x00,0x05,0x01},

};

static const char HM2056_Capture_SVGA[][3] = {
{0x00,0x05,0x00},
{0x00,0x06,0x03},
{0x00,0x0D,0x11},
{0x00,0x0E,0x11},
{0x00,0x12,0x1C},
{0x00,0x13,0x01},
{0x00,0x2A,0x1F},
{0x00,0x71,0xFF},
{0x00,0x82,0xA2},
{0x01,0x1F,0xF7},
{0x01,0x25,0xDF},
{0x01,0x26,0x70},
{0x01,0x31,0xBD},
{0x01,0x44,0x06},
{0x01,0x90,0x80},
{0x01,0x92,0x48},
{0x03,0x8F,0x04},
{0x03,0x90,0xE8},
{0x03,0xB7,0xEA},
{0x03,0xB8,0x00},
{0x03,0xB9,0x3A},
{0x03,0xBA,0x01},
{0x05,0x41,0x9D},
{0x05,0x43,0xBC},
{0x05,0xE4,0x05},
{0x05,0xE5,0x00},
{0x05,0xE6,0x24},
{0x05,0xE7,0x03},
{0x05,0xE8,0x08},
{0x05,0xE9,0x00},
{0x05,0xEA,0x5F},
{0x05,0xEB,0x02},
{0x00,0x7C,0x04},
{0x00,0x00,0x01},
{0x01,0x00,0x01},
{0x01,0x01,0x01},
{0x00,0x05,0x01},

};

static const char HM2056_Capture_QSVGA[][3] = {
{0x00,0x05,0x00},
{0x00,0x06,0x03},
{0x00,0x0D,0x11},
{0x00,0x0E,0x11},
{0x00,0x12,0x1C},
{0x00,0x13,0x01},
{0x00,0x2A,0x1F},
{0x00,0x71,0xFF},
{0x00,0x82,0xA2},
{0x01,0x1F,0xF7},
{0x01,0x25,0xFF},
{0x01,0x26,0x70},
{0x01,0x31,0xBD},
{0x01,0x44,0x06},
{0x01,0x90,0x80},
{0x01,0x92,0x48},
{0x03,0x8F,0x04},
{0x03,0x90,0xE8},
{0x03,0xB7,0xEA},
{0x03,0xB8,0x00},
{0x03,0xB9,0x3A},
{0x03,0xBA,0x01},
{0x05,0x41,0x9D},
{0x05,0x43,0xBC},
{0x05,0xE0,0x00},//400*300 
{0x05,0xE1,0x01},
{0x05,0xE2,0x00},
{0x05,0xE3,0x01},
{0x05,0xE4,0x02},
{0x05,0xE5,0x00},
{0x05,0xE6,0x91},
{0x05,0xE7,0x01},
{0x05,0xE8,0x01},
{0x05,0xE9,0x00},
{0x05,0xEA,0x30},
{0x05,0xEB,0x01},
{0x00,0x7C,0x33},
{0x00,0x00,0x01},
{0x01,0x00,0x01},
{0x01,0x01,0x01},
{0x00,0x05,0x01},

};
