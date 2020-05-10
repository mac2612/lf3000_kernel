
#ifndef _ALIVE_H_
#define _ALIVE_H_

/* Get and set panic value in alive scratch register */
void alive_set_panic(unsigned value);
unsigned alive_get_panic(void);

#if 1	/* 15dec11 */

/* from emerald-boot's gpio.h */
/*
 * Scratchpad Register usage
 */

#define SCRATCH_POWER_POS          0
#define SCRATCH_POWER_SIZE         2

#define SCRATCH_SHUTDOWN_POS       2
#define SCRATCH_SHUTDOWN_SIZE      1

#define SCRATCH_REQUEST_POS 	   3
#define SCRATCH_REQUEST_SIZE  	   3

#define SCRATCH_BOOT_IMAGE_POS     6
#define SCRATCH_BOOT_IMAGE_SIZE    2

#define SCRATCH_BOARD_ID_POS       8
#define SCRATCH_BOARD_ID_SIZE      5

#define SCRATCH_CART_ID_POS       13
#define SCRATCH_CART_ID_SIZE       4

#define SCRATCH_BOOT_SOURCE_POS   17
#define SCRATCH_BOOT_SOURCE_SIZE   3

#define SCRATCH_PANIC_POS	  20
#define SCRATCH_PANIC_SIZE         2

#define SCRATCH_REBOOT_POS 22
#define SCRATCH_REBOOT_SIZE 1

#define SCRATCH_USER_0_POS        23
#define SCRATCH_USER_0_SIZE       (32-SCRATCH_USER_0_POS)

/*
 * SCRATCHPAD Enums
 * Note that scratchpad register is cleared to zero on first power up
 */

/* Track power state */
enum scratch_power {                 // set by bootstrap, read by others
        SCRATCH_POWER_FIRSTBOOT = 0, // only seen in bootstrap
        SCRATCH_POWER_COLDBOOT  = 1, // batteries replaced
        SCRATCH_POWER_WARMBOOT  = 2, // second and subsequent boots
};

/* Shutdown semaphore.  Set by Linux shutdown to signal clean system shutdown */
enum scratch_shutdown {
        SCRATCH_SHUTDOWN_CLEAN = 0,  // set by Linux at clean shutdown
        SCRATCH_SHUTDOWN_DIRTY = 1,  // cleared by bootstrap at boot
};

/* Choose boot partition.  Used by REQUEST_BOOT and ACTUAL_BOOT bits to
 * indicate the preferred boot partition and the partition booted
 */
enum scratch_boot_image {
        SCRATCH_BOOT_IMAGE_RECOVERY = 0,   // boot recovery image
        SCRATCH_BOOT_IMAGE_PLAY     = 1,   // normal boot
        SCRATCH_BOOT_IMAGE_2        = 2,   // unused
        SCRATCH_BOOT_IMAGE_3        = 3,   // unused
};

/* Choose boot partition.  Used by REQUEST_BOOT and ACTUAL_BOOT bits to
 * indicate the preferred boot partition and the partition booted
 */
enum scratch_boot_source {
        SCRATCH_BOOT_SOURCE_UNKNOWN = 0,   //
        SCRATCH_BOOT_SOURCE_NOR     = 1,   //
        SCRATCH_BOOT_SOURCE_NAND    = 2,   //
        SCRATCH_BOOT_SOURCE_UART    = 3,   //
        SCRATCH_BOOT_SOURCE_USB     = 4,   //
};

/* save boot source. */
enum scratch_request {
	SCRATCH_REQUEST_PLAY    = 0,  // Launch Play if possible
	SCRATCH_REQUEST_RETURN  = 1,  // Return to Play if possible
	SCRATCH_REQUEST_UPDATE  = 2,  // Enter recovery in update mode
#ifdef CONFIG_PLAT_NXP4330_GLASGOW
	SCRATCH_REQUEST_SLEEP   = 3,  // Enter sleep
#else
	SCRATCH_REQUEST_BATTERY = 3,  // Enter play: battery failed
#endif
    SCRATCH_REQUEST_UNCLEAN = 4,  // Enter play: dirty shutdown
	SCRATCH_REQUEST_FAILED  = 5,  // Enter recovery in update mode: boot failed
	SCRATCH_REQUEST_SHORT   = 6,  // Enter play in short-circuit mode
	SCRATCH_REQUEST_TRAPDOOR= 7,  // Enter recovery in trapdoor mode
};

/* save state if it was a reboot */
enum scratch_reboot {
	SCRATCH_REBOOT_NO = 0,
	SCRATCH_REBOOT_YES = 1,
};

void alive_set_request(enum scratch_request request);
enum scratch_request alive_get_request(void);
#endif

#endif //_ALIVE_H_

