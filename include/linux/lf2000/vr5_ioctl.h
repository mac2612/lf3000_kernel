//+-----------------------------------------------------------------------
//|	inclusion guard
//+-----------------------------------------------------------------------
#ifndef __VR5_IOCTL_H__
#define __VR5_IOCTL_H__

//+-----------------------------------------------------------------------
//|	pre-including files
//+-----------------------------------------------------------------------
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/fs.h>

//+-----------------------------------------------------------------------
//|	define
//+-----------------------------------------------------------------------
#define VR5_GRP_BASE		0x08	
#define VR5_GRP_COMMAND		(VR5_GRP_BASE + 0x10)
#define VR5_GRP_MEMORY		(VR5_GRP_BASE + 0x20)
#define VR5_GRP_MISC		(VR5_GRP_BASE + 0x30)

//+-----------------------------------------------------------------------
typedef enum {
	VR5_IOCTL_GET_BUILD_NUMBER,
	VR5_IOCTL_START_COMMAND,
	//VR5_IOCTL_GET_MEM_BLOCK,
	//VR5_IOCTL_FREE_MEM_BLOCK,
	VR5_IOCTL_SIZE,
} _ioctl_nr_t;

typedef struct {
	unsigned int user_ctx;
	unsigned int cmd_phys_base;
	unsigned int cmd_size;
} _ioc_command_t;

typedef struct {
	unsigned int block;
	unsigned int phys_base;
	unsigned int size;
} _ioc_mem_block_t;


//+-----------------------------------------------------------------------
//| end of inclusion guard
//+-----------------------------------------------------------------------
#endif //__VR5_IOCTL_H__
