#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/uaccess.h>
#include <mach/platform.h>
#include <mach/platform_id.h>


/*
 * Definitions
 */
#define MODULE_NAME "virtual-ts"
#define DEVICE_NAME MODULE_NAME

#define CABO_SCREEN_RES_X 480
#define CABO_SCREEN_RES_Y 272
#define BOGOTA_SCREEN_RES_X 1024
#define BOGOTA_SCREEN_RES_Y 600
#define XANADU_SCREEN_RES_X 1024
#define XANADU_SCREEN_RES_Y 600
#define QUITO_SCREEN_RES_X 800
#define QUITO_SCREEN_RES_Y 480


#define MAX_CONTACTS 10   /* Max slots */
#define MAX_PRESSURE 100  /* Max reported pressure for both MT / ST */


/*
 * Function Prototypes
 */
static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
static void process_event(char, int);


/*
 * Globals
 */
static int Major;            /* Major number assigned to our device driver */
static int Device_Open = 0;  /* Is device open?  Used to prevent multiple
                                        access to the device */
struct class * cl;
struct device * dev;
static struct input_dev *input_dev;

struct file_operations fops = {
  read: dev_read,
  write: dev_write,
  open: dev_open,
  release: dev_release,
};


/*
 * Functions
 */
static int dev_open(struct inode *inode, struct file *file) {
  if (Device_Open) return -EBUSY;
  Device_Open++;
  return 0;
}


static int dev_release(struct inode *inode, struct file *file) {
  Device_Open--;
  return 0;
}


static ssize_t dev_read(struct file *filp, char *buffer, size_t length, loff_t *offset) {
  loff_t off = *offset;
  const char* message =
      "Usage: write a command to /dev/"DEVICE_NAME"\n"

      "\n  Single touch:\n"
      "    x num \t set ABS_X\n"
      "    y num \t set ABS_Y\n"
      "    p num \t set ABS_PRESSURE\n"

      "\n  Key touch:\n"
      "    b num \t set BTN_TOUCH\n"

      "\n  Multi-touch:\n"
      "    s slot \t assign current slot ID\n"
      "    a num \t report slot state\n"
      "    c num \t report finger count\n"
      "    X num \t set ABS_MT_POSITION_X\n"
      "    Y num \t set ABS_MT_POSITION_Y\n"
      "    P num \t set ABS_MT_PRESSURE\n"
      "    t num \t assign tracking ID\n"
      "    e     \t trigger emulated pointer (single touch events) \n"

      "\n  Synchronization:\n"
      "    S     \t sync events\n"
      "    M     \t multi-touch sync events\n"
      "\nFor information about events, see <linux/input.h>, <linux/input/mt.h>\n"
      "\n";

  const size_t msgsize = strlen(message);

  if (off >= msgsize)
      return 0;

  if (length > msgsize - off)
      length = msgsize - off;

  if (copy_to_user(buffer, message+off, length) != 0)
      return -EFAULT;

  *offset += length;
  return length;
}

static ssize_t dev_write(struct file *filp, const char *buff, size_t len, loff_t *off) {
  char command;
  int value;

  int i;
  int p = 0;
  for(i = 0; i < len; ++i) {
    if (buff[i] == '\n') {
      sscanf(buff+p, "%c%d", &command, &value);
      p = i + 1;
      value = (value < 0) ? 0 : value; // not every command has an argument
      process_event(command, value);
    }
  }

  return len;
}


static void process_event(char cmd, int val) {
  switch(cmd) {
    /* single touch */
    case 'x':
        input_report_abs(input_dev, ABS_X, val);
        break;
    case 'y':
        input_report_abs(input_dev, ABS_Y, val);
        break;
    case 'p':
        input_report_abs(input_dev, ABS_PRESSURE, val);
        break;

    /* key touch */
    case 'b':
        if (val == 0 || val == 1)
          input_report_key(input_dev, BTN_TOUCH, val);
        else
          printk("Invalid state: %d, Set to 0 or 1\n", val);
        break;

    /* multi-touch */
    case 's':
        if (val >= 0 && val < MAX_CONTACTS)
          input_mt_slot(input_dev, val);
        else
          printk("Invalid slot: %d, %d slots [0,%d]\n", val, MAX_CONTACTS,
               MAX_CONTACTS-1);
        break;
    case 'a':
        if (val == 0 || val == 1)
          input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, val);
        else
          printk("Invalid state: %d, Set to 0 or 1\n", val);
        break;
    case 'c':
        if (val > 0 && val <= 5)
          input_mt_report_finger_count(input_dev, val);
        else
          printk("Invalid count: %d, Set to 0-5\n", val);
        break;
    case 'X':
        input_report_abs(input_dev, ABS_MT_POSITION_X, val);
        break;
    case 'Y':
        input_report_abs(input_dev, ABS_MT_POSITION_Y, val);
        break;
    case 'P':
        input_report_abs(input_dev, ABS_MT_PRESSURE, val);
        break;
    case 'e':
        /* compat: single touch */
        if (val == 0 || val == 1)
          input_mt_report_pointer_emulation(input_dev, val);
        else
          printk("Invalid state: %d, Set to 0 or 1\n", val);
        break;
    case 't':
        input_report_abs(input_dev, ABS_MT_TRACKING_ID, val);
        break;

    /* synchronization */
    case 'S':
        input_sync(input_dev);
        break;
    case 'M':
        input_mt_sync(input_dev);
        break;

    default:
        printk(KERN_WARNING "%s: Unknown cmd = %c, val = %d\n",
            MODULE_NAME, cmd, val);
  }
}


static int __init virtual_ts_init(void)
{
  int err;

  int screen_res_x = 0;
  int screen_res_y = 0;

	switch (get_leapfrog_platform()) {
    case BOGOTA:
      screen_res_x = BOGOTA_SCREEN_RES_X;
      screen_res_y = BOGOTA_SCREEN_RES_Y;
      break;
    case CABO:
      screen_res_x = CABO_SCREEN_RES_X;
      screen_res_y = CABO_SCREEN_RES_Y;
      break;
    case XANADU:
      screen_res_x = XANADU_SCREEN_RES_X;
      screen_res_y = XANADU_SCREEN_RES_Y;
      break;
    case QUITO:
      screen_res_x = QUITO_SCREEN_RES_X;
      screen_res_y = QUITO_SCREEN_RES_Y;
      break;
    default:
      printk("virtual-ts: Could not detect touchscreen resolution from board ID %d!\n", get_leapfrog_platform());
      break;
  }


  printk (MODULE_NAME ": init\n");

  input_dev = input_allocate_device();
  if (!input_dev)
    return -ENOMEM;

  input_dev->name = DEVICE_NAME;

  input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
  input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) |
      BIT_MASK(BTN_TOOL_FINGER) | BIT_MASK(BTN_TOOL_DOUBLETAP) |
      BIT_MASK(BTN_TOOL_TRIPLETAP) | BIT_MASK(BTN_TOOL_QUADTAP) |
      BIT_MASK(BTN_TOOL_QUINTTAP);

  input_set_abs_params(input_dev, ABS_X, 0, screen_res_x, 0, 0);
  input_set_abs_params(input_dev, ABS_Y, 0, screen_res_y, 0, 0);
  input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_PRESSURE, 0, 0);

  __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
  input_mt_init_slots(input_dev, MAX_CONTACTS);

  input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, screen_res_x, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, screen_res_y, 0, 0);
  input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_PRESSURE, 0, 0);

  err = input_register_device(input_dev);
  if (err)
    goto fail1;

  Major = register_chrdev(0, DEVICE_NAME, &fops);
  if (Major < 0) {
    printk ("Registering the character device failed with %d\n", Major);
    goto fail1;
  }

  cl = class_create(THIS_MODULE, DEVICE_NAME);
  if (!IS_ERR(cl)) {
    dev = device_create(cl, NULL, MKDEV(Major,0), NULL, DEVICE_NAME);
  }

  return 0;

fail1:
  input_free_device(input_dev);
  return err;
}


static void __exit virtual_ts_exit(void)
{
  printk (MODULE_NAME ": exit\n");

  input_unregister_device(input_dev);

  if ( !IS_ERR(cl) ) {
    device_destroy(cl, MKDEV(Major,0));
    class_destroy(cl);
  }
  unregister_chrdev(0, DEVICE_NAME);
}

module_init(virtual_ts_init);
module_exit(virtual_ts_exit);

MODULE_AUTHOR("Robert Alfaro <ralfaro@leapfrog.com>");
MODULE_DESCRIPTION("Virtual Touchscreen driver");
MODULE_LICENSE("GPL");
