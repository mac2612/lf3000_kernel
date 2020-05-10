/* drivers/input/touchscreen/ektf2k.c - ELAN EKTF2K verions of driver
 *
 * Copyright (C) 2011 Elan Microelectronics Corporation.
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
 * 2011/12/06: The first release, version 0x0001
 * 2012/2/15:  The second release, version 0x0002 for new bootcode
 * 2012/5/8:   Release version 0x0003 for china market
 *             Integrated 2 and 5 fingers driver code together and
 *             auto-mapping resolution.
 * 2012/12/1:  Release version 0x0005: support up to 10 fingers but no buffer mode.
 *             Please change following parameters
 *                 1. For 5 fingers protocol, please enable ELAN_PROTOCOL.
                      The packet size is 18 or 24 bytes.
 *                 2. For 10 fingers, please enable both ELAN_PROTOCOL and ELAN_TEN_FINGERS.
                      The packet size is 40 or 4+40+40+40 (Buffer mode) bytes.
 *                 3. Please enable the ELAN_BUTTON configuraton to support button.
 *                 4. For ektf3k serial, Add Re-Calibration Machanism
 *                    So, please enable the define of RE_CALIBRATION.
 * SZ: gpio_get_value() failed. Need to convert it.
 * 2014/03/12 Barry: 1. Change to parse 5-finger protocol.
 *                   2. Modify I2C Address to 0x10.
 *                   3. Modify TP resolution for 7" Lima.
 *                   4. Modify FW size for eKTF2144.
 *                   5. Using {0x45 0x49 0x41 0x50} as the IAP command.
 *                   6. Fix the bug of missing finger up event.
 *
 */

/* The ELAN_PROTOCOL support normanl packet format */

//#define ELAN_BUFFER_MODE
//#define ELAN_TEN_FINGERS   /* james check: Can not be use to auto-resolution mapping */
#define ELAN_PROTOCOL
//#define ELAN_BUTTON //0523-12
//#define RE_CALIBRATION    /* The Re-Calibration was designed for ektf3k serial. */
//#define ELAN_2WIREICE
//#define ELAN_POWER_SOURCE

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
//SZSZ not exist yet #include

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>

#include <linux/input/mt.h>

#include <mach/soc.h>
#include <cfg_type.h>
#include <plat/ektf2k7.h>
// #include "ektf2k7.h" // also used by device.c

#ifdef ELAN_TEN_FINGERS
#define PACKET_SIZE   45    /* support 10 fingers packet for nexus7 55 */
#else
//#define PACKET_SIZE   8     /* support 2 fingers packet  */
#define PACKET_SIZE   25      /* support 5 fingers packet  */
#endif

#define PWR_STATE_DEEP_SLEEP  0
#define PWR_STATE_NORMAL      1
#define PWR_STATE_MASK        BIT(3)

#define CMD_S_PKT   0x52
#define CMD_R_PKT   0x53
#define CMD_W_PKT   0x54

#define HELLO_PKT   0x55

#define TWO_FINGERS_PKT   0x5A
#define FIVE_FINGERS_PKT  0x5D
#define MTK_FINGERS_PKT   0x6D
#define TEN_FINGERS_PKT   0x62
#define BUFFER_PKT        0x63
#define BUFFER55_PKT      0x66

#define RESET_PKT   0x77
#define CALIB_PKT   0x66

#define MAX_Y    1024    // portrait mode
#define MAX_X     600

// modify
#define CFG_IO_TOUCH_SCREEN_RESET     ((PAD_GPIO_B + 24))
//TODO
//#define CFG_IO_TOUCH_SCREEN_INT       (TOUCHSCREEN_X2)
//#define CFG_IO_TOUCH_SCREEN_RESET     (TOUCHSCREEN_Y2)
/*
  .gpio_reset = CFG_IO_TOUCH_SCREEN_RESET,
  // .reset_polarity  = 0,
  .reset_cfg = 0, // Active low
*/
// #define SYSTEM_RESET_PIN_SR 62 // nexus7 TEGRA_GPIO_PH6  62
#define SYSTEM_RESET_PIN_SR CFG_IO_TOUCH_SCREEN_RESET // LF Cabo
//Add these Define
#define IAP_PORTION
#define PAGERETRY  30
#define IAPRESTART 5


// For Firmware Update
#define ELAN_IOCTLID  0xD0
#define IOCTL_I2C_SLAVE           _IOW(ELAN_IOCTLID, 1, int)
#define IOCTL_MAJOR_FW_VER        _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER        _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET               _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK       _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER              _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION        _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION        _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID               _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE     _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK     _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT             _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME              _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK          _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK        _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE           _IOR(ELAN_IOCTLID, 17, int)
#define IOCTL_BC_VER              _IOR(ELAN_IOCTLID, 18, int)
#define IOCTL_2WIREICE            _IOR(ELAN_IOCTLID, 19, int)

#define CUSTOMER_IOCTLID  0xA0
#define IOCTL_CIRCUIT_CHECK       _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE  _IOR(CUSTOMER_IOCTLID, 2, int)

#define FINGER_NUM        5 //0522

#undef gpio_get_value
#define gpio_get_value    nxp_soc_gpio_get_in_value

//SZSZ resolution settings did not work. Use scale factor for now.
//int x_scale = 272.0/1280;
//int y_scale = 480.0/2112;
//FW showed 1024x1792
static float x_scale = MAX_X*1.0/ELAN_X_MAX;  //  960
static float y_scale = MAX_Y*1.0/ELAN_Y_MAX;  // 1728

static uint8_t RECOVERY=0x00;
static int FW_VERSION=0x00;
static int X_RESOLUTION=ELAN_X_MAX;  // nexus7 1280
static int Y_RESOLUTION=ELAN_Y_MAX; // nexus7 2112
static int FW_ID=0x00;
static int work_lock=0x00;
static int power_lock=0x00;
static int circuit_ver=0x01;
/*++++i2c transfer start+++++++*/
#if 0
static int file_fops_addr=0x10;
#endif
/*++++i2c transfer end+++++++*/

static int button_state = 0;

#ifdef IAP_PORTION
static int update_progree=0;
static uint8_t I2C_DATA[3] = {0x10, 0x20, 0x21};/*I2C devices address*/


/*The newest firmware, if update must be changed here*/
static uint8_t file_fw_data[] = {
//#include "fw_data.i"
};


enum
{
  PageSize    = 132,
  PageNum     = 377,
  ACK_Fail    = 0x00,
  ACK_OK      = 0xAA,
  ACK_REWRITE = 0x55,
};

enum
{
  E_FD        = -1,
};
#endif
struct elan_ktf2k_ts_data {
  struct i2c_client *client;
  struct input_dev *input_dev;
  struct workqueue_struct *elan_wq;
  struct work_struct work;
  //SZSZ  struct early_suspend early_suspend;
  int intr_gpio;
// Firmware Information
  int fw_ver;
  int fw_id;
  int bc_ver;
  int x_resolution;
  int y_resolution;
// For Firmare Update
  struct miscdevice firmware;
  struct attribute_group attrs;
};

static struct elan_ktf2k_ts_data *private_ts;
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client);
static int elan_ktf2k_ts_resume(struct i2c_client *client);

#ifdef IAP_PORTION
static int Update_FW_One(/*struct file *filp,*/ struct i2c_client *client, int recovery);
static int __hello_packet_handler(struct i2c_client *client);
#endif

#ifdef ELAN_2WIREICE
int elan_TWO_WIRE_ICE( struct i2c_client *client);
#endif

//0512 start
#define NO_DEBUG       0
#define DEBUG_TRACE    1
#define DEBUG_INFO     2
#define DEBUG_ERROR    3

static int Debug_Option = DEBUG_TRACE;
#define touch_debug(level, ...) \
        do { \
             if (Debug_Option >= (level)) \
                 printk("[elan]:" __VA_ARGS__); \
        } while (0)
//0512 end

//SZSZ
static void panel_reset(void)
{
/*  ELAN ORG
 *          gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
            msleep(20);
            gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
            msleep(5);
*/
/*  Setting for PAP11XX
 *          nxp_soc_gpio_set_out_value(pdata->gpio_reset,  (pdata->reset_cfg)); // reset
          // msleep (160); //quadruple x 10
          msleep (16); //quadruple
          nxp_soc_gpio_set_out_value(pdata->gpio_reset, !(pdata->reset_cfg)); // unreset/ena
          msleep (200); // waiting for PixArt's spec
          */
  nxp_soc_gpio_set_out_value(SYSTEM_RESET_PIN_SR, 0); // reset
  msleep (20); // waiting for ELAN's spec
  nxp_soc_gpio_set_out_value(SYSTEM_RESET_PIN_SR, 1); // unreset/ena
  msleep (20); // waiting for ELAN's spec
  return;
}



// For Firmware Update
static int elan_iap_open(struct inode *inode, struct file *filp){
        //printk("[ELAN]into elan_iap_open\n");
        touch_debug(DEBUG_INFO,"[ELAN]into elan_iap_open\n");
    if (private_ts == NULL) printk("private_ts is NULL~~~");

    return 0;
}

static int elan_iap_release(struct inode *inode, struct file *filp){
      return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp){
    int ret;
    char *tmp;
    //printk("[ELAN]into elan_iap_write\n");
    touch_debug(DEBUG_TRACE,"[ELAN]into elan_iap_write\n");
    /*++++i2c transfer start+++++++* //SZSZ /
    struct i2c_adapter *adap = private_ts->client->adapter;
    struct i2c_msg msg;
    / *++++i2c transfer end+++++++*/

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }

/*++++i2c transfer start+++++++*/
#if 0
  //down(&worklock);
  msg.addr = file_fops_addr;
  msg.flags = 0x00;// 0x00
  msg.len = count;
  msg.buf = (char *)tmp;
  //up(&worklock);
  ret = i2c_transfer(adap, &msg, 1);
#else

    ret = i2c_master_send(private_ts->client, tmp, count);
#endif
/*++++i2c transfer end+++++++*/

    if (ret != count) printk("ELAN i2c_master_send fail, ret=%d \n", ret);
    kfree(tmp);
    //return ret;
    return (ret == 1) ? count : ret;

}

static ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp){
    char *tmp;
    int ret;
    long rc;
    //printk("[ELAN]into elan_iap_read\n");
    touch_debug(DEBUG_TRACE,"[ELAN]into elan_iap_read\n");
   /*++++i2c transfer start+++++++*  //SZSZ /
      struct i2c_adapter *adap = private_ts->client->adapter;
      struct i2c_msg msg;
   / *++++i2c transfer end+++++++*/
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;
/*++++i2c transfer start+++++++*/
#if 0
  //down(&worklock);
  msg.addr = file_fops_addr;
  //msg.flags |= I2C_M_RD;
  msg.flags = 0x00;
  msg.flags |= I2C_M_RD;
  msg.len = count;
  msg.buf = tmp;
  //up(&worklock);
  ret = i2c_transfer(adap, &msg, 1);
#else
    ret = i2c_master_recv(private_ts->client, tmp, count);
#endif
/*++++i2c transfer end+++++++*/
    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);

    kfree(tmp);

    //return ret;
    return (ret == 1) ? count : ret;

}

static long elan_iap_ioctl( struct file *filp,    unsigned int cmd, unsigned long arg){

  int __user *ip = (int __user *)arg;
  touch_debug(DEBUG_INFO, "[ELAN]into elan_iap_ioctl cmd value %x\n", cmd);

  switch (cmd) {
    case IOCTL_I2C_SLAVE:
      private_ts->client->addr = (int __user)arg;
      //file_fops_addr = 0x15;
      break;
    case IOCTL_MAJOR_FW_VER:
      break;
    case IOCTL_MINOR_FW_VER:
      break;
    case IOCTL_RESET:
// modify
/*            gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
            msleep(20);
            gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
            msleep(5);
*/
/*  Setting for PAP11XX
 *          nxp_soc_gpio_set_out_value(pdata->gpio_reset,  (pdata->reset_cfg)); // reset
          // msleep (160); //quadruple x 10
          msleep (16); //quadruple
          nxp_soc_gpio_set_out_value(pdata->gpio_reset, !(pdata->reset_cfg)); // unreset/ena
          msleep (200); // waiting for PixArt's spec
          */
      /*
        nxp_soc_gpio_set_out_value(SYSTEM_RESET_PIN_SR, 0); // reset
        msleep (20); // waiting for ELAN's spec
        nxp_soc_gpio_set_out_value(SYSTEM_RESET_PIN_SR, 1); // unreset/ena
        msleep (20); // waiting for ELAN's spec
        */
        panel_reset();

      break;
    case IOCTL_IAP_MODE_LOCK:
      if(work_lock==0)
      {
        work_lock=1;
        disable_irq(private_ts->client->irq);
        cancel_work_sync(&private_ts->work);
      }
      break;
    case IOCTL_IAP_MODE_UNLOCK:
      if(work_lock==1)
      {
        work_lock=0;
        enable_irq(private_ts->client->irq);
      }
      break;
    case IOCTL_CHECK_RECOVERY_MODE:
      return RECOVERY;
      break;
    case IOCTL_FW_VER:
      __fw_packet_handler(private_ts->client);
      return FW_VERSION;
      break;
    case IOCTL_X_RESOLUTION:
      __fw_packet_handler(private_ts->client);
      return X_RESOLUTION;
      break;
    case IOCTL_Y_RESOLUTION:
      __fw_packet_handler(private_ts->client);
      return Y_RESOLUTION;
      break;
    case IOCTL_FW_ID:
      __fw_packet_handler(private_ts->client);
      return FW_ID;
      break;
    case IOCTL_ROUGH_CALIBRATE:
      return elan_ktf2k_ts_rough_calibrate(private_ts->client);
    case IOCTL_I2C_INT:
      put_user(gpio_get_value(private_ts->intr_gpio), ip);
      break;
    case IOCTL_RESUME:
      elan_ktf2k_ts_resume(private_ts->client);
      break;
    case IOCTL_POWER_LOCK:
      power_lock=1;
      break;
    case IOCTL_POWER_UNLOCK:
      power_lock=0;
      break;
#ifdef IAP_PORTION
    case IOCTL_GET_UPDATE_PROGREE:
      update_progree=(int __user)arg;
      break;
    case IOCTL_FW_UPDATE:
      Update_FW_One(private_ts->client, 0);
      break;
#endif
#ifdef ELAN_2WIREICE
    case IOCTL_2WIREICE:
      elan_TWO_WIRE_ICE(private_ts->client);
      break;
#endif
    case IOCTL_CIRCUIT_CHECK:
      return circuit_ver;
      break;
    default:
      printk("[elan] Un-known IOCTL Command %d\n", cmd);
      break;
  }
  return 0;
}

struct file_operations elan_touch_fops = {
        .open           =  elan_iap_open,
        .write          =  elan_iap_write,
        .read           =  elan_iap_read,
        .release        =  elan_iap_release,
        .unlocked_ioctl =  elan_iap_ioctl,
 };

 
#ifdef IAP_PORTION
static int EnterISPMode(struct i2c_client *client, uint8_t  *isp_cmd)
{
  char buff[4] = {0};
  int len = 0;

  len = i2c_master_send(private_ts->client, isp_cmd,  sizeof(isp_cmd));
  if (len != sizeof(buff)) {
    printk("[ELAN] ERROR: EnterISPMode fail! len=%d\r\n", len);
    return -1;
  }
  else
    printk("[ELAN] IAPMode write data successfully! cmd = [%2x, %2x, %2x, %2x]\n", isp_cmd[0], isp_cmd[1], isp_cmd[2], isp_cmd[3]);
  return 0;
}

#if 0 //old flow
static int ExtractPage(struct file *filp, uint8_t * szPage, int byte)
{
  int len = 0;

  len = filp->f_op->read(filp, szPage,byte, &filp->f_pos);
  if (len != byte)
  {
    printk("[ELAN] ExtractPage ERROR: read page error, read error. len=%d\r\n", len);
    return -1;
  }

  return 0;
}
#endif

static int WritePage(uint8_t * szPage, int byte)
{
  int len = 0;

  len = i2c_master_send(private_ts->client, szPage,  byte);
  if (len != byte)
  {
    printk("[ELAN] ERROR: write page error, write error. len=%d\r\n", len);
    return -1;
  }

  return 0;
}

static int GetAckData(struct i2c_client *client)
{
  int len = 0;

  char buff[2] = {0};

  len=i2c_master_recv(private_ts->client, buff, sizeof(buff));
  if (len != sizeof(buff)) {
    printk("[ELAN] ERROR: read data error, write 50 times error. len=%d\r\n", len);
    return -1;
  }

  pr_info("[ELAN] GetAckData:%x,%x",buff[0],buff[1]);
  if (buff[0] == 0xaa)
    return ACK_OK;
  else if (buff[0] == 0x55 && buff[1] == 0x55)
    return ACK_REWRITE;
  else
    return ACK_Fail;

  return 0;
}

static void print_progress(int page, int ic_num, int j)
{
  int i, percent,page_tatol,percent_tatol;
  char str[256];
  str[0] = '\0';
  for (i=0; i<((page)/10); i++) {
    str[i] = '#';
    str[i+1] = '\0';
  }

  page_tatol=page+377*(ic_num-j);
  percent = ((100*page)/(377));
  percent_tatol = ((100*page_tatol)/(377*ic_num));

  if ((page) == (377))
    percent = 100;

  if ((page_tatol) == (377*ic_num))
    percent_tatol = 100;

  printk("\rprogress %s| %d%%", str, percent);

  if (page == (377))
    printk("\n");
}
///////////// */


/*
* Restet and (Send normal_command ?)
* Get Hello Packet
*/
#if 0
static void elan_ktf2k_ts_hw_reset(void)
{

  panel_reset();
}
#endif

static int Update_FW_One(struct i2c_client *client, int recovery)
{
  int res = 0,ic_num = 1;
  int iPage = 0, rewriteCnt = 0; //rewriteCnt for PAGE_REWRITE
  int i = 0;
  uint8_t data;
  //struct timeval tv1, tv2;
  int restartCnt = 0; // For IAP_RESTART

  // uint8_t recovery_buffer[4] = {0};
  int byte_count;
  uint8_t *szBuff = NULL;
  int curIndex = 0;
  uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50}; //{0x45, 0x49, 0x41, 0x50};
                                                // 0x54, 0x00, 0x12, 0x34

  dev_dbg(&client->dev, "[ELAN] %s:  ic_num=%d\n", __func__, ic_num);
IAP_RESTART:
  //reset
// modify


  data=I2C_DATA[0];//Master
  dev_dbg(&client->dev, "[ELAN] %s: address data=0x%x \r\n", __func__, data);

  if(recovery != 0x80)
  {
        printk("[ELAN] Firmware upgrade normal mode !\n");
        /*
    gpio_set_value(SYSTEM_RESET_PIN_SR,0);
    mdelay(20);
    gpio_set_value(SYSTEM_RESET_PIN_SR,1);
    mdelay(5);
    */
        panel_reset();

    res = EnterISPMode(private_ts->client, isp_cmd);   //enter ISP mode
  } else
        printk("[ELAN] Firmware upgrade recovery mode !\n");
  //res = i2c_master_recv(private_ts->client, recovery_buffer, 4);   //55 aa 33 cc
  //printk("[ELAN] recovery byte data:%x,%x,%x,%x \n",recovery_buffer[0],recovery_buffer[1],recovery_buffer[2],recovery_buffer[3]);

  // Send Dummy Byte
  printk("[ELAN] send one byte data:%x,%x",private_ts->client->addr,data);
  res = i2c_master_send(private_ts->client, &data,  sizeof(data));
  if(res!=sizeof(data))
  {
    printk("[ELAN] dummy error code = %d\n",res);
  }
  mdelay(10);


  // Start IAP
  for( iPage = 1; iPage <= PageNum; iPage++ )
  {
PAGE_REWRITE:
#if 1 // 8byte mode
    // 8 bytes
    //szBuff = fw_data + ((iPage-1) * PageSize);
    for(byte_count=1;byte_count<=17;byte_count++)
    {
      if(byte_count!=17)
      {
        szBuff = file_fw_data + curIndex;
        curIndex =  curIndex + 8;
        res = WritePage(szBuff, 8);
      }
      else
      {
  //      printk("byte %d\n",byte_count);
  //      printk("curIndex =%d\n",curIndex);
        szBuff = file_fw_data + curIndex;
        curIndex =  curIndex + 4;
        res = WritePage(szBuff, 4);
      }
    } // end of for(byte_count=1;byte_count<=17;byte_count++)
#endif
#if 0 // 132byte mode
    szBuff = file_fw_data + curIndex;
    curIndex =  curIndex + PageSize;
    res = WritePage(szBuff, PageSize);
#endif
//#if 0
    if(iPage==377 || iPage==1)
    {
      mdelay(600);
    }
    else
    {
      mdelay(50);
    }
    res = GetAckData(private_ts->client);

    if (ACK_OK != res)
    {
      mdelay(50);
      printk("[ELAN] ERROR: GetAckData fail! res=%d\r\n", res);
      if ( res == ACK_REWRITE )
      {
        rewriteCnt = rewriteCnt + 1;
        if (rewriteCnt == PAGERETRY)
        {
          printk("[ELAN] ID 0x%02x %dth page ReWrite %d times fails!\n", data, iPage, PAGERETRY);
          return E_FD;
        }
        else
        {
          printk("[ELAN] ---%d--- page ReWrite %d times!\n",  iPage, rewriteCnt);
          goto PAGE_REWRITE;
        }
      }
      else
      {
        restartCnt = restartCnt + 1;
        if (restartCnt >= 5)
        {
          printk("[ELAN] ID 0x%02x ReStart %d times fails!\n", data, IAPRESTART);
          return E_FD;
        }
        else
        {
          printk("[ELAN] ===%d=== page ReStart %d times!\n",  iPage, restartCnt);
          goto IAP_RESTART;
        }
      }
    }
    else
    {       printk("  data : 0x%02x ",  data);
      rewriteCnt=0;
      print_progress(iPage,ic_num,i);
    }

    mdelay(10);
  } // end of for(iPage = 1; iPage <= PageNum; iPage++)

  printk("[ELAN] read Hello packet data!\n");
  res= __hello_packet_handler(client);
  if (res > 0)
    printk("[ELAN] Update ALL Firmware successfully!\n");
  return res;
}

#endif
// End Firmware Update

// Star 2wireIAP which used I2C to simulate JTAG function
#ifdef ELAN_2WIREICE
// 2WireICE removed
#endif

char FW_PATH[100]="/data/local/tmp/fw.ekt";
// Start sysfs
static ssize_t elan_ktf2k_gpio_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  int ret = 0;
  struct elan_ktf2k_ts_data *ts = private_ts;

  ret = gpio_get_value(ts->intr_gpio);
  touch_debug(DEBUG_INFO, "[ELAN]GPIO_TP_INT_N=%d\n", ret);
  sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
  ret = strlen(buf) + 1;
  return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);

static ssize_t elan_ktf2k_vendor_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
  ssize_t ret = 0;
  struct elan_ktf2k_ts_data *ts = private_ts;
  touch_debug(DEBUG_INFO, "[ELAN]FW Ver=%x\n", ts->fw_ver);
  sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF2K", ts->fw_ver);
  ret = strlen(buf) + 1;
  return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf2k_vendor_show, NULL);

//0512 start
static ssize_t elan_ktf2k_debug_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{ 
    touch_debug(DEBUG_INFO,"[ELAN]Debug_Option Show %d\n", Debug_Option);
    return scnprintf(buf, PAGE_SIZE, "%d\n", Debug_Option);
}
static ssize_t elan_ktf2k_debug_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
  sscanf(buf,"%d",&Debug_Option);
  printk("[ELAN]Debug_Option Set %d\n", Debug_Option);
  return count;
}

static DEVICE_ATTR(debug, S_IRUGO |S_IWUSR, elan_ktf2k_debug_show, elan_ktf2k_debug_store);
//---------------------------------------------------------
static ssize_t elan_ktf2k_fwpath_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{ 
    printk("[ELAN]FW PATH Show %s\n", FW_PATH);
    //call FW Update Function
    //Update_FW_One();

  return scnprintf(buf, PAGE_SIZE, "%s\n", FW_PATH); 
}

static ssize_t elan_ktf2k_fwpath_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
     //char FW_PATH[100];
     mm_segment_t oldfs;

     oldfs=get_fs();
     set_fs(KERNEL_DS);
     memset(FW_PATH, 0, 100);
     sscanf(buf, "%s\n", FW_PATH);
     printk("Update touch firmware with the file path:%s\n", FW_PATH);

   sscanf(buf,"%s",FW_PATH);
   return count;
}

static DEVICE_ATTR(fwpath, S_IRUGO |S_IWUSR, elan_ktf2k_fwpath_show, elan_ktf2k_fwpath_store);

//0512 end

static struct kobject *android_touch_kobj;

static int elan_ktf2k_touch_sysfs_init(void)
{
  int ret ;

  android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
  if (android_touch_kobj == NULL) {
    printk(KERN_ERR "[elan]%s: subsystem_register failed\n", __func__);
    ret = -ENOMEM;
    return ret;
  }
  ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
  if (ret) {
    printk(KERN_ERR "[elan]%s: sysfs_create_file failed\n", __func__);
    return ret;
  }
  ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
  if (ret) {
    printk(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
    return ret;
  }
  //0512
  ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug.attr);
  if (ret) {
    printk(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
    return ret;
  }
  
  ret = sysfs_create_file(android_touch_kobj, &dev_attr_fwpath.attr);
  if (ret) {
    printk(KERN_ERR "[elan]%s: sysfs_create_group failed\n", __func__);
    return ret;
  }
  //0512
  return 0 ;
}

static void elan_touch_sysfs_deinit(void)
{
  sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
  sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
  sysfs_remove_file(android_touch_kobj, &dev_attr_debug.attr);  //0512
  sysfs_remove_file(android_touch_kobj, &dev_attr_fwpath.attr); //0512
  kobject_del(android_touch_kobj);
}

// end sysfs

static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
  struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
  int status = 0, retry = 10;

  do {
    status = gpio_get_value(ts->intr_gpio);
    //printk("%s: status = %d\n", __func__, status);
    touch_debug(DEBUG_TRACE,"%s: status = %d\n", __func__, status);
    retry--;
    mdelay(50);
  } while (status == 1 && retry > 0);

  touch_debug(DEBUG_TRACE, "[elan]%s: poll interrupt status %s\n",
      __func__, status == 1 ? "high" : "low");
  return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
  return __elan_ktf2k_ts_poll(client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
      uint8_t *buf, size_t size)
{
  int rc;

  dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);

  if (buf == NULL)
    return -EINVAL;

  if ((i2c_master_send(client, cmd, 4)) != 4) {
    dev_err(&client->dev,
      "[elan]%s: i2c_master_send failed\n", __func__);
    return -EINVAL;
  }

  rc = elan_ktf2k_ts_poll(client);
  if (rc < 0)
    return -EINVAL;
  else {
    if (i2c_master_recv(client, buf, size) != size ||
        buf[0] != CMD_S_PKT)
      return -EINVAL;
  }
#ifdef RE_CALIBRATION
  mdelay(200);
  rc = i2c_master_recv(client, buf_recv, 8);
  printk("[elan] %s: Re-Calibration Packet %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
  if (buf_recv[0] != 0x66) {
    mdelay(200);
    rc = i2c_master_recv(client, buf_recv, 8);
    printk("[elan] %s: Re-Calibration Packet, re-try again %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
  }
#endif

  return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
  int rc;
  uint8_t buf_recv[8] = { 0 };

  rc = elan_ktf2k_ts_poll(client);
  if (rc < 0) {
    printk( "[elan] %s: Int poll failed!\n", __func__);
    RECOVERY=0x80;
    return RECOVERY;
    //return -EINVAL;
  }
  mdelay(100);
  rc = i2c_master_recv(client, buf_recv, 8);
  printk("[elan] %s: hello packet %2x:%2X:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3] , buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

  if(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x80 && buf_recv[3]==0x80)
  {
             RECOVERY=0x80;
       return RECOVERY;
  }
  mdelay(200);
  rc = i2c_master_recv(client, buf_recv, 8);
  printk("[elan] %s: Re-Calibration packet %2x:%2X:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

  return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
  struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
  int rc;
  int major, minor;
  uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};/* Get Firmware Version*/
  uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
  uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
  uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
    uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01};/* Get BootCode Version*/
  uint8_t buf_recv[4] = {0};
// Firmware version
  rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
  if (rc < 0)
    return rc;
  major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
  minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
  ts->fw_ver = major << 8 | minor;
  FW_VERSION = ts->fw_ver;
// Firmware ID
  rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
  if (rc < 0)
    return rc;
  major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
  minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
  ts->fw_id = major << 8 | minor;
  FW_ID = ts->fw_id;
// Bootcode version
        rc = elan_ktf2k_ts_get_data(client, cmd_bc, buf_recv, 4);
        if (rc < 0)
                return rc;
        major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
        minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
        ts->bc_ver = major << 8 | minor;

// X Resolution
  rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
  if (rc < 0)
    return rc;
  minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
  //SZSZ
  // ts->x_resolution =minor;
  ts->x_resolution =minor;
#ifndef ELAN_TEN_FINGERS
  X_RESOLUTION = ts->x_resolution;
#endif

// Y Resolution
  rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
  if (rc < 0)
    return rc;
  minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
  ts->y_resolution =minor;
#ifndef ELAN_TEN_FINGERS
  Y_RESOLUTION = ts->y_resolution;
#endif

  printk(KERN_INFO "[elan] %s: Firmware version: 0x%4.4x\n",
      __func__, ts->fw_ver);
  printk(KERN_INFO "[elan] %s: Firmware ID: 0x%4.4x\n",
      __func__, ts->fw_id);
  printk(KERN_INFO "[elan] %s: Bootcode Version: 0x%4.4x\n",
      __func__, ts->bc_ver);
  printk(KERN_INFO "[elan] %s: x resolution: %d, y resolution: %d\n",
      __func__, X_RESOLUTION, Y_RESOLUTION);

  return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data,
      uint16_t *x, uint16_t *y)
{
  *x = *y = 0;

  *x = (data[0] & 0xf0);
  *x <<= 4;
  *x |= data[1];
  //SZSZ
  *x = *x * x_scale;

  *y = (data[0] & 0x0f);
  *y <<= 8;
  *y |= data[2];
  //SZSZ
  *y = *y * y_scale;

  return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
  int rc;

  rc = __hello_packet_handler(client);
  printk("[elan] hellopacket's rc = %d\n",rc);

  mdelay(10);
  if (rc != 0x80){
      rc = __fw_packet_handler(client);
      if (rc < 0)
        printk("[elan] %s, fw_packet_handler fail, rc = %d", __func__, rc);
      dev_dbg(&client->dev, "[elan] %s: firmware checking done.\n", __func__);
//Check for FW_VERSION, if 0x0000 means FW update fail!
      if ( FW_VERSION == 0x00)
      {
    rc = 0x80;
    printk("[elan] FW_VERSION = %d, last FW update fail\n", FW_VERSION);
      }
      }
  return rc;
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client){
      uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

  //dev_info(&client->dev, "[elan] %s: enter\n", __func__);
  printk("[elan] %s: enter\n", __func__);
  dev_info(&client->dev,
    "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
    cmd[0], cmd[1], cmd[2], cmd[3]);

  if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
    dev_err(&client->dev,
      "[elan] %s: i2c_master_send failed\n", __func__);
    return -EINVAL;
  }

  return 0;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
  uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

  dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

  cmd[1] |= (state << 3);

  dev_dbg(&client->dev,
    "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
    cmd[0], cmd[1], cmd[2], cmd[3]);

  if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
    dev_err(&client->dev,
      "[elan] %s: i2c_master_send failed\n", __func__);
    return -EINVAL;
  }

  return 0;
}

static int elan_ktf2k_ts_get_power_state(struct i2c_client *client)
{
  int rc = 0;
  uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
  uint8_t buf[4], power_state;

  rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
  if (rc)
    return rc;

  power_state = buf[1];
  dev_dbg(&client->dev, "[elan] dump repsponse: %0x\n", power_state);
  power_state = (power_state & PWR_STATE_MASK) >> 3;
  dev_dbg(&client->dev, "[elan] power state = %s\n",
    power_state == PWR_STATE_DEEP_SLEEP ?
    "Deep Sleep" : "Normal/Idle");

  return power_state;
}

#ifdef ELAN_POWER_SOURCE
static unsigned now_usb_cable_status=0;

#if 0
static int elan_ktf2k_ts_hw_reset(struct i2c_client *client)
{
    struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
    touch_debug(DEBUG_INFO, "[ELAN] Start HW reset!\n");
    gpio_direction_output(ts->rst_gpio, 0);
    usleep_range(1000,1500);
    gpio_direction_output(ts->rst_gpio, 1);
    msleep(5);
    return 0;
}
static int elan_ktf2k_ts_set_power_source(struct i2c_client *client, u8 state)
{
        uint8_t cmd[] = {CMD_W_PKT, 0x40, 0x00, 0x01};
        int length = 0;

        dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);
    /*0x52 0x40 0x00 0x01  =>    Battery Mode
       0x52 0x41 0x00 0x01  =>   AC Adapter Mode
       0x52 0x42 0x00 0x01 =>    USB Mode */
        cmd[1] |= state & 0x0F;

        dev_dbg(&client->dev,
                "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
                cmd[0], cmd[1], cmd[2], cmd[3]);

      down(&pSem);
      length = i2c_master_send(client, cmd, sizeof(cmd));
      up(&pSem);
        if (length != sizeof(cmd)) {
                dev_err(&client->dev,
                        "[elan] %s: i2c_master_send failed\n", __func__);
                return -EINVAL;
        }

        return 0;
}



static void update_power_source(){
      unsigned power_source = now_usb_cable_status;
        if(private_ts == NULL || work_lock) return;

        if(private_ts->abs_x_max == ELAN_X_MAX) //TF 700T device
            return; // do nothing for TF700T;

      touch_debug(DEBUG_INFO, "Update power source to %d\n", power_source);
      switch(power_source){
        case USB_NO_Cable:
            elan_ktf2k_ts_set_power_source(private_ts->client, 0);
            break;
        case USB_Cable:
          elan_ktf2k_ts_set_power_source(private_ts->client, 1);
            break;
        case USB_AC_Adapter:
          elan_ktf2k_ts_set_power_source(private_ts->client, 2);
      }
}
#endif

void touch_callback(unsigned cable_status){
      now_usb_cable_status = cable_status;
      //update_power_source();
}
#endif

static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf, int bytes_to_recv)
{

  int rc;
  if (buf == NULL)
    return -EINVAL;

  memset(buf, 0, bytes_to_recv);

/* The ELAN_PROTOCOL support normanl packet format */
#ifdef ELAN_PROTOCOL
  rc = i2c_master_recv(client, buf, bytes_to_recv);
  //printk("[elan] Elan protocol rc = %d \n", rc);
  touch_debug(DEBUG_INFO, "[elan] Elan protocol rc = %d \n", rc);
  if (rc != bytes_to_recv) {
    dev_err(&client->dev, "[elan] %s: i2c_master_recv error?! \n", __func__);
    return -1;
  }

#else
  rc = i2c_master_recv(client, buf, 8);
  if (rc != 8)
  {
     printk("[elan] Read the first package error.\n");
     mdelay(30);
     return -1;
    }
    printk("[elan_debug] %x %x %x %x %x %x %x %x \n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  mdelay(1);

  if (buf[0] == 0x5D){    //for five finger //0509
    rc = i2c_master_recv(client, buf+ 8, 8);
    if (rc != 8)
    {
          printk("[elan] Read the second package error.\n");
          mdelay(30);
          return -1;
    }
    printk("[elan_debug] %x %x %x %x %x %x %x %x\n", buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
    rc = i2c_master_recv(client, buf+ 16, 2);
    if (rc != 2)
    {
          printk("[elan] Read the third package error.\n");
          mdelay(30);
          return -1;
    }
    mdelay(1);
    printk("[elan_debug] %x %x \n", buf[16], buf[17]);
  }
#endif
//printk("[elan_debug] end ts_work\n");
  return rc;
}

int previous_x[5]={0,0,0,0,0}; //0605 
int previous_y[5]={0,0,0,0,0}; //0605

static int mTouchStatus[FINGER_NUM] = {0};

void force_release_pos(struct i2c_client *client)
{
        struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
        int i;
        for (i=0; i < FINGER_NUM; i++) {
                if (mTouchStatus[i] == 0) continue;
                input_mt_slot(ts->input_dev, i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
                mTouchStatus[i] = 0;
        }

        input_sync(ts->input_dev);
}

static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
  struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
  struct input_dev *idev = ts->input_dev;
  uint16_t x, y;
  uint16_t fbits=0;
  uint8_t i, num, reported = 0;
  uint16_t active = 0; 
  uint8_t idx, btn_idx;
  int finger_num;
  int j;
  //int checksum = 0;
  //int pressure_size=80;

/* for 10 fingers */
  if (buf[0] == TEN_FINGERS_PKT){
        finger_num = 10;
        num = buf[2] & 0x0f;
        fbits = buf[2] & 0x30;
        fbits = (fbits << 4) | buf[1];
        idx=3;
        btn_idx=33;
      }
/* for 5 fingers  */
      else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
        finger_num = 5;
        num = buf[1] & 0x07;
        fbits = buf[1] >>3;
        idx=2;
        btn_idx=17;
      }else{
/* for 2 fingers */
        finger_num = 2;
        num = buf[7] & 0x03;    // for elan old 5A protocol the finger ID is 0x06
        fbits = buf[7] & 0x03;
//        fbits = (buf[7] & 0x03) >> 1; // for elan old 5A protocol the finger ID is 0x06
        idx=1;
        btn_idx=7;
      }

    switch (buf[0]) {
    case MTK_FINGERS_PKT:
    case TWO_FINGERS_PKT:
    case FIVE_FINGERS_PKT:
    case TEN_FINGERS_PKT:
        for(i = 0; i < FINGER_NUM; i++){
            active = fbits & 0x1;
            if(active || mTouchStatus[i]){
                input_mt_slot(ts->input_dev, i);
                input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, active);
                if ((active ==0) && (mTouchStatus[i] == 1)) {/*Finger UP*/
                    // printk("[ELAN] Finger Up1\n");
                    input_report_key(idev, BTN_TOUCH, 0); //for all finger up
                    input_report_abs(idev, ABS_PRESSURE, 0);
                    input_report_abs(idev, ABS_MT_PRESSURE, 0);
                    input_event(idev, EV_ABS, ABS_MT_TRACKING_ID, -1);
                }
                if(active){
                    elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
                    x=600-x;
                    y=1024-y;
                    if (!((x<=0) || (y<=0) || (x>=MAX_X) || (y>=MAX_Y))) { //0603
                        input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100);
                        input_report_abs(idev, ABS_MT_PRESSURE, 40);
                        input_report_abs(idev, ABS_MT_POSITION_X, y);
                        input_report_abs(idev, ABS_MT_POSITION_Y, x);
                        if (i==0) {
                            input_report_key(idev, BTN_TOUCH, 1);
                          input_report_abs(idev, ABS_PRESSURE, 40);
                          input_report_abs(idev, ABS_X, y);
                          input_report_abs(idev, ABS_Y, x);
                      }
                      touch_debug(DEBUG_INFO, "[elan] finger id=%d X=%d y=%d \n", i, x, y);
                      //printk( "[elan] finger id=%d X=%d y=%d \n", i, x, y);
                  }
                }
            }
            mTouchStatus[i] = active;
            fbits = fbits >> 1;
            idx += 3;
        }
        if (num == 0){
            // printk("[ELAN] ALL Finger Up0\n");
            input_report_key(idev, BTN_TOUCH, 0); //for all finger up
            input_report_abs(idev, ABS_PRESSURE, 0);
            force_release_pos(client);
        }
        //SZ not sure the purpose of the emulation but it reported btn 1 after btn 0 has reported
        //	causing Linearity Test line link and first UI touch not working properly
        // input_mt_report_pointer_emulation(idev, true);
        input_sync(idev);
        break;
    case 78://0512
        touch_debug(DEBUG_TRACE,"%x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] );
        break;
    default:
        dev_err(&client->dev,
            "[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
        break;
    } // end switch

    return;
}

#if 0
static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
  struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
  struct input_dev *idev = ts->input_dev;
  uint16_t x, y;
  uint16_t fbits=0;
  uint8_t i, num, reported = 0;
  uint8_t idx, btn_idx;
  int finger_num;
  int j;
  //int checksum = 0;
  //int pressure_size=80;

/* for 10 fingers */
  if (buf[0] == TEN_FINGERS_PKT){
        finger_num = 10;
        num = buf[2] & 0x0f;
        fbits = buf[2] & 0x30;
        fbits = (fbits << 4) | buf[1];
        idx=3;
        btn_idx=33;
      }
/* for 5 fingers  */
      else if ((buf[0] == MTK_FINGERS_PKT) || (buf[0] == FIVE_FINGERS_PKT)){
        finger_num = 5;
        num = buf[1] & 0x07;
        fbits = buf[1] >>3;
        idx=2;
        btn_idx=17;
      }else{
/* for 2 fingers */
        finger_num = 2;
        num = buf[7] & 0x03;    // for elan old 5A protocol the finger ID is 0x06
        fbits = buf[7] & 0x03;
//        fbits = (buf[7] & 0x03) >> 1; // for elan old 5A protocol the finger ID is 0x06
        idx=1;
        btn_idx=7;
      }

    switch (buf[0]) {
      case MTK_FINGERS_PKT:
      case TWO_FINGERS_PKT:
      case FIVE_FINGERS_PKT:
      case TEN_FINGERS_PKT:

          if (num == 0) {
            touch_debug(DEBUG_TRACE,"[Version20]num=0\n");  //0530-14
            input_report_abs(idev, ABS_PRESSURE, 0); //0523-12
            input_report_key(idev, BTN_TOUCH, num>0); //0530-14
            input_mt_report_slot_state(idev, MT_TOOL_FINGER, num>0);
            //input_report_abs(idev, ABS_MT_TRACKING_ID, -1);

            for (i=0 ; i<5 ; i++){  //0605-18
                /*
                if (previous_x[i]!=0 &&previous_y[i]!=0){  //0605-18
                       input_mt_slot(idev, i);               //0605-18
                   input_mt_report_slot_state(idev, MT_TOOL_FINGER, false); //0605-18
                   input_report_abs(idev, ABS_MT_TRACKING_ID, -1);          //0605-18
                   input_report_key(idev, ABS_MT_PRESSURE, 0); //0605-18
                }
                */
              previous_x[i]=0;
              previous_y[i]=0;
            }

            //input_mt_slot(idev, 0); //0605-17
            //input_mt_report_slot_state(idev, MT_TOOL_FINGER, false); //0605-17
            //input_report_key(idev, ABS_MT_PRESSURE, 0);  //0605-17
            
          } else {
            dev_dbg(&client->dev, "[elan] %d fingers\n", num);
            touch_debug(DEBUG_TRACE,"num=%d\n", num); //0530-14
            input_report_key(idev, BTN_TOUCH, 1); //0604
            for (i = 0; i < finger_num; i++) {
              if ((fbits & 0x01)) {
                elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
                //elan_ktf2k_ts_parse_xy(&buf[idx], &y, &x);
                //printk("[elan_debug] %s, x=%d, y=%d\n",__func__, x , y);
                touch_debug(DEBUG_INFO,"[elan_debug] %s, x=%d, y=%d\n",__func__, x , y);
                //x = X_RESOLUTION-x;
                //y = Y_RESOLUTION-y;
                x=600-x;
                y=1024-y;


                if (!((x<=0) || (y<=0) || (x>=MAX_X) || (y>=MAX_Y))) { //0603
                  input_mt_slot(idev, i); //0604-16
                  input_mt_report_slot_state(idev, MT_TOOL_FINGER, true); //0604-16
                  input_report_abs(idev, ABS_MT_TRACKING_ID, i); //0528-13
                  input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 100); //0603-15
                  input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 100); //0603-15
                  input_report_abs(idev, ABS_MT_PRESSURE, 80);
                  input_report_abs(idev, ABS_MT_POSITION_X, y);   //0507
                  input_report_abs(idev, ABS_MT_POSITION_Y, x);   //0507
              
                  touch_debug(DEBUG_INFO,"                     x=%d, y=%d\n", y , x);

                  //SZSZ
                  if (i==0) {
                    input_report_abs(idev, ABS_PRESSURE, 80);
                    input_report_abs(idev, ABS_X, y);
                    input_report_abs(idev, ABS_Y, x);
                    //printk("                     x=%d, y=%d\n", y , x);
                    //touch_debug(DEBUG_INFO,"                     x=%d, y=%d\n", y , x);
                  }

                  // reported++;
                }
                //reported++; //0603-15
              } // end if finger status (fbits & 0x01)
              else {
                  touch_debug(DEBUG_TRACE,"3.finger %d up ....\n",i);
                  input_mt_slot(idev, i); //0604
                  input_mt_report_slot_state(idev, MT_TOOL_FINGER, false); //0604
                  /*0605
                  if(previous_x[i]!=0 &&previous_y[i]!=0){
                     touch_debug(DEBUG_TRACE,"3.finger %d up ....\n",i);
                     //touch_debug(DEBUG_INFO,"previous_x[%d]!=0 &&previous_y[%d]!=0\n", i , i);
                     input_mt_slot(idev, i); //0604
                     input_report_key(idev, ABS_MT_PRESSURE, 0); //0530
                     input_report_abs(idev, ABS_MT_TRACKING_ID, -1); //0528-13
                     input_sync(idev);
                     input_mt_slot(idev, i); //0604
                     input_mt_report_slot_state(idev, MT_TOOL_FINGER, false); //0604

                  }/*else{

                     touch_debug(DEBUG_INFO,"previous_x[%d]==0 || previous_y[%d]==0\n", i , i);
                     input_mt_slot(idev, i); //0604
                     input_report_key(idev, ABS_MT_PRESSURE, 0); //0530
                     input_report_abs(idev, ABS_MT_TRACKING_ID, -1); //0528-13
                     input_sync(idev);
                     input_mt_slot(idev, i); //0604
                     input_mt_report_slot_state(idev, MT_TOOL_FINGER, false); //0604

                  }*/
              }
              previous_x[i]=y; //0610
              previous_y[i]=x; //0610
              fbits = fbits >> 1;
              idx += 3;
            } // end for
          }


            input_mt_report_pointer_emulation(idev, true); /*0609*/
            input_sync(idev);

          break;
      case 78://0512
         touch_debug(DEBUG_TRACE,"%x %x %x %x\n",buf[0],buf[1],buf[2],buf[3] );
         break;
      default:
        dev_err(&client->dev,
                "[elan] %s: unknown packet type: %0x\n", __func__, buf[0]);
        break;
    } // end switch

  return;
}
#endif

static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
  int rc;
  struct elan_ktf2k_ts_data *ts =
  container_of(work, struct elan_ktf2k_ts_data, work);
  //uint8_t buf[4+PACKET_SIZE] = { 0 };
  uint8_t buf[PACKET_SIZE] = { 0 }; //0509-2
  // uint8_t buf1[PACKET_SIZE] = { 0 };

    //SZSZ
    if (gpio_get_value(ts->intr_gpio))
    {
        //SZSZ report error and continue for debugging purpose
        printk("[elan] Detected the jitter on INT pin\n");
        if (0) {
            enable_irq(ts->client->irq);
            return;
        }
    }

    //rc = elan_ktf2k_ts_recv_data(ts->client, buf,4+PACKET_SIZE);
    rc = elan_ktf2k_ts_recv_data(ts->client, buf,PACKET_SIZE); //0509-2

    if (rc < 0)
    {
      printk("[elan] Received the packet Error.\n");
      enable_irq(ts->client->irq);
      return;
    }
    //0509-2
    //printk("[elan_debug] %2x,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x,,%2x,,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14],buf[15],buf[16],buf[17],buf[18],buf[19],buf[20],buf[21],buf[22],buf[23],buf[24]);
    touch_debug(DEBUG_INFO,"[elan_debug] %2x,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x,,%2x,,%2x,,%2x,%2x,%2x,,%2x,%2x,%2x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14],buf[15],buf[16],buf[17],buf[18],buf[19],buf[20],buf[21],buf[22],buf[23],buf[24]);

#ifndef ELAN_BUFFER_MODE
    elan_ktf2k_ts_report_data(ts->client, buf);
#else
    elan_ktf2k_ts_report_data(ts->client, buf+4);

  // Second package
  if (((buf[0] == 0x63) || (buf[0] == 0x66)) && ((buf[1] == 2) || (buf[1] == 3))) {
    rc = elan_ktf2k_ts_recv_data(ts->client, buf1, PACKET_SIZE);
    if (rc < 0){
      enable_irq(ts->client->irq);
                                return;
    }
    elan_ktf2k_ts_report_data(ts->client, buf1);
  // Final package
    if (buf[1] == 3) {
      rc = elan_ktf2k_ts_recv_data(ts->client, buf1, PACKET_SIZE);
      if (rc < 0){
        enable_irq(ts->client->irq);
        return;
      }
      elan_ktf2k_ts_report_data(ts->client, buf1);
    }
  }
#endif

    enable_irq(ts->client->irq);

  return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
  struct elan_ktf2k_ts_data *ts = dev_id;
  struct i2c_client *client = ts->client;

  dev_dbg(&client->dev, "[elan] %s\n", __func__);
  disable_irq_nosync(ts->client->irq);
  queue_work(ts->elan_wq, &ts->work);

  return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
  struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
  int err = 0;

    err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,
                       IRQF_TRIGGER_LOW, client->name, ts);
    //err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,
    //                 IRQF_TRIGGER_FALLING, client->name, ts);
  if (err)
    dev_err(&client->dev, "[elan] %s: request_irq %d failed\n",
        __func__, client->irq);

  return err;
}

static int elan_ktf2k_ts_probe(struct i2c_client *client,
      const struct i2c_device_id *id)
{
  int err = 0;
  int fw_err = 0;
  struct elan_ktf2k_i2c_platform_data *pdata;
  struct elan_ktf2k_ts_data *ts;
  int New_FW_ID;
  int New_FW_VER;

  printk(KERN_ERR "[elan] %s: entering probe\n", __func__);
  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
    printk(KERN_ERR "[elan] %s: i2c check functionality error\n", __func__);
    err = -ENODEV;
    goto err_check_functionality_failed;
  }

  ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
  if (ts == NULL) {
    printk(KERN_ERR "[elan] %s: allocate elan_ktf2k_ts_data failed\n", __func__);
    err = -ENOMEM;
    goto err_alloc_data_failed;
  }

  ts->elan_wq = create_singlethread_workqueue("elan_wq");
  if (!ts->elan_wq) {
    printk(KERN_ERR "[elan] %s: create workqueue failed\n", __func__);
    err = -ENOMEM;
    goto err_create_wq_failed;
  }

  INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);
  ts->client = client;
  i2c_set_clientdata(client, ts);
// james: maybe remove
  pdata = client->dev.platform_data;
  if (likely(pdata != NULL)) {
    ts->intr_gpio = pdata->intr_gpio;
  }

  printk(KERN_ERR "[elan] %s: ts allocated, performing setup (after reset)... \n", __func__);

  //SZSZ
  panel_reset();

  fw_err = elan_ktf2k_ts_setup(client);
  if (fw_err < 0) {
    printk(KERN_INFO "No Elan chip inside\n");
//    fw_err = -ENODEV;
  }


  printk(KERN_ERR "[elan] %s: allocating device... \n", __func__);

  ts->input_dev = input_allocate_device();
  if (ts->input_dev == NULL) {
    err = -ENOMEM;
    dev_err(&client->dev, "[elan] Failed to allocate input device\n");
    goto err_input_dev_alloc_failed;
  }
    //SZSZSZ
  ts->input_dev->name = "touchscreen interface"; //"elan-touchscreen";

  printk(KERN_ERR "[elan] %s: setting bits... \n", __func__);


  
#ifdef ELAN_BUTTON
  set_bit(KEY_BACK, ts->input_dev->keybit);
  set_bit(KEY_MENU, ts->input_dev->keybit);
  set_bit(KEY_HOME, ts->input_dev->keybit);
  set_bit(KEY_SEARCH, ts->input_dev->keybit);
#endif
  //SZSZ 
  /*single touch*/
  input_set_abs_params(ts->input_dev, ABS_X, 0,  MAX_Y, 0, 0); //0523-12
  input_set_abs_params(ts->input_dev, ABS_Y, 0,  MAX_X, 0, 0); //0523-12
  input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 100, 0, 0); //0523-12
  
  /*muti-touch*/
  input_mt_init_slots(ts->input_dev, FINGER_NUM); //0522
  input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
  input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_Y, 0, 0); //0523-12
  input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_X, 0, 0); //0523-12
  input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);
  input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0); //0522-11
  input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 0, 255, 0); //0523-12
  input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0); //0523-12
  
//SZSZ
  /*
  __set_bit(EV_ABS, ts->input_dev->evbit);
  __set_bit(EV_SYN, ts->input_dev->evbit);
  __set_bit(EV_KEY, ts->input_dev->evbit);
*/
  set_bit(EV_ABS, ts->input_dev->evbit);
  set_bit(EV_SYN, ts->input_dev->evbit);
  set_bit(EV_KEY, ts->input_dev->evbit);
  set_bit(BTN_TOUCH, ts->input_dev->keybit);

  err = input_register_device(ts->input_dev);
  if (err) {
    dev_err(&client->dev,
      "[elan]%s: unable to register %s input device\n",
      __func__, ts->input_dev->name);
    goto err_input_register_device_failed;
  }

  elan_ktf2k_ts_register_interrupt(ts->client);

  if (gpio_get_value(ts->intr_gpio) == 0) {
    printk(KERN_INFO "[elan]%s: handle missed interrupt\n", __func__);
    elan_ktf2k_ts_irq_handler(client->irq, ts);
  }

  private_ts = ts;

  elan_ktf2k_touch_sysfs_init();

  dev_info(&client->dev, "[elan] Start touchscreen %s in interrupt mode\n",
    ts->input_dev->name);

// Firmware Update
  ts->firmware.minor = MISC_DYNAMIC_MINOR;
  ts->firmware.name = "elan-iap";
  ts->firmware.fops = &elan_touch_fops;
//SZSZ  ts->firmware.mode = S_IFREG|S_IRWXUGO;
  ts->firmware.mode = S_IFCHR|S_IRWXUGO;

  if (misc_register(&ts->firmware) < 0)
      printk("[ELAN]misc_register failed!!");
    else
    printk("[ELAN]misc_register finished!!");
// End Firmware Update
#ifdef IAP_PORTION
  if(0)
  {
    printk("[ELAN]misc_register finished!!");
    work_lock=1;
    disable_irq(ts->client->irq);
    cancel_work_sync(&ts->work);

    power_lock = 1;
/* FW ID & FW VER*/
#if 0  /* For ektf21xx and ektf20xx  */
    printk("[ELAN]  [7bd0]=0x%02x,  [7bd1]=0x%02x, [7bd2]=0x%02x, [7bd3]=0x%02x\n",  file_fw_data[31696],file_fw_data[31697],file_fw_data[31698],file_fw_data[31699]);
    New_FW_ID = file_fw_data[31699]<<8  | file_fw_data[31698] ;
    New_FW_VER = file_fw_data[31697]<<8  | file_fw_data[31696] ;
#endif

#if 0   /* for ektf31xx 2 wire ice ex: 2wireice -b xx.bin */
    printk(" [7c16]=0x%02x,  [7c17]=0x%02x, [7c18]=0x%02x, [7c19]=0x%02x\n",  file_fw_data[31766],file_fw_data[31767],file_fw_data[31768],file_fw_data[31769]);
    New_FW_ID = file_fw_data[31769]<<8  | file_fw_data[31768] ;
    New_FW_VER = file_fw_data[31767]<<8  | file_fw_data[31766] ;
#endif
    /* for ektf31xx iap ekt file   */
    printk(" [7bd8]=0x%02x,  [7bd9]=0x%02x, [7bda]=0x%02x, [7bdb]=0x%02x\n",  file_fw_data[31704],file_fw_data[31705],file_fw_data[31706],file_fw_data[31707]);
    New_FW_ID = file_fw_data[31707]<<8  | file_fw_data[31708] ;
    New_FW_VER = file_fw_data[31705]<<8  | file_fw_data[31704] ;
    printk(" FW_ID=0x%x,   New_FW_ID=0x%x \n",  FW_ID, New_FW_ID);
    printk(" FW_VERSION=0x%x,   New_FW_VER=0x%x \n",  FW_VERSION  , New_FW_VER);

/* for firmware auto-upgrade
    if (New_FW_ID   ==  FW_ID){
        if (New_FW_VER > (FW_VERSION))
                    Update_FW_One(client, RECOVERY);
    } else {
                    printk("FW_ID is different!");
    }
*/
    if (FW_ID == 0)  RECOVERY=0x80;
      Update_FW_One(client, RECOVERY);
    power_lock = 0;

    work_lock=0;
    enable_irq(ts->client->irq);

  }
#endif
  return 0;

err_input_register_device_failed:
  if (ts->input_dev)
    input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
  if (ts->elan_wq)
    destroy_workqueue(ts->elan_wq);

err_create_wq_failed:
  kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:

  return err;
}

static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
  struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

  elan_touch_sysfs_deinit();

  //SZSZ unregister_early_suspend(&ts->early_suspend);
  free_irq(client->irq, ts);

  if (ts->elan_wq)
    destroy_workqueue(ts->elan_wq);
  input_unregister_device(ts->input_dev);
  kfree(ts);

  return 0;
}

static int elan_ktf2k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
  struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
  int rc = 0;
  if(power_lock==0) /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
  {
    printk(KERN_INFO "[elan] %s: enter\n", __func__);

    disable_irq(client->irq);

    rc = cancel_work_sync(&ts->work);
    if (rc)
      enable_irq(client->irq);

    rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_DEEP_SLEEP);
  }
  return 0;
}

static int elan_ktf2k_ts_resume(struct i2c_client *client)
{

  int rc = 0, retry = 3;
  // uint8_t buf_recv[4] = { 0 };
  if(power_lock==0)   /* The power_lock can be removed when firmware upgrade procedure will not be enter into suspend mode.  */
  {
    printk(KERN_INFO "[elan] %s: enter\n", __func__);

    do {
      rc = elan_ktf2k_ts_set_power_state(client, PWR_STATE_NORMAL);
      mdelay(200);
#ifdef RE_CALIBRATION
      rc = i2c_master_recv(client, buf_recv, 4);
      printk("[elan] %s: Re-Calibration Packet %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
      if (buf_recv[0] != 0x66) {
        mdelay(200);
        rc = i2c_master_recv(client, buf_recv, 4);
        printk("[elan] %s: Re-Calibration Packet, re-try again %2x:%2x:%2x:%2x\n", __func__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
      }
#endif
      rc = elan_ktf2k_ts_get_power_state(client);
      if (rc != PWR_STATE_NORMAL)
        printk(KERN_ERR "[elan] %s: wake up tp failed! err = %d\n",
          __func__, rc);
      else
        break;
    } while (--retry);

    enable_irq(client->irq);
  }
  return 0;
}

static const struct i2c_device_id elan_ktf2k_ts_id[] = {
  { ELAN_KTF2K_NAME7, 0 },
  { }
};

static struct i2c_driver ektf2k_ts_driver = {
  .probe    = elan_ktf2k_ts_probe,
  .remove   = elan_ktf2k_ts_remove,
  .suspend  = elan_ktf2k_ts_suspend,
  .resume   = elan_ktf2k_ts_resume,
  .id_table = elan_ktf2k_ts_id,
  .driver   = {
    .name = ELAN_KTF2K_NAME7,
  },
};

static int __devinit elan_ktf2k_ts_init(void)
{
  printk(KERN_INFO "[elan] %s driver version 0x0005: Integrated 2, 5, and 10 fingers together and auto-mapping resolution\n", __func__);
  return i2c_add_driver(&ektf2k_ts_driver);
}

static void __exit elan_ktf2k_ts_exit(void)
{
  i2c_del_driver(&ektf2k_ts_driver);
  return;
}

module_init(elan_ktf2k_ts_init);
module_exit(elan_ktf2k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");


