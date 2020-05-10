/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/gpio.h>

#include <mach/platform.h>
#include <mach/devices.h>
#include <mach/soc.h>

/*
#define pr_debug(msg...)
*/

#define NOSTOP_GPIO 		(0)
//#define NOSTOP_GPIO 		(1)
#define	I2C_CLOCK_RATE		(100000)	/* default I2C clock rate */
#define TRANS_RETRY_CNT		(1)
#define	WAIT_ACK_TIME		(500)		/* wait 50 msec */

const static int i2c_gpio [NUMBER_OF_I2C_MODULE][2] = {
	/* I2C SCL PIN       I2C SDA PIN */
	{ (PAD_GPIO_D + 2), (PAD_GPIO_D + 3) },
	{ (PAD_GPIO_D + 4), (PAD_GPIO_D + 5) },
	{ (PAD_GPIO_D + 6), (PAD_GPIO_D + 7) },
};
const static int i2c_reset[NUMBER_OF_I2C_MODULE] = {RESET_ID_I2C0, RESET_ID_I2C1, RESET_ID_I2C2};

/*
 * 	local data and macro
 */
struct nxp_i2c_hw {
	int port;
	int irqno;
	int scl_io;
	int sda_io;
	int i2c_prescaler;
	int i2c_divisor;
	/* Register */
	void __iomem *base_addr;
};

#define	I2C_TRANS_RUN		(1<<0)
#define	I2C_TRANS_DONE		(1<<1)
#define	I2C_TRANS_ERR		(1<<2)

struct nxp_i2c_param {
	struct nxp_i2c_hw hw;
	struct mutex  lock;
	wait_queue_head_t wait_q;
	unsigned int condition;
	unsigned long rate;
	int	stop;
	u8	pre_data;
	int	request_ack;
	int	timeout;
	/* i2c trans data */
	struct i2c_adapter adapter;
	struct i2c_msg *msg;
	struct clk *clk;
	int	trans_count;
	int	trans_mode;
	int polling;
	int running;
	unsigned int trans_status;
	/* test */
	int irq_count;
	int thd_count;
};

/* I2C Registers */
#define I2C_ICCR_OFFS		0x00
#define I2C_ICSR_OFFS		0x04
#define I2C_IDSR_OFFS		0x0C
#define I2C_STOP_OFFS		0x10

/* ICCR bit definitions */
#define ICCR_IRQ_CLR		( 1 << 8 )
#define ICCR_ACK_ENB		( 1 << 7 )
#define ICCR_CLK_SRC		( 1 << 6 )
#define ICCR_IRQ_ENB		( 1 << 5 )
#define ICCR_IRQ_PND		( 1 << 4 )
#define ICCR_CLK_MASK		( 0xF << 0 )	/* mask for clock bits */

/* ICSR bit definitions */
#define ICSR_TXRXMODE_SLAVE_RX	( 0 << 6 )	/* Slave Receive Mode   */
#define ICSR_TXRXMODE_SLAVE_TX	( 1 << 6 )	/* Slave Transmit Mode  */
#define ICSR_TXRXMODE_MASTER_RX	( 2 << 6 )	/* Master Receive Mode  */
#define ICSR_TXRXMODE_MASTER_TX	( 3 << 6 )	/* Master Transmit Mode */
#define ICSR_TXRXMODE_MASK	( 3 << 6 )	/* bit mask */
#define ICSR_SIG_GEN_WRITE	( 1 << 5 )	/* Busy Signal Generation on WRITE */
#define ICSR_BUS_BUSY_READ	( 1 << 5 )	/* Busy Signal Status on READ */
#define ICSR_OUT_ENB		( 1 << 4 )
#define ICSR_ARI_STA		( 1 << 3 )	/* Arbitration */
#define ICSR_ACK_REV		( 1 << 0 )	/* ACK */

/* STOP bit definitions */
#define STOP_ACK_GEN		( 1 << 2 )
#define STOP_DAT_REL		( 1 << 1 )	/* only slave transmode */
#define STOP_CLK_REL		( 1 << 0 )	/* only master transmode */


static inline void i2c_start_dev(struct nxp_i2c_param *par)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0, ICCR = 0;

	ICSR = ioread32(base+I2C_ICSR_OFFS);
	ICSR  =  ICSR_OUT_ENB;
	iowrite32(ICSR, (base+I2C_ICSR_OFFS));

	iowrite32(par->pre_data, (base+I2C_IDSR_OFFS));

	ICCR = ioread32(base+I2C_ICCR_OFFS);
	ICCR &= ~ICCR_ACK_ENB;
	ICCR |=  ICCR_IRQ_ENB;
	iowrite32(ICCR, (base+I2C_ICCR_OFFS));
	ICSR  =  par->trans_mode | ICSR_SIG_GEN_WRITE | ICSR_OUT_ENB;
	iowrite32(ICSR, (base+I2C_ICSR_OFFS));
}

static inline void i2c_trans_dev(void __iomem *base, unsigned int ack, int stop)
{
	unsigned int ICCR = 0, STOP = 0;

	ICCR = ioread32(base+I2C_ICCR_OFFS);
	if (ack)
		ICCR |= ICCR_ACK_ENB;
	else
		ICCR &= ~ICCR_ACK_ENB;
	iowrite32(ICCR, (base+I2C_ICCR_OFFS));

	/* Errata fix, Ensure master generates Not-acknowledge in receive mode */
	if (stop) {
		STOP  = ioread32(base+I2C_STOP_OFFS);
		STOP |= STOP_ACK_GEN;
		iowrite32(STOP, base+I2C_STOP_OFFS);
	}

	ICCR  = ioread32((base+I2C_ICCR_OFFS));
	ICCR &= ~ICCR_IRQ_PND;
	ICCR |= ICCR_IRQ_CLR;
	ICCR |= ICCR_IRQ_ENB;
	iowrite32(ICCR, (base+I2C_ICCR_OFFS));
}

static int i2c_stop_scl(struct nxp_i2c_param *par)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0, ICCR = 0;
	int gpio = par->hw.scl_io;
	unsigned long start;
	int timeout = 5, ret = 0;

	iowrite32(STOP_CLK_REL, base+I2C_STOP_OFFS);	/* Errata: Ensure SCL bus freed */
	ICSR= ioread32(base+I2C_ICSR_OFFS);
	ICSR &= ~ICSR_OUT_ENB;
	ICSR |= par->trans_mode;
	iowrite32(ICSR, (base+I2C_ICSR_OFFS));
	ICCR = ICCR_IRQ_CLR;
	iowrite32(ICCR, (base+I2C_ICCR_OFFS));

	if (!nxp_soc_gpio_get_in_value(gpio)) {
		gpio_request(gpio,NULL);
		gpio_direction_output(gpio, 1);
		start = jiffies;
		while (!gpio_get_value(gpio)) {
			if (time_after(jiffies, start + timeout)) {
				if (gpio_get_value(gpio))
					break;
				ret = -ETIMEDOUT;
				goto _stop_timeout;
			}
			cpu_relax();
		}
	}

_stop_timeout:
	nxp_soc_gpio_set_io_func(gpio, 1);
	return ret;
}

static inline void i2c_stop_dev(struct nxp_i2c_param *par, int stop, int read)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0, ICCR = 0;

	if (stop) {
		gpio_request(par->hw.sda_io,NULL);	 //gpio_Request
		gpio_direction_output(par->hw.sda_io,0); //SDA LOW
		udelay(1);

		i2c_stop_scl(par);

		udelay(1);
		gpio_set_value(par->hw.sda_io,1);	 //STOP Signal Gen
		nxp_soc_gpio_set_io_func(par->hw.sda_io, 1);
	} else {
		#if (NOSTOP_GPIO)
		gpio_request(par->hw.sda_io,NULL);	 //gpio_Request
		gpio_direction_output(par->hw.sda_io,1); //SDA LOW
		udelay(1);
		gpio_request(par->hw.scl_io,NULL);	 //gpio_Request
		gpio_direction_output(par->hw.scl_io,1); //SDA LOW
		#endif
		/*
		ICSR  = ioread32(base+I2C_ICSR_OFFS);
		ICSR &= ~ICSR_OUT_ENB;
		*/
		ICSR  = par->trans_mode;
		iowrite32(ICSR, (base+I2C_ICSR_OFFS));

		ICCR = ICCR_IRQ_CLR;
		iowrite32(ICCR, (base+I2C_ICCR_OFFS));


		#if (NOSTOP_GPIO)
		nxp_soc_gpio_set_io_func(par->hw.sda_io, 1);
		nxp_soc_gpio_set_io_func(par->hw.scl_io, 1);
		#endif
	}
}

static inline void i2c_wait_dev(struct nxp_i2c_param *par, int wait)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0;

	do {
		ICSR = ioread32(base+I2C_ICSR_OFFS);
		if ( !(ICSR & ICSR_BUS_BUSY_READ) &&  !(ICSR & ICSR_ARI_STA) )
			break;
		mdelay(1);
	} while (wait-- > 0);
}

static inline void i2c_bus_off(struct nxp_i2c_param *par)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICSR = 0;

	ICSR = ioread32(base+I2C_ICSR_OFFS);
	ICSR &= ~ICSR_OUT_ENB;
	iowrite32(ICSR, (base+I2C_ICSR_OFFS));
}

/*
 * 	Hardware I2C
 */
static inline void i2c_set_clock(struct nxp_i2c_param *par, int enable)
{
	void __iomem *base = par->hw.base_addr;
	unsigned int ICCR = 0, ICSR = 0;

	if (par->hw.i2c_divisor > 16 || (par->hw.i2c_prescaler == 16 && par->hw.i2c_divisor < 2))
		pr_debug("%s: Invalid CLOCK settings\n" \
			"i2c.%d, par->hw.i2c_prescaler:%d, par->hw.i2c_divisor:%d, %s\n",
			__func__,  par->hw.port, par->hw.i2c_prescaler, par->hw.i2c_divisor, enable?"on":"off");

	pr_debug("%s: i2c.%d, par->hw.i2c_prescaler:%d, par->hw.i2c_divisor:%d, %s\n",
		__func__,  par->hw.port, par->hw.i2c_prescaler, par->hw.i2c_divisor, enable?"on":"off");

	if (enable) {
		ICCR = ioread32(base+I2C_ICCR_OFFS);
		ICCR &= ~(ICCR_CLK_SRC | ICCR_CLK_MASK); /* remove old prescaler and clock divisor */
		if (par->hw.i2c_prescaler == 256)
			ICCR |= ICCR_CLK_SRC;	/* prescaler /256 */

		ICCR |= (par->hw.i2c_divisor-1);	/* clock value size already checked */
		iowrite32(ICCR,(base+I2C_ICCR_OFFS));
	} else {
		ICSR  = ioread32(base+I2C_ICSR_OFFS);
		ICSR &= ~ICSR_OUT_ENB;
		iowrite32(ICSR, (base+I2C_ICSR_OFFS));
	}
}

static inline int i2c_wait_busy(struct nxp_i2c_param *par)
{
	void __iomem *base = par->hw.base_addr;
	int wait = 500;
	int ret = 0;

	pr_debug("%s(i2c.%d, stop:%d)\n", __func__, par->hw.port, par->stop);

	/* busy status check*/
	i2c_wait_dev(par, wait);

	if (0 > wait) {
		printk(KERN_ERR "Fail, i2c.%d is busy, arbitration %s ...\n",
			par->hw.port,
			(ioread32(base + I2C_ICSR_OFFS) & ICSR_ARI_STA)?"busy":"free");
		ret = -1;
	}
	return ret;
}

static irqreturn_t i2c_irq_thread(int irqno, void *dev_id)
{
	struct nxp_i2c_param *par = dev_id;
	struct i2c_msg *msg = par->msg;
	void __iomem *base = par->hw.base_addr;
	u16 flags = par->msg->flags;
	int len = msg->len;
	int cnt = par->trans_count;

	par->thd_count++;

	/* Arbitration Check. */
	if (((ioread32(base + I2C_ICSR_OFFS) & ICSR_ARI_STA) != 0)) {
		pr_err("Fail, arbit i2c.%d addr[0x%02x] %s, data[0x%02x], trans[%2d:%2d]\n",
			par->hw.port, msg->addr, (msg->flags&I2C_M_RD)?"R":"W", par->pre_data, cnt, len);
		par->trans_status = I2C_TRANS_ERR;
		goto __irq_end;
	}

	/* if ACK was requested, was it received? */
	if (par->request_ack &&
		(ioread32(base + I2C_ICSR_OFFS) & ICSR_ACK_REV)) {
		pr_err("Fail, noack i2c.%d addr[0x%02x] %s, data[0x%02x], trans[%2d:%2d]\n",
			par->hw.port, msg->addr, (msg->flags&I2C_M_RD)?"R":"W", par->pre_data, cnt, len);
		par->trans_status = I2C_TRANS_ERR;
		goto __irq_end;
	}

	if (I2C_TRANS_RUN == par->trans_status) {
		if (flags & I2C_M_RD) {
			int ack  = (len == cnt + 1) ? 0: 1;
			int last = (len == cnt + 1) ? 1: 0;

			par->request_ack = 0;
			if (0 == cnt) {
				/* send address */
				i2c_trans_dev(base, ack, 0);
				par->trans_count += 1;
				return IRQ_HANDLED;
			}

			/* read data */
			msg->buf[cnt - 1] = ioread8(base + I2C_IDSR_OFFS);

			if (len == par->trans_count) {
				par->trans_status = I2C_TRANS_DONE;
				goto __irq_end;
			} else {
				i2c_trans_dev(base, ack, last);
				par->trans_count += 1;
				return IRQ_HANDLED;
			}
		} else {
			par->pre_data = msg->buf[cnt];
			par->request_ack = (msg->flags & I2C_M_IGNORE_NAK) ? 0 : 1;
			par->trans_count += 1;

			if (len == par->trans_count)
				par->trans_status = I2C_TRANS_DONE;

			iowrite8(msg->buf[cnt], base + I2C_IDSR_OFFS);	/* write data */
			i2c_trans_dev(base, 0, par->trans_status == I2C_TRANS_DONE ? 1 : 0);

			return IRQ_HANDLED;
		}
	}

__irq_end:
	i2c_stop_dev(par, par->stop, (flags & I2C_M_RD ? 0 : 1));

	par->condition = 1;
	wake_up(&par->wait_q);

	return IRQ_HANDLED;
}

static irqreturn_t i2c_irq_handler(int irqno, void *dev_id)
{
	struct nxp_i2c_param *par = dev_id;
    unsigned int ICCR = 0;
    void __iomem *base = par->hw.base_addr;

	par->irq_count++;

	if (!par->running) {
		pr_err("************ I2C.%d IRQ NO RUN ************\n", par->hw.port);
		return IRQ_HANDLED;
	}

	if (par->polling) {
		pr_err("************ I2C.%d IRQ POLLING ************\n", par->hw.port);
		i2c_irq_thread(irqno,dev_id);
		return IRQ_HANDLED;
	}

	ICCR  = ioread32((base+I2C_ICCR_OFFS));
	ICCR  |= ICCR_IRQ_CLR;
	iowrite32(ICCR, base+I2C_ICCR_OFFS);

	return IRQ_WAKE_THREAD;
}

static int i2c_trans_done(struct nxp_i2c_param *par)
{
	void __iomem *base = par->hw.base_addr;
	struct i2c_msg *msg = par->msg;
	int wait, timeout, ret = 0;

	par->condition = 0;

	if (!par->polling) {
		wait = msg->len * msecs_to_jiffies(par->timeout);
		timeout = wait_event_timeout(par->wait_q, par->condition, wait);
	} else {
		wait = par->timeout/10;
		while (wait--) {
			if (I2C_TRANS_DONE == par->trans_status ||
			    I2C_TRANS_ERR == par->trans_status) {
			   	par->condition = 1;
				break;
			}
			mdelay(10);
		}
	}

	if (par->condition) { /* done or error */
		if (I2C_TRANS_ERR == par->trans_status)
			ret = -1;
	} else {
		pr_err("Fail, i2c.%d addr[0x%02x] %s, cond(%d) pend(%s) arbit(%s) mod(%s) tran(%d:%d,%d:%d) wait(%dms)\n",
			par->hw.port, par->msg->addr, (msg->flags&I2C_M_RD)?"R":"W",
			par->condition,
			(ioread32(base + I2C_ICCR_OFFS) & ICCR_IRQ_PND)?"yes":"no",
			(ioread32(base + I2C_ICSR_OFFS) & ICSR_ARI_STA)?"busy":"free",
			par->polling?"polling":"irq", par->trans_count, msg->len,
			par->irq_count,	par->thd_count, par->timeout);
		ret = -1;
	}

	if (0 > ret)
		i2c_stop_dev(par, 1, 0);

	return ret;
}

static inline int i2c_trans_data(struct nxp_i2c_param *par, struct i2c_msg *msg)
{
	if (msg->flags & I2C_M_TEN) {
		printk(KERN_ERR "Fail, i2c.%d not support ten bit addr:0x%02x, flags:0x%x \n",
			par->hw.port, msg->addr, msg->flags);
		return -1;
	}
	if (msg->flags & I2C_M_RD) {
		par->pre_data = msg->addr << 1 | 1;	/* READ */
		par->trans_mode = ICSR_TXRXMODE_MASTER_RX;
	} else {
		par->pre_data = msg->addr << 1 | 0;	/* WRITE */
		par->trans_mode = ICSR_TXRXMODE_MASTER_TX;
	}

	pr_debug("%s(i2c.%d, (addr<<1)|RW:addr[0x%02x])\n",
		__func__, par->hw.port, par->pre_data);

 	/* clear irq condition */
	par->msg = msg;
	par->condition = 0;
	par->request_ack = (msg->flags & I2C_M_IGNORE_NAK ) ? 0 : 1;
	par->trans_count = 0;
	par->trans_status = I2C_TRANS_RUN;

	i2c_start_dev(par);

	/* wait for end of transfer */
	return i2c_trans_done(par);
}

static int nxp_i2c_transfer(struct nxp_i2c_param *par, struct i2c_msg *msg)
{
	int ret = -EAGAIN;

	pr_debug("\n%s(flags:0x%x, %c)\n",
		__func__, msg->flags, msg->flags&I2C_M_RD?'R':'W');

	i2c_set_clock(par, 1);	/* start clock */

	if (0 > i2c_wait_busy(par)) {
		pr_err("%s : i2c.%d addr[0x%02x] %s, Err", __func__, par->hw.port, msg->addr, (msg->flags&I2C_M_RD)?"R":"W");
		goto err_i2c;
	}

	if (0 > i2c_trans_data(par, msg)) {
		pr_err("%s : i2c.%d addr[0x%02x] %s, Err", __func__, par->hw.port, msg->addr, (msg->flags&I2C_M_RD)?"R":"W");
		goto err_i2c;
	}

	ret = msg->len;

err_i2c:
	if (ret != msg->len)
		msg->flags &= ~I2C_M_NOSTART;

	i2c_wait_busy(par);
	i2c_set_clock(par, 0);

	return ret;
}

static int nxp_i2c_algo_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct nxp_i2c_param *par  = adapter->algo_data;
	struct i2c_msg  *tmsg = msgs;
	int i = adapter->retries;
	int j = 1;
	int ret = -EAGAIN;
	int len = 0;

	par->polling = 1;

	/* lock */
	if (!preempt_count()) {
		mutex_lock(&par->lock);
		par->polling = 0;
	}

	par->running   = 1;
	par->irq_count = 0;
	par->thd_count = 0;

	pr_debug("\n %s(msg num:%d)\n", __func__, num);

	par->stop = 0;		/* clear STOP signal */

	/* loop once for each message */
	for (j=1; j <= num; j++, tmsg++) {

		if (j == num) {
			par->stop = 1; /* signal STOP after last message */
		}
		len = tmsg->len;

		/* transfer */
		for (i = adapter->retries; i > 0; i--) {
			ret = nxp_i2c_transfer(par, tmsg);
			if (ret == len)
				break;
			pr_err("i2c.%d addr[0x%02x] %s, (try:%d)\n",
				par->hw.port, tmsg->addr, (tmsg->flags&I2C_M_RD)?"R":"W", adapter->retries-i+1);
		}

		/* Error */
		if (ret != len)
			break;
	}

	par->running = 0;
	par->polling = 0;

	if (!preempt_count())
		mutex_unlock(&par->lock);

	if (ret == len)
		return num;

	pr_err("Error: i2c.%d, addr[0x%02x] %s, trans len:%d(%d), try:%d\n",
		par->hw.port, msgs->addr, (msgs->flags&I2C_M_RD)?"R":"W", ret, len, adapter->retries);

	return ret;
}

static u32 nxp_i2c_algo_fn(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK) ;
}

static struct i2c_algorithm nxp_i2c_algo = {
	.master_xfer 	= nxp_i2c_algo_xfer,
	.functionality 	= nxp_i2c_algo_fn,
};

static int prescaler_divisor[2][3]={	/* prescale, min divisor, max divisor */
	{ 16, 2, 16},			/* prescaler 16 has divisor range [2,16]  */
	{256, 1, 16},			/* prescaler 256 has divisor range [1,16] */
};
#define PRESCALER_INDEX		0
#define MIN_DIVISOR_INDEX	1
#define MAX_DIVISOR_INDEX	2

static int nxp_i2c_set_param(struct nxp_i2c_param *par, struct platform_device *pdev)
{
	struct nxp_i2c_plat_data *plat = pdev->dev.platform_data;
	unsigned long pclk_rate = 0;
	int ret = 0;

	unsigned long actual_i2c_rate = 0, requested_i2c_rate =0 ;
	unsigned long i2c_rate_error , candidate_i2c_rate_error = 0;
	unsigned int candidate_i2c_prescaler = 0,  candidate_i2c_divisor = 0;
	int divisor = 0 ;
	struct clk *clk;
	unsigned int i=0, prescaler = 0;

	/* set par hardware */
	par->hw.port	= plat->port;
	par->hw.irqno	= plat->irq;
	par->hw.base_addr = (void __iomem *)IO_ADDRESS(plat->base_addr);
	par->hw.scl_io	= plat->gpio->scl_pin ? plat->gpio->scl_pin : i2c_gpio[plat->port][0];
	par->hw.sda_io	= plat->gpio->sda_pin ? plat->gpio->sda_pin : i2c_gpio[plat->port][1];
	par->stop	= 1;
	par->timeout	= WAIT_ACK_TIME;
	par->rate	= plat->rate ? plat->rate : I2C_CLOCK_RATE;

	nxp_soc_gpio_set_io_func(par->hw.scl_io, 1);
	nxp_soc_gpio_set_io_func(par->hw.sda_io, 1);

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		return ret;
	}

	pclk_rate = clk_get_rate(clk);
	clk_enable(clk);
	par->clk = clk;

	requested_i2c_rate = par->rate;

	candidate_i2c_rate_error = ULONG_MAX;	/* Ensure update inside search loop */

	/* prime with slowest I2C speed */
	candidate_i2c_prescaler = prescaler_divisor[1][PRESCALER_INDEX];
	candidate_i2c_divisor   = prescaler_divisor[1][MAX_DIVISOR_INDEX];


	/* determine prescaler and divisor values */
	for (i = 0; i < 2; i ++) {

		/* choose prescaler divide by 16 or 256 */
		prescaler = prescaler_divisor[i][PRESCALER_INDEX];

		/* Loop though I2C speeds, from slowest to fastest.
		 * Find the I2C speed at or below the requested rate.
		 */
		for (divisor = prescaler_divisor[i][MAX_DIVISOR_INDEX];
		     divisor >= prescaler_divisor[i][MIN_DIVISOR_INDEX];
		     divisor--) {
			actual_i2c_rate = pclk_rate/prescaler/divisor;	/* proposed I2C speed */
			if (actual_i2c_rate <= requested_i2c_rate ) {
				i2c_rate_error = requested_i2c_rate - actual_i2c_rate;
				/* better speed match found, save settings */
				if (i2c_rate_error <= candidate_i2c_rate_error) {
					candidate_i2c_rate_error = i2c_rate_error;
					candidate_i2c_divisor    = divisor;
					candidate_i2c_prescaler  = prescaler;
				}
			} else {
				break;	/* remaining divisors are faster than requested I2C rate */
			}
		}
		if (candidate_i2c_rate_error == 0)
			break;	/* exact I2C rate match, stop searching */
	}

	par->hw.i2c_prescaler = candidate_i2c_prescaler;
	par->hw.i2c_divisor = candidate_i2c_divisor;

	nxp_soc_rsc_reset(i2c_reset[plat->port]);

	/* init par resource */
	mutex_init(&par->lock);
	init_waitqueue_head(&par->wait_q);

	printk("%s.%d: %8ld hz [pclk=%ld, clk = %3d, scale=%2d, timeout=%4d ms]\n",
		DEV_NAME_I2C, par->hw.port, pclk_rate/candidate_i2c_prescaler/candidate_i2c_divisor,
		pclk_rate, par->hw.i2c_prescaler, par->hw.i2c_divisor-1, par->timeout);

	ret = request_threaded_irq(par->hw.irqno, i2c_irq_handler, i2c_irq_thread,
				IRQF_DISABLED|IRQF_SHARED , DEV_NAME_I2C, par);
	if (ret)
		printk(KERN_ERR "Fail, i2c.%d request irq %d ...\n", par->hw.port, par->hw.irqno);

	i2c_bus_off(par);

	return ret;
}

static int nxp_i2c_probe(struct platform_device *pdev)
{
	struct nxp_i2c_param *par = NULL;
	int ret = 0;
	pr_debug("%s name:%s, id:%d\n", __func__, pdev->name, pdev->id);

	/* allocate nxp_i2c_param data */
	par = kzalloc(sizeof(struct nxp_i2c_param), GFP_KERNEL);
	if (!par) {
		printk(KERN_ERR "Fail, %s allocate driver info ...\n", pdev->name);
		return -ENOMEM;
	}

	/* init par data struct */
	ret = nxp_i2c_set_param(par, pdev);
	if (0 > ret)
		goto err_mem;

	/* init par adapter */
	strlcpy(par->adapter.name, DEV_NAME_I2C, I2C_NAME_SIZE);

	par->adapter.owner 	= THIS_MODULE;
	par->adapter.nr 	= par->hw.port;
	par->adapter.class 	= I2C_CLASS_HWMON | I2C_CLASS_SPD;
	par->adapter.algo 	= &nxp_i2c_algo;
	par->adapter.algo_data	= par;
	par->adapter.dev.parent	= &pdev->dev;
	par->adapter.retries 	= TRANS_RETRY_CNT;

	ret = i2c_add_numbered_adapter(&par->adapter);
	if (ret) {
		printk(KERN_ERR "Fail, i2c.%d add to adapter ...\n", par->hw.port);
		goto err_irq;
	}

	/* set driver data */
	platform_set_drvdata(pdev, par);
	return ret;

err_irq:
	free_irq(par->hw.irqno, par);

err_mem:
	kfree(par);
	return ret;
}

static int nxp_i2c_remove(struct platform_device *pdev)
{
	struct nxp_i2c_param *par = platform_get_drvdata(pdev);
	int rsc = i2c_reset[par->hw.port];
	int irq = par->hw.irqno;

	nxp_soc_rsc_enter(rsc);
	clk_disable(par->clk);

	free_irq(irq, par);
	i2c_del_adapter(&par->adapter);
	kfree(par);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int nxp_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct nxp_i2c_param *par = platform_get_drvdata(pdev);
	int rsc = i2c_reset[par->hw.port];
	nxp_soc_rsc_enter(rsc);
	PM_DBGOUT("%s \n", __func__);
	return 0;
}

static int nxp_i2c_resume(struct platform_device *pdev)
{
	struct nxp_i2c_param *par = platform_get_drvdata(pdev);
	int sda = par->hw.sda_io;
	int scl = par->hw.scl_io;
	int rsc = i2c_reset[par->hw.port];

	PM_DBGOUT("%s\n", __func__);
	nxp_soc_gpio_set_io_func(scl, 1);
	nxp_soc_gpio_set_io_func(sda, 1);
	nxp_soc_rsc_reset(rsc);

	clk_enable(par->clk);
	mdelay(1);

	i2c_bus_off(par);
	return 0;
}

#else
#define nxp_i2c_suspend		NULL
#define nxp_i2c_resume		NULL
#endif

static struct platform_driver i2c_plat_driver = {
	.probe	 = nxp_i2c_probe,
	.remove	 = nxp_i2c_remove,
	.suspend = nxp_i2c_suspend,
	.resume	 = nxp_i2c_resume,
	.driver	 = {
		.owner	 = THIS_MODULE,
		.name	 = DEV_NAME_I2C,
	},
};

static int __init nxp_i2c_init(void)
{
	return platform_driver_register(&i2c_plat_driver);
}

static void __exit nxp_i2c_exit(void)
{
	platform_driver_unregister(&i2c_plat_driver);
}

subsys_initcall(nxp_i2c_init);
module_exit(nxp_i2c_exit);

MODULE_DESCRIPTION("I2C driver for the Nexell");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: nexell par");
