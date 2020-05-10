/*
 * (C) Copyright 2009
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
#include <linux/init.h>
#include <linux/device.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/oom.h>
#include <linux/platform_device.h>
#include <mach/platform.h>
#include <mach/devices.h>

/*
#define pr_debug	printk
*/

// Limited Temperature Throthold for Drone.
#define TEMPERTURE_LIMIT_NONE	60
#define TEMPERTURE_LIMIT_LEVEL0	65
#define TEMPERTURE_LIMIT_LEVEL1	70
#define TEMPERTURE_LIMIT_LEVEL2	75

// CPU Usage Throthold for Drone.
#define	CPU_OVER_USAGELEVEL		49
#define	CPU_UNDER_USAGELEVEL	20

struct cpufreq_limit_data {
	char **limit_name;
	int limit_num;
	long aval_max_freq; 	/* unit Khz */
	long op_max_freq;		/* unit Khz */
	long limit_level0_freq; 	/* unit Khz */
	long limit_level1_freq; 	/* unit Khz */
	long min_max_freq;			/* unit Khz */
	long timer_duration;	/* unit ms */
	long op_timeout;		/* unit ms */
	int timer_chkcpu_mod;
	long time_stamp;
	long frequency; 		/* unit Khz */
	long pre_max_freq;		/* unit Khz */
	struct hrtimer limit_timer;
	struct work_struct limit_work;
	struct notifier_block limit_nb;

};


extern int curMaxCpu;
int isCpuMaxFrequency(void)
{
	return curMaxCpu;
}
EXPORT_SYMBOL_GPL(isCpuMaxFrequency);


#define SIZE_RD_STAT 128
extern int NXL_Get_BoardTemperature(void);
#if defined(CONFIG_ARM_NXP4330_CPUFREQ_BY_RESOURCE) && defined(CONFIG_BATTERY_NXE2000)
extern int isOccured_dieError(void);
extern void isReset_dieErrorFlag(void);
#endif

typedef struct {
    unsigned long user;
    unsigned long system;
    unsigned long nice;
    unsigned long idle;
    unsigned long wait;
    unsigned long hi;
    unsigned long si;
    unsigned long zero;
} procstat_t;

typedef struct nxp_cpuLimit_control {
	int cpuUsageNx4330;
	int board_temperature;
	int tmp_voltage;

	int prev_board_temperature;

	int overCnt_temperature;
	int underCnt_temperature;
	int overTemperature;
	int dieErrorFlag;
	int OccuredDieError;

	procstat_t m_prev;
	procstat_t m_curr;
	int usageOverContinuousCount;
	int usageOverFlag;

}NXP_LIMITCPU_CTRL;
NXP_LIMITCPU_CTRL ctrl_cpuTemp;


//
bool max_limit_min = 0;
bool max_limit_under = 0;
bool max_limit_dangerous = 0;
void isReset_LimitMaxUner(void)
{
 	max_limit_min = 0;
 	max_limit_under = 0;
	max_limit_dangerous = 0;
}
void isSet_LimitMaxUner(void)
{
 	max_limit_under = 1;
}
bool isGet_LimitMaxUner(void)
{
 	return max_limit_under;
}

void isReset_LimitMin(void)
{
 	max_limit_min = 0;
}
void isSet_LimitMin(void)
{
 	max_limit_min = 1;
}
bool isGet_LimitMin(void)
{
 	return max_limit_min;
}

//


/* d = a - b */
void diff_proc_stat(procstat_t *a, procstat_t *b, procstat_t *d)
{
    d->user     = a->user   - b->user;
    d->system   = a->system - b->system;
    d->nice     = a->nice   - b->nice;
    d->idle     = a->idle   - b->idle;
    d->wait     = a->wait   - b->wait;
    d->hi       = a->hi     - b->hi;
    d->si       = a->si     - b->si;
    d->zero     = a->zero   - b->zero;
}

int cpuUsage_getToken(char *buf, int *offset_usage)
{
	int cnt = 0;

	while(1)
	{
		if(buf[cnt]==' ')
			break;
		cnt++;
	}
	*offset_usage = cnt+1;

	if(buf[cnt+1] == ' ')
	{
		cnt++;
		while(1)
		{
			cnt++;
			if(buf[cnt]!=' ')
			{
				*offset_usage = cnt;
				break;
			}
		}
	}

	return *offset_usage;
}


void cpuUsage_getValue(char *buf, int *offsetUsage)
{
	int cnt = 0;
	unsigned long long value[11];
	int value_cnt =0;
	int bFinish=0;

	value_cnt = 0;
_reParsing:

	value[value_cnt] = 0;

	// get the value
	while(1)
	{
		if(buf[cnt]==' ')
			break;
		if(buf[cnt]=='\n' || buf[cnt]=='\r' || buf[cnt]=='\0')
		{
			bFinish = 1;
			break;
		}

		value[value_cnt] = 10*value[value_cnt] + (buf[cnt]-'0');
		cnt++;
	}
	if(bFinish)
	{
	    ctrl_cpuTemp.m_curr.user = value[0];
	    ctrl_cpuTemp.m_curr.system = value[1];
	    ctrl_cpuTemp.m_curr.nice = value[2];
	    ctrl_cpuTemp.m_curr.idle = value[3];
	    ctrl_cpuTemp.m_curr.wait = value[4];
	    ctrl_cpuTemp.m_curr.hi = value[5];
	    ctrl_cpuTemp.m_curr.si = value[6];
	    ctrl_cpuTemp.m_curr.zero = value[7];
		return;
	}

	cnt++;
	while(1)
	{
		if(buf[cnt]=='\n' || buf[cnt]=='\r' || buf[cnt]=='\0')
		{
			bFinish = 1;
			break;
		}

		if(buf[cnt]!=' ')
			break;
		cnt++;
	}

	*offsetUsage += cnt;

	if(bFinish)
	{
	    ctrl_cpuTemp.m_curr.user = value[0];
	    ctrl_cpuTemp.m_curr.system = value[1];
	    ctrl_cpuTemp.m_curr.nice = value[2];
	    ctrl_cpuTemp.m_curr.idle = value[3];
	    ctrl_cpuTemp.m_curr.wait = value[4];
	    ctrl_cpuTemp.m_curr.hi = value[5];
	    ctrl_cpuTemp.m_curr.si = value[6];
	    ctrl_cpuTemp.m_curr.zero = value[7];
		return;
	}

	value_cnt++;
goto _reParsing;

	return ;
}

static int _read_statfile(char *path, char *buf, int size)
{
	int fd;
	mm_segment_t old_fs;

	fd = sys_open(path, O_RDONLY, 0);
	old_fs = get_fs();
	if (0 > fd)
		return 1;

	set_fs(KERNEL_DS);
	sys_read(fd, (void*)buf, size);

	set_fs(old_fs);
	sys_close(fd);

	return 0;

}

void GetCPUInfo(char *buf)
{
    procstat_t d;
    procstat_t u100; /* percentages(%x100) per total */
    long total, usage100;
	int offset_usage;

	cpuUsage_getToken(buf, &offset_usage);
	cpuUsage_getValue(&buf[offset_usage], &offset_usage);

    diff_proc_stat(&ctrl_cpuTemp.m_curr, &ctrl_cpuTemp.m_prev, &d);
    total = d.user + d.system + d.nice + d.idle + d.wait + d.hi + d.si;
    u100.user   = d.user * 10000 / total;
    u100.system = d.system * 10000 / total;
    u100.nice   = d.nice * 10000 / total;
    u100.idle   = (d.idle * 10000) / total;
    u100.wait   = d.wait * 10000 / total;
    u100.hi     = d.hi * 10000 / total;
    u100.si     = d.si * 10000 / total;
    usage100 = 10000 - u100.idle;
 	ctrl_cpuTemp.cpuUsageNx4330 = usage100;

    pr_debug("cpuusage :%02d.%02d %% curMaxCpu(%d) temperature(%d) die(%d)ntic(%d)\n",
    	(usage100/100), (usage100%100), curMaxCpu, NXL_Get_BoardTemperature(),
#if defined(CONFIG_ARM_NXP4330_CPUFREQ_BY_RESOURCE) && defined(CONFIG_BATTERY_NXE2000)
    	isOccured_dieError(),
#else
		0,
#endif
    	max_limit_dangerous);

    ctrl_cpuTemp.m_prev.user = ctrl_cpuTemp.m_curr.user ;
    ctrl_cpuTemp.m_prev.system =  ctrl_cpuTemp.m_curr.system ;
    ctrl_cpuTemp.m_prev.nice = ctrl_cpuTemp.m_curr.nice;
    ctrl_cpuTemp.m_prev.idle = ctrl_cpuTemp.m_curr.idle;
    ctrl_cpuTemp.m_prev.wait = ctrl_cpuTemp.m_curr.wait;
    ctrl_cpuTemp.m_prev.hi = ctrl_cpuTemp.m_curr.hi;
    ctrl_cpuTemp.m_prev.si = ctrl_cpuTemp.m_curr.si;
    ctrl_cpuTemp.m_prev.zero = ctrl_cpuTemp.m_curr.zero;

}

void _GetCupInfomation(void)
{
	char buffer[SIZE_RD_STAT];

	if(_read_statfile("/proc/stat", buffer, SIZE_RD_STAT) == 0)
		GetCPUInfo(buffer);
}

long funcGetMaxFreq(struct cpufreq_limit_data *limit)
{
	long max_freq = 0;
	int temperature = NXL_Get_BoardTemperature();
	int cpuUsage = ctrl_cpuTemp.cpuUsageNx4330/100;

	if(temperature < TEMPERTURE_LIMIT_NONE)
		isReset_LimitMaxUner();

	if(temperature < TEMPERTURE_LIMIT_LEVEL0)
	{
		max_limit_dangerous = 0;
#if defined(CONFIG_ARM_NXP4330_CPUFREQ_BY_RESOURCE) && defined(CONFIG_BATTERY_NXE2000)
		isReset_dieErrorFlag();
#endif
		isReset_LimitMin();
		if(cpuUsage>CPU_OVER_USAGELEVEL)
		{
			if(isGet_LimitMaxUner() == 0)
				{
				printk("max return \n");
				max_freq = limit->aval_max_freq;
				}
			else
				max_freq = limit->limit_level0_freq;
		}
		else if(cpuUsage>CPU_UNDER_USAGELEVEL)
		{
			max_freq = limit->limit_level0_freq;
		}
		else
		{
			max_freq = limit->limit_level1_freq;
		}
	}
	else if(temperature < TEMPERTURE_LIMIT_LEVEL1)
	{
		max_limit_dangerous = 0;
#if defined(CONFIG_ARM_NXP4330_CPUFREQ_BY_RESOURCE) && defined(CONFIG_BATTERY_NXE2000)
		isReset_dieErrorFlag();
#endif
		isSet_LimitMaxUner();
		if(cpuUsage>CPU_OVER_USAGELEVEL)
		{

			if(isGet_LimitMin() == 0)
				max_freq = limit->limit_level0_freq;
			else
				max_freq = limit->limit_level1_freq;
		}
		else
			max_freq = limit->limit_level1_freq;
	}
	else if(temperature < TEMPERTURE_LIMIT_LEVEL2)
	{
		isSet_LimitMin();
		if(cpuUsage>CPU_OVER_USAGELEVEL)
			max_freq = limit->limit_level1_freq;
		else
			max_freq = limit->min_max_freq;
	}
	else // Over Threthold -> set the min max_freq.
	{
		max_limit_dangerous = 1;
	}


	if(
#if defined(CONFIG_ARM_NXP4330_CPUFREQ_BY_RESOURCE) && defined(CONFIG_BATTERY_NXE2000)
		isOccured_dieError() || 
#endif
		max_limit_dangerous) // DieError
	{
		return limit->min_max_freq;
	}


	return max_freq;

}

long cpuUsage_Process(struct cpufreq_limit_data *limit, int boost)
{
	if(boost) return limit->aval_max_freq;

	_GetCupInfomation();

	return funcGetMaxFreq(limit);
}

