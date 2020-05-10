/*
 * Customer HW 4 dependant file
 *
 * $Copyright Open Broadcom Corporation$
 *
 * $Id: dhd_sec_feature.h$
 */

#ifndef _dhd_sec_feature_h_
#define _dhd_sec_feature_h_

/* PROJECTS */

#if defined(CONFIG_MACH_SAMSUNG_ESPRESSO) || defined(CONFIG_MACH_SAMSUNG_ESPRESSO_10)
#define READ_MACADDR
#define HW_OOB
#endif /* CONFIG_MACH_SAMSUNG_ESPRESSO && CONFIG_MACH_SAMSUNG_ESPRESSO_10 */

/* Q1 also uses this feature */
#if defined(CONFIG_MACH_U1) || defined(CONFIG_MACH_TRATS)
#ifdef CONFIG_MACH_Q1_BD
#define HW_OOB
#endif /* CONFIG_MACH_Q1_BD */
#define USE_CID_CHECK
#define WRITE_MACADDR
#endif /* CONFIG_MACH_U1 || CONFIG_MACH_TRATS */

#ifdef CONFIG_ARCH_MSM7X30
#define HW_OOB
#define READ_MACADDR
#endif /* CONFIG_ARCH_MSM7X30 */

#if defined(CONFIG_MACH_GC1) || defined(CONFIG_MACH_U1_NA_SPR)
#undef USE_CID_CHECK
#define READ_MACADDR
#endif /* CONFIG_MACH_GC1 || CONFIG_MACH_U1_NA_SPR */

#ifdef CONFIG_MACH_P10
#define READ_MACADDR
#endif /* CONFIG_MACH_P10 */

#ifdef CONFIG_ARCH_MSM8960
#undef WIFI_TURNOFF_DELAY
#define WIFI_TURNOFF_DELAY	200
#endif /* CONFIG_ARCH_MSM8960 */

/* REGION CODE */
#ifndef CONFIG_WLAN_REGION_CODE
#define CONFIG_WLAN_REGION_CODE 100
#endif /* CONFIG_WLAN_REGION_CODE */

#if (CONFIG_WLAN_REGION_CODE >= 100) && (CONFIG_WLAN_REGION_CODE < 200)     /* EUR */
#if (CONFIG_WLAN_REGION_CODE == 101)     /* EUR ORG */
/* GAN LITE NAT KEEPALIVE FILTER */
#define GAN_LITE_NAT_KEEPALIVE_FILTER
#endif /* CONFIG_WLAN_REGION_CODE == 101 */
#endif /* CONFIG_WLAN_REGION_CODE >= 100 && CONFIG_WLAN_REGION_CODE < 200 */

#if (CONFIG_WLAN_REGION_CODE >= 200) && (CONFIG_WLAN_REGION_CODE < 300)     /* KOR */
#undef USE_INITIAL_2G_SCAN_ORG
#ifndef ROAM_ENABLE
#define ROAM_ENABLE
#endif /* ROAM_ENABLE */
#ifndef ROAM_API
#define ROAM_API
#endif /* ROAM_API */
#ifndef ROAM_CHANNEL_CACHE
#define ROAM_CHANNEL_CACHE
#endif /* ROAM_CHANNEL_CACHE */
#ifndef OKC_SUPPORT
#define OKC_SUPPORT
#endif /* OKC_SUPPORT */

#ifndef ROAM_AP_ENV_DETECTION
#define ROAM_AP_ENV_DETECTION
#endif /* ROAM_AP_ENV_DETECTION */

#undef WRITE_MACADDR
#undef READ_MACADDR
#ifdef CONFIG_BCM4334
#define READ_MACADDR
#else
#define RDWR_MACADDR
#endif /* CONFIG_BCM4334 */

#if (CONFIG_WLAN_REGION_CODE == 201)     /* SKT */
#ifdef CONFIG_MACH_UNIVERSAL5410
/* Make CPU core clock 300MHz & assign dpc thread workqueue to CPU1 */
#define FIX_CPU_MIN_CLOCK
#endif /* CONFIG_MACH_UNIVERSAL5410 */
#endif /* CONFIG_WLAN_REGION_CODE == 201 */

#if (CONFIG_WLAN_REGION_CODE == 202)     /* KTT */
#define VLAN_MODE_OFF
#define CUSTOM_KEEP_ALIVE_SETTING	30000
#define FULL_ROAMING_SCAN_PERIOD_60_SEC

#ifdef CONFIG_MACH_UNIVERSAL5410
/* Make CPU core clock 300MHz & assign dpc thread workqueue to CPU1 */
#define FIX_CPU_MIN_CLOCK
#endif /* CONFIG_MACH_UNIVERSAL5410 */
#endif /* CONFIG_WLAN_REGION_CODE == 202 */

#if (CONFIG_WLAN_REGION_CODE == 203)     /* LGT */
#ifdef CONFIG_MACH_UNIVERSAL5410
/* Make CPU core clock 300MHz & assign dpc thread workqueue to CPU1 */
#define FIX_CPU_MIN_CLOCK
#define FIX_BUS_MIN_CLOCK
#endif /* CONFIG_MACH_UNIVERSAL5410 */
#endif /* CONFIG_WLAN_REGION_CODE == 203 */
#endif /* CONFIG_WLAN_REGION_CODE >= 200 && CONFIG_WLAN_REGION_CODE < 300 */

#if (CONFIG_WLAN_REGION_CODE >= 300) && (CONFIG_WLAN_REGION_CODE < 400)     /* CHN */
#define BCMWAPI_WPI
#define BCMWAPI_WAI
#endif /* CONFIG_WLAN_REGION_CODE >= 300 && CONFIG_WLAN_REGION_CODE < 400 */

#if !defined(READ_MACADDR) && !defined(WRITE_MACADDR) && !defined(RDWR_KORICS_MACADDR) \
	&& !defined(RDWR_MACADDR)
#define GET_MAC_FROM_OTP
#define SHOW_NVRAM_TYPE
#endif /* !READ_MACADDR && !WRITE_MACADDR && !RDWR_KORICS_MACADDR && !RDWR_MACADDR */

#endif /* _dhd_sec_feature_h_ */
