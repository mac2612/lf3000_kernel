#DBCMDRIVER -DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE -DCUSTOMER_HW -DDHDTHREAD -DBDC -DTOE -DDHD_BCMEVENTS -DSHOW_EVENTS -DPROP_TXSTATUS -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DBCMSDIOH_TXGLOM -DARP_OFFLOAD_SUPPORT -DWL_WIRELESS_EXT -DWLLXIW -DUSE_IW -DSOFTAP -DBCMFW_ROAM_ENABLE_WEXT -DSDIO_ISR_THREAD -DWIFI_ACT_FRAME


EXTRA_CFLAGS += -DLINUX \
                -DBCMDRIVER \
                -DBCMDONGLEHOST \
		-DUNRELEASEDCHIP \
		-DBCMDMA32 \
		-DBCMFILEIMAGE \
                -DDHDTHREAD \
		-DPROP_TXSTATUS \
		-DBCMSDIO \
                -DBCMWPA2 \
		-DBCMSDIOH_TXGLOM \
		-DARP_OFFLOAD_SUPPORT \
		-DWL_WIRELESS_EXT \
		-DWLLXIW \
		-DUSE_IW \
		-DSOFTAP \
		-DBCMFW_ROAM_ENABLE_WEXT \
		-DSDIO_ISR_THREAD  \
		-DWIFI_ACT_FRAME \
                -DBCMWAPI_WPI \
                -DDHD_GPL \
                -DDHD_SCHED \
                -DSDTEST \
                -DBDC \
                -DTOE \
                -DEMBEDDED_PLATFORM \
                -DDHD_BCMEVENTS \
                -DSHOW_EVENTS \
                -DSDIO_ISR_THREAD \
                -DBCMSDIO \
                -DDHD_GPL \
                -DBCMLXSDMMC \
                -DBCMPLATFORM_BUS \
                -DCC_HAVE_ASM_GOTO \
                -DDOWNLOAD_FIRMWARE_CHECK \
                -DCUSTOMER_HW \
                -DEJECT_SAFE \
                -Idrivers/net/wireless/arcadyan/include

obj-$(CONFIG_ARCADYAN_BCM43143) += dhd.o
dhd-objs := aiutils.o bcmsdh.o \
            bcmsdh_linux.o bcmsdh_sdmmc.o \
            bcmsdh_sdmmc_linux.o bcmutils.o \
            dhd_cdc.o \
            dhd_common.o dhd_custom_gpio.o \
            dhd_linux.o dhd_linux_sched.o \
            dhd_sdio.o hndpmu.o \
            linux_osl.o sbutils.o \
            siutils.o wl_iw.o \
	    dhd_wlfc.o dhd_ip.o \
	    bcmwifi_channels.o bcmevent.o
