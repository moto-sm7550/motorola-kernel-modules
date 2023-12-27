DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(DRM_PANEL_EVENT_NOTIFICATIONS),true)
	KERNEL_CFLAGS += CONFIG_DRM_PANEL_EVENT_NOTIFICATIONS=y
endif

ifeq ($(DRM_PANEL_NOTIFICATIONS),true)
	KERNEL_CFLAGS += CONFIG_DRM_PANEL_NOTIFICATIONS=y
endif

ifeq ($(TOUCHSCREEN_GOODIX_BRL_SPI),true)
	KERNEL_CFLAGS += CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI=y
endif

ifeq ($(GTP_LIMIT_USE_SUPPLIER),true)
	KERNEL_CFLAGS += CONFIG_GTP_LIMIT_USE_SUPPLIER=y
endif

ifeq ($(GTP_ENABLE_PM_QOS),true)
	KERNEL_CFLAGS += CONFIG_GTP_ENABLE_PM_QOS=y
endif

ifeq ($(GOODIX_ESD_ENABLE),true)
	KERNEL_CFLAGS += CONFIG_GOODIX_ESD_ENABLE=y
endif
ifeq ($(findstring factory, $(TARGET_PRODUCT)), factory)
	KERNEL_CFLAGS += CONFIG_TARGET_BUILD_FACTORY=y
endif

ifeq ($(TOUCHSCREEN_FOD),true)
	KERNEL_CFLAGS += CONFIG_GTP_FOD=y
endif

ifeq ($(TOUCHSCREEN_DELAY_RELEASE),true)
	KERNEL_CFLAGS += CONFIG_GTP_DELAY_RELEASE=y
endif

ifeq ($(TOUCHSCREEN_LAST_TIME),true)
	KERNEL_CFLAGS += CONFIG_GTP_LAST_TIME=y
endif

ifeq ($(MTK_PANEL_NOTIFICATIONS),true)
	 KERNEL_CFLAGS += CONFIG_MTK_PANEL_NOTIFICATIONS=y
endif

ifeq ($(GTP_ENABLE_DDA_STYLUS),true)
	KERNEL_CFLAGS += CONFIG_GTP_DDA_STYLUS=y
endif

ifeq ($(ENABLE_TP_GHOST_LOG_CAPTURE),true)
	KERNEL_CFLAGS += CONFIG_GTP_GHOST_LOG_CAPTURE=y
endif

ifeq ($(ENABLE_GTP_PALM_CANCEL),true)
	KBUILD_OPTIONS += CONFIG_ENABLE_GTP_PALM_CANCEL=y
endif

ifeq ($(BOARD_USES_DOUBLE_TAP_CTRL),true)
	KERNEL_CFLAGS += CONFIG_BOARD_USES_DOUBLE_TAP_CTRL=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := goodix_brl_u_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/mmi_info.ko

ifeq ($(GTP_USE_MMI_INFO),true)
	KERNEL_CFLAGS += CONFIG_GTP_USE_MMI_INFO=y
	LOCAL_REQUIRED_MODULES := mmi_info.ko
endif
ifneq ($(findstring touchscreen_mmi.ko,$(PRODUCT_PACKAGES)),)
	KERNEL_CFLAGS += CONFIG_INPUT_TOUCHSCREEN_MMI=y
	LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/touchscreen_mmi.ko
	LOCAL_REQUIRED_MODULES += touchscreen_mmi.ko
endif
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk
