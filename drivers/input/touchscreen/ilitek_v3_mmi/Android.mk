DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifneq ($(BOARD_USES_DOUBLE_TAP),)
	KERNEL_CFLAGS += CONFIG_INPUT_ILI_0FLASH_MMI_ENABLE_DOUBLE_TAP=y
endif

ifneq ($(BOARD_USES_PANEL_NOTIFICATIONS),)
	KERNEL_CFLAGS += CONFIG_INPUT_ILI_PANEL_NOTIFICATIONS=y
endif

ifneq ($(BOARD_USES_MTK_GET_PANEL),)
	KERNEL_CFLAGS += CONFIG_INPUT_ILI_MTK_GET_PANEL=y
endif

ifneq ($(DL_FW_BY_DISPLAY),)
	KERNEL_CFLAGS += CONFIG_ILITEK_RESUME_BY_DDI=y
endif

ifeq ($(CONFIG_INPUT_CHARGER_DETECTION),true)
	KERNEL_CFLAGS += CONFIG_ILITEK_CHARGER=y
endif

ifeq ($(ILITEK_ESD),true)
	KERNEL_CFLAGS += CONFIG_ILITEK_ESD=y
endif

ifeq ($(ILITEK_GESTURE),true)
	KERNEL_CFLAGS += CONFIG_ILITEK_GESTURE=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := ilitek_v3_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk
