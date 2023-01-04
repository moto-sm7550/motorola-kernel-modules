DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifneq ($(BOARD_USES_DOUBLE_TAP),)
	KERNEL_CFLAGS += CONFIG_INPUT_ILI_0FLASH_MMI_ENABLE_DOUBLE_TAP=y
endif

ifneq ($(BOARD_USES_PANEL_NOTIFICATIONS),)
	KERNEL_CFLAGS += CONFIG_INPUT_ILI_PANEL_NOTIFICATIONS=y
endif

ifneq ($(MOTO_PANEL_TOUCH_GESTURE_STATUS),)
	KERNEL_CFLAGS += CONFIG_INPUT_ILI_PANEL_TP_GESTURE_STATUS=y
endif

ifneq ($(BOARD_USES_MTK_GET_PANEL),)
	KERNEL_CFLAGS += CONFIG_INPUT_ILI_MTK_GET_PANEL=y
endif

ifneq ($(CONFIG_INPUT_CHECK_PANEL),)
	KERNEL_CFLAGS += CONFIG_INPUT_ILI_MTK_CHECK_PANEL=y
endif

ifneq ($(DL_FW_BY_DISPLAY),)
	KERNEL_CFLAGS += CONFIG_ILITEK_RESUME_BY_DDI=y
endif

ifeq ($(ILITEK_FW_PANEL),true)
        KERNEL_CFLAGS += CONFIG_ILITEK_FW_PANEL=y
endif

ifneq ($(BOARD_USES_PEN_NOTIFIER),)
	KERNEL_CFLAGS += CONFIG_INPUT_ILITEK_MMI_PEN_NOTIFIER=y
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

# enable gesture mode by panel config
# not all panels need enable gesture mode for some products
ifeq ($(ILITEK_PANEL_GESTURE),true)
	KERNEL_CFLAGS += CONFIG_ILITEK_PANEL_GESTURE=y
endif

ifeq ($(ILITEK_PASSIVE_PEN),true)
	KERNEL_CFLAGS += CONFIG_ILITEK_PASSIVE_PEN=y
endif

ifeq ($(GTP_ENABLE_DDA_STYLUS),true)
	KBUILD_OPTIONS += CONFIG_GTP_DDA_STYLUS=y
endif

include $(CLEAR_VARS)

ifneq ($(BOARD_USES_PEN_NOTIFIER),)
LOCAL_ADDITIONAL_DEPENDENCIES += $(KERNEL_MODULES_OUT)/bu520xx_pen.ko
endif

LOCAL_MODULE := ili9882_mmi.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
