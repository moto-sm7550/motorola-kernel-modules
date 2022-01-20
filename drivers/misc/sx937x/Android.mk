DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

ifeq ($(SX937X_USB_CAL),true)
	KERNEL_CFLAGS += CONFIG_SX937X_USB_CAL=y
endif

ifeq ($(SX937X_FLIP_CAL),true)
	KERNEL_CFLAGS += CONFIG_SX937X_FLIP_CAL=y
endif

ifeq ($(SX937X_USB_USE_ONLINE),true)
	KERNEL_CFLAGS += CONFIG_SX937X_POWER_SUPPLY_ONLINE=y
endif

ifeq ($(SX937X_POWER_CONTROL_SUPPORT),true)
	KERNEL_CFLAGS += CONFIG_SX937X_POWER_CONTROL_SUPPORT=y
endif

ifeq ($(SX937X_MTK_KERNEL419_CHARGER_TYPE), true)
    KERNEL_CFLAGS  += CONFIG_SX937X_MTK_KERNEL419_CHARGER_TYPE=y
endif

include $(CLEAR_VARS)
LOCAL_MODULE := sx937x_sar.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/sensors_class.ko
LOCAL_REQUIRED_MODULES := sensors_class.ko
KBUILD_OPTIONS_GKI += GKI_OBJ_MODULE_DIR=gki
include $(DLKM_DIR)/AndroidKernelModule.mk

