DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)


ifeq ($(SEC_NFC_PRODUCT_N3), true)
	KERNEL_CFLAGS += CONFIG_SEC_NFC_PRODUCT_N3=y
else
	KERNEL_CFLAGS += CONFIG_SEC_NFC_PRODUCT_N5=y
endif
KERNEL_CFLAGS += CONFIG_SEC_NFC_PRODUCT_N5=y
KERNEL_CFLAGS += CONFIG_SEC_NFC_IF_I2C=y

include $(CLEAR_VARS)
LOCAL_MODULE := sec_nfc.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
LOCAL_ADDITIONAL_DEPENDENCIES := $(KERNEL_MODULES_OUT)/mmi_info.ko
include $(DLKM_DIR)/AndroidKernelModule.mk

