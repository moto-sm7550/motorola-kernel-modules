/*
 * Copyright (C) 2023 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __GOODIX_TS_MMI_H__
#define __GOODIX_TS_MMI_H__

#include <linux/platform_device.h>
#include <linux/touchscreen_mmi.h>
#include <linux/mmi_wake_lock.h>
#include "goodix_ts_core.h"

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
int goodix_ts_mmi_dev_register(struct platform_device *ts_device);
void goodix_ts_mmi_dev_unregister(struct platform_device *ts_device);
#else
static int inline goodix_ts_mmi_dev_register(struct platform_device *ts_device) {
	return -ENOSYS;
}
static int inline goodix_ts_mmi_dev_unregister(struct platform_device *ts_device) {
	return -ENOSYS;
}
#endif

#define ENTER_STOWED_MODE_CMD   0x97
#define CHARGER_MODE_CMD    0xAF

#endif
