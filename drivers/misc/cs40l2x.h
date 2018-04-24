/*
 * cs40l2x.h -- CS40L20/CS40L25/CS40L25A/CS40L25B Haptics Driver
 *
 * Copyright 2018 Cirrus Logic, Inc.
 *
 * Author: Jeff LaBundy <jeff.labundy@cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __CS40L2X_H__
#define __CS40L2X_H__

#include <linux/regmap.h>

#define CS40L2X_FIRSTREG		0x00000000
#define CS40L2X_LASTREG			0x03804FE8
#define CS40L2X_DEVID			0x00000000
#define CS40L2X_REVID			0x00000004
#define CS40L2X_FABID			0x00000008
#define CS40L2X_RELID			0x0000000C
#define CS40L2X_OTPID			0x00000010
#define CS40L2X_SFT_RESET		0x00000020
#define CS40L2X_TEST_KEY_CTL		0x00000040
#define CS40L2X_USER_KEY_CTL		0x00000044
#define CS40L2X_CTRL_ASYNC1		0x00000054
#define CS40L2X_OTP_MEM0		0x00000400
#define CS40L2X_OTP_MEM31		0x0000047C
#define CS40L2X_OTP_CTRL0		0x00000500
#define CS40L2X_OTP_CTRL1		0x00000504
#define CS40L2X_OTP_CTRL3		0x00000508
#define CS40L2X_OTP_CTRL4		0x0000050C
#define CS40L2X_OTP_CTRL5		0x00000510
#define CS40L2X_OTP_CTRL6		0x00000514
#define CS40L2X_OTP_CTRL7		0x00000518
#define CS40L2X_OTP_CTRL8		0x0000051C
#define CS40L2X_PWR_CTRL1		0x00002014
#define CS40L2X_PWR_CTRL2		0x00002018
#define CS40L2X_PWR_CTRL3		0x0000201C
#define CS40L2X_CTRL_OVRRIDE		0x00002020
#define CS40L2X_AMP_OUT_MUTE		0x00002024
#define CS40L2X_PROTECT_REL_ERR_IGN	0x00002034
#define CS40L2X_LRCK_PAD_CONTROL	0x00002418
#define CS40L2X_SDIN_PAD_CONTROL	0x00002420
#define CS40L2X_GPIO_PAD_CONTROL	0x0000242C
#define CS40L2X_JTAG_CONTROL		0x00002438
#define CS40L2X_PWRMGT_CTL		0x00002900
#define CS40L2X_WAKESRC_CTL		0x00002904
#define CS40L2X_PWRMGT_STS		0x00002908
#define CS40L2X_PWRMGT_TEST		0x0000290C
#define CS40L2X_PLL_CLK_CTRL		0x00002C04
#define CS40L2X_DSP_CLK_CTRL		0x00002C08
#define CS40L2X_GLOBAL_CLK_CTRL		0x00002C0C
#define CS40L2X_DATA_FS_SEL		0x00002C10
#define CS40L2X_PLL_MISC_CTRL		0x00003014
#define CS40L2X_MDSYNC_EN		0x00003400
#define CS40L2X_MDSYNC_TX_ID		0x00003408
#define CS40L2X_MDSYNC_PWR_CTRL		0x0000340C
#define CS40L2X_MDSYNC_DATA_TX		0x00003410
#define CS40L2X_MDSYNC_TX_STATUS	0x00003414
#define CS40L2X_MDSYNC_DATA_RX		0x0000341C
#define CS40L2X_MDSYNC_RX_STATUS	0x00003420
#define CS40L2X_MDSYNC_ERR_STATUS	0x00003424
#define CS40L2X_MDSYNC_SYNC_PTE2	0x00003528
#define CS40L2X_MDSYNC_SYNC_PTE3	0x0000352C
#define CS40L2X_MDSYNC_SYNC_MSM_STATUS	0x0000353C
#define CS40L2X_BSTCVRT_VCTRL1		0x00003800
#define CS40L2X_BSTCVRT_VCTRL2		0x00003804
#define CS40L2X_BSTCVRT_PEAK_CUR	0x00003808
#define CS40L2X_BSTCVRT_SFT_RAMP	0x0000380C
#define CS40L2X_BSTCVRT_COEFF		0x00003810
#define CS40L2X_BSTCVRT_SLOPE_LBST	0x00003814
#define CS40L2X_BSTCVRT_SW_FREQ		0x00003818
#define CS40L2X_BSTCVRT_DCM_CTRL	0x0000381C
#define CS40L2X_BSTCVRT_DCM_MODE_FORCE	0x00003820
#define CS40L2X_BSTCVRT_OVERVOLT_CTRL	0x00003830
#define CS40L2X_VI_VOL_POL		0x00004000
#define CS40L2X_DTEMP_WARN_THLD		0x00004220
#define CS40L2X_DTEMP_CFG		0x00004224
#define CS40L2X_DTEMP_EN		0x00004308
#define CS40L2X_VPVBST_FS_SEL		0x00004400
#define CS40L2X_SP_ENABLES		0x00004800
#define CS40L2X_SP_RATE_CTRL		0x00004804
#define CS40L2X_SP_FORMAT		0x00004808
#define CS40L2X_SP_HIZ_CTRL		0x0000480C
#define CS40L2X_SP_FRAME_TX_SLOT	0x00004810
#define CS40L2X_SP_FRAME_RX_SLOT	0x00004820
#define CS40L2X_SP_TX_WL		0x00004830
#define CS40L2X_SP_RX_WL		0x00004840
#define CS40L2X_DAC_PCM1_SRC		0x00004C00
#define CS40L2X_ASP_TX1_SRC		0x00004C20
#define CS40L2X_ASP_TX2_SRC		0x00004C24
#define CS40L2X_ASP_TX3_SRC		0x00004C28
#define CS40L2X_ASP_TX4_SRC		0x00004C2C
#define CS40L2X_DSP1_RX1_SRC		0x00004C40
#define CS40L2X_DSP1_RX2_SRC		0x00004C44
#define CS40L2X_DSP1_RX3_SRC		0x00004C48
#define CS40L2X_DSP1_RX4_SRC		0x00004C4C
#define CS40L2X_DSP1_RX5_SRC		0x00004C50
#define CS40L2X_DSP1_RX6_SRC		0x00004C54
#define CS40L2X_DSP1_RX7_SRC		0x00004C58
#define CS40L2X_DSP1_RX8_SRC		0x00004C5C
#define CS40L2X_NGATE1_SRC		0x00004C60
#define CS40L2X_NGATE2_SRC		0x00004C64
#define CS40L2X_AMP_DIG_VOL_CTRL	0x00006000
#define CS40L2X_VPBR_CFG		0x00006404
#define CS40L2X_VBBR_CFG		0x00006408
#define CS40L2X_VPBR_STATUS		0x0000640C
#define CS40L2X_VBBR_STATUS		0x00006410
#define CS40L2X_OVERTEMP_CFG		0x00006414
#define CS40L2X_AMP_ERR_VOL		0x00006418
#define CS40L2X_VOL_STATUS_TO_DSP	0x00006450
#define CS40L2X_CLASSH_CFG		0x00006800
#define CS40L2X_WKFET_CFG		0x00006804
#define CS40L2X_NG_CFG			0x00006808
#define CS40L2X_AMP_GAIN_CTRL		0x00006C04
#define CS40L2X_DAC_MSM_CFG		0x00007400
#define CS40L2X_IRQ1_CFG		0x00010000
#define CS40L2X_IRQ1_STATUS		0x00010004
#define CS40L2X_IRQ1_STATUS1		0x00010010
#define CS40L2X_IRQ1_STATUS2		0x00010014
#define CS40L2X_IRQ1_STATUS3		0x00010018
#define CS40L2X_IRQ1_STATUS4		0x0001001C
#define CS40L2X_IRQ1_RAW_STATUS1	0x00010090
#define CS40L2X_IRQ1_RAW_STATUS2	0x00010094
#define CS40L2X_IRQ1_RAW_STATUS3	0x00010098
#define CS40L2X_IRQ1_RAW_STATUS4	0x0001009C
#define CS40L2X_IRQ1_MASK1		0x00010110
#define CS40L2X_IRQ1_MASK2		0x00010114
#define CS40L2X_IRQ1_MASK3		0x00010118
#define CS40L2X_IRQ1_MASK4		0x0001011C
#define CS40L2X_IRQ1_FRC1		0x00010190
#define CS40L2X_IRQ1_FRC2		0x00010194
#define CS40L2X_IRQ1_FRC3		0x00010198
#define CS40L2X_IRQ1_FRC4		0x0001019C
#define CS40L2X_IRQ1_EDGE1		0x00010210
#define CS40L2X_IRQ1_EDGE4		0x0001021C
#define CS40L2X_IRQ1_POL1		0x00010290
#define CS40L2X_IRQ1_POL2		0x00010294
#define CS40L2X_IRQ1_POL3		0x00010298
#define CS40L2X_IRQ1_POL4		0x0001029C
#define CS40L2X_IRQ1_DB3		0x00010318
#define CS40L2X_IRQ2_CFG		0x00010800
#define CS40L2X_IRQ2_STATUS		0x00010804
#define CS40L2X_IRQ2_STATUS1		0x00010810
#define CS40L2X_IRQ2_STATUS2		0x00010814
#define CS40L2X_IRQ2_STATUS3		0x00010818
#define CS40L2X_IRQ2_STATUS4		0x0001081C
#define CS40L2X_IRQ2_RAW_STATUS1	0x00010890
#define CS40L2X_IRQ2_RAW_STATUS2	0x00010894
#define CS40L2X_IRQ2_RAW_STATUS3	0x00010898
#define CS40L2X_IRQ2_RAW_STATUS4	0x0001089C
#define CS40L2X_IRQ2_MASK1		0x00010910
#define CS40L2X_IRQ2_MASK2		0x00010914
#define CS40L2X_IRQ2_MASK3		0x00010918
#define CS40L2X_IRQ2_MASK4		0x0001091C
#define CS40L2X_IRQ2_FRC1		0x00010990
#define CS40L2X_IRQ2_FRC2		0x00010994
#define CS40L2X_IRQ2_FRC3		0x00010998
#define CS40L2X_IRQ2_FRC4		0x0001099C
#define CS40L2X_IRQ2_EDGE1		0x00010A10
#define CS40L2X_IRQ2_EDGE4		0x00010A1C
#define CS40L2X_IRQ2_POL1		0x00010A90
#define CS40L2X_IRQ2_POL2		0x00010A94
#define CS40L2X_IRQ2_POL3		0x00010A98
#define CS40L2X_IRQ2_POL4		0x00010A9C
#define CS40L2X_IRQ2_DB3		0x00010B18
#define CS40L2X_GPIO_STATUS1		0x00011000
#define CS40L2X_GPIO1_CTRL1		0x00011008
#define CS40L2X_GPIO2_CTRL1		0x0001100C
#define CS40L2X_GPIO3_CTRL1		0x00011010
#define CS40L2X_GPIO4_CTRL1		0x00011014
#define CS40L2X_MIXER_NGATE_CFG		0x00012000
#define CS40L2X_MIXER_NGATE_CH1_CFG	0x00012004
#define CS40L2X_MIXER_NGATE_CH2_CFG	0x00012008
#define CS40L2X_DSP_MBOX_1		0x00013000
#define CS40L2X_DSP_MBOX_2		0x00013004
#define CS40L2X_DSP_MBOX_3		0x00013008
#define CS40L2X_DSP_MBOX_4		0x0001300C
#define CS40L2X_DSP_MBOX_5		0x00013010
#define CS40L2X_DSP_MBOX_6		0x00013014
#define CS40L2X_DSP_MBOX_7		0x00013018
#define CS40L2X_DSP_MBOX_8		0x0001301C
#define CS40L2X_DSP_VIRT1_MBOX_1	0x00013020
#define CS40L2X_DSP_VIRT1_MBOX_2	0x00013024
#define CS40L2X_DSP_VIRT1_MBOX_3	0x00013028
#define CS40L2X_DSP_VIRT1_MBOX_4	0x0001302C
#define CS40L2X_DSP_VIRT1_MBOX_5	0x00013030
#define CS40L2X_DSP_VIRT1_MBOX_6	0x00013034
#define CS40L2X_DSP_VIRT1_MBOX_7	0x00013038
#define CS40L2X_DSP_VIRT1_MBOX_8	0x0001303C
#define CS40L2X_DSP_VIRT2_MBOX_1	0x00013040
#define CS40L2X_DSP_VIRT2_MBOX_2	0x00013044
#define CS40L2X_DSP_VIRT2_MBOX_3	0x00013048
#define CS40L2X_DSP_VIRT2_MBOX_4	0x0001304C
#define CS40L2X_DSP_VIRT2_MBOX_5	0x00013050
#define CS40L2X_DSP_VIRT2_MBOX_6	0x00013054
#define CS40L2X_DSP_VIRT2_MBOX_7	0x00013058
#define CS40L2X_DSP_VIRT2_MBOX_8	0x0001305C
#define CS40L2X_CLOCK_DETECT_1		0x00014000
#define CS40L2X_TIMER1_CONTROL		0x00015000
#define CS40L2X_TIMER1_COUNT_PRESET	0x00015004
#define CS40L2X_TIMER1_START_STOP	0x0001500C
#define CS40L2X_TIMER1_STATUS		0x00015010
#define CS40L2X_TIMER1_COUNT_READBACK	0x00015014
#define CS40L2X_TIMER1_DSP_CLK_CFG	0x00015018
#define CS40L2X_TIMER1_DSP_CLK_STATUS	0x0001501C
#define CS40L2X_TIMER2_CONTROL		0x00015100
#define CS40L2X_TIMER2_COUNT_PRESET	0x00015104
#define CS40L2X_TIMER2_START_STOP	0x0001510C
#define CS40L2X_TIMER2_STATUS		0x00015110
#define CS40L2X_TIMER2_COUNT_READBACK	0x00015114
#define CS40L2X_TIMER2_DSP_CLK_CFG	0x00015118
#define CS40L2X_TIMER2_DSP_CLK_STATUS	0x0001511C
#define CS40L2X_DFT_JTAG_CONTROL	0x00016000
#define CS40L2X_DIE_STS1		0x00017040
#define CS40L2X_DIE_STS2		0x00017044
#define CS40L2X_TEMP_CAL1		0x00017048
#define CS40L2X_TEMP_CAL2		0x0001704C
#define CS40L2X_DSP1_XMEM_PACK_0	0x02000000
#define CS40L2X_DSP1_XMEM_PACK_3068	0x02002FF0
#define CS40L2X_DSP1_XMEM_UNPACK32_0	0x02400000
#define CS40L2X_DSP1_XMEM_UNPACK32_2046	0x02401FF8
#define CS40L2X_DSP1_TIMESTAMP_COUNT	0x025C0800
#define CS40L2X_DSP1_SYS_ID		0x025E0000
#define CS40L2X_DSP1_SYS_VERSION	0x025E0004
#define CS40L2X_DSP1_SYS_CORE_ID	0x025E0008
#define CS40L2X_DSP1_SYS_AHB_ADDR	0x025E000C
#define CS40L2X_DSP1_SYS_XSRAM_SIZE	0x025E0010
#define CS40L2X_DSP1_SYS_YSRAM_SIZE	0x025E0018
#define CS40L2X_DSP1_SYS_PSRAM_SIZE	0x025E0020
#define CS40L2X_DSP1_SYS_PM_BOOT_SIZE	0x025E0028
#define CS40L2X_DSP1_SYS_FEATURES	0x025E002C
#define CS40L2X_DSP1_SYS_FIR_FILTERS	0x025E0030
#define CS40L2X_DSP1_SYS_LMS_FILTERS	0x025E0034
#define CS40L2X_DSP1_SYS_XM_BANK_SIZE	0x025E0038
#define CS40L2X_DSP1_SYS_YM_BANK_SIZE	0x025E003C
#define CS40L2X_DSP1_SYS_PM_BANK_SIZE	0x025E0040
#define CS40L2X_DSP1_AHBM_WIN0_CTRL0	0x025E2000
#define CS40L2X_DSP1_AHBM_WIN0_CTRL1	0x025E2004
#define CS40L2X_DSP1_AHBM_WIN1_CTRL0	0x025E2008
#define CS40L2X_DSP1_AHBM_WIN1_CTRL1	0x025E200C
#define CS40L2X_DSP1_AHBM_WIN2_CTRL0	0x025E2010
#define CS40L2X_DSP1_AHBM_WIN2_CTRL1	0x025E2014
#define CS40L2X_DSP1_AHBM_WIN3_CTRL0	0x025E2018
#define CS40L2X_DSP1_AHBM_WIN3_CTRL1	0x025E201C
#define CS40L2X_DSP1_AHBM_WIN4_CTRL0	0x025E2020
#define CS40L2X_DSP1_AHBM_WIN4_CTRL1	0x025E2024
#define CS40L2X_DSP1_AHBM_WIN5_CTRL0	0x025E2028
#define CS40L2X_DSP1_AHBM_WIN5_CTRL1	0x025E202C
#define CS40L2X_DSP1_AHBM_WIN6_CTRL0	0x025E2030
#define CS40L2X_DSP1_AHBM_WIN6_CTRL1	0x025E2034
#define CS40L2X_DSP1_AHBM_WIN7_CTRL0	0x025E2038
#define CS40L2X_DSP1_AHBM_WIN7_CTRL1	0x025E203C
#define CS40L2X_DSP1_AHBM_WIN_DBG_CTRL0	0x025E2040
#define CS40L2X_DSP1_AHBM_WIN_DBG_CTRL1	0x025E2044
#define CS40L2X_DSP1_XMEM_UNPACK24_0	0x02800000
#define CS40L2X_DSP1_XMEM_UNPACK24_4093	0x02803FF4
#define CS40L2X_DSP1_CTRL_BASE		0x02B80000
#define CS40L2X_DSP1_CORE_SOFT_RESET	0x02B80010
#define CS40L2X_DSP1_DEBUG		0x02B80040
#define CS40L2X_DSP1_TIMER_CTRL		0x02B80048
#define CS40L2X_DSP1_STREAM_ARB_CTRL	0x02B80050
#define CS40L2X_DSP1_RX1_RATE		0x02B80080
#define CS40L2X_DSP1_RX2_RATE		0x02B80088
#define CS40L2X_DSP1_RX3_RATE		0x02B80090
#define CS40L2X_DSP1_RX4_RATE		0x02B80098
#define CS40L2X_DSP1_RX5_RATE		0x02B800A0
#define CS40L2X_DSP1_RX6_RATE		0x02B800A8
#define CS40L2X_DSP1_RX7_RATE		0x02B800B0
#define CS40L2X_DSP1_RX8_RATE		0x02B800B8
#define CS40L2X_DSP1_TX1_RATE		0x02B80280
#define CS40L2X_DSP1_TX2_RATE		0x02B80288
#define CS40L2X_DSP1_TX3_RATE		0x02B80290
#define CS40L2X_DSP1_TX4_RATE		0x02B80298
#define CS40L2X_DSP1_TX5_RATE		0x02B802A0
#define CS40L2X_DSP1_TX6_RATE		0x02B802A8
#define CS40L2X_DSP1_TX7_RATE		0x02B802B0
#define CS40L2X_DSP1_TX8_RATE		0x02B802B8
#define CS40L2X_DSP1_NMI_CTRL1		0x02B80480
#define CS40L2X_DSP1_NMI_CTRL2		0x02B80488
#define CS40L2X_DSP1_NMI_CTRL3		0x02B80490
#define CS40L2X_DSP1_NMI_CTRL4		0x02B80498
#define CS40L2X_DSP1_NMI_CTRL5		0x02B804A0
#define CS40L2X_DSP1_NMI_CTRL6		0x02B804A8
#define CS40L2X_DSP1_NMI_CTRL7		0x02B804B0
#define CS40L2X_DSP1_NMI_CTRL8		0x02B804B8
#define CS40L2X_DSP1_RESUME_CTRL	0x02B80500
#define CS40L2X_DSP1_IRQ1_CTRL		0x02B80508
#define CS40L2X_DSP1_IRQ2_CTRL		0x02B80510
#define CS40L2X_DSP1_IRQ3_CTRL		0x02B80518
#define CS40L2X_DSP1_IRQ4_CTRL		0x02B80520
#define CS40L2X_DSP1_IRQ5_CTRL		0x02B80528
#define CS40L2X_DSP1_IRQ6_CTRL		0x02B80530
#define CS40L2X_DSP1_IRQ7_CTRL		0x02B80538
#define CS40L2X_DSP1_IRQ8_CTRL		0x02B80540
#define CS40L2X_DSP1_IRQ9_CTRL		0x02B80548
#define CS40L2X_DSP1_IRQ10_CTRL		0x02B80550
#define CS40L2X_DSP1_IRQ11_CTRL		0x02B80558
#define CS40L2X_DSP1_IRQ12_CTRL		0x02B80560
#define CS40L2X_DSP1_IRQ13_CTRL		0x02B80568
#define CS40L2X_DSP1_IRQ14_CTRL		0x02B80570
#define CS40L2X_DSP1_IRQ15_CTRL		0x02B80578
#define CS40L2X_DSP1_IRQ16_CTRL		0x02B80580
#define CS40L2X_DSP1_IRQ17_CTRL		0x02B80588
#define CS40L2X_DSP1_IRQ18_CTRL		0x02B80590
#define CS40L2X_DSP1_IRQ19_CTRL		0x02B80598
#define CS40L2X_DSP1_IRQ20_CTRL		0x02B805A0
#define CS40L2X_DSP1_IRQ21_CTRL		0x02B805A8
#define CS40L2X_DSP1_IRQ22_CTRL		0x02B805B0
#define CS40L2X_DSP1_IRQ23_CTRL		0x02B805B8
#define CS40L2X_DSP1_SCRATCH1		0x02B805C0
#define CS40L2X_DSP1_SCRATCH2		0x02B805C8
#define CS40L2X_DSP1_SCRATCH3		0x02B805D0
#define CS40L2X_DSP1_SCRATCH4		0x02B805D8
#define CS40L2X_DSP1_CCM_CORE_CTRL	0x02BC1000
#define CS40L2X_DSP1_CCM_CLK_OVERRIDE	0x02BC1008
#define CS40L2X_DSP1_XM_MSTR_EN		0x02BC2000
#define CS40L2X_DSP1_XM_CORE_PRI	0x02BC2008
#define CS40L2X_DSP1_XM_AHB_PACK_PL_PRI	0x02BC2010
#define CS40L2X_DSP1_XM_AHB_UP_PL_PRI	0x02BC2018
#define CS40L2X_DSP1_XM_ACCEL_PL0_PRI	0x02BC2020
#define CS40L2X_DSP1_XM_NPL0_PRI	0x02BC2078
#define CS40L2X_DSP1_YM_MSTR_EN		0x02BC20C0
#define CS40L2X_DSP1_YM_CORE_PRI	0x02BC20C8
#define CS40L2X_DSP1_YM_AHB_PACK_PL_PRI	0x02BC20D0
#define CS40L2X_DSP1_YM_AHB_UP_PL_PRI	0x02BC20D8
#define CS40L2X_DSP1_YM_ACCEL_PL0_PRI	0x02BC20E0
#define CS40L2X_DSP1_YM_NPL0_PRI	0x02BC2138
#define CS40L2X_DSP1_PM_MSTR_EN		0x02BC2180
#define CS40L2X_DSP1_PM_PATCH0_ADDR	0x02BC2188
#define CS40L2X_DSP1_PM_PATCH0_EN	0x02BC218C
#define CS40L2X_DSP1_PM_PATCH0_DATA_LO	0x02BC2190
#define CS40L2X_DSP1_PM_PATCH0_DATA_HI	0x02BC2194
#define CS40L2X_DSP1_PM_PATCH1_ADDR	0x02BC2198
#define CS40L2X_DSP1_PM_PATCH1_EN	0x02BC219C
#define CS40L2X_DSP1_PM_PATCH1_DATA_LO	0x02BC21A0
#define CS40L2X_DSP1_PM_PATCH1_DATA_HI	0x02BC21A4
#define CS40L2X_DSP1_PM_PATCH2_ADDR	0x02BC21A8
#define CS40L2X_DSP1_PM_PATCH2_EN	0x02BC21AC
#define CS40L2X_DSP1_PM_PATCH2_DATA_LO	0x02BC21B0
#define CS40L2X_DSP1_PM_PATCH2_DATA_HI	0x02BC21B4
#define CS40L2X_DSP1_PM_PATCH3_ADDR	0x02BC21B8
#define CS40L2X_DSP1_PM_PATCH3_EN	0x02BC21BC
#define CS40L2X_DSP1_PM_PATCH3_DATA_LO	0x02BC21C0
#define CS40L2X_DSP1_PM_PATCH3_DATA_HI	0x02BC21C4
#define CS40L2X_DSP1_PM_PATCH4_ADDR	0x02BC21C8
#define CS40L2X_DSP1_PM_PATCH4_EN	0x02BC21CC
#define CS40L2X_DSP1_PM_PATCH4_DATA_LO	0x02BC21D0
#define CS40L2X_DSP1_PM_PATCH4_DATA_HI	0x02BC21D4
#define CS40L2X_DSP1_PM_PATCH5_ADDR	0x02BC21D8
#define CS40L2X_DSP1_PM_PATCH5_EN	0x02BC21DC
#define CS40L2X_DSP1_PM_PATCH5_DATA_LO	0x02BC21E0
#define CS40L2X_DSP1_PM_PATCH5_DATA_HI	0x02BC21E4
#define CS40L2X_DSP1_PM_PATCH6_ADDR	0x02BC21E8
#define CS40L2X_DSP1_PM_PATCH6_EN	0x02BC21EC
#define CS40L2X_DSP1_PM_PATCH6_DATA_LO	0x02BC21F0
#define CS40L2X_DSP1_PM_PATCH6_DATA_HI	0x02BC21F4
#define CS40L2X_DSP1_PM_PATCH7_ADDR	0x02BC21F8
#define CS40L2X_DSP1_PM_PATCH7_EN	0x02BC21FC
#define CS40L2X_DSP1_PM_PATCH7_DATA_LO	0x02BC2200
#define CS40L2X_DSP1_PM_PATCH7_DATA_HI	0x02BC2204
#define CS40L2X_DSP1_MPU_XM_ACCESS0	0x02BC3000
#define CS40L2X_DSP1_MPU_YM_ACCESS0	0x02BC3004
#define CS40L2X_DSP1_MPU_WNDW_ACCESS0	0x02BC3008
#define CS40L2X_DSP1_MPU_XREG_ACCESS0	0x02BC300C
#define CS40L2X_DSP1_MPU_YREG_ACCESS0	0x02BC3014
#define CS40L2X_DSP1_MPU_XM_ACCESS1	0x02BC3018
#define CS40L2X_DSP1_MPU_YM_ACCESS1	0x02BC301C
#define CS40L2X_DSP1_MPU_WNDW_ACCESS1	0x02BC3020
#define CS40L2X_DSP1_MPU_XREG_ACCESS1	0x02BC3024
#define CS40L2X_DSP1_MPU_YREG_ACCESS1	0x02BC302C
#define CS40L2X_DSP1_MPU_XM_ACCESS2	0x02BC3030
#define CS40L2X_DSP1_MPU_YM_ACCESS2	0x02BC3034
#define CS40L2X_DSP1_MPU_WNDW_ACCESS2	0x02BC3038
#define CS40L2X_DSP1_MPU_XREG_ACCESS2	0x02BC303C
#define CS40L2X_DSP1_MPU_YREG_ACCESS2	0x02BC3044
#define CS40L2X_DSP1_MPU_XM_ACCESS3	0x02BC3048
#define CS40L2X_DSP1_MPU_YM_ACCESS3	0x02BC304C
#define CS40L2X_DSP1_MPU_WNDW_ACCESS3	0x02BC3050
#define CS40L2X_DSP1_MPU_XREG_ACCESS3	0x02BC3054
#define CS40L2X_DSP1_MPU_YREG_ACCESS3	0x02BC305C
#define CS40L2X_DSP1_MPU_XM_VIO_ADDR	0x02BC3100
#define CS40L2X_DSP1_MPU_XM_VIO_STATUS	0x02BC3104
#define CS40L2X_DSP1_MPU_YM_VIO_ADDR	0x02BC3108
#define CS40L2X_DSP1_MPU_YM_VIO_STATUS	0x02BC310C
#define CS40L2X_DSP1_MPU_PM_VIO_ADDR	0x02BC3110
#define CS40L2X_DSP1_MPU_PM_VIO_STATUS	0x02BC3114
#define CS40L2X_DSP1_MPU_LOCK_CONFIG	0x02BC3140
#define CS40L2X_DSP1_MPU_WDT_RST_CTRL	0x02BC3180
#define CS40L2X_DSP1_STRMARB_MSTR0_CFG0	0x02BC5000
#define CS40L2X_DSP1_STRMARB_MSTR0_CFG1	0x02BC5004
#define CS40L2X_DSP1_STRMARB_MSTR0_CFG2	0x02BC5008
#define CS40L2X_DSP1_STRMARB_MSTR1_CFG0	0x02BC5010
#define CS40L2X_DSP1_STRMARB_MSTR1_CFG1	0x02BC5014
#define CS40L2X_DSP1_STRMARB_MSTR1_CFG2	0x02BC5018
#define CS40L2X_DSP1_STRMARB_MSTR2_CFG0	0x02BC5020
#define CS40L2X_DSP1_STRMARB_MSTR2_CFG1	0x02BC5024
#define CS40L2X_DSP1_STRMARB_MSTR2_CFG2	0x02BC5028
#define CS40L2X_DSP1_STRMARB_MSTR3_CFG0	0x02BC5030
#define CS40L2X_DSP1_STRMARB_MSTR3_CFG1	0x02BC5034
#define CS40L2X_DSP1_STRMARB_MSTR3_CFG2	0x02BC5038
#define CS40L2X_DSP1_STRMARB_MSTR4_CFG0	0x02BC5040
#define CS40L2X_DSP1_STRMARB_MSTR4_CFG1	0x02BC5044
#define CS40L2X_DSP1_STRMARB_MSTR4_CFG2	0x02BC5048
#define CS40L2X_DSP1_STRMARB_MSTR5_CFG0	0x02BC5050
#define CS40L2X_DSP1_STRMARB_MSTR5_CFG1	0x02BC5054
#define CS40L2X_DSP1_STRMARB_MSTR5_CFG2	0x02BC5058
#define CS40L2X_DSP1_STRMARB_MSTR6_CFG0	0x02BC5060
#define CS40L2X_DSP1_STRMARB_MSTR6_CFG1	0x02BC5064
#define CS40L2X_DSP1_STRMARB_MSTR6_CFG2	0x02BC5068
#define CS40L2X_DSP1_STRMARB_MSTR7_CFG0	0x02BC5070
#define CS40L2X_DSP1_STRMARB_MSTR7_CFG1	0x02BC5074
#define CS40L2X_DSP1_STRMARB_MSTR7_CFG2	0x02BC5078
#define CS40L2X_DSP1_STRMARB_TX0_CFG0	0x02BC5200
#define CS40L2X_DSP1_STRMARB_TX0_CFG1	0x02BC5204
#define CS40L2X_DSP1_STRMARB_TX1_CFG0	0x02BC5208
#define CS40L2X_DSP1_STRMARB_TX1_CFG1	0x02BC520C
#define CS40L2X_DSP1_STRMARB_TX2_CFG0	0x02BC5210
#define CS40L2X_DSP1_STRMARB_TX2_CFG1	0x02BC5214
#define CS40L2X_DSP1_STRMARB_TX3_CFG0	0x02BC5218
#define CS40L2X_DSP1_STRMARB_TX3_CFG1	0x02BC521C
#define CS40L2X_DSP1_STRMARB_TX4_CFG0	0x02BC5220
#define CS40L2X_DSP1_STRMARB_TX4_CFG1	0x02BC5224
#define CS40L2X_DSP1_STRMARB_TX5_CFG0	0x02BC5228
#define CS40L2X_DSP1_STRMARB_TX5_CFG1	0x02BC522C
#define CS40L2X_DSP1_STRMARB_TX6_CFG0	0x02BC5230
#define CS40L2X_DSP1_STRMARB_TX6_CFG1	0x02BC5234
#define CS40L2X_DSP1_STRMARB_TX7_CFG0	0x02BC5238
#define CS40L2X_DSP1_STRMARB_TX7_CFG1	0x02BC523C
#define CS40L2X_DSP1_STRMARB_RX0_CFG0	0x02BC5400
#define CS40L2X_DSP1_STRMARB_RX0_CFG1	0x02BC5404
#define CS40L2X_DSP1_STRMARB_RX1_CFG0	0x02BC5408
#define CS40L2X_DSP1_STRMARB_RX1_CFG1	0x02BC540C
#define CS40L2X_DSP1_STRMARB_RX2_CFG0	0x02BC5410
#define CS40L2X_DSP1_STRMARB_RX2_CFG1	0x02BC5414
#define CS40L2X_DSP1_STRMARB_RX3_CFG0	0x02BC5418
#define CS40L2X_DSP1_STRMARB_RX3_CFG1	0x02BC541C
#define CS40L2X_DSP1_STRMARB_RX4_CFG0	0x02BC5420
#define CS40L2X_DSP1_STRMARB_RX4_CFG1	0x02BC5424
#define CS40L2X_DSP1_STRMARB_RX5_CFG0	0x02BC5428
#define CS40L2X_DSP1_STRMARB_RX5_CFG1	0x02BC542C
#define CS40L2X_DSP1_STRMARB_RX6_CFG0	0x02BC5430
#define CS40L2X_DSP1_STRMARB_RX6_CFG1	0x02BC5434
#define CS40L2X_DSP1_STRMARB_RX7_CFG0	0x02BC5438
#define CS40L2X_DSP1_STRMARB_RX7_CFG1	0x02BC543C
#define CS40L2X_DSP1_STRMARB_IRQ0_CFG0	0x02BC5600
#define CS40L2X_DSP1_STRMARB_IRQ0_CFG1	0x02BC5604
#define CS40L2X_DSP1_STRMARB_IRQ0_CFG2	0x02BC5608
#define CS40L2X_DSP1_STRMARB_IRQ1_CFG0	0x02BC5610
#define CS40L2X_DSP1_STRMARB_IRQ1_CFG1	0x02BC5614
#define CS40L2X_DSP1_STRMARB_IRQ1_CFG2	0x02BC5618
#define CS40L2X_DSP1_STRMARB_IRQ2_CFG0	0x02BC5620
#define CS40L2X_DSP1_STRMARB_IRQ2_CFG1	0x02BC5624
#define CS40L2X_DSP1_STRMARB_IRQ2_CFG2	0x02BC5628
#define CS40L2X_DSP1_STRMARB_IRQ3_CFG0	0x02BC5630
#define CS40L2X_DSP1_STRMARB_IRQ3_CFG1	0x02BC5634
#define CS40L2X_DSP1_STRMARB_IRQ3_CFG2	0x02BC5638
#define CS40L2X_DSP1_STRMARB_IRQ4_CFG0	0x02BC5640
#define CS40L2X_DSP1_STRMARB_IRQ4_CFG1	0x02BC5644
#define CS40L2X_DSP1_STRMARB_IRQ4_CFG2	0x02BC5648
#define CS40L2X_DSP1_STRMARB_IRQ5_CFG0	0x02BC5650
#define CS40L2X_DSP1_STRMARB_IRQ5_CFG1	0x02BC5654
#define CS40L2X_DSP1_STRMARB_IRQ5_CFG2	0x02BC5658
#define CS40L2X_DSP1_STRMARB_IRQ6_CFG0	0x02BC5660
#define CS40L2X_DSP1_STRMARB_IRQ6_CFG1	0x02BC5664
#define CS40L2X_DSP1_STRMARB_IRQ6_CFG2	0x02BC5668
#define CS40L2X_DSP1_STRMARB_IRQ7_CFG0	0x02BC5670
#define CS40L2X_DSP1_STRMARB_IRQ7_CFG1	0x02BC5674
#define CS40L2X_DSP1_STRMARB_IRQ7_CFG2	0x02BC5678
#define CS40L2X_DSP1_STRMARB_RESYNC_MSK	0x02BC5A00
#define CS40L2X_DSP1_STRMARB_ERR_STATUS	0x02BC5A08
#define CS40L2X_DSP1_INTPCTL_RES_STATIC	0x02BC6000
#define CS40L2X_DSP1_INTPCTL_RES_DYN	0x02BC6004
#define CS40L2X_DSP1_INTPCTL_NMI_CTRL	0x02BC6008
#define CS40L2X_DSP1_INTPCTL_IRQ_INV	0x02BC6010
#define CS40L2X_DSP1_INTPCTL_IRQ_MODE	0x02BC6014
#define CS40L2X_DSP1_INTPCTL_IRQ_EN	0x02BC6018
#define CS40L2X_DSP1_INTPCTL_IRQ_MSK	0x02BC601C
#define CS40L2X_DSP1_INTPCTL_IRQ_FLUSH	0x02BC6020
#define CS40L2X_DSP1_INTPCTL_IRQ_MSKCLR	0x02BC6024
#define CS40L2X_DSP1_INTPCTL_IRQ_FRC	0x02BC6028
#define CS40L2X_DSP1_INTPCTL_IRQ_MSKSET	0x02BC602C
#define CS40L2X_DSP1_INTPCTL_IRQ_ERR	0x02BC6030
#define CS40L2X_DSP1_INTPCTL_IRQ_PEND	0x02BC6034
#define CS40L2X_DSP1_INTPCTL_IRQ_GEN	0x02BC6038
#define CS40L2X_DSP1_INTPCTL_TESTBITS	0x02BC6040
#define CS40L2X_DSP1_WDT_CONTROL	0x02BC7000
#define CS40L2X_DSP1_WDT_STATUS		0x02BC7008
#define CS40L2X_DSP1_YMEM_PACK_0	0x02C00000
#define CS40L2X_DSP1_YMEM_PACK_1532	0x02C017F0
#define CS40L2X_DSP1_YMEM_UNPACK32_0	0x03000000
#define CS40L2X_DSP1_YMEM_UNPACK32_1022	0x03000FF8
#define CS40L2X_DSP1_YMEM_UNPACK24_0	0x03400000
#define CS40L2X_DSP1_YMEM_UNPACK24_2045	0x03401FF4
#define CS40L2X_DSP1_PMEM_0		0x03800000
#define CS40L2X_DSP1_PMEM_5114		0x03804FE8

/* trim fields unpacked from OTP memory */
#define CS40L2X_OTP_TRIM_1		0x0000208C
#define CS40L2X_OTP_TRIM_2		0x00002090
#define CS40L2X_OTP_TRIM_3		0x00003010
#define CS40L2X_OTP_TRIM_4		0x0000300C
#define CS40L2X_OTP_TRIM_5		0x0000394C
#define CS40L2X_OTP_TRIM_6		0x00003950
#define CS40L2X_OTP_TRIM_7		0x00003954
#define CS40L2X_OTP_TRIM_8		0x00003958
#define CS40L2X_OTP_TRIM_9		0x0000395C
#define CS40L2X_OTP_TRIM_10		0x0000416C
#define CS40L2X_OTP_TRIM_11		0x00004160
#define CS40L2X_OTP_TRIM_12		0x00004170
#define CS40L2X_OTP_TRIM_13		0x00004360
#define CS40L2X_OTP_TRIM_14		0x00004448
#define CS40L2X_OTP_TRIM_15		0x0000444C
#define CS40L2X_OTP_TRIM_16		0x00006E30
#define CS40L2X_OTP_TRIM_17		0x00006E34
#define CS40L2X_OTP_TRIM_18		0x00006E38
#define CS40L2X_OTP_TRIM_19		0x00006E3C
#define CS40L2X_OTP_TRIM_20		0x00006E40
#define CS40L2X_OTP_TRIM_21		0x00006E44
#define CS40L2X_OTP_TRIM_22		0x00006E48
#define CS40L2X_OTP_TRIM_23		0x00006E4C
#define CS40L2X_OTP_TRIM_24		0x00006E50
#define CS40L2X_OTP_TRIM_25		0x00006E54
#define CS40L2X_OTP_TRIM_26		0x00006E58
#define CS40L2X_OTP_TRIM_27		0x00006E5C
#define CS40L2X_OTP_TRIM_28		0x00006E60
#define CS40L2X_OTP_TRIM_29		0x00006E64
#define CS40L2X_OTP_TRIM_30		0x00007418
#define CS40L2X_OTP_TRIM_31		0x0000741C
#define CS40L2X_OTP_TRIM_32		0x00007434
#define CS40L2X_OTP_TRIM_33		0x00007068
#define CS40L2X_OTP_TRIM_34		0x0000410C
#define CS40L2X_OTP_TRIM_35		0x0000400C
#define CS40L2X_OTP_TRIM_36		0x00002030

#define CS40L2X_GLOBAL_EN_MASK		0x00000001
#define CS40L2X_GLOBAL_EN_SHIFT		0

#define CS40L2X_GP2_CTRL_MASK		0x07000000
#define CS40L2X_GP2_CTRL_SHIFT		24
#define CS40L2X_GP2_CTRL_MCLK		0x3

#define CS40L2X_MEM_RDY_MASK		0x00000002
#define CS40L2X_MEM_RDY_SHIFT		1

#define CS40L2X_PLL_FORCE_EN_MASK	0x00010000
#define CS40L2X_PLL_FORCE_EN_SHIFT	16

#define CS40L2X_PLL_REFCLK_SEL_MASK	0x00000007
#define CS40L2X_PLL_REFCLK_SEL_SHIFT	0
#define CS40L2X_PLL_REFCLK_SEL_MCLK	0x5

#define CS40L2X_DAC_PCM1_SRC_MASK	0x7F
#define CS40L2X_DAC_PCM1_SRC_SHIFT	0
#define CS40L2X_DAC_PCM1_SRC_DSP1TX1	0x32

#define CS40L2X_DSP1_RXn_SRC_MASK	0x7F
#define CS40L2X_DSP1_RXn_SRC_SHIFT	0
#define CS40L2X_DSP1_RXn_SRC_VMON	0x18
#define CS40L2X_DSP1_RXn_SRC_IMON	0x19

#define CS40L2X_DSP1_RESET_MASK		0x200
#define CS40L2X_DSP1_RESET_SHIFT	9

#define CS40L2X_DSP1_EN_MASK		0x1
#define CS40L2X_DSP1_EN_SHIFT		0

#define CS40L2X_BST_IPK_MASK		0x0000007F
#define CS40L2X_BST_IPK_SHIFT		0

#define CS40L2X_BST_K1_MASK		0x000000FF
#define CS40L2X_BST_K1_SHIFT		0
#define CS40L2X_BST_K2_MASK		0x0000FF00
#define CS40L2X_BST_K2_SHIFT		8

#define CS40L2X_BST_SLOPE_MASK		0x0000FF00
#define CS40L2X_BST_SLOPE_SHIFT		8

#define CS40L2X_BST_LBST_VAL_MASK	0x00000003
#define CS40L2X_BST_LBST_VAL_SHIFT	0

#define CS40L2X_AMP_VOL_PCM_MASK	0x00003FF8
#define CS40L2X_AMP_VOL_PCM_SHIFT	3

#define CS40L2X_OTP_BOOT_ERR		0x80000000
#define CS40L2X_OTP_BOOT_DONE		0x00000002

#define CS40L2X_DEVID_L20		0x035A40
#define CS40L2X_DEVID_L25		0x40A250
#define CS40L2X_DEVID_L25A		0x40A25A
#define CS40L2X_DEVID_L25B		0x40A25B

#define CS40L2X_REVID_A0		0xA0
#define CS40L2X_REVID_B0		0xB0
#define CS40L2X_REVID_B1		0xB1

#define CS40L2X_MPU_UNLOCK_CODE1	0x5555
#define CS40L2X_MPU_UNLOCK_CODE2	0xAAAA

#define CS40L2X_XM_UNPACKED_TYPE	0x05
#define CS40L2X_YM_UNPACKED_TYPE	0x06
#define CS40L2X_PM_PACKED_TYPE		0x10
#define CS40L2X_XM_PACKED_TYPE		0x11
#define CS40L2X_YM_PACKED_TYPE		0x12
#define CS40L2X_ALGO_INFO_TYPE		0xF2

#define CS40L2X_FW_FILE_HEADER_SIZE	40
#define CS40L2X_FW_DBLK_OFFSET_SIZE	3
#define CS40L2X_FW_DBLK_TYPE_SIZE	1
#define CS40L2X_FW_DBLK_LENGTH_SIZE	4

#define CS40L2X_WT_FILE_HEADER_SIZE	16
#define CS40L2X_WT_DBLK_OFFSET_SIZE	2
#define CS40L2X_WT_DBLK_TYPE_SIZE	2
#define CS40L2X_WT_ALGO_ID_SIZE		4
#define CS40L2X_WT_ALGO_REV_SIZE	4
#define CS40L2X_WT_SAMPLE_RATE_SIZE	4
#define CS40L2X_WT_DBLK_LENGTH_SIZE	4

#define CS40L2X_XM_FW_ID		(CS40L2X_DSP1_XMEM_UNPACK24_0 + 12)
#define CS40L2X_XM_NUM_ALGOS		(CS40L2X_DSP1_XMEM_UNPACK24_0 + 36)
#define CS40L2X_ALGO_ID_OFFSET		0
#define CS40L2X_ALGO_REV_OFFSET		4
#define CS40L2X_ALGO_XM_BASE_OFFSET	8
#define CS40L2X_ALGO_XM_SIZE_OFFSET	12
#define CS40L2X_ALGO_YM_BASE_OFFSET	16
#define CS40L2X_ALGO_YM_SIZE_OFFSET	20
#define CS40L2X_ALGO_ENTRY_SIZE		24
#define CS40L2X_ALGO_LIST_TERM		0xBEDEAD
#define CS40L2X_ALGO_REV_MASK		0xFF0000

#define CS40L2X_NUM_ALGOS_MAX		16
#define CS40L2X_FW_REV_MIN		0x030000

#define CS40L2X_ALGO_ID_SIZE		4
#define CS40L2X_COEFF_COUNT_SIZE	4
#define CS40L2X_COEFF_OFFSET_SIZE	2
#define CS40L2X_COEFF_TYPE_SIZE		2
#define CS40L2X_COEFF_LENGTH_SIZE	4

#define CS40L2X_TIMEOUT_MS_MAX		0x02AAAA  /* ~174 sec */
#define CS40L2X_HALO_STATE_RUNNING	0x2
#define CS40L2X_STATUS_IDLE		0xFFFFFF

#define CS40L2X_MBOX_TRIGGERINDEX	CS40L2X_DSP_VIRT1_MBOX_1
#define CS40L2X_MBOX_TRIGGER_MS		CS40L2X_DSP_VIRT1_MBOX_2
#define CS40L2X_MBOX_STIMULUS_MODE	CS40L2X_DSP_VIRT1_MBOX_3

#define CS40L2X_INDEX_MASK		0x7FFF
#define CS40L2X_INDEX_VIBE		0x0000
#define CS40L2X_INDEX_CLICK_MIN		0x0001
#define CS40L2X_INDEX_CLICK_MAX		0x7FFF
#define CS40L2X_INDEX_CONT_MIN		0x8000
#define CS40L2X_INDEX_CONT_MAX		0xFFFD
#define CS40L2X_INDEX_PBQ		0xFFFE
#define CS40L2X_INDEX_DIAG		0xFFFF

#define CS40L2X_PBQ_SEG_LEN_MAX		10
#define CS40L2X_PBQ_DEPTH_MAX		256
#define CS40L2X_PBQ_SCALE_MAX		100
#define CS40L2X_PBQ_DELAY_MAX		10000
#define CS40L2X_PBQ_REPEAT_MAX		16
#define CS40L2X_PBQ_POLL_FAST_NS	5000000
#define CS40L2X_PBQ_POLL_SLOW_NS	25000000
#define CS40L2X_PBQ_STATE_IDLE		0x00
#define CS40L2X_PBQ_STATE_PLAYING	0x01
#define CS40L2X_PBQ_STATE_SILENT	0x02

#define CS40L2X_DIAG_STATE_INIT		0x00
#define CS40L2X_DIAG_STATE_RUN		0x01
#define CS40L2X_DIAG_STATE_DONE		0x02
#define CS40L2X_DIAG_STATE_DELAY_MS	500

#define CS40L2X_DSP_TIMEOUT_COUNT	10	/* 1 count = 10 ms */
#define CS40L2X_OTP_TIMEOUT_COUNT	10
#define CS40L2X_NUM_OTP_MAPS		2
#define CS40L2X_NUM_OTP_WORDS		32

#define CS40L2X_TEST_KEY_UNLOCK_CODE1	0x55
#define CS40L2X_TEST_KEY_UNLOCK_CODE2	0xAA
#define CS40L2X_TEST_KEY_RELOCK_CODE1	0xCC
#define CS40L2X_TEST_KEY_RELOCK_CODE2	0x33

#define CS40L2X_DIG_SCALE_MAX		816	/* -102 dB */
#define CS40L2X_DIG_SCALE_ZERO		0x800
#define CS40L2X_DIG_SCALE_MASK		0x7FF

#define CS40L2X_MAX_WLEN		4096

#define CS40L2X_DEVICE_NAME		"vibrator"

#define CS40L2X_FW_NAME_A0		"cs40l20.wmfw"
#define CS40L2X_FW_NAME_B1		"cs40l25a.wmfw"

#define CS40L2X_WT_NAME			"cs40l20.bin"

bool cs40l2x_readable_reg(struct device *dev, unsigned int reg);
bool cs40l2x_precious_reg(struct device *dev, unsigned int reg);

struct cs40l2x_trim {
	unsigned int reg;
	unsigned char shift;
	unsigned char size;
};

struct cs40l2x_otp_desc {
	unsigned int id;
	unsigned char row_start;
	unsigned char col_start;
	unsigned int num_trims;
	const struct cs40l2x_trim *trim_table;
};

struct cs40l2x_coeff_desc {
	unsigned int parent_id;
	unsigned int block_offset;
	unsigned int block_type;
	unsigned char name[32];
	unsigned int reg;
	struct list_head list;
};

struct cs40l2x_pbq_pair {
	unsigned int tag;
	unsigned int mag;
};

struct cs40l2x_dig_scale_map {
	unsigned int encoded;
	unsigned int dig_scale;
};

extern const unsigned char cs40l2x_bst_k1_table[4][5];
extern const unsigned char cs40l2x_bst_k2_table[4][5];
extern const unsigned char cs40l2x_bst_slope_table[4];

extern const struct cs40l2x_otp_desc cs40l2x_otp_map[CS40L2X_NUM_OTP_MAPS];

extern const struct cs40l2x_dig_scale_map
			cs40l2x_pbq_scale[CS40L2X_PBQ_SCALE_MAX + 1];

#endif /*__CS40L2X_H__*/
