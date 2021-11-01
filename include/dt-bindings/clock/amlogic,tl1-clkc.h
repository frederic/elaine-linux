/*
 * include/dt-bindings/clock/tl1-clkc.h
 *
 * Copyright (C) 2018 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef __DT_BINDINGS_TL1_CLKC_H
#define __DT_BINDINGS_TL1_CLKC_H

/*
 * CLKID index values
 */
#define CLKID_SYS_PLL				0
#define CLKID_FIXED_PLL				1
#define CLKID_FCLK_DIV2				2
#define CLKID_FCLK_DIV3				3
#define CLKID_FCLK_DIV4				4
#define CLKID_FCLK_DIV5				5
#define CLKID_FCLK_DIV7				6
#define CLKID_FCLK_DIV2P5			7
#define CLKID_GP0_PLL				8
#define CLKID_GP1_PLL				9
#define CLKID_HIFI_PLL				10
#define CLKID_MPEG_SEL				11
#define CLKID_MPEG_DIV				12
#define CLKID_CLK81				13
#define CLKID_MPLL0				14
#define CLKID_MPLL1				15
#define CLKID_MPLL2				16
#define CLKID_MPLL3				17
#define CLKID_CPU_FCLK_P00			18
#define CLKID_CPU_FCLK_P01			19
#define CLKID_CPU_FCLK_P0			20
#define CLKID_CPU_FCLK_P10			21
#define CLKID_CPU_FCLK_P11			22
#define CLKID_CPU_FCLK_P1			23
#define CLKID_CPU_FCLK_P			24
#define CLKID_CPU_CLK				25
/*#define CLKID_ADC_PLL				24*/

/*HHI_GCLK_MPEG0: 0x50*/
#define GATE_BASE0				(26)

#define CLKID_DDR				(GATE_BASE0 + 0)
#define CLKID_DOS				(GATE_BASE0 + 1)
#define CLKID_ETH_PHY				(GATE_BASE0 + 2)
#define CLKID_ISA				(GATE_BASE0 + 3)
#define CLKID_PL310				(GATE_BASE0 + 4)
#define CLKID_PERIPHS				(GATE_BASE0 + 5)
#define CLKID_SPICC0				(GATE_BASE0 + 6)
#define CLKID_I2C				(GATE_BASE0 + 7)
#define CLKID_SANA				(GATE_BASE0 + 8)
#define CLKID_SMART_CARD			(GATE_BASE0 + 9)
#define CLKID_UART0				(GATE_BASE0 + 10)
#define CLKID_SPICC1				(GATE_BASE0 + 11)
#define CLKID_STREAM				(GATE_BASE0 + 12)
#define CLKID_ASYNC_FIFO			(GATE_BASE0 + 13)
#define CLKID_TVFE				(GATE_BASE0 + 14)
#define CLKID_HIU_REG				(GATE_BASE0 + 15)
#define CLKID_HDMIRX_PCLK			(GATE_BASE0 + 16)
#define CLKID_ATV_DEMOD				(GATE_BASE0 + 17)
#define CLKID_ASSIST_MISC			(GATE_BASE0 + 18)
#define CLKID_SD_EMMC_B				(GATE_BASE0 + 19)
#define CLKID_SD_EMMC_C				(GATE_BASE0 + 20)
#define CLKID_ADEC				(GATE_BASE0 + 21)
#define CLKID_ACODEC				(GATE_BASE0 + 22)
#define CLKID_TCON				(GATE_BASE0 + 23)
#define CLKID_SPI				(GATE_BASE0 + 24)
#define CLKID_DSP				(GATE_BASE0 + 25)

/*HHI_GCLK_MPEG1: 0x51*/
#define GATE_BASE1				(GATE_BASE0 + 26)

#define CLKID_AUDIO				(GATE_BASE1 + 0)
#define CLKID_ETH_CORE				(GATE_BASE1 + 1)
#define CLKID_DEMUX				(GATE_BASE1 + 2)
#define CLKID_AIFIFO				(GATE_BASE1 + 3)
#define CLKID_ADC				(GATE_BASE1 + 4)
#define CLKID_UART1				(GATE_BASE1 + 5)
#define CLKID_G2D				(GATE_BASE1 + 6)
#define CLKID_RESET				(GATE_BASE1 + 7)
#define CLKID_U_PARSER				(GATE_BASE1 + 8)
#define CLKID_USB_GENERAL			(GATE_BASE1 + 9)
#define CLKID_AHB_ARB0				(GATE_BASE1 + 10)

/*HHI_GCLK_MPEG2: 0x52*/
#define GATE_BASE2				(GATE_BASE1 + 11)

#define CLKID_AHB_DATA_BUS			(GATE_BASE2 + 0)
#define CLKID_AHB_CTRL_BUS			(GATE_BASE2 + 1)
#define CLKID_BT656				(GATE_BASE2 + 2)
#define CLKID_USB1_TO_DDR			(GATE_BASE2 + 3)
#define CLKID_MMC_PCLK				(GATE_BASE2 + 4)
#define CLKID_HDCP22_PCLK			(GATE_BASE0 + 5)
#define CLKID_UART2				(GATE_BASE2 + 6)
#define CLKID_TS				(GATE_BASE2 + 7)
#define CLKID_VPU_INTR				(GATE_BASE2 + 8)
#define CLKID_DEMOD_COMB			(GATE_BASE2 + 9)
#define CLKID_GIC				(GATE_BASE2 + 10)

/*HHI_GCLK_OTHER: 0x55*/
#define GATE_OTHER				(GATE_BASE2 + 11)

#define CLKID_VCLK2_VENCI0			(GATE_OTHER + 0)
#define CLKID_VCLK2_VENCI1			(GATE_OTHER + 1)
#define CLKID_VCLK2_VENCP0			(GATE_OTHER + 2)
#define CLKID_VCLK2_VENCP1			(GATE_OTHER + 3)
#define CLKID_VCLK2_VENCT0			(GATE_OTHER + 4)
#define CLKID_VCLK2_VENCT1			(GATE_OTHER + 5)
#define CLKID_VCLK2_OTHER			(GATE_OTHER + 6)
#define CLKID_VCLK2_ENCI			(GATE_OTHER + 7)
#define CLKID_VCLK2_ENCP			(GATE_OTHER + 8)
#define CLKID_DAC_CLK				(GATE_OTHER + 9)
#define CLKID_ENC480P				(GATE_OTHER + 10)
#define CLKID_RNG1				(GATE_OTHER + 11)
#define CLKID_VCLK2_ENCT			(GATE_OTHER + 12)
#define CLKID_VCLK2_ENCL			(GATE_OTHER + 13)
#define CLKID_VCLK2_VENCLMMC			(GATE_OTHER + 14)
#define CLKID_VCLK2_VENCL			(GATE_OTHER + 15)
#define CLKID_VCLK2_OTHER1			(GATE_OTHER + 16)

/*HHI_GCLK_OTHER: 0x55*/
#define GATE_AO_BASE				(GATE_OTHER + 17)

#define CLKID_DMA				(GATE_AO_BASE + 0)
#define CLKID_EFUSE				(GATE_AO_BASE + 1)
#define CLKID_ROM_BOOT				(GATE_AO_BASE + 2)
#define CLKID_RESET_SEC				(GATE_AO_BASE + 3)
#define CLKID_SEC_AHB_APB3			(GATE_AO_BASE + 4)

#define CLOCK_GATE				(GATE_AO_BASE + 5)

#define CLKID_SD_EMMC_B_P0_MUX			(CLOCK_GATE + 0)
#define CLKID_SD_EMMC_B_P0_DIV			(CLOCK_GATE + 1)
#define CLKID_SD_EMMC_B_P0_GATE			(CLOCK_GATE + 2)
#define CLKID_SD_EMMC_B_P0_COMP			(CLOCK_GATE + 3)
#define CLKID_SD_EMMC_C_P0_MUX			(CLOCK_GATE + 4)
#define CLKID_SD_EMMC_C_P0_DIV			(CLOCK_GATE + 5)
#define CLKID_SD_EMMC_C_P0_GATE			(CLOCK_GATE + 6)
#define CLKID_SD_EMMC_C_P0_COMP			(CLOCK_GATE + 7)
#define CLKID_SD_EMMC_B_MUX			(CLOCK_GATE + 8)
#define CLKID_SD_EMMC_B_DIV			(CLOCK_GATE + 9)
#define CLKID_SD_EMMC_B_GATE			(CLOCK_GATE + 10)
#define CLKID_SD_EMMC_B_COMP			(CLOCK_GATE + 11)
#define CLKID_SD_EMMC_C_MUX			(CLOCK_GATE + 12)
#define CLKID_SD_EMMC_C_DIV			(CLOCK_GATE + 13)
#define CLKID_SD_EMMC_C_GATE			(CLOCK_GATE + 14)
#define CLKID_SD_EMMC_C_COMP			(CLOCK_GATE + 15)
#define CLKID_MEDIA_BASE			(CLOCK_GATE + 16)

#define CLKID_VPU_P0_MUX			(CLKID_MEDIA_BASE + 0)
#define CLKID_VPU_P0_DIV			(CLKID_MEDIA_BASE + 1)
#define CLKID_VPU_P0_GATE			(CLKID_MEDIA_BASE + 2)
#define CLKID_VPU_P0_COMP			(CLKID_MEDIA_BASE + 3)
#define CLKID_VPU_P1_MUX			(CLKID_MEDIA_BASE + 4)
#define CLKID_VPU_P1_DIV			(CLKID_MEDIA_BASE + 5)
#define CLKID_VPU_P1_GATE			(CLKID_MEDIA_BASE + 6)
#define CLKID_VPU_P1_COMP			(CLKID_MEDIA_BASE + 7)
#define CLKID_VPU_MUX				(CLKID_MEDIA_BASE + 8)
#define CLKID_VAPB_P0_MUX			(CLKID_MEDIA_BASE + 9)
#define CLKID_VAPB_P0_DIV			(CLKID_MEDIA_BASE + 10)
#define CLKID_VAPB_P0_GATE			(CLKID_MEDIA_BASE + 11)
#define CLKID_VAPB_P0_COMP			(CLKID_MEDIA_BASE + 12)
#define CLKID_VAPB_P1_MUX			(CLKID_MEDIA_BASE + 13)
#define CLKID_VAPB_P1_DIV			(CLKID_MEDIA_BASE + 14)
#define CLKID_VAPB_P1_GATE			(CLKID_MEDIA_BASE + 15)
#define CLKID_VAPB_P1_COMP			(CLKID_MEDIA_BASE + 16)
#define CLKID_VAPB_MUX				(CLKID_MEDIA_BASE + 17)
#define CLKID_GE2D_GATE				(CLKID_MEDIA_BASE + 18)
#define CLKID_VDIN_MEAS_MUX			(CLKID_MEDIA_BASE + 19)
#define CLKID_VDIN_MEAS_DIV			(CLKID_MEDIA_BASE + 20)
#define CLKID_VDIN_MEAS_GATE			(CLKID_MEDIA_BASE + 21)
#define CLKID_VDIN_MEAS_COMP			(CLKID_MEDIA_BASE + 22)

#define CLKID_VPU_CLKB_TMP_MUX			(CLKID_MEDIA_BASE + 23)
#define CLKID_VPU_CLKB_TMP_DIV			(CLKID_MEDIA_BASE + 24)
#define CLKID_VPU_CLKB_TMP_GATE			(CLKID_MEDIA_BASE + 25)
#define CLKID_VPU_CLKB_DIV			(CLKID_MEDIA_BASE + 26)
#define CLKID_VPU_CLKB_GATE			(CLKID_MEDIA_BASE + 27)
#define CLKID_VPU_CLKB_TMP_COMP			(CLKID_MEDIA_BASE + 28)
#define CLKID_VPU_CLKB_COMP			(CLKID_MEDIA_BASE + 29)
#define CLKID_VID_LOCK_COMP			(CLKID_MEDIA_BASE + 30)
#define CLKID_BT656_CLK0_COMP			(CLKID_MEDIA_BASE + 31)
#define CLKID_TCON_PLL_COMP			(CLKID_MEDIA_BASE + 32)
#define CLKID_DEMOD_COMP			(CLKID_MEDIA_BASE + 33)
#define CLKID_ADC_EXTCLK_COMP			(CLKID_MEDIA_BASE + 34)
#define CLKID_VPU_CLKC_P0_COMP			(CLKID_MEDIA_BASE + 35)
#define CLKID_VPU_CLKC_P1_COMP			(CLKID_MEDIA_BASE + 36)
#define CLKID_VPU_CLKC_MUX			(CLKID_MEDIA_BASE + 37)

#define CLKID_HDMIRX_BASE			(CLKID_MEDIA_BASE + 38)

#define CLKID_HDMIRX_CFG_COMP			(CLKID_HDMIRX_BASE + 0)
#define CLKID_HDMIRX_MODET_COMP			(CLKID_HDMIRX_BASE + 1)
#define CLKID_HDMIRX_METER_COMP			(CLKID_HDMIRX_BASE + 2)
#define CLKID_HDMIRX_ACR_COMP			(CLKID_HDMIRX_BASE + 3)
#define CLKID_HDCP22_SKP_COMP			(CLKID_HDMIRX_BASE + 4)
#define CLKID_HDCP22_ESM_COMP			(CLKID_HDMIRX_BASE + 5)
#define CLKID_HDMIRX_AXI_COMP			(CLKID_HDMIRX_BASE + 6)

#define CLKID_VHDEC_BASE			(CLKID_HDMIRX_BASE + 7)

#define CLKID_VDEC_P0_MUX			(CLKID_VHDEC_BASE + 0)
#define CLKID_VDEC_P0_DIV			(CLKID_VHDEC_BASE + 1)
#define CLKID_VDEC_P0_GATE			(CLKID_VHDEC_BASE + 2)
#define CLKID_VDEC_P0_COMP			(CLKID_VHDEC_BASE + 3)
#define CLKID_VDEC_P1_MUX			(CLKID_VHDEC_BASE + 4)
#define CLKID_VDEC_P1_DIV			(CLKID_VHDEC_BASE + 5)
#define CLKID_VDEC_P1_GATE			(CLKID_VHDEC_BASE + 6)
#define CLKID_VDEC_P1_COMP			(CLKID_VHDEC_BASE + 7)
#define CLKID_HCODEC_P0_MUX			(CLKID_VHDEC_BASE + 8)
#define CLKID_HCODEC_P0_DIV			(CLKID_VHDEC_BASE + 9)
#define CLKID_HCODEC_P0_GATE			(CLKID_VHDEC_BASE + 10)
#define CLKID_HCODEC_P0_COMP			(CLKID_VHDEC_BASE + 11)
#define CLKID_HCODEC_P1_MUX			(CLKID_VHDEC_BASE + 12)
#define CLKID_HCODEC_P1_DIV			(CLKID_VHDEC_BASE + 13)
#define CLKID_HCODEC_P1_GATE			(CLKID_VHDEC_BASE + 14)
#define CLKID_HCODEC_P1_COMP			(CLKID_VHDEC_BASE + 15)
#define CLKID_HEVC_P0_MUX			(CLKID_VHDEC_BASE + 16)
#define CLKID_HEVC_P0_DIV			(CLKID_VHDEC_BASE + 17)
#define CLKID_HEVC_P0_GATE			(CLKID_VHDEC_BASE + 18)
#define CLKID_HEVC_P0_COMP			(CLKID_VHDEC_BASE + 19)
#define CLKID_HEVC_P1_MUX			(CLKID_VHDEC_BASE + 20)
#define CLKID_HEVC_P1_DIV			(CLKID_VHDEC_BASE + 21)
#define CLKID_HEVC_P1_GATE			(CLKID_VHDEC_BASE + 22)
#define CLKID_HEVC_P1_COMP			(CLKID_VHDEC_BASE + 23)
#define CLKID_VDEC_MUX				(CLKID_VHDEC_BASE + 24)
#define CLKID_HCODEC_MUX			(CLKID_VHDEC_BASE + 25)
#define CLKID_HEVC_MUX				(CLKID_VHDEC_BASE + 26)
#define CLKID_HEVCF_P0_COMP			(CLKID_VHDEC_BASE + 27)
#define CLKID_HEVCF_P1_COMP			(CLKID_VHDEC_BASE + 28)
#define CLKID_HEVCF_MUX				(CLKID_VHDEC_BASE + 29)


#define CLKID_MISC_BASE				(CLKID_VHDEC_BASE + 30)
#define CLKID_TS_CLK_COMP			(CLKID_MISC_BASE + 0)
#define CLKID_SPICC0_COMP			(CLKID_MISC_BASE + 1)
#define CLKID_SPICC1_COMP			(CLKID_MISC_BASE + 2)

#define CLKID_GPU_BASE				(CLKID_MISC_BASE + 3)
#define CLKID_GPU_P0_COMP			(CLKID_GPU_BASE + 0)
#define CLKID_GPU_P1_COMP			(CLKID_GPU_BASE + 1)
#define CLKID_GPU_MUX				(CLKID_GPU_BASE + 2)
#define CLKID_AO_BASE				(CLKID_GPU_BASE + 3)
#define CLKID_AO_CLK81				(CLKID_AO_BASE + 0)
#define CLKID_SARADC_MUX			(CLKID_AO_BASE + 1)
#define CLKID_SARADC_DIV			(CLKID_AO_BASE + 2)
#define CLKID_SARADC_GATE			(CLKID_AO_BASE + 3)
#define CLKID_AO_AHB_BUS			(CLKID_AO_BASE + 4)
#define CLKID_AO_IR				(CLKID_AO_BASE + 5)
#define CLKID_AO_I2C_MASTER			(CLKID_AO_BASE + 6)
#define CLKID_AO_I2C_SLAVE			(CLKID_AO_BASE + 7)
#define CLKID_AO_UART1				(CLKID_AO_BASE + 8)
#define CLKID_AO_PROD_I2C			(CLKID_AO_BASE + 9)
#define CLKID_AO_UART2				(CLKID_AO_BASE + 10)
#define CLKID_AO_IR_BLASTER			(CLKID_AO_BASE + 11)
#define CLKID_AO_SAR_ADC			(CLKID_AO_BASE + 12)
#define NR_CLKS					(CLKID_AO_BASE + 13)
#endif /* __DT_BINDINGS_TL1_CLKC_H */