/***************************************************************************//**
 *   @file   adpd188.h
 *   @brief  ADPD188 driver header file.
 *   @author Andrei Drimbarean (Andrei.Drimbarean@analog.com)
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef ADPD188_H_
#define ADPD188_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "no_os_i2c.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define ADPD188_REG_STATUS		0x00
#define ADPD188_REG_INT_MASK		0x01
#define ADPD188_REG_GPIO_DRV		0x02
#define ADPD188_REG_BG_STATUS		0x04
#define ADPD188_REG_FIFO_THRESH		0x06
#define ADPD188_REG_DEVID		0x08
#define ADPD188_REG_I2CS_ID		0x09
#define ADPD188_REG_CLK_RATIO		0x0A
#define ADPD188_REG_GPIO_CTRL		0x0B
#define ADPD188_REG_SLAVE_ADDR_KEY	0x0D
#define ADPD188_REG_SW_RESET		0x0F
#define ADPD188_REG_MODE		0x10
#define ADPD188_REG_SLOT_EN		0x11
#define ADPD188_REG_FSAMPLE		0x12
#define ADPD188_REG_PD_LED_SELECT	0x14
#define ADPD188_REG_NUM_AVG		0x15
#define ADPD188_REG_BG_MEAS_A		0x16
#define ADPD188_REG_INT_SEQ_A		0x17
#define ADPD188_REG_SLOTA_CH1_OFFSET	0x18
#define ADPD188_REG_SLOTA_CH2_OFFSET	0x19
#define ADPD188_REG_SLOTA_CH3_OFFSET	0x1A
#define ADPD188_REG_SLOTA_CH4_OFFSET	0x1B
#define ADPD188_REG_BG_MEAS_B		0x1C
#define ADPD188_REG_INT_SEQ_B		0x1D
#define ADPD188_REG_SLOTB_CH1_OFFSET	0x1E
#define ADPD188_REG_SLOTB_CH2_OFFSET	0x1F
#define ADPD188_REG_SLOTB_CH3_OFFSET	0x20
#define ADPD188_REG_SLOTB_CH4_OFFSET	0x21
#define ADPD188_REG_ILED3_COARSE	0x22
#define ADPD188_REG_ILED1_COARSE	0x23
#define ADPD188_REG_ILED2_COARSE	0x24
#define ADPD188_REG_ILED_FINE		0x25
#define ADPD188_REG_SLOTA_LED_PULSE	0x30
#define ADPD188_REG_SLOTA_NUMPULSES	0x31
#define ADPD188_REG_LED_DISABLE		0x34
#define ADPD188_REG_SLOTB_LED_PULSE	0x35
#define ADPD188_REG_SLOTB_NUMPULSES	0x36
#define ADPD188_REG_ALT_PWR_DN		0x37
#define ADPD188_REG_EXT_SYNC_STARTUP	0x38
#define ADPD188_REG_SLOTA_AFE_WINDOW	0x39
#define ADPD188_REG_SLOTB_AFE_WINDOW	0x3B
#define ADPD188_REG_AFE_PWR_CFG1	0x3C
#define ADPD188_REG_SLOTA_FLOAT_LED	0x3E
#define ADPD188_REG_SLOTB_FLOAT_LED	0x3F
#define ADPD188_REG_SLOTA_TIA_CFG	0x42
#define ADPD188_REG_SLOTA_AFE_CFG	0x43
#define ADPD188_REG_SLOTB_TIA_CFG	0x44
#define ADPD188_REG_SLOTB_AFE_CFG	0x45
#define ADPD188_REG_SAMPLE_CLK		0x4B
#define ADPD188_REG_CLK32M_ADJUST	0x4D
#define ADPD188_REG_EXT_SYNC_SEL	0x4F
#define ADPD188_REG_CLK32M_CAL_EN	0x50
#define ADPD188_REG_AFE_PWR_CFG2	0x54
#define ADPD188_REG_TIA_INDEP_GAIN	0x55
#define ADPD188_REG_MATH		0x58
#define ADPD188_REG_FLT_CONFIG_B	0x59
#define ADPD188_REG_FLT_LED_FIRE	0x5A
#define ADPD188_REG_FLT_CONFIG_A	0x5E
#define ADPD188_REG_DATA_ACCESS_CTL	0x5F
#define ADPD188_REG_FIFO_ACCESS		0x60
#define ADPD188_REG_SLOTA_PD1_16BIT	0x64
#define ADPD188_REG_SLOTA_PD2_16BIT	0x65
#define ADPD188_REG_SLOTA_PD3_16BIT	0x66
#define ADPD188_REG_SLOTA_PD4_16BIT	0x67
#define ADPD188_REG_SLOTB_PD1_16BIT	0x68
#define ADPD188_REG_SLOTB_PD2_16BIT	0x69
#define ADPD188_REG_SLOTB_PD3_16BIT	0x6A
#define ADPD188_REG_SLOTB_PD4_16BIT	0x6B
#define ADPD188_REG_A_PD1_LOW		0x70
#define ADPD188_REG_A_PD2_LOW		0x71
#define ADPD188_REG_A_PD3_LOW		0x72
#define ADPD188_REG_A_PD4_LOW		0x73
#define ADPD188_REG_A_PD1_HIGH		0x74
#define ADPD188_REG_A_PD2_HIGH		0x75
#define ADPD188_REG_A_PD3_HIGH		0x76
#define ADPD188_REG_A_PD4_HIGH		0x77
#define ADPD188_REG_B_PD1_LOW		0x78
#define ADPD188_REG_B_PD2_LOW		0x79
#define ADPD188_REG_B_PD3_LOW		0x7A
#define ADPD188_REG_B_PD4_LOW		0x7B
#define ADPD188_REG_B_PD1_HIGH		0x7C
#define ADPD188_REG_B_PD2_HIGH		0x7D
#define ADPD188_REG_B_PD3_HIGH		0x7E
#define ADPD188_REG_B_PD4_HIGH		0x7F

/* ADPD188_REG_STATUS */
#define ADPD188_STATUS_FIFO_SAMPLES_MASK	0xFF00
#define ADPD188_STATUS_SLOTB_INT_MASK		0x0040
#define ADPD188_STATUS_SLOTA_INT_MASK		0x0020
#define ADPD188_STATUS_FIFO_SAMPLES_POS		8
#define ADPD188_STATUS_SLOTB_INT_POS		6
#define ADPD188_STATUS_SLOTA_INT_POS		5

/* ADPD188_REG_INT_MASK */
#define ADPD188_INT_MASK_FIFO_INT_MASK_MASK	0x0100
#define ADPD188_INT_MASK_SLOTB_INT_MASK_MASK	0x0040
#define ADPD188_INT_MASK_SLOTA_INT_MASK_MASK	0x0020
#define ADPD188_INT_MASK_FIFO_INT_MASK_POS	8
#define ADPD188_INT_MASK_SLOTB_INT_MASK_POS	6
#define ADPD188_INT_MASK_SLOTA_INT_MASK_POS	5

/* ADPD188_REG_GPIO_DRV */
#define ADPD188_GPIO_DRV_GPIO1_DRV_MASK	0x0200
#define ADPD188_GPIO_DRV_GPIO1_POL_MASK	0x0100
#define ADPD188_GPIO_DRV_GPIO0_ENA_MASK	0x0004
#define ADPD188_GPIO_DRV_GPIO0_DRV_MASK	0x0002
#define ADPD188_GPIO_DRV_GPIO0_POL_MASK	0x0001
#define ADPD188_GPIO_DRV_GPIO1_DRV_POS	9
#define ADPD188_GPIO_DRV_GPIO1_POL_POS	8
#define ADPD188_GPIO_DRV_GPIO0_ENA_POS	2
#define ADPD188_GPIO_DRV_GPIO0_DRV_POS	1
#define ADPD188_GPIO_DRV_GPIO0_POL_POS	0

/* ADPD188_REG_BG_STATUS */
#define ADPD188_BG_STATUS_BG_STATUS_B_MASK	0x00F0
#define ADPD188_BG_STATUS_BG_STATUS_A_MASK	0x000F
#define ADPD188_BG_STATUS_BG_STATUS_B_POS	4
#define ADPD188_BG_STATUS_BG_STATUS_A_POS	0

/* ADPD188_REG_FIFO_THRESH */
#define ADPD188_FIFO_THRESH_FIFO_THRESH_MASK	0x3F00
#define ADPD188_FIFO_THRESH_FIFO_THRESH_POS	8
#define ADPD188_FIFO_THRESH_MAX_THRESHOLD	63

/* ADPD188_REG_DEVID */
#define ADPD188_DEVID_REV_NUM_MASK	0xFF00
#define ADPD188_DEVID_DEV_ID_MASK	0x00FF
#define ADPD188_DEVID_REV_NUM_POS	8
#define ADPD188_DEVID_DEV_ID_POS	0
/* ADPD188BI specific */
#define ADPD188_DEVICE_ID		0x0916
/* ADPD108X specific */
#define ADPD108X_DEVICE_ID		0x0A16

/* ADPD188_REG_I2CS_ID */
#define ADPD188_I2CS_ID_ADDRESS_WRITE_KEY_MASK	0xFF00
#define ADPD188_I2CS_ID_SLAVE_ADDRESS_MASK	0x00FE
#define ADPD188_I2CS_ID_ADDRESS_WRITE_KEY_POS	8
#define ADPD188_I2CS_ID_SLAVE_ADDRESS_POS	1

/* ADPD188_REG_CLK_RATIO */
#define ADPD188_CLK_RATIO_CLK_RATIO_MASK	0x0FFF
#define ADPD188_CLK_RATIO_CLK_RATIO_POS		0

/* ADPD188_REG_GPIO_CTRL */
#define ADPD188_GPIO_CTRL_GPIO1_ALT_CFG_MASK	0x1F00
#define ADPD188_GPIO_CTRL_GPIO0_ALT_CFG_MASK	0x001F
#define ADPD188_GPIO_CTRL_GPIO1_ALT_CFG_POS	8
#define ADPD188_GPIO_CTRL_GPIO0_ALT_CFG_POS	0

/* ADPD188_REG_SW_RESET */
#define ADPD188_SW_RESET_SW_RESET_MASK	0x0001
#define ADPD188_SW_RESET_SW_RESET_POS	0

/* ADPD188_REG_MODE */
#define ADPD188_MODE_MODE_MASK	0x0003
#define ADPD188_MODE_MODE_POS	0

/* ADPD188_REG_SLOT_EN */
#define ADPD188_SLOT_EN_RDOUT_MODE_MASK		0x2000
#define ADPD188_SLOT_EN_FIFO_OVRN_PREVENT_MASK	0x1000
#define ADPD188_SLOT_EN_SLOTB_FIFO_MODE_MASK	0x01C0
#define ADPD188_SLOT_EN_SLOTB_EN_MASK		0x0020
#define ADPD188_SLOT_EN_SLOTA_FIFO_MODE_MASK	0x001C
#define ADPD188_SLOT_EN_SLOTA_EN_MASK		0x0001
#define ADPD188_SLOT_EN_RDOUT_MODE_POS		13
#define ADPD188_SLOT_EN_FIFO_OVRN_PREVENT_POS	12
#define ADPD188_SLOT_EN_SLOTB_FIFO_MODE_POS	6
#define ADPD188_SLOT_EN_SLOTB_EN_POS		5
#define ADPD188_SLOT_EN_SLOTA_FIFO_MODE_POS	2
#define ADPD188_SLOT_EN_SLOTA_EN_POS		0

/* ADPD188_REG_PD_LED_SELECT */
#define ADPD188_PD_LED_SELECT_SLOTB_PD_SEL_MASK		0x0F00
#define ADPD188_PD_LED_SELECT_SLOTA_PD_SEL_MASK		0x00F0
#define ADPD188_PD_LED_SELECT_SLOTB_LED_SEL_MASK	0x000C
#define ADPD188_PD_LED_SELECT_SLOTA_LED_SEL_MASK	0x0003
#define ADPD188_PD_LED_SELECT_SLOTB_PD_SEL_POS		8
#define ADPD188_PD_LED_SELECT_SLOTA_PD_SEL_POS		4
#define ADPD188_PD_LED_SELECT_SLOTB_LED_SEL_POS		2
#define ADPD188_PD_LED_SELECT_SLOTA_LED_SEL_POS		0

/* ADPD188_REG_NUM_AVG */
#define ADPD188_NUM_AVG_SLOTB_NUM_AVG_MASK	0x0700
#define ADPD188_NUM_AVG_SLOTA_NUM_AVG_MASK	0x0070
#define ADPD188_NUM_AVG_SLOTB_NUM_AVG_POS	8
#define ADPD188_NUM_AVG_SLOTA_NUM_AVG_POS	4

/* ADPD188_REG_BG_MEAS_A */
#define ADPD188_BG_MEAS_A_BG_COUNT_A_MASK	0xC000
#define ADPD188_BG_MEAS_A_BG_THRESH_A_MASK	0x3FFF
#define ADPD188_BG_MEAS_A_BG_COUNT_A_POS	14
#define ADPD188_BG_MEAS_A_BG_THRESH_A_POS	0

/* ADPD188_REG_INT_SEQ_A */
#define ADPD188_INT_SEQ_A_INTEG_ORDER_A_MASK	0x000F
#define ADPD188_INT_SEQ_A_INTEG_ORDER_A_POS	0

/* ADPD188_REG_BG_MEAS_B */
#define ADPD188_BG_MEAS_B_BG_COUNT_B_MASK	0xC000
#define ADPD188_BG_MEAS_B_BG_THRESH_B_MASK	0x3FFF
#define ADPD188_BG_MEAS_B_BG_COUNT_B_POS	14
#define ADPD188_BG_MEAS_B_BG_THRESH_B_POS	0

/* ADPD188_REG_INT_SEQ_B */
#define ADPD188_INT_SEQ_B_INTEG_ORDER_B_MASK	0x000F
#define ADPD188_INT_SEQ_B_INTEG_ORDER_B_POS	0

/* ADPD188_REG_ILED3_COARSE */
#define ADPD188_ILED3_COARSE_ILED3_SCALE_MASK	0x2000
#define ADPD188_ILED3_COARSE_ILED3_SLEW_MASK	0x0070
#define ADPD188_ILED3_COARSE_ILED3_COARSE_MASK	0x000F
#define ADPD188_ILED3_COARSE_ILED3_SCALE_POS	13
#define ADPD188_ILED3_COARSE_ILED3_SLEW_POS	4
#define ADPD188_ILED3_COARSE_ILED3_COARSE_POS	0

/* ADPD188_REG_ILED1_COARSE */
#define ADPD188_ILED1_COARSE_ILED1_SCALE_MASK	0x2000
#define ADPD188_ILED1_COARSE_ILED1_SLEW_MASK	0x0070
#define ADPD188_ILED1_COARSE_ILED1_COARSE_MASK	0x000F
#define ADPD188_ILED1_COARSE_ILED1_SCALE_POS	13
#define ADPD188_ILED1_COARSE_ILED1_SLEW_POS	4
#define ADPD188_ILED1_COARSE_ILED1_COARSE_POS	0

/* ADPD188_REG_ILED2_COARSE */
#define ADPD188_ILED2_COARSE_ILED2_SCALE_MASK	0x2000
#define ADPD188_ILED2_COARSE_ILED2_SLEW_MASK	0x0070
#define ADPD188_ILED2_COARSE_ILED2_COARSE_MASK	0x000F
#define ADPD188_ILED2_COARSE_ILED2_SCALE_POS	13
#define ADPD188_ILED2_COARSE_ILED2_SLEW_POS	4
#define ADPD188_ILED2_COARSE_ILED2_COARSE_POS	0

/* ADPD188_REG_ILED_FINE */
#define ADPD188_ILED_FINE_ILED3_FINE_MASK	0xF800
#define ADPD188_ILED_FINE_ILED2_FINE_MASK	0x07C0
#define ADPD188_ILED_FINE_ILED1_FINE_MASK	0x001F
#define ADPD188_ILED_FINE_ILED3_FINE_POS	11
#define ADPD188_ILED_FINE_ILED2_FINE_POS	6
#define ADPD188_ILED_FINE_ILED1_FINE_POS	0

/* ADPD188_REG_SLOTA_LED_PULSE */
#define ADPD188_SLOTA_LED_PULSE_SLOTA_LED_WIDTH_MASK	0x1F00
#define ADPD188_SLOTA_LED_PULSE_SLOTA_LED_OFFSET_MASK	0x00FF
#define ADPD188_SLOTA_LED_PULSE_SLOTA_LED_WIDTH_POS	8
#define ADPD188_SLOTA_LED_PULSE_SLOTA_LED_OFFSET_POS	0

/* ADPD188_REG_SLOTA_NUMPULSES */
#define ADPD188_SLOTA_NUMPULSES_SLOTA_PULSES_MASK	0xFF00
#define ADPD188_SLOTA_NUMPULSES_SLOTA_PERIOD_MASK	0x00FF
#define ADPD188_SLOTA_NUMPULSES_SLOTA_PULSES_POS	8
#define ADPD188_SLOTA_NUMPULSES_SLOTA_PERIOD_POS	0

/* ADPD188_REG_LED_DISABLE */
#define ADPD188_LED_DISABLE_SLOTB_LED_DIS_MASK	0x0200
#define ADPD188_LED_DISABLE_SLOTA_LED_DIS_MASK	0x0100
#define ADPD188_LED_DISABLE_SLOTB_LED_DIS_POS	9
#define ADPD188_LED_DISABLE_SLOTA_LED_DIS_POS	8

/* ADPD188_REG_SLOTB_LED_PULSE */
#define ADPD188_SLOTB_LED_PULSE_SLOTB_LED_WIDTH_MASK	0x1F00
#define ADPD188_SLOTB_LED_PULSE_SLOTB_LED_OFFSET_MASK	0x00FF
#define ADPD188_SLOTB_LED_PULSE_SLOTB_LED_WIDTH_POS	8
#define ADPD188_SLOTB_LED_PULSE_SLOTB_LED_OFFSET_POS	0

/* ADPD188_REG_SLOTB_NUMPULSES */
#define ADPD188_SLOTB_NUMPULSES_SLOTB_PULSES_MASK	0xFF00
#define ADPD188_SLOTB_NUMPULSES_SLOTB_PERIOD_MASK	0x00FF
#define ADPD188_SLOTB_NUMPULSES_SLOTB_PULSES_POS	8
#define ADPD188_SLOTB_NUMPULSES_SLOTB_PERIOD_POS	0

/* ADPD188_REG_ALT_PWR_DN */
#define ADPD188_ALT_PWR_DN_CH34_DISABLE_MASK	0xE000
#define ADPD188_ALT_PWR_DN_CH2_DISABLE_MASK	0x1C00
#define ADPD188_ALT_PWR_DN_SLOTB_PERIOD_MASK	0x0300
#define ADPD188_ALT_PWR_DN_SLOTA_PERIOD_MASK	0x0003
#define ADPD188_ALT_PWR_DN_CH34_DISABLE_POS	13
#define ADPD188_ALT_PWR_DN_CH2_DISABLE_POS	10
#define ADPD188_ALT_PWR_DN_SLOTB_PERIOD_POS	8
#define ADPD188_ALT_PWR_DN_SLOTA_PERIOD_POS	0

/* ADPD188_REG_SLOTA_AFE_WINDOW */
#define ADPD188_SLOTA_AFE_WINDOW_SLOTA_AFE_WIDTH_MASK	0xF800
#define ADPD188_SLOTA_AFE_WINDOW_SLOTA_AFE_OFFSET_MASK	0x07FF
#define ADPD188_SLOTA_AFE_WINDOW_SLOTA_AFE_WIDTH_POS	11
#define ADPD188_SLOTA_AFE_WINDOW_SLOTA_AFE_OFFSET_POS	0

/* ADPD188_REG_SLOTB_AFE_WINDOW */
#define ADPD188_SLOTB_AFE_WINDOW_SLOTB_AFE_WIDTH_MASK	0xF800
#define ADPD188_SLOTB_AFE_WINDOW_SLOTB_AFE_OFFSET_MASK	0x07FF
#define ADPD188_SLOTB_AFE_WINDOW_SLOTB_AFE_WIDTH_POS	11
#define ADPD188_SLOTB_AFE_WINDOW_SLOTB_AFE_OFFSET_POS	0

/* ADPD188_REG_AFE_PWR_CFG1 */
#define ADPD188_AFE_PWR_CFG1_V_CATHODE_MASK	0x0200
#define ADPD188_AFE_PWR_CFG1_AFE_POWERDOWN_MASK	0x01F8
#define ADPD188_AFE_PWR_CFG1_V_CATHODE_POS	9
#define ADPD188_AFE_PWR_CFG1_AFE_POWERDOWN_POS	3

/* ADPD188_REG_SLOTA_FLOAT_LED */
#define ADPD188_SLOTA_FLOAT_LED_FLT_LED_SELECT_A_MASK	0xC000
#define ADPD188_SLOTA_FLOAT_LED_FLT_LED_WIDTH_A_MASK	0x1F00
#define ADPD188_SLOTA_FLOAT_LED_FLT_LED_OFFSET_A_MASK	0x00FF
#define ADPD188_SLOTA_FLOAT_LED_FLT_LED_SELECT_A_POS	14
#define ADPD188_SLOTA_FLOAT_LED_FLT_LED_WIDTH_A_POS	8
#define ADPD188_SLOTA_FLOAT_LED_FLT_LED_OFFSET_A_POS	0

/* ADPD188_REG_SLOTB_FLOAT_LED */
#define ADPD188_SLOTB_FLOAT_LED_FLT_LED_SELECT_B_MASK	0xC000
#define ADPD188_SLOTB_FLOAT_LED_FLT_LED_WIDTH_B_MASK	0x1F00
#define ADPD188_SLOTB_FLOAT_LED_FLT_LED_OFFSET_B_MASK	0x00FF
#define ADPD188_SLOTB_FLOAT_LED_FLT_LED_SELECT_B_POS	14
#define ADPD188_SLOTB_FLOAT_LED_FLT_LED_WIDTH_B_POS	8
#define ADPD188_SLOTB_FLOAT_LED_FLT_LED_OFFSET_B_POS	0

/* ADPD188_REG_SLOTA_TIA_CFG */
/* ADPD188BI specific registers */
#define ADPD188_SLOTA_TIA_CFG_SLOTA_BUF_GAIN_MASK	0x0200
#define ADPD188_SLOTA_TIA_CFG_SLOTA_BUF_GAIN_POS	9
/* ADPD108X specific registers */
#define ADPD108X_SLOTA_TIA_CFG_SLOTA_BUF_GAIN_MASK	0x0300
#define ADPD108X_SLOTA_TIA_CFG_SLOTA_INT_GAIN_POS	8
/* ADPD188_REG_SLOTA_TIA_CFG common registers */
#define ADPD188_SLOTA_TIA_CFG_SLOTA_AFE_MODE_MASK	0xFC00
#define ADPD188_SLOTA_TIA_CFG_SLOTA_INT_AS_BUF_MASK	0x0080
#define ADPD188_SLOTA_TIA_CFG_SLOTA_TIA_IND_EN_MASK	0x0040
#define ADPD188_SLOTA_TIA_CFG_SLOTA_TIA_VREF_MASK	0x0030
#define ADPD188_SLOTA_TIA_CFG_SLOTA_TIA_GAIN_MASK	0x0003
#define ADPD188_SLOTA_TIA_CFG_SLOTA_AFE_MODE_POS	10
#define ADPD188_SLOTA_TIA_CFG_SLOTA_INT_AS_BUF_POS	7
#define ADPD188_SLOTA_TIA_CFG_SLOTA_TIA_IND_EN_POS	6
#define ADPD188_SLOTA_TIA_CFG_SLOTA_TIA_VREF_POS	4
#define ADPD188_SLOTA_TIA_CFG_SLOTA_TIA_GAIN_POS	0

/* ADPD188_REG_SLOTB_TIA_CFG */
/* ADPD188BI specific registers */
#define ADPD188_SLOTB_TIA_CFG_SLOTA_BUF_GAIN_MASK	0x0200
#define ADPD188_SLOTB_TIA_CFG_SLOTA_BUF_GAIN_POS	9
/* ADPD108X specific registers */
#define ADPD108X_SLOTB_TIA_CFG_SLOTA_BUF_GAIN_MASK	0x0300
#define ADPD108X_SLOTB_TIA_CFG_SLOTA_INT_GAIN_POS	8
/* ADPD188_REG_SLOTA_TIA_CFG common registers */
#define ADPD188_SLOTB_TIA_CFG_SLOTB_AFE_MODE_MASK	0xFC00
#define ADPD188_SLOTB_TIA_CFG_SLOTB_INT_AS_BUF_MASK	0x0080
#define ADPD188_SLOTB_TIA_CFG_SLOTB_TIA_IND_EN_MASK	0x0040
#define ADPD188_SLOTB_TIA_CFG_SLOTB_TIA_VREF_MASK	0x0030
#define ADPD188_SLOTB_TIA_CFG_SLOTB_TIA_GAIN_MASK	0x0003
#define ADPD188_SLOTB_TIA_CFG_SLOTB_AFE_MODE_POS	10
#define ADPD188_SLOTB_TIA_CFG_SLOTB_INT_AS_BUF_POS	7
#define ADPD188_SLOTB_TIA_CFG_SLOTB_TIA_IND_EN_POS	6
#define ADPD188_SLOTB_TIA_CFG_SLOTB_TIA_VREF_POS	4
#define ADPD188_SLOTB_TIA_CFG_SLOTB_TIA_GAIN_POS	0

/* ADPD188_REG_SAMPLE_CLK */
#define ADPD188_SAMPLE_CLK_CLK32K_BYP_MASK	0x0100
#define ADPD188_SAMPLE_CLK_CLK32K_EN_MASK	0x0080
#define ADPD188_SAMPLE_CLK_CLK32K_ADJUST_MASK	0x003F
#define ADPD188_SAMPLE_CLK_CLK32K_BYP_POS	8
#define ADPD188_SAMPLE_CLK_CLK32K_EN_POS	7
#define ADPD188_SAMPLE_CLK_CLK32K_ADJUST_POS	0

/* ADPD188_REG_CLK32M_ADJUST */
#define ADPD188_CLK32M_ADJUST_CLK32M_ADJUST_MASK	0x00FF
#define ADPD188_CLK32M_ADJUST_CLK32M_ADJUST_POS		0

/* ADPD188_REG_EXT_SYNC_SEL */
#define ADPD188_EXT_SYNC_SEL_GPIO1_OE_MASK	0x0040
#define ADPD188_EXT_SYNC_SEL_GPIO1_IE_MASK	0x0020
#define ADPD188_EXT_SYNC_SEL_EXT_SYNC_SEL_MASK	0x000C
#define ADPD188_EXT_SYNC_SEL_GPIO0_IE_MASK	0x0002
#define ADPD188_EXT_SYNC_SEL_GPIO1_OE_POS	6
#define ADPD188_EXT_SYNC_SEL_GPIO1_IE_POS	5
#define ADPD188_EXT_SYNC_SEL_EXT_SYNC_SEL_POS	2
#define ADPD188_EXT_SYNC_SEL_GPIO0_IE_POS	1

/* ADPD188_REG_CLK32M_CAL_EN */
#define ADPD188_CLK32M_CAL_EN_GPIO1_CTRL_MASK		0x0040
#define ADPD188_CLK32M_CAL_EN_CLK32M_CAL_EN_MASK	0x0020
#define ADPD188_CLK32M_CAL_EN_GPIO1_CTRL_POS		6
#define ADPD188_CLK32M_CAL_EN_CLK32M_CAL_EN_POS		5

/* ADPD188_REG_AFE_PWR_CFG2 */
#define ADPD188_AFE_PWR_CFG2_SLEEP_V_CATHODE_MASK	0x3000
#define ADPD188_AFE_PWR_CFG2_SLOTB_V_CATHODE_MASK	0x0C00
#define ADPD188_AFE_PWR_CFG2_SLOTA_V_CATHODE_MASK	0x0300
#define ADPD188_AFE_PWR_CFG2_REG54_VCAT_ENABLE_MASK	0x0080
#define ADPD188_AFE_PWR_CFG2_SLEEP_V_CATHODE_POS	12
#define ADPD188_AFE_PWR_CFG2_SLOTB_V_CATHODE_POS	10
#define ADPD188_AFE_PWR_CFG2_SLOTA_V_CATHODE_POS	8
#define ADPD188_AFE_PWR_CFG2_REG54_VCAT_ENABLE_POS	7

/* ADPD188_REG_TIA_INDEP_GAIN */
#define ADPD188_TIA_INDEP_GAIN_SLOTB_TIA_GAIN_4_MASK	0x0C00
#define ADPD188_TIA_INDEP_GAIN_SLOTB_TIA_GAIN_3_MASK	0x0300
#define ADPD188_TIA_INDEP_GAIN_SLOTB_TIA_GAIN_2_MASK	0x00C0
#define ADPD188_TIA_INDEP_GAIN_SLOTA_TIA_GAIN_4_MASK	0x0030
#define ADPD188_TIA_INDEP_GAIN_SLOTA_TIA_GAIN_3_MASK	0x000C
#define ADPD188_TIA_INDEP_GAIN_SLOTA_TIA_GAIN_2_MASK	0x0003
#define ADPD188_TIA_INDEP_GAIN_SLOTB_TIA_GAIN_4_POS	10
#define ADPD188_TIA_INDEP_GAIN_SLOTB_TIA_GAIN_3_POS	8
#define ADPD188_TIA_INDEP_GAIN_SLOTB_TIA_GAIN_2_POS	6
#define ADPD188_TIA_INDEP_GAIN_SLOTA_TIA_GAIN_4_POS	4
#define ADPD188_TIA_INDEP_GAIN_SLOTA_TIA_GAIN_3_POS	2
#define ADPD188_TIA_INDEP_GAIN_SLOTA_TIA_GAIN_2_POS	0

/* ADPD188_REG_MATH */
#define ADPD188_MATH_FLT_MATH34_B_MASK		0x0C00
#define ADPD188_MATH_FLT_MATH34_A_MASK		0x0300
#define ADPD188_MATH_ENA_INT_AS_BUF_MASK	0x0080
#define ADPD188_MATH_FLT_MATH12_B_MASK		0x0060
#define ADPD188_MATH_FLT_MATH12_A_MASK		0x0006
#define ADPD188_MATH_FLT_MATH34_B_POS		10
#define ADPD188_MATH_FLT_MATH34_A_POS		8
#define ADPD188_MATH_ENA_INT_AS_BUF_POS		7
#define ADPD188_MATH_FLT_MATH12_B_POS		5
#define ADPD188_MATH_FLT_MATH12_A_POS		1

/* ADPD188_REG_FLT_CONFIG_B */
#define ADPD188_FLT_CONFIG_B_FLT_EN_B_MASK	0x6000
#define ADPD188_FLT_CONFIG_B_FLT_PRECON_B_MASK	0x1F00
#define ADPD188_FLT_CONFIG_B_FLT_EN_B_POS	13
#define ADPD188_FLT_CONFIG_B_FLT_PRECON_B_POS	8

/* ADPD188_REG_FLT_LED_FIRE */
#define ADPD188_FLT_LED_FIRE_FLT_LED_FIRE_B_MASK	0xF000
#define ADPD188_FLT_LED_FIRE_FLT_LED_FIRE_A_MASK	0x0F00
#define ADPD188_FLT_LED_FIRE_FLT_LED_FIRE_B_POS		12
#define ADPD188_FLT_LED_FIRE_FLT_LED_FIRE_A_POS		8

/* ADPD188_REG_FLT_CONFIG_A */
#define ADPD188_FLT_CONFIG_A_FLT_EN_A_MASK	0x6000
#define ADPD188_FLT_CONFIG_A_FLT_PRECON_A_MASK	0x1F00
#define ADPD188_FLT_CONFIG_A_FLT_EN_A_POS	13
#define ADPD188_FLT_CONFIG_A_FLT_PRECON_A_POS	8

/* ADPD188_REG_DATA_ACCESS_CTL */
#define ADPD188_DATA_ACCESS_CTL_SLOTB_DATA_HOLD_MASK	0x0004
#define ADPD188_DATA_ACCESS_CTL_SLOTA_DATA_HOLD_MASK	0x0002
#define ADPD188_DATA_ACCESS_CTL_DIGITAL_CLOCK_ENA_MASK	0x0001
#define ADPD188_DATA_ACCESS_CTL_SLOTB_DATA_HOLD_POS	2
#define ADPD188_DATA_ACCESS_CTL_SLOTA_DATA_HOLD_POS	1
#define ADPD188_DATA_ACCESS_CTL_DIGITAL_CLOCK_ENA_POS	0

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adpd_supported_devices` enumeration defines a set of constants
 * representing the different devices supported by the ADPD188 driver.
 * Each enumerator corresponds to a specific device model, allowing the
 * driver to handle device-specific operations and configurations. This
 * enumeration is used to identify and differentiate between the
 * supported devices within the driver code.
 *
 * @param ADPD188BI Represents the ADPD188BI device.
 * @param ADPD1080 Represents the ADPD1080 device.
 * @param ADPD1081 Represents the ADPD1081 device.
 ******************************************************************************/
enum adpd_supported_devices {
	ADPD188BI,
	ADPD1080,
	ADPD1081
};

/***************************************************************************//**
 * @brief The `adpd188_phy_init` union is a communication protocol
 * initialization structure that allows for the configuration of either
 * I2C or SPI communication interfaces. It contains two members,
 * `i2c_phy` and `spi_phy`, which are structures for initializing the
 * respective I2C and SPI protocols. This union is used to abstract the
 * physical communication layer, enabling the ADPD188 device to be
 * configured for either protocol based on the specific requirements of
 * the application.
 *
 * @param i2c_phy I2C initialization structure.
 * @param spi_phy SPI initialization structure.
 ******************************************************************************/
union adpd188_phy_init {
	/** I2C initialization structure. */
	struct no_os_i2c_init_param i2c_phy;
	/** SPI initialization structure. */
	struct no_os_spi_init_param spi_phy;
};

/***************************************************************************//**
 * @brief The `adpd188_phy_opt` is an enumeration that defines the types of
 * physical communication protocols available for the ADPD188 device. It
 * provides two options: SPI and I2C, allowing the user to select the
 * appropriate communication method for interfacing with the device. This
 * enum is used to configure the communication protocol during the
 * initialization of the ADPD188 driver.
 *
 * @param ADPD188_SPI Represents the SPI communication protocol option.
 * @param ADPD188_I2C Represents the I2C communication protocol option.
 ******************************************************************************/
enum adpd188_phy_opt {
	/** SPI communication. */
	ADPD188_SPI,
	/** I2C communication. */
	ADPD188_I2C
};

/***************************************************************************//**
 * @brief The `adpd188_mode` enumeration defines the different operational modes
 * available for the ADPD188 device, which include standby, program, and
 * normal modes. These modes are used to control the state of the device,
 * allowing it to be set to a low-power standby state, a configuration
 * state for programming, or a normal operational state for regular use.
 *
 * @param ADPD188_STANDBY Represents the standby mode of the ADPD188 device.
 * @param ADPD188_PROGRAM Represents the program mode of the ADPD188 device.
 * @param ADPD188_NORMAL Represents the normal operation mode of the ADPD188
 * device.
 ******************************************************************************/
enum adpd188_mode {
	/** Standby mode. */
	ADPD188_STANDBY,
	/** Program mode. */
	ADPD188_PROGRAM,
	/** Normal mode. */
	ADPD188_NORMAL
};

/***************************************************************************//**
 * @brief The `adpd188_interrupt` enumeration defines interrupt flags for the
 * ADPD188 device, which are used to signal specific events such as the
 * completion of a conversion in Slot A or Slot B, or when the FIFO
 * threshold is reached. These flags are represented as bitwise values,
 * allowing them to be combined and checked efficiently in the device's
 * interrupt handling logic.
 *
 * @param ADPD188_SLOTA_INT Slot A conversion interrupt flag.
 * @param ADPD188_SLOTB_INT Slot B conversion interrupt flag.
 * @param ADPD188_FIFO_INT FIFO threshold reached interrupt flag.
 ******************************************************************************/
enum adpd188_interrupt {
	/** Slot A conversion interrupt flag. */
	ADPD188_SLOTA_INT = 0x1,
	/** Slot B conversion interrupt flag. */
	ADPD188_SLOTB_INT = 0x2,
	/** FIFO threshold reached interrupt flag. */
	ADPD188_FIFO_INT = 0x4
};

/***************************************************************************//**
 * @brief The `adpd188_gpio_config` structure is used to configure the General
 * Purpose Input/Output (GPIO) settings for the ADPD188 device. It
 * includes fields to specify the GPIO ID, polarity, driver status, and
 * an enable flag specifically for GPIO0. This configuration is crucial
 * for setting up the GPIOs to interact correctly with the device's other
 * components and external systems.
 *
 * @param gpio_id Represents the GPIO ID, which can be either 0 or 1.
 * @param gpio_pol Indicates the polarity of the GPIO.
 * @param gpio_drv Specifies the status of the GPIO driver, whether it is driven
 * or open-drain.
 * @param gpio_en Enables the GPIO, applicable only for GPIO0.
 ******************************************************************************/
struct adpd188_gpio_config {
	/** GPIO ID (0 or 1) */
	uint8_t gpio_id;
	/** GPIO polarity */
	uint8_t gpio_pol;
	/** Status of the GPIO driver (driven or open-drain) */
	uint8_t gpio_drv;
	/** GPIO enable (only for GPIO0) */
	uint8_t gpio_en;
};

/***************************************************************************//**
 * @brief The `adpd188_gpio_alt_config` is an enumeration that defines various
 * configurations for the GPIO pins of the ADPD188 device. These
 * configurations include compatibility with older devices, interrupt
 * functions, pulse outputs for different time slots, and specific output
 * states. This enum allows for flexible control over the GPIO behavior,
 * enabling it to be used for different signaling and timing purposes in
 * conjunction with the ADPD188's operational modes.
 *
 * @param ADPD188_ADPD103 GPIO backwards compatible with the ADPD103 INT
 * functionality.
 * @param ADPD188_INT_FUNC Interrupt function provided on GPIO.
 * @param ADPD188_ACTIVE_PULSE Asserts at the start of the first time slot and
 * deasserts at end of last time slot.
 * @param ADPD188_SLOTA_PULSE Time Slot A pulse output.
 * @param ADPD188_SLOTB_PULSE Time Slot B pulse output.
 * @param ADPD188_ANYSLOT_PULSE Pulse output of both time slots.
 * @param ADPD188_SLOTA_OUT Output data cycle occurred for Time Slot A.
 * @param ADPD188_SLOTB_OUT Output data cycle occurred for Time Slot B.
 * @param ADPD188_ANYSLOT_OUT Output data cycle occurred.
 * @param ADPD188_HALF_SAMPLING Toggles on every sample, which provides a signal
 * at half the sampling rate.
 * @param ADPD188_OUT_LOW Output = 0.
 * @param ADPD188_OUT_HIGH Output = 1.
 * @param ADPD188_32KHZ_OSC 32 kHz oscillator output.
 ******************************************************************************/
enum adpd188_gpio_alt_config {
	/** GPIO backwards compatible with the ADPD103 INT functionality. */
	ADPD188_ADPD103 = 0x00,
	/** Interrupt function provided on GPIO. */
	ADPD188_INT_FUNC = 0x01,
	/**
	 * Asserts at the start of the first time slot and deasserts at end of last
	 * time slot.
	 */
	ADPD188_ACTIVE_PULSE = 0x02,
	/** Time Slot A pulse output. */
	ADPD188_SLOTA_PULSE = 0x05,
	/** Time Slot B pulse output. */
	ADPD188_SLOTB_PULSE = 0x06,
	/** Pulse output of both time slots. */
	ADPD188_ANYSLOT_PULSE = 0x07,
	/** Output data cycle occurred for Time Slot A. */
	ADPD188_SLOTA_OUT = 0x0C,
	/** Output data cycle occurred for Time Slot B. */
	ADPD188_SLOTB_OUT = 0x0D,
	/** Output data cycle occurred. */
	ADPD188_ANYSLOT_OUT = 0x0E,
	/**
	 * Toggles on every sample, which provides a signal at half the sampling
	 * rate.
	 */
	ADPD188_HALF_SAMPLING = 0x0F,
	/** Output = 0. */
	ADPD188_OUT_LOW = 0x10,
	/** Output = 1. */
	ADPD188_OUT_HIGH = 0x11,
	/** 32 kHz oscillator output. */
	ADPD188_32KHZ_OSC = 0x13
};

/***************************************************************************//**
 * @brief The `adpd188_slots` enumeration defines two time slots, SLOTA and
 * SLOTB, used in the ADPD188 device for managing different operational
 * phases or tasks. These slots are typically used to configure and
 * control the timing of data acquisition or processing tasks within the
 * device, allowing for efficient management of resources and scheduling
 * of operations.
 *
 * @param ADPD188_SLOTA Represents the first time slot in the ADPD188 device.
 * @param ADPD188_SLOTB Represents the second time slot in the ADPD188 device.
 ******************************************************************************/
enum adpd188_slots {
	/** First slot. */
	ADPD188_SLOTA,
	/** Second slot. */
	ADPD188_SLOTB
};

/***************************************************************************//**
 * @brief The `adpd188_slot_fifo_mode` enumeration defines the various modes in
 * which data can be stored in the FIFO for a time slot in the ADPD188
 * device. It provides options for storing no data, sums of channel data
 * in either 16-bit or 32-bit formats, or individual channel data in
 * 16-bit or 32-bit formats. This allows for flexible data handling
 * depending on the application's requirements for data precision and
 * storage efficiency.
 *
 * @param ADPD188_NO_FIFO No data is stored in the FIFO.
 * @param ADPD188_16BIT_SUM Stores a 16-bit sum of all four channels in the
 * FIFO.
 * @param ADPD188_32BIT_SUM Stores a 32-bit sum of all four channels in the
 * FIFO.
 * @param ADPD188_16BIT_4CHAN Stores four channels of 16-bit sample data for the
 * time slot in the FIFO.
 * @param ADPD188_32BIT_4CHAN Stores four channels of 32-bit sample data for the
 * time slot in the FIFO.
 ******************************************************************************/
enum adpd188_slot_fifo_mode {
	/** No data to FIFO. */
	ADPD188_NO_FIFO,
	/** 16-bit sum of all four channels. */
	ADPD188_16BIT_SUM,
	/** 32-bit sum of all four channels. */
	ADPD188_32BIT_SUM,
	/** Four channels of 16-bit sample data for the time slot. */
	ADPD188_16BIT_4CHAN = 0x4,
	/** Four channels of 32-bit sample data for the time slot. */
	ADPD188_32BIT_4CHAN = 0x6
};

/***************************************************************************//**
 * @brief The `adpd188_slot_config` structure is used to configure a time slot
 * for the ADPD188 device. It includes a time slot ID, a boolean to
 * enable or disable the slot, and a FIFO mode to determine how data is
 * stored in the FIFO. This structure is essential for setting up the
 * operational parameters of a time slot, allowing for flexible data
 * handling and processing in the ADPD188 sensor system.
 *
 * @param slot_id Specifies the time slot ID using the `adpd188_slots`
 * enumeration.
 * @param slot_en Indicates whether the time slot is enabled or not as a boolean
 * value.
 * @param sot_fifo_mode Defines the FIFO mode for the time slot using the
 * `adpd188_slot_fifo_mode` enumeration.
 ******************************************************************************/
struct adpd188_slot_config {
	/** Time slot ID. */
	enum adpd188_slots slot_id;
	/** Enable time slot. */
	bool slot_en;
	/** Time slot FIFO mode. */
	enum adpd188_slot_fifo_mode sot_fifo_mode;
};

/***************************************************************************//**
 * @brief The `adpd188_dev` structure is a driver descriptor for the ADPD188
 * device, encapsulating essential information about the device
 * configuration and communication setup. It includes the selected device
 * type, the communication protocol option (either I2C or SPI), and
 * pointers to the physical communication descriptor and GPIO
 * descriptors. This structure is crucial for initializing and managing
 * the ADPD188 device's operations within a system.
 *
 * @param device Specifies the selected device from the supported devices
 * enumeration.
 * @param phy_opt Indicates the type of physical communication protocol used.
 * @param phy_desc A pointer to the communication physical descriptor.
 * @param gpio0 Descriptor for GPIO 0.
 * @param gpio1 Descriptor for GPIO 1.
 ******************************************************************************/
struct adpd188_dev {
	/** Selected device */
	enum adpd_supported_devices device;
	/** Communication physical type. */
	enum adpd188_phy_opt phy_opt;
	/** Communication physical descriptor. */
	void *phy_desc;
	/** GPIO 0 descriptor. */
	struct no_os_gpio_desc *gpio0;
	/** GPIO 1 descriptor. */
	struct no_os_gpio_desc *gpio1;
};

/***************************************************************************//**
 * @brief The `adpd188_init_param` structure is used to initialize the ADPD188
 * device driver. It includes fields for selecting the specific device
 * model, the communication protocol (SPI or I2C), and the necessary
 * initialization parameters for the chosen protocol. Additionally, it
 * provides initialization structures for two GPIOs, allowing for
 * configuration of the device's input/output pins. This structure is
 * essential for setting up the device's communication and control
 * interfaces before operation.
 *
 * @param device Specifies the selected device from the supported devices
 * enumeration.
 * @param phy_opt Defines the communication physical type, either SPI or I2C.
 * @param phy_init Holds the initialization parameters for the selected
 * communication protocol, either I2C or SPI.
 * @param gpio0_init Contains the initialization parameters for GPIO 0.
 * @param gpio1_init Contains the initialization parameters for GPIO 1.
 ******************************************************************************/
struct adpd188_init_param {
	/** Selected device */
	enum adpd_supported_devices device;
	/** Communication physical type. */
	enum adpd188_phy_opt phy_opt;
	/** Communication physical initialization structure. */
	union adpd188_phy_init phy_init;
	/** GPIO 0 initialization structure. */
	struct no_os_gpio_init_param gpio0_init;
	/** GPIO 0 initialization structure. */
	struct no_os_gpio_init_param gpio1_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Initialize the ADPD188 driver. */
/***************************************************************************//**
 * @brief This function initializes the ADPD188 device using the provided
 * initialization parameters, setting up the communication interface and
 * configuring GPIOs. It must be called before any other operations on
 * the device. The function allocates memory for the device structure and
 * initializes the communication interface based on the specified device
 * type and physical communication protocol. It also verifies the device
 * ID to ensure the correct device is connected. If any step fails, the
 * function cleans up allocated resources and returns an error code.
 *
 * @param device A pointer to a pointer of type `struct adpd188_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct adpd188_init_param` containing
 * initialization parameters such as device type,
 * communication protocol, and GPIO configurations. Must not
 * be null and must be properly initialized with valid values.
 * @return Returns 0 on successful initialization, or -1 if an error occurs
 * during initialization, such as memory allocation failure or device ID
 * mismatch.
 ******************************************************************************/
int32_t adpd188_init(struct adpd188_dev **device,
		     struct adpd188_init_param *init_param);

/* Free resources allocated by adpd188_init(). */
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * ADPD188 device when it is no longer needed. This function should be
 * called after the device has been initialized and used, to ensure that
 * all allocated resources, such as communication interfaces and GPIOs,
 * are correctly freed. It is important to ensure that the `dev`
 * parameter is valid and was previously initialized by `adpd188_init`.
 * The function handles both SPI and I2C communication options and will
 * return an error if the removal of any resource fails.
 *
 * @param dev A pointer to an `adpd188_dev` structure representing the device to
 * be removed. Must not be null and should point to a valid,
 * initialized device structure. The function will handle invalid
 * communication options by returning an error.
 * @return Returns 0 on successful removal of all resources, or -1 if an error
 * occurs during the removal process.
 ******************************************************************************/
int32_t adpd188_remove(struct adpd188_dev *dev);

/* Read one 16 bit register of the ADPD188. */
/***************************************************************************//**
 * @brief Use this function to read a 16-bit register from the ADPD188 device.
 * It requires a valid device structure and a register address to read
 * from. The function supports both SPI and I2C communication protocols,
 * as specified in the device structure. Ensure that the device is
 * properly initialized before calling this function. The function will
 * return an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read. Must be a valid register
 * address for the ADPD188 device.
 * @param reg_val A pointer to a uint16_t variable where the read register value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or -1 if the read operation fails.
 ******************************************************************************/
int32_t adpd188_reg_read(struct adpd188_dev *dev, uint8_t reg_addr,
			 uint16_t *reg_val);

/* Write one 16 bit register of the ADPD188. */
/***************************************************************************//**
 * @brief This function is used to write a 16-bit value to a specific register
 * of the ADPD188 device. It requires a valid device structure that has
 * been initialized and configured for either SPI or I2C communication.
 * The function should be called when there is a need to configure or
 * modify the settings of the ADPD188 device by writing to its registers.
 * It handles both SPI and I2C communication protocols based on the
 * device's configuration. The function returns an error code if the
 * communication fails or if the communication protocol is not supported.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address for the ADPD188 device.
 * @param reg_val The 16-bit value to write to the specified register.
 * @return Returns 0 on success, or -1 if an error occurs during communication
 * or if the protocol is unsupported.
 ******************************************************************************/
int32_t adpd188_reg_write(struct adpd188_dev *dev, uint8_t reg_addr,
			  uint16_t reg_val);

/* Get the mode of operation of the ADPD188. */
/***************************************************************************//**
 * @brief Use this function to obtain the current operational mode of the
 * ADPD188 device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function reads
 * the mode register and extracts the mode information, which is then
 * stored in the provided mode parameter. This function is useful for
 * verifying the device's current state or for debugging purposes. If the
 * function encounters an error while reading the register, it returns an
 * error code.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param mode A pointer to an adpd188_mode enum where the current mode will be
 * stored. Must not be null.
 * @return Returns 0 on success, or -1 if an error occurs while reading the mode
 * register.
 ******************************************************************************/
int32_t adpd188_mode_get(struct adpd188_dev *dev, enum adpd188_mode *mode);

/* Set the mode of operation of the ADPD188. */
/***************************************************************************//**
 * @brief Use this function to change the operational mode of the ADPD188 device
 * to one of the predefined modes such as standby, program, or normal.
 * This function should be called when you need to switch the device's
 * mode of operation, for example, to prepare it for data acquisition or
 * to put it into a low-power state. Ensure that the device has been
 * properly initialized before calling this function. The function writes
 * the new mode to the device's mode register and returns a status code
 * indicating success or failure.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param new_mode An enum value of type adpd188_mode representing the new mode
 * to set. Valid values are ADPD188_STANDBY, ADPD188_PROGRAM,
 * and ADPD188_NORMAL. Invalid values may result in undefined
 * behavior.
 * @return Returns an int32_t status code. A value of 0 indicates success, while
 * a negative value indicates an error occurred during the operation.
 ******************************************************************************/
int32_t adpd188_mode_set(struct adpd188_dev *dev, enum adpd188_mode new_mode);

/* Get the number of bytes currently present in FIFO. */
/***************************************************************************//**
 * @brief Use this function to determine the number of bytes currently stored in
 * the FIFO of the ADPD188 device. It must be called with a valid device
 * structure that has been initialized. The function reads the status
 * register to extract the FIFO sample count and stores it in the
 * provided output parameter. This function should be called when you
 * need to monitor or manage the FIFO data, such as before reading or
 * clearing the FIFO.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param bytes_no A pointer to a uint8_t where the number of bytes in the FIFO
 * will be stored. Must not be null.
 * @return Returns 0 on success, or -1 if there is an error reading the status
 * register.
 ******************************************************************************/
int32_t adpd188_fifo_status_get(struct adpd188_dev *dev, uint8_t *bytes_no);

/* Empty the FIFO. */
/***************************************************************************//**
 * @brief Use this function to clear the FIFO of the ADPD188 device, which is
 * necessary when you want to reset the FIFO state or prepare it for new
 * data collection. This function should be called when the device is in
 * a state where clearing the FIFO will not disrupt ongoing operations.
 * It reads the status register, modifies it to clear the FIFO, and
 * writes it back. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if the pointer is invalid.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as when reading or writing to the device registers fails.
 ******************************************************************************/
int32_t adpd188_fifo_clear(struct adpd188_dev *dev);

/*
 * Set the number of 16 bit words that need to be in the FIFO to trigger an
 * interrupt.
 */
/***************************************************************************//**
 * @brief This function configures the number of 16-bit words that must be
 * present in the FIFO of the ADPD188 device to trigger an interrupt. It
 * should be used when you need to set a specific threshold for data
 * collection before an interrupt is generated. The function must be
 * called with a valid device structure and a threshold value within the
 * allowed range. If the threshold value exceeds the maximum allowed, the
 * function returns an error.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param word_no The number of 16-bit words to set as the FIFO threshold. Valid
 * range is 0 to ADPD188_FIFO_THRESH_MAX_THRESHOLD (63). If the
 * value exceeds this range, the function returns an error.
 * @return Returns 0 on success or -1 on failure, such as when the threshold
 * value is out of range or a register read/write operation fails.
 ******************************************************************************/
int32_t adpd188_fifo_thresh_set(struct adpd188_dev *dev, uint8_t word_no);

/* Get the slot and FIFO interrupt flags. */
/***************************************************************************//**
 * @brief Use this function to check the current interrupt status of the ADPD188
 * device. It reads the status register and updates the provided flags
 * variable with the interrupt flags for Slot A and Slot B. This function
 * should be called when you need to determine which interrupts have been
 * triggered. Ensure that the device has been properly initialized before
 * calling this function. The function will return an error if the
 * register read operation fails.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param flags A pointer to a uint8_t variable where the interrupt flags will
 * be stored. Must not be null. The variable will be set to 0
 * before any flags are set.
 * @return Returns 0 on success, or -1 if there is an error reading the status
 * register.
 ******************************************************************************/
int32_t adpd188_interrupt_get(struct adpd188_dev *dev, uint8_t *flags);

/* Clear the slot and FIFO interrupt flags. */
/***************************************************************************//**
 * @brief This function is used to clear specific interrupt flags on the ADPD188
 * device, which can include Slot A, Slot B, and FIFO interrupts. It
 * should be called when you need to reset these interrupt flags after
 * they have been handled. The function requires a valid device structure
 * and a set of flags indicating which interrupts to clear. If no flags
 * are provided, the function returns immediately with no action. It is
 * important to ensure that the device has been properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param flags A bitmask indicating which interrupt flags to clear. Valid
 * values are combinations of ADPD188_SLOTA_INT, ADPD188_SLOTB_INT,
 * and ADPD188_FIFO_INT. If zero, no interrupts are cleared.
 * @return Returns 0 on success, or -1 if an error occurs during register read
 * or write operations.
 ******************************************************************************/
int32_t adpd188_interrupt_clear(struct adpd188_dev *dev, uint8_t flags);

/* Enable the slot and FIFO interrupt flags. */
/***************************************************************************//**
 * @brief Use this function to enable specific interrupts on the ADPD188 device
 * by providing the appropriate flags. This function should be called
 * after the device has been initialized and is ready to handle
 * interrupts. It modifies the interrupt mask register to enable the
 * specified interrupts, allowing the device to generate interrupt
 * signals for the selected events. Ensure that the device pointer is
 * valid and that the flags parameter contains valid interrupt flags. The
 * function returns an error code if the operation fails.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param flags A bitmask of interrupt flags to enable. Valid flags include
 * ADPD188_SLOTA_INT, ADPD188_SLOTB_INT, and ADPD188_FIFO_INT.
 * Invalid flags are ignored.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t adpd188_interrupt_en(struct adpd188_dev *dev, uint8_t flags);

/* Setup drive and polarity of the GPIOs. */
/***************************************************************************//**
 * @brief This function is used to configure the drive strength and polarity of
 * the GPIO pins on the ADPD188 device. It allows setting the drive mode
 * and polarity for GPIO0 and GPIO1, and enables GPIO0 if required. This
 * function should be called after the device has been initialized and
 * before the GPIOs are used for any specific functionality. It returns
 * an error code if the configuration fails, ensuring that the caller can
 * handle such cases appropriately.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param config A structure of type adpd188_gpio_config containing the desired
 * GPIO configuration. The gpio_id must be either 0 or 1. The
 * gpio_pol and gpio_drv fields determine the polarity and drive
 * mode, respectively. The gpio_en field is only applicable for
 * GPIO0.
 * @return Returns 0 on success or a negative error code if the configuration
 * fails.
 ******************************************************************************/
int32_t adpd188_gpio_setup(struct adpd188_dev *dev,
			   struct adpd188_gpio_config config);

/* Setup the GPIO source. */
/***************************************************************************//**
 * @brief This function configures the alternate function of a specified GPIO
 * pin on the ADPD188 device. It should be called when you need to change
 * the GPIO's behavior to one of the predefined alternate configurations.
 * The function requires a valid device structure and a GPIO ID of either
 * 0 or 1. The configuration parameter must be a valid value from the
 * `adpd188_gpio_alt_config` enumeration. If the GPIO ID is invalid or if
 * there is an error reading or writing to the device, the function will
 * return an error code.
 *
 * @param dev A pointer to an initialized `adpd188_dev` structure representing
 * the device. Must not be null.
 * @param gpio_id The ID of the GPIO to configure, either 0 or 1. Invalid values
 * will result in an error.
 * @param config The desired alternate configuration for the GPIO, specified as
 * a value from the `adpd188_gpio_alt_config` enumeration.
 * @return Returns 0 on success, or -1 if an error occurs (e.g., invalid GPIO ID
 * or device communication failure).
 ******************************************************************************/
int32_t adpd188_gpio_alt_setup(struct adpd188_dev *dev, uint8_t gpio_id,
			       enum adpd188_gpio_alt_config config);

/* Do software reset of the device. */
/***************************************************************************//**
 * @brief Use this function to reset the ADPD188 device to its default state via
 * a software command. This is typically used to ensure the device is in
 * a known state before starting configuration or operation. It must be
 * called with a valid device structure that has been properly
 * initialized. The function will return an error code if the reset
 * command fails.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device to reset. Must not be null. The caller retains ownership of
 * the structure.
 * @return Returns an int32_t value indicating success or failure of the reset
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t adpd188_sw_reset(struct adpd188_dev *dev);

/* Do internal 32MHz clock calibration. */
/***************************************************************************//**
 * @brief This function performs a calibration of the internal 32MHz clock for
 * the ADPD188 device, which is necessary to ensure accurate timing and
 * operation. It should be called when precise clock calibration is
 * required, such as after device initialization or when clock drift is
 * suspected. The function requires a valid device structure and will
 * return an error code if any register read or write operation fails. It
 * is important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null. The caller retains ownership of the
 * structure.
 * @return Returns 0 on success or -1 on failure, indicating an error in reading
 * or writing to the device registers.
 ******************************************************************************/
int32_t adpd188_clk32mhz_cal(struct adpd188_dev *dev);

/* Enable slot and setup its FIFO interaction. */
/***************************************************************************//**
 * @brief This function configures a specified time slot on the ADPD188 device,
 * enabling or disabling it and setting its FIFO mode. It should be
 * called when you need to set up or modify the configuration of a time
 * slot, either Slot A or Slot B, on the device. The function requires a
 * valid device descriptor and a slot configuration structure. It returns
 * an error code if the operation fails, such as when the device is not
 * properly initialized or if there is a communication error.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param config A structure of type adpd188_slot_config specifying the slot ID,
 * whether the slot is enabled, and the FIFO mode. The slot_id
 * must be either ADPD188_SLOTA or ADPD188_SLOTB.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * issues such as communication errors.
 ******************************************************************************/
int32_t adpd188_slot_setup(struct adpd188_dev *dev,
			   struct adpd188_slot_config config);

/* Set sample frequency of the ADC. */
/***************************************************************************//**
 * @brief This function configures the sample frequency of the ADC for the
 * ADPD188 device. It should be called when you need to adjust the
 * sampling rate of the ADC to a specific frequency, expressed in hertz.
 * The function expects the frequency to be within a valid range, and it
 * will return an error if the frequency exceeds the maximum allowable
 * value. Ensure that the device is properly initialized before calling
 * this function.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param freq_hz The desired sample frequency in hertz. Valid range is 0 to
 * 2000 Hz. If the value exceeds 2000 Hz, the function returns an
 * error.
 * @return Returns 0 on success, or -1 if the frequency is invalid (greater than
 * 2000 Hz).
 ******************************************************************************/
int32_t adpd188_adc_fsample_set(struct adpd188_dev *dev, uint16_t freq_hz);

/* Get sample frequency of the ADC. */
/***************************************************************************//**
 * @brief Use this function to obtain the current sample frequency of the ADC in
 * the ADPD188 device. It must be called with a valid device structure
 * that has been initialized. The function reads the frequency setting
 * from the device and calculates the actual frequency in hertz, storing
 * it in the provided output parameter. This function should be called
 * when you need to know the current sampling rate of the ADC for
 * configuration or monitoring purposes.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null.
 * @param freq_hz A pointer to a uint16_t where the function will store the
 * calculated sample frequency in hertz. Must not be null.
 * @return Returns 0 on success, or -1 if there is an error reading the
 * register.
 ******************************************************************************/
int32_t adpd188_adc_fsample_get(struct adpd188_dev *dev, uint16_t *freq_hz);

/* Do initial configuration of the device to use as a smoke detector. */
/***************************************************************************//**
 * @brief This function sets up the ADPD188 device to operate as a smoke
 * detector by configuring various registers and settings specific to
 * this application. It must be called after the device has been
 * initialized using the appropriate initialization function. The
 * function configures slots A and B, sets LED and photodiode selections,
 * adjusts LED power, and configures integrator windows and other
 * parameters necessary for smoke detection. It returns an error code if
 * any configuration step fails, ensuring that the device is correctly
 * set up for smoke detection before proceeding.
 *
 * @param dev A pointer to an initialized adpd188_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if any register read/write
 * operation fails.
 * @return Returns 0 on success, or a negative error code if any configuration
 * step fails.
 ******************************************************************************/
int32_t adpd188_smoke_detect_setup(struct adpd188_dev *dev);

#endif /* ADPD188_H_ */
