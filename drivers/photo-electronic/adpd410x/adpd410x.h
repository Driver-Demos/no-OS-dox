/***************************************************************************//**
 *   @file   ADPD410X.h
 *   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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

#ifndef ADPD410X_H_
#define ADPD410X_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_i2c.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define ADPD410X_REG_FIFO_STATUS	0x0000
#define ADPD410X_REG_INT_STATUS_DATA	0x0001
#define ADPD410X_REG_INT_STATUS_LEV0	0x0002
#define ADPD410X_REG_INT_STATUS_LEV1	0x0003
#define ADPD410X_REG_FIFO_TH		0x0006
#define ADPD410X_REG_INT_ACLEAR		0x0007
#define ADPD410X_REG_CHIP_ID		0x0008
#define ADPD410X_REG_OSC32M		0x0009
#define ADPD410X_REG_OSC32M_CAL		0x000A
#define ADPD410X_REG_OSC1M		0x000B
#define ADPD410X_REG_OSC32K		0x000C
#define ADPD410X_REG_TS_FREQ		0x000D
#define ADPD410X_REG_TS_FREQH		0x000E
#define ADPD410X_REG_SYS_CTL		0x000F
#define ADPD410X_REG_OPMODE          	0x0010
#define ADPD410X_REG_STAMP_L         	0x0011
#define ADPD410X_REG_STAMP_H         	0x0012
#define ADPD410X_REG_STAMPDELTA      	0x0013
#define ADPD410X_REG_INT_ENABLE_XD   	0x0014
#define ADPD410X_REG_INT_ENABLE_YD   	0x0015
#define ADPD410X_REG_INT_ENABLE_XL0  	0x0016
#define ADPD410X_REG_INT_ENABLE_XL1	0x0017
#define ADPD410X_REG_INT_ENABLE_YL0	0x001a
#define ADPD410X_REG_INT_ENABLE_YL1	0x001b
#define ADPD410X_REG_FIFO_STATUS_BYTES	0x001e
#define ADPD410X_REG_INPUT_SLEEP     	0x0020
#define ADPD410X_REG_INPUT_CFG       	0x0021
#define ADPD410X_REG_GPIO_CFG        	0x0022
#define ADPD410X_REG_GPIO01          	0x0023
#define ADPD410X_REG_GPIO23          	0x0024
#define ADPD410X_REG_GPIO_IN         	0x0025
#define ADPD410X_REG_GPIO_EXT           0x0026
#define ADPD410X_REG_DATA_HOLD_FLAG  	0x002E
#define ADPD410X_REG_FIFO_DATA       	0x002F
#define ADPD410X_REG_SIGNAL1_L(ts)     	(0x0030 + (ts) * 0x08)
#define ADPD410X_REG_SIGNAL1_H(ts)     	(0x0031 + (ts) * 0x08)
#define ADPD410X_REG_SIGNAL2_L(ts)     	(0x0032 + (ts) * 0x08)
#define ADPD410X_REG_SIGNAL2_H(ts)     	(0x0033 + (ts) * 0x08)
#define ADPD410X_REG_DARK1_L(ts)       	(0x0034 + (ts) * 0x08)
#define ADPD410X_REG_DARK1_H(ts)       	(0x0035 + (ts) * 0x08)
#define ADPD410X_REG_DARK2_L(ts)       	(0x0036 + (ts) * 0x08)
#define ADPD410X_REG_DARK2_H(ts)       	(0x0037 + (ts) * 0x08)
#define ADPD410X_REG_IO_ADJUST        	0x00B4
#define ADPD410X_REG_I2C_KEY         	0x00B6
#define ADPD410X_REG_I2C_ADDR        	0x00B7
#define ADPD410X_REG_TS_CTRL(ts)       	(0x0100 + (ts) * 0x20)
#define ADPD410X_REG_TS_PATH(ts)       	(0x0101 + (ts) * 0x20)
#define ADPD410X_REG_INPUTS(ts)        	(0x0102 + (ts) * 0x20)
#define ADPD410X_REG_CATHODE(ts)       	(0x0103 + (ts) * 0x20)
#define ADPD410X_REG_AFE_TRIM(ts)      	(0x0104 + (ts) * 0x20)
#define ADPD410X_REG_LED_POW12(ts)     	(0x0105 + (ts) * 0x20)
#define ADPD410X_REG_LED_POW34(ts)     	(0x0106 + (ts) * 0x20)
#define ADPD410X_REG_COUNTS(ts)        	(0x0107 + (ts) * 0x20)
#define ADPD410X_REG_PERIOD(ts)        	(0x0108 + (ts) * 0x20)
#define ADPD410X_REG_LED_PULSE(ts)     	(0x0109 + (ts) * 0x20)
#define ADPD410X_REG_INTEG_WIDTH(ts)   	(0x010A + (ts) * 0x20)
#define ADPD410X_REG_INTEG_OFFSET(ts)  	(0x010B + (ts) * 0x20)
#define ADPD410X_REG_MOD_PULSE(ts)     	(0x010C + (ts) * 0x20)
#define ADPD410X_REG_PATTERN(ts)       	(0x010D + (ts) * 0x20)
#define ADPD410X_REG_ADC_OFF1(ts)      	(0x010E + (ts) * 0x20)
#define ADPD410X_REG_ADC_OFF2(ts)      	(0x010F + (ts) * 0x20)
#define ADPD410X_REG_DATA1(ts)         	(0x0110 + (ts) * 0x20)
#define ADPD410X_REG_DATA2(ts)         	(0x0111 + (ts) * 0x20)
#define ADPD410X_REG_DECIMATE(ts)      	(0x0112 + (ts) * 0x20)
#define ADPD410X_REG_DIGINT_LIT(ts)    	(0x0113 + (ts) * 0x20)
#define ADPD410X_REG_DIGINT_DARK(ts)   	(0x0114 + (ts) * 0x20)
#define ADPD410X_REG_THRESH_CFG(ts)    	(0x0115 + (ts) * 0x20)
#define ADPD410X_REG_THRESH0(ts)       	(0x0116 + (ts) * 0x20)
#define ADPD410X_REG_THRESH1(ts)       	(0x0117 + (ts) * 0x20)


/* ADPD410X_REG_FIFO_STATUS */
#define BITP_INT_STATUS_FIFO_FIFO_BYTE_COUNT    	0
#define BITP_INT_STATUS_FIFO_INT_FIFO_OFLOW     	13
#define BITP_INT_STATUS_FIFO_INT_FIFO_UFLOW     	14
#define BITP_INT_STATUS_FIFO_CLEAR_FIFO         	15
#define BITM_INT_STATUS_FIFO_FIFO_BYTE_COUNT    	0x07ff
#define BITM_INT_STATUS_FIFO_INT_FIFO_OFLOW     	0x2000
#define BITM_INT_STATUS_FIFO_INT_FIFO_UFLOW     	0x4000
#define BITM_INT_STATUS_FIFO_CLEAR_FIFO         	0x8000


/* ADPD410X_REG_INT_STATUS_DATA */
#define BITP_INT_STATUS_DATA_INT_DATA_A         	0
#define BITP_INT_STATUS_DATA_INT_DATA_B         	1
#define BITP_INT_STATUS_DATA_INT_DATA_C         	2
#define BITP_INT_STATUS_DATA_INT_DATA_D         	3
#define BITP_INT_STATUS_DATA_INT_DATA_E         	4
#define BITP_INT_STATUS_DATA_INT_DATA_F         	5
#define BITP_INT_STATUS_DATA_INT_DATA_G         	6
#define BITP_INT_STATUS_DATA_INT_DATA_H         	7
#define BITP_INT_STATUS_DATA_INT_DATA_I         	8
#define BITP_INT_STATUS_DATA_INT_DATA_J         	9
#define BITP_INT_STATUS_DATA_INT_DATA_K         	10
#define BITP_INT_STATUS_DATA_INT_DATA_L         	11
#define BITP_INT_STATUS_DATA_INT_FIFO_TH        	15
#define BITM_INT_STATUS_DATA_INT_DATA_A         	0x0001
#define BITM_INT_STATUS_DATA_INT_DATA_B         	0x0002
#define BITM_INT_STATUS_DATA_INT_DATA_C         	0x0004
#define BITM_INT_STATUS_DATA_INT_DATA_D         	0x0008
#define BITM_INT_STATUS_DATA_INT_DATA_E         	0x0010
#define BITM_INT_STATUS_DATA_INT_DATA_F         	0x0020
#define BITM_INT_STATUS_DATA_INT_DATA_G         	0x0040
#define BITM_INT_STATUS_DATA_INT_DATA_H         	0x0080
#define BITM_INT_STATUS_DATA_INT_DATA_I         	0x0100
#define BITM_INT_STATUS_DATA_INT_DATA_J         	0x0200
#define BITM_INT_STATUS_DATA_INT_DATA_K         	0x0400
#define BITM_INT_STATUS_DATA_INT_DATA_L         	0x0800
#define BITM_INT_STATUS_DATA_INT_FIFO_TH        	0x8000

/* ADPD410X_REG_INT_STATUS_LEV0 */
#define BITP_INT_STATUS_LEV0_INT_LEV0_A         	0
#define BITP_INT_STATUS_LEV0_INT_LEV0_B         	1
#define BITP_INT_STATUS_LEV0_INT_LEV0_C         	2
#define BITP_INT_STATUS_LEV0_INT_LEV0_D         	3
#define BITP_INT_STATUS_LEV0_INT_LEV0_E         	4
#define BITP_INT_STATUS_LEV0_INT_LEV0_F         	5
#define BITP_INT_STATUS_LEV0_INT_LEV0_G         	6
#define BITP_INT_STATUS_LEV0_INT_LEV0_H         	7
#define BITP_INT_STATUS_LEV0_INT_LEV0_I         	8
#define BITP_INT_STATUS_LEV0_INT_LEV0_J         	9
#define BITP_INT_STATUS_LEV0_INT_LEV0_K         	10
#define BITP_INT_STATUS_LEV0_INT_LEV0_L         	11
#define BITM_INT_STATUS_LEV0_INT_LEV0_A         	0x0001
#define BITM_INT_STATUS_LEV0_INT_LEV0_B         	0x0002
#define BITM_INT_STATUS_LEV0_INT_LEV0_C         	0x0004
#define BITM_INT_STATUS_LEV0_INT_LEV0_D         	0x0008
#define BITM_INT_STATUS_LEV0_INT_LEV0_E         	0x0010
#define BITM_INT_STATUS_LEV0_INT_LEV0_F         	0x0020
#define BITM_INT_STATUS_LEV0_INT_LEV0_G         	0x0040
#define BITM_INT_STATUS_LEV0_INT_LEV0_H         	0x0080
#define BITM_INT_STATUS_LEV0_INT_LEV0_I         	0x0100
#define BITM_INT_STATUS_LEV0_INT_LEV0_J         	0x0200
#define BITM_INT_STATUS_LEV0_INT_LEV0_K         	0x0400
#define BITM_INT_STATUS_LEV0_INT_LEV0_L         	0x0800

/* ADPD410X_REG_INT_STATUS_LEV1 */
#define BITP_INT_STATUS_LEV1_INT_LEV1_A         	0
#define BITP_INT_STATUS_LEV1_INT_LEV1_B         	1
#define BITP_INT_STATUS_LEV1_INT_LEV1_C         	2
#define BITP_INT_STATUS_LEV1_INT_LEV1_D         	3
#define BITP_INT_STATUS_LEV1_INT_LEV1_E         	4
#define BITP_INT_STATUS_LEV1_INT_LEV1_F         	5
#define BITP_INT_STATUS_LEV1_INT_LEV1_G         	6
#define BITP_INT_STATUS_LEV1_INT_LEV1_H         	7
#define BITP_INT_STATUS_LEV1_INT_LEV1_I         	8
#define BITP_INT_STATUS_LEV1_INT_LEV1_J         	9
#define BITP_INT_STATUS_LEV1_INT_LEV1_K         	10
#define BITP_INT_STATUS_LEV1_INT_LEV1_L         	11
#define BITM_INT_STATUS_LEV1_INT_LEV1_A         	0x0001
#define BITM_INT_STATUS_LEV1_INT_LEV1_B         	0x0002
#define BITM_INT_STATUS_LEV1_INT_LEV1_C         	0x0004
#define BITM_INT_STATUS_LEV1_INT_LEV1_D         	0x0008
#define BITM_INT_STATUS_LEV1_INT_LEV1_E         	0x0010
#define BITM_INT_STATUS_LEV1_INT_LEV1_F         	0x0020
#define BITM_INT_STATUS_LEV1_INT_LEV1_G         	0x0040
#define BITM_INT_STATUS_LEV1_INT_LEV1_H         	0x0080
#define BITM_INT_STATUS_LEV1_INT_LEV1_I         	0x0100
#define BITM_INT_STATUS_LEV1_INT_LEV1_J         	0x0200
#define BITM_INT_STATUS_LEV1_INT_LEV1_K         	0x0400
#define BITM_INT_STATUS_LEV1_INT_LEV1_L         	0x0800


/* ADPD410X_REG_FIFO_TH */
#define BITP_FIFO_CTL_FIFO_TH                   	0
#define BITM_FIFO_CTL_FIFO_TH                   	0x03ff

/* ADPD410X_REG_INT_ACLEAR */
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_A       	0
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_B       	1
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_C       	2
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_D       	3
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_E       	4
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_F       	5
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_G       	6
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_H       	7
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_I       	8
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_J       	9
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_K       	10
#define BITP_INT_ACLEAR_INT_ACLEAR_DATA_L       	11
#define BITP_INT_ACLEAR_INT_ACLEAR_FIFO         	15
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_A       	0x0001
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_B       	0x0002
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_C       	0x0004
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_D       	0x0008
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_E       	0x0010
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_F       	0x0020
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_G       	0x0040
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_H       	0x0080
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_I       	0x0100
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_J       	0x0200
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_K       	0x0400
#define BITM_INT_ACLEAR_INT_ACLEAR_DATA_L       	0x0800
#define BITM_INT_ACLEAR_INT_ACLEAR_FIFO         	0x8000

/* ADPD410X_REG_CHIP_ID */
#define BITP_CHIP_ID		                    	0
#define BITP_CHIP_VERSION                    		8
#define BITM_CHIP_ID		                    	0x00ff
#define BITM_CHIP_VERSION                    		0xff00
#define ADPD410X_CHIP_ID				0xC2

/* ADPD410X_REG_OSC32M */
#define BITP_OSC32M_OSC_32M_FREQ_ADJ            	0
#define BITM_OSC32M_OSC_32M_FREQ_ADJ            	0x00ff

/* ADPD410X_REG_OSC32M_CAL */
#define BITP_OSC32M_CAL_OSC_32M_CAL_COUNT       	0
#define BITP_OSC32M_CAL_OSC_32M_CAL_START       	15
#define BITM_OSC32M_CAL_OSC_32M_CAL_COUNT       	0x7fff
#define BITM_OSC32M_CAL_OSC_32M_CAL_START       	0x8000

/* ADPD410X_REG_OSC1M */
#define BITP_OSC1M_OSC_1M_FREQ_ADJ              	0
#define BITP_OSC1M_OSC_CLK_CAL_ENA              	10
#define BITM_OSC1M_OSC_1M_FREQ_ADJ              	0x03ff
#define BITM_OSC1M_OSC_CLK_CAL_ENA              	0x0400

/* ADPD410X_REG_OSC32K */
#define BITP_OSC32K_OSC_32K_ADJUST              	0
#define BITP_OSC32K_CAPTURE_TIMESTAMP           	15
#define BITM_OSC32K_OSC_32K_ADJUST              	0x003f
#define BITM_OSC32K_CAPTURE_TIMESTAMP           	0x8000

/* ADPD410X_REG_TS_FREQ */
#define BITP_TS_FREQ_TIMESLOT_PERIOD_L          	0
#define BITM_TS_FREQ_TIMESLOT_PERIOD_L          	0xffff

/* ADPD410X_REG_TS_FREQH */
#define BITP_TS_FREQH_TIMESLOT_PERIOD_H         	0
#define BITM_TS_FREQH_TIMESLOT_PERIOD_H         	0x007f

/* ADPD410X_REG_SYS_CTL */
#define BITP_SYS_CTL_OSC_32K_EN                 	0
#define BITP_SYS_CTL_OSC_1M_EN                  	1
#define BITP_SYS_CTL_LFOSC_SEL                  	2
#define BITP_SYS_CTL_RANDOM_SLEEP               	3
#define BITP_SYS_CTL_GO_SLEEP                   	4
#define BITP_SYS_CTL_ALT_CLK_GPIO               	6
#define BITP_SYS_CTL_ALT_CLOCKS                 	8
#define BITP_SYS_CTL_SW_RESET                   	15
#define BITM_SYS_CTL_OSC_32K_EN                 	0x0001
#define BITM_SYS_CTL_OSC_1M_EN                  	0x0002
#define BITM_SYS_CTL_LFOSC_SEL                  	0x0004
#define BITM_SYS_CTL_RANDOM_SLEEP               	0x0008
#define BITM_SYS_CTL_GO_SLEEP                   	0x0010
#define BITM_SYS_CTL_ALT_CLK_GPIO               	0x00c0
#define BITM_SYS_CTL_ALT_CLOCKS                 	0x0300
#define BITM_SYS_CTL_SW_RESET                   	0x8000

/* ADPD410X_REG_OPMODE */
#define BITP_OPMODE_OP_MODE                     	0
#define BITP_OPMODE_TIMESLOT_EN                 	8
#define BITM_OPMODE_OP_MODE                     	0x0001
#define BITM_OPMODE_TIMESLOT_EN                 	0x0f00

/* ADPD410X_REG_STAMP_L */
#define BITP_STAMP_L_TIMESTAMP_COUNT_L          	0
#define BITM_STAMP_L_TIMESTAMP_COUNT_L          	0xffff

/* ADPD410X_REG_STAMP_H */
#define BITP_STAMP_H_TIMESTAMP_COUNT_H          	0
#define BITM_STAMP_H_TIMESTAMP_COUNT_H          	0xffff

/* ADPD410X_REG_STAMPDELTA */
#define BITP_STAMPDELTA_TIMESTAMP_SLOT_DELTA    	0
#define BITM_STAMPDELTA_TIMESTAMP_SLOT_DELTA    	0xffff

/* ADPD410X_REG_INT_ENABLE_XD */
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_A       	0
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_B       	1
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_C       	2
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_D       	3
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_E       	4
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_F       	5
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_G       	6
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_H       	7
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_I       	8
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_J       	9
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_K       	10
#define BITP_INT_ENABLE_XD_INTX_EN_DATA_L       	11
#define BITP_INT_ENABLE_XD_INTX_EN_FIFO_OFLOW   	13
#define BITP_INT_ENABLE_XD_INTX_EN_FIFO_UFLOW   	14
#define BITP_INT_ENABLE_XD_INTX_EN_FIFO_TH      	15
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_A       	0x0001
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_B       	0x0002
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_C       	0x0004
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_D       	0x0008
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_E       	0x0010
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_F       	0x0020
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_G       	0x0040
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_H       	0x0080
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_I       	0x0100
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_J       	0x0200
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_K       	0x0400
#define BITM_INT_ENABLE_XD_INTX_EN_DATA_L       	0x0800
#define BITM_INT_ENABLE_XD_INTX_EN_FIFO_OFLOW   	0x2000
#define BITM_INT_ENABLE_XD_INTX_EN_FIFO_UFLOW   	0x4000
#define BITM_INT_ENABLE_XD_INTX_EN_FIFO_TH      	0x8000

/* ADPD410X_REG_INT_ENABLE_YD */
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_A       	0
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_B       	1
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_C       	2
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_D       	3
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_E       	4
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_F       	5
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_G       	6
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_H       	7
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_I       	8
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_J       	9
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_K       	10
#define BITP_INT_ENABLE_YD_INTY_EN_DATA_L       	11
#define BITP_INT_ENABLE_YD_INTY_EN_FIFO_OFLOW   	13
#define BITP_INT_ENABLE_YD_INTY_EN_FIFO_UFLOW   	14
#define BITP_INT_ENABLE_YD_INTY_EN_FIFO_TH      	15
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_A       	0x0001
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_B       	0x0002
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_C       	0x0004
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_D       	0x0008
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_E       	0x0010
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_F       	0x0020
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_G       	0x0040
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_H       	0x0080
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_I       	0x0100
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_J       	0x0200
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_K       	0x0400
#define BITM_INT_ENABLE_YD_INTY_EN_DATA_L       	0x0800
#define BITM_INT_ENABLE_YD_INTY_EN_FIFO_OFLOW   	0x2000
#define BITM_INT_ENABLE_YD_INTY_EN_FIFO_UFLOW   	0x4000
#define BITM_INT_ENABLE_YD_INTY_EN_FIFO_TH      	0x8000

/* ADPD410X_REG_INT_ENABLE_XL0 */
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_A      	0
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_B      	1
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_C      	2
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_D      	3
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_E      	4
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_F      	5
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_G      	6
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_H      	7
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_I      	8
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_J      	9
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_K      	10
#define BITP_INT_ENABLE_XL0_INTX_EN_LEV0_L      	11
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_A      	0x0001
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_B      	0x0002
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_C      	0x0004
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_D      	0x0008
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_E      	0x0010
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_F      	0x0020
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_G      	0x0040
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_H      	0x0080
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_I      	0x0100
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_J      	0x0200
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_K      	0x0400
#define BITM_INT_ENABLE_XL0_INTX_EN_LEV0_L      	0x0800

/* ADPD410X_REG_INT_ENABLE_XL1 */
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_A      	0
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_B      	1
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_C      	2
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_D      	3
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_E      	4
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_F      	5
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_G      	6
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_H      	7
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_I      	8
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_J      	9
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_K      	10
#define BITP_INT_ENABLE_XL1_INTX_EN_LEV1_L      	11
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_A      	0x0001
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_B      	0x0002
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_C      	0x0004
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_D      	0x0008
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_E      	0x0010
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_F      	0x0020
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_G      	0x0040
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_H      	0x0080
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_I      	0x0100
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_J      	0x0200
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_K      	0x0400
#define BITM_INT_ENABLE_XL1_INTX_EN_LEV1_L      	0x0800

/* ADPD410X_REG_INT_ENABLE_YL0 */
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_A      	0
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_B      	1
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_C      	2
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_D      	3
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_E      	4
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_F      	5
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_G      	6
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_H      	7
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_I      	8
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_J      	9
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_K      	10
#define BITP_INT_ENABLE_YL0_INTY_EN_LEV0_L      	11
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_A      	0x0001
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_B      	0x0002
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_C      	0x0004
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_D      	0x0008
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_E      	0x0010
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_F      	0x0020
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_G      	0x0040
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_H      	0x0080
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_I      	0x0100
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_J      	0x0200
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_K      	0x0400
#define BITM_INT_ENABLE_YL0_INTY_EN_LEV0_L      	0x0800

/* ADPD410X_REG_INT_ENABLE_YL1 */
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_A      	0
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_B      	1
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_C      	2
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_D      	3
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_E      	4
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_F      	5
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_G      	6
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_H      	7
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_I      	8
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_J      	9
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_K      	10
#define BITP_INT_ENABLE_YL1_INTY_EN_LEV1_L      	11
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_A      	0x0001
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_B      	0x0002
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_C      	0x0004
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_D      	0x0008
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_E      	0x0010
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_F      	0x0020
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_G      	0x0040
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_H      	0x0080
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_I      	0x0100
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_J      	0x0200
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_K      	0x0400
#define BITM_INT_ENABLE_YL1_INTY_EN_LEV1_L      	0x0800

/* ADPD410X_REG_FIFO_STATUS_BYTES */
#define BITP_FIFO_STATUS_BYTES_ENA_STAT_SUM     	0
#define BITP_FIFO_STATUS_BYTES_ENA_STAT_D1      	1
#define BITP_FIFO_STATUS_BYTES_ENA_STAT_D2      	2
#define BITP_FIFO_STATUS_BYTES_ENA_STAT_L0      	3
#define BITP_FIFO_STATUS_BYTES_ENA_STAT_L1      	4
#define BITP_FIFO_STATUS_BYTES_ENA_STAT_LX      	5
#define BITP_FIFO_STATUS_BYTES_ENA_STAT_TS1     	6
#define BITP_FIFO_STATUS_BYTES_ENA_STAT_TS2     	7
#define BITP_FIFO_STATUS_BYTES_ENA_STAT_TSX     	8
#define BITM_FIFO_STATUS_BYTES_ENA_STAT_SUM     	0x0001
#define BITM_FIFO_STATUS_BYTES_ENA_STAT_D1      	0x0002
#define BITM_FIFO_STATUS_BYTES_ENA_STAT_D2      	0x0004
#define BITM_FIFO_STATUS_BYTES_ENA_STAT_L0      	0x0008
#define BITM_FIFO_STATUS_BYTES_ENA_STAT_L1      	0x0010
#define BITM_FIFO_STATUS_BYTES_ENA_STAT_LX      	0x0020
#define BITM_FIFO_STATUS_BYTES_ENA_STAT_TS1     	0x0040
#define BITM_FIFO_STATUS_BYTES_ENA_STAT_TS2     	0x0080
#define BITM_FIFO_STATUS_BYTES_ENA_STAT_TSX     	0x0100

/* ADPD410X_REG_INPUT_SLEEP */
#define BITP_INPUT_SLEEP_INP_SLEEP_12           	0
#define BITP_INPUT_SLEEP_INP_SLEEP_34           	4
#define BITP_INPUT_SLEEP_INP_SLEEP_56           	8
#define BITP_INPUT_SLEEP_INP_SLEEP_78           	12
#define BITM_INPUT_SLEEP_INP_SLEEP_12           	0x000f
#define BITM_INPUT_SLEEP_INP_SLEEP_34           	0x00f0
#define BITM_INPUT_SLEEP_INP_SLEEP_56           	0x0f00
#define BITM_INPUT_SLEEP_INP_SLEEP_78           	0xf000

/* ADPD410X_REG_INPUT_CFG */
#define BITP_INPUT_CFG_PAIR12                   	0
#define BITP_INPUT_CFG_PAIR34                   	1
#define BITP_INPUT_CFG_PAIR56                   	2
#define BITP_INPUT_CFG_PAIR78                   	3
#define BITP_INPUT_CFG_VC1_SLEEP                	4
#define BITP_INPUT_CFG_VC2_SLEEP                	6
#define BITM_INPUT_CFG_PAIR12                   	0
#define BITM_INPUT_CFG_PAIR34                   	0x0002
#define BITM_INPUT_CFG_PAIR56                   	0x0004
#define BITM_INPUT_CFG_PAIR78                   	0x0008
#define BITM_INPUT_CFG_VC1_SLEEP                	0x0030
#define BITM_INPUT_CFG_VC2_SLEEP                	0x00c0

/* ADPD410X_REG_GPIO_CFG */
#define BITP_GPIO_CFG_GPIO_PIN_CFG0             	0
#define BITP_GPIO_CFG_GPIO_PIN_CFG1             	3
#define BITP_GPIO_CFG_GPIO_PIN_CFG2             	6
#define BITP_GPIO_CFG_GPIO_PIN_CFG3             	9
#define BITP_GPIO_CFG_GPIO_DRV                  	12
#define BITP_GPIO_CFG_GPIO_SLEW                 	14
#define BITM_GPIO_CFG_GPIO_PIN_CFG0             	0x0007
#define BITM_GPIO_CFG_GPIO_PIN_CFG1             	0x0038
#define BITM_GPIO_CFG_GPIO_PIN_CFG2             	0x01c0
#define BITM_GPIO_CFG_GPIO_PIN_CFG3             	0x0e00
#define BITM_GPIO_CFG_GPIO_DRV                  	0x3000
#define BITM_GPIO_CFG_GPIO_SLEW                 	0xc000

/* ADPD410X_REG_GPIO01 */
#define BITP_GPIO01_GPIOOUT0                    	0
#define BITP_GPIO01_GPIOOUT1                    	8
#define BITP_GPIO01_TIMESTAMP_INV               	14
#define BITP_GPIO01_TIMESTAMP_ALWAYS_EN         	15
#define BITM_GPIO01_GPIOOUT0                    	0x001f
#define BITM_GPIO01_GPIOOUT1                    	0x1f00
#define BITM_GPIO01_TIMESTAMP_INV               	0x4000
#define BITM_GPIO01_TIMESTAMP_ALWAYS_EN         	0x8000

/* ADPD410X_REG_GPIO23 */
#define BITP_GPIO23_GPIOOUT2                    	0
#define BITP_GPIO23_GPIOOUT3                    	8
#define BITP_GPIO23_EXT_SYNC_EN                 	14
#define BITM_GPIO23_GPIOOUT2                    	0x001f
#define BITM_GPIO23_GPIOOUT3                    	0x1f00
#define BITM_GPIO23_EXT_SYNC_EN                 	0x4000

/* ADPD410X_REG_GPIO_IN */
#define BITP_GPIO_IN_GPIO_INPUT                 	0
#define BITM_GPIO_IN_GPIO_INPUT                 	0x000f

/* ADPD410X_REG_GPIO_EXT */
#define BITP_GPIO_EXT_EXT_SYNC_GPIO                 	0
#define BITP_GPIO_EXT_EXT_SYNC_EN                 	2
#define BITP_GPIO_EXT_TIMESTAMP_GPIO                 	4
#define BITP_GPIO_EXT_TIMESTAMP_ALWAYS_EN               6
#define BITP_GPIO_EXT_TIMESTAMP_INV                 	7
#define BITP_GPIO_EXT_TS_GPIO_SLEEP                 	8
#define BITM_GPIO_EXT_EXT_SYNC_GPIO                 	0x0003
#define BITM_GPIO_EXT_EXT_SYNC_EN                 	0x0004
#define BITM_GPIO_EXT_TIMESTAMP_GPIO                 	0x0030
#define BITM_GPIO_EXT_TIMESTAMP_ALWAYS_EN               0x0040
#define BITM_GPIO_EXT_TIMESTAMP_INV                 	0x0080
#define BITM_GPIO_EXT_TS_GPIO_SLEEP                 	0x0100

/* ADPD410X_REG_DATA_HOLD_FLAG */
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_A         	0
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_B         	1
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_C         	2
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_D         	3
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_E         	4
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_F         	5
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_G         	6
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_H         	7
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_I         	8
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_J         	9
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_K         	10
#define BITP_DATA_HOLD_FLAG_HOLD_REGS_L         	11
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_A         	0x0001
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_B         	0x0002
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_C         	0x0004
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_D         	0x0008
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_E         	0x0010
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_F         	0x0020
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_G         	0x0040
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_H         	0x0080
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_I         	0x0100
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_J         	0x0200
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_K         	0x0400
#define BITM_DATA_HOLD_FLAG_HOLD_REGS_L         	0x0800

/* ADPD410X_REG_FIFO_DATA */
#define BITP_FIFO_DATA_FIFO_DATA                	0
#define BITM_FIFO_DATA_FIFO_DATA                	0xffff

/* ADPD410X_REG_SIGNAL1_L */
#define BITP_SIGNAL1_L_A_SIGNAL1_L           	 	0
#define BITM_SIGNAL1_L_A_SIGNAL1_L            		0xffff

/* ADPD410X_REG_SIGNAL1_H */
#define BITP_SIGNAL1_H_A_SIGNAL1_H           	 	0
#define BITM_SIGNAL1_H_A_SIGNAL1_H            		0xffff

/* ADPD410X_REG_SIGNAL2_L */
#define BITP_SIGNAL2_L_A_SIGNAL2_L           	 	0
#define BITM_SIGNAL2_L_A_SIGNAL2_L            		0xffff

/* ADPD410X_REG_SIGNAL2_H */
#define BITP_SIGNAL2_H_A_SIGNAL2_H            		0
#define BITM_SIGNAL2_H_A_SIGNAL2_H            		0xffff

/* ADPD410X_REG_DARK1_L */
#define BITP_DARK1_L_A_DARK1_L               	 	0
#define BITM_DARK1_L_A_DARK1_L                		0xffff

/* ADPD410X_REG_DARK1_H */
#define BITP_DARK1_H_A_DARK1_H      	          	0
#define BITM_DARK1_H_A_DARK1_H                		0xffff

/* ADPD410X_REG_DARK2_L */
#define BITP_DARK2_L_A_DARK2_L              	  	0
#define BITM_DARK2_L_A_DARK2_L                		0xffff

/* ADPD410X_REG_DARK2_H */
#define BITP_DARK2_H_A_DARK2_H          	      	0
#define BITM_DARK2_H_A_DARK2_H	                	0xffff

/* ADPD410X_REG_IO_ADJUST */
#define BITP_IO_ADJUST_SPI_DRV				0
#define BITP_IO_ADJUST_SPI_SLEW				2
#define BITM_IO_ADJUST_SPI_DRV				0x0003
#define BITM_IO_ADJUST_SPI_SLEW				0x000c

/* ADPD410X_REG_I2C_KEY */
#define BITP_I2C_KEY_I2C_KEY				0
#define BITP_I2C_KEY_I2C_KEY_MATCH			12
#define BITM_I2C_KEY_I2C_KEY				0x0fff
#define BITM_I2C_KEY_I2C_KEY_MATCH			0xf000

/* ADPD410X_REG_I2C_ADDR */
#define BITP_I2C_ADDR_I2C_SLAVE_ADDR			1
#define BITP_I2C_ADDR_I2C_SLAVE_KEY2			8
#define BITM_I2C_ADDR_I2C_SLAVE_ADDR			0x00fe
#define BITM_I2C_ADDR_I2C_SLAVE_KEY2			0xff00

/* ADPD410X_REG_TS_CTRL */
#define BITP_TS_CTRL_A_TIMESLOT_OFFSET        		0
#define BITP_TS_CTRL_A_INPUT_R_SELECT         		10
#define BITP_TS_CTRL_A_SAMPLE_TYPE            		12
#define BITP_TS_CTRL_A_CH2_EN                 		14
#define BITP_TS_CTRL_A_SUBSAMPLE                    15
#define BITM_TS_CTRL_A_TIMESLOT_OFFSET        		0x03ff
#define BITM_TS_CTRL_A_INPUT_R_SELECT         		0x0c00
#define BITM_TS_CTRL_A_SAMPLE_TYPE            		0x3000
#define BITM_TS_CTRL_A_CH2_EN                 		0x4000
#define BITM_TS_CTRL_A_SUBSAMPLE                    0x8000

/* ADPD410X_REG_TS_PATH */
#define BITP_TS_PATH_A_AFE_PATH_CFG          	 	0
#define BITP_TS_PATH_A_PRE_WIDTH              		12
#define BITM_TS_PATH_A_AFE_PATH_CFG           		0x01ff
#define BITM_TS_PATH_A_PRE_WIDTH              		0xf000

/* ADPD410X_REG_INPUTS */
#define BITP_INPUTS_A_INP12                 	  	0
#define BITP_INPUTS_A_INP34                   		4
#define BITP_INPUTS_A_INP56                   		8
#define BITP_INPUTS_A_INP78                   		12
#define BITM_INPUTS_A_INP12                  		0x000f
#define BITM_INPUTS_A_INP34                   		0x00f0
#define BITM_INPUTS_A_INP56                   		0x0f00
#define BITM_INPUTS_A_INP78                   		0xf000

/* ADPD410X_REG_CATHODE */
#define BITP_CATHODE_A_VC1_SEL                		0
#define BITP_CATHODE_A_VC1_ALT                		2
#define BITP_CATHODE_A_VC1_PULSE              		4
#define BITP_CATHODE_A_VC2_SEL                		6
#define BITP_CATHODE_A_VC2_ALT                		8
#define BITP_CATHODE_A_VC2_PULSE              		10
#define BITP_CATHODE_A_PRECON                 		12
#define BITM_CATHODE_A_VC1_SEL                		0x0003
#define BITM_CATHODE_A_VC1_ALT                		0x000c
#define BITM_CATHODE_A_VC1_PULSE              		0x0030
#define BITM_CATHODE_A_VC2_SEL                		0x00c0
#define BITM_CATHODE_A_VC2_ALT                		0x0300
#define BITM_CATHODE_A_VC2_PULSE              		0x0c00
#define BITM_CATHODE_A_PRECON                 		0x7000

/* ADPD410X_REG_AFE_TRIM */
#define BITP_AFE_TRIM_A_TIA_GAIN_CH1          		0
#define BITP_AFE_TRIM_A_TIA_GAIN_CH2          		3
#define BITP_AFE_TRIM_A_VREF_PULSE_VAL        		6
#define BITP_AFE_TRIM_A_AFE_TRIM_VREF         		8
#define BITP_AFE_TRIM_A_VREF_PULSE            		10
#define BITP_AFE_TRIM_A_CH1_TRIM_INT          		11
#define BITP_AFE_TRIM_A_CH2_TRIM_INT          		13
#define BITP_AFE_TRIM_A_TIA_CEIL_DETECT_EN          	15
#define BITM_AFE_TRIM_A_TIA_GAIN_CH1          		0x0007
#define BITM_AFE_TRIM_A_TIA_GAIN_CH2          		0x0038
#define BITM_AFE_TRIM_A_VREF_PULSE_VAL        		0x00c0
#define BITM_AFE_TRIM_A_AFE_TRIM_VREF         		0x0300
#define BITM_AFE_TRIM_A_VREF_PULSE            		0x0400
#define BITM_AFE_TRIM_A_CH1_TRIM_INT          		0x1800
#define BITM_AFE_TRIM_A_CH2_TRIM_INT          		0x6000
#define BITM_AFE_TRIM_A_TIA_CEIL_DETECT_EN          	0x8000

/* ADPD410X_REG_LED_POW12 */
#define BITP_LED_POW12_A_LED_CURRENT1         		0
#define BITP_LED_POW12_A_LED_DRIVESIDE1       		7
#define BITP_LED_POW12_A_LED_CURRENT2         		8
#define BITP_LED_POW12_A_LED_DRIVESIDE2       		15
#define BITM_LED_POW12_A_LED_CURRENT1         		0x007f
#define BITM_LED_POW12_A_LED_DRIVESIDE1       		0x0080
#define BITM_LED_POW12_A_LED_CURRENT2         		0x7f00
#define BITM_LED_POW12_A_LED_DRIVESIDE2       		0x8000

/* ADPD410X_REG_LED_POW34 */
#define BITP_LED_POW34_A_LED_CURRENT3         		0
#define BITP_LED_POW34_A_LED_DRIVESIDE3       		7
#define BITP_LED_POW34_A_LED_CURRENT4         		8
#define BITP_LED_POW34_A_LED_DRIVESIDE4       		15
#define BITM_LED_POW34_A_LED_CURRENT3         		0x007f
#define BITM_LED_POW34_A_LED_DRIVESIDE3       		0x0080
#define BITM_LED_POW34_A_LED_CURRENT4         		0x7f00
#define BITM_LED_POW34_A_LED_DRIVESIDE4       		0x8000

/* ADPD410X_REG_COUNTS */
#define BITP_COUNTS_A_NUM_REPEAT              		0
#define BITP_COUNTS_A_NUM_INT                 		8
#define BITM_COUNTS_A_NUM_REPEAT              		0x00ff
#define BITM_COUNTS_A_NUM_INT                 		0xff00

/* ADPD410X_REG_PERIOD */
#define BITP_PERIOD_A_MIN_PERIOD              		0
#define BITP_PERIOD_A_MOD_TYPE                		12
#define BITM_PERIOD_A_MIN_PERIOD              		0x03ff
#define BITM_PERIOD_A_MOD_TYPE                		0x3000

/* ADPD410X_REG_LED_PULSE */
#define BITP_LED_PULSE_A_LED_OFFSET           		0
#define BITP_LED_PULSE_A_LED_WIDTH            		8
#define BITM_LED_PULSE_A_LED_OFFSET           		0x00ff
#define BITM_LED_PULSE_A_LED_WIDTH            		0xff00

/* ADPD410X_REG_INTEG_WIDTH */
#define BITP_INTEG_WIDTH_A_INTEG_WIDTH        		0
#define BITP_INTEG_WIDTH_A_ADC_COUNT          		6
#define BITP_INTEG_WIDTH_A_CH1_AMP_DISABLE    		8
#define BITP_INTEG_WIDTH_A_AFE_INT_C_BUF      		11
#define BITP_INTEG_WIDTH_A_CH2_AMP_DISABLE    		12
#define BITP_INTEG_WIDTH_A_SINGLE_INTEG       		15
#define BITM_INTEG_WIDTH_A_INTEG_WIDTH        		0x001f
#define BITM_INTEG_WIDTH_A_ADC_COUNT          		0x00c0
#define BITM_INTEG_WIDTH_A_CH1_AMP_DISABLE    		0x0700
#define BITM_INTEG_WIDTH_A_AFE_INT_C_BUF      		0x0800
#define BITM_INTEG_WIDTH_A_CH2_AMP_DISABLE    		0x7000
#define BITM_INTEG_WIDTH_A_SINGLE_INTEG       		0x8000

/* ADPD410X_REG_INTEG_OFFSET */
#define BITP_INTEG_OFFSET_A_INTEG_OFFSET      		0
#define BITM_INTEG_OFFSET_A_INTEG_OFFSET      		0x1fff
#define BITP_INTEG_OFFSET_A_INTEG_OFFSET_UPPER      5
#define BITM_INTEG_OFFSET_A_INTEG_OFFSET_UPPER      0xff

/* ADPD410X_REG_MOD_PULSE */
#define BITP_MOD_PULSE_A_MOD_OFFSET           		0
#define BITP_MOD_PULSE_A_MOD_WIDTH            		8
#define BITM_MOD_PULSE_A_MOD_OFFSET           		0x00ff
#define BITM_MOD_PULSE_A_MOD_WIDTH            		0xff00

/* ADPD410X_REG_PATTERN */
#define BITP_PATTERN_A_REVERSE_INTEG         	 	0
#define BITP_PATTERN_A_SUBTRACT               		4
#define BITP_PATTERN_A_MOD_DISABLE            		8
#define BITP_PATTERN_A_LED_DISABLE           	 	12
#define BITM_PATTERN_A_REVERSE_INTEG          		0x000f
#define BITM_PATTERN_A_SUBTRACT               		0x00f0
#define BITM_PATTERN_A_MOD_DISABLE            		0x0f00
#define BITM_PATTERN_A_LED_DISABLE            		0xf000

/* ADPD410X_REG_ADC_OFF1 */
#define BITP_ADC_OFF1_A_CH1_ADC_ADJUST        		0
#define BITM_ADC_OFF1_A_CH1_ADC_ADJUST        		0x3fff

/* ADPD410X_REG_ADC_OFF2 */
#define BITP_ADC_OFF2_A_CH2_ADC_ADJUST        		0
#define BITP_ADC_OFF2_A_ZERO_ADJUST       		15
#define BITM_ADC_OFF2_A_CH2_ADC_ADJUST        		0x3fff
#define BITM_ADC_OFF2_A_ZERO_ADJUST		       	0x8000

/* ADPD410X_REG_DATA1 */
#define BITP_DATA1_A_SIGNAL_SIZE              		0
#define BITP_DATA1_A_SIGNAL_SHIFT             		3
#define BITP_DATA1_A_DARK_SIZE                		8
#define BITP_DATA1_A_DARK_SHIFT               		11
#define BITM_DATA1_A_SIGNAL_SIZE              		0x0007
#define BITM_DATA1_A_SIGNAL_SHIFT             		0x00f8
#define BITM_DATA1_A_DARK_SIZE                		0x0700
#define BITM_DATA1_A_DARK_SHIFT               		0xf800

/* ADPD410X_REG_DATA2 */
#define BITP_DATA2_A_LIT_SIZE                 		0
#define BITP_DATA2_A_LIT_SHIFT                		3
#define BITM_DATA2_A_LIT_SIZE                 		0x0007
#define BITM_DATA2_A_LIT_SHIFT                		0x00f8

/* ADPD410X_REG_DECIMATE */
#define BITP_DECIMATE_A_DECIMATE_TYPE         		0
#define BITP_DECIMATE_A_DECIMATE_FACTOR       		4
#define BITM_DECIMATE_A_DECIMATE_TYPE         		0x000f
#define BITM_DECIMATE_A_DECIMATE_FACTOR       		0x07f0

/* ADPD410X_REG_DIGINT_LIT */
#define BITP_DIGINT_LIT_A_LIT_OFFSET          		0
#define BITM_DIGINT_LIT_A_LIT_OFFSET          		0x01ff

/* ADPD410X_REG_DIGINT_DARK */
#define BITP_DIGINT_DARK_A_DARK1_OFFSET       		0
#define BITP_DIGINT_DARK_A_DARK2_OFFSET       		7
#define BITM_DIGINT_DARK_A_DARK1_OFFSET       		0x007f
#define BITM_DIGINT_DARK_A_DARK2_OFFSET       		0xff80

/* ADPD410X_REG_THRESH_CFG */
#define BITP_THRESH_CFG_A_THRESH0_TYPE        		0
#define BITP_THRESH_CFG_A_THRESH0_DIR         		2
#define BITP_THRESH_CFG_A_THRESH0_CHAN       		3
#define BITP_THRESH_CFG_A_THRESH1_TYPE        		4
#define BITP_THRESH_CFG_A_THRESH1_DIR         		6
#define BITP_THRESH_CFG_A_THRESH1_CHAN        		7
#define BITM_THRESH_CFG_A_THRESH0_TYPE        		0x0003
#define BITM_THRESH_CFG_A_THRESH0_DIR         		0x0004
#define BITM_THRESH_CFG_A_THRESH0_CHAN        		0x0008
#define BITM_THRESH_CFG_A_THRESH1_TYPE        		0x0030
#define BITM_THRESH_CFG_A_THRESH1_DIR         		0x0040
#define BITM_THRESH_CFG_A_THRESH1_CHAN        		0x0080

/* ADPD410X_REG_THRESH0 */
#define BITP_THRESH0_A_THRESH0_VALUE         	 	0
#define BITP_THRESH0_A_THRESH0_SHIFT        	  	8
#define BITM_THRESH0_A_THRESH0_VALUE         	 	0x00ff
#define BITM_THRESH0_A_THRESH0_SHIFT          		0x1f00

/* ADPD410X_REG_THRESH1 */
#define BITP_THRESH1_A_THRESH1_VALUE          		0
#define BITP_THRESH1_A_THRESH1_SHIFT          		8
#define BITM_THRESH1_A_THRESH1_VALUE          		0x00ff
#define BITM_THRESH1_A_THRESH1_SHIFT          		0x1f00

#define ADPD410X_LOW_FREQ_OSCILLATOR_FREQ1		1000000
#define ADPD410X_LOW_FREQ_OSCILLATOR_FREQ2		32768
#define ADPD410X_HIGH_FREQ_OSCILLATOR_FREQ		32000000

#define ADPD410X_MAX_SLOT_NUMBER			12
#define ADPD410X_LED_CURR_LSB				1.333
#define ADPD410X_MAX_NUM_INT                255
#define ADPD410X_MAX_PULSE_LENGTH           255
#define ADPD410X_MAX_INTEG_OS               255
#define ADPD410X_FIFO_DEPTH                 512
#define ADPD410X_MAX_SAMPLING_FREQ          9000

#define ADPD410X_UPPDER_BYTE_SPI_MASK			0x7f80
#define ADPD410X_LOWER_BYTE_SPI_MASK			0xfe
#define ADPD410X_UPPDER_BYTE_I2C_MASK			0x7f00
#define ADPD410X_LOWER_BYTE_I2C_MASK			0xff

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `phy_comm_dev` union is designed to encapsulate a physical
 * communication handler, allowing for either SPI or I2C communication
 * interfaces. It provides a flexible way to manage different
 * communication protocols by using a single data structure, where only
 * one of the handlers (either SPI or I2C) is active at any given time.
 * This union is particularly useful in scenarios where a device can be
 * interfaced using multiple communication protocols, enabling the
 * software to switch between them as needed.
 *
 * @param spi_phy_dev Pointer to a SPI handler structure.
 * @param i2c_phy_dev Pointer to an I2C handler structure.
 ******************************************************************************/
union phy_comm_dev {
	/** SPI handler */
	struct no_os_spi_desc *spi_phy_dev;
	/** I2C handler */
	struct no_os_i2c_desc *i2c_phy_dev;
};

/***************************************************************************//**
 * @brief The `phy_comm_init_param` is a union that encapsulates initialization
 * parameters for physical communication interfaces, specifically SPI and
 * I2C. It allows for the selection of either SPI or I2C initialization
 * parameters, depending on the communication protocol being used. This
 * union is useful in scenarios where a device can be configured to
 * communicate over either SPI or I2C, providing flexibility in the
 * initialization process.
 *
 * @param spi_phy_init This member is a structure for SPI initialization
 * parameters.
 * @param i2c_phy_init This member is a structure for I2C initialization
 * parameters.
 ******************************************************************************/
union phy_comm_init_param {
	/** SPI initialization structure */
	struct no_os_spi_init_param spi_phy_init;
	/** I2C initialization structure */
	struct no_os_i2c_init_param i2c_phy_init;
};

/***************************************************************************//**
 * @brief The `adpd410x_supported_dev` enumeration defines the set of devices
 * supported by the ADPD410x driver, specifically the ADPD4100 and
 * ADPD4101 devices. This enumeration is used to specify the type of
 * device being interfaced with, allowing the driver to handle device-
 * specific configurations and operations.
 *
 * @param ADPD4100 Represents the ADPD4100 device.
 * @param ADPD4101 Represents the ADPD4101 device.
 ******************************************************************************/
enum adpd410x_supported_dev {
	ADPD4100,
	ADPD4101
};

/***************************************************************************//**
 * @brief The `adpd410x_opmode` is an enumeration that defines the operational
 * modes of the ADPD410x device. It includes two modes:
 * `ADPD410X_STANDBY`, which is used when the device is in a standby
 * state for programming purposes, and `ADPD410X_GOMODE`, which is used
 * when the device is actively sampling data. This enumeration allows for
 * easy switching between these two modes, facilitating the control of
 * the device's operational state.
 *
 * @param ADPD410X_STANDBY Represents the standby mode, used for programming the
 * device.
 * @param ADPD410X_GOMODE Represents the active mode, used for sampling data.
 ******************************************************************************/
enum adpd410x_opmode {
	/** Standby mode, used for programming */
	ADPD410X_STANDBY,
	/** Active mode, used for sampling data */
	ADPD410X_GOMODE
};

/***************************************************************************//**
 * @brief The `adpd410x_ts_input_pair` is an enumeration that defines the
 * available input pair options for time slots in the ADPD410x device.
 * Each enumerator represents a specific pair of inputs that can be used
 * for data acquisition, allowing the user to select between different
 * input configurations for the device's time slots.
 *
 * @param ADPD410X_INP12 Use input pair 1 and 2.
 * @param ADPD410X_INP34 Use input pair 3 and 4.
 * @param ADPD410X_INP56 Use input pair 5 and 6.
 * @param ADPD410X_INP78 Use input pair 7 and 8.
 ******************************************************************************/
enum adpd410x_ts_input_pair {
	/** Use input pair 1 and 2 */
	ADPD410X_INP12,
	/** Use input pair 3 and 4 */
	ADPD410X_INP34,
	/** Use input pair 5 and 6 */
	ADPD410X_INP56,
	/** Use input pair 7 and 8 */
	ADPD410X_INP78
};

/***************************************************************************//**
 * @brief The `adpd410x_ts_input_opt` is an enumeration that defines various
 * configurations for connecting inputs to channels in a time slot for
 * the ADPD410x device. It provides options for connecting either or both
 * inputs to one of two channels, or disconnecting them entirely,
 * allowing for flexible input routing based on the specific application
 * requirements.
 *
 * @param ADPD410X_INaDIS_INbDIS Both inputs are disconnected.
 * @param ADPD410X_INaCH1_INbDIS First input is connected to channel 1.
 * @param ADPD410X_INaCH2_INbDIS First input is connected to channel 2.
 * @param ADPD410X_INaDIS_INbCH1 Second input is connected to channel 1.
 * @param ADPD410X_INaDIS_INbCH2 Second input is connected to channel 2.
 * @param ADPD410X_INaCH1_INbCH2 First input is connected to channel 1 and
 * second input to channel 2.
 * @param ADPD410X_INaCH2_INbCH1 First input is connected to channel 2 and
 * second input to channel 1.
 * @param ADPD410X_INaCH1_INbCH1 Both inputs are connected to channel 1.
 * @param ADPD410X_INaCH2_INbCH2 Both inputs are connected to channel 2.
 ******************************************************************************/
enum adpd410x_ts_input_opt {
	/** Both inputs disconnected */
	ADPD410X_INaDIS_INbDIS,
	/** First input connected to channel 1 */
	ADPD410X_INaCH1_INbDIS,
	/** First input connected to channel 2 */
	ADPD410X_INaCH2_INbDIS,
	/** Second input connected to channel 1 */
	ADPD410X_INaDIS_INbCH1,
	/** Second input connected to channel 2 */
	ADPD410X_INaDIS_INbCH2,
	/** First input -> channel 1, second input -> channel 2 */
	ADPD410X_INaCH1_INbCH2,
	/** First input -> channel 2, second input -> channel 1 */
	ADPD410X_INaCH2_INbCH1,
	/** First input + Second input -> channel 1 */
	ADPD410X_INaCH1_INbCH1,
	/** First input + Second input -> channel 2 */
	ADPD410X_INaCH2_INbCH2
};

/***************************************************************************//**
 * @brief The `adpd410x_ts_inputs` structure is used to configure the input
 * settings for a time slot in the ADPD410x device. It includes options
 * for selecting which input pair to use and how those inputs are
 * connected to the device's channels, allowing for flexible
 * configuration of the device's input handling capabilities.
 *
 * @param pair Specifies the input pair option for the time slot.
 * @param option Defines the input pair channel connection option for the time
 * slot.
 ******************************************************************************/
struct adpd410x_ts_inputs {
	/** Input pair option */
	enum adpd410x_ts_input_pair pair;
	/** Input pair t channel connection option */
	enum adpd410x_ts_input_opt option;
};

/***************************************************************************//**
 * @brief The `adpd410x_precon_opt` enumeration defines various preconditioning
 * options for time slot inputs in the ADPD410x device. These options
 * specify how the inputs should be prepared or conditioned before being
 * used, such as floating the inputs, preconditioning them to specific
 * voltage levels (VC1, VC2, VICM), or connecting them to the TIA input
 * or reference voltage. Additionally, there is an option to short the
 * differential pair of inputs.
 *
 * @param ADPD410X_FLOAT_INS Represents floating inputs.
 * @param ADPD410X_VC1 Preconditions inputs to VC1.
 * @param ADPD410X_VC2 Preconditions inputs to VC2.
 * @param ADPD410X_VICM Preconditions inputs to VICM.
 * @param ADPD410X_TIA_IN Preconditions inputs to TIA input.
 * @param ADPD410X_TIA_VREF Preconditions inputs to TIA reference voltage.
 * @param ADPD410X_SHORT_INS Preconditions inputs by shorting the differential
 * pair.
 ******************************************************************************/
enum adpd410x_precon_opt {
	/** Float inputs */
	ADPD410X_FLOAT_INS,
	/** Precondition inputs to VC1 */
	ADPD410X_VC1,
	/** Precondition inputs to VC2 */
	ADPD410X_VC2,
	/** Precondition inputs to VICM */
	ADPD410X_VICM,
	/** Precondition inputs to TIA input */
	ADPD410X_TIA_IN,
	/** Precondition inputs to TIA reference voltage */
	ADPD410X_TIA_VREF,
	/** Precondition inputs by shorting the differential pair */
	ADPD410X_SHORT_INS
};

/***************************************************************************//**
 * @brief The `adpd410x_tia_vref_volt` enumeration defines a set of constants
 * representing different reference voltage levels for the Transimpedance
 * Amplifier (TIA) in the ADPD410x device. These voltage levels are used
 * to configure the TIA's reference voltage, which is crucial for the
 * device's analog front-end performance. Each enumerator corresponds to
 * a specific voltage value, allowing for precise control over the TIA's
 * operating conditions.
 *
 * @param ADPD410X_TIA_VREF_1V1385 Represents a TIA reference voltage of 1.1385
 * V.
 * @param ADPD410X_TIA_VREF_1V012 Represents a TIA reference voltage of 1.012 V.
 * @param ADPD410X_TIA_VREF_0V8855 Represents a TIA reference voltage of 0.8855
 * V.
 * @param ADPD410X_TIA_VREF_1V256 Represents a TIA reference voltage of 1.256 V.
 ******************************************************************************/
enum adpd410x_tia_vref_volt {
	/** 1,1385 V */
	ADPD410X_TIA_VREF_1V1385,
	/** 1,012 V */
	ADPD410X_TIA_VREF_1V012,
	/** 0,8855 V */
	ADPD410X_TIA_VREF_0V8855,
	/** 1,256 V */
	ADPD410X_TIA_VREF_1V256
};

/***************************************************************************//**
 * @brief The `adpd410x_tia_vref_ref` enumeration defines various resistor gain
 * settings for the Transimpedance Amplifier (TIA) in the ADPD410x
 * device. Each enumerator corresponds to a specific resistance value,
 * which is used to configure the TIA's reference voltage, thereby
 * affecting the gain and sensitivity of the amplifier. This
 * configuration is crucial for optimizing the performance of the device
 * in different application scenarios.
 *
 * @param ADPD410X_TIA_VREF_200K Represents a TIA resistor gain setting of 200
 * kOhm.
 * @param ADPD410X_TIA_VREF_100K Represents a TIA resistor gain setting of 100
 * kOhm.
 * @param ADPD410X_TIA_VREF_50K Represents a TIA resistor gain setting of 50
 * kOhm.
 * @param ADPD410X_TIA_VREF_25K Represents a TIA resistor gain setting of 25
 * kOhm.
 * @param ADPD410X_TIA_VREF_12K5 Represents a TIA resistor gain setting of 12.5
 * kOhm.
 ******************************************************************************/
enum adpd410x_tia_vref_ref {
	/** 200 kOhm */
	ADPD410X_TIA_VREF_200K,
	/** 100 kOhm */
	ADPD410X_TIA_VREF_100K,
	/** 50 kOhm */
	ADPD410X_TIA_VREF_50K,
	/** 25 kOhm */
	ADPD410X_TIA_VREF_25K,
	/** 12,5 kOhm */
	ADPD410X_TIA_VREF_12K5
};

/***************************************************************************//**
 * @brief The `adpd410x_led_output_opt` is an enumeration that defines the
 * possible LED output options for the ADPD410x device. It provides two
 * options, `ADPD410X_OUTPUT_A` and `ADPD410X_OUTPUT_B`, which can be
 * used to configure the LED output behavior of the device. This
 * enumeration is part of the configuration settings for controlling LED
 * outputs in the ADPD410x series of devices.
 *
 * @param ADPD410X_OUTPUT_A Represents LED output option A.
 * @param ADPD410X_OUTPUT_B Represents LED output option B.
 ******************************************************************************/
enum adpd410x_led_output_opt {
	/** Option A */
	ADPD410X_OUTPUT_A,
	/** Option B */
	ADPD410X_OUTPUT_B
};

/***************************************************************************//**
 * @brief The `_adpd410x_led_control` structure is designed to manage the LED
 * output settings for the ADPD410x device. It combines the LED output
 * strength and the LED option into a single byte, optimizing the control
 * of LED parameters. The `let_current_select` field uses 7 bits to
 * define the strength of the LED output, while the `led_output_select`
 * field uses 1 bit to specify the LED option, as defined by the
 * `adpd410x_led_output_opt` enumeration. This compact representation
 * allows for efficient manipulation and storage of LED control settings.
 *
 * @param let_current_select Represents the LED output strength using 7 bits.
 * @param led_output_select Specifies the LED option using 1 bit, defined by the
 * enum adpd410x_led_output_opt.
 ******************************************************************************/
struct _adpd410x_led_control {
	/** LED output strength */
	uint8_t let_current_select : 7;
	/** LED option */
	enum adpd410x_led_output_opt led_output_select : 1;
};

/***************************************************************************//**
 * @brief The `adpd410x_led_control` is a union that provides two ways to access
 * LED control data: as a structured mapping of LED output options and
 * strength through the `fields` member, or as a raw byte value through
 * the `value` member. This design allows for flexible manipulation and
 * representation of LED control settings in the ADPD410X device.
 *
 * @param fields A structure that maps LED output options and strength to a
 * single byte.
 * @param value A single byte representing the LED control value.
 ******************************************************************************/
union adpd410x_led_control {
	/** LED control mapping */
	struct _adpd410x_led_control fields;
	/** LED control value */
	uint8_t value;
};

/***************************************************************************//**
 * @brief The `adpd410x_timeslots` enum defines a set of constants representing
 * different time slots (A through L) available in the ADPD410x device.
 * Each constant corresponds to a specific time slot that can be used for
 * configuring and managing the device's operation, particularly in the
 * context of data sampling and processing.
 *
 * @param ADPD410X_TS_A Represents time slot A.
 * @param ADPD410X_TS_B Represents time slot B.
 * @param ADPD410X_TS_C Represents time slot C.
 * @param ADPD410X_TS_D Represents time slot D.
 * @param ADPD410X_TS_E Represents time slot E.
 * @param ADPD410X_TS_F Represents time slot F.
 * @param ADPD410X_TS_G Represents time slot G.
 * @param ADPD410X_TS_H Represents time slot H.
 * @param ADPD410X_TS_I Represents time slot I.
 * @param ADPD410X_TS_J Represents time slot J.
 * @param ADPD410X_TS_K Represents time slot K.
 * @param ADPD410X_TS_L Represents time slot L.
 ******************************************************************************/
enum adpd410x_timeslots {
	/** Time slot A */
	ADPD410X_TS_A,
	/** Time slot B */
	ADPD410X_TS_B,
	/** Time slot C */
	ADPD410X_TS_C,
	/** Time slot D */
	ADPD410X_TS_D,
	/** Time slot E */
	ADPD410X_TS_E,
	/** Time slot F */
	ADPD410X_TS_F,
	/** Time slot G */
	ADPD410X_TS_G,
	/** Time slot H */
	ADPD410X_TS_H,
	/** Time slot I */
	ADPD410X_TS_I,
	/** Time slot J */
	ADPD410X_TS_J,
	/** Time slot K */
	ADPD410X_TS_K,
	/** Time slot L */
	ADPD410X_TS_L
};

/***************************************************************************//**
 * @brief The `adpd410x_timeslot_init` structure is used to initialize and
 * configure time slots for the ADPD410x device. It includes settings for
 * enabling ADC channels, configuring input options, setting TIA
 * reference voltages and resistor gains, and controlling LED outputs.
 * The structure also specifies patterns for LED pulses, the number of
 * bytes per time slot, decimation factors, and the number of ADC
 * integration cycles and LED pulses per cycle. This configuration is
 * crucial for setting up the device to accurately capture and process
 * sensor data in various time slots.
 *
 * @param enable_ch2 Enable ADC channel 2 for a time slot.
 * @param ts_inputs Time slot input configuration.
 * @param precon_option Time slot input precondition option.
 * @param afe_trim_opt TIA reference voltage.
 * @param vref_pulse_opt TIA alternative reference voltage for pulsing property.
 * @param chan1 TIA resistor gain setting for channel 1.
 * @param chan2 TIA resistor gain setting for channel 2.
 * @param pulse4_reverse LED pulse reverse pattern.
 * @param pulse4_subtract LED pulse subtraction pattern.
 * @param byte_no Bytes number for time slot.
 * @param dec_factor Decimate factor - 1.
 * @param led2 LED 2 output and current control.
 * @param led1 LED 1 output and current control.
 * @param led4 LED 4 output and current control.
 * @param led3 LED 3 output and current control.
 * @param adc_cycles ADC integration cycles per conversion.
 * @param repeats_no ADC number of LED pulses per cycle.
 ******************************************************************************/
struct adpd410x_timeslot_init {
	/** Enable ADC channel 2 for a time slot */
	bool enable_ch2;
	/** Time slot input configuration */
	struct adpd410x_ts_inputs ts_inputs;
	/** Time slot input precondition option */
	enum adpd410x_precon_opt precon_option;
	/** TIA reference voltage */
	enum adpd410x_tia_vref_volt afe_trim_opt;
	/** TIA alternative reference voltage for pulsing property */
	enum adpd410x_tia_vref_volt vref_pulse_opt;
	/** TIA resistor gain setting for channel 1 */
	enum adpd410x_tia_vref_ref chan1;
	/** TIA resistor gain setting for channel 2 */
	enum adpd410x_tia_vref_ref chan2;
	/** LED pulse reverse pattern */
	uint8_t pulse4_reverse;
	/** LED pulse subtracion pattern */
	uint8_t pulse4_subtract;
	/** Bytes number for time slot */
	uint8_t byte_no;
	/** Decimate factor - 1 */
	uint8_t dec_factor;
	/** LED 2 output and current control */
	union adpd410x_led_control led2;
	/** LED 1 output and current control */
	union adpd410x_led_control led1;
	/** LED 4 output and current control */
	union adpd410x_led_control led4;
	/** LED 3 output and current control */
	union adpd410x_led_control led3;
	/** ADC integration cycles per conversion */
	uint8_t adc_cycles;
	/** ADC number of LED pulses per cycle */
	uint8_t repeats_no;
};

/***************************************************************************//**
 * @brief The `adpd410x_clk_opt` is an enumeration that defines the clock
 * configuration options for the ADPD410x device. It provides four
 * different configurations for selecting between internal and external
 * low and high frequency oscillators. This allows the device to be
 * configured for different operational scenarios depending on the
 * availability and preference for internal or external clock sources.
 *
 * @param ADPD410X_INTLFO_INTHFO Use internal low frequency and high frequency
 * oscillators.
 * @param ADPD410X_EXTLFO_INTHFO Use external low frequency and internal high
 * frequency oscillator.
 * @param ADPD410X_INTLFO_EXTHFO Use internal low frequency and external high
 * frequency oscillator.
 * @param ADPD410X_GENLFO_EXTHFO Use external high frequency oscillator and
 * generate low frequency using it.
 ******************************************************************************/
enum adpd410x_clk_opt {
	/** Use internal low frequency and high frequency oscillators */
	ADPD410X_INTLFO_INTHFO,
	/** Use external low frequency and internal high frequency oscillator */
	ADPD410X_EXTLFO_INTHFO,
	/** Use internal low frequency and external high frequency oscillator */
	ADPD410X_INTLFO_EXTHFO,
	/** Use external high frequency oscillator and generate low frequency
	 *  using it */
	ADPD410X_GENLFO_EXTHFO
};

/***************************************************************************//**
 * @brief The `adpd410x_init_param` structure is used to initialize the ADPD410x
 * device driver. It includes parameters for setting up physical
 * communication interfaces (SPI or I2C), selecting the device type,
 * configuring clock options, and initializing GPIO pins. Additionally,
 * it allows specifying the frequency of an external low frequency
 * oscillator if one is used. This structure is essential for configuring
 * the device before it is used for data acquisition or other operations.
 *
 * @param dev_ops_init Contains the initialization parameters for physical
 * communication, either SPI or I2C.
 * @param dev_type Specifies the type of device supported by the driver.
 * @param clk_opt Defines the clock options for the device, including internal
 * and external oscillator configurations.
 * @param gpio0 Initialization parameters for GPIO 0.
 * @param gpio1 Initialization parameters for GPIO 1.
 * @param gpio2 Initialization parameters for GPIO 2.
 * @param gpio3 Initialization parameters for GPIO 3.
 * @param ext_lfo_freq Specifies the frequency of the external low frequency
 * oscillator, if applicable.
 ******************************************************************************/
struct adpd410x_init_param {
	/** Physical communication */
	union phy_comm_init_param dev_ops_init;
	/** Device option */
	enum adpd410x_supported_dev dev_type;
	/** Device clock option */
	enum adpd410x_clk_opt clk_opt;
	/** GPIO 0 initialization */
	struct no_os_gpio_init_param gpio0;
	/** GPIO 1 initialization */
	struct no_os_gpio_init_param gpio1;
	/** GPIO 2 initialization */
	struct no_os_gpio_init_param gpio2;
	/** GPIO 3 initialization */
	struct no_os_gpio_init_param gpio3;
	/** External low frequency oscillator frequency, if applicable */
	uint32_t ext_lfo_freq;
};

/***************************************************************************//**
 * @brief The `adpd410x_dev` structure is a device driver handler for the
 * ADPD410x series, encapsulating the necessary components for device
 * communication and configuration. It includes a union for physical
 * communication operations, allowing for either SPI or I2C interfaces,
 * and enumerations for device type and clock options. Additionally, it
 * manages GPIO handlers for up to four GPIOs and can store the frequency
 * of an external low frequency oscillator if applicable. This structure
 * is essential for initializing and controlling the ADPD410x device
 * within a software application.
 *
 * @param dev_ops Handles the physical communication through either SPI or I2C.
 * @param dev_type Specifies the type of device supported by the driver.
 * @param clk_opt Defines the clock option for the device, whether internal or
 * external.
 * @param gpio0 Pointer to the GPIO descriptor for GPIO 0.
 * @param gpio1 Pointer to the GPIO descriptor for GPIO 1.
 * @param gpio2 Pointer to the GPIO descriptor for GPIO 2.
 * @param gpio3 Pointer to the GPIO descriptor for GPIO 3.
 * @param ext_lfo_freq Frequency of the external low frequency oscillator, if
 * used.
 ******************************************************************************/
struct adpd410x_dev {
	/** Physical communication */
	union phy_comm_dev dev_ops;
	/** Device option */
	enum adpd410x_supported_dev dev_type;
	/** Device clock option */
	enum adpd410x_clk_opt clk_opt;
	/** GPIO 0 handler */
	struct no_os_gpio_desc *gpio0;
	/** GPIO 1 handler */
	struct no_os_gpio_desc *gpio1;
	/** GPIO 2 handler */
	struct no_os_gpio_desc *gpio2;
	/** GPIO 3 handler */
	struct no_os_gpio_desc *gpio3;
	/** External low frequency oscillator frequency, if applicable */
	uint32_t ext_lfo_freq;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function reads a 16-bit value from a specified register address
 * on the ADPD410x device. It is typically used to retrieve configuration
 * or status information from the device. The function requires a valid
 * device structure and a register address to read from. The result is
 * stored in the provided data pointer. It is important to ensure that
 * the device has been properly initialized before calling this function.
 * The function returns an error code if the read operation fails.
 *
 * @param dev A pointer to an adpd410x_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param address A 16-bit unsigned integer specifying the register address to
 * read from. Valid register addresses are defined by the
 * device's register map.
 * @param data A pointer to a 16-bit unsigned integer where the read data will
 * be stored. Must not be null.
 * @return Returns an int32_t value. A return value of 0 indicates success,
 * while a non-zero value indicates an error occurred during the read
 * operation.
 ******************************************************************************/
int32_t adpd410x_reg_read(struct adpd410x_dev *dev, uint16_t address,
			  uint16_t *data);

/***************************************************************************//**
 * @brief This function reads a specified number of bytes from a register of the
 * ADPD410x device. It is used when you need to retrieve multiple bytes
 * of data from a specific register address. The function requires a
 * valid device structure and a register address to read from. The number
 * of bytes to read must be specified and should not exceed 255 for
 * devices using I2C communication. The function will fill the provided
 * data buffer with the read bytes. It returns an error code if the
 * operation fails, such as when the device type is unsupported or if the
 * number of bytes exceeds the limit for I2C devices.
 *
 * @param dev A pointer to an adpd410x_dev structure representing the device.
 * Must not be null and should be properly initialized.
 * @param address The 16-bit register address from which to read data. Must be a
 * valid register address for the device.
 * @param data A pointer to a buffer where the read bytes will be stored. Must
 * not be null and should have enough space to hold num_bytes.
 * @param num_bytes The number of bytes to read from the register. Must be
 * between 1 and 255 for I2C devices; no specific limit for SPI
 * devices is mentioned.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int32_t adpd410x_reg_read_bytes(struct adpd410x_dev *dev, uint16_t address,
				uint8_t *data, uint16_t num_bytes);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specific
 * register address on the ADPD410x device. It requires a valid device
 * structure that has been properly initialized and configured. The
 * function supports both ADPD4100 and ADPD4101 device types,
 * automatically selecting the appropriate communication protocol (SPI or
 * I2C) based on the device type. It returns an error code if the device
 * type is unsupported or if the write operation fails. This function
 * should be called when a register value needs to be updated on the
 * device.
 *
 * @param dev A pointer to an adpd410x_dev structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param address A 16-bit unsigned integer specifying the register address to
 * write to. The address must be valid for the ADPD410x device.
 * @param data A 16-bit unsigned integer representing the data to be written to
 * the specified register.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A negative value indicates an error, such as an
 * unsupported device type or a communication failure.
 ******************************************************************************/
int32_t adpd410x_reg_write(struct adpd410x_dev *dev, uint16_t address,
			   uint16_t data);

/***************************************************************************//**
 * @brief Use this function to update specific bits of a register in the
 * ADPD410x device by applying a mask. It first reads the current value
 * of the register, applies the mask to clear the bits, and then writes
 * the new data shifted into the correct position. This function is
 * useful when only certain bits of a register need to be modified
 * without affecting the others. Ensure that the device is properly
 * initialized before calling this function. If the read operation fails,
 * the function returns an error code.
 *
 * @param dev A pointer to an adpd410x_dev structure representing the device.
 * Must not be null.
 * @param address The 16-bit address of the register to be written to. Must be a
 * valid register address for the device.
 * @param data The 16-bit data to be written to the register. Only the bits
 * specified by the mask will be written.
 * @param mask A 16-bit mask indicating which bits of the register should be
 * updated. Bits set to 1 in the mask will be updated with the
 * corresponding bits from data.
 * @return Returns 0 on success, or -1 if the read operation fails.
 ******************************************************************************/
int32_t adpd410x_reg_write_mask(struct adpd410x_dev *dev, uint16_t address,
				uint16_t data, uint16_t mask);

/***************************************************************************//**
 * @brief This function is used to perform a software reset on the ADPD410x
 * device, which is necessary to reinitialize the device to its default
 * state. It should be called when the device needs to be reset, such as
 * after a configuration change or to recover from an error state. The
 * function requires a valid device structure that has been properly
 * initialized. It returns an error code if the reset operation fails,
 * allowing the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an initialized `adpd410x_dev` structure representing
 * the device. This parameter must not be null, and the device must
 * be properly set up before calling this function.
 * @return Returns an `int32_t` value indicating the success or failure of the
 * operation. A non-zero return value indicates an error occurred during
 * the reset process.
 ******************************************************************************/
int32_t adpd410x_reset(struct adpd410x_dev *dev);

/***************************************************************************//**
 * @brief Use this function to configure the ADPD410x device to operate in a
 * specific mode, such as standby or active mode. This function should be
 * called when you need to change the device's operational state, for
 * example, to start or stop data sampling. Ensure that the device is
 * properly initialized before calling this function. The function will
 * return an error code if the operation fails.
 *
 * @param dev A pointer to an initialized adpd410x_dev structure representing
 * the device. Must not be null.
 * @param mode An enum value of type adpd410x_opmode specifying the desired
 * operation mode. Valid values are ADPD410X_STANDBY and
 * ADPD410X_GOMODE.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating failure.
 ******************************************************************************/
int32_t adpd410x_set_opmode(struct adpd410x_dev *dev,
			    enum adpd410x_opmode mode);

/***************************************************************************//**
 * @brief Use this function to obtain the current operational mode of an
 * ADPD410x device. It is essential to ensure that the device is properly
 * initialized before calling this function. The function reads the
 * operation mode from the device and stores it in the provided mode
 * variable. This function is useful for verifying the current state of
 * the device, especially when debugging or configuring the device for
 * specific tasks.
 *
 * @param dev A pointer to an adpd410x_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param mode A pointer to an adpd410x_opmode enum where the current operation
 * mode will be stored. Must not be null.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails. The mode parameter is updated with the current
 * operation mode on success.
 ******************************************************************************/
int32_t adpd410x_get_opmode(struct adpd410x_dev *dev,
			    enum adpd410x_opmode *mode);

/***************************************************************************//**
 * @brief Use this function to configure the last active timeslot on the
 * ADPD410x device, which determines the number of active timeslots for
 * data acquisition. This function should be called when you need to
 * adjust the timeslot configuration, typically during device setup or
 * reconfiguration. Ensure that the device is properly initialized before
 * calling this function. The function modifies the device's operation
 * mode register to reflect the specified timeslot.
 *
 * @param dev A pointer to an initialized adpd410x_dev structure representing
 * the device. Must not be null.
 * @param timeslot_no An enum value of type adpd410x_timeslots representing the
 * timeslot to be set as the last active one. Valid values
 * are ADPD410X_TS_A through ADPD410X_TS_L.
 * @return Returns an int32_t status code. A non-negative value indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int32_t adpd410x_set_last_timeslot(struct adpd410x_dev *dev,
				   uint8_t timeslot_no);

/***************************************************************************//**
 * @brief This function is used to obtain the last active timeslot configured on
 * the ADPD410x device. It should be called when you need to verify or
 * utilize the current timeslot setting of the device. The function
 * requires a valid device structure and a pointer to an enum where the
 * timeslot number will be stored. It is important to ensure that the
 * device has been properly initialized before calling this function. The
 * function will return an error code if the operation fails, which
 * should be checked to ensure successful execution.
 *
 * @param dev A pointer to an adpd410x_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param timeslot_no A pointer to an enum adpd410x_timeslots where the last
 * active timeslot number will be stored. Must not be null.
 * @return Returns an int32_t error code. A value of 0 indicates success, while
 * a non-zero value indicates an error occurred during the operation.
 ******************************************************************************/
int32_t adpd410x_get_last_timeslot(struct adpd410x_dev *dev,
				   enum adpd410x_timeslots *timeslot_no);

/***************************************************************************//**
 * @brief This function configures the sampling frequency of the ADPD410x
 * device. It should be called after the device has been properly
 * initialized and configured. The function calculates the appropriate
 * register values based on the provided sampling frequency and writes
 * them to the device. It handles different clock options and adjusts the
 * frequency settings accordingly. If the device is using an internal
 * oscillator, it reads the current oscillator settings to determine the
 * correct frequency. The function returns an error code if the register
 * read or write operations fail.
 *
 * @param dev A pointer to an initialized adpd410x_dev structure representing
 * the device. Must not be null.
 * @param sampling_freq The desired sampling frequency in Hz. Must be a positive
 * integer and should be within the supported range of the
 * device.
 * @return Returns 0 on success or a negative error code if an error occurs
 * during register read or write operations.
 ******************************************************************************/
int32_t adpd410x_set_sampling_freq(struct adpd410x_dev *dev,
				   uint32_t sampling_freq);

/***************************************************************************//**
 * @brief This function is used to obtain the current sampling frequency of the
 * ADPD410x device. It should be called when you need to know the
 * device's sampling rate, which is essential for configuring data
 * acquisition or processing tasks. The function requires a valid device
 * structure and a pointer to store the retrieved sampling frequency. It
 * handles different clock configurations and reads the necessary
 * registers to compute the frequency. Ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized adpd410x_dev structure representing
 * the device. Must not be null.
 * @param sampling_freq A pointer to a uint32_t variable where the function will
 * store the retrieved sampling frequency. Must not be
 * null.
 * @return Returns an int32_t indicating success (0) or an error code if the
 * operation fails. The sampling frequency is stored in the variable
 * pointed to by sampling_freq.
 ******************************************************************************/
int32_t adpd410x_get_sampling_freq(struct adpd410x_dev *dev,
				   uint32_t *sampling_freq);

/***************************************************************************//**
 * @brief This function is used to configure a specific time slot on the
 * ADPD410x device, allowing for customization of various parameters such
 * as input configuration, LED power, and ADC settings. It should be
 * called when setting up the device for data acquisition, ensuring that
 * the time slot number is within the range of enabled time slots. The
 * function requires a valid device structure and initialization
 * parameters, and it returns an error code if the configuration fails.
 *
 * @param dev A pointer to an adpd410x_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param timeslot_no An enum value of type adpd410x_timeslots specifying the
 * time slot to configure. Must be within the range of
 * enabled time slots on the device.
 * @param init A pointer to an adpd410x_timeslot_init structure containing the
 * configuration settings for the time slot. Must not be null and
 * should be properly populated with desired settings.
 * @return Returns an int32_t error code: 0 on success, or a negative error code
 * on failure, such as -EINVAL if the time slot number is invalid.
 ******************************************************************************/
int32_t adpd410x_timeslot_setup(struct adpd410x_dev *dev,
				enum adpd410x_timeslots timeslot_no,
				struct adpd410x_timeslot_init *init);

/***************************************************************************//**
 * @brief Use this function to determine the current number of bytes stored in
 * the FIFO of the ADPD410x device. This function is useful for managing
 * data flow and ensuring that the FIFO does not overflow. It must be
 * called with a valid device structure and a pointer to a uint16_t
 * variable where the byte count will be stored. The function will return
 * an error code if the read operation fails.
 *
 * @param dev A pointer to an adpd410x_dev structure representing the device.
 * Must not be null.
 * @param bytes A pointer to a uint16_t variable where the number of bytes in
 * the FIFO will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails. The number of bytes in the FIFO is stored in the location
 * pointed to by the bytes parameter.
 ******************************************************************************/
int32_t adpd410x_get_fifo_bytecount(struct adpd410x_dev *dev, uint16_t *bytes);

/***************************************************************************//**
 * @brief Use this function to read a specified number of samples from the
 * ADPD410x device's FIFO. It is essential to ensure that the `data`
 * pointer is not null and that the total number of bytes to be read does
 * not exceed the FIFO's depth. The function requires the device to be
 * properly initialized and configured before calling. It handles
 * different data widths and reads the data accordingly. If the data
 * width is greater than 4 or if the total bytes exceed the FIFO depth,
 * the function will return an error. This function is useful for
 * retrieving sensor data from the device in a structured manner.
 *
 * @param dev A pointer to an initialized `adpd410x_dev` structure representing
 * the device. Must not be null.
 * @param data A pointer to a buffer where the read samples will be stored. Must
 * not be null.
 * @param num_samples The number of samples to read from the FIFO. The total
 * bytes (num_samples * datawidth) must not exceed the FIFO
 * depth.
 * @param datawidth The width of each data sample in bytes. Valid values are 1
 * to 4. Values greater than 4 will result in an error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int32_t adpd410x_read_fifo(struct adpd410x_dev *dev, uint32_t *data,
			   uint16_t num_samples,
			   uint8_t datawidth);

/***************************************************************************//**
 * @brief This function is used to obtain a complete data packet from the
 * ADPD410x device, which includes data from all active time slots. It
 * should be called when the device is in an appropriate operational mode
 * to read data. The function reads the necessary registers to determine
 * the number of active time slots and whether dual channels are enabled,
 * then retrieves the data accordingly. It is important to ensure that
 * the device is properly initialized and configured before calling this
 * function. The function returns an error code if the data retrieval
 * fails.
 *
 * @param dev A pointer to an initialized adpd410x_dev structure representing
 * the device. Must not be null.
 * @param data A pointer to a uint32_t variable where the retrieved data packet
 * will be stored. Must not be null.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t adpd410x_get_data(struct adpd410x_dev *dev, uint32_t *data);

/***************************************************************************//**
 * @brief This function initializes and configures the ADPD410x device based on
 * the provided initialization parameters. It sets up the communication
 * interface (SPI or I2C) and configures GPIOs required for device
 * operation. The function must be called before any other operations on
 * the device to ensure proper setup. It handles device reset and
 * verifies the device ID to ensure the correct device is being
 * configured. If any step in the setup process fails, the function will
 * clean up any allocated resources and return an error code.
 *
 * @param device A pointer to a pointer of type `struct adpd410x_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A pointer to a `struct adpd410x_init_param` containing the
 * initialization parameters for the device. This includes
 * communication settings, GPIO configurations, and clock
 * options. The pointer must not be null, and the structure
 * must be properly initialized before calling the function.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the `device` pointer will point to a newly allocated and
 * initialized `adpd410x_dev` structure.
 ******************************************************************************/
int32_t adpd410x_setup(struct adpd410x_dev **device,
		       struct adpd410x_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADPD410x device when it is no longer needed. This includes closing
 * communication interfaces and freeing memory. It is essential to call
 * this function to prevent resource leaks after the device is no longer
 * in use. Ensure that the device pointer is valid and initialized before
 * calling this function.
 *
 * @param dev A pointer to an adpd410x_dev structure representing the device to
 * be removed. Must not be null. If the pointer is null, the function
 * returns an error code.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during resource deallocation.
 ******************************************************************************/
int32_t adpd410x_remove(struct adpd410x_dev *dev);

#endif /* ADPD410X_H_ */
