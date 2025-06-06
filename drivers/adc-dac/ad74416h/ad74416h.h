/***************************************************************************//**
 *   @file   ad74416h.h
 *   @brief  Header file of AD74416h Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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
#ifndef _AD74416H_H
#define _AD74416H_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "stdint.h"
#include "stdbool.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define AD74416H_N_CHANNELS             4

#define AD74416H_CH_A                   0
#define AD74416H_CH_B                   1
#define AD74416H_CH_C                   2
#define AD74416H_CH_D                   3

/** The value of the sense resistor in ohms */
#define AD74416H_RSENSE                 12
/** 16 bit ADC */
#define AD74416H_ADC_MAX_VALUE		16777215

/** Register map */
#define AD74416H_NOP				0x00
#define AD74416H_CH_FUNC_SETUP(x)               (0x01 + (x * 12))
#define AD74416H_ADC_CONFIG(x)                  (0x02 + (x * 12))
#define AD74416H_DIN_CONFIG0(x)			(0x03 + (x * 12))
#define AD74416H_DIN_CONFIG1(x)			(0x04 + (x * 12))
#define AD74416H_OUTPUT_CONFIG(x)               (0x05 + (x * 12))
#define AD74416H_RTD_CONFIG(x)			(0x06 + (x * 12))
#define AD74416H_FET_LKG_COMP(x)		(0x07 + (x * 12))
#define AD74416H_DO_EXT_CONFIG(x)		(0x08 + (x * 12))
#define AD74416H_I_BURNOUT_CONFIG(x)		(0x09 + (x * 12))
#define AD74416H_DAC_CODE(x)			(0x0A + (x * 12))
#define AD74416H_DAC_ACTIVE(x)			(0x0C + (x * 12))
#define AD74416H_GPIO_CONFIG(x)			(0x32 + x)
#define AD74416H_PWR_OPTIM_CONFIG		0x38
#define AD74416H_ADC_CONV_CTRL			0x39
#define AD74416H_DIAG_ASSIGN			0x3A
#define AD74416H_WTD_CONFIG			0x3B
#define AD74416H_DIN_COMP_OUT			0x3E
#define AD74416H_ALERT_STATUS			0x3F
#define AD74416H_LIVE_STATUS			0x40
#define AD74416H_ADC_RESULT_UPR(x)		(0x41 + (x * 2))
#define AD74416H_ADC_RESULT(x)			(0x42 + (x * 2))
#define AD74416H_ADC_DIAG_RESULT(x)		(0x49 + x)
#define AD74416H_LAST_ADC_RESULT_UPR		0x4D
#define AD74416H_LAST_ADC_RESULT		0x4E
#define AD74416H_DIN_COUNTER(x)			(0x50 + (x * 2))
#define AD74416H_SUPPLY_ALERT_STATUS		0x57
#define AD74416H_CHANNEL_ALERT_STATUS(x)	(0x58 + x)
#define AD74416H_ALERT_MASK			0x5C
#define AD74416H_SUPPLY_ALERT_MASK		0x5D
#define AD74416H_CHANNEL_ALERT_MASK(x)		(0x5E + x)
#define AD74416H_READ_SELECT			0x6E
#define AD74416H_BURST_READ_SEL			0x6F
#define AD74416H_THERM_RST			0x73
#define AD74416H_CMD_KEY			0x74
#define AD74416H_BORADCAST_CMD_KEY		0x75
#define AD74416H_SCRATCH(x)			(0x76 + x)
#define AD74416H_GENERIC_ID			0x7A
#define AD74416H_SILICON_REV			0x7B
#define AD74416H_SILICON_ID0			0x7D
#define AD74416H_SILICON_ID1			0x7E
#define AD74416H_HART_ALERT_STATUS(x)		(0x80 + (x * 16))
#define AD74416H_HART_RX(x)			(0x81 + (x * 16))
#define AD74416H_HART_TX(x)			(0x82 + (x * 16))
#define AD74416H_HART_FCR(x)			(0x83 + (x * 16))
#define AD74416H_HART_MCR(x)			(0x84 + (x * 16))
#define AD74416H_HART_RFC(x)			(0x85 + (x * 16))
#define AD74416H_HART_TFC(x)			(0x86 + (x * 16))
#define AD74416H_HART_ALERT_MASK(x)		(0x87 + (x * 16))
#define AD74416H_HART_CONFIG(x)			(0x88 + (x * 16))
#define AD74416H_HART_TX_PREM(x)		(0x89 + (x * 16))
#define AD74416H_HART_EVDET(x)			(0x8A + (x * 16))
#define AD74416H_HART_TX_GAIN(x)		(0x8B + (x * 16))
#define AD74416H_HART_GPIO_IF_CONFIG		0xC0
#define AD74416H_HART_GPIO_MON_CONFIG(x)	(0xC1 + x)

/** Software reset sequence */
#define AD74416H_CMD_KEY_RESET_1                0x15FA
#define AD74416H_CMD_KEY_RESET_2                0xAF51

#define AD74416H_SPI_RD_RET_INFO_MSK		NO_OS_BIT(8)
#define AD74416H_ERR_CLR_MSK			NO_OS_GENMASK(15, 0)
#define AD74416H_SPI_CRC_ERR_MSK		NO_OS_BIT(13)

/* AD74416H_CH_FUNC_SETUP */
#define AD74416H_CH_FUNC_SETUP_MSK		NO_OS_GENMASK(3, 0)

/* AD74416H_ADC_CONFIG */
#define AD74416H_ADC_CONV_RATE_MSK		NO_OS_GENMASK(11, 8)
#define AD74416H_ADC_CONV_RANGE_MSK		NO_OS_GENMASK(6, 4)
#define AD74416H_CONV_MUX_MSK			NO_OS_GENMASK(2, 0)

/* AD74416H_DIN_CONFIG0 */
#define AD74416H_COUNT_EN_MSK			NO_OS_BIT(15)
#define AD74416H_DIN_INV_COMP_OUT_MSK		NO_OS_BIT(14)
#define AD74416H_COMPARATOR_EN_MSK		NO_OS_BIT(13)
#define AD74416H_DIN_SINK_RANGE_MSK		NO_OS_BIT(12)
#define AD74416H_DIN_SINK_MSK			NO_OS_GENMASK(11, 7)
#define AD74416H_DEBOUNCE_MODE_MSK		NO_OS_BIT(6)
#define AD74416H_DEBOUNCE_TIME_MSK		NO_OS_GENMASK(4, 0)

/* AD74416H_DIN_CONFIG1 Register */
#define AD74416H_DIN_INPUT_SELECT_MSK		NO_OS_BIT(10)
#define AD74416H_DIN_SC_DET_EN_MSK		NO_OS_BIT(9)
#define AD74416H_DIN_OC_DET_EN_MSK		NO_OS_BIT(8)
#define AD74416H_DIN_THRESH_MODE_MSK		NO_OS_BIT(7)
#define AD74416H_COMP_THRESH_MSK		NO_OS_GENMASK(6, 0)

/** OUTPUT_CONFIGx Register */
#define AD74416H_AVDD_SELECT_MSK		NO_OS_GENMASK(15, 14)
#define AD74416H_ALARM_DEG_PERIOD_MSK		NO_OS_BIT(12)
#define AD74VOUT_4W_EN_MSK			NO_OS_BIT(11)
#define AD74416H_WAIT_LDAC_CMD_MSK		NO_OS_BIT(10)
#define AD74416H_VOUT_RANGE_MSK			NO_OS_BIT(7)
#define AD74416H_SLEW_EN_MSK			NO_OS_GENMASK(6, 5)
#define AD74416H_SLEW_LIN_STEP_MSK		NO_OS_GENMASK(4, 3)
#define AD74416H_SLEW_LIN_RATE_MSK		NO_OS_GENMASK(2, 1)
#define AD74416H_I_LIMIT_MSK			NO_OS_BIT(0)

/** RTD_CONFIG Register */
#define AD74416H_RTD_ADC_REF_MSK		NO_OS_BIT(3)
#define AD74416H_RTD_MODE_SEL_MSK		NO_OS_BIT(2)
#define AD74416H_RTD_EXC_SWAP_MSK		NO_OS_BIT(1)
#define AD74416H_RTD_CURRENT_MSK		NO_OS_BIT(0)

/** FET_LKG_COMP Register */
#define AD74416H_FET_SRC_LKG_COMP_EN_MSK	NO_OS_BIT(0)

/** DO_EXT_CONFIG Register */
#define AD74416H_DO_T2_MSK			NO_OS_GENMASK(12, 8)
#define AD74416H_DO_DATA_MSK			NO_OS_BIT(7)
#define AD74416H_DO_T1_MSK			NO_OS_GENMASK(6, 2)
#define AD74416H_DO_SRC_SEL_MSK			NO_OS_BIT(1)
#define AD74416H_DO_MODE_MSK			NO_OS_BIT(0)

/** I_BURNOUT_CONFIG Register */
#define AD74416H_BRN_SEN_VSENSEN_CURR_MSK	NO_OS_GENMASK(6, 5)
#define AD74416H_BRN_SEN_VSENSEN_POL_MSK	NO_OS_BIT(4)
#define AD74416H_BRN_VIOUT_CURR_MSK		NO_OS_GENAMSK(2, 1)
#define AD74416H_BRN_VIOUT_POL_MSK		NO_OS_BIT(0)

/** DAC_CODE Register*/
#define AD74416H_DAC_CODE_MSK			NO_OS_GENMASK(15, 0)

/** DAC_ACTIVE Register */
#define AD74416H_DAC_ACTIVE_MSK			NO_OS_GENMASK(15, 0)

/** ADC_CONV_CTRL Register */
#define AD74416H_ADC_RDY_CTRL_MSK		NO_OS_BIT(13)
#define AD74416H_CONV_RATE_DIAG_MSK		NO_OS_GENMASK(12, 10)
#define AD74416H_CONV_SEQ_MSK			NO_OS_GENMASK(9, 8)
#define AD74416H_DIAG_EN_MSK(x)			(NO_OS_BIT(x) << 4)
#define AD74416H_CH_EN_MSK(x)			NO_OS_BIT(x)

/** DIAG_ASSIGN register */
#define AD74416H_DIAG_ASSIGN_MSK(x)		(NO_OS_GENMASK(3, 0) << ((x) * 4))

/** GPIO_CONFIGx register */
#define AD74416H_DIN_DO_MSK			NO_OS_BIT(7, 6)
#define AD74416H_GPI_DATA_MSK			NO_OS_BIT(5)
#define AD74416H_GPO_DATA_MSK			NO_OS_BIT(4)
#define AD74416H_GP_WK_PD_EN_MSK		NO_OS_BIT(3)
#define AD74416H_GPIO_SELECT_MSK		NO_OS_GENMASK(2, 0)

/** PWR_OPTIM_CONFIG Register */
#define AD74416H_REF_EN_MSK			NO_OS_BIT(13)
#define AD74416H_SENSE_AGND_OPT_MSK		NO_OS_BIT(12)
#define AD74416H_SENSE_HF_OPT_D_MSK		NO_OS_BIT(11)
#define AD74416H_SENSE_HF_OPT_C_MSK		NO_OS_BIT(10)
#define AD74416H_SENSE_HF_OPT_B_MSK		NO_OS_BIT(9)
#define AD74416H_SENSE_HF_OPT_A_MSK		NO_OS_BIT(8)
#define AD74416H_SENSE_LF_OPT_D_MSK		NO_OS_BIT(7)
#define AD74416H_SENSE_LF_OPT_C_MSK		NO_OS_BIT(6)
#define AD74416H_SENSE_LF_OPT_B_MSK		NO_OS_BIT(5)
#define AD74416H_SENSE_LF_OPT_A_MSK		NO_OS_BIT(4)
#define AD74416H_VSENSEN_OPT_D_MSK		NO_OS_BIT(3)
#define AD74416H_VSENSEN_OPT_C_MSK		NO_OS_BIT(2)
#define AD74416H_VSENSEN_OPT_B_MSK		NO_OS_BIT(1)
#define AD74416H_VSENSEN_OPT_A_MSK		NO_OS_BIT(0)

/** WDT_CONFIG Register */
#define AD74416H_WDT_EN_MSK			NO_OS_BIT(4)
#define AD74416H_WDT_TIMEOUT_MSK		NO_OS_GENMASK(3, 0)

/** DIN_COMP_OUT Register */
#define AD74416H_DIN_COMP_OUT_D_MSK		NO_OS_BIT(3)
#define AD74416H_DIN_COMP_OUT_C_MSK		NO_OS_BIT(2)
#define AD74416H_DIN_COMP_OUT_B_MSK		NO_OS_BIT(1)
#define AD74416H_DIN_COMP_OUT_A_MSK		NO_OS_BIT(0)

/** ALERT_STATUS Register */
#define AD74416H_HART_ALERT_D_MSK		NO_OS_BIT(15)
#define AD74416H_HART_ALERT_C_MSK		NO_OS_BIT(14)
#define AD74416H_HART_ALERT_B_MSK		NO_OS_BIT(13)
#define AD74416H_HART_ALERT_A_MSK		NO_OS_BIT(12)
#define AD74416H_CHANNEL_ALERT_D_MSK		NO_OS_BIT(11)
#define AD74416H_CHANNEL_ALERT_C_MSK		NO_OS_BIT(10)
#define AD74416H_CHANNEL_ALERT_B_MSK		NO_OS_BIT(9)
#define AD74416H_CHANNEL_ALERT_A_MSK		NO_OS_BIT(8)
#define AD74416H_ADC_ERR_MSK			NO_OS_BIT(5)
#define AD74416H_TEMP_ALERT_MSK			NO_OS_BIT(4)
#define AD74416H_SPI_ERR_MSK			NO_OS_BIT(3)
#define AD74416H_SUPPLY_ERR_MSK			NO_OS_BIT(2)
#define AD74416H_RESET_OCCURRED_MSK		NO_OS_BIT(0)

/** LIVE_STATUS Register */
#define AD74416H_ANALOG_IO_STATUS_D_MSK		NO_OS_BIT(15)
#define AD74416H_ANALOG_IO_STATUS_C_MSK		NO_OS_BIT(14)
#define AD74416H_ANALOG_IO_STATUS_B_MSK		NO_OS_BIT(13)
#define AD74416H_ANALOG_IO_STATUS_A_MSK		NO_OS_BIT(12)
#define AD74416H_DO_STATUS_D_MSK		NO_OS_BIT(11)
#define AD74416H_DO_STATUS_C_MSK		NO_OS_BIT(10)
#define AD74416H_DO_STATUS_B_MSK		NO_OS_BIT(9)
#define AD74416H_DO_STATUS_A_MSK		NO_OS_BIT(8)
#define AD74416H_DIN_STATUS_D_MSK		NO_OS_BIT(7)
#define AD74416H_DIN_STATUS_C_MSK		NO_OS_BIT(6)
#define AD74416H_DIN_STATUS_B_MSK		NO_OS_BIT(5)
#define AD74416H_DIN_STATUS_A_MSK		NO_OS_BIT(4)
#define AD74416H_TEMP_ALERT_STATUS_MSK		NO_OS_BIT(3)
#define AD74416H_ADC_DATA_RDY_MSK		NO_OS_BIT(2)
#define AD74416H_ADC_BUSY_MSK			NO_OS_BIT(1)
#define AD74416H_SUPPLY_STATUS_MSK		NO_OS_BIT(0)

/* AD74416H_ADC_RESULT_UPRn Register */
#define AD74416H_CONV_RES_MUX_MSK		NO_OS_GENMASK(15, 13)
#define AD74416H_CONV_RES_RANGE_MSK		NO_OS_GENMASK(12, 10)
#define AD74416H_CONV_SEQ_COUNT_MSK		NO_OS_GENAMSK(9. 8)
#define AD74416H_CONV_RES_UPR_MSK		NO_OS_GENMASK(7, 0)

/* AD74416H_ADC_RESULTn Register */
#define AD74416H_CONV_RESULT_MSK		NO_OS_GENMASK(15, 0)

/* AD74416H_ADC_DIAG_RESULTn */
#define AD74416H_DIAG_RESULT_MSK		NO_OS_GENMASK(15, 0)

/* AD74416H_LAST_ADC_RESULT_UPRn Register */
#define AD74416H_LAST_CONV_CH_MSK		NO_OS_GENAMSK(10 8)
#define AD74416H_LAST_CONV_RES_UPR_MSK		NO_OS_GENMASK(7, 0)

/* AD74416H_ADC_RESULTn Register */
#define AD74416H_LAST_CONV_RES_MSK		NO_OS_GENMASK(15, 0)

/* AD74416H_DIN_COUNTER_UPRn Register */
#define AD74416H_DIN_CNT_UPR_MSK		NO_OS_GENMASK(15, 0)

/* AD74416H_DIN_COUNTERn Register */
#define AD74416H_DIN_CNT_MSK			NO_OS_GENMASK(15, 0)

/* AD74416H SUPPLY_ALERT_STATUS Register */
#define AD74416H_AVDD_HI_ERR_MSK		NO_OS_BIT(6)
#define AD74416H_AVDD_LO_ERR_MSK		NO_OS_BIT(5)
#define AD74416H_DO_VDD_ERR_MSK			NO_OS_BIT(4)
#define AD74416H_AVCC_ERR_MSK			NO_OS_BIT(3)
#define AD74416H_DVCC_ERR_MSK			NO_OS_BIT(2)
#define AD74416H_AVSS_ERR_MSK			NO_OS_BIT(1)
#define AD74416H_CAL_MEM_ERR_MSK		NO_OS_BIT(0)

/* AD74416H CHANNEL_ALERT_STATUS Register */
#define AD74416H_ANALOG_IO_OC_MSK		NO_OS_BIT(5)
#define AD74416H_ANALOG_IO_SC_MSK		NO_OS_BIT(4)
#define AD74416H_DO_TIMEOUT_MSK			NO_OS_BIT(3)
#define AD74416H_DO_SC_MSK			NO_OS_BIT(2)
#define AD74416H_DIN_OC_MSK			NO_OS_BIT(1)
#define AD74416H_DIN_SC_MSK			NO_OS_BIT(0)

/* AD74416H READ_SELECT Register */
#define AD74416H_READBACK_ADDR_MSK		NO_OS_GENMASK(8, 0)

/* AD74416H BURST_READ_SEL Register */
#define AD74416H_BURST_READ_SEL_MSK		NO_OS_GENMASK(15, 0)

/* AD74416H THERM_RST Register */
#define AD74416H_THERM_RST_EN_MSK		NO_OS_BIT(0)

/* AD74416H CMD_KEY Register */
#define AD74416H_CMD_KEY_MSK			NO_OS_GENMASK(15, 0)

/* AD74416H BROADCAST_CMD_KEY Register */
#define AD74416H_BROADCAST_CMD_KEY_MSK		NO_OS_GENMASK(15, 0)

/* AD74416H SCRATCHn Register */
#define AD74416H_SCRATCH_BITS_MSK		NO_OS_GENMASK(15, 0)

/* AD74416H GENERIC_ID Register */
#define AD74416H_GENERIC_ID_MSK			NO_OS_GENMASK(2, 0)

/* AD74416H SILICON_REV Register */
#define AD74416H_SILICON_REV_ID_MSK		NO_OS_GENMASK(7, 0)

/* AD74416H SILICON_ID0 Register */
#define AD74416H_UID0_MSK			NO_OS_GENMASK(6, 0)

/* AD74416H SILICON_ID1 Register */
#define AD74416H_UID2_MSK			NO_OS_GENMASK(11, 6)
#define AD74416H_UID1_MSK			NO_OS_GENMASK(5, 0)

/* AD74416H HART_ALERT_STATUSn Register */
#define AD74416H_FRM_MON_STATE_MSK		NO_OS_GENMASK(15, 13)
#define AD74416H_EOM_MSK			NO_OS_BIT(12)
#define AD74416H_RX_BCNT_MSK			NO_OS_BIT(11)
#define AD74416H_RX_CMD_MSK			NO_OS_BIT(10)
#define AD74416H_SOM_MSK			NO_OS_BIT(9)
#define AD74416H_CD_MSK				NO_OS_BIT(8)
#define AD74416H_CD_EDGE_DET_MSK		NO_OS_BIT(7)
#define AD74416H_TX_COMPLETE_MSK		NO_OS_BIT(6)
#define AD74416H_TX_FIFO_ALERT_MSK		NO_OS_BIT(5)
#define AD74416H_RX_FIFO_ALERT_MSK		NO_OS_BIT(4)
#define AD74416H_RX_OVERFLOW_ERR_MSK		NO_OS_BIT(3)
#define AD74416H_FRAME_ERR_MSK			NO_OS_BIT(2)
#define AD74416H_PARITY_ERR_MSK			NO_OS_BIT(1)
#define AD74416H_GAP_ERR_MSK			NO_OS_BIT(0)

/* AD74416H HART_RXn Register */
#define AD74416H_RFGI_MSK			NO_OS_BIT(11)
#define AD74416H_RFFE_MSK			NO_OS_BIT(10)
#define AD74416H_RFPE_MSK			NO_OS_BIT(9)
#define AD74416H_RFBI_MSK			NO_OS_BIT(8)
#define AD74416H_RBR_MSK			NO_OS_GENMASK(7, 0)

/* AD74416H HART_TXn Register */
#define AD74416H_TDR_MSK			NO_OS_GENMASK(7, 0)

/* AD74416H HART_FCRn Register */
#define AD74416H_TFTRIG_MSK			NO_OS_GENMASK(11, 8)
#define AD74416H_RFTRIG_MSK			NO_OS_GENMASK(6, 3)
#define AD74416H_TFCLR_MSK			NO_OS_BIT(2)
#define AD74416H_RFCLR_MSK			NO_OS_BIT(1)
#define AD74416H_FIFOEN_MSK			NO_OS_BIT(0)

/* AD74416H HART_MCRn Register */
#define AD74416H_RTS_MSK			NO_OS_BIT(0)

/* AD74416H HART_RFCn Register */
#define AD74416H_RFC_MSK			NO_OS_GENMASK(4, 0)

/* AD74416H HART_TFCn Register */
#define AD74416H_TFC_MSK			NO_OS_GENMASK(4, 0)

/* AD74416H HART_CONFIGn Register */
#define AD74416H_CD_EXTD_QUAL_MSK		NO_OS_BIT(13)
#define AD74416H_FRM_MON_RX_PREMX2_MSK		NO_OS_BIT(12)
#define AD74416H_FRM_MON_RST_GAP_MSK		NO_OS_BIT(11)
#define AD74416H_FRM_MON_RST_CD_MSK		NO_OS_BIT(10)
#define AD74416H_RX_ALL_CHARS_MSK		NO_OS_BIT(9)
#define AD74416H_FRM_MON_EN_MSK			NO_OS_BIT(8)
#define AD74416H_EVENT_DET_SEL_MSK		NO_OS_GENMASK(7, 6)
#define AD74416H_TX_1B_AFTER_RST_MSK		NO_OS_BIT(5)
#define AD74416H_AUTO_CLR_RST_MSK		NO_OS_BIT(4)
#define AD74416H_CD_EDGE_SEL_MSK		NO_OS_GENAMSK(3, 2)
#define AD74416H_MODEM_DUPLEX_MSK		NO_OS_BIT(1)
#define AD74416H_MODEM_PWRUP_MSK		NO_OS_BIT(0)

/* AD74416H HART_TX_PREMn Register */
#define AD74416H_TX_PREM_CNT_MSK		NO_OS_GENMASK(4, 0)

/* AD74416H HART_EVDETn Register */
#define AD74416H_EVENT_DET_TIME_MSK		NO_OS_GENMASK(15, 0)

/* AD74416H HART_TX_GAINn Register */
#define AD74416H_TX_GAIN_MSK			NO_OS_GENMASK(3, 0)

/* AD74416H HART_GPIO_IF_CONFIG Register */
#define AD74416H_HART_GPIO_IF_CH_MSK		NO_OS_GENMASK(3, 2)
#define AD74416H_HART_GPIO_IF_SEL_MSK		NO_OS_GENMASK(1, 0)

/* AD74416H HART_GPIO_MON_CONFIG Register */
#define AD74416H_HART_GPIO_MON_CH_MSK		NO_OS_GENMASK(4, 3)
#define AD74416H_HART_GPIO_MON_SEL_MSK		NO_OS_GENMASK(2, 0)

#define AD74416H_TEMP_OFFSET			-2392
#define AD74416H_TEMP_SCALE			8950
#define AD74416H_TEMP_SCALE_DIV			1000

#define AD74416H_FRAME_SIZE 			5
#define AD74416H_THRESHOLD_DAC_RANGE		98
#define AD74416H_THRESHOLD_RANGE		30000
#define AD74416H_DAC_RANGE			12000
#define AD74416H_DAC_CURRENT_RANGE		25000
#define AD74416H_DAC_RESOLUTION			16
#define AD74414H_DAC_RESOLUTION			14
#define AD74116H_CONV_TIME_US			1000000

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad74416h_dev_id` enumeration defines the supported device
 * identifiers for the AD74416H and AD74414H models. This enumeration is
 * used to specify which specific device model is being referenced or
 * utilized in the code, allowing for differentiation between the two
 * models in the AD74416H series.
 *
 * @param ID_AD74416H Represents the device ID for the AD74416H model.
 * @param ID_AD74414H Represents the device ID for the AD74414H model.
 ******************************************************************************/
enum ad74416h_dev_id {
	ID_AD74416H,
	ID_AD74414H,
};

/***************************************************************************//**
 * @brief The `ad74416h_op_mode` enumeration defines the various operational
 * modes available for the AD74416H device, which include modes for
 * handling different types of electrical signals such as voltage,
 * current, and digital inputs. Each mode is associated with a specific
 * functionality, such as high impedance, voltage output, current output,
 * and various input configurations, including those compatible with HART
 * communication. This enumeration is crucial for configuring the device
 * to operate in the desired mode for specific applications.
 *
 * @param AD74416H_HIGH_Z Represents a high impedance state.
 * @param AD74416H_VOLTAGE_OUT Represents a voltage output mode.
 * @param AD74416H_CURRENT_OUT Represents a current output mode.
 * @param AD74416H_VOLTAGE_IN Represents a voltage input mode.
 * @param AD74416H_CURRENT_IN_EXT Represents an external current input mode.
 * @param AD74416H_CURRENT_IN_LOOP Represents a loop current input mode.
 * @param AD74416H_RESISTANCE Represents a resistance measurement mode.
 * @param AD74416H_DIGITAL_INPUT Represents a digital input mode.
 * @param AD74416H_DIGITAL_INPUT_LOOP Represents a loop digital input mode.
 * @param AD74416H_CURRENT_OUT_HART Represents a current output mode with HART
 * communication.
 * @param AD74416H_CURRENT_IN_EXT_HART Represents an external current input mode
 * with HART communication.
 * @param AD74416H_CURRENT_IN_LOOP_HART Represents a loop current input mode
 * with HART communication.
 ******************************************************************************/
enum ad74416h_op_mode {
	AD74416H_HIGH_Z = 0x0,
	AD74416H_VOLTAGE_OUT = 0x1,
	AD74416H_CURRENT_OUT = 0x2,
	AD74416H_VOLTAGE_IN = 0x3,
	AD74416H_CURRENT_IN_EXT = 0x4,
	AD74416H_CURRENT_IN_LOOP = 0x5,
	AD74416H_RESISTANCE = 0x7,
	AD74416H_DIGITAL_INPUT = 0x8,
	AD74416H_DIGITAL_INPUT_LOOP = 0x9,
	AD74416H_CURRENT_OUT_HART = 0xA,
	AD74416H_CURRENT_IN_EXT_HART = 0xB,
	AD74416H_CURRENT_IN_LOOP_HART = 0xC,
};

/***************************************************************************//**
 * @brief The `ad74416h_gpio_select` enumeration defines the possible
 * configurations for the General Purpose Input/Output (GPIO) pins on the
 * AD74416H device. Each enumerator represents a distinct mode of
 * operation for the GPIO, allowing it to function in various roles such
 * as high impedance, data handling, input, comparator, or digital
 * output. This flexibility is crucial for adapting the GPIO to different
 * application requirements in embedded systems.
 *
 * @param AD74416H_GPIO_CONFIG_HIGH_Z Represents a high impedance state for the
 * GPIO configuration.
 * @param AD74416H_GPIO_CONFIG_DATA Configures the GPIO to handle data.
 * @param AD74416H_GPIO_CONFIG_IN Sets the GPIO as an input.
 * @param AD74416H_GPIO_CONFIG_COMP Configures the GPIO for comparator
 * functionality.
 * @param AD74416H_GPIO_CONFIG_DO Sets the GPIO as a digital output.
 ******************************************************************************/
enum ad74416h_gpio_select {
	AD74416H_GPIO_CONFIG_HIGH_Z,
	AD74416H_GPIO_CONFIG_DATA,
	AD74416H_GPIO_CONFIG_IN,
	AD74416H_GPIO_CONFIG_COMP,
	AD74416H_GPIO_CONFIG_DO
};

/***************************************************************************//**
 * @brief The `ad74416h_adc_range` enumeration defines various voltage ranges
 * that the AD74416H device's ADC can measure. Each enumerator represents
 * a specific voltage range, allowing the ADC to be configured for
 * different measurement scenarios, from millivolt to volt levels,
 * including both positive and negative ranges. This flexibility is
 * crucial for applications requiring precise voltage measurements across
 * a wide range of values.
 *
 * @param AD74416H_RNG_0_12_V Represents a range of 0 to 12 volts.
 * @param AD74416H_RNG_NEG12_12_V Represents a range of -12 to 12 volts.
 * @param AD74416H_RNG_NEG0P3125_0P3125V Represents a range of -0.3125 to 0.3125
 * volts.
 * @param AD74416H_RNG_NEG0P3125_0V Represents a range of -0.3125 to 0 volts.
 * @param AD74416H_RNG_0_0P3125V Represents a range of 0 to 0.3125 volts.
 * @param AD74416H_RNG_0_0P625V Represents a range of 0 to 0.625 volts.
 * @param AD74416H_RNG_NEG104_104MV Represents a range of -104 to 104
 * millivolts.
 * @param AD74416H_RNG_NEG2P5_2P5V Represents a range of -2.5 to 2.5 volts.
 ******************************************************************************/
enum ad74416h_adc_range {
	AD74416H_RNG_0_12_V,
	AD74416H_RNG_NEG12_12_V,
	AD74416H_RNG_NEG0P3125_0P3125V,
	AD74416H_RNG_NEG0P3125_0V,
	AD74416H_RNG_0_0P3125V,
	AD74416H_RNG_0_0P625V,
	AD74416H_RNG_NEG104_104MV,
	AD74416H_RNG_NEG2P5_2P5V,
};

/***************************************************************************//**
 * @brief The `ad74416h_adc_rate` enumeration defines various ADC sample rates
 * and their associated configurations for the AD74416H device. Each
 * enumerator represents a specific sample rate in samples per second
 * (SPS) and may include additional features such as 50/60 Hz rejection
 * or HART rejection, which are used to filter out specific types of
 * noise or interference during analog-to-digital conversion.
 *
 * @param AD74416H_10SPS_50_60HZ_HART_REJECTION Represents a sample rate of 10
 * samples per second with 50/60 Hz
 * HART rejection.
 * @param AD74416H_20SPS_50_60HZ_REJECTION Represents a sample rate of 20
 * samples per second with 50/60 Hz
 * rejection.
 * @param AD74416H_20SPS_50_60HZ_HART_REJECTION Represents a sample rate of 20
 * samples per second with 50/60 Hz
 * HART rejection.
 * @param AD74416H_200SPS_HART_REJECTION Represents a sample rate of 200 samples
 * per second with HART rejection.
 * @param AD74416H_200SPS_SIGN_HART_REJECTION Represents a sample rate of 200
 * samples per second with sign and
 * HART rejection.
 * @param AD74416H_1K2SPS Represents a sample rate of 1200 samples per second.
 * @param AD74416H_1K2SPS_HART_REJECTION Represents a sample rate of 1200
 * samples per second with HART rejection.
 * @param AD74416H_4K8SPS Represents a sample rate of 4800 samples per second.
 * @param AD74416H_9K6SPS Represents a sample rate of 9600 samples per second.
 ******************************************************************************/
enum ad74416h_adc_rate {
	AD74416H_10SPS_50_60HZ_HART_REJECTION = 0,
	AD74416H_20SPS_50_60HZ_REJECTION = 1,
	AD74416H_20SPS_50_60HZ_HART_REJECTION = 2,
	AD74416H_200SPS_HART_REJECTION = 4,
	AD74416H_200SPS_SIGN_HART_REJECTION = 6,
	AD74416H_1K2SPS = 8,
	AD74416H_1K2SPS_HART_REJECTION = 9,
	AD74416H_4K8SPS = 12,
	AD74416H_9K6SPS = 13,
};

/***************************************************************************//**
 * @brief The `ad74416h_diag_rate` enumeration defines various diagnostic
 * conversion rates for the AD74416H device, each with specific sample
 * rates and rejection characteristics. These rates are used to configure
 * the device's diagnostic capabilities, allowing it to perform
 * conversions at different speeds and with different noise rejection
 * features, such as HART or 50/60 Hz rejection. This configuration is
 * crucial for optimizing the device's performance in various diagnostic
 * scenarios.
 *
 * @param AD74416H_DIAG_20SPS_50_60HZ_REJECTION Represents a diagnostic
 * conversion rate with 20 samples
 * per second and 50/60 Hz
 * rejection.
 * @param AD74416H_DIAG_20SPS_50_60HZ_HART_REJECTION Represents a diagnostic
 * conversion rate with 20
 * samples per second and
 * 50/60 Hz HART rejection.
 * @param AD74416H_DIAG_1K2SPS_HART_REJECTION Represents a diagnostic conversion
 * rate with 1.2k samples per second
 * and HART rejection.
 * @param AD74416H_DIAG_4K8SPS Represents a diagnostic conversion rate with 4.8k
 * samples per second.
 * @param AD74416H_DIAG_9K6SPS Represents a diagnostic conversion rate with 9.6k
 * samples per second.
 * @param AD74416H_DIAG_19K2SPS Represents a diagnostic conversion rate with
 * 19.2k samples per second.
 ******************************************************************************/
enum ad74416h_diag_rate {
	AD74416H_DIAG_20SPS_50_60HZ_REJECTION,
	AD74416H_DIAG_20SPS_50_60HZ_HART_REJECTION,
	AD74416H_DIAG_1K2SPS_HART_REJECTION,
	AD74416H_DIAG_4K8SPS,
	AD74416H_DIAG_9K6SPS,
	AD74416H_DIAG_19K2SPS,
};

/***************************************************************************//**
 * @brief The `ad74416h_adc_conv_mux` enumeration defines the possible
 * configurations for the ADC conversion multiplexer in the AD74416H
 * device. Each enumerator represents a specific connection path for the
 * ADC inputs, allowing the user to select which input signals are routed
 * to the ADC for conversion. This configuration is crucial for setting
 * up the device to measure different types of signals by selecting the
 * appropriate input path.
 *
 * @param AD74416H_MUX_LF_TO_AGND Connects the low frequency (LF) input to
 * analog ground (AGND).
 * @param AD74416H_MUX_HF_TO_LF Connects the high frequency (HF) input to the
 * low frequency (LF) input.
 * @param AD74416H_MUX_VSENSEN_TO_AGND Connects the VSENSEN input to analog
 * ground (AGND).
 * @param AD74416H_MUX_LF_TO_VSENSEN Connects the low frequency (LF) input to
 * the VSENSEN input.
 * @param AD74416H_MUX_AGND_TO_AGND Connects analog ground (AGND) to itself,
 * effectively a no-operation state.
 ******************************************************************************/
enum ad74416h_adc_conv_mux {
	AD74416H_MUX_LF_TO_AGND,
	AD74416H_MUX_HF_TO_LF,
	AD74416H_MUX_VSENSEN_TO_AGND,
	AD74416H_MUX_LF_TO_VSENSEN,
	AD74416H_MUX_AGND_TO_AGND,
};

/***************************************************************************//**
 * @brief The `ad74416h_debounce_mode` is an enumeration that defines the
 * possible debounce modes for the IOx inputs when the device is
 * operating in digital input mode. It provides two distinct modes,
 * `AD74416H_DEBOUNCE_MODE_0` and `AD74416H_DEBOUNCE_MODE_1`, which can
 * be used to configure the debounce behavior of the digital inputs,
 * potentially affecting how input signals are filtered to avoid false
 * triggering due to noise or signal fluctuations.
 *
 * @param AD74416H_DEBOUNCE_MODE_0 Represents the first debounce mode option.
 * @param AD74416H_DEBOUNCE_MODE_1 Represents the second debounce mode option.
 ******************************************************************************/
enum ad74416h_debounce_mode {
	AD74416H_DEBOUNCE_MODE_0,
	AD74416H_DEBOUNCE_MODE_1
};

/***************************************************************************//**
 * @brief The `ad74416h_conv_seq` enumeration defines the possible commands for
 * controlling the ADC conversion sequence in the AD74416H device. It
 * includes commands to start and stop conversions, both in single and
 * continuous modes, as well as to manage the power state of the ADC
 * during these operations. This enumeration is crucial for managing the
 * ADC's operational state and ensuring efficient power usage.
 *
 * @param AD74416H_STOP_PWR_UP Represents the command to stop and power up the
 * ADC conversion sequence.
 * @param AD74416H_START_SINGLE Represents the command to start a single ADC
 * conversion.
 * @param AD74416H_START_CONT Represents the command to start continuous ADC
 * conversions.
 * @param AD74416H_STOP_PWR_DOWN Represents the command to stop and power down
 * the ADC conversion sequence.
 ******************************************************************************/
enum ad74416h_conv_seq {
	AD74416H_STOP_PWR_UP,
	AD74416H_START_SINGLE,
	AD74416H_START_CONT,
	AD74416H_STOP_PWR_DOWN,
};

/***************************************************************************//**
 * @brief The `ad74416h_diag_mode` is an enumeration that defines various
 * diagnostic modes for the AD74416H device. Each enumerator represents a
 * specific diagnostic test or measurement that can be performed on
 * different parts of the device, such as supply voltages, ground,
 * temperature, and input/output voltages. This enumeration is used to
 * configure the device to perform specific diagnostic checks, which are
 * essential for ensuring the device's proper operation and identifying
 * potential issues.
 *
 * @param AD74416H_DIAG_AGND Represents a diagnostic mode for analog ground.
 * @param AD74416H_DIAG_TEMP Represents a diagnostic mode for temperature.
 * @param AD74416H_DIAG_DVCC Represents a diagnostic mode for digital supply
 * voltage.
 * @param AD74416H_DIAG_AVCC Represents a diagnostic mode for analog supply
 * voltage.
 * @param AD74416H_DIAG_LDO1V8 Represents a diagnostic mode for 1.8V LDO.
 * @param AD74416H_DIAG_AVDD_HI Represents a diagnostic mode for high analog
 * supply voltage.
 * @param AD74416H_DIAG_AVDD_LO Represents a diagnostic mode for low analog
 * supply voltage.
 * @param AD74416H_DIAG_AVSS Represents a diagnostic mode for analog supply
 * ground.
 * @param AD74416H_DIAG_LVIN Represents a diagnostic mode for low voltage input.
 * @param AD74416H_DIAG_DO_VDD Represents a diagnostic mode for digital output
 * supply voltage.
 * @param AD74416H_VSENSEP_C Represents a diagnostic mode for positive sense
 * voltage.
 * @param AD74416H_VSENSEN_C Represents a diagnostic mode for negative sense
 * voltage.
 * @param AD74416H_DO_C Represents a diagnostic mode for digital output.
 * @param AD74416H_AVDD_C Represents a diagnostic mode for analog supply
 * voltage.
 ******************************************************************************/
enum ad74416h_diag_mode {
	AD74416H_DIAG_AGND,
	AD74416H_DIAG_TEMP,
	AD74416H_DIAG_DVCC,
	AD74416H_DIAG_AVCC,
	AD74416H_DIAG_LDO1V8,
	AD74416H_DIAG_AVDD_HI,
	AD74416H_DIAG_AVDD_LO,
	AD74416H_DIAG_AVSS,
	AD74416H_DIAG_LVIN,
	AD74416H_DIAG_DO_VDD,
	AD74416H_VSENSEP_C,
	AD74416H_VSENSEN_C,
	AD74416H_DO_C,
	AD74416H_AVDD_C,
};

/***************************************************************************//**
 * @brief The `ad74416h_slew_lin_step` enumeration defines the possible linear
 * step sizes for the DAC's slew rate control in the AD74416H device.
 * These step sizes determine how quickly the DAC output can change,
 * expressed as a percentage of the full-scale output. This allows for
 * controlled transitions in the output voltage, which can be critical in
 * applications requiring precise voltage changes.
 *
 * @param AD74416H_STEP_0_8_PERCENT Represents a linear step size of 0.8% for
 * the DAC slew rate control.
 * @param AD74416H_STEP_1_5_PERCENT Represents a linear step size of 1.5% for
 * the DAC slew rate control.
 * @param AD74416H_STEP_6_1_PERCENT Represents a linear step size of 6.1% for
 * the DAC slew rate control.
 * @param AD74416H_STEP_22_2_PERCENT Represents a linear step size of 22.2% for
 * the DAC slew rate control.
 ******************************************************************************/
enum ad74416h_slew_lin_step {
	AD74416H_STEP_0_8_PERCENT,
	AD74416H_STEP_1_5_PERCENT,
	AD74416H_STEP_6_1_PERCENT,
	AD74416H_STEP_22_2_PERCENT,
};

/***************************************************************************//**
 * @brief The `ad74416h_lin_rate` enumeration defines possible update rates for
 * a Digital-to-Analog Converter (DAC) when slew control is enabled. Each
 * enumerator represents a specific frequency at which the DAC can update
 * its output, allowing for precise control over the rate of change in
 * the output signal. This is particularly useful in applications
 * requiring smooth transitions in analog output signals.
 *
 * @param AD74416H_LIN_RATE_4KHZ8 Represents a linear update rate of 4.8 kHz for
 * the DAC.
 * @param AD74416H_LIN_RATE_76KHZ8 Represents a linear update rate of 76.8 kHz
 * for the DAC.
 * @param AD74416H_LIN_RATE_153KHZ6 Represents a linear update rate of 153.6 kHz
 * for the DAC.
 * @param AD74416H_LIN_RATE_230KHZ4 Represents a linear update rate of 230.4 kHz
 * for the DAC.
 ******************************************************************************/
enum ad74416h_lin_rate {
	AD74416H_LIN_RATE_4KHZ8,
	AD74416H_LIN_RATE_76KHZ8,
	AD74416H_LIN_RATE_153KHZ6,
	AD74416H_LIN_RATE_230KHZ4,
};

/***************************************************************************//**
 * @brief The `ad74416h_vout_range` enumeration defines the possible voltage
 * output ranges for the AD74416H device's DAC. It provides two options:
 * a unipolar range from 0 to 12 volts and a bipolar range from -12 to 12
 * volts. This enumeration is used to configure the voltage output range
 * for specific channels in the device, allowing for flexible voltage
 * output configurations depending on the application requirements.
 *
 * @param AD74416H_VOUT_RANGE_0_12V Represents a voltage output range from 0 to
 * 12 volts.
 * @param AD74416H_VOUT_RANGE_NEG12_12V Represents a voltage output range from
 * -12 to 12 volts.
 ******************************************************************************/
enum ad74416h_vout_range {
	AD74416H_VOUT_RANGE_0_12V,
	AD74416H_VOUT_RANGE_NEG12_12V,
};

/***************************************************************************//**
 * @brief The `ad74416h_i_limit` enumeration defines the possible current limit
 * settings for the DAC when operating in voltage output (Vout) mode. It
 * provides two distinct current limit options, `AD74416H_I_LIMIT0` and
 * `AD74416H_I_LIMIT1`, which can be used to configure the maximum
 * allowable current output by the DAC, ensuring safe operation within
 * specified limits.
 *
 * @param AD74416H_I_LIMIT0 Represents the first current limit setting for the
 * DAC in Vout mode.
 * @param AD74416H_I_LIMIT1 Represents the second current limit setting for the
 * DAC in Vout mode.
 ******************************************************************************/
enum ad74416h_i_limit {
	AD74416H_I_LIMIT0,
	AD74416H_I_LIMIT1,
};

/***************************************************************************//**
 * @brief The `_ad74416h_live_status` structure is a bitfield representation of
 * the live status register for the AD74416H device. It contains various
 * status indicators for the device's power supply, ADC operations,
 * temperature alerts, digital input/output statuses, and analog I/O
 * statuses. Each field is a single bit, reflecting the real-time
 * operational state of the device's components, allowing for efficient
 * monitoring and control.
 *
 * @param SUPPLY_STATUS Indicates the status of the power supply.
 * @param ADC_BUSY Indicates if the ADC is currently busy.
 * @param ADC_DATA_RDY Indicates if ADC data is ready for reading.
 * @param TEMP_ALERT_STATUS Indicates if there is a temperature alert.
 * @param DIN_STATUS_A Indicates the status of digital input A.
 * @param DIN_STATUS_B Indicates the status of digital input B.
 * @param DIN_STATUS_C Indicates the status of digital input C.
 * @param DIN_STATUD_D Indicates the status of digital input D.
 * @param DO_STATUS_A Indicates the status of digital output A.
 * @param DO_STATUS_B Indicates the status of digital output B.
 * @param DO_STATUS_C Indicates the status of digital output C.
 * @param DO_STATUS_D Indicates the status of digital output D.
 * @param ANALOG_IO_STATUS_A Indicates the status of analog I/O A.
 * @param ANALOG_IO_STATUS_B Indicates the status of analog I/O B.
 * @param ANALOG_IO_STATUS_C Indicates the status of analog I/O C.
 * @param ANALOG_IO_STATUS_D Indicates the status of analog I/O D.
 ******************************************************************************/
struct _ad74416h_live_status {
	uint8_t SUPPLY_STATUS: 1;
	uint8_t ADC_BUSY: 1;
	uint8_t ADC_DATA_RDY: 1;
	uint8_t TEMP_ALERT_STATUS: 1;
	uint8_t DIN_STATUS_A: 1;
	uint8_t DIN_STATUS_B: 1;
	uint8_t DIN_STATUS_C: 1;
	uint8_t DIN_STATUD_D: 1;
	uint8_t DO_STATUS_A: 1;
	uint8_t DO_STATUS_B: 1;
	uint8_t DO_STATUS_C: 3;
	uint8_t DO_STATUS_D: 1;
	uint8_t ANALOG_IO_STATUS_A: 1;
	uint8_t ANALOG_IO_STATUS_B: 1;
	uint8_t ANALOG_IO_STATUS_C: 1;
	uint8_t ANALOG_IO_STATUS_D: 1;
};

/***************************************************************************//**
 * @brief The `ad74416h_live_status` union is designed to store the live status
 * of the AD74416H device. It provides two ways to access the status
 * information: as a structured set of bit fields through the
 * `status_bits` member, which allows for easy access to individual
 * status indicators, and as a single 16-bit integer through the `value`
 * member, which can be useful for operations that require the entire
 * status to be treated as a single entity. This dual representation
 * facilitates both detailed and high-level status management.
 *
 * @param status_bits A struct containing bit fields representing various live
 * status indicators.
 * @param value A 16-bit unsigned integer representing the live status as a
 * whole.
 ******************************************************************************/
union ad74416h_live_status {
	struct _ad74416h_live_status status_bits;
	uint16_t value;
};

/***************************************************************************//**
 * @brief The `ad74416h_init_param` structure is used to initialize the AD74416H
 * device, encapsulating essential parameters such as the device ID,
 * communication address, and initialization parameters for SPI and GPIO
 * interfaces. This structure is crucial for setting up the device's
 * communication and control interfaces before it can be used in an
 * application.
 *
 * @param id Specifies the device ID from the supported device IDs enumeration.
 * @param dev_addr Holds the device address for communication.
 * @param spi_ip Contains the SPI initialization parameters for the device.
 * @param reset_gpio_param Pointer to the GPIO initialization parameters for the
 * reset pin.
 ******************************************************************************/
struct ad74416h_init_param {
	enum ad74416h_dev_id id;
	uint8_t dev_addr;
	struct no_os_spi_init_param spi_ip;
	struct no_os_gpio_init_param *reset_gpio_param;
};

/**
 * @brief Device channel state.
 */

/***************************************************************************//**
 * @brief The `ad74416h_channel_config` structure is used to configure
 * individual channels of the AD74416H device. It includes settings for
 * enabling the channel, selecting the operational mode, defining the
 * voltage output range, and setting the current limit. This
 * configuration allows for flexible control of the device's channels to
 * suit various application requirements.
 *
 * @param enabled Indicates whether the channel is enabled or not.
 * @param function Specifies the operational mode of the channel.
 * @param vout_range Defines the voltage output range for the channel.
 * @param i_limit Sets the current limit for the channel in voltage output mode.
 ******************************************************************************/
struct ad74416h_channel_config {
	bool enabled;
	enum ad74416h_op_mode function;
	enum ad74416h_vout_range vout_range;
	enum ad74416h_i_limit i_limit;
};

/***************************************************************************//**
 * @brief The `ad74416h_desc` structure is a descriptor for the AD74416H device,
 * encapsulating all necessary information for device communication and
 * configuration. It includes the device ID, communication address, SPI
 * descriptor for interfacing, a communication buffer for data exchange,
 * channel configurations for each of the device's channels, and a GPIO
 * descriptor for handling device resets. This structure is essential for
 * managing the device's operations and interactions in a software
 * environment.
 *
 * @param id An enumeration representing the device ID of the AD74416H.
 * @param dev_addr A uint8_t representing the device address for communication.
 * @param spi_desc A pointer to a no_os_spi_desc structure for SPI
 * communication.
 * @param comm_buff A buffer array of size AD74416H_FRAME_SIZE for communication
 * frames.
 * @param channel_configs An array of ad74416h_channel_config structures for
 * each channel configuration.
 * @param reset_gpio A pointer to a no_os_gpio_desc structure for GPIO reset
 * control.
 ******************************************************************************/
struct ad74416h_desc {
	enum ad74416h_dev_id id;
	uint8_t dev_addr;
	struct no_os_spi_desc *spi_desc;
	uint8_t comm_buff[AD74416H_FRAME_SIZE];
	struct ad74416h_channel_config channel_configs[AD74416H_N_CHANNELS];
	struct no_os_gpio_desc *reset_gpio;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to convert a given voltage in millivolts to a
 * corresponding DAC code for a specified channel on the AD74416H device.
 * It requires a valid device descriptor and channel number, and the
 * voltage must be within the supported range for the channel's
 * configured output range. The function will return an error if the
 * device ID or channel configuration is invalid, or if the voltage is
 * out of range.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null and should be properly initialized.
 * @param mvolts The voltage in millivolts to be converted. Must be within the
 * range supported by the channel's output configuration.
 * @param code A pointer to a uint16_t where the resulting DAC code will be
 * stored. Must not be null.
 * @param ch The channel number for which the conversion is to be performed.
 * Must be a valid channel index for the device.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or the conversion cannot be performed.
 ******************************************************************************/
int ad74416h_dac_voltage_to_code(struct ad74416h_desc *, int32_t,
				 uint16_t *, uint32_t);

/***************************************************************************//**
 * @brief This function is used to convert a specified current value, in
 * microamps, into a corresponding digital-to-analog converter (DAC) code
 * for the AD74416H or AD74414H devices. It is essential to ensure that
 * the device descriptor is correctly initialized and that the current
 * value does not exceed the maximum allowable range for the DAC. The
 * function will return an error if the device ID is unrecognized or if
 * the current value is out of range.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param uamps The current value in microamps to be converted. Must be within
 * the allowable range for the DAC; otherwise, the function returns
 * an error.
 * @param code A pointer to a uint16_t where the resulting DAC code will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device ID is
 * invalid or the current value is out of range.
 ******************************************************************************/
int ad74416h_dac_current_to_code(struct ad74416h_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit value to a specific register
 * of the AD74416H device. It is essential for configuring the device's
 * registers to desired settings. The function requires a valid device
 * descriptor and the register address to be written. It is typically
 * used in device initialization or configuration routines. Ensure that
 * the device descriptor is properly initialized before calling this
 * function.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param addr A 32-bit unsigned integer representing the register address to
 * write to. Must be a valid register address for the AD74416H
 * device.
 * @param val A 16-bit unsigned integer representing the value to write to the
 * specified register.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a negative value indicates an error occurred during the SPI write
 * operation.
 ******************************************************************************/
int ad74416h_reg_write(struct ad74416h_desc *, uint32_t, uint16_t);

/***************************************************************************//**
 * @brief This function is used to read a raw communication frame from a
 * specified register address on the AD74416H device. It is typically
 * called when low-level access to the device's registers is required.
 * The function requires a valid device descriptor and a register address
 * to read from. The caller must ensure that the device has been properly
 * initialized before calling this function. The function writes the
 * register address to a specific register to prepare for reading, and
 * then performs the read operation. It is important to handle the return
 * value to check for any errors during the SPI communication.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device
 * descriptor. Must not be null and should be properly initialized
 * before use.
 * @param addr A 32-bit unsigned integer representing the register address to
 * read from. Must be a valid register address for the AD74416H
 * device.
 * @param val A pointer to a uint8_t buffer where the read data will be stored.
 * Must not be null and should have enough space to store the data.
 * @return Returns an integer status code. A non-zero value indicates an error
 * during the SPI communication.
 ******************************************************************************/
int ad74416h_reg_read_raw(struct ad74416h_desc *, uint32_t, uint8_t *);

/***************************************************************************//**
 * @brief This function is used to read a 16-bit value from a specified register
 * address of the AD74416H device. It is essential to ensure that the
 * device descriptor is properly initialized before calling this
 * function. The function performs a CRC check to validate the integrity
 * of the data read from the device. If the CRC check fails, the function
 * returns an error code. This function is typically used when precise
 * register data is needed from the device for further processing or
 * configuration.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param addr The 32-bit address of the register to read from. Must be a valid
 * register address for the AD74416H device.
 * @param val A pointer to a 16-bit unsigned integer where the read value will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails or the CRC check is invalid.
 ******************************************************************************/
int ad74416h_reg_read(struct ad74416h_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function is used to modify a specific field within a register of
 * the AD74416H device. It reads the current value of the register,
 * applies a mask to clear the bits of interest, and then sets them to
 * the new value provided. This function should be called when a specific
 * configuration or setting needs to be updated in the device's register
 * map. It is important to ensure that the device descriptor is properly
 * initialized before calling this function. The function returns an
 * error code if the read or write operation fails.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param addr The address of the register to be updated. Must be a valid
 * register address for the device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Only bits set in this mask will be modified.
 * @param val The new value to be written to the specified bits in the register,
 * as defined by the mask.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad74416h_reg_update(struct ad74416h_desc *, uint32_t, uint16_t,
			uint16_t);

/***************************************************************************//**
 * @brief This function is used to determine how many channels are currently
 * active on the AD74416H device. It should be called when you need to
 * know the active channel count for configuration or monitoring
 * purposes. The function requires a valid device descriptor and a
 * pointer to store the result. It reads the relevant register to
 * calculate the number of active channels and returns this count through
 * the provided pointer. Ensure that the device descriptor is properly
 * initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param nb_channels A pointer to a uint8_t where the number of active channels
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the register read
 * fails.
 ******************************************************************************/
int ad74416h_nb_active_channels(struct ad74416h_desc *, uint8_t *);

/**
 * Select which information the device will respond with (in the readback field)
 * when a read operation is performed
 */
int ad74416h_set_info(struct ad74416h_desc *desc, uint16_t mode);

/***************************************************************************//**
 * @brief This function configures a specified channel on the AD74416H device to
 * operate in a given mode. It should be used when you need to change the
 * functionality of a channel, such as switching between voltage output,
 * current input, or other supported modes. The function must be called
 * with a valid device descriptor and channel number. Note that certain
 * modes are not supported for specific device IDs, and attempting to set
 * an unsupported mode will result in an error. The function includes
 * necessary delays as per the device datasheet to ensure proper mode
 * transition.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null, and should be properly initialized before
 * calling this function.
 * @param ch The channel number to configure, ranging from 0 to 3, corresponding
 * to channels A to D. Values outside this range are invalid.
 * @param ch_func An enum value of type ad74416h_op_mode representing the
 * desired operation mode for the channel. Must be a valid mode
 * supported by the device; otherwise, an error is returned.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if an invalid mode is specified for the device.
 ******************************************************************************/
int ad74416h_set_channel_function(struct ad74416h_desc *,
				  uint32_t, enum ad74416h_op_mode);

/***************************************************************************//**
 * @brief This function configures the voltage output range for a specific
 * channel on the AD74416H device. It should be used when you need to
 * adjust the voltage output characteristics of a channel to match the
 * requirements of your application. Ensure that the device descriptor is
 * properly initialized before calling this function. The function
 * updates the device's configuration and returns an error code if the
 * operation fails.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number to configure, ranging from 0 to 3, corresponding
 * to channels A to D.
 * @param vout_range An enum value of type ad74416h_vout_range specifying the
 * desired voltage output range. Must be a valid range defined
 * in the enum.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74416h_set_channel_vout_range(struct ad74416h_desc *desc, uint32_t ch,
				    enum ad74416h_vout_range vout_range);

/***************************************************************************//**
 * @brief This function configures the current limit for a specific DAC channel
 * on the AD74416H device when operating in voltage output mode. It
 * should be used when you need to ensure that the current does not
 * exceed a certain threshold for a given channel. The function must be
 * called with a valid device descriptor and channel index. It is
 * important to ensure that the channel index is within the valid range
 * of available channels, and the current limit value is one of the
 * predefined limits. If the operation is successful, the function
 * returns 0; otherwise, it returns an error code indicating the failure.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param ch The index of the channel for which the current limit is to be set.
 * Valid values are 0 to 3, corresponding to channels A to D.
 * @param i_limit An enum value of type ad74416h_i_limit representing the
 * desired current limit. Must be a valid enum value defined for
 * current limits.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ad74416h_set_channel_i_limit(struct ad74416h_desc *, uint32_t,
				 enum ad74416h_i_limit);

/***************************************************************************//**
 * @brief This function is used to obtain the raw ADC conversion result from a
 * specified channel of the AD74416H device. It should be called when a
 * raw ADC value is needed for further processing or analysis. The
 * function requires a valid device descriptor and a channel number
 * within the supported range. The result is stored in the provided
 * output parameter. Ensure that the device is properly initialized
 * before calling this function. The function returns an error code if
 * the read operation fails.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null and should be properly initialized.
 * @param ch The channel number from which to read the ADC result. Valid values
 * are 0 to 3, corresponding to channels A to D.
 * @param val A pointer to a uint32_t where the raw ADC result will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74416h_get_raw_adc_result(struct ad74416h_desc *, uint32_t,
				uint32_t *);

/***************************************************************************//**
 * @brief This function is used to enable or disable a specific ADC channel on
 * the AD74416H device. It should be called when you need to control the
 * activation state of an ADC channel, typically as part of configuring
 * the device for a specific measurement task. The function requires a
 * valid device descriptor and a channel index within the supported
 * range. It updates the device's internal configuration to reflect the
 * new state of the specified channel. Ensure that the device descriptor
 * is properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The index of the ADC channel to be enabled or disabled. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param status A boolean value indicating whether to enable (true) or disable
 * (false) the specified ADC channel.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74416h_set_adc_channel_enable(struct ad74416h_desc *, uint32_t,
				    bool);

/***************************************************************************//**
 * @brief This function is used to control the diagnostic conversion feature for
 * a specific channel on the AD74416H device. It allows enabling or
 * disabling diagnostic conversions, which are useful for monitoring and
 * testing the channel's performance. This function should be called when
 * you need to start or stop diagnostic data collection for a channel.
 * Ensure that the device descriptor is properly initialized before
 * calling this function. The function returns an integer status code
 * indicating success or failure of the operation.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number to configure, ranging from 0 to 3, corresponding
 * to channels A to D.
 * @param status A boolean value where true enables diagnostic conversions and
 * false disables them.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad74416h_set_diag_channel_enable(struct ad74416h_desc *, uint32_t, bool);

/***************************************************************************//**
 * @brief This function is used to obtain the current ADC measurement range
 * setting for a specific channel on the AD74416H device. It is typically
 * called when you need to verify or log the ADC configuration of a
 * channel. Ensure that the device descriptor is properly initialized
 * before calling this function. The function will return an error code
 * if the read operation fails, otherwise it will store the range value
 * in the provided pointer.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the ADC range is to be retrieved.
 * Valid values are 0 to 3, corresponding to channels A to D.
 * @param val A pointer to a uint16_t where the ADC range value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74416h_get_adc_range(struct ad74416h_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function configures the ADC measurement range for a specified
 * channel on the AD74416H device. It should be called when you need to
 * adjust the ADC range to match the expected input signal levels for
 * accurate measurements. The function must be called with a valid device
 * descriptor and channel number. If the device is an AD74414H, certain
 * range values are not supported and will result in an error. Ensure the
 * device is properly initialized before calling this function.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null and should be properly initialized.
 * @param ch The channel number to configure, ranging from 0 to 3, corresponding
 * to channels A to D.
 * @param val An enum value of type ad74416h_adc_range representing the desired
 * ADC range. Some values may be invalid for certain device types,
 * such as AD74414H, and will result in an error.
 * @return Returns 0 on success or a negative error code if the operation fails,
 * such as -EINVAL for unsupported range values on certain devices.
 ******************************************************************************/
int ad74416h_set_adc_range(struct ad74416h_desc *, uint32_t,
			   enum ad74416h_adc_range);

/***************************************************************************//**
 * @brief Use this function to obtain the current ADC sample rate setting for a
 * specific channel on the AD74416H device. This function is useful when
 * you need to verify or log the ADC configuration for a channel. Ensure
 * that the device descriptor is properly initialized before calling this
 * function. The function will return an error code if the read operation
 * fails.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which to retrieve the ADC rate. Valid values
 * are 0 to 3, corresponding to channels A to D.
 * @param val A pointer to an enum ad74416h_adc_rate variable where the ADC rate
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74416h_get_adc_rate(struct ad74416h_desc *, uint32_t,
			  enum ad74416h_adc_rate *);

/***************************************************************************//**
 * @brief This function configures the ADC sample rate for a specific channel on
 * the AD74416H device. It should be used when you need to adjust the
 * conversion rate of the ADC to match the requirements of your
 * application. Ensure that the device descriptor is properly initialized
 * before calling this function. The function will return an error code
 * if the operation fails, so it is important to check the return value
 * to ensure the rate was set successfully.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the ADC rate is to be set. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param val The desired ADC sample rate, specified as a value from the
 * ad74416h_adc_rate enumeration. This determines the conversion
 * speed and any additional filtering or rejection features.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74416h_set_adc_rate(struct ad74416h_desc *, uint32_t,
			  enum ad74416h_adc_rate);

/***************************************************************************//**
 * @brief This function is used to obtain the current ADC conversion multiplexer
 * setting for a specific channel on the AD74416H device. It is typically
 * called when you need to verify or log the current multiplexer
 * configuration of a channel. The function requires a valid device
 * descriptor and channel number, and it outputs the multiplexer setting
 * through a pointer. Ensure that the device descriptor is properly
 * initialized before calling this function. The function returns an
 * error code if the operation fails, otherwise it returns 0.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which to retrieve the ADC conversion
 * multiplexer setting. Valid values are 0 to 3, corresponding to
 * channels A to D.
 * @param val A pointer to an enum ad74416h_adc_conv_mux where the current
 * multiplexer setting will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74416h_get_adc_conv_mux(struct ad74416h_desc *, uint32_t,
			      enum ad74416h_adc_conv_mux *);

/***************************************************************************//**
 * @brief This function configures the ADC conversion multiplexer for a given
 * channel on the AD74416H device. It should be used to select the input
 * source for ADC conversions. The function must be called with a valid
 * device descriptor and channel number. If the device is an AD74414H,
 * certain multiplexer values are not allowed and will result in an
 * error. This function is typically used during the setup phase of ADC
 * configuration.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null.
 * @param ch The channel number to configure, typically ranging from 0 to 3.
 * @param val The desired ADC conversion multiplexer setting, specified as an
 * ad74416h_adc_conv_mux enum value. Certain values may be invalid
 * for specific device types.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if an invalid multiplexer value is specified for the device type.
 ******************************************************************************/
int ad74416h_set_adc_conv_mux(struct ad74416h_desc *, uint32_t,
			      enum ad74416h_adc_conv_mux);

/***************************************************************************//**
 * @brief This function is used to control the ADC conversion sequence of the
 * AD74416H device. It allows the user to start or stop ADC conversions
 * by specifying the desired conversion sequence. This function should be
 * called when there is a need to change the ADC conversion state, such
 * as starting a new conversion or stopping an ongoing one. It is
 * important to ensure that the device descriptor is properly initialized
 * before calling this function. After setting the conversion sequence,
 * the function introduces a delay to ensure the ADC is ready for
 * operation, especially if it was previously powered down.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param status An enum value of type ad74416h_conv_seq indicating the desired
 * ADC conversion sequence. Valid values are AD74416H_STOP_PWR_UP,
 * AD74416H_START_SINGLE, AD74416H_START_CONT, and
 * AD74416H_STOP_PWR_DOWN.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74416h_set_adc_conv_seq(struct ad74416h_desc *, enum ad74416h_conv_seq);

/***************************************************************************//**
 * @brief Use this function to obtain a single ADC conversion result from a
 * specified channel on the AD74416H device. It enables the ADC channel,
 * starts a single conversion sequence, waits for the conversion to
 * complete, retrieves the result, and then powers down the ADC channel.
 * This function should be called when a single measurement is needed
 * from a specific channel. Ensure that the device is properly
 * initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number from which to retrieve the ADC result. Valid
 * values are 0 to 3, corresponding to AD74416H_CH_A to AD74416H_CH_D.
 * @param val A pointer to a uint16_t variable where the ADC result will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74416h_get_adc_single(struct ad74416h_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function retrieves the temperature reading from a specified
 * channel of the AD74416H device. It should be called when a temperature
 * measurement is needed for a particular channel. The function requires
 * a valid device descriptor and channel number, and it outputs the
 * temperature reading through a pointer. Ensure that the device is
 * properly initialized before calling this function. The function
 * returns an error code if the operation fails, otherwise it returns 0.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ch The channel number from which to retrieve the temperature. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param temp A pointer to a uint16_t where the temperature reading will be
 * stored. Must not be null. The function writes the temperature
 * value to this location.
 * @return Returns 0 on success, or a non-zero error code if the operation
 * fails.
 ******************************************************************************/
int ad74416h_get_temp(struct ad74416h_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function sets the digital-to-analog converter (DAC) code for a
 * specified channel on the AD74416H device. It is used to configure the
 * output voltage or current for the channel by writing the provided DAC
 * code to the appropriate register. This function should be called when
 * the user needs to update the DAC output for a specific channel. Ensure
 * that the device descriptor is properly initialized before calling this
 * function. The function returns an error code if the operation fails.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the DAC code is to be set. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param dac_code The 16-bit DAC code to be set for the specified channel. The
 * value should be within the valid range for the DAC.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int ad74416h_set_channel_dac_code(struct ad74416h_desc *, uint32_t, uint16_t);

/***************************************************************************//**
 * @brief This function configures the diagnostic mode for a specific channel on
 * the AD74416H device. It should be used when you need to assign a
 * diagnostic function to a channel, which can help in monitoring and
 * troubleshooting the device's operation. The function must be called
 * with a valid device descriptor and channel number. Note that certain
 * diagnostic modes are not supported for the AD74414H device, and
 * attempting to set these will result in an error. Ensure that the
 * device is properly initialized before calling this function.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null, and the device should be initialized.
 * @param ch The channel number to configure, typically ranging from 0 to 3,
 * corresponding to channels A to D.
 * @param diag_code An enum value of type ad74416h_diag_mode representing the
 * diagnostic mode to set. Some modes may not be valid for
 * certain device IDs, such as ID_AD74414H.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as when an unsupported diagnostic mode is specified for
 * the device type.
 ******************************************************************************/
int ad74416h_set_diag(struct ad74416h_desc *, uint32_t,
		      enum ad74416h_diag_mode);

/***************************************************************************//**
 * @brief This function is used to obtain the diagnostic code from a specific
 * channel of the AD74416H device. It should be called when diagnostic
 * information is needed for a particular channel. The function requires
 * a valid device descriptor and a channel number within the supported
 * range. The diagnostic code is returned through a pointer provided by
 * the caller. Ensure that the device is properly initialized before
 * calling this function.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null and should be properly initialized.
 * @param ch The channel number from which to retrieve the diagnostic code.
 * Valid values are 0 to 3, corresponding to channels A to D.
 * @param diag_code A pointer to a uint16_t where the diagnostic code will be
 * stored. Must not be null.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the operation.
 ******************************************************************************/
int ad74416h_get_diag(struct ad74416h_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function configures the debounce mode for the digital input of a
 * specified channel on the AD74416H device. It is used when the ADC is
 * operating in digital input mode to manage how input signal changes are
 * filtered or debounced. The function should be called with a valid
 * device descriptor and channel number. The channel number must be
 * within the range of available channels on the device. The debounce
 * mode is specified using an enumeration, and the function will return
 * an error code if the operation fails.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param ch The channel number to configure. Must be a valid channel index
 * within the range supported by the device (e.g., 0 to 3 for a
 * 4-channel device).
 * @param mode An enumeration value of type ad74416h_debounce_mode specifying
 * the desired debounce mode. Must be a valid value from the
 * ad74416h_debounce_mode enum.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad74416h_set_debounce_mode(struct ad74416h_desc *, uint32_t,
			       enum ad74416h_debounce_mode);

/***************************************************************************//**
 * @brief This function configures the debounce settle time for the IOx inputs
 * of a specified channel when the ADC is operating in digital input
 * mode. It should be used to adjust the debounce time to match the
 * specific requirements of the input signal to ensure reliable digital
 * input detection. The function must be called with a valid device
 * descriptor and channel number. The debounce time is mapped to the
 * closest supported value, and the function updates the device
 * configuration accordingly.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ch The channel number for which to set the debounce time. Valid values
 * are 0 to 3, corresponding to channels A to D.
 * @param time The desired debounce time in microseconds. The function maps this
 * to the closest supported debounce time value.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74416h_set_debounce_time(struct ad74416h_desc *, uint32_t, uint32_t);

/***************************************************************************//**
 * @brief Use this function to obtain the current GPIO value for a specific
 * channel on the AD74416H device. It is essential to ensure that the
 * device descriptor is properly initialized before calling this
 * function. The function reads the digital input comparator output
 * register and extracts the value for the specified channel. This
 * function is useful for monitoring the digital state of a channel
 * configured as a digital input. It returns an error code if the read
 * operation fails.
 *
 * @param desc A pointer to an initialized ad74416h_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number to read from, ranging from 0 to 3, corresponding
 * to channels A to D.
 * @param val A pointer to a uint8_t where the GPIO value will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ad74416h_gpio_get(struct ad74416h_desc *, uint32_t, uint8_t *);

/***************************************************************************//**
 * @brief This function is used to set the GPIO configuration for a specific
 * channel on the AD74416H device. It should be called when you need to
 * change the GPIO mode of a channel to one of the predefined modes. The
 * function requires a valid device descriptor and a channel number
 * within the supported range. The configuration parameter must be a
 * valid enum value representing the desired GPIO mode. This function
 * returns an integer status code indicating success or failure of the
 * operation.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null, and should be properly initialized before
 * calling this function.
 * @param ch The channel number to configure. Must be a valid channel index (0
 * to 3) corresponding to AD74416H_CH_A to AD74416H_CH_D.
 * @param config An enum value of type ad74416h_gpio_select representing the
 * desired GPIO configuration. Must be a valid enum value.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad74416h_set_gpio_config(struct ad74416h_desc *, uint32_t,
			     enum ad74416h_gpio_select);

/***************************************************************************//**
 * @brief This function is used to configure the threshold voltage level for a
 * specific channel on the AD74416H device when operating in digital
 * input mode. It ensures that the threshold is set within a fixed range
 * of 0 to 16 volts, independent of the Vadd voltage. The function should
 * be called with a valid device descriptor and channel number. It is
 * important to ensure that the threshold value does not exceed the
 * defined maximum range, as this will result in an error. The function
 * modifies the device's register settings to apply the threshold.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device
 * descriptor. Must not be null, and should be properly initialized
 * before calling this function.
 * @param ch The channel number for which the threshold is being set. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param threshold The desired threshold value in millivolts. Must be less than
 * or equal to AD74416H_THRESHOLD_RANGE. If the value exceeds
 * this range, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the threshold is
 * out of range or if there is a failure in updating the register.
 ******************************************************************************/
int ad74416h_set_threshold(struct ad74416h_desc *, uint32_t, uint32_t);

/***************************************************************************//**
 * @brief This function is used to set the digital output value for a specific
 * channel on the AD74416H device. It is typically called when you need
 * to control the digital output state of a channel, such as turning a
 * connected device on or off. The function requires a valid device
 * descriptor and a channel number within the supported range. The
 * digital output value is specified as a boolean, where a non-zero value
 * typically represents a high state and zero represents a low state.
 * Ensure that the device is properly initialized before calling this
 * function.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null, and the device must be initialized.
 * @param ch The channel number to set the digital output for. Valid values are
 * 0 to 3, corresponding to channels A to D.
 * @param val The digital output value to set. Typically, 0 represents a low
 * state and any non-zero value represents a high state.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74416h_do_set(struct ad74416h_desc *, uint32_t, uint8_t);

/***************************************************************************//**
 * @brief This function is used to set the logic value of a General Purpose
 * Output (GPO) pin on a specified channel of the AD74416H device. It
 * should be called when the GPIO pin needs to be configured to a
 * specific logic level. The function requires a valid device descriptor
 * and channel number, and it assumes that the GPIO configuration for the
 * channel is already set to allow data output. The function returns an
 * error code if the operation fails, which can occur if the channel
 * number is invalid or if there is a communication error with the
 * device.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param ch The channel number on which to set the GPIO value. Valid values are
 * 0 to 3, corresponding to channels A to D.
 * @param val The logic value to set on the GPIO pin. Typically, 0 represents a
 * low logic level and 1 represents a high logic level.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74416h_gpio_set(struct ad74416h_desc *, uint32_t, uint8_t);

/***************************************************************************//**
 * @brief This function is used to obtain the current live status of the
 * AD74416H device, which includes various status bits indicating the
 * state of the device. It should be called when the user needs to
 * monitor the real-time status of the device, such as checking for
 * alerts or operational states. The function requires a valid device
 * descriptor and a pointer to a status union where the live status will
 * be stored. It is important to ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device
 * descriptor. This must be a valid, initialized descriptor and must
 * not be null.
 * @param status A pointer to an ad74416h_live_status union where the live
 * status will be stored. This must not be null, and the caller is
 * responsible for allocating this memory.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74416h_get_live(struct ad74416h_desc *,
		      union ad74416h_live_status *);

/***************************************************************************//**
 * @brief This function configures and enables the slew rate control for a DAC
 * on a specified channel of the AD74416H device. It should be used when
 * precise control over the rate of change of the DAC output is required.
 * The function must be called with a valid device descriptor and channel
 * number. It is important to ensure that the channel is properly
 * configured before enabling slew rate control. The function will return
 * an error code if the operation fails, which can occur if the device
 * descriptor is invalid or if the channel number is out of range.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ch The channel number on which to enable slew rate control. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param step An enum value of type ad74416h_slew_lin_step specifying the
 * voltage step size for the DAC. Must be a valid enum value.
 * @param rate An enum value of type ad74416h_lin_rate specifying the update
 * rate for the DAC when slew control is enabled. Must be a valid
 * enum value.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74416h_dac_slew_enable(struct ad74416h_desc *, uint32_t,
			     enum ad74416h_slew_lin_step,
			     enum ad74416h_lin_rate);

/***************************************************************************//**
 * @brief This function is used to disable the slew rate control for the DAC on
 * a specified channel of the AD74416H device. It should be called when
 * precise control over the DAC output is required without the influence
 * of slew rate limiting. The function requires a valid device descriptor
 * and a channel number within the supported range. It returns an integer
 * status code indicating success or failure of the operation.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null, and should be properly initialized before
 * calling this function. The caller retains ownership.
 * @param ch An unsigned 32-bit integer representing the channel number. Valid
 * values are 0 to 3, corresponding to channels A to D. If an invalid
 * channel is specified, the function may return an error.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad74416h_dac_slew_disable(struct ad74416h_desc *, uint32_t);

/***************************************************************************//**
 * @brief This function is used to control the thermal reset feature of the
 * AD74416H device. It should be called when there is a need to enable or
 * disable the thermal reset functionality, which can be useful for
 * managing thermal conditions in the device. The function requires a
 * valid device descriptor and a boolean value indicating whether to
 * enable or disable the feature. It returns an integer status code
 * indicating success or failure of the operation.
 *
 * @param desc A pointer to an ad74416h_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param enable A boolean value where 'true' enables the thermal reset and
 * 'false' disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad74416h_set_therm_rst(struct ad74416h_desc *, bool);

/***************************************************************************//**
 * @brief This function resets the AD74416H device, either by toggling a reset
 * GPIO pin if available or by issuing a software reset command sequence.
 * It should be called when a reset of the device is required, such as
 * during initialization or to recover from an error state. The function
 * ensures the device is reset by waiting for the required reset
 * duration. It must be called with a valid device descriptor that has
 * been properly initialized.
 *
 * @param desc A pointer to an `ad74416h_desc` structure representing the
 * device. This must be a valid, initialized descriptor. If the
 * `reset_gpio` member is non-null, it will be used to perform a
 * hardware reset; otherwise, a software reset is performed.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ad74416h_reset(struct ad74416h_desc *);

/***************************************************************************//**
 * @brief This function sets up the AD74416H device by allocating and
 * initializing the necessary resources, including SPI and optional GPIO
 * interfaces. It must be called before any other operations on the
 * device. The function requires a valid initialization parameter
 * structure and will return an error if memory allocation fails or if
 * the initialization of the SPI or GPIO interfaces encounters an issue.
 * The caller is responsible for managing the memory of the descriptor
 * pointer.
 *
 * @param desc A pointer to a pointer of type `struct ad74416h_desc`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and is responsible for freeing it
 * using `ad74416h_remove`.
 * @param init_param A pointer to a `struct ad74416h_init_param` containing
 * initialization parameters such as device ID, device
 * address, SPI initialization parameters, and optional GPIO
 * parameters. Must not be null. If null, the function returns
 * an error.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -EINVAL for invalid parameters,
 * -ENOMEM for memory allocation failure).
 ******************************************************************************/
int ad74416h_init(struct ad74416h_desc **, struct ad74416h_init_param *);

/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD74416h device descriptor when it is no longer needed. This function
 * should be called to avoid memory leaks and to ensure that any
 * associated hardware resources, such as GPIO and SPI, are properly
 * released. It is important to ensure that the descriptor is valid and
 * initialized before calling this function. If the descriptor is null,
 * the function will return an error code.
 *
 * @param desc A pointer to the AD74416h device descriptor to be removed. Must
 * not be null. The function will return -EINVAL if this parameter
 * is null. The caller retains ownership of the pointer, but the
 * resources it points to will be freed.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the removal process.
 ******************************************************************************/
int ad74416h_remove(struct ad74416h_desc *desc);

#endif // _AD74416H_H
