/***************************************************************************//**
 *   @file   ADF7023.h
 *   @brief  Header file of ADF7023 Driver.
 *   @author DBogdan (Dragos.Bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
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
#ifndef __ADF7023_H__
#define __ADF7023_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"

/* Status Word */
#define STATUS_SPI_READY  (0x1 << 7)
#define STATUS_IRQ_STATUS (0x1 << 6)
#define STATUS_CMD_READY  (0x1 << 5)
#define STATUS_FW_STATE   (0x1F << 0)

/* FW_STATE Description */
#define FW_STATE_INIT             0x0F
#define FW_STATE_BUSY             0x00
#define FW_STATE_PHY_OFF          0x11
#define FW_STATE_PHY_ON           0x12
#define FW_STATE_PHY_RX           0x13
#define FW_STATE_PHY_TX           0x14
#define FW_STATE_PHY_SLEEP        0x06
#define FW_STATE_GET_RSSI         0x05
#define FW_STATE_IR_CAL           0x07
#define FW_STATE_AES_DECRYPT_INIT 0x08
#define FW_STATE_AES_DECRYPT      0x09
#define FW_STATE_AES_ENCRYPT      0x0A

/* SPI Memory Access Commands */
#define SPI_MEM_WR  0x18 // Write data to packet RAM sequentially.
#define SPI_MEM_RD  0x38 // Read data from packet RAM sequentially.
#define SPI_MEMR_WR 0x08 // Write data to packet RAM nonsequentially.
#define SPI_MEMR_RD 0x28 // Read data from packet RAM nonsequentially.
#define SPI_NOP     0xFF // No operation.

/* Radio Controller Commands */
#define CMD_SYNC             0xA2 // This is an optional command. It is not necessary to use it during device initialization
#define CMD_PHY_OFF          0xB0 // Performs a transition of the device into the PHY_OFF state.
#define CMD_PHY_ON           0xB1 // Performs a transition of the device into the PHY_ON state.
#define CMD_PHY_RX           0xB2 // Performs a transition of the device into the PHY_RX state.
#define CMD_PHY_TX           0xB5 // Performs a transition of the device into the PHY_TX state.
#define CMD_PHY_SLEEP        0xBA // Performs a transition of the device into the PHY_SLEEP state.
#define CMD_CONFIG_DEV       0xBB // Configures the radio parameters based on the BBRAM values.
#define CMD_GET_RSSI         0xBC // Performs an RSSI measurement.
#define CMD_BB_CAL           0xBE // Performs a calibration of the IF filter.
#define CMD_HW_RESET         0xC8 // Performs a full hardware reset. The device enters the PHY_SLEEP state.
#define CMD_RAM_LOAD_INIT    0xBF // Prepares the program RAM for a firmware module download.
#define CMD_RAM_LOAD_DONE    0xC7 // Performs a reset of the communications processor after download of a firmware module to program RAM.
#define CMD_IR_CAL           0xBD // Initiates an image rejection calibration routine.
#define CMD_AES_ENCRYPT      0xD0 // Performs an AES encryption on the transmit payload data stored in packet RAM.
#define CMD_AES_DECRYPT      0xD2 // Performs an AES decryption on the received payload data stored in packet RAM.
#define CMD_AES_DECRYPT_INIT 0xD1 // Initializes the internal variables required for AES decryption.
#define CMD_RS_ENCODE_INIT   0xD1 // Initializes the internal variables required for the Reed Solomon encoding.
#define CMD_RS_ENCODE        0xD0 // Calculates and appends the Reed Solomon check bytes to the transmit payload data stored in packet RAM.
#define CMD_RS_DECODE        0xD2 // Performs a Reed Solomon error correction on the received payload data stored in packet RAM.

/* Battery Backup Memory (BBRAM) */
#define BBRAM_REG_INTERRUPT_MASK_0                  0x100
#define BBRAM_REG_INTERRUPT_MASK_1                  0x101
#define BBRAM_REG_NUMBER_OF_WAKEUPS_0               0x102
#define BBRAM_REG_NUMBER_OF_WAKEUPS_1               0x103
#define BBRAM_REG_NUMBER_OF_WAKEUPS_IRQ_THRESHOLD_0 0x104
#define BBRAM_REG_NUMBER_OF_WAKEUPS_IRQ_THRESHOLD_1 0x105
#define BBRAM_REG_RX_DWELL_TIME                     0x106
#define BBRAM_REG_PARMTIME_DIVIDER                  0x107
#define BBRAM_REG_SWM_RSSI_THRESH                   0x108
#define BBRAM_REG_CHANNEL_FREQ_0                    0x109
#define BBRAM_REG_CHANNEL_FREQ_1                    0x10A
#define BBRAM_REG_CHANNEL_FREQ_2                    0x10B
#define BBRAM_REG_RADIO_CFG_0                       0x10C
#define BBRAM_REG_RADIO_CFG_1                       0x10D
#define BBRAM_REG_RADIO_CFG_2                       0x10E
#define BBRAM_REG_RADIO_CFG_3                       0x10F
#define BBRAM_REG_RADIO_CFG_4                       0x110
#define BBRAM_REG_RADIO_CFG_5                       0x111
#define BBRAM_REG_RADIO_CFG_6                       0x112
#define BBRAM_REG_RADIO_CFG_7                       0x113
#define BBRAM_REG_RADIO_CFG_8                       0x114
#define BBRAM_REG_RADIO_CFG_9                       0x115
#define BBRAM_REG_RADIO_CFG_10                      0x116
#define BBRAM_REG_RADIO_CFG_11                      0x117
#define BBRAM_REG_IMAGE_REJECT_CAL_PHASE            0x118
#define BBRAM_REG_IMAGE_REJECT_CAL_AMPLITUDE        0x119
#define BBRAM_REG_MODE_CONTROL                      0x11A
#define BBRAM_REG_PREAMBLE_MATCH                    0x11B
#define BBRAM_REG_SYMBOL_MODE                       0x11C
#define BBRAM_REG_PREAMBLE_LEN                      0x11D
#define BBRAM_REG_CRC_POLY_0                        0x11E
#define BBRAM_REG_CRC_POLY_1                        0x11F
#define BBRAM_REG_SYNC_CONTROL                      0x120
#define BBRAM_REG_SYNC_BYTE_0                       0x121
#define BBRAM_REG_SYNC_BYTE_1                       0x122
#define BBRAM_REG_SYNC_BYTE_2                       0x123
#define BBRAM_REG_TX_BASE_ADR                       0x124
#define BBRAM_REG_RX_BASE_ADR                       0x125
#define BBRAM_REG_PACKET_LENGTH_CONTROL             0x126
#define BBRAM_REG_PACKET_LENGTH_MAX                 0x127
#define BBRAM_REG_STATIC_REG_FIX                    0x128
#define BBRAM_REG_ADDRESS_MATCH_OFFSET              0x129
#define BBRAM_REG_ADDRESS_LENGTH                    0x12A
#define BBRAM_REG_ADDRESS_FILTERING_0               0x12B
#define BBRAM_REG_ADDRESS_FILTERING_1               0x12C
#define BBRAM_REG_ADDRESS_FILTERING_2               0x12D
#define BBRAM_REG_ADDRESS_FILTERING_3               0x12E
#define BBRAM_REG_ADDRESS_FILTERING_4               0x12F
#define BBRAM_REG_ADDRESS_FILTERING_5               0x130
#define BBRAM_REG_ADDRESS_FILTERING_6               0x131
#define BBRAM_REG_ADDRESS_FILTERING_7               0x132
#define BBRAM_REG_ADDRESS_FILTERING_8               0x133
#define BBRAM_REG_ADDRESS_FILTERING_9               0x134
#define BBRAM_REG_ADDRESS_FILTERING_10              0x135
#define BBRAM_REG_ADDRESS_FILTERING_11              0x136
#define BBRAM_REG_ADDRESS_FILTERING_12              0x137
#define BBRAM_REG_RSSI_WAIT_TIME                    0x138
#define BBRAM_REG_TESTMODES                         0x139
#define BBRAM_REG_TRANSITION_CLOCK_DIV              0x13A
#define BBRAM_REG_RESERVED_0                        0x13B
#define BBRAM_REG_RESERVED_1                        0x13C
#define BBRAM_REG_RESERVED_2                        0x13D
#define BBRAM_REG_RX_SYNTH_LOCK_TIME                0x13E
#define BBRAM_REG_TX_SYNTH_LOCK_TIME                0x13F

/* BBRAM_REG_INTERRUPT_MASK_0 - 0x100 */
#define BBRAM_INTERRUPT_MASK_0_INTERRUPT_NUM_WAKEUPS     (0x1 << 7)
#define BBRAM_INTERRUPT_MASK_0_INTERRUPT_SWM_RSSI_DET    (0x1 << 6)
#define BBRAM_INTERRUPT_MASK_0_INTERRUPT_AES_DONE        (0x1 << 5)
#define BBRAM_INTERRUPT_MASK_0_INTERRUPT_TX_EOF          (0x1 << 4)
#define BBRAM_INTERRUPT_MASK_0_INTERRUPT_ADDRESS_MATCH   (0x1 << 3)
#define BBRAM_INTERRUPT_MASK_0_INTERRUPT_CRC_CORRECT     (0x1 << 2)
#define BBRAM_INTERRUPT_MASK_0_INTERRUPT_SYNC_DETECT     (0x1 << 1)
#define BBRAM_INTERRUPT_MASK_0_INTERRUPT_PREMABLE_DETECT (0x1 << 0)

/* BBRAM_REG_INTERRUPT_MASK_1 - 0x101*/
#define BBRAM_INTERRUPT_MASK_1_BATTERY_ALARM (0x1 << 7)
#define BBRAM_INTERRUPT_MASK_1_CMD_READY     (0x1 << 6)
#define BBRAM_INTERRUPT_MASK_1_WUC_TIMEOUT   (0x1 << 4)
#define BBRAM_INTERRUPT_MASK_1_SPI_READY     (0x1 << 1)
#define BBRAM_INTERRUPT_MASK_1_CMD_FINISHED  (0x1 << 0)

/* BBRAM_REG_RADIO_CFG_0 - 0x10C */
#define BBRAM_RADIO_CFG_0_DATA_RATE_7_0(x) ((x & 0xFF) << 0)

/* BBRAM_REG_RADIO_CFG_1 - 0x10D */
#define BBRAM_RADIO_CFG_1_FREQ_DEVIATION_11_8(x) ((x & 0xF) << 4)
#define BBRAM_RADIO_CFG_1_DATA_RATE_11_8(x)      ((x & 0xF) << 0)

/* BBRAM_REG_RADIO_CFG_2 - 0x10E */
#define BBRAM_RADIO_CFG_2_FREQ_DEVIATION_7_0(x) ((x & 0xFF) << 0)

/* BBRAM_REG_RADIO_CFG_6 - 0x112 */
#define BBRAM_RADIO_CFG_6_SYNTH_LUT_CONFIG_0(x) ((x & 0x3F) << 2)
#define BBRAM_RADIO_CFG_6_DISCRIM_PHASE(x)      ((x & 0x3) << 0)

/* BBRAM_REG_RADIO_CFG_7 - 0x113 */
#define BBRAM_RADIO_CFG_7_AGC_LOCK_MODE(x)      ((x & 0x3) << 6)
#define BBRAM_RADIO_CFG_7_SYNTH_LUT_CONTROL(x)  ((x & 0x3) << 4)
#define BBRAM_RADIO_CFG_7_SYNTH_LUT_CONFIG_1(x) ((x & 0xF) << 0)

/* BBRAM_REG_RADIO_CFG_8 - 0x114 */
#define BBRAM_RADIO_CFG_8_PA_SINGLE_DIFF_SEL (0x1 << 7)
#define BBRAM_RADIO_CFG_8_PA_LEVEL(x)        ((x & 0xF) << 3)
#define BBRAM_RADIO_CFG_8_PA_RAMP(x)         ((x & 0x7) << 0)

/* BBRAM_REG_RADIO_CFG_9 - 0x115 */
#define BBRAM_RADIO_CFG_9_IFBW(x)         ((x & 0x3) << 6)
#define BBRAM_RADIO_CFG_9_MOD_SCHEME(x)   ((x & 0x7) << 3)
#define BBRAM_RADIO_CFG_9_DEMOD_SCHEME(x) ((x & 0x7) << 0)

/* BBRAM_REG_RADIO_CFG_10 - 0x116 */
#define BBRAM_RADIO_CFG_10_AFC_POLARITY     (0x0 << 4)
#define BBRAM_RADIO_CFG_10_AFC_SCHEME(x)    ((x & 0x3) << 2)
#define BBRAM_RADIO_CFG_10_AFC_LOCK_MODE(x) ((x & 0x3) << 0)

/* BBRAM_REG_RADIO_CFG_11 - 0x117 */
#define BBRAM_RADIO_CFG_11_AFC_KP(x) ((x & 0xF) << 4)
#define BBRAM_RADIO_CFG_11_AFC_KI(x) ((x & 0xF) << 0)

/* BBRAM_REG_MODE_CONTROL - 0x11A */
#define BBRAM_MODE_CONTROL_SWM_EN                        (0x1 << 7)
#define BBRAM_MODE_CONTROL_BB_CAL                        (0x1 << 6)
#define BBRAM_MODE_CONTROL_SWM_RSSI_QUAL                 (0x1 << 5)
#define BBRAM_MODE_CONTROL_TX_TO_RX_AUTO_TURNAROUND      (0x1 << 4)
#define BBRAM_MODE_CONTROL_RX_TO_TX_AUTO_TURNAROUND      (0x1 << 3)
#define BBRAM_MODE_CONTROL_CUSTOM_TRX_SYNTH_LOCK_TIME_EN (0x1 << 2)
#define BBRAM_MODE_CONTROL_EXT_LNA_EN                    (0x1 << 1)
#define BBRAM_MODE_CONTROL_EXT_PA_EN                     (0x1 << 0)

/* BBRAM_REG_SYMBOL_MODE - 0x11C */
#define BBRAM_SYMBOL_MODE_MANCHESTER_ENC   (0x1 << 6)
#define BBRAM_SYMBOL_MODE_PROG_CRC_EN      (0x1 << 5)
#define BBRAM_SYMBOL_MODE_EIGHT_TEN_ENC    (0x1 << 4)
#define BBRAM_SYMBOL_MODE_DATA_WHITENING   (0x1 << 3)
#define BBRAM_SYMBOL_MODE_SYMBOL_LENGTH(x) ((x & 0x7) << 0)

/* BBRAM_REG_SYNC_CONTROL - 0x120 */
#define BBRAM_SYNC_CONTROL_SYNC_ERROR_TOL(x)   ((x & 0x3) << 6)
#define BBRAM_SYNC_CONTROL_SYNC_WORD_LENGTH(x) ((x & 0x1F) << 0)

/* BBRAM_REG_PACKET_LENGTH_CONTROL - 0x126 */
#define BBRAM_PACKET_LENGTH_CONTROL_DATA_BYTE        (0x1 << 7)
#define BBRAM_PACKET_LENGTH_CONTROL_PACKET_LEN       (0x1 << 6)
#define BBRAM_PACKET_LENGTH_CONTROL_CRC_EN           (0x1 << 5)
#define BBRAM_PACKET_LENGTH_CONTROL_DATA_MODE(x)     ((x & 0x3) << 3)
#define BBRAM_PACKET_LENGTH_CONTROL_LENGTH_OFFSET(x) ((x & 0x7) << 0)

/* BBRAM_REG_TESTMODES - 0x139 */
#define BBRAM_TESTMODES_EXT_PA_LNA_ATB_CONFIG (0x1 << 7)
#define BBRAM_TESTMODES_PER_IRQ_SELF_CLEAR    (0x1 << 3)
#define BBRAM_TESTMODES_PER_ENABLE            (0x1 << 2)
#define BBRAM_TESTMODES_CONTINUOUS_TX         (0x1 << 1)
#define BBRAM_TESTMODES_CONTINUOUS_RX         (0x1 << 0)

/* Modem Configuration Memory (MCR) */
#define MCR_REG_PA_LEVEL_MCR                      0x307
#define MCR_REG_WUC_CONFIG_HIGH                   0x30C
#define MCR_REG_WUC_CONFIG_LOW                    0x30D
#define MCR_REG_WUC_VALUE_HIGH                    0x30E
#define MCR_REG_WUC_VALUE_LOW                     0x30F
#define MCR_REG_WUC_FLAG_RESET                    0x310
#define MCR_REG_WUC_STATUS                        0x311
#define MCR_REG_RSSI_READBACK                     0x312
#define MCR_REG_MAX_AFC_RANGE                     0x315
#define MCR_REG_IMAGE_REJECT_CAL_CONFIG           0x319
#define MCR_REG_CHIP_SHUTDOWN                     0x322
#define MCR_REG_POWERDOWN_RX                      0x324
#define MCR_REG_POWERDOWN_AUX                     0x325
#define MCR_REG_ADC_READBACK_HIGH                 0x327
#define MCR_REG_ADC_READBACK_LOW                  0x328
#define MCR_REG_BATTERY_MONITOR_THRESHOLD_VOLTAGE 0x32D
#define MCR_REG_EXT_UC_CLK_DIVIDE                 0x32E
#define MCR_REG_AGC_CLK_DIVIDE                    0x32F
#define MCR_REG_INTERRUPT_SOURCE_0                0x336
#define MCR_REG_INTERRUPT_SOURCE_1                0x337
#define MCR_REG_CALIBRATION_CONTROL               0x338
#define MCR_REG_CALIBRATION_STATUS                0x339
#define MCR_REG_RXBB_CAL_CALWRD_READBACK          0x345
#define MCR_REG_RXBB_CAL_CALWRD_OVERWRITE         0x346
#define MCR_REG_RCOSC_CAL_READBACK_HIGH           0x34F
#define MCR_REG_RCOSC_CAL_READBACK_LOW            0x350
#define MCR_REG_ADC_CONFIG_LOW                    0x359
#define MCR_REG_ADC_CONFIG_HIGH                   0x35A
#define MCR_REG_AGC_OOK_CONTROL                   0x35B
#define MCR_REG_AGC_CONFIG                        0x35C
#define MCR_REG_AGC_MODE                          0x35D
#define MCR_REG_AGC_LOW_THRESHOLD                 0x35E
#define MCR_REG_AGC_HIGH_THRESHOLD                0x35F
#define MCR_REG_AGC_GAIN_STATUS                   0x360
#define MCR_REG_AGC_ADC_WORD                      0x361
#define MCR_REG_FREQUENCY_ERROR_READBACK          0x372
#define MCR_REG_VCO_BAND_OVRW_VAL                 0x3CB
#define MCR_REG_VCO_AMPL_OVRW_VAL                 0x3CC
#define MCR_REG_VCO_OVRW_EN                       0x3CD
#define MCR_REG_VCO_CAL_CFG                       0x3D0
#define MCR_REG_OSC_CONFIG                        0x3D2
#define MCR_REG_VCO_BAND_READBACK                 0x3DA
#define MCR_REG_VCO_AMPL_READBACK                 0x3DB
#define MCR_REG_ANALOG_TEST_BUS                   0x3F8
#define MCR_REG_RSSI_TSTMUX_SEL                   0x3F9
#define MCR_REG_GPIO_CONFIGURE                    0x3FA
#define MCR_REG_TEST_DAC_GAIN                     0x3FD

/***************************************************************************//**
 * @brief The `adf7023_bbram` structure is a comprehensive configuration data
 * structure for the ADF7023 radio transceiver, containing numerous
 * fields that control various aspects of the device's operation. These
 * fields include interrupt masks, wakeup settings, channel frequency,
 * radio configuration parameters, synchronization settings, and address
 * filtering options, among others. Each field is associated with a
 * specific register address, allowing for precise control over the
 * radio's behavior. This structure is essential for configuring the
 * ADF7023 to meet specific application requirements, ensuring optimal
 * performance and functionality.
 *
 * @param interrupt_mask0 Defines the first set of interrupt masks for the
 * device.
 * @param interrupt_mask1 Defines the second set of interrupt masks for the
 * device.
 * @param number_of_wakeups0 Specifies the lower byte of the number of wakeups.
 * @param number_of_wakeups1 Specifies the upper byte of the number of wakeups.
 * @param number_of_wakeups_irq_threshold0 Specifies the lower byte of the
 * wakeup interrupt threshold.
 * @param number_of_wakeups_irq_threshold1 Specifies the upper byte of the
 * wakeup interrupt threshold.
 * @param rx_dwell_time Defines the dwell time for the receiver.
 * @param parmtime_divider Sets the parameter time divider.
 * @param swm_rssi_thresh Sets the RSSI threshold for the software modem.
 * @param channel_freq0 Specifies the lower byte of the channel frequency.
 * @param channel_freq1 Specifies the middle byte of the channel frequency.
 * @param channel_freq2 Specifies the upper byte of the channel frequency.
 * @param radio_cfg0 Contains configuration settings for the radio.
 * @param radio_cfg1 Contains additional configuration settings for the radio.
 * @param radio_cfg2 Contains further configuration settings for the radio.
 * @param radio_cfg3 Contains more configuration settings for the radio.
 * @param radio_cfg4 Contains additional configuration settings for the radio.
 * @param radio_cfg5 Contains further configuration settings for the radio.
 * @param radio_cfg6 Contains more configuration settings for the radio.
 * @param radio_cfg7 Contains additional configuration settings for the radio.
 * @param radio_cfg8 Contains further configuration settings for the radio.
 * @param radio_cfg9 Contains more configuration settings for the radio.
 * @param radio_cfg10 Contains additional configuration settings for the radio.
 * @param radio_cfg11 Contains further configuration settings for the radio.
 * @param image_reject_cal_phase Specifies the phase for image rejection
 * calibration.
 * @param image_reject_cal_amplitude Specifies the amplitude for image rejection
 * calibration.
 * @param mode_control Controls various modes of the device.
 * @param preamble_match Defines the preamble match settings.
 * @param symbol_mode Specifies the symbol mode settings.
 * @param preamble_len Defines the length of the preamble.
 * @param crc_poly0 Specifies the lower byte of the CRC polynomial.
 * @param crc_poly1 Specifies the upper byte of the CRC polynomial.
 * @param syncControl Controls synchronization settings.
 * @param sync_byte0 Specifies the first synchronization byte.
 * @param sync_byte1 Specifies the second synchronization byte.
 * @param sync_byte2 Specifies the third synchronization byte.
 * @param tx_base_adr Defines the base address for transmission.
 * @param rx_base_adr Defines the base address for reception.
 * @param packet_length_control Controls the packet length settings.
 * @param packet_length_max Specifies the maximum packet length.
 * @param static_reg_fix Indicates static register fix settings.
 * @param address_match_offset Specifies the offset for address matching.
 * @param address_length Defines the length of the address.
 * @param address_filtering0 Specifies the first address filtering setting.
 * @param address_filtering1 Specifies the second address filtering setting.
 * @param address_filtering2 Specifies the third address filtering setting.
 * @param address_filtering3 Specifies the fourth address filtering setting.
 * @param address_filtering4 Specifies the fifth address filtering setting.
 * @param address_filtering5 Specifies the sixth address filtering setting.
 * @param address_filtering6 Specifies the seventh address filtering setting.
 * @param address_filtering7 Specifies the eighth address filtering setting.
 * @param address_filtering8 Specifies the ninth address filtering setting.
 * @param address_filtering9 Specifies the tenth address filtering setting.
 * @param address_filtering10 Specifies the eleventh address filtering setting.
 * @param address_filtering11 Specifies the twelfth address filtering setting.
 * @param address_filtering12 Specifies the thirteenth address filtering
 * setting.
 * @param rssi_wait_time Defines the wait time for RSSI measurement.
 * @param testmodes Specifies the test modes for the device.
 * @param transition_clock_div Sets the clock divider for transitions.
 * @param reserved0 Reserved for future use.
 * @param reserved1 Reserved for future use.
 * @param reserved2 Reserved for future use.
 * @param rx_synth_lock_time Specifies the lock time for the RX synthesizer.
 * @param tx_synth_lock_time Specifies the lock time for the TX synthesizer.
 ******************************************************************************/
struct adf7023_bbram {
	uint8_t interrupt_mask0;                  // 0x100
	uint8_t interrupt_mask1;                  // 0x101
	uint8_t number_of_wakeups0;               // 0x102
	uint8_t number_of_wakeups1;               // 0x103
	uint8_t number_of_wakeups_irq_threshold0; // 0x104
	uint8_t number_of_wakeups_irq_threshold1; // 0x105
	uint8_t rx_dwell_time;                    // 0x106
	uint8_t parmtime_divider;                 // 0x107
	uint8_t swm_rssi_thresh;                  // 0x108
	uint8_t channel_freq0;                    // 0x109
	uint8_t channel_freq1;                    // 0x10A
	uint8_t channel_freq2;                    // 0x10B
	uint8_t radio_cfg0;                       // 0x10C
	uint8_t radio_cfg1;                       // 0x10D
	uint8_t radio_cfg2;                       // 0x10E
	uint8_t radio_cfg3;                       // 0x10F
	uint8_t radio_cfg4;                       // 0x110
	uint8_t radio_cfg5;                       // 0x111
	uint8_t radio_cfg6;                       // 0x112
	uint8_t radio_cfg7;                       // 0x113
	uint8_t radio_cfg8;                       // 0x114
	uint8_t radio_cfg9;                       // 0x115
	uint8_t radio_cfg10;                      // 0x116
	uint8_t radio_cfg11;                      // 0x117
	uint8_t image_reject_cal_phase;           // 0x118
	uint8_t image_reject_cal_amplitude;       // 0x119
	uint8_t mode_control;                     // 0x11A
	uint8_t preamble_match;                   // 0x11B
	uint8_t symbol_mode;                      // 0x11C
	uint8_t preamble_len;                     // 0x11D
	uint8_t crc_poly0;                        // 0x11E
	uint8_t crc_poly1;                        // 0x11F
	uint8_t syncControl;                      // 0x120
	uint8_t sync_byte0;                       // 0x121
	uint8_t sync_byte1;                       // 0x122
	uint8_t sync_byte2;                       // 0x123
	uint8_t tx_base_adr;                      // 0x124
	uint8_t rx_base_adr;                      // 0x125
	uint8_t packet_length_control;            // 0x126
	uint8_t packet_length_max;                // 0x127
	uint8_t static_reg_fix;                   // 0x128
	uint8_t address_match_offset;             // 0x129
	uint8_t address_length;                   // 0x12A
	uint8_t address_filtering0;               // 0x12B
	uint8_t address_filtering1;               // 0x12C
	uint8_t address_filtering2;               // 0x12D
	uint8_t address_filtering3;               // 0x12E
	uint8_t address_filtering4;               // 0x12F
	uint8_t address_filtering5;               // 0x130
	uint8_t address_filtering6;               // 0x131
	uint8_t address_filtering7;               // 0x132
	uint8_t address_filtering8;               // 0x133
	uint8_t address_filtering9;               // 0x134
	uint8_t address_filtering10;              // 0x135
	uint8_t address_filtering11;              // 0x136
	uint8_t address_filtering12;              // 0x137
	uint8_t rssi_wait_time;                   // 0x138
	uint8_t testmodes;                        // 0x139
	uint8_t transition_clock_div;             // 0x13A
	uint8_t reserved0;                        // 0x13B
	uint8_t reserved1;                        // 0x13C
	uint8_t reserved2;                        // 0x13D
	uint8_t rx_synth_lock_time;               // 0x13E
	uint8_t tx_synth_lock_time;               // 0x13F
};

#define ADF7023_TX_BASE_ADR 0x10
#define ADF7023_RX_BASE_ADR 0x10

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adf7023_dev` structure is designed to encapsulate the necessary
 * components for interfacing with the ADF7023 transceiver device. It
 * includes pointers to SPI and GPIO descriptors for managing
 * communication and control lines, as well as a structure to hold the
 * current configuration settings of the device. This structure is
 * essential for initializing and operating the ADF7023, providing a
 * centralized representation of the device's state and interface
 * connections.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_cs Pointer to a GPIO descriptor for chip select control.
 * @param gpio_miso Pointer to a GPIO descriptor for MISO line control.
 * @param adf7023_bbram_current Current settings of the ADF7023 device stored in
 * battery-backed RAM.
 ******************************************************************************/
struct adf7023_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_cs;
	struct no_os_gpio_desc	*gpio_miso;
	/* Device Settings */
	struct adf7023_bbram	adf7023_bbram_current;
};

/***************************************************************************//**
 * @brief The `adf7023_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the ADF7023 device.
 * It includes configuration details for the SPI interface and the
 * necessary GPIO pins, specifically the chip select and MISO pins, which
 * are essential for communication with the device. This structure is
 * typically used during the initialization process to ensure that the
 * device is correctly configured for operation.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_cs Holds the initialization parameters for the GPIO chip select
 * pin.
 * @param gpio_miso Holds the initialization parameters for the GPIO MISO pin.
 ******************************************************************************/
struct adf7023_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_cs;
	struct no_os_gpio_init_param	gpio_miso;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Initializes the ADF7023. */
/***************************************************************************//**
 * @brief This function initializes the ADF7023 device by setting up the
 * necessary SPI and GPIO interfaces and configuring the device with
 * default settings. It must be called before any other operations on the
 * ADF7023 device. The function allocates memory for the device structure
 * and initializes the SPI and GPIO descriptors based on the provided
 * initialization parameters. It also ensures the device is ready to
 * accept commands by checking the status and setting the necessary
 * configurations. If initialization fails at any step, the function
 * returns an error code.
 *
 * @param device A pointer to a pointer of type `struct adf7023_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct adf7023_init_param` containing
 * the initialization parameters for SPI and GPIO. This must
 * be properly configured before calling the function.
 * @return Returns 0 on success or a negative error code if initialization
 * fails. The `device` pointer is set to point to the initialized device
 * structure on success.
 ******************************************************************************/
int32_t adf7023_init(struct adf7023_dev **device,
		     struct adf7023_init_param init_param);

/* Free the resources allocated by adf7023_init(). */
/***************************************************************************//**
 * @brief This function is used to release all resources that were allocated for
 * an ADF7023 device during its initialization. It should be called when
 * the device is no longer needed to ensure that all associated
 * resources, such as SPI and GPIO descriptors, are properly freed. This
 * function must be called after the device has been initialized with
 * `adf7023_init`. Failure to call this function may result in resource
 * leaks.
 *
 * @param dev A pointer to an `adf7023_dev` structure representing the device to
 * be removed. This pointer must not be null, and it must point to a
 * valid device structure that was previously initialized. If the
 * pointer is invalid, the behavior is undefined.
 * @return Returns an `int32_t` value indicating the success or failure of the
 * resource deallocation. A return value of 0 indicates success, while a
 * non-zero value indicates an error occurred during the removal
 * process.
 ******************************************************************************/
int32_t adf7023_remove(struct adf7023_dev *dev);

/* Reads the status word of the ADF7023. */
/***************************************************************************//**
 * @brief Use this function to retrieve the current status word from the ADF7023
 * device. This function is typically called to check the device's
 * operational status or to verify that it is ready for further
 * operations. It must be called with a valid device structure that has
 * been properly initialized. The function communicates with the device
 * over SPI and writes the status word to the provided memory location.
 *
 * @param dev A pointer to an initialized adf7023_dev structure representing the
 * device. Must not be null.
 * @param status A pointer to a uint8_t variable where the status word will be
 * stored. Must not be null.
 * @return None
 ******************************************************************************/
void adf7023_get_status(struct adf7023_dev *dev,
			uint8_t* status);

/* Initiates a command. */
/***************************************************************************//**
 * @brief This function is used to send a command to the ADF7023 device, which
 * is a radio transceiver. It should be called when a specific operation
 * or state transition is required on the device. The function asserts
 * the chip select line, sends the command byte, and then deasserts the
 * chip select line, ensuring the command is properly communicated to the
 * device. It is important to ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an adf7023_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param command A uint8_t value representing the command to be sent to the
 * device. This should be a valid command as defined by the
 * device's command set.
 * @return None
 ******************************************************************************/
void adf7023_set_command(struct adf7023_dev *dev,
			 uint8_t command);

/* Sets a FW state and waits until the device enters in that state. */
/***************************************************************************//**
 * @brief This function is used to change the firmware state of an ADF7023
 * device to a specified state and waits until the device successfully
 * transitions to that state. It should be called when a change in the
 * operational state of the device is required, such as transitioning
 * between PHY_OFF, PHY_ON, PHY_RX, PHY_TX, or PHY_SLEEP states. The
 * function ensures that the device has entered the desired state before
 * returning, making it suitable for use in scenarios where state
 * confirmation is necessary.
 *
 * @param dev A pointer to an adf7023_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param fw_state A uint8_t value representing the desired firmware state.
 * Valid values include FW_STATE_PHY_OFF, FW_STATE_PHY_ON,
 * FW_STATE_PHY_RX, FW_STATE_PHY_TX, and FW_STATE_PHY_SLEEP. If
 * an invalid state is provided, the device will transition to
 * the PHY_SLEEP state.
 * @return None
 ******************************************************************************/
void adf7023_set_fw_state(struct adf7023_dev *dev,
			  uint8_t fw_state);

/* Reads data from the RAM. */
/***************************************************************************//**
 * @brief This function is used to read a specified number of bytes from the RAM
 * of an ADF7023 device starting at a given address. It is typically
 * called when data stored in the device's RAM needs to be accessed for
 * processing or analysis. The function requires a valid device structure
 * and a pre-allocated buffer to store the read data. It is important to
 * ensure that the address and length parameters are within the valid
 * range of the device's RAM to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized adf7023_dev structure representing the
 * device. Must not be null.
 * @param address The starting address in the device's RAM from which data will
 * be read. Must be within the valid RAM address range of the
 * device.
 * @param length The number of bytes to read from the RAM. Must be a positive
 * integer and should not exceed the available RAM size from the
 * specified address.
 * @param data A pointer to a buffer where the read data will be stored. The
 * buffer must be pre-allocated and large enough to hold 'length'
 * bytes. Must not be null.
 * @return None
 ******************************************************************************/
void adf7023_get_ram(struct adf7023_dev *dev,
		     uint32_t address,
		     uint32_t length,
		     uint8_t* data);

/* Writes data to RAM. */
/***************************************************************************//**
 * @brief Use this function to write a sequence of bytes to a specific address
 * in the RAM of an ADF7023 device. It is essential to ensure that the
 * device is properly initialized before calling this function. The
 * function requires a valid device structure, a starting address within
 * the device's RAM, the length of the data to be written, and a pointer
 * to the data buffer. The function does not perform any validation on
 * the address or length, so the caller must ensure these parameters are
 * within valid ranges for the device's memory.
 *
 * @param dev A pointer to an initialized adf7023_dev structure representing the
 * device. Must not be null.
 * @param address The starting address in the device's RAM where data will be
 * written. The caller must ensure this is within the valid range
 * of the device's memory.
 * @param length The number of bytes to write to the device's RAM. The caller
 * must ensure this does not exceed the available memory space
 * from the starting address.
 * @param data A pointer to the buffer containing the data to be written. Must
 * not be null and must have at least 'length' bytes available.
 * @return None
 ******************************************************************************/
void adf7023_set_ram(struct adf7023_dev *dev,
		     uint32_t address,
		     uint32_t length,
		     uint8_t* data);

/* Receives one packet. */
/***************************************************************************//**
 * @brief Use this function to receive a packet from the ADF7023 device after it
 * has been properly initialized and configured. The function transitions
 * the device to the PHY_RX state and waits for a valid packet with a
 * correct CRC to be received. It then retrieves the packet length and
 * the packet data itself. This function should be called when the device
 * is ready to receive data, and the caller must ensure that the provided
 * buffers are large enough to hold the expected data.
 *
 * @param dev A pointer to an initialized adf7023_dev structure representing the
 * device. Must not be null.
 * @param packet A pointer to a buffer where the received packet data will be
 * stored. The buffer must be large enough to hold the expected
 * packet data. Must not be null.
 * @param length A pointer to a uint8_t variable where the length of the
 * received packet will be stored. Must not be null.
 * @return None
 ******************************************************************************/
void adf7023_receive_packet(struct adf7023_dev *dev,
			    uint8_t* packet,
			    uint8_t* length);

/* Transmits one packet. */
/***************************************************************************//**
 * @brief Use this function to send a data packet through the ADF7023
 * transceiver. It must be called with a valid device structure and a
 * properly formatted packet. The function sets the device to the PHY_TX
 * state and waits for the transmission to complete. Ensure the device is
 * initialized and configured before calling this function. The packet
 * length should not exceed the maximum allowed by the device.
 *
 * @param dev A pointer to an initialized adf7023_dev structure representing the
 * device. Must not be null.
 * @param packet A pointer to the data buffer containing the packet to be
 * transmitted. The caller retains ownership and must ensure the
 * buffer is valid and contains at least 'length' bytes.
 * @param length The length of the packet to be transmitted. Must be a positive
 * value and should not exceed the device's maximum packet length.
 * @return None
 ******************************************************************************/
void adf7023_transmit_packet(struct adf7023_dev *dev,
			     uint8_t* packet,
			     uint8_t length);

/* Sets the channel frequency. */
/***************************************************************************//**
 * @brief Use this function to configure the channel frequency of the ADF7023
 * device. This function must be called after the device has been
 * initialized and is typically used when setting up or changing the
 * communication parameters of the device. The function takes a frequency
 * value in Hertz and converts it to the appropriate format for the
 * device's internal registers. Ensure that the device is in a state that
 * allows frequency changes before calling this function.
 *
 * @param dev A pointer to an initialized adf7023_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param ch_freq The desired channel frequency in Hertz. Must be a positive
 * integer. The function will convert this value to the
 * appropriate format for the device.
 * @return None
 ******************************************************************************/
void adf7023_set_channel_frequency(struct adf7023_dev *dev,
				   uint32_t ch_freq);

/* Sets the data rate. */
/***************************************************************************//**
 * @brief Use this function to configure the data rate of the ADF7023 device. It
 * must be called with a valid device structure and a desired data rate
 * value. The function updates the device's configuration registers to
 * reflect the new data rate and reconfigures the device accordingly.
 * This function should be called when the device is not actively
 * transmitting or receiving data to avoid communication errors.
 *
 * @param dev A pointer to an initialized adf7023_dev structure representing the
 * device. Must not be null.
 * @param data_rate The desired data rate in bits per second. The value is
 * divided by 100 internally, so it should be a multiple of 100
 * for accurate configuration.
 * @return None
 ******************************************************************************/
void adf7023_set_data_rate(struct adf7023_dev *dev,
			   uint32_t data_rate);

/* Sets the frequency deviation. */
/***************************************************************************//**
 * @brief Use this function to configure the frequency deviation of the ADF7023
 * device, which is a critical parameter for modulation schemes. This
 * function should be called when you need to adjust the frequency
 * deviation to match specific communication requirements. It must be
 * called after the device has been initialized and before starting any
 * transmission or reception operations. The function updates the
 * device's configuration and ensures the device is in the correct state
 * for the new settings to take effect.
 *
 * @param dev A pointer to an initialized adf7023_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param freq_dev The desired frequency deviation in Hertz. The value is
 * divided by 100 internally, so it should be a multiple of 100
 * for precise configuration.
 * @return None
 ******************************************************************************/
void adf7023_set_frequency_deviation(struct adf7023_dev *dev,
				     uint32_t freq_dev);

#endif // __ADF7023_H__
