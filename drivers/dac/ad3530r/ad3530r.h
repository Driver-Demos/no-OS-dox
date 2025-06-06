/***************************************************************************//**
 *   @file   ad3530r.c
 *   @author Sai Kiran Gudla (Saikiran.Gudla@analog.com)
********************************************************************************
 * Copyright (c) 2025 Analog Devices, Inc.
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

#ifndef _AD3530R_H_
#define _AD3530R_H_

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_util.h"
#include "no_os_crc8.h"

/*****************************************************************************/
/******************** Macros and Constants Definitions ***********************/
/*****************************************************************************/


#define AD3530R_R1B          (1ul << 16)
#define AD3530R_R2B          (2ul << 16)
#define AD3530R_LEN(x)       (x >> 16)
#define AD3530R_ADDR(x)      (x & 0xFFFF)

#define AD3530R_MAX_REG_SIZE					2
#define AD3530R_MAX_CHANNEL_OP_MODE_0			4
#define AD3530R_MAX_SHORT_REG_ADDR				(AD3530R_R1B | 0x80)
#define AD3530R_READ_BIT						NO_OS_BIT(7)
#define AD3530R_ADDR_MASK						(~AD3530R_READ_BIT)
#define AD3530R_WRITE_BIT_LONG_INSTR			0x00
#define AD3530R_EXTERNAL_VREF_MASK				NO_OS_BIT(1)
#define AD3530R_DEFAULT_STATUS_REG_VAL			0x04
#define AD3530R_SCRATCH_PAD_TEST_VAL			0x34
#define AD3530R_CRC_POLY						0x07
#define AD3530R_CRC_SEED						0xA5
#define AD3530R_REG_ADDR_OPERATING_MODE_CHN(x) (AD3530R_R1B | (0x20 + x/4))

/* Register addresses */
/* Primary address space */
#define AD3530R_REG_ADDR_INTERFACE_CONFIG_A		(AD3530R_R1B | 0x00)
#define AD3530R_REG_ADDR_INTERFACE_CONFIG_B		(AD3530R_R1B | 0x01)
#define AD3530R_REG_ADDR_DEVICE_CONFIG			(AD3530R_R1B | 0x02)
#define AD3530R_REG_ADDR_CHIP_TYPE			    (AD3530R_R1B | 0x03)
#define AD3530R_REG_ADDR_PRODUCT_ID_L			(AD3530R_R1B | 0x04)
#define AD3530R_REG_ADDR_PRODUCT_ID_H			(AD3530R_R1B | 0x05)
#define AD3530R_REG_ADDR_CHIP_GRADE             (AD3530R_R1B | 0x06)
#define AD3530R_REG_ADDR_SCRATCH_PAD			(AD3530R_R1B | 0x0A)
#define AD3530R_REG_ADDR_SPI_REVISION			(AD3530R_R1B | 0x0B)
#define AD3530R_REG_ADDR_VENDOR_L               (AD3530R_R1B | 0x0C)
#define AD3530R_REG_ADDR_VENDOR_H               (AD3530R_R1B | 0x0D)
#define AD3530R_REG_ADDR_STREAM_MODE			(AD3530R_R1B | 0x0E)
#define AD3530R_REG_ADDR_TRANSFER_REGISTER		(AD3530R_R1B | 0x0F)
#define AD3530R_REG_ADDR_INTERFACE_CONFIG_C		(AD3530R_R1B | 0x10)
#define AD3530R_REG_ADDR_INTERFACE_STATUS_A		(AD3530R_R1B | 0x11)
#define AD3530R_REG_ADDR_OPERATING_MODE_0       (AD3530R_R1B | 0x20)
#define AD3530R_REG_ADDR_OPERATING_MODE_1       (AD3530R_R1B | 0x21)
#define AD3530R_REG_ADDR_OUTPUT_CONTROL_0       (AD3530R_R1B | 0x2A)
#define AD3530R_REG_ADDR_REF_CONTROL_0          (AD3530R_R1B | 0x3C)
#define AD3530R_REG_ADDR_MUX_OUT_SELECT         (AD3530R_R1B | 0x93)
#define AD3530R_REG_ADDR_STATUS_CONTROL         (AD3530R_R1B | 0xC2)

/* DAC configuration registers */
#define AD3530R_REG_ADDR_HW_LDAC_EN_0           (AD3530R_R1B | 0xD0)
#define AD3530R_REG_ADDR_SW_LDAC_EN_0           (AD3530R_R1B | 0xD1)
#define AD3530R_REG_ADDR_DAC_CHN(x)             (AD3530R_R2B | (0xD2 + (x * 2)))
#define AD3530R_REG_ADDR_MULTI_DAC_CH           (AD3530R_R2B | 0XE2)
#define AD3530R_REG_ADDR_MULTI_DAC_SEL_0        (AD3530R_R1B | 0XE4)
#define AD3530R_REG_ADDR_SW_LDAC_TRIG_A         (AD3530R_R1B | 0XE5)
#define AD3530R_REG_ADDR_MULTI_INPUT_CH         (AD3530R_R2B | 0XE6)
#define AD3530R_REG_ADDR_MULTI_INPUT_SEL_0      (AD3530R_R1B | 0XE8)
#define AD3530R_REG_ADDR_SW_LDAC_TRIG_B         (AD3530R_R1B | 0XE9)
#define AD3530R_REG_ADDR_INPUT_CHN(x)           (AD3530R_R2B | (0xEA + (x * 2)))

/* Register masks */
/* AD3530R_REG_ADDR_INTERFACE_CONFIG_A bit masks */
#define AD3530R_MASK_SOFTWARE_RESET             (NO_OS_BIT(7) | NO_OS_BIT(0))
#define AD3530R_MASK_ADDR_ASCENSION			    NO_OS_BIT(5)
#define AD3530R_MASK_SDO_ACTIVE			        NO_OS_BIT(4)

/* AD3530R_REG_ADDR_INTERFACE_CONFIG_B bit masks */
#define AD3530R_MASK_SINGLE_INST			    NO_OS_BIT(7)
#define AD3530R_MASK_SHORT_INSTRUCTION		    NO_OS_BIT(3)

/* AD3530R_REG_ADDR_DEVICE_CONFIG bit masks */
#define AD3530R_MASK_OPERATING_MODES			NO_OS_GENMASK(1, 0)

/* AD3530R_REG_ADDR_CHIP_GRADE bit masks */
#define AD3530R_MASK_GRADE				        NO_OS_GENMASK(7, 4)
#define AD3530R_MASK_DEVICE_REVISION			NO_OS_GENMASK(3, 0)

/* AD3530R_REG_ADDR_STREAM_MODE bit masks */
#define AD3530R_MASK_LENGTH				        0xFF

/* AD3530R_REG_ADDR_TRANSFER_REGISTER bit masks */
#define AD3530R_MASK_STREAM_LENGTH_KEEP_VALUE   NO_OS_BIT(2)

/* AD3530R_REG_ADDR_INTERFACE_CONFIG_C bit masks */
#define AD3530R_MASK_CRC_ENABLE			        (NO_OS_GENMASK(7, 6) | NO_OS_GENMASK(1, 0))
#define AD3530R_MASK_STRICT_REGISTER_ACCESS		NO_OS_BIT(5)
#define AD3530R_MASK_ACTIVE_INTERFACE_MODE		NO_OS_GENMASK(3, 2)

/* AD3530R_REG_ADDR_STREAM_MODE bit masks */
#define AD3530R_MASK_INTERFACE_NOT_READY		NO_OS_BIT(7)
#define AD3530R_MASK_CLOCK_COUNTING_ERROR		NO_OS_BIT(4)
#define AD3530R_MASK_INVALID_OR_NO_CRC          NO_OS_BIT(3)
#define AD3530R_MASK_PARTIAL_REGISTER_ACCESS    NO_OS_BIT(1)

/* AD3530R_REG_ADDR_OPERATING_MODE bit masks */
#define AD3530R_MASK_OPERATING_MODE(x)          0x03 << ((x % 4)*2)

/* AD3530R_REG_ADDR_OUTPUT_CONTROL_0 bit masks */
#define AD3530R_MASK_OUTPUT_RANGE               NO_OS_BIT(2)

/* AD3530R_REG_ADDR_REF_CONTROL_0 bit masks */
#define AD3530R_MASK_REERENCE_SELECT            NO_OS_BIT(0)

/* AD3530R_REG_ADDR_MUX_OUT_SELECT bit masks */
#define AD3530R_MASK_MUX_SELECT                 NO_OS_GENMASK(4, 0)

/* AD3530R_REG_ADDR_HW_LDAC_EN_0 bit masks */
#define AD3530R_MASK_HW_LDAC_EN_0(x)            NO_OS_BIT(x)

/* AD3530R_REG_ADDR_SW_LDAC_EN_0 bit masks */
#define AD3530R_MASK_SW_LDAC_EN_0(x)            NO_OS_BIT(x)

/* AD3530R_REG_ADDR_SW_LDAC_TRIG_B bit masks */
#define AD3530R_MASK_SW_LDAC_TRIG_B             NO_OS_BIT(7)

/* Useful defines */
#define AD3530R_REG_ADDR_MAX	                0xF9
#define AD3530R_NUM_CH					        8
#define AD3530R_MASK_CH(ch)				        NO_OS_BIT(ch)
#define AD3530R_LDAC_PULSE_US                   1
#define AD3530R_CH_DAC_DATA_LSB(x)				((x) & 0xFF)
#define AD3530R_CH_DAC_DATA_MSB(x)				((x) >> 8 & 0xFF)
#define AD3530R_CRC_ENABLE_VALUE			    (NO_OS_BIT(6) | NO_OS_BIT(1))
#define AD3530R_CRC_DISABLE_VALUE			    (NO_OS_BIT(1) | NO_OS_BIT(0))
#define AD3530R_NUM_MUX_OUT_SELECTS				27
#define AD3530R_NUM_REGS			45  // Number of valid registers (mb regs considered a single entity)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `ad3530r_id` enumeration defines a single constant, `AD3530R_ID`,
 * which serves as a unique identifier for the AD3530R device. This
 * enumeration is used to specify the device type in various
 * configurations and operations within the AD3530R driver code.
 *
 * @param AD3530R_ID Represents a unique identifier for the AD3530R device.
 ******************************************************************************/
enum ad3530r_id {
	AD3530R_ID
};

/***************************************************************************//**
 * @brief The `ad3530r_ch_vref_select` enumeration defines the available options
 * for selecting the voltage reference source for the AD3530R device
 * channels. It allows the user to choose between using an external
 * voltage reference or an internal 2.5V reference, which is crucial for
 * configuring the device's reference voltage input/output behavior.
 *
 * @param AD3530R_EXTERNAL_VREF_PIN_INPUT Represents an external voltage
 * reference source with Vref I/O as
 * input.
 * @param AD3530R_INTERNAL_VREF_PIN_2P5V Represents an internal voltage
 * reference source with Vref I/O set to
 * 2.5V.
 ******************************************************************************/
enum ad3530r_ch_vref_select {
	/* External source with Vref I/O as input */
	AD3530R_EXTERNAL_VREF_PIN_INPUT,
	/* Internal source with Vref I/O at 2.5V */
	AD3530R_INTERNAL_VREF_PIN_2P5V,
};

/***************************************************************************//**
 * @brief The `ad3530r_status` enum defines various status and error codes for
 * the AD3530R device, including readiness states and specific error
 * conditions. Each enumerator is associated with a specific bit pattern
 * that represents a particular status or error, allowing for efficient
 * status checking and error handling in the device's operation.
 *
 * @param AD3530R_DEVICE_NOT_READY Indicates that the device is not ready,
 * represented by the bit 0x0001.
 * @param AD3530R_INTERFACE_NOT_READY Indicates that the interface is not ready,
 * represented by the bit 0x0002.
 * @param AD3530R_RESET_STATUS Indicates the reset status, represented by the
 * bit 0x0004.
 * @param AD3530R_DAC_UPDATE_STATUS Indicates the DAC update status, represented
 * by the bit 0x0008.
 * @param AD3530R_PARTIAL_REGISTER_ACCESS Indicates a partial register access
 * error, represented by the bit 0x0002.
 * @param AD3530R_INVALID_OR_NO_CRC Indicates an invalid or missing CRC error,
 * represented by the bit 0x0008.
 * @param AD3530R_CLOCK_COUNTING_ERROR Indicates a clock counting error,
 * represented by the bit 0x0010.
 * @param AD3530R_DEVICE_NOT_READY_ERR Indicates a device not ready error,
 * represented by the bit 0x0080.
 ******************************************************************************/
enum ad3530r_status {
	/* Status bits */
	AD3530R_DEVICE_NOT_READY = 0x0001,
	AD3530R_INTERFACE_NOT_READY = 0x0002,
	AD3530R_RESET_STATUS = 0x0004,
	AD3530R_DAC_UPDATE_STATUS = 0x0008,

	/* Errors */
	AD3530R_PARTIAL_REGISTER_ACCESS = 0x0002,
	AD3530R_INVALID_OR_NO_CRC = 0x0008,
	AD3530R_CLOCK_COUNTING_ERROR = 0x0010,
	AD3530R_DEVICE_NOT_READY_ERR = 0x0080,
};

/***************************************************************************//**
 * @brief The `ad3530r_ch_output_range` enumeration defines the possible output
 * voltage ranges for a channel in the AD3530R device. It provides
 * options for setting the output range either from 0 V to the reference
 * voltage (VREF) or from 0 V to twice the reference voltage (2*VREF).
 * This allows for flexible configuration of the output range based on
 * the application requirements.
 *
 * @param AD3530R_CH_OUTPUT_RANGE_0_VREF Represents a channel output range from
 * 0 V to VREF.
 * @param AD3530R_CH_OUTPUT_RANGE_0_2VREF Represents a channel output range from
 * 0 V to 2*VREF.
 ******************************************************************************/
enum ad3530r_ch_output_range {
	/* Range from 0 V to VREF */
	AD3530R_CH_OUTPUT_RANGE_0_VREF,
	/* Range from 0 V to 2*VREF */
	AD3530R_CH_OUTPUT_RANGE_0_2VREF,
};

/***************************************************************************//**
 * @brief The `ad3530r_operating_mode` enumeration defines the possible
 * operating modes for the AD3530R device's channels. Each mode
 * corresponds to a specific configuration or behavior of the channel,
 * allowing for flexible control over the device's operation. This
 * enumeration is used to set or query the current operating mode of a
 * channel within the AD3530R device.
 *
 * @param AD3530R_CH_OPERATING_MODE_0 Represents the first operating mode for
 * the AD3530R channel.
 * @param AD3530R_CH_OPERATING_MODE_1 Represents the second operating mode for
 * the AD3530R channel.
 * @param AD3530R_CH_OPERATING_MODE_2 Represents the third operating mode for
 * the AD3530R channel.
 * @param AD3530R_CH_OPERATING_MODE_3 Represents the fourth operating mode for
 * the AD3530R channel.
 ******************************************************************************/
enum ad3530r_operating_mode {
	AD3530R_CH_OPERATING_MODE_0,
	AD3530R_CH_OPERATING_MODE_1,
	AD3530R_CH_OPERATING_MODE_2,
	AD3530R_CH_OPERATING_MODE_3,
};

/***************************************************************************//**
 * @brief The `ad3530r_write_mode` enumeration defines the different modes
 * available for writing data to the AD3530R device. It specifies whether
 * the data should be written directly to the DAC registers or to the
 * input registers, and whether the LDAC (Load DAC) signal needs to be
 * manually triggered by the user or automatically by the driver. This
 * allows for flexible control over how and when the DAC outputs are
 * updated.
 *
 * @param AD3530R_WRITE_DAC_REGS Write to DAC registers without needing to
 * trigger LDAC.
 * @param AD3530R_WRITE_INPUT_REGS Write to input registers, requiring the user
 * to trigger LDAC.
 * @param AD3530R_WRITE_INPUT_REGS_AND_TRIGGER_LDAC Write to input registers
 * with LDAC triggered by the
 * driver.
 ******************************************************************************/
enum ad3530r_write_mode {
	/* Write to DAC registers. No need to trigger LDAC */
	AD3530R_WRITE_DAC_REGS,
	/* Write to input registers. User needs to trigger LDAC */
	AD3530R_WRITE_INPUT_REGS,
	/* Write to input registers. LDAC is triggered by the driver */
	AD3530R_WRITE_INPUT_REGS_AND_TRIGGER_LDAC
};

/***************************************************************************//**
 * @brief The `ad3530r_mux_out_select` enumeration defines the various signals
 * that can be monitored on the MUX_OUT pin of the AD3530R device. It
 * includes options for monitoring the output voltage and current modes
 * (source and sink) for each of the eight channels, as well as the die
 * temperature and an option for the pin to be internally tied to analog
 * ground. This enumeration is used to configure the MUX_OUT pin to
 * output the desired signal for diagnostic or monitoring purposes.
 *
 * @param POWERED_DOWN Represents the powered down state of the MUX_OUT pin.
 * @param VOUT0 Selects the output voltage of channel 0 to be monitored on the
 * MUX_OUT pin.
 * @param IOUT0_SOURCE_MODE Selects the source mode current output of channel 0
 * to be monitored on the MUX_OUT pin.
 * @param IOUT0_SINK_MODE Selects the sink mode current output of channel 0 to
 * be monitored on the MUX_OUT pin.
 * @param VOUT1 Selects the output voltage of channel 1 to be monitored on the
 * MUX_OUT pin.
 * @param IOUT1_SOURCE_MODE Selects the source mode current output of channel 1
 * to be monitored on the MUX_OUT pin.
 * @param IOUT1_SINK_MODE Selects the sink mode current output of channel 1 to
 * be monitored on the MUX_OUT pin.
 * @param VOUT2 Selects the output voltage of channel 2 to be monitored on the
 * MUX_OUT pin.
 * @param IOUT2_SOURCE_MODE Selects the source mode current output of channel 2
 * to be monitored on the MUX_OUT pin.
 * @param IOUT2_SINK_MODE Selects the sink mode current output of channel 2 to
 * be monitored on the MUX_OUT pin.
 * @param VOUT3 Selects the output voltage of channel 3 to be monitored on the
 * MUX_OUT pin.
 * @param IOUT3_SOURCE_MODE Selects the source mode current output of channel 3
 * to be monitored on the MUX_OUT pin.
 * @param IOUT3_SINK_MODE Selects the sink mode current output of channel 3 to
 * be monitored on the MUX_OUT pin.
 * @param VOUT4 Selects the output voltage of channel 4 to be monitored on the
 * MUX_OUT pin.
 * @param IOUT4_SOURCE_MODE Selects the source mode current output of channel 4
 * to be monitored on the MUX_OUT pin.
 * @param IOUT4_SINK_MODE Selects the sink mode current output of channel 4 to
 * be monitored on the MUX_OUT pin.
 * @param VOUT5 Selects the output voltage of channel 5 to be monitored on the
 * MUX_OUT pin.
 * @param IOUT5_SOURCE_MODE Selects the source mode current output of channel 5
 * to be monitored on the MUX_OUT pin.
 * @param IOUT5_SINK_MODE Selects the sink mode current output of channel 5 to
 * be monitored on the MUX_OUT pin.
 * @param VOUT6 Selects the output voltage of channel 6 to be monitored on the
 * MUX_OUT pin.
 * @param IOUT6_SOURCE_MODE Selects the source mode current output of channel 6
 * to be monitored on the MUX_OUT pin.
 * @param IOUT6_SINK_MODE Selects the sink mode current output of channel 6 to
 * be monitored on the MUX_OUT pin.
 * @param VOUT7 Selects the output voltage of channel 7 to be monitored on the
 * MUX_OUT pin.
 * @param IOUT7_SOURCE_MODE Selects the source mode current output of channel 7
 * to be monitored on the MUX_OUT pin.
 * @param IOUT7_SINK_MODE Selects the sink mode current output of channel 7 to
 * be monitored on the MUX_OUT pin.
 * @param DIE_TEMPERATURE Selects the die temperature to be monitored on the
 * MUX_OUT pin.
 * @param TIED_TO_AGND_INTERNALLY Indicates that the MUX_OUT pin is internally
 * tied to analog ground.
 ******************************************************************************/
enum ad3530r_mux_out_select {
	POWERED_DOWN,
	VOUT0,
	IOUT0_SOURCE_MODE,
	IOUT0_SINK_MODE,
	VOUT1,
	IOUT1_SOURCE_MODE,
	IOUT1_SINK_MODE,
	VOUT2,
	IOUT2_SOURCE_MODE,
	IOUT2_SINK_MODE,
	VOUT3,
	IOUT3_SOURCE_MODE,
	IOUT3_SINK_MODE,
	VOUT4,
	IOUT4_SOURCE_MODE,
	IOUT4_SINK_MODE,
	VOUT5,
	IOUT5_SOURCE_MODE,
	IOUT5_SINK_MODE,
	VOUT6,
	IOUT6_SOURCE_MODE,
	IOUT6_SINK_MODE,
	VOUT7,
	IOUT7_SOURCE_MODE,
	IOUT7_SINK_MODE,
	DIE_TEMPERATURE,
	TIED_TO_AGND_INTERNALLY
};

/* By default all values are set to 0 */
/***************************************************************************//**
 * @brief The `ad3530r_transfer_config` structure is used to configure the data
 * transfer settings for the AD3530R device. It includes parameters for
 * controlling the streaming mode length, addressing behavior,
 * instruction mode, and address length. Additionally, it provides an
 * option to maintain the stream mode length value without resetting,
 * which is crucial for continuous data streaming operations.
 *
 * @param stream_mode_length Defines the length of the loop when streaming data.
 * @param addr_asc Determines Sequential Addressing Behavior.
 * @param single_instr Selects Streaming or Single Instruction Mode.
 * @param short_instr Determines the length of the address in the instruction
 * phase.
 * @param stream_length_keep_value Prevents the STREAM_MODE LENGTH value from
 * automatically resetting to zero.
 ******************************************************************************/
struct ad3530r_transfer_config {
	/* Defines the length of the loop when streaming data */
	uint8_t		stream_mode_length;
	/* Determines Sequential Addressing Behavior */
	uint8_t		addr_asc;
	/* Select Streaming or Single Instruction Mode */
	uint8_t		single_instr;
	/* Determines the length of the address in the instruction phase */
	uint8_t		short_instr;
	/*
	 * Set this bit to prevent the STREAM_MODE LENGTH value from
	 * automatically resetting to zero
	 */
	uint8_t		stream_length_keep_value;
};

/***************************************************************************//**
 * @brief The `ad3530r_transfer_data` structure is designed to encapsulate the
 * necessary information for a data transfer operation in the AD3530R
 * device. It includes the starting address, the data to be transferred,
 * the size of the data, and a flag to indicate whether the operation is
 * a read or write. Additionally, it holds a pointer to an SPI
 * configuration structure, which can be set to NULL to use the default
 * or previously configured settings. This structure is essential for
 * managing data transactions with the AD3530R device, ensuring that all
 * required parameters are specified for successful communication.
 *
 * @param addr Starting address for the data transfer.
 * @param data Pointer to the data to be transferred.
 * @param len Size of the data to be transferred.
 * @param is_read Indicates if the transaction is a read (true) or write
 * (false).
 * @param spi_cfg Pointer to the SPI configuration, default or last used if
 * NULL.
 ******************************************************************************/
struct ad3530r_transfer_data {
	/* Starting address for transfer */
	uint16_t		addr;
	/* Data to transfer */
	uint8_t		*data;
	/* Size of data to transfer */
	uint32_t	len;
	/* Read transaction if true, write transfer otherwise */
	uint8_t		is_read;
	/* If NULL will be default or last configured will be used */
	struct ad3530r_transfer_config *spi_cfg;
};


/***************************************************************************//**
 * @brief The `ad3530r_desc` structure is a comprehensive descriptor for
 * managing and configuring the AD3530R device, a digital-to-analog
 * converter (DAC). It encapsulates various configuration parameters and
 * hardware interface pointers necessary for device operation, including
 * SPI communication settings, GPIO controls for LDAC and reset
 * functionalities, and channel-specific settings such as voltage
 * reference, operating modes, and output ranges. Additionally, it
 * includes fields for CRC error checking and MUX output selection,
 * making it a central component for controlling the AD3530R's behavior
 * in embedded systems.
 *
 * @param chip_id Identifies the specific AD3530R device.
 * @param spi_cfg Holds the SPI transfer configuration settings.
 * @param spi Pointer to the SPI descriptor for communication.
 * @param ldac Pointer to the GPIO descriptor for the LDAC pin.
 * @param reset Pointer to the GPIO descriptor for the reset pin.
 * @param vref_enable Specifies the voltage reference selection for the
 * channels.
 * @param chn_op_mode Array defining the operating mode for each channel.
 * @param range Defines the output range for the channels.
 * @param hw_ldac_mask Mask for hardware LDAC control.
 * @param sw_ldac_mask Mask for software LDAC control.
 * @param crc_en Flag indicating if CRC is enabled.
 * @param crc_table Array holding the CRC table for error checking.
 * @param mux_out_sel Selects the signal to monitor on the MUX_OUT pin.
 ******************************************************************************/
struct ad3530r_desc {
	enum ad3530r_id	chip_id;
	struct ad3530r_transfer_config spi_cfg;
	struct no_os_spi_desc *spi;
	struct no_os_gpio_desc *ldac;
	struct no_os_gpio_desc *reset;
	enum ad3530r_ch_vref_select vref_enable;
	enum ad3530r_operating_mode chn_op_mode[AD3530R_NUM_CH];
	enum ad3530r_ch_output_range range;
	uint16_t hw_ldac_mask;
	uint16_t sw_ldac_mask;
	uint8_t crc_en;
	uint8_t crc_table[NO_OS_CRC8_TABLE_SIZE];
	enum ad3530r_mux_out_select mux_out_sel;
};

/***************************************************************************//**
 * @brief The `ad3530r_init_param` structure is used to initialize the AD3530R
 * device, encapsulating various configuration parameters such as chip
 * identification, SPI configuration, optional GPIO settings for reset
 * and LDAC, voltage reference selection, channel operating modes, output
 * range, LDAC masks, CRC enablement, and MUX output selection. This
 * structure is essential for setting up the device's initial state and
 * ensuring proper communication and functionality according to the
 * user's requirements.
 *
 * @param chip_id Specifies the ID of the AD3530R chip.
 * @param spi_cfg Holds the SPI transfer configuration settings.
 * @param spi_param Pointer to the SPI initialization parameters.
 * @param reset_gpio_param_optional Optional GPIO parameters for hardware reset.
 * @param ldac_gpio_param_optional Optional GPIO parameters for LDAC pulse
 * control.
 * @param vref_enable Specifies the voltage reference selection for the channel.
 * @param chn_op_mode Array defining the operating mode for each channel.
 * @param range Defines the output range for the channel.
 * @param hw_ldac_mask Mask for hardware LDAC control.
 * @param sw_ldac_mask Mask for software LDAC control.
 * @param crc_en Flag to enable or disable CRC.
 * @param mux_out_sel Selects the signal to monitor on the MUX_OUT pin.
 ******************************************************************************/
struct ad3530r_init_param {
	enum ad3530r_id	chip_id;
	struct ad3530r_transfer_config spi_cfg;
	struct no_os_spi_init_param *spi_param;
	/* If set, reset is done with RESET pin, otherwise it will be soft */
	struct no_os_gpio_init_param *reset_gpio_param_optional;
	/* If set, input register are used and LDAC pulse is sent */
	struct no_os_gpio_init_param *ldac_gpio_param_optional;
	/* If set, uses internal reference and outputs internal Vref on Vref pin */
	enum ad3530r_ch_vref_select vref_enable;
	enum ad3530r_operating_mode chn_op_mode[AD3530R_NUM_CH];
	enum ad3530r_ch_output_range range;
	uint16_t hw_ldac_mask;
	uint16_t sw_ldac_mask;
	/* Set to enable CRC */
	uint8_t crc_en;
	enum ad3530r_mux_out_select mux_out_sel;
};

/*****************************************************************************/
/************************* Functions Declarations ****************************/
/***************************************************************************//**
 * @brief Use this function to write a 16-bit value to a specific register of
 * the AD3530R device. It requires a valid device descriptor and the
 * register address to be specified. The function handles the SPI
 * communication necessary to perform the write operation, including
 * optional CRC validation if enabled. Ensure that the device descriptor
 * is properly initialized before calling this function. The function
 * returns an error code if the descriptor is null or if the SPI
 * communication fails.
 *
 * @param desc A pointer to an initialized ad3530r_desc structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to write to. It is a 32-bit value
 * where the upper bits may define the register length.
 * @param reg_val The 16-bit value to write to the specified register.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for a null descriptor or -EBADMSG for CRC mismatch.
 ******************************************************************************/
int ad3530r_reg_write(struct ad3530r_desc *desc,
		      uint32_t reg_addr,
		      uint16_t reg_val);
/***************************************************************************//**
 * @brief Use this function to read a value from a specified register of the
 * AD3530R device. It requires a valid device descriptor and a register
 * address to read from. The function will store the read value in the
 * provided memory location. Ensure that the descriptor is properly
 * initialized and that the register address is valid for the device. The
 * function handles CRC checking if enabled in the descriptor
 * configuration. It returns an error code if the descriptor or output
 * pointer is null, or if the read operation fails.
 *
 * @param desc A pointer to an initialized ad3530r_desc structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD3530R device.
 * @param reg_val A pointer to a uint16_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid arguments or -EBADMSG for CRC mismatch.
 ******************************************************************************/
int ad3530r_reg_read(struct ad3530r_desc *desc,
		     uint32_t reg_addr,
		     uint16_t *reg_val);
/***************************************************************************//**
 * @brief Use this function to modify specific bits of a register in the AD3530R
 * device by applying a mask and writing a new value. This function is
 * useful when only certain bits of a register need to be updated without
 * affecting the other bits. It reads the current value of the register,
 * applies the mask to clear the bits to be modified, and then writes the
 * new value. Ensure that the device descriptor is properly initialized
 * before calling this function.
 *
 * @param desc A pointer to an initialized ad3530r_desc structure representing
 * the device. Must not be null.
 * @param addr The register address to be accessed. Must be a valid address
 * within the device's register map.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param val The new value to be written to the masked bits of the register.
 * The value is prepared using the mask to ensure only the intended
 * bits are modified.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad3530r_spi_write_mask(struct ad3530r_desc *desc,
			   uint32_t addr, uint32_t mask, uint16_t val);
/***************************************************************************//**
 * @brief This function is used to update the interface configuration settings
 * of an AD3530R device using the provided configuration structure. It
 * must be called with a valid device descriptor and a configuration
 * structure to apply the desired settings. The function modifies the
 * device's interface configuration registers based on the values in the
 * configuration structure. It returns an error code if the descriptor or
 * configuration is null, or if any of the register updates fail.
 *
 * @param desc A pointer to an ad3530r_desc structure representing the device
 * descriptor. Must not be null. The caller retains ownership.
 * @param cfg A pointer to an ad3530r_transfer_config structure containing the
 * new configuration settings. Must not be null. The caller retains
 * ownership.
 * @return Returns 0 on success, or a negative error code if the descriptor or
 * configuration is null, or if any register update fails.
 ******************************************************************************/
int ad3530r_update_interface_cfg(struct ad3530r_desc *desc,
				 struct ad3530r_transfer_config *cfg);
/***************************************************************************//**
 * @brief This function is used to write a sequence of registers starting from a
 * specified address on the AD3530R device. It is essential to ensure
 * that the descriptor and buffer are valid and that the starting address
 * does not exceed the maximum allowed register address. The function
 * supports both short and long instruction modes based on the starting
 * address. It should be called when multiple consecutive registers need
 * to be updated, and it handles CRC if enabled in the descriptor.
 *
 * @param desc A pointer to an ad3530r_desc structure that must be initialized
 * and not null. It contains the device configuration and state.
 * @param start_addr The starting register address for the write operation. It
 * must be less than or equal to AD3530R_REG_ADDR_MAX.
 * @param count The number of registers to write. It determines the length of
 * the data to be written from the buffer.
 * @param buff A pointer to a buffer containing the data to be written. It must
 * not be null and should have at least 'count' bytes of data.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int ad3530r_multiple_reg_write(struct ad3530r_desc *desc,
			       uint32_t start_addr,
			       uint8_t buff_len,
			       uint8_t *buff);
/***************************************************************************//**
 * @brief Use this function to read a sequence of registers from the AD3530R
 * device starting at a specified address. It is essential to ensure that
 * the `desc` and `buff` parameters are valid and not null before calling
 * this function. The function supports reading up to 255 bytes in a
 * single operation, and it automatically handles the selection of short
 * or long instruction modes based on the address. This function should
 * be called only after the device has been properly initialized.
 *
 * @param desc A pointer to an initialized `ad3530r_desc` structure representing
 * the device. Must not be null.
 * @param addr The starting register address from which to begin reading. Must
 * be a valid register address within the device's address space.
 * @param count The number of bytes to read from the device. Must be greater
 * than 0 and less than or equal to 255.
 * @param buff A pointer to a buffer where the read data will be stored. Must
 * not be null and should be large enough to hold `count` bytes.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL if `desc` or `buff` is null.
 ******************************************************************************/
int ad3530r_multiple_reg_read(struct ad3530r_desc *desc,
			      uint32_t addr,
			      uint8_t buff_len,
			      uint8_t *buff);
/***************************************************************************//**
 * @brief This function configures the voltage reference source for the AD3530R
 * device, allowing the user to select between an internal or external
 * reference. It should be called when the reference source needs to be
 * changed, typically during initialization or when reconfiguring the
 * device. The function requires a valid descriptor and a reference
 * selector, and it updates the device's configuration accordingly. It is
 * important to ensure that the descriptor is properly initialized before
 * calling this function.
 *
 * @param desc A pointer to an initialized ad3530r_desc structure. This must not
 * be null, and the structure should be properly configured before
 * use. The function will modify the vref_enable field based on the
 * selected reference.
 * @param reference_selector An enum value of type ad3530r_ch_vref_select,
 * indicating the desired voltage reference source.
 * Valid options are AD3530R_EXTERNAL_VREF_PIN_INPUT
 * for an external reference and
 * AD3530R_INTERNAL_VREF_PIN_2P5V for an internal 2.5V
 * reference.
 * @return Returns 0 on success or a negative error code if the operation fails,
 * indicating an issue with the SPI write operation.
 ******************************************************************************/
int ad3530r_set_reference(struct ad3530r_desc *desc,
			  enum ad3530r_ch_vref_select reference_selector);
/***************************************************************************//**
 * @brief This function configures the operating mode of a specific channel on
 * the AD3530R device. It should be called when you need to change the
 * operating mode of a channel, which is identified by its channel
 * number. The function requires a valid descriptor for the device and a
 * valid channel number within the supported range. It updates the
 * device's internal state to reflect the new operating mode. Ensure that
 * the descriptor is properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad3530r_desc structure representing
 * the device. Must not be null.
 * @param chn_num The channel number to configure. Must be within the valid
 * range of channels supported by the device.
 * @param chn_op_mode The desired operating mode for the specified channel,
 * represented by an ad3530r_operating_mode enum value.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad3530r_set_operating_mode(struct ad3530r_desc *desc,
			       uint8_t chn_num,
			       enum ad3530r_operating_mode chn_op_mode);
/***************************************************************************//**
 * @brief This function configures the output range of the AD3530R device to the
 * specified range. It should be called when you need to change the
 * output voltage range of the device channels. The function requires a
 * valid device descriptor and a range selection from the predefined
 * output range options. It updates the device's internal state to
 * reflect the new range setting. Ensure that the device descriptor is
 * properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized `ad3530r_desc` structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param range_sel An `enum ad3530r_ch_output_range` value specifying the
 * desired output range. Must be a valid range option defined
 * in the enumeration.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad3530r_set_output_range(struct ad3530r_desc *desc,
			     enum ad3530r_ch_output_range range_sel);
/***************************************************************************//**
 * @brief This function is used to enable or disable the CRC (Cyclic Redundancy
 * Check) feature on the AD3530R device. It should be called when there
 * is a need to change the CRC setting, typically during device
 * configuration or initialization. The function updates the device's
 * configuration register to reflect the desired CRC state and also
 * updates the descriptor to keep track of the current CRC setting. It is
 * important to ensure that the device descriptor is properly initialized
 * before calling this function.
 *
 * @param desc A pointer to an initialized `ad3530r_desc` structure representing
 * the device. Must not be null.
 * @param en_di A boolean value where `true` enables CRC and `false` disables
 * it.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad3530r_set_crc_enable(struct ad3530r_desc *desc, bool en_di);
/***************************************************************************//**
 * @brief This function configures the MUX_OUT pin of the AD3530R device to
 * output a specified signal. It should be called when you need to change
 * the signal being monitored on the MUX_OUT pin. The function requires a
 * valid device descriptor and a valid output selection from the
 * predefined enumeration. It updates the device's internal state to
 * reflect the new selection. Ensure that the device descriptor is
 * properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad3530r_desc structure representing
 * the device. Must not be null.
 * @param mux_output_sel An enumerated value of type ad3530r_mux_out_select
 * specifying the desired output signal for the MUX_OUT
 * pin. Must be a valid enumeration value.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad3530r_set_mux_out_select(struct ad3530r_desc *desc,
			       enum ad3530r_mux_out_select mux_output_sel);
/***************************************************************************//**
 * @brief This function sets the hardware Load DAC (LDAC) mask for the AD3530R
 * device, which determines which DAC channels are updated when a
 * hardware LDAC signal is triggered. It should be called when you need
 * to configure or change the hardware LDAC behavior of the device.
 * Ensure that the device descriptor is properly initialized before
 * calling this function. The function updates the device's internal
 * state to reflect the new LDAC mask.
 *
 * @param desc A pointer to an initialized `ad3530r_desc` structure representing
 * the device. Must not be null.
 * @param mask_hw_ldac A 16-bit mask specifying which DAC channels should
 * respond to a hardware LDAC signal. Each bit corresponds
 * to a channel, and only valid channel bits should be set.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad3530r_set_hw_ldac(struct ad3530r_desc *desc, uint16_t mask_hw_ldac);
/***************************************************************************//**
 * @brief This function configures the software LDAC (Load DAC) mask for the
 * AD3530R device, which determines which DAC channels are updated when a
 * software LDAC trigger is issued. It should be called when you need to
 * update the LDAC configuration for specific channels. The function must
 * be called with a valid device descriptor and a mask that specifies the
 * channels to be affected. It updates the internal state of the
 * descriptor with the new mask.
 *
 * @param desc A pointer to an ad3530r_desc structure representing the device.
 * Must not be null, and should be properly initialized before
 * calling this function.
 * @param mask_sw_ldac A 16-bit mask specifying which DAC channels should be
 * affected by the software LDAC. Each bit corresponds to a
 * channel, and only valid channel bits should be set.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad3530r_set_sw_ldac(struct ad3530r_desc *desc, uint16_t mask_sw_ldac);
/***************************************************************************//**
 * @brief This function sets the digital-to-analog converter (DAC) value for a
 * specified channel on the AD3530R device. It can write the value
 * directly to the DAC registers or to the input registers, with an
 * optional trigger of the LDAC (Load DAC) signal. The function must be
 * called with a valid device descriptor and appropriate parameters for
 * the DAC value, channel, and write mode. It handles triggering the LDAC
 * signal either through hardware or software if required by the write
 * mode. The function returns an error code if the operation fails,
 * otherwise it returns 0 on success.
 *
 * @param desc A pointer to an ad3530r_desc structure representing the device
 * descriptor. Must not be null and should be properly initialized
 * before calling this function.
 * @param dac_value A 16-bit unsigned integer representing the DAC value to be
 * set. The valid range depends on the DAC's resolution and
 * configuration.
 * @param dac_channel An 8-bit unsigned integer specifying the DAC channel to
 * which the value should be written. The channel number must
 * be within the valid range supported by the device.
 * @param write_mode An enum value of type ad3530r_write_mode indicating the
 * write mode. It determines whether the value is written to
 * the DAC registers or input registers, and whether the LDAC
 * signal is triggered automatically.
 * @return Returns 0 on success, or a non-zero error code if the operation
 * fails.
 ******************************************************************************/
int ad3530r_set_dac_value(struct ad3530r_desc *desc,
			  uint16_t dac_value,
			  uint8_t dac_channel,
			  enum ad3530r_write_mode write_mode);
/***************************************************************************//**
 * @brief This function sets the specified value to multiple DAC channels on the
 * AD3530R device, as indicated by the channel mask. It can write
 * directly to DAC registers or to input registers, with an optional LDAC
 * trigger, depending on the selected write mode. This function should be
 * called when you need to update the output of multiple DAC channels
 * simultaneously. Ensure that the device descriptor is properly
 * initialized before calling this function. The function returns an
 * error code if the operation fails.
 *
 * @param desc A pointer to an initialized ad3530r_desc structure representing
 * the device. Must not be null.
 * @param dac_value The 16-bit value to be set for the selected DAC channels.
 * Valid range is 0 to 65535.
 * @param dac_chn_mask A 16-bit mask indicating which DAC channels to update.
 * Each bit corresponds to a channel.
 * @param write_mode An enum value of type ad3530r_write_mode specifying how the
 * value should be written (e.g., directly to DAC registers or
 * to input registers with or without LDAC trigger).
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad3530r_set_multidac_value(struct ad3530r_desc *desc,
			       uint16_t dac_value,
			       uint16_t dac_chn_mask,
			       enum ad3530r_write_mode write_mode);
/***************************************************************************//**
 * @brief This function is used to trigger a software Load DAC (LDAC) event on
 * the AD3530R device, which updates the DAC outputs with the values
 * stored in the input registers. It should be called when the input
 * registers have been updated and the new values need to be applied to
 * the DAC outputs. The function requires a valid device descriptor and
 * will return an error if the descriptor is null.
 *
 * @param desc A pointer to an ad3530r_desc structure representing the device
 * descriptor. Must not be null. If null, the function returns an
 * error.
 * @return Returns 0 on success or a negative error code if the descriptor is
 * null or if the operation fails.
 ******************************************************************************/
int ad3530r_sw_ldac_trigger(struct ad3530r_desc *desc);
/***************************************************************************//**
 * @brief This function is used to trigger a hardware Load DAC (LDAC) pulse on
 * the AD3530R device, which updates the DAC outputs with the values
 * stored in the input registers. It must be called with a valid
 * descriptor that has been properly initialized and configured with a
 * valid LDAC GPIO descriptor. The function will return an error if the
 * descriptor or the LDAC GPIO is not set. It is typically used when the
 * device is configured to update DAC outputs via hardware LDAC control.
 *
 * @param desc A pointer to an ad3530r_desc structure that must be initialized
 * and must contain a valid LDAC GPIO descriptor. The pointer must
 * not be null, and the function will return an error if these
 * conditions are not met.
 * @return Returns 0 on success, or a negative error code if the descriptor is
 * invalid or if there is a failure in setting the GPIO value.
 ******************************************************************************/
int ad3530r_hw_ldac_trigger(struct ad3530r_desc *desc);
/***************************************************************************//**
 * @brief Use this function to reset the AD3530R device, either through a
 * hardware reset using the reset GPIO pin or a software reset if the pin
 * is not available. This function should be called when the device needs
 * to be reinitialized to its default state. It ensures that the device's
 * configuration is reset, and it verifies the reset by checking the
 * status register. The function must be called with a valid descriptor,
 * and it will return an error if the descriptor is null or if the reset
 * operation fails.
 *
 * @param desc A pointer to an ad3530r_desc structure representing the device
 * descriptor. Must not be null. The function will return an error
 * if this parameter is invalid.
 * @return Returns 0 on success, a negative error code on failure, such as
 * -EINVAL if the descriptor is null or -ENODEV if the device status
 * check fails after reset.
 ******************************************************************************/
int ad3530r_reset(struct ad3530r_desc *desc);
/***************************************************************************//**
 * @brief This function sets up and initializes the AD3530R device descriptor
 * based on the provided initialization parameters. It must be called
 * before any other operations on the AD3530R device. The function
 * allocates memory for the descriptor, initializes SPI and optional GPIO
 * interfaces, and configures the device according to the specified
 * parameters. If any step fails, the function ensures proper cleanup and
 * returns an error code. The caller is responsible for providing valid
 * initialization parameters and handling the descriptor's lifecycle,
 * including its removal.
 *
 * @param desc A pointer to a pointer of type `struct ad3530r_desc`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad3530r_init_param` containing the
 * initialization parameters for the device. Must not be null.
 * The structure should be properly populated with valid
 * configuration settings before calling this function.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, `*desc` is set to point to a newly allocated and initialized
 * descriptor.
 ******************************************************************************/
int ad3530r_init(struct ad3530r_desc **desc,
		 struct ad3530r_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD3530R device descriptor when it is no longer needed. This function
 * should be called to clean up after using the device to ensure that all
 * allocated resources, such as GPIO and SPI interfaces, are correctly
 * freed. It is important to pass a valid descriptor to avoid undefined
 * behavior. If the descriptor is null, the function will return an error
 * code indicating invalid input.
 *
 * @param desc A pointer to an ad3530r_desc structure representing the device
 * descriptor to be removed. Must not be null. If null, the function
 * returns -EINVAL.
 * @return Returns 0 on success, or a negative error code if any resource
 * removal fails.
 ******************************************************************************/
int ad3530r_remove(struct ad3530r_desc *desc);

#endif /* _AD3530R_H_ */