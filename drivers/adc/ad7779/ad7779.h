/***************************************************************************//**
 *   @file   ad7779.h
 *   @brief  Header file of AD7779 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2016(c) Analog Devices, Inc.
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
#ifndef AD7779_H_
#define AD7779_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD7779_REG_CH_CONFIG(ch)		(0x00 + (ch))		// Channel Configuration
#define AD7779_REG_CH_DISABLE			0x08			// Disable clocks to ADC channel
#define AD7779_REG_CH_SYNC_OFFSET(ch)		(0x09 + (ch))		// Channel SYNC Offset
#define AD7779_REG_GENERAL_USER_CONFIG_1	0x11			// General User Config 1
#define AD7779_REG_GENERAL_USER_CONFIG_2	0x12			// General User Config 2
#define AD7779_REG_GENERAL_USER_CONFIG_3	0x13			// General User Config 3
#define AD7779_REG_DOUT_FORMAT			0x14			// Data out format
#define AD7779_REG_ADC_MUX_CONFIG		0x15			// Main ADC meter and reference Mux control
#define AD7779_REG_GLOBAL_MUX_CONFIG		0x16			// Global diagnostics mux
#define AD7779_REG_GPIO_CONFIG			0x17			// GPIO config
#define AD7779_REG_GPIO_DATA			0x18			// GPIO Data
#define AD7779_REG_BUFFER_CONFIG_1		0x19			// Buffer Config 1
#define AD7779_REG_BUFFER_CONFIG_2		0x1A			// Buffer Config 2
#define AD7779_REG_CH_OFFSET_UPPER_BYTE(ch)	(0x1C + (ch) * 6)	// Channel offset upper byte
#define AD7779_REG_CH_OFFSET_MID_BYTE(ch)	(0x1D + (ch) * 6)	// Channel offset middle byte
#define AD7779_REG_CH_OFFSET_LOWER_BYTE(ch)	(0x1E + (ch) * 6)	// Channel offset lower byte
#define AD7779_REG_CH_GAIN_UPPER_BYTE(ch)	(0x1F + (ch) * 6)	// Channel gain upper byte
#define AD7779_REG_CH_GAIN_MID_BYTE(ch)		(0x20 + (ch) * 6)	// Channel gain middle byte
#define AD7779_REG_CH_GAIN_LOWER_BYTE(ch)	(0x21 + (ch) * 6)	// Channel gain lower byte
#define AD7779_REG_CH_ERR_REG(ch)		(0x4C + (ch))		// Channel Status Register
#define AD7779_REG_CH0_1_SAT_ERR		0x54			// Channel 0/1 DSP errors
#define AD7779_REG_CH2_3_SAT_ERR		0x55			// Channel 2/3 DSP errors
#define AD7779_REG_CH4_5_SAT_ERR		0x56			// Channel 4/5 DSP errors
#define AD7779_REG_CH6_7_SAT_ERR		0x57			// Channel 6/7 DSP errors
#define AD7779_REG_CHX_ERR_REG_EN		0x58			// Channel 0-7 Error Reg Enable
#define AD7779_REG_GEN_ERR_REG_1		0x59			// General Errors Register 1
#define AD7779_REG_GEN_ERR_REG_1_EN		0x5A			// General Errors Register 1 Enable
#define AD7779_REG_GEN_ERR_REG_2		0x5B			// General Errors Register 2
#define AD7779_REG_GEN_ERR_REG_2_EN		0x5C			// General Errors Register 2 Enable
#define AD7779_REG_STATUS_REG_1			0x5D			// Error Status Register 1
#define AD7779_REG_STATUS_REG_2			0x5E			// Error Status Register 2
#define AD7779_REG_STATUS_REG_3			0x5F			// Error Status Register 3
#define AD7779_REG_SRC_N_MSB			0x60			// Decimation Rate (N) MSB
#define AD7779_REG_SRC_N_LSB			0x61			// Decimation Rate (N) LSB
#define AD7779_REG_SRC_IF_MSB			0x62			// Decimation Rate (IF) MSB
#define AD7779_REG_SRC_IF_LSB			0x63			// Decimation Rate (IF) LSB
#define AD7779_REG_SRC_UPDATE			0x64			// SRC load source and load update

/* AD7779_REG_CHx_CONFIG */
#define AD7779_CH_GAIN(x)			(((x) & 0x3) << 6)
#define AD7779_CH_RX				(1 << 4)

/* AD7779_REG_CH_DISABLE */
#define AD7779_CH_DISABLE(x)			(1 << (x))

/* AD7779_REG_GENERAL_USER_CONFIG_1 */
#define AD7779_ALL_CH_DIS_MCLK_EN		(1 << 7)
#define AD7779_MOD_POWERMODE			(1 << 6)
#define AD7779_PDB_VCM				(1 << 5)
#define AD7779_PDB_REFOUT_BUF			(1 << 4)
#define AD7779_PDB_SAR				(1 << 3)
#define AD7779_PDB_RC_OSC			(1 << 2)
#define AD7779_SOFT_RESET(x)			(((x) & 0x3) << 0)

/* AD7779_REG_GENERAL_USER_CONFIG_2 */
#define AD7771_FILTER_MODE			(1 << 6)
#define AD7779_SAR_DIAG_MODE_EN			(1 << 5)
#define AD7779_SDO_DRIVE_STR(x)			(((x) & 0x3) << 3)
#define AD7779_DOUT_DRIVE_STR(x)		(((x) & 0x3) << 1)
#define AD7779_SPI_SYNC				(1 << 0)

/* AD7779_REG_GENERAL_USER_CONFIG_3 */
#define AD7779_CONVST_DEGLITCH_DIS(x)		(((x) & 0x3) << 6)
#define AD7779_SPI_SLAVE_MODE_EN		(1 << 4)
#define AD7779_CLK_QUAL_DIS			(1 << 0)

/* AD7779_REG_DOUT_FORMAT */
#define AD7779_DOUT_FORMAT(x)			(((x) & 0x3) << 6)
#define AD7779_DOUT_HEADER_FORMAT		(1 << 5)
#define AD7779_DCLK_CLK_DIV(x)			(((x) & 0x7) << 1)

/* AD7779_REG_ADC_MUX_CONFIG */
#define AD7779_REF_MUX_CTRL(x)			(((x) & 0x3) << 6)

/* AD7779_REG_GLOBAL_MUX_CONFIG */
#define AD7779_GLOBAL_MUX_CTRL(x)		(((x) & 0x1F) << 3)

/* AD7779_REG_BUFFER_CONFIG_1 */
#define AD7779_REF_BUF_POS_EN			(1 << 4)
#define AD7779_REF_BUF_NEG_EN			(1 << 3)

/* AD7779_REG_BUFFER_CONFIG_2 */
#define AD7779_REFBUFP_PREQ			(1 << 7)
#define AD7779_REFBUFN_PREQ			(1 << 6)
#define AD7779_PDB_ALDO1_OVRDRV			(1 << 2)
#define AD7779_PDB_ALDO2_OVRDRV			(1 << 1)
#define AD7779_PDB_DLDO_OVRDRV			(1 << 0)

/* AD7779_REG_GEN_ERR_REG_1_EN */
#define AD7779_MEMMAP_CRC_TEST_EN		(1 << 5)
#define AD7779_ROM_CRC_TEST_EN			(1 << 4)
#define AD7779_SPI_CLK_COUNT_TEST_EN		(1 << 3)
#define AD7779_SPI_INVALID_READ_TEST_EN		(1 << 2)
#define AD7779_SPI_INVALID_WRITE_TEST_EN	(1 << 1)
#define AD7779_SPI_CRC_TEST_EN			(1 << 0)

#define AD7779_CRC8_POLY			0x07

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad7779_ctrl_mode` is an enumeration that defines the control
 * modes for the AD7779 device, allowing it to be controlled either
 * through pin configuration (`AD7779_PIN_CTRL`) or through SPI
 * communication (`AD7779_SPI_CTRL`). This enumeration is used to specify
 * how the device will be interfaced and controlled, providing
 * flexibility in the hardware setup.
 *
 * @param AD7779_PIN_CTRL Represents the control mode using pin configuration.
 * @param AD7779_SPI_CTRL Represents the control mode using SPI configuration.
 ******************************************************************************/
typedef enum {
	AD7779_PIN_CTRL,
	AD7779_SPI_CTRL,
} ad7779_ctrl_mode;

/***************************************************************************//**
 * @brief The `ad7779_spi_op_mode` is an enumeration that defines the different
 * operation modes for the AD7779 SPI interface. It includes modes for
 * internal register access, sigma-delta conversion, and successive
 * approximation register conversion, allowing the user to configure the
 * SPI interface according to the desired operation of the AD7779 device.
 *
 * @param AD7779_INT_REG Represents the internal register operation mode for the
 * AD7779 SPI interface.
 * @param AD7779_SD_CONV Represents the sigma-delta conversion operation mode
 * for the AD7779 SPI interface.
 * @param AD7779_SAR_CONV Represents the successive approximation register
 * conversion operation mode for the AD7779 SPI
 * interface.
 ******************************************************************************/
typedef enum {
	AD7779_INT_REG,
	AD7779_SD_CONV,
	AD7779_SAR_CONV,
} ad7779_spi_op_mode;

/***************************************************************************//**
 * @brief The `ad7779_ch` enumeration defines the available channels (0 through
 * 7) for the AD7779 device, which is a multi-channel analog-to-digital
 * converter. Each enumerator corresponds to a specific channel on the
 * device, allowing for easy reference and manipulation of individual
 * channels in the software.
 *
 * @param AD7779_CH0 Represents channel 0 of the AD7779 device.
 * @param AD7779_CH1 Represents channel 1 of the AD7779 device.
 * @param AD7779_CH2 Represents channel 2 of the AD7779 device.
 * @param AD7779_CH3 Represents channel 3 of the AD7779 device.
 * @param AD7779_CH4 Represents channel 4 of the AD7779 device.
 * @param AD7779_CH5 Represents channel 5 of the AD7779 device.
 * @param AD7779_CH6 Represents channel 6 of the AD7779 device.
 * @param AD7779_CH7 Represents channel 7 of the AD7779 device.
 ******************************************************************************/
typedef enum {
	AD7779_CH0,
	AD7779_CH1,
	AD7779_CH2,
	AD7779_CH3,
	AD7779_CH4,
	AD7779_CH5,
	AD7779_CH6,
	AD7779_CH7,
} ad7779_ch;

/***************************************************************************//**
 * @brief The `ad7779_state` is an enumeration that defines the possible states
 * of the AD7779 device, specifically whether it is enabled or disabled.
 * This enum is used to manage and control the operational state of the
 * device, allowing for easy toggling between active and inactive modes.
 *
 * @param AD7779_ENABLE Represents the enabled state for the AD7779 device.
 * @param AD7779_DISABLE Represents the disabled state for the AD7779 device.
 ******************************************************************************/
typedef enum {
	AD7779_ENABLE,
	AD7779_DISABLE,
} ad7779_state;

/***************************************************************************//**
 * @brief The `ad7779_gain` enumeration defines the possible gain settings for
 * the AD7779 device, which is an analog-to-digital converter. Each
 * enumerator corresponds to a specific gain factor that can be applied
 * to the input signal, allowing for amplification by factors of 1, 2, 4,
 * or 8. This gain setting is crucial for adjusting the input signal
 * level to optimize the dynamic range and performance of the ADC.
 *
 * @param AD7779_GAIN_1 Represents a gain setting of 1 for the AD7779 device.
 * @param AD7779_GAIN_2 Represents a gain setting of 2 for the AD7779 device.
 * @param AD7779_GAIN_4 Represents a gain setting of 4 for the AD7779 device.
 * @param AD7779_GAIN_8 Represents a gain setting of 8 for the AD7779 device.
 ******************************************************************************/
typedef enum {
	AD7779_GAIN_1,
	AD7779_GAIN_2,
	AD7779_GAIN_4,
	AD7779_GAIN_8,
} ad7779_gain;

/***************************************************************************//**
 * @brief The `ad7779_dclk_div` is an enumeration that defines various clock
 * division factors for the AD7779 device, allowing the user to select
 * the desired division ratio for the device's clock signal. This is
 * useful for configuring the device's timing and synchronization with
 * other components in a system.
 *
 * @param AD7779_DCLK_DIV_1 Represents a clock division factor of 1.
 * @param AD7779_DCLK_DIV_2 Represents a clock division factor of 2.
 * @param AD7779_DCLK_DIV_4 Represents a clock division factor of 4.
 * @param AD7779_DCLK_DIV_8 Represents a clock division factor of 8.
 * @param AD7779_DCLK_DIV_16 Represents a clock division factor of 16.
 * @param AD7779_DCLK_DIV_32 Represents a clock division factor of 32.
 * @param AD7779_DCLK_DIV_64 Represents a clock division factor of 64.
 * @param AD7779_DCLK_DIV_128 Represents a clock division factor of 128.
 ******************************************************************************/
typedef enum {
	AD7779_DCLK_DIV_1,
	AD7779_DCLK_DIV_2,
	AD7779_DCLK_DIV_4,
	AD7779_DCLK_DIV_8,
	AD7779_DCLK_DIV_16,
	AD7779_DCLK_DIV_32,
	AD7779_DCLK_DIV_64,
	AD7779_DCLK_DIV_128,
} ad7779_dclk_div;

/***************************************************************************//**
 * @brief The `ad7779_pwr_mode` is an enumeration that defines the power modes
 * available for the AD7779 device, which is a high-performance analog-
 * to-digital converter. It provides two modes: `AD7779_LOW_PWR` for low
 * power consumption and `AD7779_HIGH_RES` for high resolution, allowing
 * users to select the appropriate mode based on their application
 * requirements.
 *
 * @param AD7779_LOW_PWR Represents the low power mode for the AD7779 device.
 * @param AD7779_HIGH_RES Represents the high resolution mode for the AD7779
 * device.
 ******************************************************************************/
typedef enum {
	AD7779_LOW_PWR,
	AD7779_HIGH_RES,
} ad7779_pwr_mode;

/***************************************************************************//**
 * @brief The `ad7779_ref_type` is an enumeration that defines the different
 * types of reference sources that can be used with the AD7779 device. It
 * includes options for external and internal references, as well as an
 * external supply and an inverted external reference. This enumeration
 * is used to configure the reference type for the AD7779, which is a
 * critical setting for its operation.
 *
 * @param AD7779_EXT_REF Represents an external reference type for the AD7779.
 * @param AD7779_INT_REF Represents an internal reference type for the AD7779.
 * @param AD7779_EXT_SUPPLY Represents an external supply reference type for the
 * AD7779.
 * @param AD7779_EXT_REF_INV Represents an inverted external reference type for
 * the AD7779.
 ******************************************************************************/
typedef enum {
	AD7779_EXT_REF,
	AD7779_INT_REF,
	AD7779_EXT_SUPPLY,
	AD7779_EXT_REF_INV,
} ad7779_ref_type;

/***************************************************************************//**
 * @brief The `ad7779_refx_pin` is an enumeration that defines the reference
 * pins for the AD7779 device, specifically distinguishing between the
 * positive (AD7779_REFX_P) and negative (AD7779_REFX_N) reference pins.
 * This enumeration is used to specify which reference pin is being
 * configured or utilized in the context of the AD7779's operation.
 *
 * @param AD7779_REFX_P Represents the positive reference pin for the AD7779
 * device.
 * @param AD7779_REFX_N Represents the negative reference pin for the AD7779
 * device.
 ******************************************************************************/
typedef enum {
	AD7779_REFX_P,
	AD7779_REFX_N,
} ad7779_refx_pin;

/***************************************************************************//**
 * @brief The `ad7779_ref_buf_op_mode` is an enumeration that defines the
 * operational modes of the reference buffer in the AD7779 device. It
 * includes three possible states: enabled, precharged, and disabled,
 * which control the behavior of the reference buffer in the device's
 * analog-to-digital conversion process.
 *
 * @param AD7779_REF_BUF_ENABLED Represents the enabled state of the reference
 * buffer.
 * @param AD7779_REF_BUF_PRECHARGED Represents the precharged state of the
 * reference buffer.
 * @param AD7779_REF_BUF_DISABLED Represents the disabled state of the reference
 * buffer.
 ******************************************************************************/
typedef enum {
	AD7779_REF_BUF_ENABLED,
	AD7779_REF_BUF_PRECHARGED,
	AD7779_REF_BUF_DISABLED,
} ad7779_ref_buf_op_mode;

/***************************************************************************//**
 * @brief The `ad7779_sar_mux` is an enumeration that defines various
 * multiplexer configurations for the AD7779 device, which is a high-
 * performance, low-power, 24-bit analog-to-digital converter. Each
 * enumerator represents a specific configuration of input and reference
 * voltages, supply voltages, and ground connections, often with
 * attenuation, used to control the SAR ADC's input selection and
 * reference settings. This enumeration is crucial for configuring the
 * ADC's input multiplexer to select the appropriate signal paths for
 * conversion.
 *
 * @param AD7779_AUXAINP_AUXAINN Represents the auxiliary analog input positive
 * and negative terminals.
 * @param AD7779_DVBE_AVSSX Represents the digital voltage buffer enable and
 * analog ground.
 * @param AD7779_REF1P_REF1N Represents the first reference positive and
 * negative terminals.
 * @param AD7779_REF2P_REF2N Represents the second reference positive and
 * negative terminals.
 * @param AD7779_REF_OUT_AVSSX Represents the reference output and analog
 * ground.
 * @param AD7779_VCM_AVSSX Represents the common mode voltage and analog ground.
 * @param AD7779_AREG1CAP_AVSSX_ATT Represents the first analog regulator
 * capacitor and analog ground with
 * attenuation.
 * @param AD7779_AREG2CAP_AVSSX_ATT Represents the second analog regulator
 * capacitor and analog ground with
 * attenuation.
 * @param AD7779_DREGCAP_DGND_ATT Represents the digital regulator capacitor and
 * digital ground with attenuation.
 * @param AD7779_AVDD1A_AVSSX_ATT Represents the first analog supply voltage and
 * analog ground with attenuation.
 * @param AD7779_AVDD1B_AVSSX_ATT Represents the second analog supply voltage
 * and analog ground with attenuation.
 * @param AD7779_AVDD2A_AVSSX_ATT Represents the third analog supply voltage and
 * analog ground with attenuation.
 * @param AD7779_AVDD2B_AVSSX_ATT Represents the fourth analog supply voltage
 * and analog ground with attenuation.
 * @param AD7779_IOVDD_DGND_ATT Represents the input/output supply voltage and
 * digital ground with attenuation.
 * @param AD7779_AVDD4_AVSSX Represents the fourth analog supply voltage and
 * analog ground.
 * @param AD7779_DGND_AVSS1A_ATT Represents the digital ground and first analog
 * ground with attenuation.
 * @param AD7779_DGND_AVSS1B_ATT Represents the digital ground and second analog
 * ground with attenuation.
 * @param AD7779_DGND_AVSSX_ATT Represents the digital ground and analog ground
 * with attenuation.
 * @param AD7779_AVDD4_AVSSX_ATT Represents the fourth analog supply voltage and
 * analog ground with attenuation.
 * @param AD7779_REF1P_AVSSX Represents the first reference positive terminal
 * and analog ground.
 * @param AD7779_REF2P_AVSSX Represents the second reference positive terminal
 * and analog ground.
 * @param AD7779_AVSSX_AVDD4_ATT Represents the analog ground and fourth analog
 * supply voltage with attenuation.
 ******************************************************************************/
typedef enum {
	AD7779_AUXAINP_AUXAINN,
	AD7779_DVBE_AVSSX,
	AD7779_REF1P_REF1N,
	AD7779_REF2P_REF2N,
	AD7779_REF_OUT_AVSSX,
	AD7779_VCM_AVSSX,
	AD7779_AREG1CAP_AVSSX_ATT,
	AD7779_AREG2CAP_AVSSX_ATT,
	AD7779_DREGCAP_DGND_ATT,
	AD7779_AVDD1A_AVSSX_ATT,
	AD7779_AVDD1B_AVSSX_ATT,
	AD7779_AVDD2A_AVSSX_ATT,
	AD7779_AVDD2B_AVSSX_ATT,
	AD7779_IOVDD_DGND_ATT,
	AD7779_AVDD4_AVSSX,
	AD7779_DGND_AVSS1A_ATT,
	AD7779_DGND_AVSS1B_ATT,
	AD7779_DGND_AVSSX_ATT,
	AD7779_AVDD4_AVSSX_ATT,
	AD7779_REF1P_AVSSX,
	AD7779_REF2P_AVSSX,
	AD7779_AVSSX_AVDD4_ATT,
} ad7779_sar_mux;

/***************************************************************************//**
 * @brief The `ad7779_dev` structure is a comprehensive representation of the
 * AD7779 device configuration and state, encapsulating SPI and GPIO
 * descriptors for communication and control, as well as various device
 * settings such as control mode, SPI operation mode, channel states,
 * gain settings, decimation rates, reference types, power modes, and
 * synchronization offsets. It also includes arrays for offset and gain
 * corrections, reference buffer operation modes, and cached register
 * values, providing a detailed framework for managing the AD7779's
 * functionality and interfacing with its hardware components.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_reset Pointer to a GPIO descriptor for the reset pin.
 * @param gpio_mode0 Pointer to a GPIO descriptor for mode0 pin.
 * @param gpio_mode1 Pointer to a GPIO descriptor for mode1 pin.
 * @param gpio_mode2 Pointer to a GPIO descriptor for mode2 pin.
 * @param gpio_mode3 Pointer to a GPIO descriptor for mode3 pin.
 * @param gpio_dclk0 Pointer to a GPIO descriptor for dclk0 pin.
 * @param gpio_dclk1 Pointer to a GPIO descriptor for dclk1 pin.
 * @param gpio_dclk2 Pointer to a GPIO descriptor for dclk2 pin.
 * @param gpio_sync_in Pointer to a GPIO descriptor for sync input pin.
 * @param gpio_convst_sar Pointer to a GPIO descriptor for SAR conversion start
 * pin.
 * @param ctrl_mode Control mode of the device, either pin or SPI controlled.
 * @param spi_crc_en State indicating if SPI CRC is enabled.
 * @param spi_op_mode SPI operation mode of the device.
 * @param state Array indicating the state (enabled/disabled) of each channel.
 * @param gain Array indicating the gain setting for each channel.
 * @param dec_rate_int Integer part of the decimation rate.
 * @param dec_rate_dec Decimal part of the decimation rate.
 * @param ref_type Type of reference used by the device.
 * @param pwr_mode Power mode of the device, either low power or high
 * resolution.
 * @param dclk_div Divider setting for the DCLK.
 * @param sync_offset Array of synchronization offsets for each channel.
 * @param offset_corr Array of offset correction values for each channel.
 * @param gain_corr Array of gain correction values for each channel.
 * @param ref_buf_op_mode Array indicating the operation mode of the reference
 * buffer for two pins.
 * @param sar_state State of the SAR ADC.
 * @param sar_mux Multiplexer setting for the SAR ADC.
 * @param sinc5_state State of the SINC5 filter, applicable only for AD7771.
 * @param read_from_cache Boolean indicating if the device reads from cache.
 * @param cached_reg_val Array of cached register values.
 ******************************************************************************/
typedef struct {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_reset;
	struct no_os_gpio_desc	*gpio_mode0;
	struct no_os_gpio_desc	*gpio_mode1;
	struct no_os_gpio_desc	*gpio_mode2;
	struct no_os_gpio_desc	*gpio_mode3;
	struct no_os_gpio_desc	*gpio_dclk0;
	struct no_os_gpio_desc	*gpio_dclk1;
	struct no_os_gpio_desc	*gpio_dclk2;
	struct no_os_gpio_desc	*gpio_sync_in;
	struct no_os_gpio_desc	*gpio_convst_sar;
	/* Device Settings */
	ad7779_ctrl_mode	ctrl_mode;
	ad7779_state		spi_crc_en;
	ad7779_spi_op_mode	spi_op_mode;
	ad7779_state		state[8];
	ad7779_gain		gain[8];
	uint16_t		dec_rate_int;
	uint16_t		dec_rate_dec;
	ad7779_ref_type		ref_type;
	ad7779_pwr_mode		pwr_mode;
	ad7779_dclk_div		dclk_div;
	uint8_t			sync_offset[8];
	uint32_t		offset_corr[8];
	uint32_t		gain_corr[8];
	ad7779_ref_buf_op_mode	ref_buf_op_mode[2];
	ad7779_state		sar_state;
	ad7779_sar_mux		sar_mux;
	ad7779_state		sinc5_state;	// Can be enabled only for AD7771
	bool				read_from_cache;
	uint8_t			cached_reg_val[AD7779_REG_SRC_UPDATE + 1];
} ad7779_dev;

/***************************************************************************//**
 * @brief The `ad7779_init_param` structure is used to initialize and configure
 * the AD7779 device, which is a high-performance, low-power, 8-channel,
 * 24-bit analog-to-digital converter (ADC). This structure includes
 * parameters for SPI and GPIO initialization, device control settings,
 * channel states, gain settings, decimation rates, reference types,
 * power modes, and correction values for offset and gain. It also
 * includes settings for synchronization offsets, reference buffer
 * operation modes, and the state of the SINC5 filter, providing
 * comprehensive configuration options for the AD7779 ADC.
 *
 * @param spi_init Initializes the SPI interface parameters.
 * @param gpio_reset Configures the GPIO for the reset pin.
 * @param gpio_mode0 Configures the GPIO for mode0 pin.
 * @param gpio_mode1 Configures the GPIO for mode1 pin.
 * @param gpio_mode2 Configures the GPIO for mode2 pin.
 * @param gpio_mode3 Configures the GPIO for mode3 pin.
 * @param gpio_dclk0 Configures the GPIO for dclk0 pin.
 * @param gpio_dclk1 Configures the GPIO for dclk1 pin.
 * @param gpio_dclk2 Configures the GPIO for dclk2 pin.
 * @param gpio_sync_in Configures the GPIO for sync input pin.
 * @param gpio_convst_sar Configures the GPIO for SAR conversion start pin.
 * @param ctrl_mode Specifies the control mode (pin or SPI control).
 * @param spi_crc_en Enables or disables SPI CRC checking.
 * @param state Array indicating the state (enabled/disabled) of each channel.
 * @param gain Array specifying the gain setting for each channel.
 * @param dec_rate_int Sets the integer part of the decimation rate.
 * @param dec_rate_dec Sets the decimal part of the decimation rate.
 * @param ref_type Specifies the reference type (internal or external).
 * @param pwr_mode Specifies the power mode (low power or high resolution).
 * @param dclk_div Sets the division factor for the DCLK.
 * @param sync_offset Array specifying the synchronization offset for each
 * channel.
 * @param offset_corr Array for offset correction values for each channel.
 * @param gain_corr Array for gain correction values for each channel.
 * @param ref_buf_op_mode Array specifying the reference buffer operation mode
 * for each pin.
 * @param sinc5_state Indicates the state of the SINC5 filter
 * (enabled/disabled).
 * @param read_from_cache Indicates whether to read configuration from cache.
 ******************************************************************************/
typedef struct {
	/* SPI */
	struct no_os_spi_init_param		spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_reset;
	struct no_os_gpio_init_param	gpio_mode0;
	struct no_os_gpio_init_param	gpio_mode1;
	struct no_os_gpio_init_param	gpio_mode2;
	struct no_os_gpio_init_param	gpio_mode3;
	struct no_os_gpio_init_param	gpio_dclk0;
	struct no_os_gpio_init_param	gpio_dclk1;
	struct no_os_gpio_init_param	gpio_dclk2;
	struct no_os_gpio_init_param	gpio_sync_in;
	struct no_os_gpio_init_param	gpio_convst_sar;
	/* Device Settings */
	ad7779_ctrl_mode	ctrl_mode;
	ad7779_state		spi_crc_en;
	ad7779_state		state[8];
	ad7779_gain		gain[8];
	uint16_t		dec_rate_int;
	uint16_t		dec_rate_dec;
	ad7779_ref_type		ref_type;
	ad7779_pwr_mode		pwr_mode;
	ad7779_dclk_div		dclk_div;
	uint8_t			sync_offset[8];
	uint32_t		offset_corr[8];
	uint32_t		gain_corr[8];
	ad7779_ref_buf_op_mode	ref_buf_op_mode[2];
	ad7779_state		sinc5_state;	// Can be enabled only for AD7771
	bool			read_from_cache;
} ad7779_init_param;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Compute CRC8 checksum. */
/***************************************************************************//**
 * @brief This function calculates the CRC8 checksum for a specified buffer of
 * data, which is useful for error-checking in data transmission or
 * storage. It should be used when a CRC8 checksum is required to verify
 * the integrity of data. The function processes each byte of the input
 * data and applies a polynomial to compute the checksum. It is important
 * to ensure that the data pointer is valid and that the data size is
 * correctly specified to avoid undefined behavior.
 *
 * @param data A pointer to the buffer of data for which the CRC8 checksum is to
 * be computed. Must not be null, and the caller retains ownership
 * of the data.
 * @param data_size The number of bytes in the data buffer to be processed. Must
 * be a non-negative integer. If zero, the function will return
 * a CRC of zero.
 * @return Returns the computed CRC8 checksum as an 8-bit unsigned integer.
 ******************************************************************************/
uint8_t ad7779_compute_crc8(uint8_t *data,
			    uint8_t data_size);
/* SPI read from device. */
/***************************************************************************//**
 * @brief This function is used to read a specific register from the AD7779
 * device using SPI communication. It requires a valid device structure
 * that has been initialized and configured for SPI communication. The
 * function reads the register specified by the address and stores the
 * result in the provided data pointer. If CRC checking is enabled in the
 * device configuration, the function will also verify the integrity of
 * the data using a CRC8 checksum. It is important to ensure that the
 * device is properly initialized and that the register address is valid
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad7779_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read. Must be a valid register
 * address for the AD7779 device.
 * @param reg_data A pointer to a uint8_t where the read register data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails or if a CRC error is detected when CRC is enabled.
 ******************************************************************************/
int32_t ad7779_spi_int_reg_read(ad7779_dev *dev,
				uint8_t reg_addr,
				uint8_t *reg_data);
/* SPI write to device. */
/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * on the AD7779 device using SPI communication. It should be called when
 * there is a need to configure or modify the settings of the AD7779 by
 * writing to its internal registers. The function requires a valid
 * device structure that has been properly initialized. It handles the
 * optional inclusion of a CRC byte if the device is configured to use
 * CRC for SPI communication. The function updates the cached value of
 * the register within the device structure to reflect the new data. It
 * returns an error code if the SPI write operation fails.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null and should be initialized before calling this
 * function.
 * @param reg_addr The address of the register to write to. It should be a valid
 * register address within the AD7779's addressable range.
 * @param reg_data The data byte to write to the specified register. It is a
 * single byte value to be written.
 * @return Returns an int32_t error code indicating the success or failure of
 * the SPI write operation.
 ******************************************************************************/
int32_t ad7779_spi_int_reg_write(ad7779_dev *dev,
				 uint8_t reg_addr,
				 uint8_t reg_data);
/* SPI read from device using a mask. */
/***************************************************************************//**
 * @brief This function reads a value from a specified register of the AD7779
 * device using SPI and applies a mask to the read value. It is useful
 * when only specific bits of a register are of interest. The function
 * must be called with a valid device structure and a non-null data
 * pointer. It returns an error code if the read operation fails,
 * allowing the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an initialized ad7779_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD7779 device.
 * @param mask A bitmask to apply to the read register value. Determines which
 * bits of the register are of interest.
 * @param data A pointer to a uint8_t where the masked register value will be
 * stored. Must not be null.
 * @return Returns an int32_t error code indicating the success or failure of
 * the SPI read operation.
 ******************************************************************************/
int32_t ad7779_spi_int_reg_read_mask(ad7779_dev *dev,
				     uint8_t reg_addr,
				     uint8_t mask,
				     uint8_t *data);
/* SPI write to device using a mask. */
/***************************************************************************//**
 * @brief This function is used to write data to a specific register of the
 * AD7779 device, applying a mask to modify only certain bits of the
 * register. It is useful when only a subset of bits in a register needs
 * to be updated without affecting the other bits. The function should be
 * called when the device is properly initialized and the register
 * address is valid. It modifies the cached register value in the device
 * structure and writes the updated value back to the device.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null, and the device must be initialized before calling
 * this function.
 * @param reg_addr The address of the register to be written to. It should be a
 * valid register address within the device's register map.
 * @param mask A bitmask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be affected by the data
 * parameter.
 * @param data The data to be written to the register, masked by the mask
 * parameter. Only the bits corresponding to the mask will be
 * written.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-zero return value indicates an error.
 ******************************************************************************/
int32_t ad7779_spi_int_reg_write_mask(ad7779_dev *dev,
				      uint8_t reg_addr,
				      uint8_t mask,
				      uint8_t data);
/* SPI SAR conversion code read. */
/***************************************************************************//**
 * @brief This function retrieves a SAR conversion code from the AD7779 device
 * using SPI communication. It is essential to call this function after
 * configuring the device and selecting the appropriate SAR multiplexer
 * channel for the next conversion. The function requires a valid device
 * structure and a pointer to store the conversion result. It handles CRC
 * checking if enabled, and returns an error code if the CRC check fails.
 * Ensure that the device is properly initialized and configured before
 * invoking this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null and should be properly initialized.
 * @param mux_next_conv An ad7779_sar_mux value indicating the multiplexer
 * channel for the next conversion. Must be a valid enum
 * value.
 * @param sar_code A pointer to a uint16_t where the SAR conversion code will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating success (0) or an error code. If CRC is
 * enabled and fails, returns -1.
 ******************************************************************************/
int32_t ad7779_spi_sar_read_code(ad7779_dev *dev,
				 ad7779_sar_mux mux_next_conv,
				 uint16_t *sar_code);
/* Set SPI operation mode. */
/***************************************************************************//**
 * @brief This function configures the SPI operation mode of the AD7779 device,
 * allowing the user to select between different conversion modes or
 * internal register access. It should be called when a change in the SPI
 * operation mode is required, such as switching between standard
 * conversion and diagnostic modes. The function updates the device's
 * configuration registers accordingly and modifies the device structure
 * to reflect the new mode. It is important to ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null, and the device must be initialized before use.
 * @param mode An ad7779_spi_op_mode enumeration value specifying the desired
 * SPI operation mode. Valid values are AD7779_INT_REG,
 * AD7779_SD_CONV, and AD7779_SAR_CONV. If an invalid mode is
 * provided, the function defaults to AD7779_INT_REG.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-zero return value indicates an error.
 ******************************************************************************/
int32_t ad7779_set_spi_op_mode(ad7779_dev *dev,
			       ad7779_spi_op_mode mode);
/* Get SPI operation mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current SPI operation mode of an
 * AD7779 device. It should be called when you need to determine the mode
 * in which the device is operating, which can be one of several
 * predefined modes. The function requires a valid device structure and a
 * pointer to store the retrieved mode. It handles both cached and non-
 * cached reads, depending on the device's configuration. Ensure that the
 * device has been properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null. The device should be initialized before use.
 * @param mode A pointer to an ad7779_spi_op_mode variable where the current SPI
 * operation mode will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad7779_get_spi_op_mode(ad7779_dev *dev,
			       ad7779_spi_op_mode *mode);
/* Set the state (enable, disable) of the channel. */
/***************************************************************************//**
 * @brief This function is used to enable or disable a specific channel on the
 * AD7779 device. It should be called when you need to change the
 * operational state of a channel, such as during initialization or when
 * dynamically managing channel activity. The function requires a valid
 * device structure and channel identifier, and it updates the device's
 * internal state to reflect the new channel state. Ensure that the
 * device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param ch An enumerated value of type ad7779_ch representing the channel to
 * be modified. Valid values are AD7779_CH0 to AD7779_CH7.
 * @param state An enumerated value of type ad7779_state indicating the desired
 * state of the channel. Valid values are AD7779_ENABLE or
 * AD7779_DISABLE.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad7779_set_state(ad7779_dev *dev,
			 ad7779_ch ch,
			 ad7779_state state);
/* Get the state (enable, disable) of the selected channel. */
/***************************************************************************//**
 * @brief This function is used to obtain the current state (enabled or
 * disabled) of a specific channel on the AD7779 device. It should be
 * called when you need to check the operational status of a channel,
 * which can be useful for diagnostics or configuration purposes. The
 * function requires a valid device structure and channel identifier. It
 * writes the state to the provided pointer. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param ch An ad7779_ch enum value specifying the channel whose state is to be
 * retrieved. Must be a valid channel identifier.
 * @param state A pointer to an ad7779_state variable where the channel state
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad7779_get_state(ad7779_dev *dev,
			 ad7779_ch ch,
			 ad7779_state *state);
/* Update the state of the MODEx pins according to the settings specified in
 * the device structure. */
/***************************************************************************//**
 * @brief This function updates the mode pins of the AD7779 device to reflect
 * the current configuration settings specified in the device structure.
 * It should be called when the device's gain, decimation rate, power
 * mode, or reference type settings are changed and need to be applied to
 * the hardware. The function checks for valid gain settings and
 * decimation rates before proceeding. If the configuration is invalid or
 * unsupported, the function returns an error. The function also pulses
 * the SYNC_IN pin to apply the new settings.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null. The structure should be properly initialized and
 * configured with the desired settings before calling this function.
 * @return Returns 0 on success, or -1 if the configuration is invalid or
 * unsupported.
 ******************************************************************************/
int32_t ad7779_do_update_mode_pins(ad7779_dev *dev);
/* Set the gain of the selected channel. */
/***************************************************************************//**
 * @brief This function configures the gain setting for a specified channel on
 * the AD7779 device. It should be called when the gain needs to be
 * adjusted for a particular channel, either in pin control mode or SPI
 * control mode. In pin control mode, the gain is set for a group of
 * channels, while in SPI control mode, it is set for the specified
 * channel. The function must be called with a valid device structure and
 * channel enumeration. It returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param ch An enumeration value of type ad7779_ch representing the channel to
 * configure. Valid values are AD7779_CH0 to AD7779_CH7.
 * @param gain An enumeration value of type ad7779_gain representing the desired
 * gain setting. Valid values are AD7779_GAIN_1, AD7779_GAIN_2,
 * AD7779_GAIN_4, and AD7779_GAIN_8.
 * @return Returns an int32_t status code. A value of 0 indicates success, while
 * a negative value indicates an error.
 ******************************************************************************/
int32_t ad7779_set_gain(ad7779_dev *dev,
			ad7779_ch ch,
			ad7779_gain gain);
/* Get the gain of the selected channel. */
/***************************************************************************//**
 * @brief This function is used to obtain the current gain setting of a
 * specified channel on the AD7779 device. It should be called when the
 * gain configuration of a channel needs to be verified or logged. The
 * function requires a valid device structure and channel identifier. If
 * the device is configured to read from cache, the gain is retrieved
 * from the cached values; otherwise, it reads directly from the device
 * registers. Ensure that the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null. The caller retains ownership.
 * @param ch An ad7779_ch enum value specifying the channel for which the gain
 * is to be retrieved. Must be a valid channel identifier.
 * @param gain A pointer to an ad7779_gain variable where the retrieved gain
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad7779_get_gain(ad7779_dev *dev,
			ad7779_ch ch,
			ad7779_gain *gain);
/* Set the decimation rate. */
/***************************************************************************//**
 * @brief This function configures the decimation rate of the AD7779 device,
 * which is used to control the data output rate. It must be called with
 * a valid device structure and appropriate integer and decimal values
 * for the decimation rate. In PIN control mode, only specific integer
 * values (128, 256, 512, 1024) are allowed, and any other value will
 * result in an error. In SPI control mode, the function writes the
 * decimation rate to the device registers. The function returns an error
 * code if the operation fails.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null.
 * @param int_val The integer part of the decimation rate. In PIN control mode,
 * must be one of 128, 256, 512, or 1024. In SPI control mode,
 * any 16-bit value is allowed.
 * @param dec_val The decimal part of the decimation rate, expressed as a 16-bit
 * value. In SPI control mode, it is scaled and written to the
 * device.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad7779_set_dec_rate(ad7779_dev *dev,
			    uint16_t int_val,
			    uint16_t dec_val);
/* Get the decimation rate. */
/***************************************************************************//**
 * @brief This function is used to obtain the current decimation rate settings
 * from an AD7779 device. It should be called when you need to know the
 * decimation rate for data processing or configuration purposes. The
 * function reads the decimation rate from the device registers unless
 * the device is configured to read from a cache, in which case it
 * retrieves the values from the cached settings. Ensure that the device
 * is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null. The caller retains ownership.
 * @param int_val A pointer to a uint16_t where the integer part of the
 * decimation rate will be stored. Must not be null.
 * @param dec_val A pointer to a uint16_t where the decimal part of the
 * decimation rate will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if reading from the
 * device registers fails.
 ******************************************************************************/
int32_t ad7779_get_dec_rate(ad7779_dev *dev,
			    uint16_t *int_val,
			    uint16_t *dec_val);
/* Set the power mode. */
/***************************************************************************//**
 * @brief This function configures the power mode of the AD7779 device to either
 * low power or high resolution. It should be called when there is a need
 * to change the power consumption or performance characteristics of the
 * device. The function updates the device's internal state to reflect
 * the new power mode. It is important to ensure that the device is
 * properly initialized before calling this function. The function
 * returns an error code if the operation fails, allowing the caller to
 * handle such cases appropriately.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null, and the device must be initialized
 * before use.
 * @param pwr_mode An ad7779_pwr_mode enumeration value indicating the desired
 * power mode. Valid values are AD7779_LOW_PWR for low power
 * mode and AD7779_HIGH_RES for high resolution mode.
 * @return Returns an int32_t error code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad7779_set_power_mode(ad7779_dev *dev,
			      ad7779_pwr_mode pwr_mode);
/* Get the power mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current power mode setting of an
 * AD7779 device. It should be called when the user needs to verify or
 * log the power mode of the device. The function reads the power mode
 * from the device's internal register unless the device is configured to
 * read from a cached value, in which case it retrieves the mode from the
 * cache. This function must be called with a valid device structure that
 * has been properly initialized.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. This
 * must be a valid, initialized device structure and must not be
 * null.
 * @param pwr_mode A pointer to an ad7779_pwr_mode variable where the current
 * power mode will be stored. This pointer must not be null.
 * @return Returns 0 on success, or a negative error code if the power mode
 * could not be retrieved from the device.
 ******************************************************************************/
int32_t ad7779_get_power_mode(ad7779_dev *dev,
			      ad7779_pwr_mode *pwr_mode);
/* Set the reference type. */
/***************************************************************************//**
 * @brief This function configures the reference type for the AD7779 device,
 * which can be either internal or external. It should be called when the
 * reference type needs to be changed, typically during device
 * initialization or configuration. The function updates the device's
 * internal state to reflect the new reference type. It is important to
 * ensure that the `dev` parameter is a valid, initialized device
 * structure before calling this function. The function returns an error
 * code if the operation fails, allowing the caller to handle such cases
 * appropriately.
 *
 * @param dev A pointer to an initialized `ad7779_dev` structure representing
 * the device. Must not be null.
 * @param ref_type An `ad7779_ref_type` value indicating the desired reference
 * type. Valid values are `AD7779_INT_REF` for internal
 * reference and `AD7779_EXT_REF` for external reference.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad7779_set_reference_type(ad7779_dev *dev,
				  ad7779_ref_type ref_type);
/* Get the reference type. */
/***************************************************************************//**
 * @brief This function is used to obtain the current reference type setting of
 * the AD7779 device. It should be called when you need to know whether
 * the device is using an internal or external reference, or other
 * reference configurations. The function reads from the device's
 * register unless the device is configured to read from a cached value,
 * in which case it retrieves the reference type from the cache. Ensure
 * that the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null. The caller retains ownership.
 * @param ref_type A pointer to an ad7779_ref_type variable where the reference
 * type will be stored. Must not be null. The function writes
 * the current reference type to this location.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The reference type is written to the location pointed to by
 * ref_type.
 ******************************************************************************/
int32_t ad7779_get_reference_type(ad7779_dev *dev,
				  ad7779_ref_type *ref_type);
/* Set the DCLK divider. */
/***************************************************************************//**
 * @brief This function configures the DCLK (data clock) divider for the AD7779
 * device, which determines the division factor of the data clock. It
 * should be called when the user needs to adjust the data clock rate
 * according to the application requirements. The function can operate in
 * two modes: pin control or SPI control, depending on the device's
 * control mode setting. It updates the device's internal state to
 * reflect the new divider setting. Ensure the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null. The caller retains ownership.
 * @param div An ad7779_dclk_div enumeration value specifying the desired DCLK
 * division factor. Valid values are defined by the ad7779_dclk_div
 * enum.
 * @return Returns an int32_t indicating success (0) or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t ad7779_set_dclk_div(ad7779_dev *dev,
			    ad7779_dclk_div div);
/* Get the DCLK divider. */
/***************************************************************************//**
 * @brief This function is used to obtain the current DCLK (Data Clock) divider
 * setting from an AD7779 device. It should be called when you need to
 * know the current DCLK divider configuration, which affects the data
 * output rate of the device. The function reads the setting either from
 * the device's internal register or from a cached value, depending on
 * the device's configuration. Ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null. The caller retains ownership.
 * @param div A pointer to an ad7779_dclk_div variable where the current DCLK
 * divider setting will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. On success, the DCLK divider setting is stored in the location
 * pointed to by 'div'.
 ******************************************************************************/
int32_t ad7779_get_dclk_div(ad7779_dev *dev,
			    ad7779_dclk_div *div);
/* Set the synchronization offset of the selected channel. */
/***************************************************************************//**
 * @brief This function configures the synchronization offset for a specific
 * channel on the AD7779 device. It must be called when the device is in
 * SPI control mode, as it is not supported in PIN control mode. The
 * function updates the device's internal register and the corresponding
 * channel's offset value. It is important to ensure that the device is
 * properly initialized and in the correct control mode before calling
 * this function to avoid errors.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null and the device must be initialized and
 * in SPI control mode.
 * @param ch The channel for which the synchronization offset is being set. Must
 * be a valid channel enumeration value (e.g., AD7779_CH0 to
 * AD7779_CH7).
 * @param sync_offset The synchronization offset value to set for the specified
 * channel. It is an 8-bit unsigned integer.
 * @return Returns an int32_t indicating success (0) or failure (-1 if in PIN
 * control mode, or other error codes from the SPI write operation).
 ******************************************************************************/
int32_t ad7779_set_sync_offset(ad7779_dev *dev,
			       ad7779_ch ch,
			       uint8_t sync_offset);
/* Get the synchronization offset of the selected channel. */
/***************************************************************************//**
 * @brief This function retrieves the synchronization offset for a specified
 * channel of the AD7779 device. It should be used when the device is in
 * SPI control mode, as it is not available in PIN control mode. The
 * function reads the offset either from the device's internal register
 * or from a cached value, depending on the device's configuration. It is
 * important to ensure that the device is properly initialized and
 * configured before calling this function. The function returns an error
 * code if the operation fails, such as when the device is in an
 * unsupported control mode.
 *
 * @param dev A pointer to an initialized ad7779_dev structure representing the
 * device. Must not be null.
 * @param ch The channel for which the synchronization offset is to be
 * retrieved. Must be a valid ad7779_ch enumeration value.
 * @param sync_offset A pointer to a uint8_t where the synchronization offset
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad7779_get_sync_offset(ad7779_dev *dev,
			       ad7779_ch ch,
			       uint8_t *sync_offset);
/* Set the offset correction of the selected channel. */
/***************************************************************************//**
 * @brief This function sets the offset correction value for a specified channel
 * on the AD7779 device. It should be used when the device is in SPI
 * control mode, as it is not available in PIN control mode. The function
 * updates the internal device registers with the provided offset value,
 * which is split into three bytes. It is important to ensure that the
 * device is properly initialized and in the correct control mode before
 * calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null and the device must be initialized and in SPI control
 * mode.
 * @param ch The channel for which the offset correction is to be set. It must
 * be a valid channel enumeration value (e.g., AD7779_CH0, AD7779_CH1,
 * etc.).
 * @param offset A 24-bit unsigned integer representing the offset correction
 * value to be set. The value is split into three bytes and
 * written to the device registers.
 * @return Returns an int32_t value indicating success (0) or failure (-1 if in
 * PIN control mode, or a non-zero error code if register writes fail).
 ******************************************************************************/
int32_t ad7779_set_offset_corr(ad7779_dev *dev,
			       ad7779_ch ch,
			       uint32_t offset);
/* Get the offset correction of the selected channel. */
/***************************************************************************//**
 * @brief This function retrieves the offset correction value for a specified
 * channel of the AD7779 device. It should be used when the device is in
 * SPI control mode, as it is not available in PIN control mode. The
 * function reads the offset correction value either from the device
 * registers or from a cached value, depending on the device's
 * configuration. It is important to ensure that the device is properly
 * initialized and configured before calling this function. The function
 * returns an error code if the operation fails, such as when the device
 * is in an unsupported control mode.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null. The device should be initialized and
 * configured for SPI control mode.
 * @param ch An ad7779_ch enumeration value specifying the channel for which the
 * offset correction is to be retrieved. Valid values are AD7779_CH0
 * to AD7779_CH7.
 * @param offset A pointer to a uint32_t variable where the offset correction
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad7779_get_offset_corr(ad7779_dev *dev,
			       ad7779_ch ch,
			       uint32_t *offset);
/* Set the gain correction of the selected channel. */
/***************************************************************************//**
 * @brief This function is used to set the gain correction value for a specific
 * channel on the AD7779 device. It should be called when the device is
 * in SPI control mode, as it is not supported in PIN control mode. The
 * function takes a 24-bit gain value, which is applied to the specified
 * channel. It is important to ensure that the device is properly
 * initialized and in the correct control mode before calling this
 * function. The function updates the internal gain correction register
 * and returns a status code indicating success or failure.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null and the device must be initialized and in SPI control
 * mode.
 * @param ch The channel for which the gain correction is to be set. Must be a
 * valid ad7779_ch enumeration value.
 * @param gain A 24-bit unsigned integer representing the gain correction value.
 * The value is masked to 24 bits before being applied.
 * @return Returns an int32_t status code: 0 for success, or a negative value
 * for failure, such as when the device is in PIN control mode.
 ******************************************************************************/
int32_t ad7779_set_gain_corr(ad7779_dev *dev,
			     ad7779_ch ch,
			     uint32_t gain);
/* Get the gain correction of the selected channel. */
/***************************************************************************//**
 * @brief This function retrieves the gain correction value for a specified
 * channel of the AD7779 device. It must be called when the device is in
 * SPI control mode, as it is not supported in PIN control mode. The
 * function reads the gain correction value either from the device's
 * internal registers or from a cached value, depending on the device's
 * configuration. It is important to ensure that the device is properly
 * initialized and configured before calling this function. The function
 * returns an error code if the operation fails, such as when the device
 * is in an unsupported control mode.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null and should be properly initialized.
 * @param ch The channel for which the gain correction is to be retrieved. Must
 * be a valid channel enumeration value (e.g., AD7779_CH0 to
 * AD7779_CH7).
 * @param gain A pointer to a uint32_t where the gain correction value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad7779_get_gain_corr(ad7779_dev *dev,
			     ad7779_ch ch,
			     uint32_t *gain);
/* Set the reference buffer operation mode of the selected pin. */
/***************************************************************************//**
 * @brief This function configures the operation mode of the reference buffer
 * for a specified pin on the AD7779 device. It must be called when the
 * device is in SPI control mode, as it is not supported in PIN control
 * mode. The function allows setting the buffer to enabled, precharged,
 * or disabled states. It updates the device's internal configuration
 * registers accordingly and modifies the device structure to reflect the
 * new mode. This function is essential for managing the reference
 * buffer's behavior, which can impact the device's performance and power
 * consumption.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null and the device must be initialized and in SPI control
 * mode.
 * @param refx_pin An enum value of type ad7779_refx_pin indicating which
 * reference pin (positive or negative) to configure. Valid
 * values are AD7779_REFX_P and AD7779_REFX_N.
 * @param mode An enum value of type ad7779_ref_buf_op_mode specifying the
 * desired operation mode for the reference buffer. Valid values are
 * AD7779_REF_BUF_ENABLED, AD7779_REF_BUF_PRECHARGED, and
 * AD7779_REF_BUF_DISABLED.
 * @return Returns 0 on success or a negative error code if the operation fails,
 * such as when the device is in PIN control mode.
 ******************************************************************************/
int32_t ad7779_set_ref_buf_op_mode(ad7779_dev *dev,
				   ad7779_refx_pin refx_pin,
				   ad7779_ref_buf_op_mode mode);
/* Get the reference buffer operation mode of the selected pin. */
/***************************************************************************//**
 * @brief This function is used to obtain the current operation mode of the
 * reference buffer for a specified pin on the AD7779 device. It should
 * be called when the device is in SPI control mode, as it is not
 * supported in PIN control mode. The function reads the configuration
 * from the device registers unless the device is set to read from cache,
 * in which case it retrieves the mode from cached values. It is
 * important to ensure that the device is properly initialized and
 * configured before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null and should be properly initialized.
 * @param refx_pin An ad7779_refx_pin enum value specifying the reference pin
 * (AD7779_REFX_P or AD7779_REFX_N) for which the operation mode
 * is to be retrieved.
 * @param mode A pointer to an ad7779_ref_buf_op_mode variable where the
 * retrieved operation mode will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the feature is not available in the current control mode.
 ******************************************************************************/
int32_t ad7779_get_ref_buf_op_mode(ad7779_dev *dev,
				   ad7779_refx_pin refx_pin,
				   ad7779_ref_buf_op_mode *mode);
/* Set the SAR ADC configuration. */
/***************************************************************************//**
 * @brief This function is used to set the state and multiplexer configuration
 * of the SAR ADC in the AD7779 device. It should be called when you need
 * to enable or disable the SAR ADC and select the appropriate input
 * channel for conversion. The function updates the device's internal
 * state to reflect the new configuration. It is important to ensure that
 * the device is properly initialized before calling this function to
 * avoid unexpected behavior.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null, and the device should be initialized
 * before use.
 * @param state An ad7779_state value indicating whether to enable or disable
 * the SAR ADC. Valid values are AD7779_ENABLE or AD7779_DISABLE.
 * @param mux An ad7779_sar_mux value specifying the input channel to be used by
 * the SAR ADC. Must be a valid multiplexer setting defined in the
 * ad7779_sar_mux enumeration.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-zero return value indicates an error.
 ******************************************************************************/
int32_t ad7779_set_sar_cfg(ad7779_dev *dev,
			   ad7779_state state,
			   ad7779_sar_mux mux);
/* Get the SAR ADC configuration. */
/***************************************************************************//**
 * @brief This function is used to obtain the current configuration of the SAR
 * ADC, including its state and multiplexer settings. It should be called
 * when you need to verify or utilize the current SAR ADC settings in
 * your application. The function requires a valid device structure and
 * will update the provided state and mux pointers with the current
 * configuration. Ensure that the device has been properly initialized
 * before calling this function. If the device is set to read from cache,
 * the cached values will be used; otherwise, the configuration will be
 * read directly from the device registers.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null and should be properly initialized.
 * @param state A pointer to an ad7779_state variable where the current SAR ADC
 * state will be stored. Must not be null.
 * @param mux A pointer to an ad7779_sar_mux variable where the current SAR ADC
 * multiplexer setting will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad7779_get_sar_cfg(ad7779_dev *dev,
			   ad7779_state *state,
			   ad7779_sar_mux *mux);
/* Do a single SAR conversion. */
/***************************************************************************//**
 * @brief This function performs a single SAR (Successive Approximation
 * Register) ADC conversion using the specified multiplexer setting and
 * stores the conversion result in the provided location. It is typically
 * used when a single, precise measurement is needed from the ADC. The
 * function requires a valid device structure and a multiplexer setting
 * to specify the input channel. The conversion result is stored in the
 * memory location pointed to by the sar_code parameter. The function
 * returns an error code if any operation fails during the conversion
 * process.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param mux An ad7779_sar_mux value specifying the input channel for the SAR
 * conversion. Must be a valid multiplexer setting.
 * @param sar_code A pointer to a uint16_t where the conversion result will be
 * stored. Must not be null. The caller provides the memory
 * location.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad7779_do_single_sar_conv(ad7779_dev *dev,
				  ad7779_sar_mux mux,
				  uint16_t *sar_code);
/* Do a SPI software reset. */
/***************************************************************************//**
 * @brief Use this function to reset the AD7779 device by sending a specific
 * sequence over the SPI interface. This function is typically called
 * when the device needs to be reinitialized or reset to a known state.
 * It requires a valid device structure that has been properly
 * initialized with SPI settings. The function returns an error code if
 * the SPI communication fails, which should be handled by the caller.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. This
 * must be a valid, non-null pointer initialized with the necessary
 * SPI configuration. The function will not perform any action if
 * this pointer is invalid.
 * @return Returns an int32_t value indicating the success or failure of the SPI
 * operation. A non-zero return value indicates an error during the SPI
 * communication.
 ******************************************************************************/
int32_t ad7779_do_spi_soft_reset(ad7779_dev *dev);
/* Set the state (enable, disable) of the SINC5 filter. */
/***************************************************************************//**
 * @brief Use this function to enable or disable the SINC5 filter on an AD7771
 * device. It must be called when the device is in SPI control mode, as
 * the function is not available in PIN control mode. This function
 * modifies the device's internal state to reflect the new filter state
 * and returns a status code indicating success or failure.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device. Must
 * not be null and the device must be initialized and in SPI control
 * mode.
 * @param state An ad7779_state value indicating the desired state of the SINC5
 * filter. Valid values are AD7779_ENABLE to enable the filter and
 * AD7779_DISABLE to disable it.
 * @return Returns an int32_t status code: 0 for success, or a negative value if
 * the operation fails (e.g., if the device is in PIN control mode).
 ******************************************************************************/
int32_t ad7771_set_sinc5_filter_state(ad7779_dev *dev,
				      ad7779_state state);
/* Get the state (enable, disable) of the SINC5 filter. */
/***************************************************************************//**
 * @brief This function is used to obtain the current state of the SINC5 filter
 * for the AD7779 device. It should be called when the control mode is
 * set to SPI control, as it is not available in PIN control mode. The
 * function reads the state from the device's register or cache,
 * depending on the configuration, and stores it in the provided state
 * variable. It is important to ensure that the device is properly
 * initialized and configured before calling this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device
 * instance. Must not be null and should be properly initialized.
 * @param state A pointer to an ad7779_state variable where the current state of
 * the SINC5 filter will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the control mode is not supported.
 ******************************************************************************/
int32_t ad7771_get_sinc5_filter_state(ad7779_dev *dev,
				      ad7779_state *state);
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the AD7779 device using the provided
 * initialization parameters. It sets up the SPI and GPIO interfaces,
 * configures device settings, and prepares the device for operation.
 * This function must be called before any other operations on the AD7779
 * device. It allocates memory for the device structure and configures
 * the device according to the specified parameters. If initialization
 * fails, it returns an error code and the device pointer is not valid.
 *
 * @param device A pointer to a pointer of type ad7779_dev. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type ad7779_init_param containing the
 * initialization parameters for the device. This includes SPI
 * and GPIO initialization parameters, control mode, and
 * various device settings. The caller retains ownership.
 * @return Returns 0 on successful initialization, or a negative error code if
 * initialization fails. On success, the device pointer is set to point
 * to the initialized device structure.
 ******************************************************************************/
int32_t ad7779_init(ad7779_dev **device,
		    ad7779_init_param init_param);

/* Free the resources allocated by ad7779_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD7779
 * device when it is no longer needed. This includes deallocating memory
 * and removing any associated SPI and GPIO descriptors. It should be
 * called after the device is no longer in use to prevent resource leaks.
 * Ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param dev A pointer to an ad7779_dev structure representing the device to be
 * removed. Must not be null. The function will handle invalid
 * pointers gracefully by returning an error code.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a non-zero
 * value indicates an error occurred during resource deallocation.
 ******************************************************************************/
int32_t ad7779_remove(ad7779_dev *dev);

#endif // AD7779_H_
