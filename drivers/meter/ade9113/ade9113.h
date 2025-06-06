/***************************************************************************//**
 *   @file   ade9113.h
 *   @brief  Header file of ADE9113 Driver.
 *   @author George Mois (george.mois@analog.com)
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
#ifndef __ADE9113_H__
#define __ADE9113_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_irq.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* SPI commands */
#define ADE9113_SPI_READ			NO_OS_BIT(7)

/* Long/Short operation bit, set for lon read/write */
#define ADE9113_OP_MODE_LONG			NO_OS_BIT(6)

/* ADE9113 CRC constants */
#define ADE9113_CRC8_POLY			0x07
#define ADE9113_CRC16_POLY			0x1021
#define ADE9113_CRC16_INIT_VAL			0xFFFF

/* ENABLE and DISABLE */
#define ENABLE					1u
#define DISABLE					0u

/* ADE9113 Register Map */
#define ADE9113_REG_SWRST			0x001
#define ADE9113_REG_CONFIG0			0x002
#define ADE9113_REG_CONFIG_FILT			0x003
#define ADE9113_REG_CONFIG_ISO_ACC		0x005
#define ADE9113_REG_CRC_RESULT_HI		0x006
#define ADE9113_REG_CRC_RESULT_LO		0x007
#define ADE9113_REG_EFUSE_REFRESH		0x008
#define ADE9113_REG_EMI_CONFIG			0x009
#define ADE9113_REG_EMI_HI_MASK			0x00A
#define ADE9113_REG_EMI_LO_MASK			0x00B
#define ADE9113_REG_EMI_HI_LIMIT		0x00C
#define ADE9113_REG_EMI_MID_LIMIT		0x00D
#define ADE9113_REG_EMI_LO_LIMIT		0x00E
#define ADE9113_REG_MASK0			0x00F
#define ADE9113_REG_MASK1			0x010
#define ADE9113_REG_MASK2			0x011
#define ADE9113_REG_CONFIG_ZX			0x012
#define ADE9113_REG_SCRATCH			0x013
#define ADE9113_REG_SYNC_SNAP			0x014
#define ADE9113_REG_COUNTER_HI			0x015
#define ADE9113_REG_COUNTER_LO			0x016
#define ADE9113_REG_SNAPSHOT_COUNT_HI		0x017
#define ADE9113_REG_SNAPSHOT_COUNT_LO		0x018
#define ADE9113_REG_WR_LOCK			0x01F
#define ADE9113_REG_STATUS0			0x020
#define ADE9113_REG_STATUS1			0x021
#define ADE9113_REG_STATUS2			0x022
#define ADE9113_REG_COM_FLT_TYPE		0x023
#define ADE9113_REG_COM_FLT_COUNT		0x024
#define ADE9113_REG_CONFIG_CRC			0x025
#define ADE9113_REG_I_WAV_HI			0x026
#define ADE9113_REG_I_WAV_MD			0x027
#define ADE9113_REG_I_WAV_LO 			0x028
#define ADE9113_REG_V1_WAV_HI			0x029
#define ADE9113_REG_V1_WAV_MD			0x02A
#define ADE9113_REG_V1_WAV_LO			0x02B
#define ADE9113_REG_V2_WAV_HI			0x02C
#define ADE9113_REG_V2_WAV_MD			0x02D
#define ADE9113_REG_V2_WAV_LO			0x02E
#define ADE9113_REG_UNIQUE_PART_ID_5		0x075
#define ADE9113_REG_UNIQUE_PART_ID_4		0x076
#define ADE9113_REG_UNIQUE_PART_ID_3		0x077
#define ADE9113_REG_UNIQUE_PART_ID_2		0x078
#define ADE9113_REG_UNIQUE_PART_ID_1		0x079
#define ADE9113_REG_UNIQUE_PART_ID_0		0x07A
#define ADE9113_REG_SILICON_REVISION		0x07D
#define ADE9113_REG_VERSION_PRODUCT		0x07E

/* ADE9113 SWRST command */
#define ADE9113_SWRST_CMD			0xD6

/* ADE9113_REG_CONFIG0 Bit Definition */
#define ADE9113_STREAM_DBG_MSK			NO_OS_GENMASK(3, 2)
#define ADE9113_CRC_EN_SPI_WRITE_MSK		NO_OS_BIT(1)
#define ADE9113_CLOUT_EN_MSK			NO_OS_BIT(0)

/* ADE9113_REG_CONFIG_FILT Bit Definition */
#define ADE9113_V2_ADC_INVERT_MSK		NO_OS_BIT(6)
#define ADE9113_V1_ADC_INVERT_MSK		NO_OS_BIT(5)
#define ADE9113_I_ADC_INVERT_MSK		NO_OS_BIT(4)
#define ADE9113_LPF_BW_MSK			NO_OS_BIT(3)
#define ADE9113_DATAPATH_CONFIG_MSK		NO_OS_GENMASK(2, 0)

/* ADE9113_REG_CONFIG_ISO_ACC Bit Definition */
#define ADE9113_ISO_WR_ACC_EN_MSK		NO_OS_BIT(0)

/* ADE9113_REG_EFUSE_REFRESH Bit Definition */
#define ADE9113_EFUSE_REFRESH_MSK		NO_OS_BIT(0)

/* ADE9113_REG_EMI_CONFIG Bit Definition */
#define ADE9113_EMI_CONFIG_MSK			NO_OS_GENMASK(2, 0)

/* ADE9113_REG_MASK0/ADE9113_REG_STATUS0 Bit Definition */
#define ADE9113_STATUS1X_MSK	 		NO_OS_BIT(7)
#define ADE9113_STATUS2X_MSK 			NO_OS_BIT(6)
#define ADE9113_COM_UP_MSK	 		NO_OS_BIT(4)
#define ADE9113_CRC_CHG_MSK 			NO_OS_BIT(3)
#define ADE9113_SPI_CRC_ERR_MSK 		NO_OS_BIT(1)
#define ADE9113_COMFLT_ERR_MSK			NO_OS_BIT(0)

/* ADE9113_REG_MASK1/ADE9113_REG_STATUS1 Bit Definition */
#define ADE9113_V2_WAV_OVRNG_MSK		NO_OS_BIT(3)
#define ADE9113_V1_WAV_OVRNG_MSK		NO_OS_BIT(2)
#define ADE9113_I_WAV_OVRNG_MSK			NO_OS_BIT(1)
#define ADE9113_ADC_SYNC_DONE_MSK		NO_OS_BIT(0)

/* ADE9113_REG_MASK2/ADE9113_REG_STATUS2 Bit Definition */
#define ADE9113_ISO_CLK_STBL_ERR_MSK		NO_OS_BIT(6)
#define ADE9113_ISO_PHY_CRC_ERR_MSK		NO_OS_BIT(5)
#define ADE9113_ISO_EFUSE_MEM_ERR_MSK		NO_OS_BIT(4)
#define ADE9113_ISO_DIG_MOD_V2_OVF_MSK		NO_OS_BIT(3)
#define ADE9113_ISO_DIG_MOD_V1_OVF_MSK		NO_OS_BIT(2)
#define ADE9113_ISO_DIG_MOD_I_OVF_MSK		NO_OS_BIT(1)
#define ADE9113_ISO_TEST_MMR_ERR_MSK		NO_OS_BIT(0)

/* ADE9113_REG_CONFIG_ZX Bit Definition */
#define ADE9113_ZX_EDGE_SEL_MSK			NO_OS_GENMASK(3, 2)
#define ADE9113_ZX_CHANNEL_CONFIG_MSK		NO_OS_GENMASK(1, 0)

/* ADE9113_REG_SYNC_SNAP Bit Definition */
#define ADE9113_PREP_BROADCAST_MSK		NO_OS_BIT(2)
#define ADE9113_ALIGN_MSK			NO_OS_BIT(1)
#define ADE9113_SNAPSHOT_MSK			NO_OS_BIT(0)

/* ADE9113_REG_COUNTER_HI Bit Definition */
#define ADE9113_COUNTER_HI_MSK			NO_OS_GENMASK(5, 0)

/* ADE9113_REG_COUNTER_LO Bit Definition */
#define ADE9113_COUNTER_LO_MSK			NO_OS_GENMASK(8, 0)

/* ADE9113_REG_SNAPSHOT_COUNT_HI Bit Definition */
#define ADE9113_SNAPSHOT_COUNT_HI_MSK		NO_OS_GENMASK(5, 0)

/* ADE9113_REG_SNAPSHOT_COUNT_LO Bit Definition */
#define ADE9113_SNAPSHOT_COUNT_LO_MSK		NO_OS_GENMASK(7, 0)

/* ADE9113_REG_STATUS0 Bit Definition */
#define ADE9113_RESET_DONE_MSK			NO_OS_BIT(5)
#define ADE9113_EFUSE_MEM_ERR_MSK		NO_OS_BIT(2)

/* ADE9113_REG_COM_FLT_TYPE Bit Definition */
#define ADE9113_ISO_STATUS_RD_ECC_ERR_MSK	NO_OS_BIT(2)
#define ADE9113_ISO_PHY_ERR_MSK			NO_OS_BIT(1)
#define ADE9113_ISO_ECC_ERR_MSK			NO_OS_BIT(1)

/* ADE9113_REG_CONFIG_CRC Bit Definition */
#define ADE9113_CRC_DONE_MSK			NO_OS_BIT(1)
#define ADE9113_CRC_FORCE_MSK			NO_OS_BIT(0)

/* ADE9113_REG_SILICON_REVISION Bit Definition */
#define ADE9113_NONISO_CHIP_REV_MSK		NO_OS_GENMASK(7, 4)
#define ADE9113_ISO_CHIP_REV_MSK		NO_OS_GENMASK(3, 0)

/* Configuration Register Write Lock */
#define ADE9113_LOCK_KEY			0XD4
#define ADE9113_UNLOCK_KEY			0X5E

/* Version Product */
#define ADE9113_3_CHANNEL_ADE9113		0U
#define ADE9113_2_CHANNEL_ADE9112		1U
#define ADE9113_NONISOLATED_ADE9103		3U

/* Nominal reference voltage */
#define ADE9113_VREF				(1249810)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ade9113_stream_debug_e` is an enumeration that defines different
 * modes of operation for the ADE9113 stream debugging. It includes modes
 * such as normal, static, increments at ADC conversion rate, and
 * functional mode, each represented by a unique enumerator. This
 * enumeration is used to configure and manage the stream debugging
 * behavior of the ADE9113 device.
 *
 * @param ADE9113_STREAM_NORMAL_MODE Represents the normal mode of operation for
 * the ADE9113 stream.
 * @param ADE9113_STREAM_STATIC_MODE Represents the static mode of operation for
 * the ADE9113 stream.
 * @param ADE9113_STREAM_INCREMENTS_MODE Indicates that data increments at the
 * ADC conversion rate.
 * @param ADE9113_STREAM_FUNCTIONAL_MODE Represents a mode that is the same as
 * the functional mode.
 ******************************************************************************/
enum ade9113_stream_debug_e {
	/* Normal Mode */
	ADE9113_STREAM_NORMAL_MODE = 0,
	/* Static Mode */
	ADE9113_STREAM_STATIC_MODE,
	/* Data Increments at ADC Conversion Rate */
	ADE9113_STREAM_INCREMENTS_MODE,
	/* Same as functional mode */
	ADE9113_STREAM_FUNCTIONAL_MODE
};

/***************************************************************************//**
 * @brief The `ade9113_datapath_config_e` is an enumeration that defines various
 * digital signal processing configurations for the ADE9113 device. Each
 * enumerator represents a specific combination of filter settings and
 * sampling rates, allowing for different levels of signal processing
 * complexity and performance. This configuration is crucial for
 * tailoring the device's operation to specific application requirements,
 * such as noise reduction and signal fidelity.
 *
 * @param ADE9113_SINC3_32_KHZ_SAMPLING Represents a configuration with Sinc3
 * filter and 32KHz sampling rate.
 * @param ADE9113_SINC3_LPF_EN_32_KHZ_SAMPLING Represents a configuration with
 * Sinc3 filter, Low-Pass Filter
 * enabled, and 32KHz sampling rate.
 * @param ADE9113_SINC3_COMP_EN_LPF_EN_32_KHZ_SAMPLING Represents a
 * configuration with Sinc3
 * filter, Compensation and
 * Low-Pass Filter enabled,
 * and 32KHz sampling rate.
 * @param ADE9113_SINC3_LPF_EN_8_KHZ_SAMPLING Represents a configuration with
 * Sinc3 filter, Low-Pass Filter
 * enabled, and 8KHz sampling rate.
 * @param ADE9113_SINC3_COMP_EN_LPF_EN_8_KHZ_SAMPLING Represents a configuration
 * with Sinc3 filter,
 * Compensation and Low-Pass
 * Filter enabled, and 8KHz
 * sampling rate.
 * @param ADE9113_SINC3_LPF_EN_4_KHZ_SAMPLING Represents a configuration with
 * Sinc3 filter, Low-Pass Filter
 * enabled, and 4KHz sampling rate.
 * @param ADE9113_SINC3_LPF_EN_2_KHZ_SAMPLING Represents a configuration with
 * Sinc3 filter, Low-Pass Filter
 * enabled, and 2KHz sampling rate.
 * @param ADE9113_SINC3_LPF_EN_1_KHZ_SAMPLING Represents a configuration with
 * Sinc3 filter, Low-Pass Filter
 * enabled, and 1KHz sampling rate.
 ******************************************************************************/
enum ade9113_datapath_config_e {
	/* Sinc3, 32KHz Sampling */
	ADE9113_SINC3_32_KHZ_SAMPLING = 0,
	/* Sinc3, Low-Pass Filter (LPF) Enabled, 32kHz Sampling */
	ADE9113_SINC3_LPF_EN_32_KHZ_SAMPLING,
	/* Sinc3, Compensation Enabled, LPF Enabled, 32KHz Sampling */
	ADE9113_SINC3_COMP_EN_LPF_EN_32_KHZ_SAMPLING,
	/* Sinc3, LPF Enabled, 8KHz Sampling */
	ADE9113_SINC3_LPF_EN_8_KHZ_SAMPLING,
	/* Sinc3, Compensation Enabled, LPF Enabled, 8KHz Sampling */
	ADE9113_SINC3_COMP_EN_LPF_EN_8_KHZ_SAMPLING,
	/* Sinc3, LPF Enabled, 4kHz Sampling. */
	ADE9113_SINC3_LPF_EN_4_KHZ_SAMPLING,
	/* Sinc3, LPF Enabled, 2kHz Sampling. */
	ADE9113_SINC3_LPF_EN_2_KHZ_SAMPLING,
	/* Sinc3, LPF Enabled, 1kHz Sampling. */
	ADE9113_SINC3_LPF_EN_1_KHZ_SAMPLING
};

/***************************************************************************//**
 * @brief The `ade9113_emi_config_e` is an enumeration that defines the
 * different configurations for Electromagnetic Interference (EMI)
 * frequency hopping in the ADE9113 device. It provides options for
 * setting the EMI frequency to various patterns such as sawtooth rising,
 * sawtooth falling, ramp, and random hopping. This allows for flexible
 * EMI management in applications where the ADE9113 is used, enabling the
 * device to adapt to different EMI conditions and requirements.
 *
 * @param ADE9113_SAWTOOTH_FREQUENCY_RISING Represents a configuration where the
 * EMI frequency is set to a sawtooth
 * pattern with a rising frequency.
 * @param ADE9113_SAWTOOTH_FREQUENCY_FALLING Represents a configuration where
 * the EMI frequency is set to a
 * sawtooth pattern with a falling
 * frequency.
 * @param ADE9113_RAMP Represents a configuration where the EMI frequency is set
 * to a ramp pattern.
 * @param ADE9113_RANDOM_HOPPING_FREQUENCY Represents a configuration where the
 * EMI frequency is set to randomly hop
 * between frequencies.
 ******************************************************************************/
enum ade9113_emi_config_e {
	/* Sawtooth Frequency Rising */
	ADE9113_SAWTOOTH_FREQUENCY_RISING = 0,
	/* Sawtooth Frequency Falling */
	ADE9113_SAWTOOTH_FREQUENCY_FALLING,
	/* Ramp */
	ADE9113_RAMP,
	/* Random Hopping Frequency */
	ADE9113_RANDOM_HOPPING_FREQUENCY
};

/***************************************************************************//**
 * @brief The `ade9113_zx_edge_sel_e` enumeration defines the different modes
 * for detecting zero crossings in the ADE9113 device. It allows the
 * selection of how the zero crossing (ZX) pin behaves, either reflecting
 * the sign of the input signal or detecting zero crossings based on the
 * slope of the signal. This configuration is crucial for applications
 * that require precise detection of zero crossings for signal processing
 * or control purposes.
 *
 * @param ADE9113_ZX_INPUT_SIGNAL_SIGN ZX Pin reflects the sign of the input
 * signal.
 * @param ADE9113_ZX_DETECT_POSITIVE_SLOPE Detects zero crossings with a
 * positive slope.
 * @param ADE9113_ZX_DETECT_NEGATIVE_SLOPE Detects zero crossings with a
 * negative slope.
 * @param ADE9113_ZX_DETECT_BOTH_SLOPES Detects zero crossings with both
 * positive and negative slopes.
 ******************************************************************************/
enum ade9113_zx_edge_sel_e {
	/* ZX Pin Reflects the Sign of the Input Signal */
	ADE9113_ZX_INPUT_SIGNAL_SIGN = 0,
	/* Detect Zero Crossings with Positive Slope */
	ADE9113_ZX_DETECT_POSITIVE_SLOPE,
	/* Detect Zero Crossings with Negative Slope */
	ADE9113_ZX_DETECT_NEGATIVE_SLOPE,
	/* Detect Zero Crossings with Positive or Negative Slopes */
	ADE9113_ZX_DETECT_BOTH_SLOPES
};

/***************************************************************************//**
 * @brief The `ade9113_zx_channel_cfg_e` is an enumeration that defines the
 * configuration options for selecting the channel from which the Zero
 * Crossing (ZX) function is output on the ZX pin of the ADE9113 device.
 * It provides options to disable the ZX output or select the I, V1, or
 * V2 channels for the ZX function, allowing for flexible configuration
 * of the device's zero crossing detection capabilities.
 *
 * @param ADE9113_ZX_DISABLE Disables the Zero Crossing Output.
 * @param ADE9113_ZX_I_SEL Outputs the Zero Crossing Function from the I Channel
 * on the ZX Pin.
 * @param ADE9113_ZX_V1_SEL Outputs the Zero Crossing Function from the V1
 * Channel on the ZX Pin.
 * @param ADE9113_ZX_V2_SEL Outputs the Zero Crossing Function from the V2
 * Channel on the ZX Pin.
 ******************************************************************************/
enum ade9113_zx_channel_cfg_e {
	/* Disable Zero Crossing Output */
	ADE9113_ZX_DISABLE = 0,
	/* Output Zero Crossing Function from the I Channel on ZX Pin */
	ADE9113_ZX_I_SEL,
	/* Output Zero Crossing Function from the V1 Channel on ZX Pin */
	ADE9113_ZX_V1_SEL,
	/* Output Zero Crossing Function from the V2 Channel on ZX Pin */
	ADE9113_ZX_V2_SEL
};

/***************************************************************************//**
 * @brief The `ade9113_operation_e` is an enumeration that defines the operation
 * modes for the ADE9113 device, specifically distinguishing between long
 * and short read/write operations. This enum is used to specify the type
 * of operation to be performed when interacting with the device,
 * allowing for flexibility in communication protocols depending on the
 * operation length required.
 *
 * @param ADE9113_L_OP Represents long read/write operations.
 * @param ADE9113_S_OP Represents short read/write operations.
 ******************************************************************************/
enum ade9113_operation_e {
	/* Long read/write operations */
	ADE9113_L_OP = 0,
	/* Short read/write operations */
	ADE9113_S_OP
};

/***************************************************************************//**
 * @brief The `ade9113_wav_e` enumeration defines constants for selecting
 * waveform data channels in the ADE9113 device, specifically for current
 * and two voltage channels. This enumeration is used to specify which
 * waveform data to access or manipulate, facilitating operations on the
 * device's waveform data registers.
 *
 * @param ADE9113_I_WAV Represents the waveform data for the current channel.
 * @param ADE9113_V1_WAV Represents the waveform data for the first voltage
 * channel.
 * @param ADE9113_V2_WAV Represents the waveform data for the second voltage
 * channel.
 ******************************************************************************/
enum ade9113_wav_e {
	/* I_WAV */
	ADE9113_I_WAV = 0,
	/* V1_WAV */
	ADE9113_V1_WAV,
	/* V1_WAV */
	ADE9113_V2_WAV
};


/***************************************************************************//**
 * @brief The `ade9113_init_param` structure is used to define the
 * initialization parameters for the ADE9113 device. It includes
 * descriptors for SPI communication, GPIO signals for RDY and RESET, and
 * an IRQ controller for handling interrupts. Additionally, it allows for
 * an external callback function to manage interrupts related to the RDY
 * signal. The structure also specifies the number of devices in a daisy-
 * chain setup, facilitating the configuration of multiple devices in a
 * series.
 *
 * @param spi_init Pointer to the SPI initialization parameters for device
 * communication.
 * @param gpio_rdy Pointer to the GPIO initialization parameters for the RDY
 * signal, indicating ADC data availability.
 * @param gpio_reset Pointer to the GPIO initialization parameters for the RESET
 * signal, used for hardware reset of the device.
 * @param irq_ctrl Pointer to the IRQ controller descriptor for handling
 * interrupt routines related to the GPIO RDY signal.
 * @param drdy_callback Function pointer for an external callback to handle
 * interrupt routines for the GPIO RDY signal, or NULL if
 * the driver-defined callback is used.
 * @param no_devs Number of devices in a daisy-chain configuration, with 1
 * indicating no daisy-chain.
 ******************************************************************************/
struct ade9113_init_param {
	/* Device communication descriptor */
	struct no_os_spi_init_param 	*spi_init;
	/** GPIO RDY descriptor used to signal when ADC data is available */
	struct no_os_gpio_init_param	*gpio_rdy;
	/** GPIO RESET descriptor used to reset device (HW reset) */
	struct no_os_gpio_init_param  	*gpio_reset;
	/** IRQ device descriptor used to handle interrupt routine for GPIO RDY */
	struct no_os_irq_ctrl_desc 	*irq_ctrl;
	/** External callback used to handle interrupt routine for GPIO RDY */
	/** Set to NULL if callback defined in driver used */
	void (*drdy_callback)(void *context);
	/** number of devices in daisy-chain, if 1, then no daisy-chain */
	uint8_t no_devs;
};

/***************************************************************************//**
 * @brief The `ade9113_dev` structure is a comprehensive representation of an
 * ADE9113 device, encapsulating all necessary components for its
 * operation. It includes descriptors for SPI communication, GPIO for
 * readiness and reset, and interrupt handling, as well as pointers to
 * waveform data for current and voltage channels. The structure also
 * manages device-specific settings such as product version and CRC
 * enablement, and supports configurations for daisy-chaining multiple
 * devices.
 *
 * @param spi_desc Pointer to the SPI communication descriptor for the device.
 * @param ver_product Holds the version of the product.
 * @param crc_en Indicates if CRC is enabled for the device.
 * @param i_wav Pointer to the I_WAV waveform data.
 * @param v1_wav Pointer to the V1_WAV waveform data.
 * @param v2_wav Pointer to the V2_WAV waveform data.
 * @param gpio_rdy GPIO descriptor for signaling when ADC data is ready.
 * @param gpio_reset GPIO descriptor for hardware resetting the device.
 * @param irq_ctrl Descriptor for handling interrupt routines for GPIO RDY.
 * @param irq_cb Callback descriptor for handling GPIO RDY interrupts.
 * @param no_devs Number of devices in a daisy-chain configuration.
 ******************************************************************************/
struct ade9113_dev {
	/* Device communication descriptor */
	struct no_os_spi_desc		*spi_desc;
	/* Version product */
	uint8_t 			ver_product;
	/* CRC setting */
	uint8_t				crc_en;
	/* I_WAV */
	int32_t				*i_wav;
	/* V1_WAV */
	int32_t				*v1_wav;
	/* V2_WAV */
	int32_t				*v2_wav;
	/** GPIO RDY descriptor used to signal when ADC data is available */
	struct no_os_gpio_desc  	*gpio_rdy;
	/** GPIO RESET descriptor used to reset device (HW reset) */
	struct no_os_gpio_desc  	*gpio_reset;
	/** IRQ device descriptor used to handle interrupt routine for GPIO RDY */
	struct no_os_irq_ctrl_desc 	*irq_ctrl;
	/** IRQ callback used to handle interrupt routine for GPIO RDY */
	struct no_os_callback_desc	irq_cb;
	/** number of devices in daisy-chain, if 1, then no daisy-chain */
	uint8_t no_devs;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Read device register. */
/***************************************************************************//**
 * @brief This function is used to read a specified register from the ADE9113
 * device. It must be called after the device has been properly
 * initialized. The function supports both long and short operation
 * modes, which determine the number of bytes read and the structure of
 * the command sent to the device. If the operation mode is set to long,
 * additional data will be read. The function also performs CRC checks if
 * enabled, and it is important to handle potential error codes that
 * indicate communication issues or invalid responses.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register to read from. Valid values are
 * defined in the ADE9113 register map.
 * @param reg_data Pointer to a variable where the read data will be stored.
 * Must not be null.
 * @param op_mode Operation mode for the read operation, which can be either
 * `ADE9113_L_OP` for long operations or `ADE9113_S_OP` for short
 * operations.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int ade9113_read(struct ade9113_dev *dev, uint8_t reg_addr,
		 uint8_t *reg_data, enum ade9113_operation_e op_mode);

/* Read device register in a daisy-chain setup. */
/***************************************************************************//**
 * @brief This function is used to read a specific register from multiple
 * devices connected in a daisy-chain configuration. It must be called
 * after the device has been properly initialized and configured. The
 * function expects the number of devices in the daisy-chain to be
 * between 2 and 4, inclusive. If the number of devices is outside this
 * range, the function will return an error. The register address to read
 * from is specified by the `reg_addr` parameter, and the data read from
 * the register will be stored in the `reg_data` array. The function
 * handles CRC checks if enabled, and will return an error if the CRC
 * validation fails or if there is a communication issue.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register to read from. Valid values are
 * defined in the ADE9113 register map.
 * @param reg_data Pointer to an array where the read data will be stored. The
 * caller retains ownership and must ensure it is large enough
 * to hold the data for all devices.
 * @return Returns 0 on success, or a negative error code indicating the type of
 * failure.
 ******************************************************************************/
int ade9113_read_dc(struct ade9113_dev *dev, uint8_t reg_addr,
		    uint8_t *reg_data);

/* Write device register. */
/***************************************************************************//**
 * @brief This function is used to write a value to a specific register of the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. The `reg_addr` parameter specifies the register address
 * to which the data will be written, while `reg_data` contains the data
 * to be written. The `op_mode` parameter determines whether the
 * operation is a long or short write. If an invalid `dev` pointer is
 * provided, or if the `op_mode` is not recognized, the function will
 * handle these cases appropriately, ensuring that the device remains in
 * a safe state.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register to write to. It should be a valid
 * register address defined in the ADE9113 register map.
 * @param reg_data The data to be written to the specified register. It should
 * be a valid 8-bit value.
 * @param op_mode The operation mode, which can be either `ADE9113_L_OP` for
 * long operations or `ADE9113_S_OP` for short operations. Must
 * be a valid value from the `ade9113_operation_e` enumeration.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_write(struct ade9113_dev *dev, uint8_t reg_addr,
		  uint8_t reg_data, enum ade9113_operation_e op_mode);

/* Write device register in a daisy-chain setup. */
/***************************************************************************//**
 * @brief This function is used to write data to a specified register of the
 * ADE9113 device when multiple devices are connected in a daisy-chain
 * configuration. It is essential to call this function after
 * initializing the device and ensuring that the `dev` structure is
 * properly configured. The `reg_data` parameter should contain the data
 * to be written to the register, and the function will handle the
 * communication with the device over SPI. If the number of devices in
 * the daisy-chain is greater than one, the function will write the data
 * for each device accordingly. It is important to ensure that the
 * `reg_data` array has enough elements to accommodate the number of
 * devices specified in `dev->no_devs`.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register to which data will be written.
 * Valid register addresses are defined in the header file.
 * @param reg_data Pointer to an array of `uint8_t` containing the data to be
 * written. The array must have at least `dev->no_devs`
 * elements. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_write_dc(struct ade9113_dev *dev, uint8_t reg_addr,
		     uint8_t *reg_data);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function is used to initialize the ADE9113 device with the
 * specified parameters. It must be called before any other operations on
 * the device can be performed. The `init_param` structure must be
 * properly configured, including the SPI and GPIO settings. If the
 * number of devices in a daisy-chain is set to zero, it defaults to one.
 * The function will return an error if the number of devices exceeds
 * four or if the IRQ control is not provided for a single device setup.
 * The function allocates necessary resources and sets up the device,
 * including interrupt handling and device reset. If any allocation or
 * initialization step fails, the function will clean up and return an
 * appropriate error code.
 *
 * @param device A pointer to a pointer of `struct ade9113_dev`. The caller must
 * provide a valid pointer that will be set to the initialized
 * device structure. This pointer will be allocated and managed by
 * the function.
 * @param init_param A structure of type `struct ade9113_init_param` that
 * contains the initialization parameters for the device. This
 * structure must be properly populated by the caller before
 * passing it to the function. It includes pointers for SPI
 * and GPIO configurations, an IRQ control descriptor, an
 * optional callback for data ready interrupts, and the number
 * of devices in a daisy-chain. The `no_devs` field must be
 * between 1 and 4.
 * @return Returns 0 on successful initialization. On failure, it returns a
 * negative error code indicating the type of error encountered, such as
 * -EINVAL for invalid parameters or -ENOMEM for memory allocation
 * failures.
 ******************************************************************************/
int ade9113_init(struct ade9113_dev **device,
		 struct ade9113_init_param init_param);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief This function should be called to properly deinitialize and free
 * resources associated with an `ade9113_dev` instance. It is essential
 * to invoke this function after the device is no longer needed,
 * typically after a successful initialization and usage of the device.
 * The function handles the removal of SPI and GPIO resources,
 * unregisters any interrupt callbacks, and frees the device structure
 * itself. Ensure that the `dev` parameter is valid and points to an
 * initialized device structure; passing a null or uninitialized pointer
 * may lead to undefined behavior.
 *
 * @param dev Pointer to an `ade9113_dev` structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid, initialized device structure. Passing an invalid pointer
 * may result in undefined behavior.
 * @return Returns 0 on successful removal of the device and associated
 * resources. If an error occurs during the removal process, a negative
 * error code is returned, indicating the specific failure.
 ******************************************************************************/
int ade9113_remove(struct ade9113_dev *dev);

/* Reset the device using SW reset. */
/***************************************************************************//**
 * @brief This function is used to perform a software reset on the ADE9113
 * device. It should be called when the device needs to be reinitialized,
 * typically after a configuration change or to recover from an error
 * state. The function will enable the CRC feature and wait for a brief
 * period to allow the device to initialize before reading the version
 * product. It is important to ensure that the `dev` parameter points to
 * a valid `struct ade9113_dev` instance that has been properly
 * initialized before calling this function.
 *
 * @param dev A pointer to a `struct ade9113_dev` that represents the device to
 * be reset. This pointer must not be null and should point to a
 * valid device structure that has been initialized.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ade9113_sw_reset(struct ade9113_dev *dev);

/* Reset the device using HW reset. */
/***************************************************************************//**
 * @brief This function is used to perform a hardware reset on the ADE9113
 * device, which is necessary to reinitialize the device and clear any
 * previous states. It should be called when the device needs to be
 * reset, such as during initialization or recovery from an error state.
 * The function will set the reset GPIO pin low, wait for a brief period,
 * and then set it high again, enabling the device to start its
 * initialization process. It is important to ensure that the `dev`
 * parameter is properly initialized before calling this function.
 *
 * @param dev A pointer to a `struct ade9113_dev` that represents the device to
 * be reset. This pointer must not be null and should point to a
 * valid device structure that has been initialized.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ade9113_hw_reset(struct ade9113_dev *dev);

/* Convert a 24-bit raw sample to millivolts. */
/***************************************************************************//**
 * @brief This function is used to convert a raw sample from the ADE9113 device
 * into a millivolt representation. It should be called after the device
 * has been properly initialized and configured. The function takes a
 * device structure, a device number, and a channel identifier as inputs.
 * It is important to ensure that the channel identifier is within the
 * valid range; otherwise, an error will be returned. The result is
 * stored in the provided pointer, which must not be null.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param dev_no The device number to access. Valid values depend on the number
 * of devices initialized.
 * @param ch The channel identifier, which must be one of the values from the
 * `ade9113_wav_e` enumeration (ADE9113_I_WAV, ADE9113_V1_WAV,
 * ADE9113_V2_WAV). Values outside this range will result in an error.
 * @param mv_val Pointer to an `int32_t` where the converted millivolt value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * found or if the channel identifier is invalid.
 ******************************************************************************/
int ade9113_convert_to_millivolts(struct ade9113_dev *dev,
				  uint8_t dev_no, enum ade9113_wav_e ch, int32_t *mv_val);

/* Get STREAM_DBG mode. */
/***************************************************************************//**
 * @brief This function retrieves the current stream debug mode of the ADE9113
 * device. It should be called after the device has been initialized and
 * is ready for operation. The caller must ensure that the `stream_dbg`
 * pointer is valid and not null, as passing a null pointer will result
 * in an error. The function reads the configuration register to
 * determine the current mode and updates the value pointed to by
 * `stream_dbg` accordingly. The possible modes include normal mode,
 * static mode, increments mode, and functional mode.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param stream_dbg Pointer to an `ade9113_stream_debug_e` enum where the
 * current debug mode will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs,
 * such as if `stream_dbg` is null or if reading the device register
 * fails.
 ******************************************************************************/
int ade9113_get_stream_dbg_mode(struct ade9113_dev *dev,
				enum ade9113_stream_debug_e *stream_dbg);

/* Set STREAM_DBG mode. */
/***************************************************************************//**
 * @brief This function configures the stream debug mode of the ADE9113 device,
 * which affects how the device processes and outputs data. It should be
 * called after the device has been initialized and before starting any
 * data acquisition. The function accepts a mode parameter that specifies
 * the desired debug mode, and it will return an error if the provided
 * mode is invalid. It is important to ensure that the mode is within the
 * defined range to avoid unexpected behavior.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param stream_dbg An enumeration value of type `ade9113_stream_debug_e` that
 * specifies the desired stream debug mode. Valid values are
 * from `ADE9113_STREAM_NORMAL_MODE` to
 * `ADE9113_STREAM_FUNCTIONAL_MODE`. If the value exceeds
 * `ADE9113_STREAM_FUNCTIONAL_MODE`, the function will return
 * an error.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid.
 ******************************************************************************/
int ade9113_set_stream_dbg_mode(struct ade9113_dev *dev,
				enum ade9113_stream_debug_e stream_dbg);

/*  Get CRC enable on SPI write setting. */
/***************************************************************************//**
 * @brief This function is used to obtain the current state of the CRC enable
 * setting for SPI write operations. It should be called after the device
 * has been initialized. The caller must provide a valid pointer to a
 * `uint8_t` variable where the CRC enable state will be stored. If the
 * provided pointer is null, the function will return an error. The
 * function also updates the internal state of the device based on the
 * current configuration register value.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param crc_en_state A pointer to a `uint8_t` where the CRC enable state will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9113_get_crc_en_state(struct ade9113_dev *dev,
			     uint8_t *crc_en_state);

/* Set CRC enable on SPI write setting. */
/***************************************************************************//**
 * @brief This function is used to configure the CRC (Cyclic Redundancy Check)
 * enable state for SPI write operations on the ADE9113 device. It should
 * be called after the device has been initialized. The `crc_en_state`
 * parameter determines whether CRC is enabled (1) or disabled (0). If an
 * invalid value is provided (greater than 1), the function will return
 * an error. Upon successful execution, the device's internal state will
 * be updated to reflect the new CRC setting.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param crc_en_state An 8-bit value indicating the CRC enable state. Valid
 * values are 0 (disable) and 1 (enable). If the value is
 * greater than 1, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or if an error occurs during the operation.
 ******************************************************************************/
int ade9113_set_crc_en_state(struct ade9113_dev *dev,
			     uint8_t crc_en_state);

/* Lock device. */
/***************************************************************************//**
 * @brief This function is used to lock the device, preventing any further write
 * operations until it is unlocked. It should be called when the device
 * is ready for configuration changes that require write protection. The
 * function must be called with a valid device structure; if the provided
 * device pointer is null, it will return an error code. It is important
 * to ensure that the device is properly initialized before calling this
 * function.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code (-ENODEV). The caller retains ownership of the device
 * structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_wr_lock(struct ade9113_dev *dev);

/* Unlock device. */
/***************************************************************************//**
 * @brief This function should be called when write access to the device is
 * required. It is essential to ensure that the `dev` parameter is
 * properly initialized and not null before invoking this function. If
 * the `dev` parameter is null, the function will return an error code
 * indicating that the device is not available. Upon successful
 * execution, the device will be unlocked, allowing subsequent write
 * operations.
 *
 * @param dev A pointer to a `struct ade9113_dev` representing the device. This
 * parameter must not be null and should point to a valid device
 * structure that has been initialized. If this parameter is null,
 * the function will return an error code -ENODEV.
 * @return Returns 0 on success, indicating that the device has been
 * successfully unlocked. If the input parameter is invalid (null), it
 * returns -ENODEV.
 ******************************************************************************/
int ade9113_wr_unlock(struct ade9113_dev *dev);

/*  Write value in the scratchpad register. */
/***************************************************************************//**
 * @brief This function is used to write a specified value to the scratchpad
 * register of the ADE9113 device. It should be called after the device
 * has been properly initialized. If the `dev` parameter is null, the
 * function will return an error code indicating that the device is not
 * available. The value written to the scratchpad can be used for
 * temporary storage or configuration purposes.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param val The value to be written to the scratchpad register. This should be
 * a valid 8-bit unsigned integer.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_write_scratchpad(struct ade9113_dev *dev,
			     uint8_t val);

/* Get the value stired in the scratchpad register. */
/***************************************************************************//**
 * @brief This function is used to read the value from the scratchpad register
 * of the ADE9113 device. It should be called after the device has been
 * properly initialized. The caller must ensure that the pointer provided
 * for the output value is not null, as passing a null pointer will
 * result in an error. If the read operation is successful, the value
 * from the scratchpad register will be stored in the location pointed to
 * by the provided pointer.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param val A pointer to a `uint8_t` where the read value will be stored. Must
 * not be null; passing a null pointer will result in an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_read_scratchpad(struct ade9113_dev *dev,
			    uint8_t *val);

/* Set normal mode of operation. */
/***************************************************************************//**
 * @brief This function is used to configure the ADE9113 device to operate in
 * normal mode. It should be called after the device has been properly
 * initialized. If the provided device structure pointer is null, the
 * function will return an error code indicating that the device is not
 * available. It is important to ensure that the device is ready for mode
 * changes before invoking this function.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_set_normal_mode(struct ade9113_dev *dev);

/* Set static mode of operation. */
/***************************************************************************//**
 * @brief This function is used to configure the ADE9113 device to operate in
 * static mode, which is one of the predefined operational modes. It
 * should be called after the device has been properly initialized. If
 * the provided device structure is null, the function will return an
 * error code indicating that the device is not available. It is
 * important to ensure that the device is ready and correctly set up
 * before invoking this function to avoid unexpected behavior.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This pointer must not be null, as a null pointer will result in an
 * error code being returned.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_set_static_mode(struct ade9113_dev *dev);

/* Set static mode of operation. */
/***************************************************************************//**
 * @brief This function configures the ADE9113 device to operate in data
 * increments mode, which is useful for applications that require data to
 * be processed at the ADC conversion rate. It should be called after the
 * device has been properly initialized. If the provided device structure
 * pointer is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_set_data_increments_mode(struct ade9113_dev *dev);

/* Get ECC or PHY Error Count on ISO to NONISO Communications. */
/***************************************************************************//**
 * @brief This function is used to obtain the error count from the ADE9113
 * device, which can be useful for diagnosing communication issues or
 * device malfunctions. It should be called after the device has been
 * properly initialized. The caller must ensure that the `err_count`
 * pointer is valid and not null, as passing a null pointer will result
 * in an error. The function reads the error count from the device's
 * register and stores it in the location pointed to by `err_count.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device
 * instance. This must not be null and should point to a valid
 * initialized device.
 * @param err_count A pointer to a `uint8_t` variable where the error count will
 * be stored. This pointer must not be null; otherwise, the
 * function will return an error.
 * @return Returns 0 on success, indicating that the error count was
 * successfully retrieved and stored in `err_count`. If an error occurs,
 * such as a null pointer being passed for `err_count`, a negative error
 * code (e.g., -EINVAL) will be returned.
 ******************************************************************************/
int ade9113_get_err_count(struct ade9113_dev *dev,
			  uint8_t *err_count);

/* Invert V2 channel inputs. */
/***************************************************************************//**
 * @brief This function is used to invert the inputs of the V2 channel in the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available. This
 * function modifies the configuration register to enable the inversion
 * of the V2 channel inputs.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_invert_v2_inputs(struct ade9113_dev *dev);

/* Invert V1 channel inputs. */
/***************************************************************************//**
 * @brief This function is used to enable the inversion of the V1 channel inputs
 * for the ADE9113 device. It should be called after the device has been
 * properly initialized. If the provided `dev` pointer is null, the
 * function will return an error code indicating that the device is not
 * available. This function modifies the device's configuration register
 * to apply the inversion setting.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_invert_v1_inputs(struct ade9113_dev *dev);

/* Invert I channel inputs. */
/***************************************************************************//**
 * @brief This function is used to invert the I channel inputs of the ADE9113
 * device. It should be called after the device has been properly
 * initialized. If the provided device structure pointer is null, the
 * function will return an error code indicating that the device is not
 * available. This function modifies the configuration register to enable
 * the inversion of the I channel inputs.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_invert_i_inputs(struct ade9113_dev *dev);

/* Disable invert V2 channel inputs. */
/***************************************************************************//**
 * @brief This function is used to disable the inversion of the V2 channel
 * inputs in the ADE9113 device. It should be called when the device is
 * properly initialized and ready for configuration. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available. It is important to ensure that the
 * device is in a valid state before invoking this function to avoid
 * unexpected behavior.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_invert_v2_inputs_disable(struct ade9113_dev *dev);

/* Disable invert V1 channel inputs. */
/***************************************************************************//**
 * @brief This function is used to disable the inversion of the V1 channel
 * inputs in the ADE9113 device. It should be called when the device is
 * properly initialized and ready for configuration. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available. It is important to ensure that the
 * device is in a valid state before invoking this function to avoid
 * unexpected behavior.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_invert_v1_inputs_disable(struct ade9113_dev *dev);

/* Disable invert I channel inputs. */
/***************************************************************************//**
 * @brief This function is used to disable the inversion of the I channel inputs
 * in the ADE9113 device. It should be called when the device is properly
 * initialized and ready for configuration. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available. It is important to ensure that the device is
 * in a valid state before invoking this function to avoid unexpected
 * behavior.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_invert_i_inputs_disable(struct ade9113_dev *dev);

/* Set filter bandwidth to 2.7 kHz at 8ksps output data rate. */
/***************************************************************************//**
 * @brief This function configures the low-pass filter bandwidth of the ADE9113
 * device to 2.7 kHz, which is suitable for applications requiring this
 * specific bandwidth setting. It should be called after the device has
 * been initialized and is ready for configuration. If the provided
 * device pointer is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This pointer must not be null, as it indicates the device to be
 * configured. If it is null, the function will return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_set_lpf_bw_2_7(struct ade9113_dev *dev);

/* Set filter bandwidth to 3.3 kHz at 8ksps output data rate. */
/***************************************************************************//**
 * @brief This function configures the low-pass filter bandwidth of the ADE9113
 * device to 3.3 kHz, which is suitable for applications requiring this
 * specific bandwidth setting. It should be called after the device has
 * been properly initialized. If the provided device structure pointer is
 * null, the function will return an error code indicating that the
 * device is not available.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_set_lpf_bw_3_3(struct ade9113_dev *dev);

/* Set digital signal processing configuration. */
/***************************************************************************//**
 * @brief This function is used to configure the digital signal processing
 * settings of the ADE9113 device. It should be called after the device
 * has been initialized and before any data processing begins. The
 * configuration options are defined in the `ade9113_datapath_config_e`
 * enumeration, which specifies various sampling rates and filter
 * settings. If an invalid configuration value is provided, the function
 * will return an error code, ensuring that only valid configurations are
 * applied.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param config The desired digital signal processing configuration, specified
 * as an `ade9113_datapath_config_e` enumeration value. Valid
 * values range from `ADE9113_SINC3_32_KHZ_SAMPLING` to
 * `ADE9113_SINC3_LPF_EN_1_KHZ_SAMPLING`. If the value exceeds the
 * defined range, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the configuration
 * is invalid.
 ******************************************************************************/
int ade9113_set_dsp_config(struct ade9113_dev *dev,
			   enum ade9113_datapath_config_e config);

/* Enable write access to DC_OFFSET_MODE register. */
/***************************************************************************//**
 * @brief This function should be called when write access to the DC_OFFSET_MODE
 * register is required. It is essential to ensure that the `dev`
 * parameter is properly initialized and not null before invoking this
 * function. If the `dev` parameter is null, the function will return an
 * error code indicating that the device is not available. Successful
 * execution allows subsequent write operations to the DC_OFFSET_MODE
 * register.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This parameter must not be null and should point to a valid
 * initialized device structure. If it is null, the function will
 * return an error code.
 * @return Returns 0 on success, indicating that write access to the
 * DC_OFFSET_MODE register has been enabled. If the input parameter is
 * invalid, it returns an error code.
 ******************************************************************************/
int ade9113_enable_wa_dc_offset_mode(struct ade9113_dev *dev);

/* Disable write access to DC_OFFSET_MODE register. */
/***************************************************************************//**
 * @brief This function should be called when write access to the DC_OFFSET_MODE
 * register needs to be disabled, typically after configuration changes
 * have been made. It is important to ensure that the `dev` parameter is
 * properly initialized and not null before calling this function, as
 * passing a null pointer will result in an error. The function will
 * return an error code if the device structure is invalid.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_wa_dc_offset_mode(struct ade9113_dev *dev);

/* Get register map CRC. */
/***************************************************************************//**
 * @brief This function retrieves the CRC value from the ADE9113 device, which
 * is useful for verifying data integrity. It must be called after the
 * device has been properly initialized. The function expects a valid
 * pointer to a `uint16_t` variable where the CRC value will be stored.
 * If the provided pointer is null, the function will return an error.
 * The function performs two read operations to obtain the high and low
 * bytes of the CRC value, and it will return an error code if either
 * read operation fails.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param crc Pointer to a `uint16_t` where the CRC value will be stored. Must
 * not be null; if null, the function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the read operations.
 ******************************************************************************/
int ade9113_get_crc(struct ade9113_dev *dev, uint16_t *crc);

/* Refresh EFuse Memory. */
/***************************************************************************//**
 * @brief This function is used to refresh the EFuse memory of the ADE9113
 * device. It should be called when the device is properly initialized
 * and ready for operation. If the provided device structure pointer is
 * null, the function will return an error code indicating that the
 * device is not available. It is important to ensure that the device is
 * in a state that allows for this operation to avoid unexpected
 * behavior.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_efuse_refresh(struct ade9113_dev *dev);

/* Select EMI frequency hopping. */
/***************************************************************************//**
 * @brief This function is used to configure the EMI frequency hopping settings
 * of the ADE9113 device. It should be called after the device has been
 * initialized and is ready for configuration. The function accepts a
 * configuration parameter that specifies the desired EMI mode. If an
 * invalid configuration value is provided, the function will return an
 * error code, ensuring that only valid settings are applied.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param config An enumeration value of type `ade9113_emi_config_e` that
 * specifies the EMI configuration. Valid values range from
 * `ADE9113_SAWTOOTH_FREQUENCY_RISING` to
 * `ADE9113_RANDOM_HOPPING_FREQUENCY`. If the value exceeds
 * `ADE9113_RANDOM_HOPPING_FREQUENCY`, the function will return an
 * error.
 * @return Returns 0 on success, or a negative error code if the configuration
 * is invalid.
 ******************************************************************************/
int ade9113_set_emi_config(struct ade9113_dev *dev,
			   enum ade9113_emi_config_e config);

/* Get EMI HI mask. */
/***************************************************************************//**
 * @brief This function retrieves the EMI high mask from the ADE9113 device. It
 * should be called after the device has been properly initialized. The
 * caller must ensure that the pointer for the mask is valid and not
 * null, as passing a null pointer will result in an error. The function
 * reads the mask value from the device's register and stores it in the
 * provided memory location.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param msk Pointer to a `uint8_t` where the retrieved mask will be stored.
 * Must not be null; passing a null pointer will return an error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_get_emi_hi_mask(struct ade9113_dev *dev, uint8_t *msk);

/* Get EMI LO mask. */
/***************************************************************************//**
 * @brief This function is used to obtain the EMI low mask value from the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. The caller must ensure that the pointer for the mask is
 * valid and not null, as passing a null pointer will result in an error.
 * The function reads the mask value from the device's register and
 * stores it in the provided pointer.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param msk Pointer to a `uint8_t` where the EMI low mask value will be
 * stored. Must not be null; passing a null pointer will return an
 * error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_get_emi_lo_mask(struct ade9113_dev *dev, uint8_t *msk);

/* Set EMI HI mask. */
/***************************************************************************//**
 * @brief This function is used to configure the EMI high mask for the ADE9113
 * device. It should be called after the device has been properly
 * initialized. The function expects a valid device structure pointer and
 * a mask value. If the device pointer is null, the function will return
 * an error code indicating that the device is not available. The mask
 * value should be a valid 8-bit value, and it will be written to the
 * appropriate register in the device.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param msk An 8-bit mask value to be set. Valid values are from 0 to 255.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_set_emi_hi_mask(struct ade9113_dev *dev, uint8_t msk);

/* Set EMI LO mask. */
/***************************************************************************//**
 * @brief This function is used to configure the EMI low mask for the ADE9113
 * device. It should be called after the device has been properly
 * initialized. The function expects a valid device structure pointer and
 * a mask value. If the device pointer is null, the function will return
 * an error code indicating that the device is not available. The mask
 * value should be a valid 8-bit value, and it will be written to the
 * corresponding register in the device.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @param msk An 8-bit mask value to be set for the EMI low mask. Valid values
 * are from 0x00 to 0xFF.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_set_emi_lo_mask(struct ade9113_dev *dev, uint8_t msk);

/* Get EMI HI limit. */
/***************************************************************************//**
 * @brief This function retrieves the high limit value for EMI (Electromagnetic
 * Interference) from the ADE9113 device. It should be called after the
 * device has been properly initialized. The caller must ensure that the
 * `limit` pointer is valid and not null, as passing a null pointer will
 * result in an error. The function will read the value from the device's
 * register and store it in the location pointed to by `limit.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param limit Pointer to a `uint8_t` where the high limit value will be
 * stored. Must not be null; passing a null pointer will return an
 * error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_get_emi_hi_limit(struct ade9113_dev *dev, uint8_t *limit);

/* Get EMI MID limit. */
/***************************************************************************//**
 * @brief This function is used to obtain the EMI mid limit value from the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. The caller must ensure that the `limit` pointer is valid
 * and not null, as passing a null pointer will result in an error. The
 * function reads the value from the device's register and stores it in
 * the location pointed to by `limit`. If the read operation is
 * successful, the function will return 0; otherwise, it will return a
 * negative error code.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This must not be null and should point to a valid initialized
 * device.
 * @param limit A pointer to a `uint8_t` variable where the retrieved EMI mid
 * limit value will be stored. This pointer must not be null;
 * otherwise, the function will return an error.
 * @return Returns 0 on success, indicating that the EMI mid limit value has
 * been successfully retrieved and stored in the provided `limit`
 * pointer. If an error occurs, a negative error code is returned.
 ******************************************************************************/
int ade9113_get_emi_mid_limit(struct ade9113_dev *dev, uint8_t *limit);

/* Get EMI LO limit. */
/***************************************************************************//**
 * @brief This function is used to obtain the EMI low limit value from the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. The caller must ensure that the `limit` pointer is valid
 * and not null, as passing a null pointer will result in an error. The
 * function reads the value from the device's register and stores it in
 * the location pointed to by `limit`. If the read operation is
 * successful, the function returns 0; otherwise, it returns a negative
 * error code.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This must not be null and should point to a valid initialized
 * device.
 * @param limit A pointer to a `uint8_t` variable where the EMI low limit value
 * will be stored. This pointer must not be null; otherwise, the
 * function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_get_emi_lo_limit(struct ade9113_dev *dev, uint8_t *limit);

/* Enable/Disable interrupt. */
/***************************************************************************//**
 * @brief This function is used to control the interrupt settings for a specific
 * register in the ADE9113 device. It should be called after the device
 * has been initialized and is ready for configuration. The `dev`
 * parameter must point to a valid `ade9113_dev` structure, and the
 * `reg_addr` should correspond to a valid register address defined in
 * the device's register map. The `int_msk` parameter specifies which
 * interrupts to enable or disable, while the `en` parameter determines
 * whether to enable (1) or disable (0) the specified interrupts. If the
 * `dev` parameter is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register for which the interrupt control
 * is being set. Must be a valid register address as defined in
 * the device's register map.
 * @param int_msk A bitmask indicating which interrupts to enable or disable.
 * The valid range depends on the specific register being
 * configured.
 * @param en An integer value where 1 enables the specified interrupts and 0
 * disables them. Must be either 0 or 1.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as if the device is not available.
 ******************************************************************************/
int ade9113_control_interrupt(struct ade9113_dev *dev, uint8_t reg_addr,
			      uint8_t int_msk, uint8_t en);

/* Enable STATUS1X interrupt. */
/***************************************************************************//**
 * @brief This function should be called to enable the STATUS1X interrupt for
 * the ADE9113 device. It is essential to ensure that the `dev` parameter
 * points to a valid initialized `ade9113_dev` structure before invoking
 * this function. If the `dev` parameter is null, the function will
 * return an error code indicating that the device is not available. This
 * function is typically used in scenarios where the application needs to
 * respond to specific events indicated by the STATUS1X register.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This parameter must not be null, as it indicates the device on
 * which the interrupt is to be enabled. If the pointer is null, the
 * function will return an error code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9113_enable_status1x_int(struct ade9113_dev *dev);

/* Disable STATUS1X interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the STATUS1X interrupt for the
 * ADE9113 device. It should be called when the user no longer needs to
 * receive notifications for events related to STATUS1X. Before calling
 * this function, ensure that the `dev` parameter is properly initialized
 * and points to a valid `ade9113_dev` structure. If the `dev` parameter
 * is null, the function will return an error code indicating that the
 * device is not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; if null, the function returns -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_disable_status1x_int(struct ade9113_dev *dev);

/* Enable STATUS2X interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the STATUS2X interrupt for the ADE9113
 * device. It should be called after the device has been properly
 * initialized. If the provided device pointer is null, the function will
 * return an error code indicating that the device is not available.
 * Enabling this interrupt allows the device to signal specific events
 * related to the STATUS2X register, which can be useful for monitoring
 * the device's operational state.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9113_enable_status2x_int(struct ade9113_dev *dev);

/* Disable STATUS2X interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the STATUS2X interrupt for the
 * specified device. It should be called when the application no longer
 * needs to respond to STATUS2X events, typically after handling the
 * interrupt or when shutting down the device. The function requires a
 * valid device structure pointer, which must be initialized prior to
 * calling this function. If the provided device pointer is null, the
 * function will return an error code indicating that the device is not
 * available.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9113_disable_status2x_int(struct ade9113_dev *dev);

/* Enable COM_UP interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the COM_UP interrupt for the ADE9113
 * device. It should be called after the device has been properly
 * initialized. If the provided device pointer is null, the function will
 * return an error code indicating that the device is not available.
 * Enabling this interrupt allows the device to signal when a specific
 * condition related to communication is met, which can be useful for
 * monitoring and handling events in real-time applications.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This pointer must not be null, as the function will return an
 * error if it is. The caller retains ownership of the device
 * structure.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9113_enable_com_up_int(struct ade9113_dev *dev);

/* Disable COM_UP interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the COM_UP interrupt for the
 * specified device. It should be called when the application no longer
 * needs to respond to COM_UP events, typically during cleanup or when
 * reconfiguring the device. Before calling this function, ensure that
 * the `dev` parameter points to a valid `ade9113_dev` structure. If
 * `dev` is null, the function will return an error code indicating that
 * the device is not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_com_up_int(struct ade9113_dev *dev);

/* Enable CRC_CHG interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the CRC change interrupt for the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. If the provided device structure pointer is null, the
 * function will return an error code indicating that the device is not
 * available. Enabling this interrupt allows the system to respond to
 * changes in the CRC status, which can be critical for maintaining data
 * integrity.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_crc_chg_int(struct ade9113_dev *dev);

/* Disable CRC_CHG interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the CRC change interrupt for the
 * ADE9113 device. It should be called when the user no longer wishes to
 * receive notifications about CRC changes, typically after handling the
 * interrupt or when the device is being reconfigured. The function
 * requires a valid device structure pointer, which must be initialized
 * prior to calling this function. If the provided device pointer is
 * null, the function will return an error code indicating that the
 * device is not available.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_disable_crc_chg_int(struct ade9113_dev *dev);

/* Enable SPI_CRC_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the SPI CRC error interrupt for the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available. Enabling
 * this interrupt allows the system to respond to CRC errors detected
 * during SPI communication, which is crucial for maintaining data
 * integrity.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This parameter must not be null, as it indicates the device on
 * which the interrupt is to be enabled. If null is passed, the
 * function will return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_spi_crc_err_int(struct ade9113_dev *dev);

/* Disable SPI_CRC_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the SPI CRC error interrupt for the
 * ADE9113 device. It should be called when the user wants to stop
 * receiving notifications for CRC errors that occur during SPI
 * communication. Before calling this function, ensure that the `dev`
 * parameter points to a valid `ade9113_dev` structure. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev Pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_spi_crc_err_int(struct ade9113_dev *dev);

/* Enable COMFLT_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the COMFLT_ERR interrupt for the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available. Enabling
 * this interrupt allows the device to signal when a communication fault
 * error occurs, which can be useful for monitoring and handling errors
 * in real-time.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code (-ENODEV). The caller retains ownership of
 * the `dev` pointer.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_comflt_err_int(struct ade9113_dev *dev);

/* Disable COMFLT_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the COMFLT_ERR interrupt for the
 * specified device. It should be called when the application no longer
 * needs to respond to COMFLT_ERR events, typically after handling the
 * interrupt or when shutting down the device. The function requires a
 * valid device structure pointer; passing a null pointer will result in
 * an error. It is important to ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; passing a null pointer will return an error code
 * -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_disable_comflt_err_int(struct ade9113_dev *dev);

/* Enable V2_WAV_OVRNG interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the V2 waveform overflow interrupt for
 * the ADE9113 device. It should be called after the device has been
 * properly initialized. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * Enabling this interrupt allows the system to respond to overflow
 * conditions in the V2 waveform data, which is critical for maintaining
 * accurate measurements.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code (-ENODEV). The caller retains ownership of
 * the `dev` pointer.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_enable_v2_wav_ovrng_int(struct ade9113_dev *dev);

/* Disable V2_WAV_OVRNG interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the V2 waveform overrange interrupt
 * for the ADE9113 device. It should be called when the user no longer
 * wishes to receive notifications for this specific interrupt condition.
 * Before calling this function, ensure that the `dev` parameter points
 * to a valid `ade9113_dev` structure, as passing a null pointer will
 * result in an error. The function will return a negative error code if
 * the device is not properly initialized.
 *
 * @param dev Pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; passing a null pointer will result in an error
 * code of -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_disable_v2_wav_ovrng_int(struct ade9113_dev *dev);

/* Enable V1_WAV_OVRNG interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for the V1 waveform
 * overrange condition in the ADE9113 device. It should be called after
 * the device has been properly initialized. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available. This function is typically used in
 * applications where monitoring the V1 waveform is critical, allowing
 * the system to respond to overrange conditions.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This parameter must not be null, as it indicates the device
 * context. If null is passed, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_v1_wav_ovrng_int(struct ade9113_dev *dev);

/* Disable V1_WAV_OVRNG interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the V1 waveform overrange interrupt
 * for the ADE9113 device. It should be called when the user no longer
 * wishes to receive notifications for this specific interrupt condition.
 * Before calling this function, ensure that the `dev` parameter points
 * to a valid `ade9113_dev` structure that has been properly initialized.
 * If the `dev` parameter is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This pointer must not be null and should point to a valid
 * initialized device structure. If it is null, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_v1_wav_ovrng_int(struct ade9113_dev *dev);

/* Enable I_WAV_OVRNG interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the I_WAV_OVRNG interrupt for the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available. This
 * function is typically used in scenarios where the application needs to
 * respond to over-range conditions in the I_WAV signal.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_i_wav_ovrng_int(struct ade9113_dev *dev);

/* Disable I_WAV_OVRNG interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the I_WAV_OVRNG interrupt for the
 * ADE9113 device. It should be called when the user wants to stop
 * receiving notifications for this specific interrupt condition. Before
 * calling this function, ensure that the `dev` parameter points to a
 * valid initialized `ade9113_dev` structure. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_i_wav_ovrng_int(struct ade9113_dev *dev);

/* Enable ADC_SYNC_DONE interrupt. */
/***************************************************************************//**
 * @brief This function should be called to enable the ADC synchronization done
 * interrupt for the specified device. It is essential to ensure that the
 * `dev` parameter is a valid pointer to an initialized `ade9113_dev`
 * structure. If the device pointer is null, the function will return an
 * error code indicating that the device is not available. This function
 * is typically used in scenarios where the application needs to be
 * notified when the ADC synchronization process is complete.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9113_enable_adc_sync_done_int(struct ade9113_dev *dev);

/* Disable ADC_SYNC_DONE interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the ADC synchronization done
 * interrupt for the specified device. It should be called when the ADC
 * synchronization done interrupt is no longer needed, such as during
 * device shutdown or when reconfiguring the device. Before calling this
 * function, ensure that the `dev` parameter is properly initialized and
 * points to a valid `ade9113_dev` structure. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_adc_sync_done_int(struct ade9113_dev *dev);

/* Enable ISO_CLK_STBL_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for ISO clock stability
 * errors in the ADE9113 device. It should be called after the device has
 * been properly initialized. If the `dev` parameter is null, the
 * function will return an error code indicating that the device is not
 * available. This function is typically used in scenarios where
 * monitoring the stability of the ISO clock is critical for the
 * application's operation.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_iso_clk_stbl_err_int(struct ade9113_dev *dev);

/* Disable ISO_CLK_STBL_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the ISO clock stability error
 * interrupt for the ADE9113 device. It should be called when the user
 * wants to stop receiving notifications about ISO clock stability
 * errors. Before calling this function, ensure that the `dev` parameter
 * points to a valid `ade9113_dev` structure. If `dev` is null, the
 * function will return an error code. This function does not have any
 * side effects beyond modifying the interrupt state.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; if null, the function returns -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_iso_clk_stbl_err_int(struct ade9113_dev *dev);

/* Enable ISO_PHY_CRC_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for ISO PHY CRC errors
 * in the ADE9113 device. It should be called after the device has been
 * properly initialized. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * Enabling this interrupt allows the system to respond to CRC errors
 * detected in the ISO PHY layer, which is crucial for maintaining data
 * integrity.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9113_enable_iso_phy_crc_err_int(struct ade9113_dev *dev);

/* Disable ISO_PHY_CRC_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the interrupt for ISO PHY CRC errors
 * in the ADE9113 device. It should be called when the user wants to stop
 * receiving notifications for CRC errors related to the ISO PHY
 * interface. Before calling this function, ensure that the `dev`
 * parameter points to a valid `ade9113_dev` structure, as passing a null
 * pointer will result in an error. This function is typically used in
 * scenarios where error handling for CRC issues is no longer needed.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; passing a null pointer will return an error code
 * -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_disable_iso_phy_crc_err_int(struct ade9113_dev *dev);

/* Enable ISO_EFUSE_MEM_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for ISO EFUSE memory
 * errors in the ADE9113 device. It should be called after the device has
 * been properly initialized. If the `dev` parameter is null, the
 * function will return an error code indicating that the device is not
 * available. This function is typically used in scenarios where
 * monitoring for memory errors is critical, allowing the system to
 * respond to such events.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_iso_efuse_mem_err_int(struct ade9113_dev *dev);

/* Disable ISO_EFUSE_MEM_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the interrupt for ISO EFUSE memory
 * errors in the ADE9113 device. It should be called when the user wants
 * to stop receiving notifications for this specific interrupt condition.
 * Before calling this function, ensure that the `dev` parameter points
 * to a valid `ade9113_dev` structure. If `dev` is null, the function
 * will return an error code indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_iso_efuse_mem_err_int(struct ade9113_dev *dev);

/* Enable ISO_DIG_MOD_V2_OVF interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for the ISO_DIG_MOD_V2
 * overflow condition in the ADE9113 device. It should be called after
 * the device has been properly initialized. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available. Enabling this interrupt allows the system to
 * respond to overflow events, which can be critical for maintaining
 * accurate measurements.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_iso_dig_mod_v2_ovf_int(struct ade9113_dev *dev);

/* Disable ISO_DIG_MOD_V2_OVF interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the ISO_DIG_MOD_V2_OVF interrupt for
 * the ADE9113 device. It should be called when the interrupt is no
 * longer needed, typically during device configuration or shutdown.
 * Before calling this function, ensure that the `dev` parameter points
 * to a valid `ade9113_dev` structure. If `dev` is null, the function
 * will return an error code indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_disable_iso_dig_mod_v2_ovf_int(struct ade9113_dev *dev);

/* Enable ISO_DIG_MOD_V1_OVF interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for the ISO_DIG_MOD_V1
 * overflow condition in the ADE9113 device. It should be called after
 * the device has been properly initialized. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available. This function is typically used in scenarios
 * where monitoring the overflow condition is critical for the
 * application.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_iso_dig_mod_v1_ovf_int(struct ade9113_dev *dev);

/* Disable ISO_DIG_MOD_V1_OVF interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the ISO digital mode V1 overflow
 * interrupt for the specified device. It should be called when the
 * interrupt is no longer needed, typically during device configuration
 * or shutdown. Before calling this function, ensure that the `dev`
 * parameter points to a valid `ade9113_dev` structure. If `dev` is null,
 * the function will return an error code indicating that the device is
 * not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_disable_iso_dig_mod_v1_ovf_int(struct ade9113_dev *dev);

/* Enable ISO_DIG_MOD_I_OVF interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for the ISO_DIG_MOD_I
 * overflow condition in the ADE9113 device. It should be called after
 * the device has been properly initialized. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available. It is important to ensure that the device is
 * in a valid state before calling this function to avoid unexpected
 * behavior.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_enable_iso_dig_mod_i_ovf_int(struct ade9113_dev *dev);

/* Disable ISO_DIG_MOD_I_OVF interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the ISO digital modulation I overflow
 * interrupt for the specified device. It should be called when the
 * interrupt is no longer needed, typically during device shutdown or
 * reconfiguration. Before calling this function, ensure that the `dev`
 * parameter points to a valid `ade9113_dev` structure. If `dev` is null,
 * the function will return an error code indicating that the device is
 * not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_iso_dig_mod_i_ovf_int(struct ade9113_dev *dev);

/* Enable ISO_TEST_MMR_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to enable the ISO_TEST_MMR_ERR interrupt for the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available. This
 * function is typically used in scenarios where monitoring for specific
 * error conditions related to the ISO_TEST_MMR is required.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9113_enable_iso_test_mmr_err_int(struct ade9113_dev *dev);

/* Disable ISO_TEST_MMR_ERR interrupt. */
/***************************************************************************//**
 * @brief This function is used to disable the ISO_TEST_MMR_ERR interrupt for
 * the ADE9113 device. It should be called when the user wants to stop
 * receiving notifications for this specific interrupt condition. Before
 * calling this function, ensure that the `dev` parameter is properly
 * initialized and points to a valid `ade9113_dev` structure. If the
 * `dev` parameter is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_disable_iso_test_mmr_err_int(struct ade9113_dev *dev);

/* Select zero crossing edge. */
/***************************************************************************//**
 * @brief This function configures the zero crossing detection edge for the
 * ADE9113 device. It should be called after the device has been
 * initialized and is ready for configuration. The function allows the
 * user to specify whether to detect zero crossings on the positive
 * slope, negative slope, both slopes, or to reflect the sign of the
 * input signal. If the provided `dev` pointer is null or if the `sel`
 * parameter exceeds the valid range, the function will return an error
 * code.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param sel An enumeration value of type `ade9113_zx_edge_sel_e` that
 * specifies the edge selection for zero crossing detection. Valid
 * values are: ADE9113_ZX_INPUT_SIGNAL_SIGN,
 * ADE9113_ZX_DETECT_POSITIVE_SLOPE,
 * ADE9113_ZX_DETECT_NEGATIVE_SLOPE, and
 * ADE9113_ZX_DETECT_BOTH_SLOPES. If the value exceeds
 * ADE9113_ZX_DETECT_BOTH_SLOPES, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the selection value is invalid.
 ******************************************************************************/
int ade9113_select_zero_crossing_edge(struct ade9113_dev *dev,
				      enum ade9113_zx_edge_sel_e sel);

/* Select zero crossing channel. */
/***************************************************************************//**
 * @brief This function is used to configure which channel's zero crossing
 * output is routed to the ZX pin of the ADE9113 device. It should be
 * called after the device has been initialized and is ready for
 * configuration. The function will return an error if the provided
 * device pointer is null or if the specified channel configuration is
 * invalid. Valid configurations include disabling the zero crossing
 * output or selecting the I, V1, or V2 channels. It is important to
 * ensure that the configuration is within the defined limits to avoid
 * unexpected behavior.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param cfg The zero crossing channel configuration, which must be one of the
 * values defined in the `ade9113_zx_channel_cfg_e` enumeration.
 * Valid values are ADE9113_ZX_DISABLE, ADE9113_ZX_I_SEL,
 * ADE9113_ZX_V1_SEL, and ADE9113_ZX_V2_SEL. If an invalid value is
 * provided, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the configuration value is invalid.
 ******************************************************************************/
int ade9113_select_zero_crossing_channel(struct ade9113_dev *dev,
		enum ade9113_zx_channel_cfg_e cfg);

/* ADC prepare broadcast. */
/***************************************************************************//**
 * @brief This function should be called to configure the ADC device for
 * broadcast mode, which is necessary before initiating any broadcast
 * operations. It is important to ensure that the `dev` parameter is a
 * valid pointer to an initialized `ade9113_dev` structure. If the `dev`
 * pointer is null, the function will return an error code indicating
 * that the device is not available. This function does not modify the
 * state of the device if called with an invalid pointer.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the ADC
 * device. Must not be null; otherwise, the function will return an
 * error code.
 * @return Returns 0 on success, indicating that the ADC has been successfully
 * prepared for broadcast mode. If the preparation fails, a negative
 * error code is returned.
 ******************************************************************************/
int ade9113_adc_prepare_broadcast(struct ade9113_dev *dev);

/* ADC align. */
/***************************************************************************//**
 * @brief This function is used to align the ADC in the ADE9113 device. It
 * should be called after the device has been properly initialized. If
 * the `dev` parameter is null, the function will return an error code
 * indicating that the device is not available. Successful execution will
 * update the relevant register to enable ADC alignment.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_adc_align(struct ade9113_dev *dev);

/* ADC snapshot. */
/***************************************************************************//**
 * @brief This function is used to capture a snapshot of the ADC data from the
 * ADE9113 device. It should be called after the device has been properly
 * initialized. If the provided device structure pointer is null, the
 * function will return an error code indicating that the device is not
 * available. The function does not modify any input parameters.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_adc_snapshot(struct ade9113_dev *dev);

/* Get interrupt indicator from STATUS register. */
/***************************************************************************//**
 * @brief This function is used to obtain the interrupt status of the ADE9113
 * device by reading a specific register and applying a mask to filter
 * the relevant bits. It should be called after the device has been
 * properly initialized. The `status` parameter must point to a valid
 * memory location where the resulting status will be stored. If the
 * `status` pointer is null, the function will return an error. The
 * function may also return an error if the read operation from the
 * device fails.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param addr The address of the register to read from. Valid values are
 * defined in the ADE9113 register map.
 * @param msk A bitmask used to filter the status bits of interest. Should be a
 * valid 8-bit value.
 * @param status Pointer to a `uint8_t` where the resulting status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_get_int_status(struct ade9113_dev *dev, uint8_t addr, uint8_t msk,
			   uint8_t *status);

/* Get STATUSx register value. */
/***************************************************************************//**
 * @brief This function is used to read the status value from a specific
 * register of the ADE9113 device. It should be called after the device
 * has been properly initialized. The `addr` parameter specifies the
 * register address from which to read the status. The `status` parameter
 * is a pointer to a variable where the retrieved status value will be
 * stored. It is important to ensure that the `status` pointer is not
 * null before calling this function, as passing a null pointer will
 * result in an error. The function will return a negative error code if
 * the read operation fails, or zero on success.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param addr The address of the register to read from. Valid addresses are
 * defined in the ADE9113 register map.
 * @param status A pointer to a `uint8_t` variable where the status value will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_get_statusx_val(struct ade9113_dev *dev, uint8_t addr,
			    uint8_t *status);

/* Get STATUS1X Indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the current value of the STATUS1X
 * register from the ADE9113 device. It should be called after the device
 * has been properly initialized and configured. The function will write
 * the retrieved status value into the provided pointer. If the `status`
 * pointer is null, the function will not perform any operation and may
 * return an error code. Ensure that the device is ready to provide
 * status information before calling this function.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the status value will be stored.
 * Caller retains ownership and must ensure it is not null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_get_status1x(struct ade9113_dev *dev, uint8_t *status);

/* Get STATUS2 indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the STATUS2 indicator from the ADE9113
 * device, which provides information about the device's operational
 * status. It should be called after the device has been properly
 * initialized and configured. The function will write the retrieved
 * status value to the provided pointer. If the `dev` or `status`
 * pointers are null, the function will handle this gracefully by
 * returning an error code.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param status A pointer to a `uint8_t` variable where the STATUS2 value will
 * be stored. Caller retains ownership and must ensure it is not
 * null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_status2x(struct ade9113_dev *dev, uint8_t *status);

/* Get RESET_DONE indicator. */
/***************************************************************************//**
 * @brief This function is used to check if the reset operation of the ADE9113
 * device has been completed. It should be called after a reset command
 * has been issued to the device. The function will update the provided
 * status pointer with the result of the reset operation. It is important
 * to ensure that the `dev` parameter is a valid pointer to an
 * initialized `ade9113_dev` structure, and the `status` pointer must not
 * be null.
 *
 * @param dev A pointer to an initialized `ade9113_dev` structure representing
 * the device. Must not be null.
 * @param status A pointer to a `uint8_t` variable where the reset status will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_get_reset_done(struct ade9113_dev *dev, uint8_t *status);

/* Get COM_UP indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the communication status of the
 * ADE9113 device, specifically checking if the communication is up. It
 * should be called after the device has been properly initialized. The
 * function will write the status result to the provided pointer. If the
 * `dev` parameter is null or if the `status` pointer is null, the
 * function will handle these cases gracefully, typically by returning an
 * error code.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the communication status will be
 * stored. Must not be null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_com_up(struct ade9113_dev *dev, uint8_t *status);

/* Get CRC_CHG indicator. */
/***************************************************************************//**
 * @brief This function is used to check if there has been a change in the CRC
 * (Cyclic Redundancy Check) status of the ADE9113 device. It should be
 * called after the device has been initialized and is ready for
 * operation. The function will update the provided `status` pointer with
 * the current CRC change status, which can be used to determine if the
 * CRC has changed since the last check. It is important to ensure that
 * the `dev` parameter is valid and that the device is properly
 * configured before calling this function.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param status A pointer to a `uint8_t` variable where the CRC change status
 * will be stored. Must not be null; the function will write the
 * status to this location.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_crc_chg(struct ade9113_dev *dev, uint8_t *status);

/* Get EFUSE_MEM_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to check for any errors related to the EFUSE
 * memory of the ADE9113 device. It should be called after the device has
 * been initialized and is operational. The function will update the
 * provided `status` pointer with the error status, which can indicate
 * whether an EFUSE memory error has occurred. It is important to ensure
 * that the `status` pointer is valid and points to a writable memory
 * location before calling this function.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param status A pointer to a `uint8_t` variable where the error status will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_get_efuse_mem_err(struct ade9113_dev *dev, uint8_t *status);

/* Get SPI_CRC_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to check for any SPI CRC errors that may have
 * occurred during communication with the ADE9113 device. It should be
 * called after the device has been initialized and during normal
 * operation to monitor the integrity of data transfers. The function
 * will update the provided `status` pointer with the current SPI CRC
 * error status, which can be used to determine if any errors have been
 * detected. It is important to ensure that the `status` pointer is valid
 * and points to a memory location that can store the result.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device
 * instance. Must not be null.
 * @param status A pointer to a `uint8_t` variable where the SPI CRC error
 * status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_get_spi_crc_err(struct ade9113_dev *dev, uint8_t *status);

/* Get COMFLT_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the communication fault error status
 * from the ADE9113 device. It should be called after the device has been
 * initialized and is ready for operation. The function checks the
 * relevant status register and updates the provided status pointer with
 * the result. If the `status` pointer is null, the function will not
 * perform any operation and may return an error code. It is important to
 * ensure that the device is properly configured and operational before
 * invoking this function.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device
 * instance. Must not be null.
 * @param status A pointer to a `uint8_t` variable where the communication fault
 * error status will be stored. Must not be null; if null, the
 * function will not perform any operation.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_comflt_err(struct ade9113_dev *dev, uint8_t *status);

/* Clear the RESET_DONE int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the RESET_DONE
 * status in the ADE9113 device. It should be called when the application
 * has processed the RESET_DONE interrupt and is ready to clear it,
 * ensuring that subsequent interrupts can be detected. It is important
 * to ensure that the `dev` parameter points to a valid and initialized
 * `ade9113_dev` structure before calling this function.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This pointer must not be null and should point to a valid device
 * that has been properly initialized.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_reset_done_int(struct ade9113_dev *dev);

/* Clear the COM_UP int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the COM_UP interrupt mask in the
 * ADE9113 device. It should be called when the application has processed
 * the COM_UP interrupt and wants to reset the interrupt status. Ensure
 * that the device is properly initialized before calling this function,
 * as calling it on an uninitialized device may lead to undefined
 * behavior.
 *
 * @param dev A pointer to a `struct ade9113_dev` representing the device
 * instance. This pointer must not be null and should point to a
 * valid device structure that has been initialized.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_com_up_int(struct ade9113_dev *dev);

/* Clear the CRC_CHG int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the CRC change interrupt status in the
 * ADE9113 device. It should be called when the CRC change interrupt has
 * been triggered and the user wants to acknowledge or reset this
 * interrupt condition. It is important to ensure that the device is
 * properly initialized before calling this function, as calling it on an
 * uninitialized device may lead to undefined behavior.
 *
 * @param dev Pointer to a `struct ade9113_dev` representing the device
 * instance. Must not be null and should point to a valid initialized
 * device structure.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_crc_chg_int(struct ade9113_dev *dev);

/* Clear the SPI_CRC_ERR int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the SPI CRC error interrupt status in
 * the ADE9113 device. It should be called when the SPI CRC error has
 * been detected and needs to be acknowledged to prevent further
 * interrupt triggers. Ensure that the device is properly initialized
 * before calling this function, as calling it on an uninitialized device
 * may lead to undefined behavior.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null and should point to a valid device instance that
 * has been initialized.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int ade9113_clear_spi_crc_err_int(struct ade9113_dev *dev);

/* Clear the COMFLT_ERR int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the COMFLT_ERR interrupt mask in the
 * ADE9113 device. It should be called when the COMFLT_ERR interrupt has
 * been triggered and the user wants to acknowledge and clear the
 * interrupt condition. Ensure that the device is properly initialized
 * before calling this function. If the `dev` parameter is null, the
 * function will not perform any action.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will not execute.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_comflt_err_int(struct ade9113_dev *dev);

/* Get V2_WAV_OVRNG indicator. */
/***************************************************************************//**
 * @brief This function is used to check if there is an overflow condition in
 * the V2 waveform data. It should be called after the device has been
 * properly initialized and configured. The function will update the
 * provided `status` pointer with the overflow status, which can be used
 * to determine if the V2 waveform data is valid or if an overflow has
 * occurred. It is important to ensure that the `status` pointer is not
 * null before calling this function, as passing a null pointer will lead
 * to undefined behavior.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This pointer must not be null and should point to a valid device
 * instance that has been initialized.
 * @param status A pointer to a `uint8_t` variable where the overflow status
 * will be stored. This pointer must not be null. The function
 * will write the overflow status to this variable, indicating
 * whether an overflow has occurred.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The `status` variable will be updated with the overflow
 * status.
 ******************************************************************************/
int ade9113_get_v2_wav_ovrng(struct ade9113_dev *dev, uint8_t *status);

/* Clear the V2_WAV_OVRNG int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the interrupt status for the V2
 * waveform overflow condition in the ADE9113 device. It should be called
 * when the application detects that the V2 waveform overflow interrupt
 * has occurred, ensuring that the interrupt is acknowledged and cleared.
 * This function is typically invoked in the interrupt service routine or
 * immediately after handling the overflow condition to prevent repeated
 * triggering of the same interrupt. It is important to ensure that the
 * `dev` parameter points to a valid and initialized `ade9113_dev`
 * structure before calling this function.
 *
 * @param dev Pointer to an `ade9113_dev` structure representing the device.
 * Must not be null and should point to a valid, initialized device
 * instance.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_v2_wav_ovrng_int(struct ade9113_dev *dev);

/* Get V1_WAV_OVRNG indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the overflow status of the V1 waveform
 * from the ADE9113 device. It should be called after the device has been
 * properly initialized and configured. The function will update the
 * provided `status` pointer with the current overflow status, which can
 * indicate whether the V1 waveform data has exceeded its expected range.
 * It is important to ensure that the `status` pointer is valid and
 * points to a memory location that can store the result.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device
 * instance.
 * @param status A pointer to a `uint8_t` variable where the overflow status
 * will be stored. Must not be null; the function will write the
 * overflow status to this location.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error occurred.
 ******************************************************************************/
int ade9113_get_v1_wav_ovrng(struct ade9113_dev *dev, uint8_t *status);

/* Clear the V1_WAV_OVRNG int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the V1 waveform overrange interrupt
 * status in the ADE9113 device. It should be called when the application
 * detects that the V1 waveform overrange condition has been handled,
 * ensuring that the interrupt can be triggered again in the future if
 * the condition reoccurs. It is important to ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device
 * instance. This pointer must not be null and should point to a
 * valid device that has been initialized.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_v1_wav_ovrng_int(struct ade9113_dev *dev);

/* Get I_WAV_OVRNG indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the overflow status of the I_WAV
 * channel from the ADE9113 device. It should be called after the device
 * has been properly initialized and configured. The function checks the
 * relevant status register and updates the provided status pointer with
 * the result. If the `status` pointer is null, the function will not
 * perform any operation and may return an error code. It is important to
 * handle the return value appropriately to ensure that the status is
 * retrieved successfully.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device
 * instance. Must not be null.
 * @param status A pointer to a `uint8_t` variable where the overflow status
 * will be stored. The caller retains ownership of this variable,
 * and it must not be null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_i_wav_ovrng(struct ade9113_dev *dev, uint8_t *status);

/* Clear the I_WAV_OVRNG int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the I_WAV_OVRNG
 * condition in the ADE9113 device. It should be called when the
 * application has handled the I_WAV_OVRNG interrupt and is ready to
 * clear the corresponding status. The function expects a valid pointer
 * to an `ade9113_dev` structure, which must be properly initialized
 * before calling this function. If the provided device pointer is null,
 * the function will not perform any operation.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device.
 * This pointer must not be null and should point to a valid,
 * initialized device structure.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_clear_i_wav_ovrng_int(struct ade9113_dev *dev);

/* Get ADC_SYNC_DONE indicator. */
/***************************************************************************//**
 * @brief This function is used to check if the ADC synchronization process has
 * been completed. It should be called after the device has been properly
 * initialized and configured. The function will write the status of the
 * ADC synchronization to the provided pointer. If the `status` pointer
 * is null, the function will not perform any operation and may return an
 * error. It is important to ensure that the device is ready and
 * operational before invoking this function.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param status A pointer to a `uint8_t` variable where the ADC synchronization
 * status will be stored. Caller retains ownership and must ensure
 * it is not null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_get_adc_sync_done(struct ade9113_dev *dev, uint8_t *status);

/* Clear the ADC_SYNC_DONE int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the ADC synchronization done interrupt
 * for the ADE9113 device. It should be called when the ADC
 * synchronization process is complete and the interrupt needs to be
 * reset. Ensure that the device is properly initialized before calling
 * this function. If the `dev` parameter is invalid or null, the function
 * will handle it gracefully without causing a crash.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; the caller retains ownership of the structure.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_adc_sync_done_int(struct ade9113_dev *dev);

/* Get ISO_CLK_STBL_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to check the status of the ISO clock stability
 * error in the ADE9113 device. It should be called after the device has
 * been initialized and configured properly. The function will write the
 * status of the ISO clock stability error to the provided pointer. If
 * the `status` pointer is null, the function will not perform any
 * operation and will return an error code.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device
 * instance. Must not be null.
 * @param status A pointer to a `uint8_t` variable where the error status will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_get_iso_clk_stbl_err(struct ade9113_dev *dev, uint8_t *status);

/* Clear the ISO_CLK_STBL_ERR int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the ISO clock stability error interrupt
 * in the ADE9113 device. It should be called when the application
 * detects that the ISO clock stability error has occurred, allowing the
 * device to reset the interrupt status. It is important to ensure that
 * the device is properly initialized before calling this function, as
 * calling it on an uninitialized device may lead to undefined behavior.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device
 * instance. This pointer must not be null and should point to a
 * valid device that has been initialized.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_iso_clk_stbl_err_int(struct ade9113_dev *dev);

/* Get ISO_PHY_CRC_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the status of the ISO PHY CRC error
 * from the ADE9113 device. It should be called after the device has been
 * properly initialized and configured. The function will populate the
 * provided `status` pointer with the error status, which can indicate
 * whether a CRC error has occurred. It is important to ensure that the
 * `status` pointer is valid and points to a memory location that can
 * store the result.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device
 * instance.
 * @param status A pointer to a `uint8_t` variable where the error status will
 * be stored. Must not be null; the function will write the error
 * status to this location.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error occurred during the operation.
 ******************************************************************************/
int ade9113_get_iso_phy_crc_err(struct ade9113_dev *dev, uint8_t *status);

/* Clear the ISO_PHY_CRC_ERR int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the ISO PHY CRC error interrupt status
 * in the ADE9113 device. It should be called when the application
 * detects that the ISO PHY CRC error interrupt has been triggered,
 * allowing the system to reset the interrupt status and prepare for
 * future interrupts. It is important to ensure that the device has been
 * properly initialized before calling this function. If the `dev`
 * parameter is invalid or the device is not initialized, the function
 * may not behave as expected.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device
 * structure.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int ade9113_clear_iso_phy_crc_err_int(struct ade9113_dev *dev);

/* Get ISO_EFUSE_MEM_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to check for errors related to the ISO eFuse
 * memory in the ADE9113 device. It should be called after the device has
 * been initialized and configured. The function will update the `status`
 * pointer with the current error status, which can indicate whether an
 * error has occurred. If the `dev` parameter is null or if the device is
 * not properly initialized, the function may return an error code.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param status A pointer to a `uint8_t` variable where the error status will
 * be stored. Caller retains ownership and must ensure it points
 * to a valid memory location.
 * @return Returns 0 on success, indicating that the status has been
 * successfully retrieved. A non-zero value indicates an error occurred
 * while attempting to retrieve the status.
 ******************************************************************************/
int ade9113_get_iso_efuse_mem_err_err(struct ade9113_dev *dev, uint8_t *status);

/* Clear the ISO_EFUSE_MEM_ERR int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the ISO_EFUSE_MEM_ERR interrupt status
 * in the ADE9113 device. It should be called when the application
 * detects that the ISO_EFUSE_MEM_ERR interrupt has been triggered,
 * ensuring that the interrupt status is reset and does not cause further
 * unwanted behavior. It is important to ensure that the device is
 * properly initialized before calling this function, as calling it on an
 * uninitialized device may lead to undefined behavior.
 *
 * @param dev A pointer to a `struct ade9113_dev` representing the device
 * instance. This parameter must not be null and should point to a
 * valid device structure that has been initialized.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the status of the operation.
 ******************************************************************************/
int ade9113_clear_iso_efuse_mem_err_int(struct ade9113_dev *dev);

/* Get ISO_DIG_MOD_V2_OVF indicator. */
/***************************************************************************//**
 * @brief This function is used to check if there has been an overflow in the
 * ISO digital module V2 of the ADE9113 device. It should be called after
 * the device has been properly initialized and configured. The function
 * will write the overflow status to the provided `status` pointer, which
 * must not be null. If the device is not ready or if an error occurs
 * while retrieving the status, the function will return an error code.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param status A pointer to a `uint8_t` variable where the overflow status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9113_get_iso_dig_mod_v2_ovf(struct ade9113_dev *dev, uint8_t *status);

/* Clear the ISO_DIG_MOD_V2_OVF int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the
 * ISO_DIG_MOD_V2_OVF condition in the ADE9113 device. It should be
 * called when the application has handled the interrupt and wants to
 * reset the interrupt status. The function expects a valid pointer to an
 * `ade9113_dev` structure, which must be properly initialized before
 * calling this function. If the provided device pointer is null or
 * invalid, the behavior is undefined.
 *
 * @param dev Pointer to an `ade9113_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device
 * instance.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_iso_dig_mod_v2_ovf_int(struct ade9113_dev *dev);

/* Get ISO_DIG_MOD_V1_OVF indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the overflow status of the ISO digital
 * mode V1 from the ADE9113 device. It should be called after the device
 * has been properly initialized and configured. The function will update
 * the `status` pointer with the current overflow status, which can
 * indicate whether an overflow condition has occurred. It is important
 * to ensure that the `status` pointer is valid and points to a memory
 * location that can store the result.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param status A pointer to a `uint8_t` variable where the overflow status
 * will be stored. Must not be null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_iso_dig_mod_v1_ovf(struct ade9113_dev *dev, uint8_t *status);

/* Clear the ISO_DIG_MOD_V1_OVF int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the
 * ISO_DIG_MOD_V1_OVF condition in the ADE9113 device. It should be
 * called when the application has handled the interrupt and wants to
 * reset the interrupt status. The function must be invoked after the
 * device has been properly initialized and configured. If the `dev`
 * parameter is null, the function will not perform any operation.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This parameter must not be null, as it is required to identify the
 * device on which the operation is to be performed.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_iso_dig_mod_v1_ovf_int(struct ade9113_dev *dev);

/* Get ISO_DIG_MOD_I_OVF indicator. */
/***************************************************************************//**
 * @brief This function is used to check the overflow status of the ISO digital
 * modulator I channel. It should be called after the device has been
 * properly initialized and configured. The function will update the
 * `status` pointer with the current overflow status, which can be used
 * to determine if the I channel is experiencing overflow conditions. It
 * is important to ensure that the `status` pointer is valid and points
 * to a memory location that can store the result.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param status A pointer to a `uint8_t` variable where the overflow status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_get_iso_dig_mod_i_ovf(struct ade9113_dev *dev, uint8_t *status);

/* Clear the ISO_DIG_MOD_I_OVF int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the
 * ISO_DIG_MOD_I_OVF condition in the ADE9113 device. It should be called
 * when the interrupt has been handled and the user wants to reset the
 * interrupt status. Ensure that the `dev` parameter points to a valid
 * initialized `ade9113_dev` structure before calling this function.
 *
 * @param dev Pointer to an `ade9113_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_iso_dig_mod_i_ovf_int(struct ade9113_dev *dev);

/* Get ISO_TEST_MMR_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the status of the ISO test MMR error
 * from the ADE9113 device. It should be called after the device has been
 * properly initialized and configured. The function will write the
 * status of the ISO test MMR error into the provided `status` pointer.
 * If the `status` pointer is null, the function will not perform any
 * operation and may return an error code. It is important to check the
 * return value to ensure that the operation was successful.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This must not be null and should point to a valid initialized
 * device instance.
 * @param status A pointer to a `uint8_t` variable where the ISO test MMR error
 * status will be stored. This pointer must not be null;
 * otherwise, the function will not perform any operation.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_iso_test_mmr_err(struct ade9113_dev *dev, uint8_t *status);

/* Clear the ISO_TEST_MMR_ERR int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the ISO_TEST_MMR_ERR interrupt status
 * in the ADE9113 device. It should be called when the application
 * detects that the ISO_TEST_MMR_ERR interrupt has been triggered,
 * ensuring that the interrupt status is reset and ready for future
 * occurrences. It is important to ensure that the `dev` parameter points
 * to a valid and initialized `ade9113_dev` structure before calling this
 * function.
 *
 * @param dev A pointer to an `ade9113_dev` structure representing the device
 * instance. This pointer must not be null and should point to a
 * valid device that has been properly initialized.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_clear_iso_test_mmr_err_int(struct ade9113_dev *dev);

/* Get ISO_STATUS_RD_ECC_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the status of the ECC error indicator
 * for ISO read operations. It should be called after the device has been
 * properly initialized and configured. The function will write the
 * status of the ECC error into the provided pointer. If the `status`
 * pointer is null, the function will not perform any operation and may
 * return an error code.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This must not be null and should point to a valid initialized
 * device instance.
 * @param status A pointer to a `uint8_t` where the ECC error status will be
 * stored. This pointer must not be null; otherwise, the function
 * will not perform any operation.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_iso_status_rd_ecc_err(struct ade9113_dev *dev, uint8_t *status);

/* Get ISO_PHY_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the status of the ISO PHY error from
 * the ADE9113 device. It should be called after the device has been
 * properly initialized and configured. The function will write the
 * status of the ISO PHY error into the provided pointer. If the `status`
 * pointer is null, the function will not perform any operation and may
 * return an error code. It is important to check the return value to
 * ensure that the status was retrieved successfully.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device
 * instance. Must not be null.
 * @param status A pointer to a `uint8_t` variable where the ISO PHY error
 * status will be stored. Caller retains ownership and must ensure
 * it is not null.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_iso_phy_err(struct ade9113_dev *dev, uint8_t *status);

/* Get ISO_ECC_ERR indicator. */
/***************************************************************************//**
 * @brief This function is used to obtain the error status related to the ISO
 * ECC (Error Correction Code) from the ADE9113 device. It should be
 * called after the device has been properly initialized and configured.
 * The function will write the status of the ECC error into the provided
 * pointer. If the `status` pointer is null, the function will not
 * perform any operation and may return an error code. It is important to
 * check the return value to ensure that the status was retrieved
 * successfully.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device
 * instance. This pointer must not be null and should point to a
 * valid initialized device.
 * @param status A pointer to a `uint8_t` variable where the ECC error status
 * will be stored. This pointer must not be null; otherwise, the
 * function will not perform any operation.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error occurred during the retrieval of the ECC
 * error status.
 ******************************************************************************/
int ade9113_get_iso_ecc_err(struct ade9113_dev *dev, uint8_t *status);

/* Get CRC_DONE indicator. */
/***************************************************************************//**
 * @brief This function is used to check if the CRC (Cyclic Redundancy Check)
 * operation has been completed for the ADE9113 device. It should be
 * called after initiating a CRC operation to determine if the process
 * has finished successfully. The function expects a valid device
 * structure and will update the provided status pointer with the result.
 * If the device structure is invalid or the status pointer is null, the
 * function will handle these cases appropriately.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param status A pointer to a `uint8_t` variable where the CRC done flag
 * status will be stored. Caller retains ownership and must ensure
 * this pointer is valid.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9113_get_crc_done_flag(struct ade9113_dev *dev, uint8_t *status);

/* Clear the CRC_DONE int mask. */
/***************************************************************************//**
 * @brief This function is used to clear the CRC done interrupt for the ADE9113
 * device. It should be called when the CRC done interrupt has been
 * triggered and the interrupt needs to be acknowledged to prevent it
 * from being triggered again. The function expects a valid device
 * structure pointer, which must be initialized prior to calling this
 * function. If the provided device pointer is null, the function will
 * return an error code indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9113_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9113_clear_crc_done_int(struct ade9113_dev *dev);

/* Force background register map CRC recalculation. */
/***************************************************************************//**
 * @brief This function should be called when a CRC recalculation is needed for
 * the register map of the ADE9113 device. It is important to ensure that
 * the `dev` parameter is properly initialized and not null before
 * calling this function. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * The function does not modify any input parameters and is intended to
 * be used in scenarios where data integrity needs to be verified.
 *
 * @param dev A pointer to a `struct ade9113_dev` representing the device. This
 * pointer must not be null and should point to a valid initialized
 * device structure. If it is null, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9113_force_crc_recalculation(struct ade9113_dev *dev);

/* Get SILICON_REVISION value. */
/***************************************************************************//**
 * @brief This function is used to obtain the silicon revision of the ADE9113
 * device, which can be useful for identifying the specific version of
 * the hardware in use. It should be called after the device has been
 * properly initialized. The caller must ensure that the `silicon_rev`
 * pointer is valid and not null, as passing a null pointer will result
 * in an error. The function will read the silicon revision from the
 * device's register and store the result in the location pointed to by
 * `silicon_rev.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This structure must be properly initialized before calling the
 * function.
 * @param silicon_rev A pointer to a `uint8_t` where the silicon revision will
 * be stored. This pointer must not be null; otherwise, the
 * function will return an error.
 * @return Returns 0 on success, indicating that the silicon revision was
 * successfully read and stored. If an error occurs, such as a null
 * pointer for `silicon_rev`, it returns a negative error code.
 ******************************************************************************/
int ade9113_get_silicon_revision(struct ade9113_dev *dev, uint8_t *silicon_rev);

/* Get VERSION_PRODUCT value. */
/***************************************************************************//**
 * @brief This function is used to obtain the version product of the ADE9113
 * device, which is essential for ensuring compatibility and proper
 * operation with the specific hardware version. It must be called after
 * the device has been properly initialized. The function checks if the
 * provided pointer for the version product is valid; if it is null, an
 * error is returned. The function reads the version product register and
 * maps the returned value to a specific product identifier. If the read
 * operation fails or if the returned value does not correspond to a
 * known product, an appropriate error code is returned.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * Must not be null.
 * @param ver_product A pointer to a `uint8_t` where the version product will be
 * stored. Must not be null; if null, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or a negative error code indicating the type of
 * failure (e.g., -EINVAL for invalid input, -ENODEV for an unknown
 * product version).
 ******************************************************************************/
int ade9113_get_version_product(struct ade9113_dev *dev, uint8_t *ver_product);

/* Get wave value. */
/***************************************************************************//**
 * @brief This function is used to obtain waveform data from the ADE9113 device
 * based on the specified selection. It must be called after the device
 * has been properly initialized. The function expects a valid pointer to
 * a `struct ade9113_dev` instance and a pointer to a `uint32_t` variable
 * where the retrieved value will be stored. If the selection parameter
 * is out of range or if the value pointer is null, the function will
 * return an error code. The function performs multiple read operations
 * to construct a 24-bit value representing the waveform data.
 *
 * @param dev A pointer to the `struct ade9113_dev` instance representing the
 * device. Must not be null.
 * @param selection An enumeration value of type `enum ade9113_wav_e` that
 * specifies which waveform data to retrieve. Valid values are
 * `ADE9113_I_WAV`, `ADE9113_V1_WAV`, and `ADE9113_V2_WAV`. If
 * the value exceeds `ADE9113_V2_WAV`, an error is returned.
 * @param val A pointer to a `uint32_t` variable where the retrieved waveform
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the read operations.
 ******************************************************************************/
int ade9113_get_wav(struct ade9113_dev *dev, enum ade9113_wav_e selection,
		    uint32_t *val);

/* DRDY inerrupt enable. */
/***************************************************************************//**
 * @brief This function is used to enable the Data Ready (DRDY) interrupt for
 * the ADE9113 device. It should be called after the device has been
 * properly initialized and configured. Enabling this interrupt allows
 * the system to respond to data availability signals from the ADE9113,
 * facilitating timely data processing. If the provided `dev` pointer is
 * null, the function will return an error, ensuring that the device
 * context is valid before attempting to enable the interrupt.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This pointer must not be null, as it is required to access the
 * device's interrupt control settings. If the pointer is null, the
 * function will return an error.
 * @return Returns 0 on success, indicating that the DRDY interrupt has been
 * successfully enabled. If an error occurs, a negative value is
 * returned.
 ******************************************************************************/
int ade9113_drdy_int_enable(struct ade9113_dev *dev);

/* DRDY inerrupt disable. */
/***************************************************************************//**
 * @brief This function is used to disable the Data Ready (DRDY) interrupt for
 * the ADE9113 device. It should be called when the application no longer
 * needs to respond to data ready signals, such as during device shutdown
 * or when switching to a different mode of operation. Ensure that the
 * device has been properly initialized before calling this function. If
 * the provided `dev` pointer is null, the function will handle it
 * gracefully without causing a crash.
 *
 * @param dev A pointer to the `ade9113_dev` structure representing the device.
 * This pointer must not be null, and it is expected that the device
 * has been initialized prior to calling this function.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9113_drdy_int_disable(struct ade9113_dev *dev);

#endif // __ADE9113_H__
