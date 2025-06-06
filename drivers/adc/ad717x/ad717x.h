/**************************************************************************//**
*   @file    AD717X.h
*   @brief   AD717X header file.
*   	     Devices: AD7172-2, AD7172-4, AD7173-8, AD7175-2, AD7175-8, AD7176-2,
*            AD7177-2, AD4111, AD4112, AD4114, AD4115, AD4116
*   @author  acozma (andrei.cozma@analog.com)
*            dnechita (dan.nechita@analog.com)
*******************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
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
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*****************************************************************************/

#ifndef __AD717X_H__
#define __AD717X_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"
#include "no_os_util.h"
#include <stdbool.h>

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/* Total Number of Setups in the AD717x-AD411x family */
#define AD717x_MAX_SETUPS			8
/* Maximum number of channels in the AD717x-AD411x family */
#define AD717x_MAX_CHANNELS			16

/***************************************************************************//**
 * @brief The `ad717x_mode` enumeration defines the various modes of operation
 * for the AD717x family of analog-to-digital converters. These modes
 * include continuous and single conversion modes, as well as power
 * management modes like standby and power down. Additionally, it
 * includes modes for performing internal and system-level offset and
 * gain calibrations, allowing for precise adjustments and optimizations
 * of the ADC's performance.
 *
 * @param CONTINUOUS Continuous Mode - Default mode of operation.
 * @param SINGLE Single Mode for one-time conversion.
 * @param STANDBY Stand-by Mode to reduce power consumption.
 * @param POWER_DOWN Power Down Mode to minimize power usage.
 * @param INTERNAL_OFFSET_CALIB Internal Offset Calibration mode.
 * @param INTERNAL_GAIN_CALIB Internal Gain Calibration mode.
 * @param SYS_OFFSET_CALIB System Offset Calibration mode.
 * @param SYS_GAIN_CALIB System Gain Calibration mode.
 ******************************************************************************/
enum ad717x_mode {
	CONTINUOUS, 			/* Continuous Mode- Default */
	SINGLE, 				/* Single Mode */
	STANDBY, 				/* Stand-by Mode */
	POWER_DOWN, 			/* Power Down Mode */
	INTERNAL_OFFSET_CALIB,	/* Internal Offset Calibration*/
	INTERNAL_GAIN_CALIB, 	/* Internal Gain Calibration */
	SYS_OFFSET_CALIB, 		/* System Offset Calibraion */
	SYS_GAIN_CALIB			/* System Gain Calibration */
};

/***************************************************************************//**
 * @brief The `ad717x_reference_source` enumeration defines the types of
 * reference sources that can be used with the AD717x family of ADCs. It
 * includes options for using an external reference, an internal 2.5V
 * reference, or the AVDD - AVSS as the reference source. This
 * enumeration is used to configure the reference source for the ADC,
 * which is crucial for accurate analog-to-digital conversion.
 *
 * @param EXTERNAL_REF Represents an external reference source with a value of
 * 0x0.
 * @param INTERNAL_REF Represents an internal 2.5V reference source with a value
 * of 0x2.
 * @param AVDD_AVSS Represents the AVDD - AVSS reference source with a value of
 * 0x3.
 ******************************************************************************/
enum ad717x_reference_source {
	EXTERNAL_REF = 0x0, /* External Reference REF+/-*/
	INTERNAL_REF = 0x2,	/* Internal 2.5V Reference */
	AVDD_AVSS = 0x3		/* AVDD - AVSS */
};

/***************************************************************************//**
 * @brief The `ad717x_analog_input_pairs` enumeration defines a set of constants
 * representing various analog input pair configurations for the AD411X
 * family of devices. Each constant corresponds to a specific pair of
 * input channels or special inputs like temperature sensor and
 * reference, with unique hexadecimal values assigned to each pair. This
 * enumeration is used to configure the input channels of the ADCs in the
 * AD411X family, allowing for flexible selection of input sources.
 *
 * @param VIN0_VIN1 Represents the analog input pair VIN0 and VIN1 with a value
 * of 0x1.
 * @param VIN0_VINCOM Represents the analog input pair VIN0 and VINCOM with a
 * value of 0x10.
 * @param VIN1_VIN0 Represents the analog input pair VIN1 and VIN0 with a value
 * of 0x20.
 * @param VIN1_VINCOM Represents the analog input pair VIN1 and VINCOM with a
 * value of 0x30.
 * @param VIN2_VIN3 Represents the analog input pair VIN2 and VIN3 with a value
 * of 0x43.
 * @param VIN2_VINCOM Represents the analog input pair VIN2 and VINCOM with a
 * value of 0x50.
 * @param VIN3_VIN2 Represents the analog input pair VIN3 and VIN2 with a value
 * of 0x62.
 * @param VIN3_VINCOM Represents the analog input pair VIN3 and VINCOM with a
 * value of 0x70.
 * @param VIN4_VIN5 Represents the analog input pair VIN4 and VIN5 with a value
 * of 0x85.
 * @param VIN4_VINCOM Represents the analog input pair VIN4 and VINCOM with a
 * value of 0x90.
 * @param VIN5_VIN4 Represents the analog input pair VIN5 and VIN4 with a value
 * of 0xA4.
 * @param VIN5_VINCOM Represents the analog input pair VIN5 and VINCOM with a
 * value of 0xB0.
 * @param VIN6_VIN7 Represents the analog input pair VIN6 and VIN7 with a value
 * of 0xC7.
 * @param VIN6_VINCOM Represents the analog input pair VIN6 and VINCOM with a
 * value of 0xD0.
 * @param VIN7_VIN6 Represents the analog input pair VIN7 and VIN6 with a value
 * of 0xE6.
 * @param VIN7_VINCOM Represents the analog input pair VIN7 and VINCOM with a
 * value of 0xF0.
 * @param IIN3P_IIN3M Represents the analog input pair IIN3P and IIN3M with a
 * value of 0x18B.
 * @param IIN2P_IIN2M Represents the analog input pair IIN2P and IIN2M with a
 * value of 0x1AA.
 * @param IIN1P_IIN1M Represents the analog input pair IIN1P and IIN1M with a
 * value of 0x1C9.
 * @param IIN0P_IIN0M Represents the analog input pair IIN0P and IIN0M with a
 * value of 0x1E8.
 * @param TEMPERATURE_SENSOR Represents the temperature sensor input with a
 * value of 0x232.
 * @param REFERENCE Represents the reference input with a value of 0x2B6.
 ******************************************************************************/
enum ad717x_analog_input_pairs {
	VIN0_VIN1 = 0x1,
	VIN0_VINCOM = 0x10,
	VIN1_VIN0 = 0x20,
	VIN1_VINCOM = 0x30,
	VIN2_VIN3 = 0x43,
	VIN2_VINCOM = 0x50,
	VIN3_VIN2 = 0x62,
	VIN3_VINCOM = 0x70,
	VIN4_VIN5 = 0x85,
	VIN4_VINCOM = 0x90,
	VIN5_VIN4 = 0xA4,
	VIN5_VINCOM = 0xB0,
	VIN6_VIN7 = 0xC7,
	VIN6_VINCOM = 0xD0,
	VIN7_VIN6 = 0xE6,
	VIN7_VINCOM = 0xF0,
	IIN3P_IIN3M = 0x18B,
	IIN2P_IIN2M = 0x1AA,
	IIN1P_IIN1M = 0x1C9,
	IIN0P_IIN0M = 0x1E8,
	TEMPERATURE_SENSOR = 0x232,
	REFERENCE = 0x2B6
};

/***************************************************************************//**
 * @brief The `ad717x_analog_input` enum defines a set of constants representing
 * various analog input channels and special measurement points for the
 * AD717x family of devices. These constants are used to specify which
 * input or measurement point is being referenced in the context of
 * configuring or reading from the device. The enum includes standard
 * analog input channels (AIN0 to AIN4), as well as special inputs for
 * temperature sensors, AVDD to AVSS measurements, and reference
 * voltages.
 *
 * @param AIN0 Represents the analog input channel 0.
 * @param AIN1 Represents the analog input channel 1.
 * @param AIN2 Represents the analog input channel 2.
 * @param AIN3 Represents the analog input channel 3.
 * @param AIN4 Represents the analog input channel 4.
 * @param TEMP_SENSOR_P Represents the positive terminal of the temperature
 * sensor.
 * @param TEMP_SENSOR_M Represents the negative terminal of the temperature
 * sensor.
 * @param AVDD_AVSS_P Represents the positive terminal of the AVDD to AVSS
 * measurement.
 * @param AVDD_AVSS_M Represents the negative terminal of the AVDD to AVSS
 * measurement.
 * @param REF_P Represents the positive terminal of the reference voltage.
 * @param REF_M Represents the negative terminal of the reference voltage.
 ******************************************************************************/
enum ad717x_analog_input {
	AIN0 = 0x0,
	AIN1 = 0x1,
	AIN2 = 0x2,
	AIN3 = 0x3,
	AIN4 = 0x4,
	TEMP_SENSOR_P = 0x11,
	TEMP_SENSOR_M = 0x12,
	AVDD_AVSS_P = 0x13,
	AVDD_AVSS_M = 0x14,
	REF_P = 0x15,
	REF_M = 0x16
};

/***************************************************************************//**
 * @brief The `ad717x_analog_inputs` union is designed to represent different
 * types of analog inputs for the AD717x family of devices. It provides
 * two ways to define analog inputs: as a pair of inputs using the
 * `analog_input_pairs` enumeration, or as individual positive and
 * negative inputs using the `ainp` structure. This flexibility allows
 * for easy configuration of input channels depending on the specific
 * requirements of the application.
 *
 * @param analog_input_pairs An enumeration representing pairs of analog inputs.
 * @param ainp A structure containing positive and negative analog input
 * enumerations.
 ******************************************************************************/
union ad717x_analog_inputs {
	enum ad717x_analog_input_pairs analog_input_pairs;
	struct {
		enum ad717x_analog_input pos_analog_input;
		enum ad717x_analog_input neg_analog_input;
	} ainp;
};

/***************************************************************************//**
 * @brief The `ad717x_device_type` enumeration defines a set of constants
 * representing different device types within the AD717x and AD411x
 * family of analog-to-digital converters. Each enumerator corresponds to
 * a specific model of ADC, allowing the software to identify and handle
 * different devices appropriately. This enumeration is used in the
 * context of configuring and managing ADC devices in the AD717x driver.
 *
 * @param ID_AD4111 Represents the AD4111 device type.
 * @param ID_AD4112 Represents the AD4112 device type.
 * @param ID_AD4114 Represents the AD4114 device type.
 * @param ID_AD4115 Represents the AD4115 device type.
 * @param ID_AD4116 Represents the AD4116 device type.
 * @param ID_AD7172_2 Represents the AD7172-2 device type.
 * @param ID_AD7172_4 Represents the AD7172-4 device type.
 * @param ID_AD7173_8 Represents the AD7173-8 device type.
 * @param ID_AD7175_2 Represents the AD7175-2 device type.
 * @param ID_AD7175_8 Represents the AD7175-8 device type.
 * @param ID_AD7176_2 Represents the AD7176-2 device type.
 * @param ID_AD7177_2 Represents the AD7177-2 device type.
 ******************************************************************************/
enum ad717x_device_type {
	ID_AD4111,
	ID_AD4112,
	ID_AD4114,
	ID_AD4115,
	ID_AD4116,
	ID_AD7172_2,
	ID_AD7172_4,
	ID_AD7173_8,
	ID_AD7175_2,
	ID_AD7175_8,
	ID_AD7176_2,
	ID_AD7177_2
};

/***************************************************************************//**
 * @brief The `ad717x_channel_setup` structure is used to configure individual
 * channels in the AD717x family of ADCs. It allows the user to specify
 * whether the channel operates in bipolar or unipolar mode, and whether
 * input and reference buffers are enabled. Additionally, it defines the
 * reference voltage source for the channel, which can be external,
 * internal, or derived from the supply voltage. This setup is crucial
 * for ensuring accurate and reliable ADC conversions by tailoring the
 * channel configuration to the specific requirements of the application.
 *
 * @param bi_unipolar Indicates if the channel is set to bipolar or unipolar
 * mode.
 * @param ref_buff Specifies if the reference buffer is enabled.
 * @param input_buff Specifies if the input buffer is enabled.
 * @param ref_source Defines the source of the reference voltage for the
 * channel.
 ******************************************************************************/
struct ad717x_channel_setup {
	bool bi_unipolar;
	bool ref_buff;
	bool input_buff;
	enum ad717x_reference_source ref_source;
};

/***************************************************************************//**
 * @brief The `ad717x_enhfilt` enumeration defines various enhanced filter
 * settings for the AD717x family of devices, each with specific sample
 * rates, decibel rejection levels, and settling times. These settings
 * are used to configure the post-filtering characteristics of the
 * analog-to-digital conversion process, allowing for enhanced rejection
 * of unwanted signals and noise in the measurement data.
 *
 * @param sps27_db47_ms36p7 Represents a filter setting with 27 samples per
 * second, 47 dB rejection, and 36.7 ms settling time.
 * @param sps25_db62_ms40 Represents a filter setting with 25 samples per
 * second, 62 dB rejection, and 40 ms settling time.
 * @param sps20_db86_ms50 Represents a filter setting with 20 samples per
 * second, 86 dB rejection, and 50 ms settling time.
 * @param sps16p6_db82_ms60 Represents a filter setting with 16.6 samples per
 * second, 82 dB rejection, and 60 ms settling time.
 ******************************************************************************/
enum ad717x_enhfilt {
	sps27_db47_ms36p7 = 0x2,
	sps25_db62_ms40 = 0x3,
	sps20_db86_ms50 = 0x5,
	sps16p6_db82_ms60 = 0x6
};

/***************************************************************************//**
 * @brief The `ad717x_order` enumeration defines the order of digital filters
 * used in the AD717x family of devices. It provides two options:
 * `sinc5_sinc1` and `sinc3`, which correspond to different filter
 * configurations that can be applied to the analog-to-digital conversion
 * process. This enumeration is used to configure the digital filter
 * order in the device's filter settings.
 *
 * @param sinc5_sinc1 Represents the sinc5-sinc1 filter order with a value of
 * 0x0.
 * @param sinc3 Represents the sinc3 filter order with a value of 0x3.
 ******************************************************************************/
enum ad717x_order {
	sinc5_sinc1 = 0x0,
	sinc3 = 0x3
};

/***************************************************************************//**
 * @brief The `ad717x_odr` enumeration defines various output data rates for the
 * AD717x family of devices, each represented by a unique identifier.
 * These identifiers correspond to specific sample rates, allowing the
 * user to configure the device to operate at different speeds depending
 * on the application requirements. The enumeration provides a range of
 * data rates from 1.25 samples per second to 31250 samples per second,
 * accommodating a wide variety of use cases.
 *
 * @param sps_31250_a Represents an output data rate of 31250 samples per second
 * with a specific configuration.
 * @param sps31250_b Represents an output data rate of 31250 samples per second
 * with a different configuration.
 * @param sps_31250_c Represents an output data rate of 31250 samples per second
 * with another configuration.
 * @param sps_31250_d Represents an output data rate of 31250 samples per second
 * with yet another configuration.
 * @param sps31250_e Represents an output data rate of 31250 samples per second
 * with a further configuration.
 * @param sps_31250_f Represents an output data rate of 31250 samples per second
 * with an additional configuration.
 * @param sps_15625 Represents an output data rate of 15625 samples per second.
 * @param sps_10417 Represents an output data rate of 10417 samples per second.
 * @param sps_5208 Represents an output data rate of 5208 samples per second.
 * @param sps_2957 Represents an output data rate of 2957 samples per second.
 * @param sps_1007 Represents an output data rate of 1007 samples per second.
 * @param sps_503 Represents an output data rate of 503 samples per second.
 * @param sps_381 Represents an output data rate of 381 samples per second.
 * @param sps_200 Represents an output data rate of 200 samples per second.
 * @param sps_100 Represents an output data rate of 100 samples per second.
 * @param sps_59 Represents an output data rate of 59 samples per second.
 * @param sps_49 Represents an output data rate of 49 samples per second.
 * @param sps_20 Represents an output data rate of 20 samples per second.
 * @param sps_16 Represents an output data rate of 16 samples per second.
 * @param sps_10 Represents an output data rate of 10 samples per second.
 * @param sps_5 Represents an output data rate of 5 samples per second.
 * @param sps_2p5 Represents an output data rate of 2.5 samples per second.
 * @param sps_1p25 Represents an output data rate of 1.25 samples per second.
 ******************************************************************************/
enum ad717x_odr {
	sps_31250_a = 0x0,
	sps31250_b = 0x1,
	sps_31250_c = 0x2,
	sps_31250_d = 0x3,
	sps31250_e = 0x4,
	sps_31250_f = 0x5,
	sps_15625 = 0x6,
	sps_10417 = 0x7,
	sps_5208 = 0x8,
	sps_2957 = 0x9,
	sps_1007 = 0xA,
	sps_503 = 0xB,
	sps_381 = 0xC,
	sps_200 = 0xD,
	sps_100 = 0xE,
	sps_59 = 0xF,
	sps_49 = 0x10,
	sps_20 = 0x11,
	sps_16 = 0x12,
	sps_10 = 0x13,
	sps_5 = 0x14,
	sps_2p5 = 0x15,
	sps_1p25 = 0x16
};

/***************************************************************************//**
 * @brief The `ad717x_filtcon` structure is used to configure the filter
 * settings for the AD717x family of devices. It includes options for
 * enabling Sinc3 filter mapping and enhanced filtering, as well as
 * specifying the type of enhanced filter, the order of the digital
 * filter, and the output data rate. This structure is crucial for
 * setting up the desired filtering characteristics in the ADC's signal
 * processing chain.
 *
 * @param sinc3_map A boolean flag indicating if the Sinc3 filter mapping is
 * enabled.
 * @param enhfilten A boolean flag indicating if the enhanced filter is enabled.
 * @param enhfilt An enumeration specifying the type of enhanced filter used.
 * @param oder An enumeration specifying the order of the digital filter.
 * @param odr An enumeration specifying the output data rate.
 ******************************************************************************/
struct ad717x_filtcon {
	bool sinc3_map;
	bool enhfilten;
	enum ad717x_enhfilt enhfilt;
	enum ad717x_order oder;
	enum ad717x_odr odr;
};

/***************************************************************************//**
 * @brief The `ad717x_channel_map` structure is used to define the configuration
 * of a channel in the AD717x family of devices. It includes a boolean to
 * enable or disable the channel, a setup selection to choose the
 * configuration for the channel, and a union to specify the analog
 * inputs, which can be either a pair of inputs or individual positive
 * and negative inputs. This structure is essential for mapping and
 * configuring the channels in the AD717x series of analog-to-digital
 * converters.
 *
 * @param channel_enable Indicates whether the channel is enabled or not.
 * @param setup_sel Selects the setup configuration for the channel.
 * @param analog_inputs Specifies the analog inputs for the channel, using a
 * union of input pairs or individual inputs.
 ******************************************************************************/
struct ad717x_channel_map {
	bool channel_enable;
	uint8_t setup_sel;
	union ad717x_analog_inputs analog_inputs;
};

/***************************************************************************//**
 * @brief The `ad717x_crc_mode` is an enumeration that defines the error-
 * checking modes available for SPI communication in the AD717x family of
 * devices. It allows the user to select between disabling error
 * checking, using CRC (Cyclic Redundancy Check), or using XOR (exclusive
 * OR) for error detection during data transfers.
 *
 * @param AD717X_DISABLE Represents the mode where CRC or XOR error checking is
 * disabled.
 * @param AD717X_USE_CRC Represents the mode where CRC error checking is used.
 * @param AD717X_USE_XOR Represents the mode where XOR error checking is used.
 ******************************************************************************/
typedef enum {
	AD717X_DISABLE,
	AD717X_USE_CRC,
	AD717X_USE_XOR,
} ad717x_crc_mode;

/***************************************************************************//**
 * @brief The `ad717x_st_reg` structure is used to represent a register in the
 * AD717x family of devices. It contains fields for the register's
 * address, the value to be stored in the register, and the size of the
 * register. This structure is essential for managing the configuration
 * and operation of the AD717x devices by allowing the software to
 * interact with the device's registers efficiently.
 *
 * @param addr Stores the address of the register.
 * @param value Holds the value to be written to or read from the register.
 * @param size Indicates the size of the register in bytes.
 ******************************************************************************/
typedef struct {
	int32_t addr;
	int32_t value;
	int32_t size;
} ad717x_st_reg;

/***************************************************************************//**
 * @brief The `ad717x_dev` structure is a comprehensive representation of an
 * AD717x device, encapsulating all necessary configurations and settings
 * for its operation. It includes SPI communication details, device-
 * specific register settings, and configurations for channels, setups,
 * and filters. The structure also manages the device's operational mode
 * and error-checking mechanisms, making it integral to the AD717x driver
 * for managing and interfacing with the device.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param regs Pointer to the device's register list.
 * @param num_regs Number of registers in the device.
 * @param useCRC Specifies the CRC mode for error checking in SPI transfers.
 * @param active_device Indicates the type of active AD717x device.
 * @param ref_en Boolean flag to enable or disable the reference.
 * @param num_channels Number of channels available on the device.
 * @param setups Array of channel setup configurations.
 * @param chan_map Array mapping channels to their configurations.
 * @param pga Array of programmable gain amplifier settings for setups.
 * @param filter_configuration Array of filter configurations for setups.
 * @param mode Specifies the ADC mode of operation.
 ******************************************************************************/
typedef struct {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* Device Settings */
	ad717x_st_reg		*regs;
	uint8_t			num_regs;
	ad717x_crc_mode		useCRC;
	/* Active Device */
	enum ad717x_device_type active_device;
	/* Reference enable */
	bool ref_en;
	/* Number of channels */
	uint8_t num_channels;
	/* Setups */
	struct ad717x_channel_setup setups[AD717x_MAX_SETUPS];
	/* Channel Mapping*/
	struct ad717x_channel_map chan_map[AD717x_MAX_CHANNELS];
	/* Gain */
	uint32_t pga[AD717x_MAX_SETUPS];
	/* Filter */
	struct ad717x_filtcon filter_configuration[AD717x_MAX_SETUPS];
	/* ADC Mode */
	enum ad717x_mode mode;
} ad717x_dev;

/***************************************************************************//**
 * @brief The `ad717x_init_param` structure is used to initialize and configure
 * the AD717x family of analog-to-digital converters. It includes
 * parameters for SPI communication, device settings such as the number
 * of channels and setups, and configurations for channel mapping, gain,
 * and filtering. This structure is essential for setting up the device's
 * operational parameters and ensuring proper communication and data
 * acquisition from the AD717x series devices.
 *
 * @param spi_init Initializes the SPI communication parameters.
 * @param regs Pointer to the device's register list.
 * @param num_regs Number of registers in the device's register list.
 * @param active_device Specifies the type of active AD717x device.
 * @param ref_en Indicates if the reference is enabled.
 * @param num_channels Number of channels available on the device.
 * @param num_setups Number of setups configured for the device.
 * @param chan_map Array mapping channels to their configurations.
 * @param setups Array of channel setup configurations.
 * @param pga Array of programmable gain amplifier settings for each setup.
 * @param filter_configuration Array of filter configurations for each setup.
 * @param mode Specifies the ADC mode of operation.
 ******************************************************************************/
typedef struct {
	/* SPI */
	struct no_os_spi_init_param		spi_init;
	/* Device Settings */
	ad717x_st_reg		*regs;
	uint8_t			num_regs;
	/* Active Device */
	enum ad717x_device_type active_device;
	/* Reference Enable */
	bool ref_en;
	/* Number of Channels */
	uint8_t num_channels;
	/* Number of Setups */
	uint8_t num_setups;
	/* Channel Mapping */
	struct ad717x_channel_map chan_map[AD717x_MAX_CHANNELS];
	/* Setups */
	struct ad717x_channel_setup setups[AD717x_MAX_SETUPS];
	/* Gain */
	uint32_t pga[AD717x_MAX_SETUPS];
	/* Filter Configuration */
	struct ad717x_filtcon filter_configuration[AD717x_MAX_SETUPS];
	/* ADC Mode */
	enum ad717x_mode mode;
} ad717x_init_param;

/*****************************************************************************/
/***************** AD717X Register Definitions *******************************/
/*****************************************************************************/

/* AD717X Register Map */
#define AD717X_COMM_REG       0x00
#define AD717X_STATUS_REG     0x00
#define AD717X_ADCMODE_REG    0x01
#define AD717X_IFMODE_REG     0x02
#define AD717X_REGCHECK_REG   0x03
#define AD717X_DATA_REG       0x04
#define AD717X_GPIOCON_REG    0x06
#define AD717X_ID_REG         0x07
#define AD717X_CHMAP0_REG     0x10
#define AD717X_CHMAP1_REG     0x11
#define AD717X_CHMAP2_REG     0x12
#define AD717X_CHMAP3_REG     0x13
#define AD717X_CHMAP4_REG     0x14
#define AD717X_CHMAP5_REG     0x15
#define AD717X_CHMAP6_REG     0x16
#define AD717X_CHMAP7_REG     0x17
#define AD717X_CHMAP8_REG     0x18
#define AD717X_CHMAP9_REG     0x19
#define AD717X_CHMAP10_REG    0x1A
#define AD717X_CHMAP11_REG    0x1B
#define AD717X_CHMAP12_REG    0x1C
#define AD717X_CHMAP13_REG    0x1D
#define AD717X_CHMAP14_REG    0x1E
#define AD717X_CHMAP15_REG    0x1F
#define AD717X_SETUPCON0_REG  0x20
#define AD717X_SETUPCON1_REG  0x21
#define AD717X_SETUPCON2_REG  0x22
#define AD717X_SETUPCON3_REG  0x23
#define AD717X_SETUPCON4_REG  0x24
#define AD717X_SETUPCON5_REG  0x25
#define AD717X_SETUPCON6_REG  0x26
#define AD717X_SETUPCON7_REG  0x27
#define AD717X_FILTCON0_REG   0x28
#define AD717X_FILTCON1_REG   0x29
#define AD717X_FILTCON2_REG   0x2A
#define AD717X_FILTCON3_REG   0x2B
#define AD717X_FILTCON4_REG   0x2C
#define AD717X_FILTCON5_REG   0x2D
#define AD717X_FILTCON6_REG   0x2E
#define AD717X_FILTCON7_REG   0x2F
#define AD717X_OFFSET0_REG    0x30
#define AD717X_OFFSET1_REG    0x31
#define AD717X_OFFSET2_REG    0x32
#define AD717X_OFFSET3_REG    0x33
#define AD717X_OFFSET4_REG    0x34
#define AD717X_OFFSET5_REG    0x35
#define AD717X_OFFSET6_REG    0x36
#define AD717X_OFFSET7_REG    0x37
#define AD717X_GAIN0_REG      0x38
#define AD717X_GAIN1_REG      0x39
#define AD717X_GAIN2_REG      0x3A
#define AD717X_GAIN3_REG      0x3B
#define AD717X_GAIN4_REG      0x3C
#define AD717X_GAIN5_REG      0x3D
#define AD717X_GAIN6_REG      0x3E
#define AD717X_GAIN7_REG      0x3F

/* Communication Register bits */
#define AD717X_COMM_REG_WEN    (0 << 7)
#define AD717X_COMM_REG_WR     (0 << 6)
#define AD717X_COMM_REG_RD     (1 << 6)
#define AD717X_COMM_REG_RA(x)  ((x) & 0x3F)

/* Status Register bits */
#define AD717X_STATUS_REG_RDY      (1 << 7)
#define AD717X_STATUS_REG_ADC_ERR  (1 << 6)
#define AD717X_STATUS_REG_CRC_ERR  (1 << 5)
#define AD717X_STATUS_REG_REG_ERR  (1 << 4)
#define AD717X_STATUS_REG_CH(x)    ((x) & 0x0F)

/* ADC Mode Register bits */
#define AD717X_ADCMODE_REG_REF_EN     (1 << 15)
#define AD717X_ADCMODE_SING_CYC       (1 << 13)
#define AD717X_ADCMODE_REG_DELAY(x)   (((x) & 0x7) << 8)
#define AD717X_ADCMODE_REG_MODE(x)    (((x) & 0x7) << 4)
#define AD717X_ADCMODE_REG_CLKSEL(x)  (((x) & 0x3) << 2)

/* ADC Mode Register additional bits for AD7172-2, AD7172-4, AD4111 and AD4112 */
#define AD717X_ADCMODE_REG_HIDE_DELAY   (1 << 14)

/* Interface Mode Register bits */
#define AD717X_IFMODE_REG_ALT_SYNC      (1 << 12)
#define AD717X_IFMODE_REG_IOSTRENGTH    (1 << 11)
#define AD717X_IFMODE_REG_DOUT_RESET    (1 << 8)
#define AD717X_IFMODE_REG_CONT_READ     (1 << 7)
#define AD717X_IFMODE_REG_DATA_STAT     (1 << 6)
#define AD717X_IFMODE_REG_REG_CHECK     (1 << 5)
#define AD717X_IFMODE_REG_XOR_EN        (0x01 << 2)
#define AD717X_IFMODE_REG_CRC_EN        (0x02 << 2)
#define AD717X_IFMODE_REG_XOR_STAT(x)   (((x) & AD717X_IFMODE_REG_XOR_EN) == AD717X_IFMODE_REG_XOR_EN)
#define AD717X_IFMODE_REG_CRC_STAT(x)   (((x) & AD717X_IFMODE_REG_CRC_EN) == AD717X_IFMODE_REG_CRC_EN)
#define AD717X_IFMODE_REG_DATA_WL16     (1 << 0)

/* Interface Mode Register additional bits for AD717x family, not for AD411x */
#define AD717X_IFMODE_REG_HIDE_DELAY    (1 << 10)

/* GPIO Configuration Register bits */
#define AD717X_GPIOCON_REG_MUX_IO      (1 << 12)
#define AD717X_GPIOCON_REG_SYNC_EN     (1 << 11)
#define AD717X_GPIOCON_REG_ERR_EN(x)   (((x) & 0x3) << 9)
#define AD717X_GPIOCON_REG_ERR_DAT     (1 << 8)
#define AD717X_GPIOCON_REG_IP_EN1      (1 << 5)
#define AD717X_GPIOCON_REG_IP_EN0      (1 << 4)
#define AD717X_GPIOCON_REG_OP_EN1      (1 << 3)
#define AD717X_GPIOCON_REG_OP_EN0      (1 << 2)
#define AD717X_GPIOCON_REG_DATA1       (1 << 1)
#define AD717X_GPIOCON_REG_DATA0       (1 << 0)

/* GPIO Configuration Register additional bits for AD7172-4, AD7173-8 */
#define AD717X_GPIOCON_REG_GP_DATA3    (1 << 7)
#define AD717X_GPIOCON_REG_GP_DATA2    (1 << 6)
#define AD717X_GPIOCON_REG_GP_DATA1    (1 << 1)
#define AD717X_GPIOCON_REG_GP_DATA0    (1 << 0)

/* GPIO Configuration Register additional bits for AD7173-8 */
#define AD717X_GPIOCON_REG_PDSW        (1 << 14)
#define AD717X_GPIOCON_REG_OP_EN2_3    (1 << 13)

/* GPIO Configuration Register additional bits for AD4111, AD4112, AD4114, AD4115 */
#define AD4111_GPIOCON_REG_OP_EN0_1    (1 << 13)
#define AD4111_GPIOCON_REG_DATA1       (1 << 7)
#define AD4111_GPIOCON_REG_DATA0       (1 << 6)

/* GPIO Configuration Register additional bits for AD4116 */
#define AD4116_GPIOCON_REG_OP_EN2_3    (1 << 13)
#define AD4116_GPIOCON_REG_DATA3       (1 << 7)
#define AD4116_GPIOCON_REG_DATA2       (1 << 6)

/* GPIO Configuration Register additional bits for AD4111 */
#define AD4111_GPIOCON_REG_OW_EN       (1 << 12)

/* Channel Map Register 0-3 bits */
#define AD717X_CHMAP_REG_CH_EN         (1 << 15)
#define AD717X_CHMAP_REG_SETUP_SEL(x)  (((x) & 0x7) << 12)
#define AD717X_CHMAP_REG_AINPOS(x)     (((x) & 0x1F) << 5)
#define AD717X_CHMAP_REG_AINNEG(x)     (((x) & 0x1F) << 0)

/* Channel Map Register additional bits for AD4111, AD4112, AD4114, AD4115, AD4116 */
#define AD4111_CHMAP_REG_INPUT(x)      (((x) & 0x3FF) << 0)

/* Setup Configuration Register 0-3 bits */
#define AD717X_SETUP_CONF_REG_BI_UNIPOLAR  (1 << 12)
#define AD717X_SETUP_CONF_REG_REF_SEL(x)   (((x) & 0x3) << 4)

/* Setup Configuration Register additional bits for AD7173-8 */
#define AD717X_SETUP_CONF_REG_REF_BUF(x)  (((x) & 0x3) << 10)
#define AD717X_SETUP_CONF_REG_AIN_BUF(x)  (((x) & 0x3) << 8)
#define AD717X_SETUP_CONF_REG_BURNOUT_EN  (1 << 7)
#define AD717X_SETUP_CONF_REG_BUFCHOPMAX  (1 << 6)

/* Setup Configuration Register additional bits for AD7172-2, AD7172-4, AD7175-2 */
#define AD717X_SETUP_CONF_REG_REFBUF_P    (1 << 11)
#define AD717X_SETUP_CONF_REG_REFBUF_N    (1 << 10)
#define AD717X_SETUP_CONF_REG_AINBUF_P    (1 << 9)
#define AD717X_SETUP_CONF_REG_AINBUF_N    (1 << 8)

/* Setup Configuration Register additional bits for AD4111, AD4112, AD4114, AD4115, AD4116 */
#define AD4111_SETUP_CONF_REG_REFPOS_BUF   (1 << 11)
#define AD4111_SETUP_CONF_REG_REFNEG_BUF   (1 << 10)
#define AD4111_SETUP_CONF_REG_AIN_BUF(x)   (((x) & 0x3) << 8)
#define AD4111_SETUP_CONF_REG_BUFCHOPMAX   (1 << 6)

/* Filter Configuration Register 0-3 bits */
#define AD717X_FILT_CONF_REG_SINC3_MAP    (1 << 15)
#define AD717X_FILT_CONF_REG_ENHFILTEN    (1 << 11)
#define AD717X_FILT_CONF_REG_ENHFILT(x)   (((x) & 0x7) << 8)
#define AD717X_FILT_CONF_REG_ORDER(x)     (((x) & 0x3) << 5)
#define AD717X_FILT_CONF_REG_ODR(x)       (((x) & 0x1F) << 0)

/* ID register mask for relevant bits */
#define AD717X_ID_REG_MASK	  0xFFF0
/* AD7172-2 ID */
#define AD7172_2_ID_REG_VALUE 0x00D0
/* AD7172-4 ID */
#define AD7172_4_ID_REG_VALUE 0x2050
/* AD7173-8 ID */
#define AD7173_8_ID_REG_VALUE 0x30D0
/* AD7175-2 ID */
#define AD7175_2_ID_REG_VALUE 0x0CD0
/* AD7175-8 ID */
#define AD7175_8_ID_REG_VALUE 0x3CD0
/* AD7176-2 ID */
#define AD7176_2_ID_REG_VALUE 0x0C90
/* AD7177-2 ID */
#define AD7177_2_ID_REG_VALUE 0x4FD0
/* AD4111, AD4112 IDs */
#define AD411X_ID_REG_VALUE   0x30D0
/* AD4114, AD4115 IDs */
#define AD4114_5_ID_REG_VALUE   0x31D0
/* AD4116 ID */
#define AD4116_ID_REG_VALUE   0x34D0

/*****************************************************************************/
/******************* AD717X Constants ****************************************/
/*****************************************************************************/
#define AD717X_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */
/* Timeout for ADC Conversion */
#define AD717X_CONV_TIMEOUT			10000

#define AD717x_CHANNEL_INPUT_MASK			NO_OS_GENMASK(9,0)
#define AD717X_CHMAP_REG_SETUP_SEL_MSK  	NO_OS_GENMASK(14,12)
#define AD717X_CHMAP_REG_AINPOS_MSK    		NO_OS_GENMASK(9,5)
#define AD717X_CHMAP_REG_AINNEG_MSK    		NO_OS_GENMASK(4,0)
#define AD717X_ADCMODE_REG_MODE_MSK   		NO_OS_GENMASK(6,4)
#define AD717X_SETUP_CONF_REG_REF_SEL_MSK	NO_OS_GENMASK(5,4)
#define AD717x_ODR_MSK				NO_OS_GENMASK(4,0)

/*****************************************************************************/
/************************ Functions Declarations *****************************/
/*****************************************************************************/

/***************************************************************************//**
 * @brief This function is used to obtain a pointer to a specific register
 * within the AD717X device based on the provided register address. It is
 * essential to ensure that the device and its register list are properly
 * initialized before calling this function. The function will return a
 * null pointer if the device or its register list is not initialized, or
 * if the specified register address does not exist within the device's
 * register list. This function is useful for accessing and manipulating
 * specific registers of the device.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null and must have a valid register list
 * initialized.
 * @param reg_address The address of the register to retrieve. It should be a
 * valid register address within the device's register list.
 * @return Returns a pointer to the ad717x_st_reg structure corresponding to the
 * specified register address, or null if the register is not found or
 * the device is not properly initialized.
 ******************************************************************************/
/***************************************************************************//**
 * @brief The `AD717X_GetReg` function retrieves a pointer to a register
 * structure that corresponds to a specified register address for a given
 * AD717x device. It is used to access the register information, such as
 * address, value, and size, for the specified register address within
 * the device's register map.
 *
 * @details This function is used to obtain a reference to a specific register's
 * data structure within an AD717x device, facilitating register read
 * or write operations.
 ******************************************************************************/
ad717x_st_reg *AD717X_GetReg(ad717x_dev *device,
			     uint8_t reg_address);

/***************************************************************************//**
 * @brief This function is used to read the value of a specific register from an
 * AD717X device. It requires a valid device structure and a register
 * address to function correctly. The function checks for the validity of
 * the device and the register address before attempting to read. It
 * supports error checking through CRC or XOR if enabled in the device
 * configuration. The function should be called when the user needs to
 * retrieve the current value of a register, and it returns an error code
 * if the operation fails.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null. The function returns INVALID_VAL if this
 * parameter is null.
 * @param addr The address of the register to be read. It must correspond to a
 * valid register address within the device's register map. If the
 * address does not match a valid register, the function returns
 * INVALID_VAL.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error, such as INVALID_VAL for invalid parameters
 * or COMM_ERR for communication errors.
 ******************************************************************************/
int32_t AD717X_ReadRegister(ad717x_dev *device,
			    uint8_t addr);

/***************************************************************************//**
 * @brief This function is used to write a value to a specific register on an
 * AD717X device. It requires a valid device structure and a register
 * address. The function constructs a command word and fills a buffer
 * with the register value to be written. If CRC is enabled on the
 * device, a CRC checksum is computed and appended to the buffer. The
 * function then performs an SPI write operation to send the data to the
 * device. It is important to ensure that the device structure is
 * properly initialized and that the register address corresponds to a
 * valid register on the device. The function returns an error code if
 * the device structure is null or if the register address is invalid.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param addr The address of the register to write to. Must correspond to a
 * valid register on the device.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t AD717X_WriteRegister(ad717x_dev *device,
			     uint8_t);

/***************************************************************************//**
 * @brief Use this function to reset the AD717X device to its default state.
 * This is typically done to ensure the device is in a known state before
 * starting configuration or data acquisition. The function must be
 * called with a valid device structure that has been properly
 * initialized. If the device parameter is null, the function will return
 * an error code indicating invalid input.
 *
 * @param device A pointer to an ad717x_dev structure representing the device to
 * be reset. Must not be null. The caller retains ownership of the
 * memory.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the reset operation.
 ******************************************************************************/
int32_t AD717X_Reset(ad717x_dev *device);

/***************************************************************************//**
 * @brief This function is used to wait until the ADC device is ready to provide
 * a new conversion result, which is indicated by the RDY bit in the
 * status register. It should be called when a new conversion result is
 * expected, and the caller must specify a timeout to prevent indefinite
 * waiting. The function will return immediately if the device is already
 * ready, or it will wait until the device becomes ready or the timeout
 * expires. It is important to ensure that the device and its registers
 * are properly initialized before calling this function.
 *
 * @param device A pointer to an ad717x_dev structure representing the ADC
 * device. Must not be null, and the device's registers must be
 * initialized. If invalid, the function returns an error.
 * @param timeout The maximum number of iterations to wait for the device to
 * become ready. Must be a positive integer. If the timeout is
 * reached without the device becoming ready, the function
 * returns a timeout error.
 * @return Returns 0 if the device becomes ready before the timeout expires, or
 * a negative error code if an error occurs or the timeout is reached.
 ******************************************************************************/
int32_t AD717X_WaitForReady(ad717x_dev *device,
			    uint32_t timeout);

/***************************************************************************//**
 * @brief This function retrieves the latest conversion result from the
 * specified AD717X device and stores it in the provided memory location.
 * It should be called when a new conversion result is expected,
 * typically after ensuring the device is ready for data retrieval. The
 * function requires a valid device structure with initialized registers.
 * If the device or its registers are not properly initialized, the
 * function will return an error. The function also updates the data
 * register length based on the device configuration and options.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null and must have initialized registers. The
 * caller retains ownership.
 * @param pData A pointer to an int32_t where the conversion result will be
 * stored. Must not be null. The function writes the conversion
 * result to this location.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A non-zero value indicates an error, such as invalid input
 * or device communication failure.
 ******************************************************************************/
int32_t AD717X_ReadData(ad717x_dev *device,
			int32_t* pData);

/***************************************************************************//**
 * @brief This function calculates the size of the data register that needs to
 * be read from the AD717X device, taking into account the current
 * interface mode and device ID. It should be called when configuring the
 * device to ensure the correct number of bytes are read during data
 * acquisition. The function assumes that the device has been properly
 * initialized and configured with the correct register settings. It does
 * not handle invalid device pointers and expects the device structure to
 * be correctly populated.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null and should be properly initialized with the
 * device's register settings.
 * @return Returns 0 on successful computation of the data register size. The
 * size is updated in the device's data register structure.
 ******************************************************************************/
int32_t AD717X_ComputeDataregSize(ad717x_dev *device);

/***************************************************************************//**
 * @brief Use this function to calculate the CRC-8 checksum for a given data
 * buffer, which is useful for error-checking purposes in data
 * communication. The function processes each byte of the buffer and
 * applies a polynomial representation to compute the checksum. It is
 * important to ensure that the buffer pointer is valid and that the
 * buffer size accurately reflects the number of bytes to be processed.
 * This function does not modify the input buffer and returns the
 * computed CRC-8 value.
 *
 * @param pBuf A pointer to the data buffer for which the CRC-8 checksum is to
 * be computed. Must not be null, and the caller retains ownership
 * of the buffer.
 * @param bufSize The size of the data buffer in bytes. Must be a non-zero
 * value, as a zero size would result in no computation.
 * @return Returns the computed CRC-8 checksum as an 8-bit unsigned integer.
 ******************************************************************************/
uint8_t AD717X_ComputeCRC8(uint8_t* pBuf,
			   uint8_t bufSize);

/***************************************************************************//**
 * @brief This function calculates the XOR checksum of a given data buffer,
 * which can be used for error detection in data transmission or storage.
 * It processes each byte in the buffer, applying the XOR operation
 * cumulatively. This function is useful when a simple checksum is needed
 * to verify data integrity. Ensure that the buffer pointer is valid and
 * that the buffer size accurately reflects the number of bytes to
 * process.
 *
 * @param pBuf A pointer to the buffer containing the data to be processed. Must
 * not be null, and the caller retains ownership of the buffer.
 * @param bufSize The number of bytes in the buffer to process. Must be a non-
 * negative integer. If zero, the function returns zero as the
 * checksum.
 * @return Returns an 8-bit unsigned integer representing the computed XOR
 * checksum of the buffer.
 ******************************************************************************/
uint8_t AD717X_ComputeXOR8(uint8_t * pBuf,
			   uint8_t bufSize);

/***************************************************************************//**
 * @brief This function updates the CRC setting of the specified AD717X device
 * based on the current configuration of the interface mode register. It
 * should be called to ensure the device's CRC setting is correctly
 * configured after initialization or any configuration changes. The
 * function requires a valid device structure with initialized registers.
 * If the device or its registers are not properly initialized, the
 * function will return an error.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * This must not be null and must have its 'regs' field properly
 * initialized. If these conditions are not met, the function
 * returns an error.
 * @return Returns 0 on success, or INVALID_VAL if the input is invalid or the
 * required register cannot be accessed.
 ******************************************************************************/
int32_t AD717X_UpdateCRCSetting(ad717x_dev *device);

/***************************************************************************//**
 * @brief This function sets up the AD717X device by allocating necessary
 * resources, initializing SPI communication, and configuring the device
 * registers according to the provided initialization parameters. It must
 * be called before any other operations on the device to ensure proper
 * setup. The function handles various configurations such as ADC mode,
 * channel mapping, and filter settings. It returns an error code if
 * initialization fails at any step, ensuring that the device is not left
 * in an inconsistent state.
 *
 * @param device A pointer to a pointer of type ad717x_dev. This will be
 * allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type ad717x_init_param containing
 * initialization parameters such as SPI settings, register
 * configurations, and device-specific settings. All fields
 * must be correctly populated before calling the function.
 * @return Returns an int32_t value indicating success (0) or an error code (<0)
 * if initialization fails at any step.
 ******************************************************************************/
int32_t AD717X_Init(ad717x_dev **device,
		    ad717x_init_param init_param);

/***************************************************************************//**
 * @brief This function is used to release the resources associated with an
 * AD717X device that were previously allocated during initialization. It
 * should be called when the device is no longer needed to ensure proper
 * cleanup and to prevent memory leaks. The function handles the
 * deallocation of the SPI descriptor and the device structure itself. It
 * is important to ensure that the device pointer is valid and was
 * successfully initialized before calling this function.
 *
 * @param dev A pointer to an ad717x_dev structure representing the device to be
 * removed. Must not be null and should point to a valid, initialized
 * device structure. The function will handle invalid pointers by
 * potentially causing undefined behavior.
 * @return Returns an int32_t indicating the success or failure of the SPI
 * resource removal. A non-zero return value indicates an error occurred
 * during the SPI removal process.
 ******************************************************************************/
int32_t AD717X_remove(ad717x_dev *dev);

/* Enable/Disable Channels */
/***************************************************************************//**
 * @brief This function is used to enable or disable a specific channel on an
 * AD717x device. It should be called when you need to change the status
 * of a channel, either to activate it for data acquisition or to
 * deactivate it to save power or resources. The function requires a
 * valid device pointer and a channel ID within the supported range. It
 * modifies the channel's status based on the provided boolean flag.
 * Ensure the device is properly initialized before calling this
 * function.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null. The function returns -EINVAL if this
 * parameter is null.
 * @param channel_id An unsigned 8-bit integer representing the channel ID to be
 * modified. It should be within the valid range of channels
 * supported by the device.
 * @param channel_status A boolean value indicating the desired status of the
 * channel. 'true' to enable the channel, 'false' to
 * disable it.
 * @return Returns 0 on success. Returns a negative error code if the device
 * pointer is null or if there is an error writing to the register.
 ******************************************************************************/
int ad717x_set_channel_status(ad717x_dev *device, uint8_t channel_id,
			      bool channel_status);
/* Set ADC Mode */
/***************************************************************************//**
 * @brief This function configures the ADC mode of the specified AD717x device
 * to the desired mode. It should be called when you need to change the
 * operational mode of the ADC, such as switching between continuous and
 * single conversion modes, or entering calibration modes. The function
 * requires a valid device pointer and a mode from the predefined
 * `ad717x_mode` enumeration. It returns an error code if the device
 * pointer is null or if the mode cannot be set, ensuring that the device
 * is in a valid state before proceeding.
 *
 * @param device A pointer to an `ad717x_dev` structure representing the device
 * to configure. Must not be null. The function returns an error
 * if this parameter is invalid.
 * @param adc_mode An `enum ad717x_mode` value specifying the desired ADC mode.
 * Valid modes include continuous, single, standby, power down,
 * and various calibration modes.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the mode cannot be set.
 ******************************************************************************/
int ad717x_set_adc_mode(ad717x_dev *device, enum ad717x_mode mode);

/* Configure Analog inputs to channel */
/***************************************************************************//**
 * @brief This function is used to configure a specific channel on an AD717x
 * device to connect to a given analog input. It must be called with a
 * valid device pointer and a channel ID within the range of available
 * channels for the device. The function supports different device types,
 * and the behavior varies slightly depending on the active device type.
 * It is essential to ensure that the device is properly initialized
 * before calling this function. The function returns an error code if
 * the device pointer is null, the channel ID is invalid, or if there is
 * a failure in writing to the device register.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param channel_id An unsigned 8-bit integer representing the channel ID to
 * configure. Must be within the valid range of channels for
 * the device.
 * @param analog_input A union ad717x_analog_inputs specifying the analog input
 * configuration. The specific fields used depend on the
 * active device type.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters or operation failure.
 ******************************************************************************/
int ad717x_connect_analog_input(ad717x_dev *device, uint8_t channel_id,
				union ad717x_analog_inputs analog_input);

/* Assign setup to channel */
/***************************************************************************//**
 * @brief This function is used to assign a specific setup configuration to a
 * designated channel on an AD717x device. It is essential to ensure that
 * the `device` parameter is a valid, initialized pointer to an
 * `ad717x_dev` structure before calling this function. The function
 * modifies the channel's register to reflect the new setup configuration
 * and updates the device's internal channel map. It should be called
 * when you need to change the setup configuration for a channel, such as
 * when configuring different measurement parameters for different
 * channels. The function returns an error code if the device pointer is
 * null or if the channel register cannot be accessed or written to.
 *
 * @param device A pointer to an `ad717x_dev` structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param channel_id An unsigned 8-bit integer representing the channel ID.
 * Valid range is from 0 to AD717x_MAX_CHANNELS - 1.
 * @param setup An unsigned 8-bit integer representing the setup configuration
 * to assign. Valid range is from 0 to AD717x_MAX_SETUPS - 1.
 * @return Returns 0 on success, or a negative error code on failure (e.g.,
 * -EINVAL for invalid parameters).
 ******************************************************************************/
int ad717x_assign_setup(ad717x_dev *device, uint8_t channel_id,
			uint8_t setup);

/* Assign polarity to setup*/
/***************************************************************************//**
 * @brief This function configures the polarity setting for a given setup on an
 * AD717x device, allowing the user to choose between bipolar and
 * unipolar operation. It should be called when the device is initialized
 * and before starting any data acquisition that depends on the setup's
 * polarity. The function requires a valid device pointer and a setup ID
 * within the allowed range. If the device pointer is null or the setup
 * ID is invalid, the function returns an error code. The function
 * modifies the device's internal setup configuration to reflect the new
 * polarity setting.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null. The function returns -EINVAL if this
 * parameter is null.
 * @param bipolar A boolean value indicating the desired polarity mode. True for
 * bipolar mode, false for unipolar mode.
 * @param setup_id An unsigned 8-bit integer representing the setup ID to
 * configure. Must be within the valid range of setup IDs for
 * the device. If the setup ID is invalid, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or -EINVAL on error (e.g., invalid device
 * pointer or setup ID).
 ******************************************************************************/
int ad717x_set_polarity(ad717x_dev* device, bool bipolar,
			uint8_t setup_id);

/* Assign reference source to setup */
/***************************************************************************//**
 * @brief This function configures the reference source for a specific setup in
 * an AD717x device. It should be called when you need to change the
 * reference source for a particular setup, which is identified by the
 * setup_id. The function requires a valid device pointer and a valid
 * setup_id within the range of available setups. If the reference source
 * is set to INTERNAL_REF, the function also enables the internal
 * reference. The function returns an error code if the device pointer is
 * null or if any register operations fail.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ref_source An enum value of type ad717x_reference_source indicating
 * the desired reference source. Valid values are
 * EXTERNAL_REF, INTERNAL_REF, and AVDD_AVSS.
 * @param setup_id An unsigned 8-bit integer representing the setup identifier.
 * Must be within the range of available setups for the device.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input or register operation failure.
 ******************************************************************************/
int ad717x_set_reference_source(ad717x_dev* device,
				enum ad717x_reference_source ref_source, uint8_t setup_id);

/* Enable/Disable input and reference buffers to setup */
int ad717x_enable_buffers(ad717x_dev* device, bool inbuf_en,
			  bool refbuf_en, uint8_t setup_id);

/* Perform single conversion and read sample */
/***************************************************************************//**
 * @brief This function is used to perform a single analog-to-digital conversion
 * on a specified channel of an AD717x device and retrieve the raw
 * conversion data. It is suitable for applications where single-shot
 * measurements are required. The function must be called with a valid
 * device structure and a channel ID that is within the range of
 * available channels for the device. The function will enable the
 * specified channel, set the ADC to single conversion mode, wait for the
 * conversion to complete, and then read the conversion result. After
 * reading, the channel is disabled. The function returns an error code
 * if any step fails, ensuring that the caller can handle errors
 * appropriately.
 *
 * @param device A pointer to an ad717x_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param id The ID of the channel to perform the conversion on. Must be a valid
 * channel ID within the range supported by the device.
 * @param adc_raw_data A pointer to an int32_t where the raw ADC conversion
 * result will be stored. Must not be null. The function
 * writes the conversion result to this location.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int ad717x_single_read(ad717x_dev* device, uint8_t id,
		       int32_t *adc_raw_data);

/* Configure device ODR */
/***************************************************************************//**
 * @brief This function sets the output data rate (ODR) for a specific filter
 * configuration register on an AD717x device. It should be used when you
 * need to adjust the data sampling rate for a particular setup. The
 * function requires a valid device structure and a filter configuration
 * register identifier. It modifies the ODR bits in the specified
 * register and writes the updated configuration back to the device.
 * Ensure that the device is properly initialized before calling this
 * function. If the specified filter configuration register is invalid,
 * the function returns an error code.
 *
 * @param dev A pointer to an ad717x_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param filtcon_id An identifier for the filter configuration register to be
 * modified. Valid values depend on the device's supported
 * filter configuration registers.
 * @param odr_sel The desired output data rate selection. Must be a valid ODR
 * value as defined by the device's specifications.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad717x_configure_device_odr(ad717x_dev *dev, uint8_t filtcon_id,
				    uint8_t odr_sel);

#endif /* __AD717X_H__ */
