/***************************************************************************//**
 *   @file   ad7768.h
 *   @brief  Header file of AD7768 Driver.
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
#ifndef AD7768_H_
#define AD7768_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD7768_REG_CH_STANDBY				0x00
#define AD7768_REG_CH_MODE_A				0x01
#define AD7768_REG_CH_MODE_B				0x02
#define AD7768_REG_CH_MODE_SEL				0x03
#define AD7768_REG_PWR_MODE				0x04
#define AD7768_REG_GENERAL_CFG				0x05
#define AD7768_REG_DATA_CTRL				0x06
#define AD7768_REG_INTERFACE_CFG			0x07
#define AD7768_REG_BIST_CTRL				0x08
#define AD7768_REG_DEV_STATUS				0x09
#define AD7768_REG_REV_ID				0x0A
#define AD7768_REG_DEV_ID_MSB				0x0B
#define AD7768_REG_DEV_ID_LSB				0x0C
#define AD7768_REG_SW_REV_ID				0x0D
#define AD7768_REG_GPIO_CTRL				0x0E
#define AD7768_REG_GPIO_WR_DATA				0x0F
#define AD7768_REG_GPIO_RD_DATA				0x10
#define AD7768_REG_PRECHARGE_BUF_1			0x11
#define AD7768_REG_PRECHARGE_BUF_2			0x12
#define AD7768_REG_POS_REF_BUF				0x13
#define AD7768_REG_NEG_REF_BUF				0x14
#define AD7768_REG_CH_OFFSET_1(ch)			(0x1E + (ch) * 3)
#define AD7768_REG_CH_OFFSET_2(ch)			(0x1F + (ch) * 3)
#define AD7768_REG_CH_OFFSET_3(ch)			(0x20 + (ch) * 3)
#define AD7768_REG_CH_GAIN_1(ch)			(0x36 + (ch) * 3)
#define AD7768_REG_CH_GAIN_2(ch)			(0x37 + (ch) * 3)
#define AD7768_REG_CH_GAIN_3(ch)			(0x38 + (ch) * 3)
#define AD7768_REG_CH_SYNC_OFFSET(ch)			(0x4E + (ch) * 3)
#define AD7768_REG_DIAG_METER_RX			0x56
#define AD7768_REG_DIAG_CTRL				0x57
#define AD7768_REG_DIAG_MOD_DELAY_CTRL			0x58
#define AD7768_REG_DIAG_CHOP_CTRL			0x59

/* AD7768_REG_CH_STANDBY */
#define AD7768_CH_STANDBY(x)				(1 << (x))

/* AD7768_REG_CH_MODE_x */
#define AD7768_CH_MODE_FILTER_TYPE			(1 << 3)
#define AD7768_CH_MODE_DEC_RATE_MSK			NO_OS_GENMASK(2, 0)
#define AD7768_CH_MODE_DEC_RATE(x)			(((x) & 0x7) << 0)

/* AD7768_REG_CH_MODE_SEL */
#define AD7768_CH_MODE(x)				(1 << (x))

/* AD7768_REG_PWR_MODE */
#define AD7768_PWR_MODE_POWER_MODE_MSK			NO_OS_GENMASK(5, 4)
#define AD7768_PWR_MODE_SLEEP_MODE			(1 << 7)
#define AD7768_PWR_MODE_POWER_MODE(x)			(((x) & 0x3) << 4)
#define AD7768_PWR_MODE_LVDS_ENABLE			(1 << 3)
#define AD7768_PWR_MODE_MCLK_DIV_MSK			NO_OS_GENMASK(1, 0)
#define AD7768_PWR_MODE_MCLK_DIV(x)			(((x) & 0x3) << 0)
#define ad7768_map_power_mode_to_regval(x)		((x) ? ((x) + 1) : 0)

/* AD7768_REG_DATA_CTRL */
#define AD7768_DATA_CTRL_SPI_SYNC			(1 << 7)
#define AD7768_DATA_CTRL_SINGLE_SHOT_EN			(1 << 4)
#define AD7768_DATA_CTRL_SPI_RESET(x)			(((x) & 0x3) << 0)
#define AD7768_DATA_CONTROL_SPI_SYNC_MSK		NO_OS_BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC			NO_OS_BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC_CLEAR		0

/* AD7768_REG_INTERFACE_CFG */
#define AD7768_INTERFACE_CFG_CRC_SEL(x)			(((x) & 0x3) << 2)
#define AD7768_INTERFACE_CFG_DCLK_DIV(x)		(((x) & 0x3) << 0)
#define AD7768_INTERFACE_CFG_DCLK_DIV_MSK		NO_OS_GENMASK(1, 0)
#define AD7768_INTERFACE_CFG_DCLK_DIV_MODE(x)		(4 - no_os_find_first_set_bit(x))
#define AD7768_MAX_DCLK_DIV				8

#define AD7768_RESOLUTION				24
#define AD7768_SAMPLE_SIZE				32
#define AD7768_MAX_FREQ_PER_MODE			6
#define AD7768_NUM_CHANNELS				8

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad7768_sleep_mode` is an enumeration that defines the possible
 * sleep modes for the AD7768 device, which is a high-performance analog-
 * to-digital converter. This enum provides two states: `AD7768_ACTIVE`
 * for when the device is fully operational and `AD7768_SLEEP` for when
 * the device is in a low-power state to conserve energy. This allows for
 * efficient power management in applications where the AD7768 is used.
 *
 * @param AD7768_ACTIVE Represents the active mode of the AD7768 device.
 * @param AD7768_SLEEP Represents the sleep mode of the AD7768 device.
 ******************************************************************************/
typedef enum {
	AD7768_ACTIVE,
	AD7768_SLEEP,
} ad7768_sleep_mode;

/***************************************************************************//**
 * @brief The `ad7768_power_mode` is an enumeration that defines the different
 * power modes available for the AD7768 device. It includes three modes:
 * ECO, MEDIAN, and FAST, each represented by a specific integer value.
 * These modes allow the user to configure the power consumption and
 * performance characteristics of the AD7768 device, with ECO being the
 * most power-efficient and FAST providing the highest performance.
 *
 * @param AD7768_ECO Represents the ECO power mode with a value of 0.
 * @param AD7768_MEDIAN Represents the MEDIAN power mode with a value of 2.
 * @param AD7768_FAST Represents the FAST power mode with a value of 3.
 ******************************************************************************/
typedef enum {
	AD7768_ECO = 0,
	AD7768_MEDIAN = 2,
	AD7768_FAST = 3,
} ad7768_power_mode;

/***************************************************************************//**
 * @brief The `ad7768_power_modes_raw` enumeration defines the different power
 * modes available for the AD7768 device, which include low power,
 * median, and fast modes. These modes allow the device to operate under
 * different power consumption and performance settings, providing
 * flexibility in its application. The enumeration also includes a member
 * to represent the total number of power modes available.
 *
 * @param AD7768_LOW_POWER_MODE Represents the low power mode for the AD7768
 * device.
 * @param AD7768_MEDIAN_MODE Represents the median power mode for the AD7768
 * device.
 * @param AD7768_FAST_MODE Represents the fast power mode for the AD7768 device.
 * @param AD7768_NUM_POWER_MODES Indicates the number of power modes available
 * for the AD7768 device.
 ******************************************************************************/
enum ad7768_power_modes_raw {
	AD7768_LOW_POWER_MODE,
	AD7768_MEDIAN_MODE,
	AD7768_FAST_MODE,
	AD7768_NUM_POWER_MODES
};

/***************************************************************************//**
 * @brief The `ad7768_mclk_div` is an enumeration that defines the possible
 * division factors for the master clock (MCLK) in the AD7768 device.
 * This enumeration allows the user to select different division ratios,
 * which can be used to adjust the clock frequency supplied to the
 * device, thereby affecting the device's operation and performance
 * characteristics. The available division factors are 32, 8, and 4, each
 * represented by a specific enumerator value.
 *
 * @param AD7768_MCLK_DIV_32 Represents a master clock division factor of 32.
 * @param AD7768_MCLK_DIV_8 Represents a master clock division factor of 8.
 * @param AD7768_MCLK_DIV_4 Represents a master clock division factor of 4.
 ******************************************************************************/
typedef enum {
	AD7768_MCLK_DIV_32 = 0,
	AD7768_MCLK_DIV_8 = 2,
	AD7768_MCLK_DIV_4 = 3,
} ad7768_mclk_div;

/***************************************************************************//**
 * @brief The `ad7768_dclk_div` enumeration defines the possible division
 * factors for the DCLK (Data Clock) in the AD7768 device. This
 * enumeration allows the user to select how much the DCLK should be
 * divided, providing options for division by 8, 4, 2, or no division
 * (1). This is useful for configuring the data clock rate to match the
 * requirements of the application or system in which the AD7768 is used.
 *
 * @param AD7768_DCLK_DIV_8 Represents a division factor of 8 for the DCLK.
 * @param AD7768_DCLK_DIV_4 Represents a division factor of 4 for the DCLK.
 * @param AD7768_DCLK_DIV_2 Represents a division factor of 2 for the DCLK.
 * @param AD7768_DCLK_DIV_1 Represents a division factor of 1 for the DCLK.
 ******************************************************************************/
typedef enum {
	AD7768_DCLK_DIV_8,
	AD7768_DCLK_DIV_4,
	AD7768_DCLK_DIV_2,
	AD7768_DCLK_DIV_1,
} ad7768_dclk_div;

/***************************************************************************//**
 * @brief The `ad7768_pin_spi_ctrl` is an enumeration that defines two modes of
 * control for the AD7768 device: pin control and SPI control. This
 * enumeration is used to specify how the device is being controlled,
 * either through direct pin manipulation or via the SPI interface,
 * allowing for flexible configuration depending on the application
 * requirements.
 *
 * @param AD7768_PIN_CTRL Represents the pin control mode for the AD7768 device.
 * @param AD7768_SPI_CTRL Represents the SPI control mode for the AD7768 device.
 ******************************************************************************/
typedef enum {
	AD7768_PIN_CTRL,
	AD7768_SPI_CTRL,
} ad7768_pin_spi_ctrl;

/***************************************************************************//**
 * @brief The `ad7768_conv_op` is an enumeration that defines the conversion
 * operation modes available for the AD7768 device. It includes two
 * modes: `AD7768_STANDARD_CONV` for standard continuous conversion and
 * `AD7768_ONE_SHOT_CONV` for single conversion operations. This
 * enumeration is used to configure the conversion behavior of the
 * AD7768, a high-performance analog-to-digital converter.
 *
 * @param AD7768_STANDARD_CONV Represents the standard conversion operation mode
 * for the AD7768 device.
 * @param AD7768_ONE_SHOT_CONV Represents the one-shot conversion operation mode
 * for the AD7768 device.
 ******************************************************************************/
typedef enum {
	AD7768_STANDARD_CONV,
	AD7768_ONE_SHOT_CONV,
} ad7768_conv_op;

/***************************************************************************//**
 * @brief The `ad7768_crc_sel` is an enumeration that defines the possible CRC
 * (Cyclic Redundancy Check) configurations for the AD7768 device. It
 * allows the selection of different CRC schemes to ensure data integrity
 * during communication, with options for no CRC, a 4-bit CRC, a 16-bit
 * CRC, and an alternative 16-bit CRC method.
 *
 * @param AD7768_NO_CRC Represents no CRC (Cyclic Redundancy Check) being used.
 * @param AD7768_CRC_4 Represents a 4-bit CRC being used.
 * @param AD7768_CRC_16 Represents a 16-bit CRC being used.
 * @param AD7768_CRC_16_2ND Represents a second 16-bit CRC option being used.
 ******************************************************************************/
typedef enum {
	AD7768_NO_CRC,
	AD7768_CRC_4,
	AD7768_CRC_16,
	AD7768_CRC_16_2ND,
} ad7768_crc_sel;

/***************************************************************************//**
 * @brief The `ad7768_ch` enumeration defines the available channels for the
 * AD7768 device, which is an 8-channel analog-to-digital converter. Each
 * enumerator represents a specific channel, from channel 0 to channel 7,
 * and the `AD7768_CH_NO` is used to denote the total number of channels
 * or an invalid channel state. This enumeration is used to manage and
 * reference the different channels within the AD7768 driver.
 *
 * @param AD7768_CH0 Represents channel 0 of the AD7768 device.
 * @param AD7768_CH1 Represents channel 1 of the AD7768 device.
 * @param AD7768_CH2 Represents channel 2 of the AD7768 device.
 * @param AD7768_CH3 Represents channel 3 of the AD7768 device.
 * @param AD7768_CH4 Represents channel 4 of the AD7768 device.
 * @param AD7768_CH5 Represents channel 5 of the AD7768 device.
 * @param AD7768_CH6 Represents channel 6 of the AD7768 device.
 * @param AD7768_CH7 Represents channel 7 of the AD7768 device.
 * @param AD7768_CH_NO Indicates the number of channels or an invalid channel.
 ******************************************************************************/
typedef enum {
	AD7768_CH0,
	AD7768_CH1,
	AD7768_CH2,
	AD7768_CH3,
	AD7768_CH4,
	AD7768_CH5,
	AD7768_CH6,
	AD7768_CH7,
	AD7768_CH_NO
} ad7768_ch;

/***************************************************************************//**
 * @brief The `ad7768_ch_state` is an enumeration that defines the possible
 * states for a channel in the AD7768 device, specifically indicating
 * whether a channel is enabled or in standby mode. This enumeration is
 * used to manage and control the operational state of each channel
 * within the AD7768, a high-performance analog-to-digital converter.
 *
 * @param AD7768_ENABLED Represents the enabled state of a channel.
 * @param AD7768_STANDBY Represents the standby state of a channel.
 ******************************************************************************/
typedef enum {
	AD7768_ENABLED,
	AD7768_STANDBY,
} ad7768_ch_state;

/***************************************************************************//**
 * @brief The `ad7768_ch_mode` is an enumeration that defines the possible
 * channel modes for the AD7768 device, specifically mode A and mode B.
 * These modes are used to configure the operational state of the
 * channels within the AD7768, which is a high-performance analog-to-
 * digital converter. The selection between these modes allows for
 * different configurations and behaviors of the device's channels.
 *
 * @param AD7768_MODE_A Represents channel mode A for the AD7768 device.
 * @param AD7768_MODE_B Represents channel mode B for the AD7768 device.
 ******************************************************************************/
typedef enum {
	AD7768_MODE_A,
	AD7768_MODE_B,
} ad7768_ch_mode;

/***************************************************************************//**
 * @brief The `ad7768_filt_type` is an enumeration that defines the types of
 * filters available for the AD7768 device, specifically the wideband and
 * sinc filters. This enumeration is used to configure the filter type in
 * the AD7768 device, which is a high-performance, 24-bit ADC. The choice
 * of filter affects the frequency response and performance
 * characteristics of the ADC.
 *
 * @param AD7768_FILTER_WIDEBAND Represents the wideband filter type for the
 * AD7768 device.
 * @param AD7768_FILTER_SINC Represents the sinc filter type for the AD7768
 * device.
 ******************************************************************************/
typedef enum {
	AD7768_FILTER_WIDEBAND,
	AD7768_FILTER_SINC,
} ad7768_filt_type;

/***************************************************************************//**
 * @brief The `ad7768_dec_rate` is an enumeration that defines various
 * decimation rates for the AD7768 device, which is a high-performance
 * analog-to-digital converter. Each enumerator corresponds to a specific
 * decimation factor, which is used to reduce the sampling rate of the
 * input signal, thereby affecting the resolution and bandwidth of the
 * conversion process. This enumeration allows for easy selection of the
 * desired decimation rate in the device's configuration.
 *
 * @param AD7768_DEC_X32 Represents a decimation rate of 32.
 * @param AD7768_DEC_X64 Represents a decimation rate of 64.
 * @param AD7768_DEC_X128 Represents a decimation rate of 128.
 * @param AD7768_DEC_X256 Represents a decimation rate of 256.
 * @param AD7768_DEC_X512 Represents a decimation rate of 512.
 * @param AD7768_DEC_X1024 Represents a decimation rate of 1024.
 * @param AD7768_DEC_X1024_2ND Represents a secondary decimation rate of 1024.
 * @param AD7768_DEC_X1024_3RD Represents a tertiary decimation rate of 1024.
 ******************************************************************************/
typedef enum {
	AD7768_DEC_X32,
	AD7768_DEC_X64,
	AD7768_DEC_X128,
	AD7768_DEC_X256,
	AD7768_DEC_X512,
	AD7768_DEC_X1024,
	AD7768_DEC_X1024_2ND,
	AD7768_DEC_X1024_3RD,
} ad7768_dec_rate;

/***************************************************************************//**
 * @brief The `ad7768_dec_rate_vals` is a static constant integer array
 * containing six elements. These elements represent the decimation rates
 * available for the AD7768 device, which are 32, 64, 128, 256, 512, and
 * 1024.
 *
 * @details This array is used to define the possible decimation rates that can
 * be configured for the AD7768 device.
 ******************************************************************************/
static const int ad7768_dec_rate_vals[6] = {
	32, 64, 128, 256, 512, 1024
};

/***************************************************************************//**
 * @brief The `ad7768_mclk_div_vals` is a static constant integer array
 * containing three elements: 32, 8, and 4. These values represent the
 * possible master clock (MCLK) division factors for the AD7768 device.
 *
 * @details This array is used to configure the MCLK division settings in the
 * AD7768 device, allowing for different clock division ratios.
 ******************************************************************************/
static const int ad7768_mclk_div_vals[3] = {
	32, 8, 4
};

/***************************************************************************//**
 * @brief The `ad7768_freq_config` structure is used to configure frequency-
 * related settings for the AD7768 device, specifically defining the
 * frequency and decimation rate parameters. This structure is part of
 * the AD7768 driver and is used to manage the sampling frequency
 * configurations for different operational modes of the device.
 *
 * @param freq Represents the frequency setting for the AD7768 device.
 * @param dec_rate Specifies the decimation rate for the AD7768 device.
 ******************************************************************************/
struct ad7768_freq_config {
	unsigned int freq;
	unsigned int dec_rate;
};

/***************************************************************************//**
 * @brief The `ad7768_avail_freq` structure is designed to hold information
 * about the available frequency configurations for the AD7768 device. It
 * contains a count of the number of frequency configurations (`n_freqs`)
 * and an array (`freq_cfg`) that stores the actual frequency
 * configuration details, each defined by the `ad7768_freq_config`
 * structure. This structure is used to manage and access different
 * frequency settings that the AD7768 can operate under, allowing for
 * flexible configuration of the device's sampling rates.
 *
 * @param n_freqs Stores the number of available frequency configurations.
 * @param freq_cfg An array of frequency configurations, each defined by the
 * ad7768_freq_config structure, with a maximum size of
 * AD7768_MAX_FREQ_PER_MODE.
 ******************************************************************************/
struct ad7768_avail_freq {
	unsigned int n_freqs;
	struct ad7768_freq_config freq_cfg[AD7768_MAX_FREQ_PER_MODE];
};

/***************************************************************************//**
 * @brief The `ad7768_dev` structure is a comprehensive representation of the
 * AD7768 device configuration and state, encapsulating all necessary
 * parameters for its operation. It includes pointers to SPI and GPIO
 * descriptors for communication and control, various configuration
 * settings such as power and sleep modes, clock dividers, and conversion
 * operations. Additionally, it maintains arrays for channel states,
 * modes, filter types, and decimation rates, allowing for detailed
 * control over the device's behavior. The structure also tracks the
 * master clock frequency, data lines, and sampling frequency, along with
 * available frequency configurations for different power modes, making
 * it a central component for managing the AD7768's functionality.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_reset Pointer to the GPIO descriptor for the reset pin.
 * @param gpio_reset_value Value to set on the reset GPIO pin.
 * @param gpio_mode0 Pointer to the GPIO descriptor for mode0 pin.
 * @param gpio_mode1 Pointer to the GPIO descriptor for mode1 pin.
 * @param gpio_mode2 Pointer to the GPIO descriptor for mode2 pin.
 * @param gpio_mode3 Pointer to the GPIO descriptor for mode3 pin.
 * @param pin_spi_input_value Value for the SPI input pin configuration.
 * @param pin_spi_ctrl Control mode for the SPI pins.
 * @param sleep_mode Current sleep mode of the device.
 * @param power_mode Current power mode of the device.
 * @param power_mode_raw Raw enumeration of power modes.
 * @param mclk_div Master clock divider setting.
 * @param dclk_div Data clock divider setting.
 * @param conv_op Conversion operation mode.
 * @param crc_sel CRC selection for data integrity.
 * @param ch_state Array representing the state of each channel.
 * @param ch_mode Array representing the mode of each channel.
 * @param filt_type Array representing the filter type for each channel group.
 * @param dec_rate Array representing the decimation rate for each channel
 * group.
 * @param mclk Master clock frequency.
 * @param datalines Number of data lines used.
 * @param sampling_freq Sampling frequency of the device.
 * @param avail_freq Array of available frequencies for each power mode.
 ******************************************************************************/
typedef struct {
	struct no_os_spi_desc		*spi_desc;
	struct no_os_gpio_desc		*gpio_reset;
	uint8_t				gpio_reset_value;
	struct no_os_gpio_desc		*gpio_mode0;
	struct no_os_gpio_desc		*gpio_mode1;
	struct no_os_gpio_desc		*gpio_mode2;
	struct no_os_gpio_desc		*gpio_mode3;
	uint8_t				pin_spi_input_value;
	ad7768_pin_spi_ctrl		pin_spi_ctrl;
	ad7768_sleep_mode		sleep_mode;
	ad7768_power_mode		power_mode;
	enum ad7768_power_modes_raw	power_mode_raw;
	ad7768_mclk_div			mclk_div;
	ad7768_dclk_div			dclk_div;
	ad7768_conv_op			conv_op;
	ad7768_crc_sel			crc_sel;
	ad7768_ch_state			ch_state[8];
	ad7768_ch_mode			ch_mode[8];
	ad7768_filt_type		filt_type[2];
	ad7768_dec_rate			dec_rate[2];
	unsigned int			mclk;
	unsigned int			datalines;
	unsigned int			sampling_freq;
	struct ad7768_avail_freq	avail_freq[AD7768_NUM_POWER_MODES];
} ad7768_dev;

/***************************************************************************//**
 * @brief The `ad7768_init_param` structure is used to initialize and configure
 * the AD7768 device, a high-performance ADC. It includes parameters for
 * setting up the SPI interface, configuring GPIO pins for various modes,
 * and defining operational settings such as sleep and power modes, clock
 * dividers, and conversion operations. This structure is essential for
 * setting up the device's initial state and ensuring proper
 * communication and functionality.
 *
 * @param spi_init Initializes the SPI interface parameters.
 * @param gpio_reset Initializes the GPIO parameters for the reset pin.
 * @param gpio_reset_value Specifies the value to set on the reset GPIO pin.
 * @param gpio_mode0 Initializes the GPIO parameters for mode0 pin.
 * @param gpio_mode1 Initializes the GPIO parameters for mode1 pin.
 * @param gpio_mode2 Initializes the GPIO parameters for mode2 pin.
 * @param gpio_mode3 Initializes the GPIO parameters for mode3 pin.
 * @param pin_spi_input_value Specifies the input value for the SPI pin control.
 * @param sleep_mode Defines the sleep mode configuration for the device.
 * @param power_mode Defines the power mode configuration for the device.
 * @param mclk_div Specifies the master clock divider setting.
 * @param dclk_div Specifies the data clock divider setting.
 * @param conv_op Defines the conversion operation mode.
 * @param crc_sel Specifies the CRC selection for data integrity.
 * @param mclk Specifies the master clock frequency.
 * @param datalines Specifies the number of data lines used.
 ******************************************************************************/
typedef struct {
	/* SPI */
	struct no_os_spi_init_param		spi_init;
	/* GPIO */
	struct no_os_gpio_init_param		gpio_reset;
	uint8_t					gpio_reset_value;
	struct no_os_gpio_init_param		gpio_mode0;
	struct no_os_gpio_init_param		gpio_mode1;
	struct no_os_gpio_init_param		gpio_mode2;
	struct no_os_gpio_init_param		gpio_mode3;
	/* Configuration */
	uint8_t					pin_spi_input_value;
	ad7768_sleep_mode			sleep_mode;
	ad7768_power_mode			power_mode;
	ad7768_mclk_div				mclk_div;
	ad7768_dclk_div				dclk_div;
	ad7768_conv_op				conv_op;
	ad7768_crc_sel				crc_sel;
	unsigned int				mclk;
	unsigned int				datalines;
} ad7768_init_param;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* SPI read from device. */
/***************************************************************************//**
 * @brief This function is used to read a specific register from the AD7768
 * device using SPI communication. It requires a valid device structure
 * that has been properly initialized and configured for SPI operations.
 * The function reads the register specified by the address and stores
 * the result in the provided data pointer. It is important to ensure
 * that the device is correctly set up and that the register address is
 * within the valid range for the AD7768. The function returns an error
 * code if the SPI communication fails.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null and should be properly initialized for SPI
 * communication.
 * @param reg_addr The address of the register to read. Must be a valid register
 * address for the AD7768.
 * @param reg_data A pointer to a uint8_t where the read register data will be
 * stored. Must not be null.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error in SPI communication.
 ******************************************************************************/
int32_t ad7768_spi_read(ad7768_dev *dev,
			uint8_t reg_addr,
			uint8_t *reg_data);
/* SPI write to device. */
/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * on the AD7768 device using the SPI interface. It is typically called
 * when there is a need to configure or modify the settings of the AD7768
 * by writing to its registers. The function requires a valid device
 * structure that has been properly initialized, and it assumes that the
 * SPI interface is correctly set up. The function returns an integer
 * status code indicating the success or failure of the write operation.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. This
 * must be a valid, initialized device structure, and must not be
 * null.
 * @param reg_addr The address of the register to which data will be written. It
 * should be a valid register address within the range supported
 * by the AD7768.
 * @param reg_data The data byte to be written to the specified register. It
 * should be a valid byte value to configure the register as
 * intended.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error during the SPI write operation.
 ******************************************************************************/
int32_t ad7768_spi_write(ad7768_dev *dev,
			 uint8_t reg_addr,
			 uint8_t reg_data);
/* SPI read from device using a mask. */
/***************************************************************************//**
 * @brief Use this function to read a specific portion of a register's value
 * from the AD7768 device by applying a mask. This is useful when only
 * certain bits of a register are of interest. The function requires a
 * valid device structure and a non-null pointer for storing the masked
 * data. It returns an error code if the SPI read operation fails.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD7768.
 * @param mask A bitmask to apply to the register value. Determines which bits
 * of the register are of interest.
 * @param data A pointer to a uint8_t where the masked register value will be
 * stored. Must not be null.
 * @return Returns an int32_t error code from the SPI read operation. The masked
 * register value is stored in the location pointed to by the data
 * parameter.
 ******************************************************************************/
int32_t ad7768_spi_read_mask(ad7768_dev *dev,
			     uint8_t reg_addr,
			     uint8_t mask,
			     uint8_t *data);
/* SPI write to device using a mask. */
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register on an
 * AD7768 device by applying a mask. It first reads the current value of
 * the register, applies the mask to clear the bits that will be
 * modified, and then writes the new data to those bits. This function is
 * useful when only certain bits of a register need to be updated without
 * affecting the other bits. It should be called when the device is
 * properly initialized and communication with the device is established.
 * The function returns an error code if the read or write operation
 * fails.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null, and the device must be initialized.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD7768 device.
 * @param mask A bitmask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be affected by the data
 * parameter.
 * @param data The data to be written to the register, masked by the mask
 * parameter. Only the bits corresponding to the mask will be
 * written.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating an error in reading or writing the register.
 ******************************************************************************/
int32_t ad7768_spi_write_mask(ad7768_dev *dev,
			      uint8_t reg_addr,
			      uint8_t mask,
			      uint8_t data);
/* Set the device sleep mode. */
/***************************************************************************//**
 * @brief This function configures the AD7768 device to enter either active or
 * sleep mode based on the specified parameter. It should be called when
 * there is a need to change the power state of the device, such as
 * reducing power consumption by entering sleep mode. The function
 * requires a valid device structure and a sleep mode enumeration value.
 * It does not perform any error checking on the input parameters, so the
 * caller must ensure that the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null, and the device must be properly initialized before
 * calling this function. The caller retains ownership.
 * @param mode An enumeration value of type ad7768_sleep_mode indicating the
 * desired sleep mode. Valid values are AD7768_ACTIVE and
 * AD7768_SLEEP.
 * @return Returns 0 on success. The function does not perform any error
 * checking and assumes the inputs are valid.
 ******************************************************************************/
int32_t ad7768_set_sleep_mode(ad7768_dev *dev,
			      ad7768_sleep_mode mode);
/* Get the device sleep mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current sleep mode setting of an
 * AD7768 device. It is typically called after the device has been
 * initialized and configured, to verify or monitor the current sleep
 * mode. The function requires a valid device structure and a pointer to
 * a variable where the sleep mode will be stored. It does not perform
 * any validation on the input parameters, so the caller must ensure that
 * the device pointer is valid and the mode pointer is not null.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param mode A pointer to an ad7768_sleep_mode variable where the current
 * sleep mode will be stored. Must not be null. The function writes
 * the current sleep mode to this location.
 * @return Returns 0 on success. The current sleep mode is written to the
 * location pointed to by the mode parameter.
 ******************************************************************************/
int32_t ad7768_get_sleep_mode(ad7768_dev *dev,
			      ad7768_sleep_mode *mode);
/* Set the device power mode. */
/***************************************************************************//**
 * @brief Use this function to configure the power mode of an AD7768 device,
 * which can affect its power consumption and performance
 * characteristics. This function should be called after the device has
 * been properly initialized. The function supports both SPI and pin
 * control modes, and it will return an error if the specified power mode
 * is invalid for the current configuration. Ensure that the device is in
 * a compatible state for the desired power mode before calling this
 * function.
 *
 * @param dev A pointer to an initialized ad7768_dev structure representing the
 * device. Must not be null.
 * @param mode The desired power mode to set, specified as an ad7768_power_mode
 * enum value. Valid values are AD7768_ECO, AD7768_MEDIAN, and
 * AD7768_FAST.
 * @return Returns 0 on success, or -1 if the specified power mode is invalid
 * for the current configuration.
 ******************************************************************************/
int32_t ad7768_set_power_mode(ad7768_dev *dev,
			      ad7768_power_mode mode);
/* Get the device power mode. */
/***************************************************************************//**
 * @brief This function is used to obtain the current power mode setting of an
 * AD7768 device. It is typically called after the device has been
 * initialized and configured, to verify or monitor the power mode state.
 * The function requires a valid device structure and a pointer to a
 * variable where the power mode will be stored. It does not perform any
 * validation on the input parameters, so the caller must ensure that the
 * device is properly initialized and the mode pointer is valid.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null and should be properly initialized before calling this
 * function.
 * @param mode A pointer to an ad7768_power_mode variable where the current
 * power mode will be stored. Must not be null.
 * @return Returns 0 on success. The current power mode is stored in the
 * variable pointed to by mode.
 ******************************************************************************/
int32_t ad7768_get_power_mode(ad7768_dev *dev,
			      ad7768_power_mode *mode);
/* Set the MCLK divider. */
/***************************************************************************//**
 * @brief This function configures the master clock divider for the AD7768
 * device, which affects the internal clocking and potentially the
 * sampling rate. It should be called when a change in the clock
 * configuration is required, such as during initialization or when
 * adjusting the device's performance characteristics. The function
 * assumes that the device has been properly initialized and that the
 * provided clock divider value is valid. It does not perform any error
 * checking on the input parameters and always returns 0, indicating
 * success.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device
 * instance. Must not be null, and the device must be initialized
 * before calling this function. The caller retains ownership.
 * @param clk_div An ad7768_mclk_div enumeration value specifying the desired
 * master clock divider. Valid values are defined in the
 * ad7768_mclk_div enum, and the function does not validate this
 * parameter.
 * @return Always returns 0, indicating success. The function updates the
 * mclk_div field in the ad7768_dev structure to reflect the new clock
 * divider setting.
 ******************************************************************************/
int32_t ad7768_set_mclk_div(ad7768_dev *dev,
			    ad7768_mclk_div clk_div);
/* Get the MCLK divider. */
/***************************************************************************//**
 * @brief This function is used to obtain the current master clock (MCLK)
 * divider setting from an AD7768 device. It is typically called after
 * the device has been initialized and configured, to verify or log the
 * current MCLK divider setting. The function requires a valid device
 * structure pointer and a pointer to an `ad7768_mclk_div` variable where
 * the current MCLK divider value will be stored. The function does not
 * perform any validation on the input pointers, so they must be valid
 * and non-null before calling this function.
 *
 * @param dev A pointer to an `ad7768_dev` structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param clk_div A pointer to an `ad7768_mclk_div` variable where the current
 * MCLK divider setting will be stored. This pointer must be
 * valid and non-null.
 * @return Returns 0 on success. The current MCLK divider setting is written to
 * the location pointed to by `clk_div`.
 ******************************************************************************/
int32_t ad7768_get_mclk_div(ad7768_dev *dev,
			    ad7768_mclk_div *clk_div);
/* Set the DCLK divider. */
/***************************************************************************//**
 * @brief This function configures the DCLK divider for the AD7768 device, which
 * is essential for setting the desired clock division ratio. It should
 * be called after the device has been properly initialized. The function
 * supports both SPI and pin control modes, adjusting the configuration
 * accordingly. If the provided clock division setting is invalid for the
 * current configuration, the function returns an error. This function is
 * crucial for ensuring the correct timing and operation of the device.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device
 * instance. Must not be null, and the device should be initialized
 * before calling this function.
 * @param clk_div An ad7768_dclk_div value specifying the desired DCLK division
 * ratio. Must be a valid enumerated value; otherwise, the
 * function will return an error if the configuration is not
 * supported.
 * @return Returns 0 on success, or -1 if the specified DCLK division is invalid
 * for the current device configuration.
 ******************************************************************************/
int32_t ad7768_set_dclk_div(ad7768_dev *dev,
			    ad7768_dclk_div clk_div);
/* Get the DCLK divider. */
/***************************************************************************//**
 * @brief This function is used to obtain the current DCLK (Data Clock) divider
 * setting from an AD7768 device. It is typically called when there is a
 * need to verify or log the current DCLK configuration. The function
 * requires a valid device instance and a pointer to a variable where the
 * DCLK divider value will be stored. It is important to ensure that the
 * device has been properly initialized before calling this function. The
 * function does not perform any validation on the input parameters and
 * assumes they are valid.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device
 * instance. Must not be null and should be properly initialized
 * before calling this function.
 * @param clk_div A pointer to an ad7768_dclk_div variable where the current
 * DCLK divider setting will be stored. Must not be null.
 * @return Returns 0 on success, indicating that the DCLK divider value was
 * successfully retrieved and stored in the provided variable.
 ******************************************************************************/
int32_t ad7768_get_dclk_div(ad7768_dev *dev,
			    ad7768_dclk_div *clk_div);
/* Set the conversion operation mode. */
/***************************************************************************//**
 * @brief This function configures the conversion operation mode of the AD7768
 * device, allowing the user to switch between standard and one-shot
 * conversion modes. It must be called with a valid device structure and
 * a desired conversion operation mode. The function checks the control
 * mode of the device (SPI or pin control) and applies the appropriate
 * settings. If the configuration is invalid for the current device
 * settings, it returns an error. This function should be used when the
 * conversion mode needs to be changed after the device has been
 * initialized.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param conv_op The desired conversion operation mode, specified as an
 * ad7768_conv_op enum value. Valid values are
 * AD7768_STANDARD_CONV and AD7768_ONE_SHOT_CONV.
 * @return Returns 0 on success, or -1 if the conversion operation is invalid
 * for the current configuration.
 ******************************************************************************/
int32_t ad7768_set_conv_op(ad7768_dev *dev,
			   ad7768_conv_op conv_op);
/* Get the conversion operation mode. */
/***************************************************************************//**
 * @brief Use this function to obtain the current conversion operation mode of
 * the AD7768 device. This function is typically called after the device
 * has been initialized and configured. It provides the conversion
 * operation mode, which can be either standard or one-shot, depending on
 * the device's current settings. Ensure that the device pointer is valid
 * and properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device
 * instance. Must not be null and should be initialized prior to
 * calling this function.
 * @param conv_op A pointer to an ad7768_conv_op variable where the current
 * conversion operation mode will be stored. Must not be null.
 * @return Returns 0 on success, indicating that the conversion operation mode
 * was successfully retrieved.
 ******************************************************************************/
int32_t ad7768_get_conv_op(ad7768_dev *dev,
			   ad7768_conv_op *conv_op);
/* Set the CRC selection. */
/***************************************************************************//**
 * @brief This function configures the CRC selection mode for the AD7768 device,
 * which determines the type of CRC error checking used during SPI
 * communication. It should be called when the CRC mode needs to be
 * changed, typically during device initialization or configuration. The
 * function requires a valid device structure and a CRC selection value.
 * It updates the device's internal state to reflect the new CRC mode.
 * Ensure the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param crc_sel An ad7768_crc_sel enumeration value specifying the desired CRC
 * mode. Valid values are AD7768_NO_CRC, AD7768_CRC_4,
 * AD7768_CRC_16, and AD7768_CRC_16_2ND.
 * @return Returns 0 on success. The device's CRC selection mode is updated.
 ******************************************************************************/
int32_t ad7768_set_crc_sel(ad7768_dev *dev,
			   ad7768_crc_sel crc_sel);
/* Get the CRC selection. */
/***************************************************************************//**
 * @brief This function is used to obtain the current CRC selection setting from
 * an AD7768 device. It is typically called after the device has been
 * initialized and configured, to verify or log the current CRC setting.
 * The function requires a valid device structure pointer and a pointer
 * to a variable where the CRC selection will be stored. It does not
 * perform any error checking on the input parameters, so the caller must
 * ensure that the pointers provided are valid and that the device has
 * been properly initialized.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. This
 * must be a valid, initialized device structure. The caller retains
 * ownership and must ensure it is not null.
 * @param crc_sel A pointer to an ad7768_crc_sel variable where the current CRC
 * selection will be stored. This must not be null, and the
 * caller retains ownership of the memory.
 * @return Returns 0 on success, indicating the CRC selection was successfully
 * retrieved and stored in the provided variable.
 ******************************************************************************/
int32_t ad7768_get_crc_sel(ad7768_dev *dev,
			   ad7768_crc_sel *crc_sel);
/* Set the channel state. */
/***************************************************************************//**
 * @brief This function is used to change the operational state of a specific
 * channel on the AD7768 device, either enabling it or putting it into
 * standby. It should be called when there is a need to modify the
 * channel's activity, such as during power management or configuration
 * changes. The function requires a valid device structure and channel
 * identifier, and it updates the channel state accordingly. It is
 * important to ensure that the device has been properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param ch An ad7768_ch enumeration value specifying the channel to be
 * modified. Must be a valid channel identifier within the range of
 * available channels.
 * @param state An ad7768_ch_state enumeration value indicating the desired
 * state of the channel, either AD7768_ENABLED or AD7768_STANDBY.
 * @return Returns 0 on success, indicating the channel state was set
 * successfully.
 ******************************************************************************/
int32_t ad7768_set_ch_state(ad7768_dev *dev,
			    ad7768_ch ch,
			    ad7768_ch_state state);
/* Get the channel state. */
/***************************************************************************//**
 * @brief This function is used to obtain the current state of a specific
 * channel on the AD7768 device. It should be called when you need to
 * check whether a channel is enabled or in standby mode. Ensure that the
 * device has been properly initialized before calling this function. The
 * function requires a valid channel identifier and a pointer to store
 * the channel state. It does not perform any error checking on the
 * channel index, so the caller must ensure the channel is within the
 * valid range.
 *
 * @param dev A pointer to an initialized ad7768_dev structure representing the
 * device. Must not be null.
 * @param ch An enumerated value of type ad7768_ch representing the channel
 * whose state is to be retrieved. Must be a valid channel identifier.
 * @param state A pointer to an ad7768_ch_state variable where the channel state
 * will be stored. Must not be null.
 * @return Returns 0 on success. The state of the specified channel is written
 * to the location pointed to by the state parameter.
 ******************************************************************************/
int32_t ad7768_get_ch_state(ad7768_dev *dev,
			    ad7768_ch ch,
			    ad7768_ch_state *state);
/* Set the mode configuration. */
/***************************************************************************//**
 * @brief This function configures the mode settings of the AD7768 device by
 * setting the filter type and decimation rate for a specified channel
 * mode. It should be called when the user needs to change the
 * operational parameters of the device, such as switching between
 * different modes or adjusting the data processing characteristics. The
 * function requires a valid device structure and appropriate mode,
 * filter type, and decimation rate values. It does not perform any
 * validation on the input parameters, so the caller must ensure that the
 * provided values are within the expected range and contextually
 * appropriate for the device's current state.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param mode Specifies the channel mode to configure, either AD7768_MODE_A or
 * AD7768_MODE_B. The mode must be valid and supported by the
 * device.
 * @param filt_type The filter type to set, which can be AD7768_FILTER_WIDEBAND
 * or AD7768_FILTER_SINC. The filter type must be compatible
 * with the device's capabilities.
 * @param dec_rate The decimation rate to apply, selected from the
 * ad7768_dec_rate enumeration. The decimation rate must be
 * valid and supported by the device.
 * @return Returns 0 on success, indicating the mode configuration was set
 * successfully.
 ******************************************************************************/
int32_t ad7768_set_mode_config(ad7768_dev *dev,
			       ad7768_ch_mode mode,
			       ad7768_filt_type filt_type,
			       ad7768_dec_rate dec_rate);
/* Get the mode configuration. */
/***************************************************************************//**
 * @brief Use this function to obtain the current filter type and decimation
 * rate settings for a specific channel mode on the AD7768 device. This
 * function is useful when you need to verify or log the current
 * configuration of the device. Ensure that the device has been properly
 * initialized before calling this function. The function does not
 * perform any validation on the input parameters, so it is the caller's
 * responsibility to ensure that the mode is valid and that the pointers
 * provided for output are not null.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device
 * instance. Must not be null.
 * @param mode An ad7768_ch_mode value specifying the channel mode for which the
 * configuration is being retrieved. Must be a valid mode.
 * @param filt_type A pointer to an ad7768_filt_type variable where the current
 * filter type will be stored. Must not be null.
 * @param dec_rate A pointer to an ad7768_dec_rate variable where the current
 * decimation rate will be stored. Must not be null.
 * @return Returns 0 on success. The filt_type and dec_rate pointers are
 * populated with the current configuration values for the specified
 * mode.
 ******************************************************************************/
int32_t ad7768_get_mode_config(ad7768_dev *dev,
			       ad7768_ch_mode mode,
			       ad7768_filt_type *filt_type,
			       ad7768_dec_rate *dec_rate);
/* Set the channel mode. */
/***************************************************************************//**
 * @brief This function configures the mode of a specific channel on the AD7768
 * device. It should be used when you need to change the operational mode
 * of a channel, such as switching between different modes like Mode A or
 * Mode B. The function requires a valid device structure and channel
 * identifier, and it updates the channel mode both in the device's
 * register and in the local device structure. Ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param ch An ad7768_ch enum value specifying the channel to configure. Must
 * be a valid channel identifier within the range of available
 * channels.
 * @param mode An ad7768_ch_mode enum value indicating the mode to set for the
 * specified channel. Valid modes are defined by the ad7768_ch_mode
 * enumeration.
 * @return Returns 0 on success. The function updates the mode of the specified
 * channel in the device structure.
 ******************************************************************************/
int32_t ad7768_set_ch_mode(ad7768_dev *dev,
			   ad7768_ch ch,
			   ad7768_ch_mode mode);
/* Get the channel mode. */
/***************************************************************************//**
 * @brief Use this function to obtain the current mode configuration of a
 * specific channel on the AD7768 device. It is essential to ensure that
 * the device has been properly initialized before calling this function.
 * The function will store the mode of the specified channel in the
 * provided mode pointer. This function does not perform any error
 * checking on the channel index, so it is the caller's responsibility to
 * ensure that the channel index is within the valid range.
 *
 * @param dev A pointer to an initialized ad7768_dev structure representing the
 * device. Must not be null.
 * @param ch The channel index of type ad7768_ch for which the mode is to be
 * retrieved. Valid values are AD7768_CH0 to AD7768_CH7.
 * @param mode A pointer to an ad7768_ch_mode variable where the channel mode
 * will be stored. Must not be null.
 * @return Returns 0 on success. The mode of the specified channel is written to
 * the location pointed to by mode.
 ******************************************************************************/
int32_t ad7768_get_ch_mode(ad7768_dev *dev,
			   ad7768_ch ch,
			   ad7768_ch_mode *mode);
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the AD7768 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device to ensure proper setup. The function
 * allocates resources and configures the device according to the
 * specified parameters. If initialization fails, it returns an error
 * code and ensures that any allocated resources are properly freed.
 * Successful initialization results in a pointer to the device structure
 * being set, which can be used for subsequent operations.
 *
 * @param device A pointer to a pointer of type ad7768_dev. This must not be
 * null. On successful initialization, it will point to the
 * initialized device structure. The caller is responsible for
 * managing the memory of this pointer.
 * @param init_param A structure of type ad7768_init_param containing the
 * initialization parameters for the device. This includes SPI
 * and GPIO configurations, as well as various operational
 * settings. The structure must be properly populated before
 * calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered.
 ******************************************************************************/
int32_t ad7768_setup(ad7768_dev **device,
		     ad7768_init_param init_param);
/* Free the resources allocated by ad7768_setup(). */
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD7768 device instance when it is no longer needed. This function
 * should be called to clean up after a successful initialization and use
 * of the device, ensuring that all allocated memory and hardware
 * resources are freed. It is important to call this function to prevent
 * resource leaks in your application.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device
 * instance to be removed. Must not be null. The function assumes
 * ownership of the resources and will free them, so the caller
 * should not use the pointer after this function is called.
 * @return Returns 0 to indicate successful removal of the device resources.
 ******************************************************************************/
int ad7768_remove(ad7768_dev *dev);
/* Begin initializing the device. */
/***************************************************************************//**
 * @brief This function starts the initialization process for the AD7768 device
 * by allocating necessary resources and configuring initial settings
 * based on the provided parameters. It should be called before any other
 * operations on the device and is typically followed by a call to
 * `ad7768_setup_finish` to complete the initialization. The function
 * handles resource allocation and initial configuration, and it is
 * important to ensure that the `device` pointer is valid and that the
 * `init_param` structure is correctly populated with initialization
 * parameters. If the function fails, it returns an error code and cleans
 * up any allocated resources.
 *
 * @param device A pointer to a pointer of type `ad7768_dev`. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `ad7768_init_param` containing
 * initialization parameters such as SPI and GPIO
 * configurations. Must be properly initialized before calling
 * the function.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the `device` pointer is updated to point to the initialized
 * device structure.
 ******************************************************************************/
int32_t ad7768_setup_begin(ad7768_dev **device,
			   ad7768_init_param init_param);
/* Finish initializing the device. */
/***************************************************************************//**
 * @brief This function finalizes the setup of an AD7768 device by configuring
 * its GPIO pins and setting various operational parameters. It should be
 * called after the initial setup phase to ensure the device is fully
 * configured and ready for operation. The function configures GPIO pins
 * for mode selection and reset, and sets the device's sleep mode, master
 * clock divider, CRC selection, power mode, data clock divider, and
 * conversion operation mode. It handles any errors during GPIO
 * configuration by cleaning up resources. Ensure that the device
 * structure and initialization parameters are correctly set up before
 * calling this function.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device to be
 * configured. Must not be null.
 * @param init_param An ad7768_init_param structure containing initialization
 * parameters such as GPIO configurations and operational
 * settings. Must be properly initialized before use.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the setup fails. If an error occurs, GPIO resources are cleaned up.
 ******************************************************************************/
int32_t ad7768_setup_finish(ad7768_dev *dev,
			    ad7768_init_param init_param);
/* Set available sampling frequency. */
/***************************************************************************//**
 * @brief This function configures the available sampling frequencies for each
 * power mode of the AD7768 device based on its master clock frequency
 * and decimation rates. It should be called after the device has been
 * initialized and configured with the appropriate master clock and data
 * line settings. The function updates the `avail_freq` field of the
 * `ad7768_dev` structure, which holds the frequency configurations for
 * each power mode. Note that in configurations with a single data line,
 * the maximum frequency in fast mode is not supported and will be
 * adjusted accordingly.
 *
 * @param dev A pointer to an `ad7768_dev` structure representing the device.
 * Must not be null. The structure should be properly initialized and
 * configured with the master clock frequency and data line settings
 * before calling this function.
 * @return None
 ******************************************************************************/
void ad7768_set_available_sampl_freq(ad7768_dev *dev);
/* Set power mode and sampling frequency. */
/***************************************************************************//**
 * @brief This function configures the AD7768 device to operate in a specified
 * power mode and adjusts the sampling frequency accordingly. It should
 * be used when there is a need to change the power consumption
 * characteristics and data rate of the device. The function must be
 * called with a valid device structure and a power mode enumeration
 * value. It updates the device's internal state and communicates the
 * changes via SPI. The function handles errors internally and returns an
 * error code if any operation fails.
 *
 * @param dev A pointer to an ad7768_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function. The caller retains ownership.
 * @param mode An enumeration value of type ad7768_power_modes_raw indicating
 * the desired power mode. Valid values are AD7768_LOW_POWER_MODE,
 * AD7768_MEDIAN_MODE, and AD7768_FAST_MODE. Invalid values may
 * result in undefined behavior.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if an error occurs during the operation.
 ******************************************************************************/
int ad7768_set_power_mode_and_sampling_freq(ad7768_dev *dev,
		enum ad7768_power_modes_raw mode);
#endif // AD7768_H_
