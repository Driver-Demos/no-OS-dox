/***************************************************************************//**
 *   @file   ad4858.c
 *   @brief  Header file for the ad4858 drivers
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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
#ifndef AD4858_H_
#define AD4858_H_

#include <stdint.h>
#include <stdbool.h>
#include "no_os_util.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_error.h"

#define AD4858_R1B          (1ul << 16)
#define AD4858_R2B          (2ul << 16)
#define AD4858_R3B          (3ul << 16)
#define AD4858_R4B          (4ul << 16)
#define AD4858_LEN(x)       ((x) >> 16)
#define AD4858_ADDR(x)      ((x) & 0xFFFF)

/** Register definitions */
#define AD4858_REG_INTERFACE_CONFIG_A       (AD4858_R1B | 0x00)
#define AD4858_REG_INTERFACE_CONFIG_B       (AD4858_R1B | 0x01)
#define AD4858_REG_DEVICE_CONFIG            (AD4858_R1B | 0x02)
#define AD4858_REG_CHIP_TYPE                (AD4858_R1B | 0x03)
#define AD4858_REG_PRODUCT_ID_L             (AD4858_R1B | 0x04)
#define AD4858_REG_PRODUCT_ID_H             (AD4858_R1B | 0x05)
#define AD4858_REG_CHIP_GRADE               (AD4858_R1B | 0x06)
#define AD4858_REG_SCRATCH_PAD              (AD4858_R1B | 0x0A)
#define AD4858_REG_SPI_REV                  (AD4858_R1B | 0x0B)
#define AD4858_REG_VENDOR_L                 (AD4858_R1B | 0x0C)
#define AD4858_REG_VENDOR_H                 (AD4858_R1B | 0x0D)
#define AD4858_REG_STREAM_MODE              (AD4858_R1B | 0x0E)
#define AD4858_REG_TRANSFER_CONFIG          (AD4858_R1B | 0x0F)
#define AD4858_REG_INTERFACE_CONFIG_C       (AD4858_R1B | 0x10)
#define AD4858_REG_INTERFACE_STATUS_A       (AD4858_R1B | 0x11)
#define AD4858_REG_SPI_CONFIG_D             (AD4858_R1B | 0x14)
#define AD4858_REG_DEVICE_STATUS            (AD4858_R1B | 0x20)
#define AD4858_REG_CH_OR_STATUS             (AD4858_R1B | 0x21)
#define AD4858_REG_CH_UR_STATUS             (AD4858_R1B | 0x22)
#define AD4858_REG_REGMAP_CRC               (AD4858_R2B | 0x23)
#define AD4858_REG_DEVICE_CTRL              (AD4858_R1B | 0x25)
#define AD4858_REG_PACKET                   (AD4858_R1B | 0x26)
#define AD4858_REG_OVERSAMPLE               (AD4858_R1B | 0x27)
#define AD4858_REG_SEAMLESS_HDR             (AD4858_R1B | 0x28)
#define AD4858_REG_CH_SLEEP                 (AD4858_R1B | 0x29)
#define AD4858_REG_CH_SOFTSPAN(chn)         (AD4858_R1B | (0x2A + (0x12 * chn)))
#define AD4858_REG_CH_OFFSET(chn)           (AD4858_R3B | (0x2B + (0x12 * chn)))
#define AD4858_REG_CH_GAIN(chn)             (AD4858_R2B | (0x2E + (0x12 * chn)))
#define AD4858_REG_CH_PHASE(chn)            (AD4858_R2B | (0x30 + (0x12 * chn)))
#define AD4858_REG_CH_OR(chn)               (AD4858_R3B | (0x32 + (0x12 * chn)))
#define AD4858_REG_CH_UR(chn)               (AD4858_R3B | (0x35 + (0x12 * chn)))
#define AD4858_REG_CH_TESTPAT(chn)          (AD4858_R4B | (0x38 + (0x12 * chn)))

/** AD4858_REG_INTERFACE_CONFIG_A bit masks */
#define AD4858_SW_RESET_MSK         NO_OS_BIT(7) | NO_OS_BIT(0)
#define AD4858_SDO_ENABLE_MSK       NO_OS_BIT(4)
#define AD4858_ADDR_ASCENSION_MSK   NO_OS_BIT(5)

/** AD4858_REG_INTERFACE_CONFIG_B bit masks */
#define AD4858_SINGLE_INST_MSK      NO_OS_BIT(7)

/** AD4858_REG_DEVICE_CONFIG bit masks */
#define AD4858_OPERATING_MODES_MSK  NO_OS_GENMASK(1,0)
#define AD4858_STATUS_BIT0_MSK      NO_OS_BIT(4)
#define AD4858_STATUS_BIT1_MSK      NO_OS_BIT(5)
#define AD4858_STATUS_BIT2_MSK      NO_OS_BIT(6)
#define AD4858_STATUS_BIT3_MSK      NO_OS_BIT(7)

/** AD4858_REG_TRANSFER_CONFIG bit masks */
#define AD4858_KEEP_STRM_LEN_MSK    NO_OS_BIT(2)

/** AD4858_REG_INTERFACE_CONFIG_C bit masks */
#define AD4858_ACTIVE_INF_MODE_MSK  NO_OS_GENMASK(3,2)
#define AD4858_CRC_ENABLE_MSK       NO_OS_GENMASK(7,6)

/** AD4858_REG_INTERFACE_STATUS_A bit masks */
#define AD4858_ADDR_INVALID_ERR_MSK     NO_OS_BIT(0)
#define AD4858_WR_TO_RD_ONLY_ERR_MSK    NO_OS_BIT(2)
#define AD4858_CRC_ERR_MSK              NO_OS_BIT(3)
#define AD4858_CLK_COUNT_ERR_MSK        NO_OS_BIT(4)
#define AD4858_NOT_READY_ERR_MSK        NO_OS_BIT(7)

/** AD4858_REG_SPI_CONFIG_D bit masks */
#define AD4858_CSDO_ON_SDO_MSK      NO_OS_BIT(0)

/** AD4858_REG_PACKET bit masks */
#define AD4858_TEST_PATTERN_MSK     NO_OS_BIT(2)
#define AD4858_PACKET_FORMAT_MSK    NO_OS_GENMASK(1,0)

/** AD4858_REG_OVERSAMPLE bit masks */
#define AD4858_OS_ENABLE_MSK        NO_OS_BIT(7)
#define AD4858_OS_RATIO_MSK         NO_OS_GENMASK(3,0)

/** AD4858_REG_CH_SOFTSPAN bit masks */
#define AD4858_SOFTSPAN_MSK         NO_OS_GENMASK(3,0)

/** Status bit masks for 16-bit resolution ADC */
#define AD4858_OR_UR_STATUS_MSK_16_BIT	NO_OS_BIT(7)
#define AD4858_CHN_ID_MSK_16_BIT		NO_OS_GENMASK(6,4)
#define AD4858_SOFTSPAN_ID_MSK_16_BIT	NO_OS_GENMASK(3,0)

/** Status bit masks for 20-bit resolution ADC */
#define AD4858_OR_UR_STATUS_MSK_20_BIT	NO_OS_BIT(3)
#define AD4858_CHN_ID_MSK_20_BIT       	NO_OS_GENMASK(2,0)
#define AD4858_SOFTSPAN_ID_MSK_20_BIT  	NO_OS_GENMASK(7,4)

/** Raw data bit masks for 20-bit resolution ADC */
#define AD4858_RAW_DATA_MSK_20_BIT		NO_OS_GENMASK(23,4)
#define AD4858_RAW_DATA_MSK_EVEN_20_BIT	NO_OS_GENMASK(23,4)
#define AD4858_RAW_DATA_MSK_ODD_20_BIT 	NO_OS_GENMASK(19,0)

/** Miscellaneous Definitions */
#define AD4858_REG_RD_BIT_MSK       NO_OS_BIT(7)
#define AD4858_PRODUCT_ID_L		0x60
#define AD4857_PRODUCT_ID_L		0x61
#define AD4856_PRODUCT_ID_L		0x62
#define AD4855_PRODUCT_ID_L		0x63
#define AD4854_PRODUCT_ID_L		0x64
#define AD4853_PRODUCT_ID_L		0x65
#define AD4852_PRODUCT_ID_L		0x66
#define AD4851_PRODUCT_ID_L		0x67
#define AD4858I_PRODUCT_ID_L		0x6F
#define AD485X_PRODUCT_ID_H         0x00
#define AD4858_NUM_CHANNELS         8
#define AD4858_DEF_CHN_SOFTSPAN     0xf
#define AD4858_DEF_CHN_OFFSET       0x0
#define AD4858_DEF_CHN_GAIN         0x8000
#define AD4858_DEF_CHN_PHASE        0x0
#define AD4858_DEF_CHN_OR           0x7ffff0
#define AD4858_DEF_CHN_UR           0x800000

/***************************************************************************//**
 * @brief The `ad4858_prod_id` enumeration defines a set of constants
 * representing product IDs for various AD485X series devices. Each
 * enumerator is associated with a unique hexadecimal value that
 * identifies a specific product variant within the series. This
 * enumeration is used to facilitate the identification and
 * differentiation of these devices in software applications.
 *
 * @param AD4858_PROD_ID_L Represents the product ID for AD4858 with a value of
 * 0x60.
 * @param AD4857_PROD_ID_L Represents the product ID for AD4857 with a value of
 * 0x61.
 * @param AD4856_PROD_ID_L Represents the product ID for AD4856 with a value of
 * 0x62.
 * @param AD4855_PROD_ID_L Represents the product ID for AD4855 with a value of
 * 0x63.
 * @param AD4854_PROD_ID_L Represents the product ID for AD4854 with a value of
 * 0x64.
 * @param AD4853_PROD_ID_L Represents the product ID for AD4853 with a value of
 * 0x65.
 * @param AD4852_PROD_ID_L Represents the product ID for AD4852 with a value of
 * 0x66.
 * @param AD4851_PROD_ID_L Represents the product ID for AD4851 with a value of
 * 0x67.
 * @param AD4858I_PROD_ID_L Represents the product ID for AD4858I with a value
 * of 0x6F.
 ******************************************************************************/
enum ad4858_prod_id {
	AD4858_PROD_ID_L = 0x60,
	AD4857_PROD_ID_L = 0x61,
	AD4856_PROD_ID_L = 0x62,
	AD4855_PROD_ID_L = 0x63,
	AD4854_PROD_ID_L = 0x64,
	AD4853_PROD_ID_L = 0x65,
	AD4852_PROD_ID_L = 0x66,
	AD4851_PROD_ID_L = 0x67,
	AD4858I_PROD_ID_L = 0x6F,
};

/***************************************************************************//**
 * @brief The `ad4858_operating_mode` enumeration defines the different
 * operating modes available for the AD4858 device. It includes a normal
 * operating mode, a low power mode, and a constant representing the
 * total number of modes. This enumeration is used to configure the
 * device's power consumption and performance characteristics.
 *
 * @param AD4858_NORMAL_OP_MODE Represents the normal operating mode with a
 * value of 0x0.
 * @param AD4858_LOW_POWER_OP_MODE Represents the low power operating mode with
 * a value of 0x3.
 * @param AD4858_NUM_OF_OP_MODES Indicates the number of operating modes
 * available, with a value of 0x4.
 ******************************************************************************/
enum ad4858_operating_mode {
	AD4858_NORMAL_OP_MODE = 0x0,
	AD4858_LOW_POWER_OP_MODE = 0x3,
	AD4858_NUM_OF_OP_MODES = 0x4
};

/***************************************************************************//**
 * @brief The `ad4858_ch_sleep_value` enumeration defines the possible states
 * for enabling or disabling the sleep mode of a channel in the AD4858
 * device. It provides two options: `AD4858_SLEEP_DISABLE` to keep the
 * channel active and `AD4858_SLEEP_ENABLE` to put the channel into a
 * low-power sleep state. This enumeration is used to manage power
 * consumption by controlling the operational state of individual
 * channels.
 *
 * @param AD4858_SLEEP_DISABLE Represents the state where sleep mode is disabled
 * for a channel.
 * @param AD4858_SLEEP_ENABLE Represents the state where sleep mode is enabled
 * for a channel.
 ******************************************************************************/
enum ad4858_ch_sleep_value {
	AD4858_SLEEP_DISABLE,
	AD4858_SLEEP_ENABLE
};

/***************************************************************************//**
 * @brief The `ad4858_ch_seamless_hdr` enumeration defines two possible states
 * for the seamless high dynamic range (HDR) feature in the AD4858
 * device: disabled and enabled. This enumeration is used to configure
 * the HDR setting for each channel in the device, allowing for dynamic
 * range adjustments based on the application's requirements.
 *
 * @param AD4858_SEAMLESS_HDR_DISABLE Represents the state where seamless high
 * dynamic range is disabled.
 * @param AD4858_SEAMLESS_HDR_ENABLE Represents the state where seamless high
 * dynamic range is enabled.
 ******************************************************************************/
enum ad4858_ch_seamless_hdr {
	AD4858_SEAMLESS_HDR_DISABLE,
	AD4858_SEAMLESS_HDR_ENABLE
};

/***************************************************************************//**
 * @brief The `ad4858_interface_mode` enumeration defines the different
 * interface modes available for the AD4858 device, which include
 * configuration and data interface modes. This enumeration is used to
 * specify the mode in which the device operates, allowing for the
 * selection between configuring the device or handling data operations.
 * The enumeration also includes a member to represent the total number
 * of interface modes available.
 *
 * @param AD4858_CONFIG_INTERFACE_MODE Represents the configuration interface
 * mode for the AD4858 device.
 * @param AD4858_DATA_INTERFACE_MODE Represents the data interface mode for the
 * AD4858 device.
 * @param AD4858_NUM_OF_INTF_MODES Indicates the number of interface modes
 * available for the AD4858 device.
 ******************************************************************************/
enum ad4858_interface_mode {
	AD4858_CONFIG_INTERFACE_MODE,
	AD4858_DATA_INTERFACE_MODE,
	AD4858_NUM_OF_INTF_MODES
};

/***************************************************************************//**
 * @brief The `ad4858_spi_data_mode` is an enumeration that defines the
 * different modes of SPI data transfer for the AD4858 device. It
 * includes modes for streaming data continuously and executing single
 * instructions, providing flexibility in how data is communicated over
 * the SPI interface. The enumeration also includes a member to represent
 * the total number of SPI data modes available, which can be useful for
 * iterating over the modes or validating mode values.
 *
 * @param AD4858_STREAMING_MODE Represents the streaming mode for SPI data
 * transfer.
 * @param AD4858_SINGLE_INSTRUCTION_MODE Represents the single instruction mode
 * for SPI data transfer.
 * @param AD4858_NUM_OF_SPI_DATA_MODES Indicates the number of SPI data modes
 * available.
 ******************************************************************************/
enum ad4858_spi_data_mode {
	AD4858_STREAMING_MODE,
	AD4858_SINGLE_INSTRUCTION_MODE,
	AD4858_NUM_OF_SPI_DATA_MODES
};

/***************************************************************************//**
 * @brief The `ad4858_osr_ratio` enumeration defines a set of constants
 * representing different oversampling ratios for the AD4858 device.
 * These ratios are used to configure the oversampling settings of the
 * device, which can affect the resolution and noise performance of the
 * analog-to-digital conversion process. The enumeration provides a range
 * of oversampling options from 2 to 65536, allowing for flexible
 * configuration based on the application's requirements.
 *
 * @param AD4858_OSR_2 Represents an oversampling ratio of 2.
 * @param AD4858_OSR_4 Represents an oversampling ratio of 4.
 * @param AD4858_OSR_8 Represents an oversampling ratio of 8.
 * @param AD4858_OSR_16 Represents an oversampling ratio of 16.
 * @param AD4858_OSR_32 Represents an oversampling ratio of 32.
 * @param AD4858_OSR_64 Represents an oversampling ratio of 64.
 * @param AD4858_OSR_128 Represents an oversampling ratio of 128.
 * @param AD4858_OSR_256 Represents an oversampling ratio of 256.
 * @param AD4858_OSR_512 Represents an oversampling ratio of 512.
 * @param AD4858_OSR_1024 Represents an oversampling ratio of 1024.
 * @param AD4858_OSR_2048 Represents an oversampling ratio of 2048.
 * @param AD4858_OSR_4096 Represents an oversampling ratio of 4096.
 * @param AD4858_OSR_8192 Represents an oversampling ratio of 8192.
 * @param AD4858_OSR_16384 Represents an oversampling ratio of 16384.
 * @param AD4858_OSR_32768 Represents an oversampling ratio of 32768.
 * @param AD4858_OSR_65536 Represents an oversampling ratio of 65536.
 * @param AD4858_NUM_OF_OSR_RATIO Indicates the number of oversampling ratio
 * options available.
 ******************************************************************************/
enum ad4858_osr_ratio {
	AD4858_OSR_2,
	AD4858_OSR_4,
	AD4858_OSR_8,
	AD4858_OSR_16,
	AD4858_OSR_32,
	AD4858_OSR_64,
	AD4858_OSR_128,
	AD4858_OSR_256,
	AD4858_OSR_512,
	AD4858_OSR_1024,
	AD4858_OSR_2048,
	AD4858_OSR_4096,
	AD4858_OSR_8192,
	AD4858_OSR_16384,
	AD4858_OSR_32768,
	AD4858_OSR_65536,
	AD4858_NUM_OF_OSR_RATIO
};

/***************************************************************************//**
 * @brief The `ad4858_packet_format` enumeration defines the different packet
 * formats available for the AD4858 device, specifying the bit-width of
 * the data packets that can be used. This allows the device to handle
 * data in various bit resolutions, providing flexibility in data
 * representation and processing.
 *
 * @param AD4858_PACKET_16_BIT Represents a 16-bit packet format.
 * @param AD4858_PACKET_20_BIT Represents a 20-bit packet format.
 * @param AD4858_PACKET_24_BIT Represents a 24-bit packet format.
 * @param AD4858_PACKET_32_BIT Represents a 32-bit packet format.
 ******************************************************************************/
enum ad4858_packet_format {
	AD4858_PACKET_16_BIT,
	AD4858_PACKET_20_BIT,
	AD4858_PACKET_24_BIT,
	AD4858_PACKET_32_BIT
};

/***************************************************************************//**
 * @brief The `ad4858_chn_softspan` enumeration defines various voltage ranges
 * that can be configured for the AD4858 device's channels. Each
 * enumerator represents a specific voltage range, either unipolar or
 * bipolar, that the device can handle. This allows for flexible
 * configuration of the device to accommodate different input signal
 * ranges, enhancing its versatility in various applications.
 *
 * @param AD4858_RANGE_0V_TO_2_5V Represents a voltage range from 0V to 2.5V.
 * @param AD4858_RANGE_NEG_2_5V_TO_POS_2_5V Represents a voltage range from
 * -2.5V to +2.5V.
 * @param AD4858_RANGE_0V_TO_5_0V Represents a voltage range from 0V to 5.0V.
 * @param AD4858_RANGE_NEG_5_0V_TO_POS_5_0V Represents a voltage range from
 * -5.0V to +5.0V.
 * @param AD4858_RANGE_0V_TO_6_25V Represents a voltage range from 0V to 6.25V.
 * @param AD4858_RANGE_NEG_6_25V_TO_POS_6_25V Represents a voltage range from
 * -6.25V to +6.25V.
 * @param AD4858_RANGE_0V_TO_10_0V Represents a voltage range from 0V to 10.0V.
 * @param AD4858_RANGE_NEG_10_0V_TO_POS_10_0V Represents a voltage range from
 * -10.0V to +10.0V.
 * @param AD4858_RANGE_0V_TO_12_5V Represents a voltage range from 0V to 12.5V.
 * @param AD4858_RANGE_NEG_12_5V_TO_POS_12_5V Represents a voltage range from
 * -12.5V to +12.5V.
 * @param AD4858_RANGE_0V_TO_20_0V Represents a voltage range from 0V to 20.0V.
 * @param AD4858_RANGE_NEG_20_0V_TO_POS_20_0V Represents a voltage range from
 * -20.0V to +20.0V.
 * @param AD4858_RANGE_0V_TO_25_0V Represents a voltage range from 0V to 25.0V.
 * @param AD4858_RANGE_NEG_25_0V_TO_POS_25_0V Represents a voltage range from
 * -25.0V to +25.0V.
 * @param AD4858_RANGE_0V_TO_40_0V Represents a voltage range from 0V to 40.0V.
 * @param AD4858_RANGE_NEG_40_0V_TO_POS_40_0V Represents a voltage range from
 * -40.0V to +40.0V.
 * @param AD4858_NUM_OF_SOFTSPAN Indicates the number of softspan ranges
 * available.
 ******************************************************************************/
enum ad4858_chn_softspan {
	AD4858_RANGE_0V_TO_2_5V,
	AD4858_RANGE_NEG_2_5V_TO_POS_2_5V,
	AD4858_RANGE_0V_TO_5_0V,
	AD4858_RANGE_NEG_5_0V_TO_POS_5_0V,
	AD4858_RANGE_0V_TO_6_25V,
	AD4858_RANGE_NEG_6_25V_TO_POS_6_25V,
	AD4858_RANGE_0V_TO_10_0V,
	AD4858_RANGE_NEG_10_0V_TO_POS_10_0V,
	AD4858_RANGE_0V_TO_12_5V,
	AD4858_RANGE_NEG_12_5V_TO_POS_12_5V,
	AD4858_RANGE_0V_TO_20_0V,
	AD4858_RANGE_NEG_20_0V_TO_POS_20_0V,
	AD4858_RANGE_0V_TO_25_0V,
	AD4858_RANGE_NEG_25_0V_TO_POS_25_0V,
	AD4858_RANGE_0V_TO_40_0V,
	AD4858_RANGE_NEG_40_0V_TO_POS_40_0V,
	AD4858_NUM_OF_SOFTSPAN
};

/***************************************************************************//**
 * @brief The `ad4858_conv_data` structure is designed to store the conversion
 * data from an AD4858 ADC device. It includes arrays for raw ADC data,
 * OR/UR status, channel IDs, and softspan IDs, each corresponding to
 * multiple channels as defined by `AD4858_NUM_CHANNELS`. This structure
 * is essential for managing and accessing the conversion results and
 * status information for each channel in the ADC.
 *
 * @param raw An array holding 20-bit ADC conversion raw data for each channel.
 * @param or_ur_status An array indicating the 1-bit OR/UR status for each
 * channel.
 * @param chn_id An array holding the 3-bit channel ID for each channel.
 * @param softspan_id An array holding the 4-bit softspan ID for each channel.
 ******************************************************************************/
struct ad4858_conv_data {
	/* 20-bit ADC conversion raw data */
	uint32_t raw[AD4858_NUM_CHANNELS];
	/* 1-bit OR/UR status */
	bool or_ur_status[AD4858_NUM_CHANNELS];
	/* 3-bit channel ID */
	uint8_t chn_id[AD4858_NUM_CHANNELS];
	/* 4-bit softspan ID */
	uint32_t softspan_id[AD4858_NUM_CHANNELS];
};

/***************************************************************************//**
 * @brief The `ad4858_init_param` structure is used to initialize the AD4858
 * device, encapsulating various configuration parameters such as SPI and
 * GPIO settings, product ID, operating modes, and channel-specific
 * settings like softspan, offset, gain, phase, and range limits. It
 * provides a comprehensive setup for the AD4858 device, allowing for
 * detailed customization of its operation and data handling
 * capabilities, including enabling features like oversampling, test
 * patterns, and seamless high dynamic range (HDR) for each channel.
 *
 * @param spi_init Pointer to the SPI configuration for the host processor.
 * @param gpio_pd Pointer to the GPIO configuration for power down.
 * @param gpio_cmos_lvds Pointer to the GPIO configuration for LVDS/CMOS
 * selection.
 * @param gpio_cnv Pointer to the GPIO configuration for conversion start.
 * @param gpio_busy Pointer to the GPIO configuration for busy status.
 * @param prod_id Specifies the AD458X product ID.
 * @param addr_ascension_enable Boolean flag to enable address ascension.
 * @param operating_mode Specifies the operating mode of the device.
 * @param osr_enable Boolean flag to enable oversampling ratio (OSR).
 * @param osr_ratio Specifies the oversampling ratio.
 * @param packet_format Specifies the packet format for data transmission.
 * @param test_pattern Boolean flag to enable or disable test pattern.
 * @param use_default_chn_configs Boolean flag to use default or reset channel
 * configurations.
 * @param chn_softspan Array specifying the softspan value for each channel.
 * @param chn_offset Array specifying the offset value for each channel.
 * @param chn_gain Array specifying the gain value for each channel.
 * @param chn_phase Array specifying the phase value for each channel.
 * @param chn_or Array specifying the overrange limit value for each channel.
 * @param chn_ur Array specifying the underrange limit value for each channel.
 * @param chn_sleep_value Array specifying the sleep value for each channel.
 * @param chn_seamless_hdr Array specifying the seamless HDR value for each
 * channel.
 ******************************************************************************/
struct ad4858_init_param {
	/** Host processor SPI configuration. */
	struct no_os_spi_init_param *spi_init;
	/** Power Down GPIO configuration. */
	struct no_os_gpio_init_param *gpio_pd;
	/** LVDS/CMOS select GPIO configuration. */
	struct no_os_gpio_init_param *gpio_cmos_lvds;
	/** Conversion Start GPIO configuration. */
	struct no_os_gpio_init_param *gpio_cnv;
	/** Busy GPIO configuration. */
	struct no_os_gpio_init_param *gpio_busy;
	/** AD458X Product ID */
	enum ad4858_prod_id prod_id;
	/** Enable address ascension. */
	bool addr_ascension_enable;
	/** Operating mode. */
	enum ad4858_operating_mode operating_mode;
	/** Enable OSR. */
	bool osr_enable;
	/** OSR ratio */
	enum ad4858_osr_ratio osr_ratio;
	/** Packet format */
	enum ad4858_packet_format packet_format;
	/** Test pattern enable/disable status flag. */
	bool test_pattern;
	/** Use default/reset channel configs */
	bool use_default_chn_configs;
	/** Channel softspan value */
	enum ad4858_chn_softspan chn_softspan[AD4858_NUM_CHANNELS];
	/** Channel offset value */
	uint32_t chn_offset[AD4858_NUM_CHANNELS];
	/** Channel gain value */
	uint16_t chn_gain[AD4858_NUM_CHANNELS];
	/** Channel phase value */
	uint16_t chn_phase[AD4858_NUM_CHANNELS];
	/** Channel overrange limit value */
	uint16_t chn_or[AD4858_NUM_CHANNELS];
	/** Channel underrange limit value */
	uint16_t chn_ur[AD4858_NUM_CHANNELS];
	/** Channel sleep value */
	enum ad4858_ch_sleep_value chn_sleep_value[AD4858_NUM_CHANNELS];
	/** Channel seamless HDR value */
	enum ad4858_ch_seamless_hdr chn_seamless_hdr[AD4858_NUM_CHANNELS];
};

/***************************************************************************//**
 * @brief The `ad4858_dev` structure is a comprehensive descriptor for the
 * AD4858 device, encapsulating all necessary configurations and states
 * required for its operation. It includes pointers to SPI and GPIO
 * descriptors for communication and control, as well as various
 * configuration parameters such as product ID, operating mode, and data
 * format settings. The structure also manages channel-specific settings
 * like softspan, offset, gain, phase, and range limits, allowing for
 * detailed customization of each channel's behavior. Additionally, it
 * supports features like oversampling, test patterns, and seamless HDR,
 * making it a versatile and powerful tool for managing the AD4858
 * device.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_pd Pointer to the GPIO descriptor for power down control.
 * @param gpio_cmos_lvds Pointer to the GPIO descriptor for LVDS/CMOS selection.
 * @param gpio_cnv Pointer to the GPIO descriptor for conversion start control.
 * @param gpio_busy Pointer to the GPIO descriptor for busy status indication.
 * @param prod_id Enumeration for the AD458X product ID.
 * @param addr_ascension_enable Boolean flag indicating if address ascension is
 * enabled.
 * @param operating_mode Enumeration for the operating mode of the device.
 * @param spi_data_mode Enumeration for the SPI data mode.
 * @param osr_enable Boolean flag indicating if oversampling ratio (OSR) is
 * enabled.
 * @param osr_ratio Enumeration for the oversampling ratio.
 * @param packet_format Enumeration for the packet format.
 * @param test_pattern Boolean flag indicating if the test pattern is enabled.
 * @param chn_softspan Array of enumerations for the channel softspan values.
 * @param chn_offset Array of 32-bit unsigned integers for the channel offset
 * values.
 * @param chn_gain Array of 16-bit unsigned integers for the channel gain
 * values.
 * @param chn_phase Array of 16-bit unsigned integers for the channel phase
 * values.
 * @param chn_or Array of 16-bit unsigned integers for the channel overrange
 * limit values.
 * @param chn_ur Array of 16-bit unsigned integers for the channel underrange
 * limit values.
 * @param big_endian Boolean flag indicating if big endian format is used.
 * @param chn_sleep_value Array of enumerations for the channel sleep values.
 * @param chn_seamless_hdr Array of enumerations for the channel seamless HDR
 * values.
 ******************************************************************************/
struct ad4858_dev {
	/** SPI descriptor. */
	struct no_os_spi_desc *spi_desc;
	/** Power Down GPIO descriptor. */
	struct no_os_gpio_desc *gpio_pd;
	/** LVDS/CMOS select GPIO descriptor. */
	struct no_os_gpio_desc *gpio_cmos_lvds;
	/** Conversion Start GPIO descriptor. */
	struct no_os_gpio_desc *gpio_cnv;
	/** Busy GPIO descriptor. */
	struct no_os_gpio_desc *gpio_busy;
	/** AD458X Product ID */
	enum ad4858_prod_id prod_id;
	/** Address ascension enable status. */
	bool addr_ascension_enable;
	/** Operating mode. */
	enum ad4858_operating_mode operating_mode;
	/** SPI data mode. */
	enum ad4858_spi_data_mode spi_data_mode;
	/** OSR enable status. */
	bool osr_enable;
	/** OSR ratio */
	enum ad4858_osr_ratio osr_ratio;
	/** Packet format */
	enum ad4858_packet_format packet_format;
	/** Test pattern enable/disable status flag. */
	bool test_pattern;
	/** Channel softspan value */
	enum ad4858_chn_softspan chn_softspan[AD4858_NUM_CHANNELS];
	/** Channel offset value */
	uint32_t chn_offset[AD4858_NUM_CHANNELS];
	/** Channel gain value */
	uint16_t chn_gain[AD4858_NUM_CHANNELS];
	/** Channel phase value */
	uint16_t chn_phase[AD4858_NUM_CHANNELS];
	/** Channel overrange limit value */
	uint16_t chn_or[AD4858_NUM_CHANNELS];
	/** Channel underrange limit value */
	uint16_t chn_ur[AD4858_NUM_CHANNELS];
	/** Big endianess status flag */
	bool big_endian;
	/** Channel sleep value */
	enum ad4858_ch_sleep_value chn_sleep_value[AD4858_NUM_CHANNELS];
	/** Channel seamless HDR value */
	enum ad4858_ch_seamless_hdr chn_seamless_hdr[AD4858_NUM_CHANNELS];
};

/* Initialize the device */
/***************************************************************************//**
 * @brief This function sets up the AD4858 device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * device. The function checks for valid input pointers and returns an
 * error if they are null. It also performs a series of configuration
 * steps, including GPIO setup, SPI initialization, and device reset. If
 * any step fails, the function cleans up allocated resources and returns
 * an error code. Successful initialization results in a pointer to the
 * device structure being set.
 *
 * @param device A pointer to a pointer of type `struct ad4858_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad4858_init_param` containing the
 * initialization parameters for the device. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, `device` is set to point to the initialized device
 * structure.
 ******************************************************************************/
int ad4858_init(struct ad4858_dev **device,
		struct ad4858_init_param *init_param);

/* Remove the device */
/***************************************************************************//**
 * @brief This function is used to properly remove and deallocate resources
 * associated with an AD4858 device. It should be called when the device
 * is no longer needed to ensure that all associated resources, such as
 * GPIO and SPI descriptors, are released. The function must be called
 * with a valid device descriptor that was previously initialized. If the
 * device descriptor is null, the function returns an error code. It is
 * important to handle the return value to check for any errors during
 * the removal process.
 *
 * @param dev A pointer to an ad4858_dev structure representing the device to be
 * removed. Must not be null. If null, the function returns -EINVAL.
 * @return Returns 0 on successful removal, or a negative error code if an error
 * occurs during the removal process.
 ******************************************************************************/
int ad4858_remove(struct ad4858_dev *dev);

/* Write device register */
/***************************************************************************//**
 * @brief This function is used to write a 32-bit value to a specific register
 * of the AD4858 device. It requires a valid device descriptor, which
 * must be initialized before calling this function. The function handles
 * both big-endian and little-endian data formats based on the device
 * configuration. It also considers the address ascension setting of the
 * device. The function returns an error code if the device descriptor is
 * null or if the SPI write operation fails. This function is typically
 * used to configure device registers during initialization or runtime
 * configuration changes.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. The function returns -EINVAL if this
 * parameter is null.
 * @param reg_addr A 32-bit unsigned integer representing the register address.
 * The address should be a valid register address defined for
 * the AD4858 device.
 * @param reg_val A 32-bit unsigned integer representing the value to be written
 * to the specified register. The value should be formatted
 * according to the register's expected data size and format.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input or an error code from the SPI write
 * operation.
 ******************************************************************************/
int ad4858_reg_write(struct ad4858_dev *dev, uint32_t reg_addr,
		     uint32_t reg_val);

/* Read device register */
/***************************************************************************//**
 * @brief Use this function to read the value of a specified register from an
 * AD4858 device. It requires a valid device descriptor and a register
 * address to function correctly. The function will store the read value
 * in the provided memory location pointed to by reg_val. Ensure that the
 * device has been properly initialized before calling this function. The
 * function handles both big-endian and little-endian data formats based
 * on the device configuration. It returns an error code if the device
 * descriptor or reg_val is null, or if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null.
 * @param reg_addr A 32-bit unsigned integer representing the register address
 * to read from. The address must be valid for the device.
 * @param reg_val A pointer to a 32-bit unsigned integer where the read register
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters or an error code from the SPI
 * communication.
 ******************************************************************************/
int ad4858_reg_read(struct ad4858_dev *dev, uint32_t reg_addr,
		    uint32_t *reg_val);

/* Update specific register bits of an input register */
/***************************************************************************//**
 * @brief Use this function to modify specific bits of a register in the AD4858
 * device by applying a mask and setting new values. This function is
 * useful when only certain bits of a register need to be changed without
 * affecting the other bits. It must be called with a valid device
 * descriptor, and the register address must be within the valid range
 * for the device. The function reads the current register value, applies
 * the mask to clear specific bits, and then sets the new values before
 * writing it back. It returns an error code if the device descriptor is
 * null or if the read/write operations fail.
 *
 * @param dev Pointer to an ad4858_dev structure representing the device. Must
 * not be null. The function returns -EINVAL if this parameter is
 * null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD4858 device.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param reg_val The new values to set in the register, after applying the
 * mask. Only the bits specified by the mask will be updated with
 * these values.
 * @return Returns 0 on success, or a negative error code on failure (e.g.,
 * -EINVAL for null device, or other error codes for read/write
 * failures).
 ******************************************************************************/
int ad4858_reg_mask(struct ad4858_dev *dev,
		    uint32_t reg_addr,
		    uint32_t mask,
		    uint32_t reg_val);

/* Software reset of the device */
/***************************************************************************//**
 * @brief This function is used to perform a software reset on the AD4858
 * device, which resets the device and sets the SPI mode to 4-wire. It
 * should be called when a reset of the device is required, such as
 * during initialization or to recover from an error state. The function
 * requires a valid device descriptor and will return an error if the
 * descriptor is null. A delay is introduced after the reset to ensure
 * the device has time to stabilize.
 *
 * @param dev A pointer to an ad4858_dev structure representing the device. Must
 * not be null. If null, the function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ad4858_soft_reset(struct ad4858_dev *dev);

/* Set operating mode */
/***************************************************************************//**
 * @brief Use this function to configure the operating mode of an AD4858 device.
 * It should be called when you need to change the device's mode of
 * operation, such as switching between normal and low power modes.
 * Ensure that the device has been properly initialized before calling
 * this function. The function validates the input parameters and returns
 * an error if the device pointer is null or if the mode is invalid. It
 * updates the device's configuration register and the internal state to
 * reflect the new operating mode.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. The function will return an error if
 * this parameter is null.
 * @param mode An enum value of type ad4858_operating_mode representing the
 * desired operating mode. Valid values are defined by the
 * ad4858_operating_mode enumeration. If the mode is out of range,
 * the function returns an error.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -EINVAL for invalid parameters).
 ******************************************************************************/
int ad4858_set_operating_mode(struct ad4858_dev *dev,
			      enum ad4858_operating_mode mode);

/* Set SPI data mode */
/***************************************************************************//**
 * @brief This function configures the SPI data mode of the AD4858 device, which
 * determines how data is communicated over the SPI interface. It should
 * be called after the device has been initialized and before any SPI
 * data transactions are performed. The function validates the input
 * parameters and returns an error if the device pointer is null or if
 * the mode is invalid. Successful execution updates the device's
 * internal state to reflect the new SPI data mode.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. The function will return an error if
 * this parameter is null.
 * @param mode An enumerated value of type ad4858_spi_data_mode representing the
 * desired SPI data mode. Valid values are AD4858_STREAMING_MODE and
 * AD4858_SINGLE_INSTRUCTION_MODE. If the mode is invalid, the
 * function returns an error.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or if an error occurs during the operation.
 ******************************************************************************/
int ad4858_set_spi_data_mode(struct ad4858_dev *dev,
			     enum ad4858_spi_data_mode mode);

/* Set device config interface mode. */
/***************************************************************************//**
 * @brief This function sets the AD4858 device to operate in configuration
 * interface mode, which is necessary for setting up the device's
 * registers via SPI. It should be called when the device needs to be
 * configured for register access using a 4-wire SPI interface. The
 * function must be called with a valid device descriptor, and it will
 * return an error if the descriptor is null. This function is typically
 * used during the initialization phase of the device setup.
 *
 * @param dev A pointer to an ad4858_dev structure representing the device. Must
 * not be null. The function will return -EINVAL if this parameter is
 * null.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int ad4858_set_config_interface_mode(struct ad4858_dev *dev);

/* Set device data interface mode. */
/***************************************************************************//**
 * @brief This function sets the AD4858 device to operate in data interface
 * mode, which involves configuring the SPI data mode to single
 * instruction mode and disabling the SDO line to achieve a 3-wire SPI
 * configuration. It should be called when the device needs to switch to
 * data interface mode after initialization. The function requires a
 * valid device descriptor and will return an error if the descriptor is
 * null or if any configuration step fails.
 *
 * @param dev A pointer to an ad4858_dev structure representing the device. Must
 * not be null. The function will return -EINVAL if this parameter is
 * null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad4858_set_data_interface_mode(struct ad4858_dev *dev);

/* Enable/Disable OSR */
/***************************************************************************//**
 * @brief This function is used to control the oversampling ratio (OSR) feature
 * of the AD4858 device. It should be called when you need to enable or
 * disable the OSR functionality, which can be useful for improving
 * signal quality or reducing noise. The function must be called with a
 * valid device descriptor that has been properly initialized. If the
 * device descriptor is null, the function will return an error. The
 * function updates the device's internal state to reflect the new OSR
 * status.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param osr_status A boolean value indicating whether to enable (true) or
 * disable (false) the OSR feature.
 * @return Returns 0 on success. If the device descriptor is null, returns
 * -EINVAL. If there is an error updating the device register, returns a
 * non-zero error code.
 ******************************************************************************/
int ad4858_enable_osr(struct ad4858_dev *dev, bool osr_status);

/* Set OSR ratio */
/***************************************************************************//**
 * @brief This function configures the oversampling ratio (OSR) for the AD4858
 * device, which can affect the resolution and noise performance of the
 * ADC. It should be called after the device has been initialized and
 * before starting any conversions. The function validates the input
 * parameters and updates the device's configuration register
 * accordingly. If the parameters are invalid, it returns an error code.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. The function will return -EINVAL if this
 * parameter is null.
 * @param osr_ratio An enum value of type ad4858_osr_ratio representing the
 * desired oversampling ratio. Must be a valid enum value less
 * than AD4858_NUM_OF_OSR_RATIO. If the value is invalid, the
 * function returns -EINVAL.
 * @return Returns 0 on success. On failure, returns a negative error code, such
 * as -EINVAL for invalid parameters.
 ******************************************************************************/
int ad4858_set_osr_ratio(struct ad4858_dev *dev,
			 enum ad4858_osr_ratio osr_ratio);

/* Set packet format */
/***************************************************************************//**
 * @brief This function configures the packet format of the AD4858 device, which
 * determines the bit width of the data packets. It should be called
 * after the device has been initialized and before any data transactions
 * that depend on the packet format. The function checks the device's
 * product ID to ensure compatibility with the specified packet format
 * and returns an error if the format is not supported. It updates the
 * device's internal state to reflect the new packet format.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. The function will return an error if
 * this parameter is invalid.
 * @param packet_format An enum value of type ad4858_packet_format specifying
 * the desired packet format. The valid formats depend on
 * the device's product ID, and unsupported formats will
 * result in an error.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null, the packet format is invalid for the device, or if there is
 * a failure in updating the device's register.
 ******************************************************************************/
int ad4858_set_packet_format(struct ad4858_dev *dev,
			     enum ad4858_packet_format packet_format);

/* Enable/Disable test pattern on ADC data output. */
/***************************************************************************//**
 * @brief Use this function to control the test pattern feature of the AD4858
 * device, which can be enabled or disabled based on the provided flag.
 * This function should be called when you need to verify the data output
 * of the ADC by using a known test pattern. Ensure that the device is
 * properly initialized before calling this function. The function
 * updates the device's internal state to reflect the test pattern
 * status.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param test_pattern A boolean value indicating whether to enable (true) or
 * disable (false) the test pattern on the ADC data output.
 * @return Returns 0 on success. If the device pointer is null, returns -EINVAL.
 * If there is an error updating the register, returns the error code
 * from the register update operation.
 ******************************************************************************/
int ad4858_enable_test_pattern(struct ad4858_dev *dev, bool test_pattern);

/* Set channel softspan */
/***************************************************************************//**
 * @brief This function configures the softspan setting for a specific channel
 * on the AD4858 device, which determines the input voltage range for
 * that channel. It should be called when you need to adjust the voltage
 * range for a channel to match the expected input signal levels. The
 * function requires a valid device descriptor and channel number, and
 * the channel number must be within the supported range. The softspan
 * value must be a valid enumeration value representing the desired
 * voltage range. If the inputs are invalid, the function returns an
 * error code.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null.
 * @param chn The channel number to configure. Must be less than
 * AD4858_NUM_CHANNELS.
 * @param chn_softspan An enumeration value of type ad4858_chn_softspan
 * representing the desired softspan setting. Must be less
 * than AD4858_NUM_OF_SOFTSPAN.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int ad4858_set_chn_softspan(struct ad4858_dev *dev, uint8_t chn,
			    enum ad4858_chn_softspan chn_softspan);

/* Set channel offset */
/***************************************************************************//**
 * @brief This function is used to configure the offset for a specific channel
 * on an AD4858 device. It should be called when you need to adjust the
 * offset for a channel, which can be necessary for calibration or
 * specific application requirements. The function requires a valid
 * device descriptor and a channel number within the supported range. It
 * handles different product IDs by adjusting the offset bit span
 * accordingly. If the device descriptor is null or the channel number is
 * out of range, the function returns an error. Successful execution
 * updates the device's internal offset configuration for the specified
 * channel.
 *
 * @param dev A pointer to an ad4858_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param chn The channel number for which the offset is to be set. Must be less
 * than AD4858_NUM_CHANNELS. If out of range, the function returns an
 * error.
 * @param offset The offset value to set for the specified channel. The function
 * adjusts the bit span based on the device's product ID.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or the channel number is invalid.
 ******************************************************************************/
int ad4858_set_chn_offset(struct ad4858_dev *dev, uint8_t chn,
			  uint32_t offset);

/* Set channel gain */
/***************************************************************************//**
 * @brief This function is used to configure the gain for a specific channel on
 * the AD4858 device. It should be called when you need to adjust the
 * gain settings for a channel, typically after initializing the device.
 * The function requires a valid device descriptor and a channel number
 * within the supported range. If the device descriptor is null or the
 * channel number is out of range, the function returns an error.
 * Successful execution updates the gain setting for the specified
 * channel.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param chn The channel number for which the gain is to be set. Must be less
 * than AD4858_NUM_CHANNELS. If the channel number is out of range,
 * the function returns an error.
 * @param gain The gain value to set for the specified channel. It is a 16-bit
 * unsigned integer representing the desired gain setting.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., invalid parameters).
 ******************************************************************************/
int ad4858_set_chn_gain(struct ad4858_dev *dev, uint8_t chn, uint16_t gain);

/* Set channel phase */
/***************************************************************************//**
 * @brief This function is used to configure the phase setting for a specific
 * channel on the AD4858 device. It should be called when you need to
 * adjust the phase of a channel, typically after the device has been
 * initialized. The function requires a valid device descriptor and a
 * channel number within the supported range. If the device descriptor is
 * null or the channel number is out of range, the function returns an
 * error. Successful execution updates the phase setting for the
 * specified channel.
 *
 * @param dev A pointer to an ad4858_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param chn The channel number for which the phase is to be set. Must be less
 * than AD4858_NUM_CHANNELS. If out of range, the function returns an
 * error.
 * @param phase The phase value to set for the specified channel. It is a 16-bit
 * unsigned integer.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error.
 ******************************************************************************/
int ad4858_set_chn_phase(struct ad4858_dev *dev, uint8_t chn,
			 uint16_t phase);

/* Set channel overrange (OR) limit */
/***************************************************************************//**
 * @brief This function is used to configure the overrange limit for a specific
 * channel on the AD4858 device. It should be called when you need to set
 * or update the overrange limit for a channel, which is useful for
 * managing signal thresholds. The function requires a valid device
 * descriptor and a channel number within the supported range. If the
 * device descriptor is null or the channel number is out of range, the
 * function returns an error. The overrange limit is applied by writing
 * to the appropriate register, and the function updates the device's
 * internal state to reflect the new limit.
 *
 * @param dev A pointer to an ad4858_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param chn The channel number for which the overrange limit is being set.
 * Must be less than AD4858_NUM_CHANNELS. If out of range, the
 * function returns an error.
 * @param or_limit The overrange limit value to set for the specified channel.
 * The value is shifted and written to the device register.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or the channel number is invalid.
 ******************************************************************************/
int ad4858_set_chn_or_limit(struct ad4858_dev *dev, uint8_t chn,
			    uint32_t or_limit);

/* Set channel underrange (UR) limit */
/***************************************************************************//**
 * @brief This function is used to configure the underrange limit for a specific
 * channel on the AD4858 device. It should be called when you need to set
 * or update the underrange limit for a channel, ensuring that the device
 * is properly initialized and the channel index is within the valid
 * range. The function will return an error if the device pointer is null
 * or if the channel index is out of bounds. It updates the device's
 * internal state to reflect the new underrange limit.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null.
 * @param chn The index of the channel for which the underrange limit is being
 * set. Must be less than AD4858_NUM_CHANNELS.
 * @param ur_limit The underrange limit value to set for the specified channel.
 * The value is shifted and stored internally.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or the channel index is invalid.
 ******************************************************************************/
int ad4858_set_chn_ur_limit(struct ad4858_dev *dev, uint8_t chn,
			    uint32_t ur_limit);

/* Toggle the CNV pin to start a conversion. */
/***************************************************************************//**
 * @brief This function is used to initiate a conversion on the AD4858 device by
 * toggling the CNV (conversion start) GPIO pin. It should be called when
 * a conversion is required, typically after the device has been properly
 * initialized. The function first sets the CNV pin high and then low,
 * with a minimum high time of 40 nanoseconds, although no explicit delay
 * is added. It is important to ensure that the `dev` parameter is not
 * null before calling this function, as a null device pointer will
 * result in an error. The function returns an error code if the GPIO
 * operation fails.
 *
 * @param dev A pointer to an `ad4858_dev` structure representing the device.
 * Must not be null. The caller retains ownership. If null, the
 * function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code if the GPIO operation
 * fails.
 ******************************************************************************/
int ad4858_convst(struct ad4858_dev *dev);

/* Perform ADC conversion. */
/***************************************************************************//**
 * @brief This function initiates an analog-to-digital conversion on the AD4858
 * device by toggling the conversion start pin and monitoring the busy
 * GPIO pin to determine when the conversion is complete. It should be
 * called when a conversion is needed, and the device must be properly
 * initialized before calling this function. The function will return an
 * error code if the conversion start fails or if the busy pin does not
 * indicate completion within a specified timeout period.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if any GPIO operations fail.
 * @return Returns 0 on successful conversion completion. Returns a negative
 * error code if the conversion start fails, if the busy GPIO read
 * fails, or if the conversion does not complete within the timeout
 * period.
 ******************************************************************************/
int ad4858_perform_conv(struct ad4858_dev *dev);

/* Read ADC conversion data over SPI. */
/***************************************************************************//**
 * @brief This function reads ADC conversion data from the AD4858 device over
 * SPI and populates the provided data structure with the conversion
 * results. It must be called after the device has been properly
 * initialized and configured. The function handles different packet
 * formats and product IDs, ensuring the data is correctly interpreted
 * based on the device's configuration. It returns an error code if the
 * device or data pointers are null, or if the packet format is invalid.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. The function will return -EINVAL if this
 * parameter is null.
 * @param data A pointer to an ad4858_conv_data structure where the conversion
 * data will be stored. Must not be null. The function will return
 * -EINVAL if this parameter is null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters or other errors from the SPI read
 * operation.
 ******************************************************************************/
int ad4858_spi_data_read(struct ad4858_dev *dev, struct ad4858_conv_data *data);

/* Perform conversion and read ADC data (for all channels). */
/***************************************************************************//**
 * @brief This function is used to perform an ADC conversion and read the
 * resulting data for all channels of the AD4858 device. It should be
 * called when a complete set of conversion data is required from the
 * device. The function requires a valid device descriptor and a data
 * structure to store the conversion results. It is important to ensure
 * that the device has been properly initialized before calling this
 * function. The function will return an error code if the device or data
 * pointers are null, or if the conversion or data reading process fails.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param data A pointer to an ad4858_conv_data structure where the conversion
 * data will be stored. Must not be null. The caller retains
 * ownership.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL if the input pointers are null.
 ******************************************************************************/
int ad4858_read_data(struct ad4858_dev *dev, struct ad4858_conv_data *data);

/* Enable/Disable channel sleep */
/***************************************************************************//**
 * @brief This function is used to enable or disable the sleep mode for a
 * specific channel on the AD4858 device. It should be called when you
 * need to manage the power state of individual channels, for instance,
 * to save power by disabling unused channels. The function requires a
 * valid device descriptor and a channel number within the valid range.
 * If the device descriptor is null or the channel number is out of
 * range, the function returns an error. It updates the device's internal
 * state to reflect the new sleep status of the specified channel.
 *
 * @param dev A pointer to an ad4858_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param chn The channel number to configure. Must be less than
 * AD4858_NUM_CHANNELS. If out of range, the function returns an
 * error.
 * @param sleep_status An enum value of type ad4858_ch_sleep_value indicating
 * whether to enable or disable sleep mode for the channel.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or the channel number is invalid.
 ******************************************************************************/
int ad4858_enable_ch_sleep(struct ad4858_dev* dev, uint8_t chn,
			   enum ad4858_ch_sleep_value sleep_status);

/* Enable/Disable seamless HDR */
/***************************************************************************//**
 * @brief This function is used to enable or disable the seamless high dynamic
 * range (HDR) feature for a specific channel on the AD4858 device. It
 * should be called when you need to adjust the HDR settings for a
 * channel, typically after the device has been initialized. The function
 * requires a valid device descriptor and a channel number within the
 * valid range. If the device descriptor is null or the channel number is
 * out of range, the function returns an error. The seamless HDR status
 * is updated in the device's internal state and the corresponding
 * register is modified to reflect the change.
 *
 * @param dev A pointer to an initialized ad4858_dev structure representing the
 * device. Must not be null.
 * @param chn The channel number for which to set the seamless HDR status. Must
 * be less than AD4858_NUM_CHANNELS.
 * @param seamless_hdr_status An enum value of type ad4858_ch_seamless_hdr
 * indicating whether to enable or disable seamless
 * HDR for the specified channel.
 * @return Returns 0 on success, or a negative error code if the device is null,
 * the channel number is invalid, or if there is a failure in reading or
 * writing the device register.
 ******************************************************************************/
int ad4858_enable_ch_seamless_hdr(struct ad4858_dev* dev, uint8_t chn,
				  enum ad4858_ch_seamless_hdr seamless_hdr_status);

#endif	// AD4858_H_
