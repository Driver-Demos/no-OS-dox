/***************************************************************************//**
 *   @file   ad7606.h
 *   @brief  Header file for the ad7606 Driver.
 *   @author Stefan Popa (stefan.popa@analog.com)
 *   @author Darius Berghe (darius.berghe@analog.com)
********************************************************************************
 * Copyright 2019, 2021(c) Analog Devices, Inc.
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
#ifndef AD7606_H_
#define AD7606_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"

#include "no_os_pwm.h"
#include "clk_axi_clkgen.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD7606_REG_STATUS			0x01
#define AD7606_REG_CONFIG			0x02
#define AD7606_REG_RANGE_CH_ADDR(ch)		(0x03 + ((ch) >> 1))
#define AD7606_REG_BANDWIDTH			0x07
#define AD7606_REG_OVERSAMPLING			0x08
#define AD7606_REG_GAIN_CH(ch)			(0x09 + (ch))
#define AD7606_REG_OFFSET_CH(ch)		(0x11 + (ch))
#define AD7606_REG_PHASE_CH(ch)			(0x19 + (ch))
#define AD7606_REG_DIGITAL_DIAG_ENABLE		0x21
#define AD7606_REG_DIGITAL_DIAG_ERR		0x22
#define AD7606_REG_OPEN_DETECT_ENABLE		0x23
#define AD7606_REG_OPEN_DETECTED		0x24
#define AD7606_REG_AIN_OV_UV_DIAG_ENABLE	0x25
#define AD7606_REG_AIN_OV_DIAG_ERROR		0x26
#define AD7606_REG_AIN_UV_DIAG_ERROR		0x27
#define AD7606_REG_DIAGNOSTIC_MUX_CH(ch)	(0x28 + ((ch) >> 1))
#define AD7606_REG_OPEN_DETECT_QUEUE		0x2C
#define AD7606_REG_CLK_FS_COUNTER		0x2D
#define AD7606_REG_CLK_OS_COUNTER		0x2E
#define AD7606_REG_ID				0x2F

/* AD7606_REG_STATUS */
#define AD7606_STATUS_CHANNEL_MSK		NO_OS_GENMASK(2,0)
#define AD7606_AIN_UV_ERR_MSK			NO_OS_BIT(3)
#define AD7606_AIN_OV_ERR_MSK			NO_OS_BIT(4)
#define AD7606_OPEN_DETECTED_MSK		NO_OS_BIT(5)
#define AD7606_DIGITAL_ERROR_MSK		NO_OS_BIT(6)
#define AD7606_RESET_DETECT_MSK			NO_OS_BIT(7)

/* AD7606_REG_CONFIG */
#define AD7606_CONFIG_OPERATION_MODE_MSK	NO_OS_GENMASK(1,0)
#define AD7606_CONFIG_DOUT_FORMAT_MSK		NO_OS_GENMASK(4,3)
#define AD7606_CONFIG_EXT_OS_CLOCK_MSK		NO_OS_BIT(5)
#define AD7606_CONFIG_STATUS_HEADER_MSK		NO_OS_BIT(6)

/* AD7606_REG_RANGE_CH_X_Y */
#define AD7606_RANGE_CH_MSK(ch)		(NO_OS_GENMASK(3, 0) << (4 * ((ch) % 2)))
#define AD7606_RANGE_CH_MODE(ch, mode)	\
	((NO_OS_GENMASK(3, 0) & mode) << (4 * ((ch) % 2)))

/* AD7606_REG_OVERSAMPLING */
#define AD7606_OS_PAD_MSK			NO_OS_GENMASK(7,4)
#define AD7606_OS_RATIO_MSK			NO_OS_GENMASK(3,0)

/* AD7606_REG_ID */
#define AD7606_ID_DEVICE_ID_MSK			NO_OS_GENMASK(7,4)
#define AD7606_ID_SILICON_REVISION_MSK		NO_OS_GENMASK(3,0)

/* AD7606_REG_GAIN_CH */
#define AD7606_GAIN_MSK				NO_OS_GENMASK(5,0)

/* AD7606_REG_DIGITAL_DIAG_ENABLE */
#define AD7606_ROM_CRC_ERR_EN_MSK		NO_OS_BIT(0)
#define AD7606_MM_CRC_ERR_EN_MSK		NO_OS_BIT(1)
#define AD7606_INT_CRC_ERR_EN_MSK		NO_OS_BIT(2)
#define AD7606_SPI_WRITE_ERR_EN_MSK		NO_OS_BIT(3)
#define AD7606_SPI_READ_ERR_EN_MSK		NO_OS_BIT(4)
#define AD7606_BUSY_STUCK_HIGH_ERR_EN_MSK	NO_OS_BIT(5)
#define AD7606_CLK_FS_OS_COUNTER_EN_MSK		NO_OS_BIT(6)
#define AD7606_INTERFACE_CHECK_EN_MSK		NO_OS_BIT(7)

/* AD7606_REG_DIAGNOSTIC_MUX_CH */
#define AD7606_DIAGN_MUX_CH_MSK(ch)		(NO_OS_GENMASK(2, 0) << (3 * (ch & 0x1)))

#define AD7606_SERIAL_RD_FLAG_MSK(x)		(NO_OS_BIT(6) | ((x) & 0x3F))
#define AD7606_SERIAL_WR_FLAG_MSK(x)		((x) & 0x3F)

#define AD7606_PARALLEL_RD_FLAG_MSK(x)		(NO_OS_BIT(7) | ((x) & 0x7F))
#define AD7606_PARALLEL_WR_FLAG_MSK(x)		((x) & 0x7F)

#define AD7606_MAX_CHANNELS		8

/***************************************************************************//**
 * @brief The `ad7606_device_id` enumeration defines a set of constants
 * representing different models of the AD7606 series of data acquisition
 * systems (DAS). Each constant corresponds to a specific model
 * characterized by the number of channels, bit resolution, sampling
 * rate, and input type (bipolar or differential). This enumeration is
 * used to identify and differentiate between various AD7606 devices in
 * software, facilitating device-specific configurations and operations.
 *
 * @param ID_AD7605_4 Represents a 4-channel DAS with 16-bit, bipolar input,
 * simultaneous sampling ADC.
 * @param ID_AD7606_4 Represents a 4-channel DAS with 16-bit, bipolar input,
 * simultaneous sampling ADC.
 * @param ID_AD7606_6 Represents a 6-channel DAS with 16-bit, bipolar input,
 * simultaneous sampling ADC.
 * @param ID_AD7606_8 Represents an 8-channel DAS with 16-bit, bipolar input,
 * simultaneous sampling ADC.
 * @param ID_AD7606B Represents an 8-channel DAS with 16-bit, 800 kSPS, bipolar
 * input, simultaneous sampling ADC.
 * @param ID_AD7606C_16 Represents an 8-channel DAS with 16-bit, 1 MSPS, bipolar
 * input, simultaneous sampling ADC.
 * @param ID_AD7606C_18 Represents an 8-channel DAS with 18-bit, 1 MSPS, bipolar
 * input, simultaneous sampling ADC.
 * @param ID_AD7608 Represents an 8-channel DAS with 18-bit, bipolar,
 * simultaneous sampling ADC.
 * @param ID_AD7609 Represents an 8-channel differential DAS with 18-bit,
 * bipolar, simultaneous sampling ADC.
 ******************************************************************************/
enum ad7606_device_id {
	/** 4-Channel DAS with 16-Bit, Bipolar Input, Simultaneous Sampling ADC */
	ID_AD7605_4,
	/** 4-Channel DAS with 16-Bit, Bipolar Input, Simultaneous Sampling ADC */
	ID_AD7606_4,
	/** 6-Channel DAS with 16-Bit, Bipolar Input, Simultaneous Sampling ADC */
	ID_AD7606_6,
	/** 8-Channel DAS with 16-Bit, Bipolar Input, Simultaneous Sampling ADC */
	ID_AD7606_8,
	/** 8-Channel DAS with 16-Bit, 800 kSPS, Bipolar Input, Simultaneous Sampling ADC */
	ID_AD7606B,
	/** 8-Channel DAS with 16-Bit, 1 MSPS, Bipolar Input, Simultaneous Sampling ADC */
	ID_AD7606C_16,
	/** 8-Channel DAS with 18-Bit, 1 MSPS, Bipolar Input, Simultaneous Sampling ADC */
	ID_AD7606C_18,
	/** 8-Channel DAS with 18-Bit, Bipolar, Simultaneous Sampling ADC */
	ID_AD7608,
	/** 8-Channel Differential DAS with 18-Bit, Bipolar, Simultaneous Sampling ADC */
	ID_AD7609,
};

/***************************************************************************//**
 * @brief The `ad7606_osr` enumeration defines various oversampling ratios for
 * the AD7606 series of devices, which are used to improve the signal-to-
 * noise ratio by averaging multiple samples. The oversampling options
 * range from 1 to 256, with higher values available only in software
 * mode, allowing for flexible configuration based on the specific
 * requirements of the application.
 *
 * @param AD7606_OSR_1 Oversample by 1.
 * @param AD7606_OSR_2 Oversample by 2.
 * @param AD7606_OSR_4 Oversample by 4.
 * @param AD7606_OSR_8 Oversample by 8.
 * @param AD7606_OSR_16 Oversample by 16.
 * @param AD7606_OSR_32 Oversample by 32.
 * @param AD7606_OSR_64 Oversample by 64.
 * @param AD7606_OSR_128 Oversample by 128, available for chips that have
 * software mode only.
 * @param AD7606_OSR_256 Oversample by 256, available for chips that have
 * software mode only.
 ******************************************************************************/
enum ad7606_osr {
	/** Oversample by 1 */
	AD7606_OSR_1,
	/** Oversample by 2 */
	AD7606_OSR_2,
	/** Oversample by 4 */
	AD7606_OSR_4,
	/** Oversample by 8 */
	AD7606_OSR_8,
	/** Oversample by 16 */
	AD7606_OSR_16,
	/** Oversample by 32 */
	AD7606_OSR_32,
	/** Oversample by 64 */
	AD7606_OSR_64,
	/** Oversample by 128, available for chips that have software mode only */
	AD7606_OSR_128,
	/** Oversample by 256, available for chips that have software mode only */
	AD7606_OSR_256
};

/***************************************************************************//**
 * @brief The `ad7606_op_mode` enumeration defines the various operational modes
 * for the AD7606 device, which include normal operation, standby,
 * autostandby, and shutdown. These modes control the power and
 * functionality of the device, allowing it to operate normally, enter
 * low power states, or completely power down, depending on the
 * requirements of the application.
 *
 * @param AD7606_NORMAL Represents the normal operation mode.
 * @param AD7606_STANDBY Represents the standby mode where all PGAs and SAR ADCs
 * enter a low power mode.
 * @param AD7606_AUTOSTANDBY Represents the autostandby mode, available only in
 * software mode.
 * @param AD7606_SHUTDOWN Represents the shutdown mode where all circuitry is
 * powered down.
 ******************************************************************************/
enum ad7606_op_mode {
	/** Normal operation mode */
	AD7606_NORMAL,
	/** Standby mode, all the PGAs, and all the SAR ADCs enter a low power mode */
	AD7606_STANDBY,
	/** Autostandby mode, available only in software mode */
	AD7606_AUTOSTANDBY,
	/** Shutdown mode, all circuitry is powered down */
	AD7606_SHUTDOWN
};

/***************************************************************************//**
 * @brief The `ad7606_dout_format` enumeration defines the number of digital
 * output (DOUT) lines used in the AD7606 device, which is a data
 * acquisition system (DAS) with simultaneous sampling ADCs. Each
 * enumerator specifies a different configuration of DOUT lines, ranging
 * from a single line (DOUT A) to eight lines (DOUT A through H),
 * allowing for flexible data output configurations depending on the
 * specific application requirements.
 *
 * @param AD7606_1_DOUT DOUT A line is used.
 * @param AD7606_2_DOUT DOUT A,B lines are used.
 * @param AD7606_4_DOUT DOUT A,B,C,D lines are used.
 * @param AD7606_8_DOUT DOUT A,B,C,D,E,F,G,H lines are used.
 ******************************************************************************/
enum ad7606_dout_format {
	/** DOUT A line is used */
	AD7606_1_DOUT,
	/** DOUT A,B lines are used. */
	AD7606_2_DOUT,
	/** DOUT A,B,C,D lines are used. */
	AD7606_4_DOUT,
	/** DOUT A,B,C,D,E,F,G,H lines are used. */
	AD7606_8_DOUT
};

/***************************************************************************//**
 * @brief The `ad7606_range_type` enumeration defines the different types of
 * input range configurations available for the AD7606 device. These
 * range types determine how the input signals are interpreted by the
 * device, whether through hardware settings or software configurations,
 * and whether the signals are single-ended or differential, unipolar or
 * bipolar. This allows for flexible adaptation to various signal
 * conditions and requirements in data acquisition systems.
 *
 * @param AD7606_HW_RANGE Represents the hardware range type for the AD7606
 * device.
 * @param AD7606_SW_RANGE_SINGLE_ENDED_UNIPOLAR Represents a software-configured
 * single-ended unipolar range
 * type.
 * @param AD7606_SW_RANGE_SINGLE_ENDED_BIPOLAR Represents a software-configured
 * single-ended bipolar range type.
 * @param AD7606_SW_RANGE_DIFFERENTIAL_BIPOLAR Represents a software-configured
 * differential bipolar range type.
 ******************************************************************************/
enum ad7606_range_type {
	AD7606_HW_RANGE,
	AD7606_SW_RANGE_SINGLE_ENDED_UNIPOLAR,
	AD7606_SW_RANGE_SINGLE_ENDED_BIPOLAR,
	AD7606_SW_RANGE_DIFFERENTIAL_BIPOLAR,
};

/***************************************************************************//**
 * @brief The `ad7606_config` structure is used to configure the AD7606 device's
 * operational parameters. It includes settings for the operation mode,
 * the format of the data output lines, and switches for enabling
 * external oversampling clock and status header. This structure is
 * essential for setting up the device to operate in various modes and
 * configurations, allowing for flexibility in data acquisition and
 * processing.
 *
 * @param op_mode Specifies the operation mode of the AD7606 device.
 * @param dout_format Defines the number of DOUT lines used for data output.
 * @param ext_os_clock Indicates whether an external oversampling clock is used.
 * @param status_header Determines if the status header is enabled in the
 * configuration.
 ******************************************************************************/
struct ad7606_config {
	/** Operation mode */
	enum ad7606_op_mode op_mode;
	/** Number of DOUT lines */
	enum ad7606_dout_format dout_format;
	/** External oversampling clock switch */
	bool ext_os_clock;
	/** Status header switch */
	bool status_header;
};

/***************************************************************************//**
 * @brief The `ad7606_range` structure defines the operational range for a
 * channel in the AD7606 device, specifying the minimum and maximum
 * values in microvolts and the type of range, which can be hardware or
 * software defined, single-ended or differential, and unipolar or
 * bipolar. This structure is used to configure and manage the input
 * range settings for the AD7606's analog-to-digital conversion process.
 *
 * @param min Minimum range value, which may be negative.
 * @param max Maximum range value.
 * @param type Type of range according to the ad7606_range_type enumeration.
 ******************************************************************************/
struct ad7606_range {
	/** Minimum range value (may be negative) */
	int32_t min;
	/** Maximum range value */
	int32_t max;
	/** Type of range according to \ref ad7606_range_type */
	enum ad7606_range_type type;
};

/***************************************************************************//**
 * @brief The `ad7606_oversampling` structure is used to configure the
 * oversampling settings for the AD7606 device. It contains two fields:
 * `os_pad`, which specifies the padding for oversampling, and
 * `os_ratio`, which defines the oversampling ratio using an enumerated
 * type. This structure allows for fine-tuning the oversampling behavior
 * of the device, which can improve the signal-to-noise ratio by
 * averaging multiple samples.
 *
 * @param os_pad Oversampling padding, represented as a 4-bit unsigned integer.
 * @param os_ratio Oversampling ratio, represented as a 4-bit value from the
 * ad7606_osr enumeration.
 ******************************************************************************/
struct ad7606_oversampling {
	/** Oversampling padding */
	uint8_t os_pad: 4;
	/** Oversampling ratio */
	enum ad7606_osr os_ratio : 4;
};

/***************************************************************************//**
 * @brief The `ad7606_digital_diag` structure is used to configure various
 * digital diagnostic checks for the AD7606 device. Each member of the
 * structure is a boolean flag that enables or disables a specific
 * diagnostic check, such as CRC error checks for ROM, memory map, and
 * internal data, as well as error checks for SPI communication and
 * signal integrity. This structure allows for fine-grained control over
 * the diagnostic features of the AD7606, ensuring robust operation and
 * error detection.
 *
 * @param rom_crc_err_en Enables or disables the ROM CRC error check.
 * @param mm_crc_err_en Enables or disables the memory map CRC error check.
 * @param int_crc_err_en Enables or disables the internal CRC error check for
 * conversion and register data.
 * @param spi_write_err_en Enables or disables the SPI write error check.
 * @param spi_read_err_en Enables or disables the SPI read error check.
 * @param busy_stuck_high_err_en Enables or disables the error check for the
 * busy signal being stuck high for more than 4
 * microseconds.
 * @param clk_fs_os_counter_en Enables or disables the frame sync and
 * oversampling clock counter.
 * @param interface_check_en Enables or disables the interface check.
 ******************************************************************************/
struct ad7606_digital_diag {
	/** ROM CRC check switch */
	bool rom_crc_err_en: 1;
	/** Mempry map CRC check switch */
	bool mm_crc_err_en: 1;
	/** Conversion and register data CRC check switch */
	bool int_crc_err_en: 1;
	/** SPI write error switch  */
	bool spi_write_err_en: 1;
	/** SPI read error switch  */
	bool spi_read_err_en: 1;
	/** Busy stuck high for more than 4us error switch  */
	bool busy_stuck_high_err_en: 1;
	/** Frame sync and oversampling clock counter switch  */
	bool clk_fs_os_counter_en: 1;
	/** Interface check switch */
	bool interface_check_en: 1;
};

/**
 * @struct ad7606_dev
 * @brief Device driver structure
 */
struct ad7606_dev;

/***************************************************************************//**
 * @brief The `ad7606_axi_init_param` structure is used to define the
 * initialization parameters for the AXI drivers associated with the
 * AD7606 device. It includes pointers to initialization parameters for
 * the clock generator, PWM generator, and SPI Engine offload, as well as
 * configuration details such as the clock generator rate, base addresses
 * for the AXI core and RX DMA, and the register access speed.
 * Additionally, it provides a function pointer for invalidating a range
 * of the data cache, which is crucial for ensuring data consistency in
 * memory operations.
 *
 * @param clkgen_init Pointer to clock generator initialization parameters.
 * @param axi_clkgen_rate Specifies the clock generator rate.
 * @param trigger_pwm_init Pointer to PWM generator initialization parameters.
 * @param offload_init_param Pointer to SPI Engine offload initialization
 * parameters.
 * @param core_baseaddr Base address of the AXI core.
 * @param rx_dma_baseaddr Base address of the RX DMA.
 * @param reg_access_speed Specifies the register access speed.
 * @param dcache_invalidate_range Function pointer to invalidate a range of the
 * data cache.
 ******************************************************************************/
struct ad7606_axi_init_param {
	/* Clock generator init parameters */
	struct axi_clkgen_init *clkgen_init;
	/* Clock generator rate */
	uint32_t axi_clkgen_rate;
	/* PWM generator init structure */
	struct no_os_pwm_init_param *trigger_pwm_init;
	/* SPI Engine offload parameters */
	struct spi_engine_offload_init_param *offload_init_param;
	/* AXI Core */
	uint32_t core_baseaddr;
	/* RX DMA base address */
	uint32_t rx_dma_baseaddr;
	uint32_t reg_access_speed;
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
};

/***************************************************************************//**
 * @brief The `ad7606_init_param` structure is used to initialize the AD7606
 * device driver with various configuration parameters. It includes SPI
 * and AXI initialization parameters, GPIO settings for various control
 * signals, device identification, and configuration settings for
 * oversampling, interface mode, and digital diagnostics. Additionally,
 * it provides arrays for channel-specific calibrations such as offset,
 * phase, gain, and operating range, allowing for detailed customization
 * of the device's operation.
 *
 * @param spi_init SPI initialization parameters.
 * @param axi_init AXI initialization parameters.
 * @param gpio_reset RESET GPIO initialization parameters.
 * @param gpio_convst CONVST GPIO initialization parameters.
 * @param gpio_busy BUSY GPIO initialization parameters.
 * @param gpio_stby_n STBYn GPIO initialization parameters.
 * @param gpio_range RANGE GPIO initialization parameters.
 * @param gpio_os0 OS0 GPIO initialization parameters.
 * @param gpio_os1 OS1 GPIO initialization parameters.
 * @param gpio_os2 OS2 GPIO initialization parameters.
 * @param gpio_par_ser PARn/SER GPIO initialization parameters.
 * @param device_id Device ID.
 * @param oversampling Oversampling settings.
 * @param sw_mode Indicates if the device is in hardware or software mode.
 * @param parallel_interface Indicates if the device is in Serial or Parallel
 * interface mode.
 * @param config Configuration register settings.
 * @param digital_diag_enable Digital diagnostics register settings.
 * @param offset_ch Channel offset calibration.
 * @param phase_ch Channel phase calibration.
 * @param gain_ch Channel gain calibration.
 * @param range_ch Channel operating range.
 ******************************************************************************/
struct ad7606_init_param {
	/** SPI initialization parameters */
	struct no_os_spi_init_param spi_init;
	/* AXI initialization parameters */
	struct ad7606_axi_init_param *axi_init;
	/** RESET GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_reset;
	/** CONVST GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_convst;
	/** BUSY GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_busy;
	/** STBYn GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_stby_n;
	/** RANGE GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_range;
	/** OS0 GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_os0;
	/** OS1 GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_os1;
	/** OS2 GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_os2;
	/** PARn/SER GPIO initialization parameters */
	struct no_os_gpio_init_param *gpio_par_ser;
	/** Device ID */
	enum ad7606_device_id device_id;
	/** Oversampling settings */
	struct ad7606_oversampling oversampling;
	/** Whether the device is running in hardware or software mode */
	bool sw_mode;
	/** Serial interface mode or Parallel interface mode */
	bool parallel_interface;
	/** Configuration register settings */
	struct ad7606_config config;
	/** Digital diagnostics register settings */
	struct ad7606_digital_diag digital_diag_enable;
	/** Channel offset calibration */
	int8_t offset_ch[AD7606_MAX_CHANNELS];
	/** Channel phase calibration */
	uint8_t phase_ch[AD7606_MAX_CHANNELS];
	/** Channel gain calibration */
	uint8_t gain_ch[AD7606_MAX_CHANNELS];
	/** Channel operating range */
	struct ad7606_range range_ch[AD7606_MAX_CHANNELS];
};

/***************************************************************************//**
 * @brief Use this function to obtain the list of available input voltage ranges
 * for a specific channel of the AD7606 device. This function is useful
 * when configuring or querying the device for its operational
 * parameters. It must be called with a valid device structure and a non-
 * null pointer for the number of ranges. The function distinguishes
 * between hardware and software mode, returning the appropriate range
 * table and updating the number of ranges accordingly. If the device or
 * the number of ranges pointer is null, the function returns null.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null.
 * @param ch The channel number for which the ranges are being queried. Valid
 * range is from 0 to AD7606_MAX_CHANNELS - 1.
 * @param num_ranges A pointer to a uint32_t where the function will store the
 * number of available ranges. Must not be null.
 * @return Returns a pointer to the first element of an array of ad7606_range
 * structures representing the available ranges for the specified
 * channel, or null if the input parameters are invalid.
 ******************************************************************************/
/***************************************************************************//**
 * @brief The `ad7606_get_ch_ranges` function returns a pointer to a constant
 * `ad7606_range` structure, which represents the operational range of a
 * specified channel on the AD7606 device. It takes a device structure
 * pointer, a channel number, and a pointer to store the number of ranges
 * as parameters.
 *
 * @details This function is used to retrieve the range settings for a specific
 * channel on the AD7606 device, allowing users to understand the
 * operational limits of that channel.
 ******************************************************************************/
const struct ad7606_range *ad7606_get_ch_ranges(struct ad7606_dev *dev,
		uint8_t ch,
		uint32_t *num_ranges);

/***************************************************************************//**
 * @brief This function is used to prepare the AD7606 device for data capture by
 * enabling the necessary preconditions based on the device's interface
 * type. It should be called before initiating a data capture sequence.
 * The function checks if the device's AXI interface is initialized and
 * then selects the appropriate pre-enable routine based on whether the
 * device is configured for parallel or SPI interface. It is important to
 * ensure that the device is properly initialized before calling this
 * function to avoid unexpected behavior.
 *
 * @param dev A pointer to an ad7606_dev structure representing the device. This
 * parameter must not be null, and the device should be properly
 * initialized before calling this function. The function will handle
 * the device based on its configuration, either parallel or SPI
 * interface.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of 0 indicates that the AXI interface is
 * not initialized, while other values may indicate the result of the
 * specific pre-enable routine for the interface type.
 ******************************************************************************/
int32_t ad7606_capture_pre_enable(struct ad7606_dev *dev);
/***************************************************************************//**
 * @brief This function is used to disable any post-capture operations for an
 * AD7606 device. It should be called after data capture is complete to
 * ensure that the device is properly transitioned out of its capture
 * state. The function checks if the device's AXI interface is
 * initialized and then determines the appropriate method to disable
 * post-capture operations based on whether the device is using a
 * parallel or SPI interface. It is important to ensure that the device
 * has been initialized before calling this function, as it will return
 * immediately if the AXI interface is not initialized.
 *
 * @param dev A pointer to an ad7606_dev structure representing the device. This
 * parameter must not be null, and the device should be properly
 * initialized before calling this function. The function will return
 * immediately if the AXI interface within the device is not
 * initialized.
 * @return None
 ******************************************************************************/
void ad7606_capture_post_disable(struct ad7606_dev *dev);

/***************************************************************************//**
 * @brief This function determines whether the AD7606 device is currently
 * operating in software mode. It should be called when there is a need
 * to verify the mode of operation of the device, particularly when
 * configuring or troubleshooting the device's settings. The function
 * requires a valid device structure pointer and will return false if the
 * pointer is null, indicating that the device is not in software mode or
 * the pointer is invalid.
 *
 * @param dev A pointer to an ad7606_dev structure representing the device. Must
 * not be null. If null, the function returns false.
 * @return Returns a boolean value: true if the device is in software mode,
 * false otherwise.
 ******************************************************************************/
bool ad7606_sw_mode_enabled(struct ad7606_dev *dev);

/***************************************************************************//**
 * @brief Use this function to obtain the number of channels configured for a
 * specific AD7606 device instance. This function is essential when you
 * need to know the channel count for operations that depend on the
 * number of available channels. It must be called with a valid device
 * structure that has been properly initialized. If the device pointer is
 * null, the function will return an error code.
 *
 * @param dev A pointer to an ad7606_dev structure representing the device
 * instance. Must not be null. If null, the function returns an error
 * code (-EINVAL). The caller retains ownership of the pointer.
 * @return Returns the number of channels as a positive integer if successful.
 * Returns a negative error code (-EINVAL) if the input device pointer
 * is null.
 ******************************************************************************/
int32_t ad7606_get_channels_number(struct ad7606_dev *dev);

/***************************************************************************//**
 * @brief Use this function to read the value of a specific register from an
 * AD7606 device. It determines the communication interface (parallel or
 * SPI) based on the device configuration and performs the read operation
 * accordingly. This function is essential for retrieving configuration
 * or status information from the device. Ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read. Must be a valid register
 * address as defined by the device.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating success (0) or a negative error code if
 * the read operation fails.
 ******************************************************************************/
int32_t ad7606_reg_read(struct ad7606_dev *dev,
			uint8_t reg_addr,
			uint8_t *reg_data);
/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * on the AD7606 device. It should be called when there is a need to
 * configure or modify the settings of the device by writing to its
 * registers. The function supports both parallel and SPI interfaces,
 * automatically selecting the appropriate method based on the device
 * configuration. It is important to ensure that the device is properly
 * initialized and configured before calling this function. The function
 * returns an error code if the write operation fails.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address as defined by the device.
 * @param reg_data The data byte to write to the specified register. Any 8-bit
 * value is valid.
 * @return Returns an int32_t error code indicating success or failure of the
 * write operation.
 ******************************************************************************/
int32_t ad7606_reg_write(struct ad7606_dev *dev,
			 uint8_t reg_addr,
			 uint8_t reg_data);
int32_t ad7606_write_mask(struct ad7606_dev *dev,
			  uint32_t addr,
			  uint32_t mask,
			  uint32_t val);
/***************************************************************************//**
 * @brief This function retrieves data from the AD7606 device using the SPI
 * interface. It should be called when data acquisition is required from
 * the device. The function expects the device to be properly initialized
 * and configured before calling. It handles data integrity checks if CRC
 * error detection is enabled. The function supports devices with
 * different bit resolutions and channel configurations, and it processes
 * the data accordingly. It returns an error code if the operation fails,
 * such as when the CRC check fails or if the device configuration is
 * unsupported.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null.
 * @param data A pointer to a buffer where the read data will be stored. The
 * buffer must be large enough to hold the data for all channels.
 * Must not be null.
 * @return Returns 0 on success, a negative error code on failure, such as
 * -EBADMSG for CRC errors or -ENOTSUP for unsupported configurations.
 ******************************************************************************/
int32_t ad7606_spi_data_read(struct ad7606_dev *dev,
			     uint32_t *data);
/***************************************************************************//**
 * @brief This function retrieves a specified number of samples from the AD7606
 * device and stores them in the provided data buffer. It should be
 * called when the device is properly initialized and configured. The
 * function handles both parallel and SPI interfaces, depending on the
 * device configuration. It also manages the transition from register
 * mode to ADC reading mode if necessary. Ensure that the data buffer is
 * large enough to hold the requested number of samples, considering the
 * number of channels in the device.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null.
 * @param data A pointer to a buffer where the read samples will be stored. The
 * buffer must be large enough to hold 'samples' number of samples,
 * each consisting of data for all channels.
 * @param samples The number of samples to read. Must be a positive integer.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad7606_read_samples(struct ad7606_dev *dev,
			    uint32_t *data,
			    uint32_t samples);
/***************************************************************************//**
 * @brief This function is used to start a conversion process on an AD7606
 * device. It should be called when a new conversion is required,
 * typically after the device has been properly initialized and
 * configured. The function handles the transition from register mode to
 * conversion mode if necessary, and it controls the CONVST GPIO to
 * trigger the conversion. It is important to ensure that the device
 * structure is correctly initialized and that the GPIOs are properly
 * configured before calling this function. The function returns an error
 * code if the operation fails, which should be checked by the caller to
 * ensure successful execution.
 *
 * @param dev A pointer to an ad7606_dev structure representing the device. This
 * must be a valid, initialized device structure. The function will
 * return an error if this parameter is null or if the device is not
 * properly configured.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad7606_convst(struct ad7606_dev *dev);
/***************************************************************************//**
 * @brief Use this function to reset the AD7606 device, which is necessary to
 * ensure the device is in a known state before performing further
 * operations. This function should be called after the device has been
 * initialized. It handles the reset process by toggling the reset GPIO
 * and reconfiguring the device settings. If the device is configured to
 * use a parallel interface, it also enables the core in parallel mode.
 * The function returns an error code if any GPIO operation fails, which
 * should be checked by the caller to ensure the reset was successful.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device to reset. Must not be null. The caller retains ownership of
 * this structure.
 * @return Returns 0 on success or a negative error code if a GPIO operation
 * fails.
 ******************************************************************************/
int32_t ad7606_reset(struct ad7606_dev *dev);
/***************************************************************************//**
 * @brief This function sets the oversampling ratio and padding for the AD7606
 * device, which can be configured in either software or hardware mode.
 * It must be called with a valid device structure and oversampling
 * settings. In software mode, the function writes the oversampling
 * configuration to the device register, while in hardware mode, it
 * configures the GPIO pins accordingly. The function ensures that
 * unsupported oversampling ratios in hardware mode are clamped to the
 * maximum supported value. It returns an error code if any operation
 * fails, allowing the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param oversampling A structure of type ad7606_oversampling containing the
 * desired oversampling ratio and padding. The os_ratio must
 * be within the supported range for the current mode
 * (software or hardware).
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad7606_set_oversampling(struct ad7606_dev *dev,
				struct ad7606_oversampling oversampling);
/***************************************************************************//**
 * @brief This function is used to obtain the current oversampling configuration
 * of an AD7606 device. It should be called when you need to verify or
 * utilize the oversampling settings of the device. The function requires
 * a valid device structure and a pointer to an oversampling structure
 * where the current settings will be stored. It is important to ensure
 * that both pointers are not null before calling this function, as
 * passing null pointers will result in an error.
 *
 * @param dev A pointer to an ad7606_dev structure representing the device. Must
 * not be null. If null, the function returns an error.
 * @param oversampling A pointer to an ad7606_oversampling structure where the
 * current oversampling settings will be stored. Must not be
 * null. If null, the function returns an error.
 * @return Returns 0 on success, or a negative error code if either input
 * pointer is null.
 ******************************************************************************/
int32_t ad7606_get_oversampling(struct ad7606_dev *dev,
				struct ad7606_oversampling *oversampling);
/***************************************************************************//**
 * @brief This function is used to obtain the scale factor for a specific
 * channel of the AD7606 device. It is essential to call this function
 * when you need to understand the scaling applied to the raw data from a
 * particular channel. The function requires a valid device structure and
 * a channel index within the range of available channels. It is
 * important to ensure that the scale pointer is not null, as this is
 * where the scale factor will be stored. The function will return an
 * error if the channel index is out of range or if the scale pointer is
 * null.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null.
 * @param ch The index of the channel for which the scale factor is requested.
 * Must be less than the number of channels available on the device.
 * @param scale A pointer to a double where the scale factor will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the channel index
 * is invalid or the scale pointer is null.
 ******************************************************************************/
int32_t ad7606_get_ch_scale(struct ad7606_dev *dev, uint8_t ch,
			    double *scale);
/***************************************************************************//**
 * @brief Use this function to obtain the resolution in bits of the AD7606
 * device specified by the provided device structure. This function is
 * useful when you need to know the bit resolution of the ADC for data
 * processing or configuration purposes. Ensure that the device structure
 * is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null. The function will not perform any checks
 * on the validity of the pointer, so it is the caller's
 * responsibility to ensure it points to a valid device.
 * @return Returns an int32_t representing the number of bits of resolution for
 * the specified device.
 ******************************************************************************/
int32_t ad7606_get_resolution_bits(struct ad7606_dev *dev);
/***************************************************************************//**
 * @brief This function configures the input voltage range for a specific
 * channel on an AD7606 device. It should be used when you need to adjust
 * the input range to match the expected signal levels for accurate
 * conversion. The function must be called with a valid device structure
 * and a channel number within the device's channel count. The range
 * structure must specify a valid minimum and maximum range, where the
 * minimum is less than or equal to the maximum. The function handles
 * both hardware and software mode configurations, updating the device's
 * internal state accordingly. It returns an error code if the input
 * parameters are invalid or if the operation fails.
 *
 * @param dev A pointer to an ad7606_dev structure representing the device. Must
 * not be null, and the device must be properly initialized.
 * @param ch The channel number to configure. Must be less than the number of
 * channels supported by the device. Invalid channel numbers result in
 * an error.
 * @param range A struct ad7606_range specifying the desired input range. The
 * min field must be less than or equal to the max field. Invalid
 * ranges result in an error.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error.
 ******************************************************************************/
int32_t ad7606_set_ch_range(struct ad7606_dev *dev, uint8_t ch,
			    struct ad7606_range range);
/***************************************************************************//**
 * @brief Use this function to configure the offset for a specific channel on an
 * AD7606 device. This function should be called only when the device is
 * in software mode, as it is not supported in hardware mode. Ensure that
 * the channel index is within the valid range of available channels for
 * the device. The function updates the device's internal offset register
 * for the specified channel and stores the offset value in the device
 * structure. It returns an error code if the channel index is invalid or
 * if the device is not in software mode.
 *
 * @param dev A pointer to an ad7606_dev structure representing the device. Must
 * not be null, and the device must be initialized and configured for
 * software mode.
 * @param ch The channel index for which the offset is to be set. Must be less
 * than the number of channels available on the device. If invalid,
 * the function returns -EINVAL.
 * @param offset The offset value to set for the specified channel. It is an
 * 8-bit signed integer.
 * @return Returns 0 on success, -EINVAL if the channel index is invalid,
 * -ENOTSUP if the device is not in software mode, or a negative error
 * code from the register write operation.
 ******************************************************************************/
int32_t ad7606_set_ch_offset(struct ad7606_dev *dev, uint8_t ch,
			     int8_t offset);
/***************************************************************************//**
 * @brief This function sets the phase for a specific channel on an AD7606
 * device, which must be in software mode. It should be used when you
 * need to adjust the phase of a channel for calibration or
 * synchronization purposes. The function requires that the channel index
 * is within the valid range of available channels on the device. If the
 * device is not in software mode, the function will not support this
 * operation and will return an error. Ensure that the device is properly
 * initialized and in the correct mode before calling this function.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null.
 * @param ch The channel index for which the phase is to be set. Must be less
 * than the number of channels available on the device.
 * @param phase The phase value to set for the specified channel. The valid
 * range of phase values depends on the device specifications.
 * @return Returns 0 on success, -EINVAL if the channel index is out of range,
 * -ENOTSUP if the device is not in software mode, or a negative error
 * code if the register write operation fails.
 ******************************************************************************/
int32_t ad7606_set_ch_phase(struct ad7606_dev *dev, uint8_t ch,
			    uint8_t phase);
/***************************************************************************//**
 * @brief This function is used to configure the gain setting for a specified
 * channel on an AD7606 device. It should be called when the device is in
 * software mode, as it is not supported in hardware mode. The function
 * requires that the channel index is within the valid range of available
 * channels on the device. If the channel index is invalid or the device
 * is not in software mode, the function will return an error code.
 * Successful execution updates the gain setting for the specified
 * channel.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null.
 * @param ch The channel index for which the gain is to be set. Must be less
 * than the number of channels available on the device.
 * @param gain The desired gain setting for the specified channel. The value is
 * masked to fit within the allowable range for gain settings.
 * @return Returns 0 on success, or a negative error code if the channel index
 * is invalid or the device is not in software mode.
 ******************************************************************************/
int32_t ad7606_set_ch_gain(struct ad7606_dev *dev, uint8_t ch,
			   uint8_t gain);
/***************************************************************************//**
 * @brief Use this function to set the configuration parameters for an AD7606
 * device. This function should be called after the device has been
 * initialized and before starting any data acquisition. It allows you to
 * specify the operation mode, data output format, external oversampling
 * clock usage, and status header inclusion. The function handles both
 * software and hardware modes, adjusting GPIO settings or writing to the
 * configuration register as needed. Ensure that the `dout_format` does
 * not exceed the device's maximum data output lines, and that the
 * `op_mode` is valid for the current mode. Invalid configurations will
 * result in an error.
 *
 * @param dev A pointer to an initialized `ad7606_dev` structure representing
 * the device. Must not be null.
 * @param config A structure of type `ad7606_config` containing the desired
 * configuration settings. The `op_mode` must be a valid mode, and
 * `dout_format` must not exceed the device's maximum data output
 * lines.
 * @return Returns 0 on success, or a negative error code if the configuration
 * is invalid or if an error occurs during the configuration process.
 ******************************************************************************/
int32_t ad7606_set_config(struct ad7606_dev *dev,
			  struct ad7606_config config);
/***************************************************************************//**
 * @brief This function enables or disables various digital diagnostic features
 * on an AD7606 device. It should be called when the device is in
 * software mode, as it is not supported in hardware mode. The function
 * configures the device based on the provided diagnostic settings, which
 * include checks for CRC errors, SPI errors, and other interface
 * diagnostics. It is important to ensure that the device is properly
 * initialized and in the correct mode before calling this function. If
 * the device is not in software mode, the function will return an error.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null. The device must be in software mode for
 * this function to succeed.
 * @param diag A structure of type ad7606_digital_diag containing the desired
 * diagnostic settings. Each field in the structure represents a
 * specific diagnostic feature to enable or disable.
 * @return Returns 0 on success, or a negative error code if the device is not
 * in software mode or if there is a failure in writing to the device
 * register.
 ******************************************************************************/
int32_t ad7606_set_digital_diag(struct ad7606_dev *dev,
				struct ad7606_digital_diag diag);
/***************************************************************************//**
 * @brief This function sets up an AD7606 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function configures the device based on
 * the specified parameters, including interface type, mode, and
 * calibration settings. It handles both parallel and serial interfaces,
 * and ensures the device is correctly reset and configured. If
 * initialization fails, it returns an error code and cleans up any
 * allocated resources.
 *
 * @param device A pointer to a pointer of type `struct ad7606_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad7606_init_param` containing the
 * initialization parameters for the device. Must not be null
 * and should be properly configured with valid settings for
 * the device.
 * @return Returns 0 on successful initialization, or a negative error code if
 * initialization fails. On success, the `device` pointer is set to
 * point to the initialized device structure.
 ******************************************************************************/
int32_t ad7606_init(struct ad7606_dev **device,
		    struct ad7606_init_param *init_param);
/***************************************************************************//**
 * @brief This function is used to process raw data from the AD7606 ADC device,
 * applying necessary corrections based on the device's configuration. It
 * should be called after acquiring raw data from the device to ensure
 * the data is correctly interpreted, especially for devices configured
 * with bipolar input ranges. The function also extracts status
 * information if the device is configured to include a status header. It
 * is essential to ensure that the device structure is properly
 * initialized and that the buffers provided are adequately sized to hold
 * the number of channels configured in the device.
 *
 * @param dev A pointer to an initialized ad7606_dev structure representing the
 * device. Must not be null.
 * @param buf A pointer to a buffer containing raw ADC data. The buffer must
 * have enough space to hold data for all configured channels.
 * @param data A pointer to an array where corrected data will be stored. The
 * array must have enough space to hold data for all configured
 * channels.
 * @param status A pointer to an array where status information will be stored
 * if the device is configured to include a status header. Can be
 * null if status information is not required.
 * @return Returns 0 on success or a negative error code if an invalid parameter
 * is provided.
 ******************************************************************************/
int32_t ad7606_data_correction_serial(struct ad7606_dev *dev,
				      uint32_t *buf, int32_t *data, uint8_t *status);
/***************************************************************************//**
 * @brief Use this function to properly release all resources and memory
 * associated with an AD7606 device instance. It should be called when
 * the device is no longer needed to ensure that all allocated resources
 * are freed and any open connections are closed. This function must be
 * called after the device has been initialized and used, and it is
 * important to ensure that no other operations are performed on the
 * device after this function is called. Failure to call this function
 * may result in resource leaks.
 *
 * @param dev A pointer to an ad7606_dev structure representing the device to be
 * removed. This pointer must not be null, and the device must have
 * been previously initialized. The function will handle invalid
 * pointers by not performing any operations.
 * @return Returns an int32_t indicating the success of the operation. A return
 * value of 0 indicates success, while a non-zero value indicates an
 * error occurred during the removal process.
 ******************************************************************************/
int32_t ad7606_remove(struct ad7606_dev *dev);
#endif /* AD7606_H_ */
