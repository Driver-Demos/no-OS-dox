/***************************************************************************//**
 *   @file   ad7616.h
 *   @brief  Header file of AD7616 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
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
#ifndef AD7616_H_
#define AD7616_H_

#include "no_os_gpio.h"

#include <stdint.h>

#ifdef XILINX_PLATFORM
#include "no_os_pwm.h"
#include "clk_axi_clkgen.h"
#endif

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* AD7616 CORE */
#define AD7616_REG_PCORE_VERSION		0x400
#define AD7616_REG_ID					0x404
#define AD7616_REG_UP_SCRATCH			0x408
#define AD7616_REG_UP_IF_TYPE			0x40C
#define AD7616_REG_UP_CTRL				0x440
#define AD7616_REG_UP_CONV_RATE			0x444
#define AD7616_REG_UP_BURST_LENGTH		0x448
#define AD7616_REG_UP_READ_DATA			0x44C
#define AD7616_REG_UP_WRITE_DATA		0x450

/* AD7616_REG_UP_CTRL */
#define AD7616_CTRL_RESETN				(1 << 0)
#define AD7616_CTRL_CNVST_EN			(1 << 1)

#define AD7616_REG_CONFIG				0x02
#define AD7616_REG_CHANNEL				0x03
#define AD7616_REG_INPUT_RANGE_A1		0x04
#define AD7616_REG_INPUT_RANGE_A2		0x05
#define AD7616_REG_INPUT_RANGE_B1		0x06
#define AD7616_REG_INPUT_RANGE_B2		0x07
#define AD7616_REG_SEQUENCER_STACK(x)	(0x20 + (x))

/* AD7616_REG_CONFIG */
#define AD7616_SDEF				(1 << 7)
#define AD7616_BURSTEN(x)			((x & 1) << 6)
#define AD7616_BURSTEN_MASK			(1 << 6)
#define AD7616_SEQEN(x)				((x & 1) << 5)
#define AD7616_SEQEN_MASK			(1 << 5)
#define AD7616_OS(x)				(((x) & 0x7) << 2)
#define AD7616_STATUSEN				(1 << 1)
#define AD7616_STATUSEN_MASK			(1 << 1)
#define AD7616_CRCEN				(1 << 0)
#define AD7616_CRCEN_MASK			(1 << 0)

/* AD7616_REG_CHANNEL */
#define AD7616_CHA_MASK				0xF
#define AD7616_CHB_MASK				0xF0
#define AD7616_CHB_OFFSET			4
#define AD7616_CHANNELS_MASK			0xFF

/* AD7616_REG_INPUT_RANGE */
#define AD7616_INPUT_RANGE(ch, x)		(((x) & 0x3) << (((ch) & 0x3) * 2))

/* AD7616_REG_SEQUENCER_STACK(x) */
#define AD7616_ADDR(x)					(((x) & 0x7F) << 9)
#define AD7616_SSREN					(1 << 8)
#define AD7616_BSEL(x)					(((x) & 0xF) << 4)
#define AD7616_ASEL(x)					(((x) & 0xF) << 0)

/* AD7616_REG_STATUS */
#define AD7616_STATUS_A(x)				(((x) & 0xF) << 12)
#define AD7616_STATUS_B(x)				(((x) & 0xF) << 8)
#define AD7616_STATUS_CRC(x)			(((x) & 0xFF) << 0)

/* AD7616 conversion results */
#define AD7616_CHANNEL_A_SELF_TEST_VALUE 0xAAAA
#define AD7616_CHANNEL_B_SELF_TEST_VALUE 0x5555

/* AD7616_REG_PWM */
#define AD7616_TRIGGER_PULSE_WIDTH_NS	        50

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad7616_mode` enumeration defines the operational modes for the
 * AD7616 device, which can be set to either software (AD7616_SW) or
 * hardware (AD7616_HW) mode. This enumeration is used to configure the
 * device's mode of operation, allowing for flexibility in how the device
 * is controlled and interfaced with other components.
 *
 * @param AD7616_SW Represents the software mode for the AD7616 device.
 * @param AD7616_HW Represents the hardware mode for the AD7616 device.
 ******************************************************************************/
enum ad7616_mode {
	AD7616_SW,
	AD7616_HW,
};

/***************************************************************************//**
 * @brief The `ad7616_interface` enumeration defines the possible interface
 * modes for the AD7616 device, which can either be serial or parallel.
 * This enumeration is used to configure the communication interface type
 * for the AD7616, allowing the user to select between a serial or
 * parallel data transfer method based on their application requirements.
 *
 * @param AD7616_SERIAL Represents the serial interface mode for the AD7616
 * device.
 * @param AD7616_PARALLEL Represents the parallel interface mode for the AD7616
 * device.
 ******************************************************************************/
enum ad7616_interface {
	AD7616_SERIAL,
	AD7616_PARALLEL,
};

/***************************************************************************//**
 * @brief The `ad7616_ch` enumeration defines a set of constants representing
 * the various channels available on the AD7616 device, which is a data
 * acquisition system. These channels are divided into two groups, VA and
 * VB, each containing eight channels (0-7), a VCC channel, an ALDO
 * channel, and reserved channels for future use. Additionally, each
 * group has a self-test channel to facilitate device diagnostics. This
 * enumeration is crucial for selecting and managing the input channels
 * of the AD7616 during data acquisition operations.
 *
 * @param AD7616_VA0 Represents channel VA0 of the AD7616.
 * @param AD7616_VA1 Represents channel VA1 of the AD7616.
 * @param AD7616_VA2 Represents channel VA2 of the AD7616.
 * @param AD7616_VA3 Represents channel VA3 of the AD7616.
 * @param AD7616_VA4 Represents channel VA4 of the AD7616.
 * @param AD7616_VA5 Represents channel VA5 of the AD7616.
 * @param AD7616_VA6 Represents channel VA6 of the AD7616.
 * @param AD7616_VA7 Represents channel VA7 of the AD7616.
 * @param AD7616_VA_VCC Represents the VCC channel for VA group.
 * @param AD7616_VA_ALDO Represents the ALDO channel for VA group.
 * @param AD7616_VA_RESERVED1 Reserved channel for future use in VA group.
 * @param AD7616_VA_SELF_TEST Self-test channel for VA group.
 * @param AD7616_VA_RESERVED2 Another reserved channel for future use in VA
 * group.
 * @param AD7616_VB0 Represents channel VB0 of the AD7616.
 * @param AD7616_VB1 Represents channel VB1 of the AD7616.
 * @param AD7616_VB2 Represents channel VB2 of the AD7616.
 * @param AD7616_VB3 Represents channel VB3 of the AD7616.
 * @param AD7616_VB4 Represents channel VB4 of the AD7616.
 * @param AD7616_VB5 Represents channel VB5 of the AD7616.
 * @param AD7616_VB6 Represents channel VB6 of the AD7616.
 * @param AD7616_VB7 Represents channel VB7 of the AD7616.
 * @param AD7616_VB_VCC Represents the VCC channel for VB group.
 * @param AD7616_VB_ALDO Represents the ALDO channel for VB group.
 * @param AD7616_VB_RESERVED1 Reserved channel for future use in VB group.
 * @param AD7616_VB_SELF_TEST Self-test channel for VB group.
 * @param AD7616_VB_RESERVED2 Another reserved channel for future use in VB
 * group.
 ******************************************************************************/
enum ad7616_ch {
	AD7616_VA0,
	AD7616_VA1,
	AD7616_VA2,
	AD7616_VA3,
	AD7616_VA4,
	AD7616_VA5,
	AD7616_VA6,
	AD7616_VA7,
	AD7616_VA_VCC,
	AD7616_VA_ALDO,
	AD7616_VA_RESERVED1,
	AD7616_VA_SELF_TEST,
	AD7616_VA_RESERVED2,
	AD7616_VB0,
	AD7616_VB1,
	AD7616_VB2,
	AD7616_VB3,
	AD7616_VB4,
	AD7616_VB5,
	AD7616_VB6,
	AD7616_VB7,
	AD7616_VB_VCC,
	AD7616_VB_ALDO,
	AD7616_VB_RESERVED1,
	AD7616_VB_SELF_TEST,
	AD7616_VB_RESERVED2,
};

/***************************************************************************//**
 * @brief The `ad7616_range` enumeration defines the possible voltage input
 * ranges for the AD7616 device, allowing the user to select between
 * 2.5V, 5V, and 10V ranges. This is used to configure the device to
 * handle different input voltage levels, ensuring accurate analog-to-
 * digital conversion based on the selected range.
 *
 * @param AD7616_2V5 Represents a voltage range of 2.5 volts.
 * @param AD7616_5V Represents a voltage range of 5 volts.
 * @param AD7616_10V Represents a voltage range of 10 volts.
 ******************************************************************************/
enum ad7616_range {
	AD7616_2V5 = 1,
	AD7616_5V  = 2,
	AD7616_10V = 3,
};

/***************************************************************************//**
 * @brief The `ad7616_osr` enumeration defines various oversampling ratios for
 * the AD7616 device, which is an analog-to-digital converter. Each
 * enumerator corresponds to a specific oversampling ratio, allowing the
 * user to select the desired level of oversampling to improve signal
 * quality by reducing noise and increasing resolution. This enumeration
 * is used to configure the oversampling settings in the AD7616 device
 * driver.
 *
 * @param AD7616_OSR_0 Represents an oversampling ratio of 0.
 * @param AD7616_OSR_2 Represents an oversampling ratio of 2.
 * @param AD7616_OSR_4 Represents an oversampling ratio of 4.
 * @param AD7616_OSR_8 Represents an oversampling ratio of 8.
 * @param AD7616_OSR_16 Represents an oversampling ratio of 16.
 * @param AD7616_OSR_32 Represents an oversampling ratio of 32.
 * @param AD7616_OSR_64 Represents an oversampling ratio of 64.
 * @param AD7616_OSR_128 Represents an oversampling ratio of 128.
 ******************************************************************************/
enum ad7616_osr {
	AD7616_OSR_0,
	AD7616_OSR_2,
	AD7616_OSR_4,
	AD7616_OSR_8,
	AD7616_OSR_16,
	AD7616_OSR_32,
	AD7616_OSR_64,
	AD7616_OSR_128,
};

/***************************************************************************//**
 * @brief The `ad7616_dev` structure is a comprehensive descriptor for managing
 * the AD7616 device, which is a high-performance analog-to-digital
 * converter. It includes members for SPI communication, clock
 * generation, and PWM triggering, as well as GPIO descriptors for
 * various control signals. The structure also holds configuration
 * settings such as interface type, operation mode, input range settings
 * for multiple channels, and oversampling ratio. Additionally, it
 * provides a function pointer for cache management and supports
 * sequencer and burst mode operations.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param offload_init_param Pointer to SPI engine offload initialization
 * parameters.
 * @param clkgen Pointer to the clock generator structure for HDL design.
 * @param trigger_pwm_desc Pointer to the PWM generator descriptor for trigger
 * conversion.
 * @param reg_access_speed Speed of register access in Hz.
 * @param crc Cyclic Redundancy Check (CRC) value for data integrity.
 * @param gpio_hw_rngsel0 GPIO descriptor for hardware range select 0.
 * @param gpio_hw_rngsel1 GPIO descriptor for hardware range select 1.
 * @param gpio_reset GPIO descriptor for device reset.
 * @param gpio_os0 GPIO descriptor for oversampling setting 0.
 * @param gpio_os1 GPIO descriptor for oversampling setting 1.
 * @param gpio_os2 GPIO descriptor for oversampling setting 2.
 * @param gpio_convst GPIO descriptor for conversion start signal.
 * @param gpio_busy GPIO descriptor for busy signal indication.
 * @param core_baseaddr Base address of the AXI core.
 * @param interface Interface type, either serial or parallel.
 * @param mode Operation mode, either software or hardware.
 * @param va Array of input range settings for channel A.
 * @param vb Array of input range settings for channel B.
 * @param osr Oversampling ratio setting.
 * @param dcache_invalidate_range Function pointer to invalidate data cache over
 * a range.
 * @param layers_nb Number of layers in the sequencer for burst mode.
 ******************************************************************************/
struct ad7616_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	struct spi_engine_offload_init_param *offload_init_param;
	/* Clock gen for hdl design structure */
	struct axi_clkgen	*clkgen;
	/* Trigger conversion PWM generator descriptor */
	struct no_os_pwm_desc		*trigger_pwm_desc;
	uint32_t reg_access_speed;
	uint8_t crc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_hw_rngsel0;
	struct no_os_gpio_desc	*gpio_hw_rngsel1;
	struct no_os_gpio_desc	*gpio_reset;
	struct no_os_gpio_desc	*gpio_os0;
	struct no_os_gpio_desc	*gpio_os1;
	struct no_os_gpio_desc	*gpio_os2;
	struct no_os_gpio_desc	*gpio_convst;
	struct no_os_gpio_desc	*gpio_busy;
	/* AXI Core */
	uint32_t core_baseaddr;
	/* Device Settings */
	enum ad7616_interface	interface;
	enum ad7616_mode			mode;
	enum ad7616_range		va[8];
	enum ad7616_range		vb[8];
	enum ad7616_osr			osr;
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
	/* Sequencer and burst mode */
	uint8_t layers_nb;
};

/***************************************************************************//**
 * @brief The `ad7616_init_param` structure is used to initialize the AD7616
 * device, encapsulating various parameters required for its
 * configuration. It includes pointers to initialization parameters for
 * SPI, PWM, and clock generator components, as well as settings for GPIO
 * pins used in the device's operation. Additionally, it holds
 * configuration settings for the device's core, such as the base
 * address, operating mode, input range settings for channels A and B,
 * and the oversampling ratio. The structure also provides a function
 * pointer for cache management, ensuring efficient data handling during
 * device operation.
 *
 * @param spi_param Pointer to SPI initialization parameters.
 * @param offload_init_param Pointer to SPI engine offload initialization
 * parameters.
 * @param trigger_pwm_init Pointer to PWM generator initialization parameters.
 * @param clkgen_init Pointer to clock generator initialization parameters for
 * HDL design.
 * @param axi_clkgen_rate Clock generator rate in Hz.
 * @param reg_access_speed Register access speed in Hz.
 * @param crc Cyclic Redundancy Check (CRC) value.
 * @param gpio_hw_rngsel0_param Pointer to GPIO initialization parameters for
 * hardware range select 0.
 * @param gpio_hw_rngsel1_param Pointer to GPIO initialization parameters for
 * hardware range select 1.
 * @param gpio_reset_param Pointer to GPIO initialization parameters for reset.
 * @param gpio_os0_param Pointer to GPIO initialization parameters for
 * oversampling select 0.
 * @param gpio_os1_param Pointer to GPIO initialization parameters for
 * oversampling select 1.
 * @param gpio_os2_param Pointer to GPIO initialization parameters for
 * oversampling select 2.
 * @param gpio_convst_param Pointer to GPIO initialization parameters for
 * conversion start.
 * @param gpio_busy_param Pointer to GPIO initialization parameters for busy
 * signal.
 * @param core_baseaddr Base address of the core.
 * @param mode Operating mode of the AD7616 (software or hardware).
 * @param va Array of input range settings for channel A.
 * @param vb Array of input range settings for channel B.
 * @param osr Oversampling ratio setting.
 * @param dcache_invalidate_range Function pointer to invalidate a range of the
 * data cache.
 ******************************************************************************/
struct ad7616_init_param {
	/* SPI */
	struct no_os_spi_init_param		*spi_param;
	struct spi_engine_offload_init_param *offload_init_param;
	/* PWM generator init structure */
	struct no_os_pwm_init_param	*trigger_pwm_init;
	/* Clock gen for hdl design init structure */
	struct axi_clkgen_init	*clkgen_init;
	/* Clock generator rate */
	uint32_t axi_clkgen_rate;
	uint32_t reg_access_speed;
	uint8_t crc;
	/* GPIO */
	struct no_os_gpio_init_param		*gpio_hw_rngsel0_param;
	struct no_os_gpio_init_param		*gpio_hw_rngsel1_param;
	struct no_os_gpio_init_param		*gpio_reset_param;
	struct no_os_gpio_init_param		*gpio_os0_param;
	struct no_os_gpio_init_param		*gpio_os1_param;
	struct no_os_gpio_init_param		*gpio_os2_param;
	struct no_os_gpio_init_param		*gpio_convst_param;
	struct no_os_gpio_init_param		*gpio_busy_param;
	/* Core */
	uint32_t			core_baseaddr;
	/* Device Settings */
	enum ad7616_mode			mode;
	enum ad7616_range		va[8];
	enum ad7616_range		vb[8];
	enum ad7616_osr			osr;
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
};

/***************************************************************************//**
 * @brief The `ad7616_conversion_result` structure is designed to hold the
 * conversion results from two channels, A and B, of the AD7616 device.
 * Each channel's result is stored as a 16-bit unsigned integer, allowing
 * for the representation of analog-to-digital conversion data. This
 * structure is typically used to store and manage the results of
 * conversions performed by the AD7616, facilitating easy access and
 * manipulation of the data for further processing or analysis.
 *
 * @param channel_a Stores the conversion result for channel A as a 16-bit
 * unsigned integer.
 * @param channel_b Stores the conversion result for channel B as a 16-bit
 * unsigned integer.
 ******************************************************************************/
struct ad7616_conversion_result {
	uint16_t channel_a;
	uint16_t channel_b;
};

/***************************************************************************//**
 * @brief The `ad7616_sequencer_layer` structure is used to define a layer in
 * the sequencer configuration of the AD7616 device. It consists of two
 * members, `ch_a` and `ch_b`, which specify the channels to be used in
 * this layer of the sequencer. These channels are selected from the
 * `ad7616_ch` enumeration, allowing for flexible configuration of the
 * sequencer to handle different input channels for data acquisition.
 *
 * @param ch_a Represents the channel A selection for the sequencer layer, using
 * the ad7616_ch enumeration.
 * @param ch_b Represents the channel B selection for the sequencer layer, using
 * the ad7616_ch enumeration.
 ******************************************************************************/
struct ad7616_sequencer_layer {
	enum ad7616_ch ch_a;
	enum ad7616_ch ch_b;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* SPI read from device. */
/***************************************************************************//**
 * @brief Use this function to read a 16-bit register from the AD7616 device.
 * The function determines the communication interface (serial or
 * parallel) based on the device configuration and performs the read
 * operation accordingly. It is essential to ensure that the device is
 * properly initialized and configured before calling this function. The
 * function will return an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD7616.
 * @param reg_data A pointer to a uint16_t variable where the read register data
 * will be stored. Must not be null.
 * @return Returns an int32_t status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int32_t ad7616_read(struct ad7616_dev *dev,
		    uint8_t reg_addr,
		    uint16_t *reg_data);
/* SPI write to device. */
/***************************************************************************//**
 * @brief This function writes a 16-bit data value to a specified register
 * address on the AD7616 device. It determines the communication
 * interface (serial or parallel) from the device structure and uses the
 * appropriate method to perform the write operation. This function
 * should be called when there is a need to configure or control the
 * AD7616 device by writing to its registers. Ensure that the device
 * structure is properly initialized and configured before calling this
 * function. The function returns an error code if the write operation
 * fails.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 8-bit address of the register to which data will be
 * written. Valid register addresses are defined by the device's
 * register map.
 * @param reg_data The 16-bit data to be written to the specified register. The
 * data should be formatted according to the register's
 * requirements.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating the type of error encountered during the write operation.
 ******************************************************************************/
int32_t ad7616_write(struct ad7616_dev *dev,
		     uint8_t reg_addr,
		     uint16_t reg_data);
/* SPI read from device using a mask. */
/***************************************************************************//**
 * @brief This function reads a value from a specified register of the AD7616
 * device and applies a mask to the read value, storing the result in the
 * provided data pointer. It is used when only specific bits of a
 * register are of interest. The function requires a valid device
 * structure and a non-null data pointer. The function handles both
 * serial and parallel interfaces based on the device configuration. It
 * returns an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD7616 device.
 * @param mask A 16-bit mask to apply to the read register value. Determines
 * which bits of the register are of interest.
 * @param data A pointer to a 16-bit variable where the masked register value
 * will be stored. Must not be null.
 * @return Returns an int32_t error code: 0 for success, or a negative error
 * code if the read operation fails.
 ******************************************************************************/
int32_t ad7616_read_mask(struct ad7616_dev *dev,
			 uint8_t reg_addr,
			 uint16_t mask,
			 uint16_t *data);
/* SPI write to device using a mask. */
/***************************************************************************//**
 * @brief This function allows writing to a specific register of the AD7616
 * device by first reading the current register value, applying a mask to
 * clear specific bits, and then setting those bits to the provided data.
 * It is useful for modifying specific bits in a register without
 * affecting other bits. The function must be called with a valid device
 * structure that has been properly initialized. The operation mode
 * (serial or parallel) is determined by the device's interface setting.
 * The function returns an error code if the read or write operation
 * fails.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be written to. Must be a valid
 * register address for the AD7616 device.
 * @param mask A 16-bit mask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data A 16-bit value containing the new data to be written to the
 * masked bits of the register.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating the type of error encountered during the read or write
 * operation.
 ******************************************************************************/
int32_t ad7616_write_mask(struct ad7616_dev *dev,
			  uint8_t reg_addr,
			  uint16_t mask,
			  uint16_t data);
/* SPI read from device. */
/***************************************************************************//**
 * @brief This function is used to read a 16-bit register value from the AD7616
 * device using SPI communication. It is essential to ensure that the
 * device has been properly initialized and configured before calling
 * this function. The function requires a valid device structure and a
 * register address to read from. The read data is returned through a
 * pointer provided by the caller. This function is typically used when
 * precise control and monitoring of the AD7616's internal registers are
 * needed.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 8-bit address of the register to read from. Only the
 * lower 6 bits are used, so valid addresses range from 0x00 to
 * 0x3F.
 * @param reg_data A pointer to a uint16_t where the read register data will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating the success or failure of the SPI read
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad7616_spi_read(struct ad7616_dev *dev,
			uint8_t reg_addr,
			uint16_t *reg_data);
/* SPI write to device. */
/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specific
 * register on the AD7616 device using the SPI interface. It is typically
 * called when configuration or control data needs to be sent to the
 * device. The function requires a valid device structure that has been
 * properly initialized, and it assumes that the SPI interface is
 * correctly set up. The function returns an integer status code
 * indicating the success or failure of the write operation.
 *
 * @param dev A pointer to an ad7616_dev structure representing the device. This
 * must be a valid, initialized device structure and must not be
 * null.
 * @param reg_addr A 8-bit unsigned integer representing the address of the
 * register to which data will be written. The address should be
 * within the valid range of the device's register map.
 * @param reg_data A 16-bit unsigned integer containing the data to be written
 * to the specified register. The data should be formatted
 * according to the register's requirements.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error during the SPI write operation.
 ******************************************************************************/
int32_t ad7616_spi_write(struct ad7616_dev *dev,
			 uint8_t reg_addr,
			 uint16_t reg_data);
/* PAR read from device. */
/***************************************************************************//**
 * @brief This function reads a specified register from the AD7616 device using
 * parallel communication. It is intended for use on platforms where
 * parallel communication is supported, such as Xilinx platforms. The
 * function requires a valid device structure and a register address to
 * read from. The read data is stored in the provided memory location. If
 * the platform does not support parallel communication, the function
 * returns an error code indicating the operation is not supported.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address within the device's address space.
 * @param reg_data A pointer to a uint16_t variable where the read register data
 * will be stored. Must not be null.
 * @return Returns 0 on success, or -ENOSYS if the operation is not supported on
 * the current platform.
 ******************************************************************************/
int32_t ad7616_par_read(struct ad7616_dev *dev,
			uint8_t reg_addr,
			uint16_t *reg_data);
/* PAR write to device. */
/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specific
 * register of the AD7616 device when operating in parallel mode. It is
 * intended for use on platforms that support the Xilinx environment. The
 * function requires a valid device structure and the register address to
 * which the data should be written. It is important to ensure that the
 * device is properly initialized and configured for parallel
 * communication before calling this function. If the platform is not
 * supported, the function will return an error code indicating that the
 * operation is not implemented.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 8-bit address of the register to write to. Valid range is
 * 0 to 63.
 * @param reg_data The 16-bit data to be written to the specified register. Only
 * the lower 8 bits are used.
 * @return Returns 0 on success when the platform is supported, or -ENOSYS if
 * the operation is not implemented for the current platform.
 ******************************************************************************/
int32_t ad7616_par_write(struct ad7616_dev *dev,
			 uint8_t reg_addr,
			 uint16_t reg_data);
/* Perform a full reset of the device. */
/***************************************************************************//**
 * @brief This function is used to perform a complete reset of the AD7616
 * device, ensuring that it returns to its default state. It should be
 * called when the device needs to be reinitialized or when a known
 * starting state is required. The function manipulates the reset GPIO
 * pin to achieve the reset, and it includes necessary delays to ensure
 * the reset process is completed correctly. It is important to ensure
 * that the device structure is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized ad7616_dev structure. This parameter
 * must not be null, and the structure should be properly set up with
 * valid GPIO descriptors for the reset operation to succeed.
 * @return Returns an int32_t value indicating the success or failure of the
 * reset operation. A non-zero return value indicates an error occurred
 * during the reset process.
 ******************************************************************************/
int32_t ad7616_reset(struct ad7616_dev *dev);
/* Set the analog input range for the selected analog input channel. */
/***************************************************************************//**
 * @brief This function configures the analog input range for a specified
 * channel on the AD7616 device. It should be called when you need to
 * adjust the input range for a specific channel, either in software or
 * hardware mode. In software mode, the function updates the device's
 * internal configuration registers, while in hardware mode, it sets the
 * appropriate GPIO values. Ensure the device is properly initialized
 * before calling this function. The function returns an error code if
 * the operation fails.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param ch The channel to configure, specified as an enum ad7616_ch value.
 * Must be a valid channel identifier.
 * @param range The desired input range, specified as an enum ad7616_range
 * value. Must be a valid range identifier.
 * @return Returns an int32_t error code: 0 for success, or a negative value for
 * failure.
 ******************************************************************************/
int32_t ad7616_set_range(struct ad7616_dev *dev,
			 enum ad7616_ch ch,
			 enum ad7616_range range);
/* Set the operation mode (software or hardware). */
/***************************************************************************//**
 * @brief This function configures the AD7616 device to operate in either
 * software or hardware mode. It should be called when the device needs
 * to switch between these modes, typically during initialization or when
 * changing the operational setup. The function updates the mode setting
 * and applies the current voltage range settings to all channels. It is
 * important to ensure that the `dev` parameter is a valid, initialized
 * device structure before calling this function. The function returns a
 * status code indicating success or failure of the operation.
 *
 * @param dev A pointer to an initialized `ad7616_dev` structure representing
 * the device. Must not be null.
 * @param mode An `enum ad7616_mode` value specifying the desired operation mode
 * (either `AD7616_SW` for software mode or `AD7616_HW` for hardware
 * mode).
 * @return Returns an `int32_t` status code, where 0 indicates success and a
 * non-zero value indicates an error occurred during the operation.
 ******************************************************************************/
int32_t ad7616_set_mode(struct ad7616_dev *dev,
			enum ad7616_mode mode);
/* Set the oversampling ratio. */
/***************************************************************************//**
 * @brief This function configures the oversampling ratio of the AD7616 device,
 * which can be set in either software or hardware mode. It should be
 * called when the device is initialized and ready for configuration. The
 * function updates the oversampling ratio based on the mode of
 * operation, using either register writes or GPIO settings. It returns
 * an error code if the operation fails, ensuring that the oversampling
 * ratio is only updated on successful execution.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param osr An enum value of type ad7616_osr representing the desired
 * oversampling ratio. Valid values are defined in the ad7616_osr
 * enumeration.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad7616_set_oversampling_ratio(struct ad7616_dev *dev,
				      enum ad7616_osr osr);
/* Read data in serial mode. */
/***************************************************************************//**
 * @brief Use this function to read a specified number of conversion results
 * from the AD7616 device when operating in serial mode. It is essential
 * to ensure that the device is properly initialized and configured for
 * serial communication before calling this function. The function will
 * populate the provided results buffer with the conversion data for each
 * sample. If the device is configured to use CRC, the function will also
 * verify the integrity of the data using CRC checks. The function
 * returns an error code if any step in the process fails, such as
 * toggling the conversion or reading the channels.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param results A pointer to an array of ad7616_conversion_result structures
 * where the conversion results will be stored. Must not be null
 * and should have space for at least 'samples' elements.
 * @param samples The number of conversion samples to read. Must be a positive
 * integer.
 * @return Returns 0 on success or a negative error code if an error occurs
 * during the read process.
 ******************************************************************************/
int32_t ad7616_read_data_serial(struct ad7616_dev *dev,
				struct ad7616_conversion_result *results,
				uint32_t samples);
/* Read data in parallel mode. */
/***************************************************************************//**
 * @brief This function reads a specified number of samples from the AD7616
 * device using parallel data transfer. It is intended for use on the
 * Xilinx platform and requires the device to be properly initialized
 * before calling. The function writes the read data into the provided
 * buffer. It is important to ensure that the buffer is large enough to
 * hold the specified number of samples. The function returns an error
 * code if the operation fails, such as when the DMA initialization fails
 * or the transfer does not complete successfully.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param buf A pointer to a buffer where the read data will be stored. Must not
 * be null and should be large enough to hold 'samples' number of
 * data points.
 * @param samples The number of samples to read from the device. Must be a
 * positive integer.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad7616_read_data_parallel(struct ad7616_dev *dev,
				  uint32_t *buf,
				  uint32_t samples);
/* Initialize the core. */
/***************************************************************************//**
 * @brief This function sets up the core interface for the AD7616 device,
 * configuring it for operation. It must be called after the device has
 * been initialized but before any data operations are performed. The
 * function configures control registers, sets conversion rates, and
 * determines the interface type (serial or parallel) based on the
 * hardware configuration. It is essential for ensuring the device is
 * ready for data acquisition and communication.
 *
 * @param dev A pointer to an initialized ad7616_dev structure. This structure
 * must be properly set up with valid base addresses and other
 * necessary configurations before calling this function. The pointer
 * must not be null, and the function assumes ownership of the
 * structure for the duration of the setup process.
 * @return Returns 0 on successful setup. The function updates the 'interface'
 * field of the ad7616_dev structure to indicate the detected interface
 * type.
 ******************************************************************************/
int32_t ad7616_core_setup(struct ad7616_dev *dev);
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up the AD7616 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device to ensure proper configuration. The function
 * allocates necessary resources and configures the device's interface,
 * GPIOs, and other settings based on the platform and initialization
 * parameters. It handles both serial and parallel interfaces and
 * configures the device's mode, oversampling ratio, and input ranges. If
 * any step in the setup process fails, the function cleans up allocated
 * resources and returns an error code.
 *
 * @param device A pointer to a pointer of type `struct ad7616_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure.
 * @param init_param A pointer to a `struct ad7616_init_param` containing the
 * initialization parameters for the device. This includes SPI
 * parameters, GPIO configurations, clock settings, and
 * device-specific settings like mode and oversampling ratio.
 * The caller retains ownership of this structure, and it must
 * not be null.
 * @return Returns 0 on success, or a negative error code if initialization
 * fails. On success, the `device` pointer is set to point to the
 * initialized device structure.
 ******************************************************************************/
int32_t ad7616_setup(struct ad7616_dev **device,
		     struct ad7616_init_param *init_param);
/* Remove the device. */
/***************************************************************************//**
 * @brief This function is used to clean up and release all resources associated
 * with a given AD7616 device instance. It should be called when the
 * device is no longer needed to ensure that all allocated resources,
 * such as GPIOs and other peripherals, are properly freed. The function
 * checks if the provided device pointer is non-null before proceeding
 * with the cleanup. It is important to call this function to prevent
 * resource leaks in the system.
 *
 * @param dev A pointer to an ad7616_dev structure representing the device
 * instance to be removed. Must not be null. If null, the function
 * returns immediately without performing any action.
 * @return None
 ******************************************************************************/
void ad7616_remove(struct ad7616_dev *device);
/* Read conversion results. */
/***************************************************************************//**
 * @brief This function is used to obtain the current channel source
 * configuration for both channel A and channel B of the AD7616 device.
 * It should be called when you need to verify or utilize the current
 * channel settings. The function requires a valid device structure and
 * two pointers to store the channel configurations. It returns an error
 * code if the read operation fails, otherwise it returns 0 indicating
 * success.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param ch_a A pointer to an enum ad7616_ch variable where the function will
 * store the current source configuration for channel A. Must not be
 * null.
 * @param ch_b A pointer to an enum ad7616_ch variable where the function will
 * store the current source configuration for channel B. Must not be
 * null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int32_t ad7616_read_channel_source(struct ad7616_dev *dev, enum ad7616_ch *ch_a,
				   enum ad7616_ch *ch_b);
/* Select the input source for a channel. */
/***************************************************************************//**
 * @brief This function is used to configure the AD7616 device to select a
 * specific input source for a given channel. It should be called when
 * there is a need to change the input source for a channel, such as
 * switching between different voltage inputs. The function must be
 * called with a valid device structure and a valid channel enumeration.
 * It is important to note that after changing the channel source, the
 * next conversion result may not be correct, so users should handle this
 * in their application logic.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param ch An enumeration value of type ad7616_ch representing the channel to
 * configure. Must be a valid channel enumeration defined in the
 * ad7616_ch enum. Invalid values may result in undefined behavior.
 * @return Returns an int32_t value. A return value of 0 indicates success,
 * while a non-zero value indicates an error occurred during the
 * operation.
 ******************************************************************************/
int32_t ad7616_select_channel_source(struct ad7616_dev *dev, enum ad7616_ch ch);
/* Setup sequencer with given layers. */
/***************************************************************************//**
 * @brief This function sets up the sequencer of the AD7616 device by
 * configuring it with a specified number of sequencer layers and
 * optionally enabling burst mode. It should be called after the device
 * has been initialized and before starting any conversion operations
 * that require sequencer functionality. The function writes the
 * configuration for each layer to the device and enables the sequencer.
 * If burst mode is enabled, it configures the device accordingly. The
 * function returns an error code if any write operation fails, ensuring
 * that the sequencer is only enabled if all configurations are
 * successfully applied.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null.
 * @param layers An array of ad7616_sequencer_layer structures specifying the
 * channels for each sequencer layer. Must not be null and must
 * have at least layers_nb elements.
 * @param layers_nb The number of sequencer layers to configure. Must be greater
 * than zero and not exceed the device's maximum supported
 * layers.
 * @param burst A boolean value (0 or 1) indicating whether burst mode should be
 * enabled. If non-zero, burst mode is enabled.
 * @return Returns 0 on success or a negative error code if a write operation
 * fails.
 ******************************************************************************/
int32_t ad7616_setup_sequencer(struct ad7616_dev *dev,
			       struct ad7616_sequencer_layer *layers, uint32_t layers_nb, uint8_t burst);
/* Disable the sequencer. */
/***************************************************************************//**
 * @brief Use this function to disable the sequencer on an AD7616 device, which
 * is necessary when you want to operate the device without sequencing or
 * when reconfiguring the sequencer settings. This function should be
 * called when the device is in a state where it can accept configuration
 * changes, typically after initialization and before starting data
 * acquisition. It modifies the device's configuration to disable both
 * the sequencer and burst mode, and it returns a status code indicating
 * the success or failure of the operation.
 *
 * @param dev A pointer to an initialized ad7616_dev structure representing the
 * device. Must not be null. The caller retains ownership and is
 * responsible for ensuring the device is properly initialized before
 * calling this function.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error occurred during the operation.
 ******************************************************************************/
int32_t ad7616_disable_sequencer(struct ad7616_dev *dev);
#endif
