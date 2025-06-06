/***************************************************************************//**
 *   @file   AD719X.h
 *   @brief  Header file of AD7190/2/3/4/5 Driver.
 *   @author DNechita (Dan.Nechita@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
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

#ifndef __AD719X_H__
#define __AD719X_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"

/******************************************************************************/
/******************************** AD719X **************************************/
/******************************************************************************/

/* SPI slave device ID */
#define AD719X_SLAVE_ID         1

/* AD719X Register Map */
#define AD719X_REG_COMM         0 // Communications Register (WO, 8-bit)
#define AD719X_REG_STAT         0 // Status Register         (RO, 8-bit)
#define AD719X_REG_MODE         1 // Mode Register           (RW, 24-bit
#define AD719X_REG_CONF         2 // Configuration Register  (RW, 24-bit)
#define AD719X_REG_DATA         3 // Data Register           (RO, 24/32-bit)
#define AD719X_REG_ID           4 // ID Register             (RO, 8-bit)
#define AD719X_REG_GPOCON       5 // GPOCON Register         (RW, 8-bit)
#define AD719X_REG_OFFSET       6 // Offset Register         (RW, 24-bit
#define AD719X_REG_FULLSCALE    7 // Full-Scale Register     (RW, 24-bit)

/* Communications Register Bit Designations (AD719X_REG_COMM) */
#define AD719X_COMM_WEN         (1 << 7)           // Write Enable.
#define AD719X_COMM_WRITE       (0 << 6)           // Write Operation.
#define AD719X_COMM_READ        (1 << 6)           // Read Operation.
#define AD719X_COMM_ADDR(x)     (((x) & 0x7) << 3) // Register Address.
#define AD719X_COMM_CREAD       (1 << 2)           // Continuous Read of Data Register.

/* Status Register Bit Designations (AD719X_REG_STAT) */
#define AD719X_STAT_RDY         (1 << 7) // Ready.
#define AD719X_STAT_ERR         (1 << 6) // ADC error bit.
#define AD719X_STAT_NOREF       (1 << 5) // Error no external reference.
#define AD719X_STAT_PARITY      (1 << 4) // Parity check of the data register.
#define AD719X_STAT_CH3         (1 << 3) // Channel 3.
#define AD719X_STAT_CH2         (1 << 2) // Channel 2.
#define AD719X_STAT_CH1         (1 << 1) // Channel 1.
#define AD719X_STAT_CH0         (1 << 0) // Channel 0.

/* Mode Register Bit Designations (AD719X_REG_MODE) */
#define AD719X_MODE_SEL(x)      (((uint32_t)(x) & 0x7) << 21) // Operation Mode Select.
#define AD719X_MODE_DAT_STA     ((uint32_t)1 << 20)           // Status Register transmission.
#define AD719X_MODE_CLKSRC(x)   (((uint32_t)(x) & 0x3) << 18) // Clock Source Select.
#define AD719X_MODE_AVG(x)      (((uint32_t)(x) & 0x3) << 16) // Fast settling filter.
#define AD719X_MODE_SINC3       (1 << 15)                          // SINC3 Filter Select.
#define AD719X_MODE_ENPAR       (1 << 13)                          // Parity Enable.
#define AD719X_MODE_CLKDIV      (1 << 12)                          // Clock divide by 2 (AD7190/2 only).
#define AD719X_MODE_SCYCLE      (1 << 11)                          // Single cycle conversion.
#define AD719X_MODE_REJ60       (1 << 10)                          // 50/60Hz notch filter.
#define AD719X_MODE_RATE(x)     ((x) & 0x3FF)                      // Filter Update Rate Select.

/* Mode Register: AD719X_MODE_AVG(x) options */
#define AD719X_AVG_NONE                 0 // No averaging (fast settling mode disabled).
#define AD719X_AVG_BY_2                 1 // Average by 2.
#define AD719X_AVG_BY_8                 2 // Average by 8.
#define AD719X_AVG_BY_16                3 // Average by 16.

/* Configuration Register Bit Designations (AD719X_REG_CONF) */
#define AD719X_CONF_CHOP        ((uint32_t)1 << 23)            // CHOP enable.
#define AD719X_CONF_REFSEL      ((uint32_t)1 << 20)            // REFIN1/REFIN2 Reference Select.
#define AD719X_CONF_PSEUDO      ((uint32_t)1 << 18)            // Pseudo differential analog inputs.
#define AD719X_CONF_CHAN(x)     ((uint32_t)((x) & 0x3FF) << 8) // Channel select.
#define AD719X_CONF_BURN        (1 << 7)                            // Burnout current enable.
#define AD719X_CONF_REFDET      (1 << 6)                            // Reference detect enable.
#define AD719X_CONF_BUF         (1 << 4)                            // Buffered Mode Enable.
#define AD719X_CONF_UNIPOLAR    (1 << 3)                            // Unipolar/Bipolar Enable.
#define AD719X_CONF_GAIN(x)     ((x) & 0x7)                         // Gain Select.

/* Channel Mask */
#define AD719X_CH_MASK(channel)		NO_OS_BIT(channel)

/* Configuration Register: AD719X_CONF_CHAN(x) options */
#define AD719X_CH_0      0
#define AD719X_CH_1      1
#define AD719X_CH_2      2
#define AD719X_CH_3      3
#define AD719X_CH_4      4
#define AD719X_CH_5      5
#define AD719X_CH_6      6
#define AD719X_CH_7      7
#define AD719X_CH_TEMP   8
#define AD719X_CH_SHORT  9

/* ID Register Bit Designations (AD7193_REG_ID) */
#define AD7190_4_ID_MASK			0x0F
#define AD7195_ID_MASK				0xFF

/* GPOCON Register Bit Designations (AD719X_REG_GPOCON) */
#define AD719X_GPOCON_BPDSW     (1 << 6) // Bridge power-down switch enable
#define AD719X_GPOCON_GP32EN    (1 << 5) // Digital Output P3 and P2 enable
#define AD719X_GPOCON_GP10EN    (1 << 4) // Digital Output P1 and P0 enable
#define AD719X_GPOCON_P3DAT     (1 << 3) // P3 state
#define AD719X_GPOCON_P2DAT     (1 << 2) // P2 state
#define AD719X_GPOCON_P1DAT     (1 << 1) // P1 state
#define AD719X_GPOCON_P0DAT     (1 << 0) // P0 state

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad719x_adc_gain` enumeration defines the possible gain settings
 * for the AD719X series of analog-to-digital converters (ADCs). Each
 * enumerator corresponds to a specific gain value that can be configured
 * in the ADC's configuration register, allowing the user to adjust the
 * amplification of the input signal before conversion. This is crucial
 * for optimizing the ADC's performance across different input signal
 * ranges.
 *
 * @param AD719X_ADC_GAIN_1 Represents a gain of 1 for the ADC.
 * @param AD719X_ADC_GAIN_8 Represents a gain of 8 for the ADC.
 * @param AD719X_ADC_GAIN_16 Represents a gain of 16 for the ADC.
 * @param AD719X_ADC_GAIN_32 Represents a gain of 32 for the ADC.
 * @param AD719X_ADC_GAIN_64 Represents a gain of 64 for the ADC.
 * @param AD719X_ADC_GAIN_128 Represents a gain of 128 for the ADC.
 ******************************************************************************/
enum ad719x_adc_gain {
//                           ADC Gain (CONF Reg)
	AD719X_ADC_GAIN_1 = 0, 		// Gain 1
	AD719X_ADC_GAIN_8 = 3, 		// Gain 8
	AD719X_ADC_GAIN_16 = 4,		// Gain 16
	AD719X_ADC_GAIN_32 = 5,		// Gain 32
	AD719X_ADC_GAIN_64 = 6,		// Gain 64
	AD719X_ADC_GAIN_128 = 7		// Gain 128
};

/***************************************************************************//**
 * @brief The `ad719x_adc_clock` enumeration defines the possible clock sources
 * for the AD719X series of analog-to-digital converters. It includes
 * options for using an external crystal connected between MCLK1 and
 * MCLK2, an external clock applied to MCLK2, and two configurations of
 * an internal 4.92 MHz clock, either with MCLK2 tristated or available
 * on MCLK2. This allows for flexible clock source selection depending on
 * the application requirements.
 *
 * @param AD719X_EXT_CRYSTAL_MCLK1_MCLK2 External crystal is connected from
 * MCLK1 to MCLK2.
 * @param AD719X_EXT_CRYSTAL_MCLK2 External clock is applied to MCLK2.
 * @param AD719X_INT_CLK_4_92_MHZ_TRIST Internal 4.92 MHz clock with MCLK2 pin
 * tristated.
 * @param AD719X_INT_CLK_4_92_MHZ Internal 4.92 MHz clock is available on MCLK2.
 ******************************************************************************/
enum ad719x_adc_clock {
	// External crystal. The external crystal is connected from MCLK1 to MCLK2.
	AD719X_EXT_CRYSTAL_MCLK1_MCLK2,
	// External Clock applied to MCLK2
	AD719X_EXT_CRYSTAL_MCLK2,
	// Internal 4.92 MHz clock. Pin MCLK2 is tristated.
	AD719X_INT_CLK_4_92_MHZ_TRIST,
	// Internal 4.92 MHz clock. The internal clock is available on MCLK2.
	AD719X_INT_CLK_4_92_MHZ
};

/***************************************************************************//**
 * @brief The `ad719x_adc_modes` enumeration defines the various operational
 * modes available for the AD719X series of analog-to-digital converters
 * (ADCs). These modes include different conversion modes, such as
 * continuous and single conversion, as well as various calibration
 * modes, both internal and system-based, to ensure accurate
 * measurements. The enumeration provides a structured way to select and
 * manage the ADC's operating state, facilitating its integration and
 * control within a larger system.
 *
 * @param AD719X_MODE_CONT Represents the continuous conversion mode of the ADC.
 * @param AD719X_MODE_SINGLE Represents the single conversion mode of the ADC.
 * @param AD719X_MODE_IDLE Represents the idle mode of the ADC.
 * @param AD719X_MODE_PWRDN Represents the power-down mode of the ADC.
 * @param AD719X_MODE_CAL_INT_ZERO Represents the internal zero-scale
 * calibration mode of the ADC.
 * @param AD719X_MODE_CAL_INT_FULL Represents the internal full-scale
 * calibration mode of the ADC.
 * @param AD719X_MODE_CAL_SYS_ZERO Represents the system zero-scale calibration
 * mode of the ADC.
 * @param AD719X_MODE_CAL_SYS_FULL Represents the system full-scale calibration
 * mode of the ADC.
 ******************************************************************************/
enum ad719x_adc_modes {
	// Continuous Conversion Mode
	AD719X_MODE_CONT,
	// Single Conversion Mode
	AD719X_MODE_SINGLE,
	// Idle Mode
	AD719X_MODE_IDLE,
	// Power-Down Mode
	AD719X_MODE_PWRDN,
	// Internal Zero-Scale Calibration
	AD719X_MODE_CAL_INT_ZERO,
	// Internal Full-Scale Calibration4
	AD719X_MODE_CAL_INT_FULL,
	// System Zero-Scale Calibration5
	AD719X_MODE_CAL_SYS_ZERO,
	// System Full-Scale Calibration
	AD719X_MODE_CAL_SYS_FULL,
};

/***************************************************************************//**
 * @brief The `ad719x_chip_id` enumeration defines a set of constants
 * representing the unique identifiers for different models of the AD719x
 * series of analog-to-digital converters. Each constant corresponds to a
 * specific model, allowing the software to identify and differentiate
 * between the various AD719x devices based on their chip ID values.
 *
 * @param AD7190 Represents the chip ID for the AD7190 model with a value of
 * 0x4.
 * @param AD7192 Represents the chip ID for the AD7192 model with a value of
 * 0x0.
 * @param AD7193 Represents the chip ID for the AD7193 model with a value of
 * 0x2.
 * @param AD7194 Represents the chip ID for the AD7194 model with a value of
 * 0x3.
 * @param AD7195 Represents the chip ID for the AD7195 model with a value of
 * 0xA6.
 ******************************************************************************/
enum ad719x_chip_id {
	AD7190 = 0x4,
	AD7192 = 0x0,
	AD7193 = 0x2,
	AD7194 = 0x3,
	AD7195 = 0xA6
};

/***************************************************************************//**
 * @brief The `ad719x_dev` structure is a comprehensive representation of the
 * AD719x ADC device configuration and state, encapsulating SPI and GPIO
 * descriptors for communication, as well as various device settings such
 * as gain, operating mode, data rate, clock source, and input mode. This
 * structure is essential for managing the ADC's operation and
 * interfacing with the hardware, providing a centralized configuration
 * for the device's parameters and control settings.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_miso Pointer to the GPIO descriptor for the MISO pin.
 * @param sync_pin Pointer to the GPIO descriptor for the sync pin.
 * @param current_polarity Current polarity setting of the device.
 * @param current_gain Current gain setting of the ADC.
 * @param operating_mode Current operating mode of the ADC.
 * @param data_rate_code Code representing the data rate setting.
 * @param clock_source Current clock source setting for the ADC.
 * @param input_mode Current input mode setting of the ADC.
 * @param buffer Buffer setting for the ADC input channels.
 * @param bpdsw_mode Bridge power-down switch mode setting.
 * @param chip_id Identifier for the specific AD719x chip model.
 ******************************************************************************/
struct ad719x_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_miso;
	struct no_os_gpio_desc	*sync_pin;
	/* Device Settings */
	uint8_t			current_polarity;
	enum ad719x_adc_gain	current_gain;
	enum ad719x_adc_modes	operating_mode;
	uint16_t    		data_rate_code;
	enum ad719x_adc_clock	clock_source;
	uint8_t			input_mode;
	uint8_t			buffer;
	uint8_t     		bpdsw_mode;
	enum ad719x_chip_id chip_id;
};

/***************************************************************************//**
 * @brief The `ad719x_init_param` structure is used to initialize and configure
 * the AD719x series of ADCs. It includes parameters for SPI and GPIO
 * initialization, as well as various device settings such as gain,
 * operating mode, data rate, clock source, and input mode. This
 * structure allows for flexible configuration of the ADC to suit
 * different application requirements, including support for multiple
 * devices through an optional synchronization pin.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_miso Pointer to GPIO initialization parameters for MISO.
 * @param sync_pin Pointer to optional GPIO initialization parameters for
 * synchronization pin, used for multiple devices.
 * @param current_polarity Current polarity setting for the device.
 * @param current_gain Current gain setting for the ADC.
 * @param operating_mode Operating mode of the ADC.
 * @param data_rate_code Code representing the data rate setting.
 * @param clock_source Clock source selection for the ADC.
 * @param input_mode Input mode configuration for the ADC.
 * @param buffer Buffer enable/disable setting for the ADC input channels.
 * @param bpdsw_mode Bridge power-down switch mode setting.
 * @param chip_id Identifier for the specific AD719x chip model.
 ******************************************************************************/
struct ad719x_init_param {
	/* SPI */
	struct no_os_spi_init_param		*spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	*gpio_miso;
	/* Optional GPIO pin - only for multiple devices */
	struct no_os_gpio_init_param	*sync_pin;
	/* Device Settings */
	uint8_t			current_polarity;
	enum ad719x_adc_gain	current_gain;
	enum ad719x_adc_modes	operating_mode;
	uint16_t    		data_rate_code;
	enum ad719x_adc_clock	clock_source;
	uint8_t			input_mode;
	uint8_t			buffer;
	uint8_t     		bpdsw_mode;
	enum ad719x_chip_id chip_id;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up and initializes an AD719x device using the
 * provided initialization parameters. It configures the SPI and GPIO
 * interfaces, resets the device, and verifies the chip ID to ensure the
 * correct device is present. The function must be called before any
 * other operations on the AD719x device. If initialization fails at any
 * step, resources are cleaned up and an error code is returned. The
 * caller is responsible for managing the memory of the device structure,
 * which is allocated within this function.
 *
 * @param device A pointer to a pointer of type `struct ad719x_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct ad719x_init_param` containing
 * initialization parameters such as SPI and GPIO
 * configurations, device settings, and chip ID. All fields
 * must be properly initialized before calling the function.
 * @return Returns 0 on successful initialization, or a negative error code if
 * initialization fails.
 ******************************************************************************/
int ad719x_init(struct ad719x_dev **device,
		struct ad719x_init_param init_param);

/***************************************************************************//**
 * @brief This function is used to release all resources allocated for an AD719X
 * device instance, including SPI and GPIO descriptors. It should be
 * called when the device is no longer needed to ensure proper cleanup
 * and avoid resource leaks. The function must be called only after a
 * successful initialization of the device using `ad719x_init`. It
 * returns an error code if the removal of any resource fails, allowing
 * the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an `ad719x_dev` structure representing the device
 * instance to be removed. This pointer must not be null, and it must
 * point to a valid, initialized device structure. The function will
 * attempt to free the resources associated with this device.
 * @return Returns 0 on success, or a negative error code if any resource
 * removal fails.
 ******************************************************************************/
int ad719x_remove(struct ad719x_dev *dev);

/***************************************************************************//**
 * @brief This function is used to write a specified value into a register of
 * the AD719X device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function
 * constructs a write command and sends it over SPI to the device. It is
 * important to provide the correct number of bytes to be written, as
 * this determines how much of the register value is sent. The function
 * returns an error code if the SPI communication fails, allowing the
 * caller to handle communication errors appropriately.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address for the AD719X device.
 * @param reg_val The value to write into the register. The value should be
 * formatted according to the register's expected data format.
 * @param bytes_number The number of bytes to write from reg_val to the
 * register. Must be between 1 and 4, inclusive, depending
 * on the register's size.
 * @return Returns 0 on success or a negative error code if the SPI write
 * operation fails.
 ******************************************************************************/
int ad719x_set_register_value(struct ad719x_dev *dev, uint8_t reg_addr,
			      uint32_t reg_value, uint8_t bytes_number);

/***************************************************************************//**
 * @brief This function retrieves the value stored in a specified register of
 * the AD719X device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function
 * requires the address of the register to be read and the number of
 * bytes to read. It writes the retrieved register value into the
 * provided memory location. If the output pointer is null, the function
 * returns an error. This function is useful for obtaining configuration
 * or status information from the device.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD719X device.
 * @param bytes_number The number of bytes to read from the register. Must be
 * between 1 and 4, inclusive.
 * @param reg_data A pointer to a uint32_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if reg_data is null.
 ******************************************************************************/
int ad719x_get_register_value(struct ad719x_dev *dev, uint8_t reg_addr,
			      uint8_t bytes_number, uint32_t *reg_data);

/* Write masked data into device register. */
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * AD719X device by applying a mask and then writing the new data. It is
 * useful when only certain bits of a register need to be updated without
 * affecting the other bits. The function first reads the current value
 * of the register, applies the mask to clear the bits to be modified,
 * and then writes the new data. It must be called with a valid device
 * structure and appropriate register address. The function returns an
 * error code if the read or write operation fails.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD719X device.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param reg_data The new data to be written to the masked bits of the
 * register. Only the bits specified by the mask will be
 * updated.
 * @param bytes The number of bytes to read from and write to the register. Must
 * be appropriate for the specified register.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad719x_set_masked_register_value(struct ad719x_dev *dev,
				     uint8_t reg_addr, uint32_t mask, uint32_t data,
				     uint8_t bytes);

/***************************************************************************//**
 * @brief This function is used to reset the AD719X device, which is necessary
 * to ensure the device is in a known state before configuration or
 * operation. It sends a reset command via SPI, which requires the device
 * to be properly initialized and the SPI interface to be operational.
 * After issuing the reset command, the function enforces a mandatory
 * delay of 500 microseconds to allow the device to complete the reset
 * process. This function should be called whenever a reset of the device
 * is required, such as during initialization or error recovery.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. This parameter must not be null, and the SPI descriptor
 * within the structure must be valid. If the device is not properly
 * initialized, the function may fail.
 * @return Returns 0 on success or a negative error code if the SPI write
 * operation fails.
 ******************************************************************************/
int ad719x_reset(struct ad719x_dev *dev);

/***************************************************************************//**
 * @brief This function configures the operating mode of the AD719X device to
 * the specified mode. It should be called when a change in the ADC's
 * operating mode is required, such as switching between continuous and
 * single conversion modes. The function must be called with a valid
 * device structure that has been initialized, and a valid mode from the
 * predefined `ad719x_adc_modes` enumeration. If the operation is
 * successful, the device's internal state is updated to reflect the new
 * mode.
 *
 * @param dev A pointer to an `ad719x_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @param opt_mode An enumeration value of type `ad719x_adc_modes` specifying
 * the desired operating mode. Valid values are defined in the
 * `ad719x_adc_modes` enum, such as `AD719X_MODE_CONT` for
 * continuous conversion mode.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad719x_set_operating_mode(struct ad719x_dev *dev,
			      enum ad719x_adc_modes opt_mode);

/***************************************************************************//**
 * @brief This function is used to wait until the RDY (Ready) pin of the AD719x
 * device goes low, indicating that the device is ready for the next
 * operation. It should be called when a stable state of the device is
 * required before proceeding with further operations. The function will
 * repeatedly check the state of the RDY pin and will return an error
 * code if the operation fails or if a timeout occurs. It is important to
 * ensure that the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an ad719x_dev structure representing the device. This
 * must be a valid, initialized device structure, and must not be
 * null. The function will use this structure to access the GPIO pin
 * associated with the RDY signal.
 * @return Returns 0 on success, indicating the RDY pin went low, or a non-zero
 * error code if the operation failed or timed out.
 ******************************************************************************/
int ad719x_wait_rdy_go_low(struct ad719x_dev *dev);

/***************************************************************************//**
 * @brief This function is used to configure which channels are active on an
 * AD719x device by setting a channel mask. It should be called after the
 * device has been initialized and before starting any data acquisition.
 * The function checks the validity of the channel mask based on the
 * specific AD719x chip model being used, and returns an error if the
 * mask is invalid. This ensures that only supported channels are
 * selected for the given device model.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null.
 * @param chn_mask A 16-bit mask specifying which channels to activate. The
 * valid range depends on the chip model: up to 0x3FF for
 * AD7193, up to 0x1FF for AD7194, and up to 0xFF for other
 * models. Invalid masks result in an error return.
 * @return Returns 0 on success or a negative error code if the channel mask is
 * invalid for the device model.
 ******************************************************************************/
int ad719x_channels_select(struct ad719x_dev *dev, uint16_t chn_mask);

/***************************************************************************//**
 * @brief This function is used to calibrate a specific channel of the AD719x
 * device in a given mode. It should be called when calibration is
 * required to ensure accurate measurements. The function first selects
 * the specified channel and then sets the device to the desired
 * calibration mode. It waits for the calibration process to complete
 * before returning. This function must be called with a valid device
 * structure and appropriate mode and channel values. It returns an error
 * code if the channel selection or mode setting fails, or if the device
 * is not ready.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null.
 * @param mode A uint8_t value representing the calibration mode. Valid modes
 * are defined in the ad719x_adc_modes enumeration.
 * @param channel A uint8_t value representing the channel to calibrate. Valid
 * channels are typically defined as constants like AD719X_CH_0,
 * AD719X_CH_1, etc.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if an error occurs during channel selection, mode setting, or if
 * the device is not ready.
 ******************************************************************************/
int ad719x_calibrate(struct ad719x_dev *dev,
		     uint8_t mode, uint8_t channel);

/***************************************************************************//**
 * @brief Use this function to set the input mode of an AD7193 or AD7194 ADC
 * device. It must be called with a valid device structure and is only
 * supported for these specific chip IDs. The function updates the
 * device's configuration register to reflect the desired input mode and
 * stores the mode setting within the device structure. If the device is
 * not an AD7193 or AD7194, the function returns an error indicating that
 * the operation is not supported.
 *
 * @param dev A pointer to an ad719x_dev structure representing the ADC device.
 * This must not be null and should be properly initialized with a
 * supported chip ID (AD7193 or AD7194). The caller retains
 * ownership.
 * @param mode A uint8_t value representing the desired input mode. Valid values
 * depend on the specific configuration options available for the
 * AD7193 and AD7194 devices. Invalid modes are not explicitly
 * handled, so ensure the mode is valid for the device.
 * @return Returns 0 on success, or a negative error code if the operation is
 * not supported or if an error occurs while setting the register value.
 ******************************************************************************/
int ad719x_config_input_mode(struct ad719x_dev *dev, uint8_t mode);

/***************************************************************************//**
 * @brief Use this function to control the buffer setting of the ADC input
 * channels, which can be enabled or disabled based on the application
 * requirements. This function should be called when you need to change
 * the buffer configuration of the ADC. It updates the device
 * configuration register to reflect the buffer setting and stores the
 * last buffer setting in the device structure. Ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null.
 * @param buff_en A uint8_t value indicating whether to enable (non-zero) or
 * disable (zero) the buffer. Values other than 0 or 1 are
 * treated as enabling the buffer.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a non-zero value indicates an error occurred during
 * the operation.
 ******************************************************************************/
int ad719x_buffer_select(struct ad719x_dev *dev, uint8_t buff_en);

/***************************************************************************//**
 * @brief Use this function to set the desired output data rate for the AD719x
 * ADC device. This function should be called after the device has been
 * initialized and is ready for configuration. It updates the mode
 * register with the specified output rate code and stores this setting
 * in the device structure for future reference. Ensure that the device
 * pointer is valid and properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * ADC device. Must not be null.
 * @param out_rate_code A 16-bit unsigned integer representing the desired
 * output data rate code. Valid values depend on the
 * specific ADC configuration and requirements.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad719x_output_rate_select(struct ad719x_dev *dev,
			      uint16_t out_rate_code);

/***************************************************************************//**
 * @brief Use this function to configure the clock source for the AD719X ADC
 * device. It must be called with a valid device structure and a clock
 * source enumeration value. This function updates the device's clock
 * source setting and returns a status code indicating success or
 * failure. Ensure the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * ADC device. Must not be null.
 * @param clk_select An enumeration value of type ad719x_adc_clock specifying
 * the desired clock source. Valid values are defined in the
 * ad719x_adc_clock enum.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad719x_clock_select(struct ad719x_dev *dev,
			enum ad719x_adc_clock clk_select);

/***************************************************************************//**
 * @brief This function is used to control the bridge power-down switch of an
 * AD719X device, except for the AD7194 model, which does not support
 * this feature. It should be called when you need to enable or disable
 * the bridge power-down switch, which is part of the device's general-
 * purpose output control. The function must be called with a valid
 * device structure that has been properly initialized. If the device is
 * an AD7194, the function will return an error indicating that the
 * operation is not supported. Otherwise, it will attempt to set the
 * switch state as specified by the input parameter.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is an AD7194.
 * @param bpdsw_select A uint8_t value indicating the desired state of the
 * bridge power-down switch. Typically, 0 to disable and 1
 * to enable. Invalid values may result in undefined
 * behavior.
 * @return Returns 0 on success, a negative error code on failure, or -ENOTSUP
 * if the device is an AD7194.
 ******************************************************************************/
int ad719x_set_bridge_switch(struct ad719x_dev *dev, uint8_t bpdsw_select);

/***************************************************************************//**
 * @brief This function configures the ADC's input range by setting the desired
 * polarity and gain. It should be called when you need to adjust the
 * ADC's input characteristics to match the requirements of your
 * application. The function updates the device's configuration register
 * with the specified settings and stores the current polarity and gain
 * in the device structure for future reference. Ensure that the device
 * is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * ADC device. Must not be null.
 * @param polarity A uint8_t value representing the desired polarity setting.
 * Typically, 0 for bipolar and 1 for unipolar operation.
 * @param gain An enum ad719x_adc_gain value specifying the desired gain
 * setting. Valid values are AD719X_ADC_GAIN_1, AD719X_ADC_GAIN_8,
 * AD719X_ADC_GAIN_16, AD719X_ADC_GAIN_32, AD719X_ADC_GAIN_64, and
 * AD719X_ADC_GAIN_128.
 * @return Returns 0 on success, or a negative error code if the configuration
 * fails.
 ******************************************************************************/
int ad719x_range_setup(struct ad719x_dev *dev,
		       uint8_t polarity, enum ad719x_adc_gain range);

/***************************************************************************//**
 * @brief This function initiates a single conversion on the AD719x device and
 * retrieves the conversion result. It should be called when a single
 * measurement is required from the ADC. The function requires a valid
 * device structure and a non-null pointer to store the conversion
 * result. It returns an error code if the result pointer is null or if
 * any step in the conversion process fails.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null.
 * @param reg_data A pointer to a uint32_t variable where the conversion result
 * will be stored. Must not be null; otherwise, the function
 * returns an error.
 * @return Returns 0 on success, or a negative error code if the conversion
 * fails or if reg_data is null.
 ******************************************************************************/
int ad719x_single_conversion(struct ad719x_dev *dev, uint32_t *reg_data);

/***************************************************************************//**
 * @brief This function is used to obtain the average value of a specified
 * number of continuous readings from an AD719x device. It is useful when
 * you need a stable measurement by averaging out noise over several
 * samples. The function must be called with a properly initialized
 * device structure. It configures the device to continuous conversion
 * mode and reads the specified number of samples, accumulating their
 * values. The average is then calculated and stored in the provided
 * output parameter. Ensure that the device is ready for data acquisition
 * and that the output pointer is valid before calling this function.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null.
 * @param sample_number The number of samples to read and average. Must be
 * greater than zero.
 * @param samples_avg A pointer to a uint32_t where the average of the samples
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a non-zero error code if the operation
 * fails.
 ******************************************************************************/
int ad719x_continuous_read_avg(struct ad719x_dev *dev,
			       uint8_t sample_number, uint32_t *samples_avg);

/***************************************************************************//**
 * @brief Use this function to obtain the temperature reading from the AD719x
 * device in Celsius. It must be called with a valid device structure
 * that has been properly initialized. The function configures the device
 * for temperature measurement, performs a single conversion, and
 * calculates the temperature in Celsius. Ensure that the device is ready
 * and properly configured before calling this function. The function
 * returns an error code if the operation fails at any step.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param temp A pointer to a float where the temperature in Celsius will be
 * stored. Must not be null. The function writes the result to this
 * location.
 * @return Returns 0 on success or a negative error code if the operation fails
 * at any step.
 ******************************************************************************/
int ad719x_temperature_read(struct ad719x_dev *dev, float *temp);

/***************************************************************************//**
 * @brief This function is used to convert raw data obtained from the AD719X ADC
 * into a voltage value expressed in millivolts. It should be called
 * after acquiring raw ADC data to interpret the measurement in terms of
 * voltage. The function takes into account the device's current polarity
 * mode (bipolar or unipolar) and gain settings to accurately compute the
 * voltage. Ensure that the device structure is properly initialized and
 * configured before calling this function.
 *
 * @param dev A pointer to an initialized ad719x_dev structure representing the
 * ADC device. Must not be null.
 * @param raw_data The 24-bit raw data from the ADC, represented as a 32-bit
 * unsigned integer. It should be the result of a conversion
 * operation.
 * @param v_ref The reference voltage used by the ADC, expressed as a float. It
 * should be a positive, non-zero value.
 * @return Returns the calculated voltage in millivolts as a float.
 ******************************************************************************/
float ad719x_convert_to_volts(struct ad719x_dev *dev,
			      uint32_t raw_data, float v_ref);

#endif /* __AD719X_H__ */
