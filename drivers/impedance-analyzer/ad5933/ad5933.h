/***************************************************************************//**
 *   @file   AD5933.h
 *   @brief  Header file of AD5933 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
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
#ifndef __AD5933_H__
#define __AD5933_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_i2c.h"

/******************************************************************************/
/************************** AD5933 Definitions ********************************/
/******************************************************************************/

/* AD5933 Registers */
#define AD5933_REG_CONTROL_HB       0x80    // HB of the Control register
#define AD5933_REG_CONTROL_LB       0x81    // LB of the Control register
#define AD5933_REG_FREQ_START       0x82    // Start frequency
#define AD5933_REG_FREQ_INC         0x85    // Frequency increment
#define AD5933_REG_INC_NUM          0x88    // Number of increments
#define AD5933_REG_SETTLING_CYCLES  0x8A    // Number of settling time cycles
#define AD5933_REG_STATUS           0x8F    // Status
#define AD5933_REG_TEMP_DATA        0x92    // Temperature data
#define AD5933_REG_REAL_DATA        0x94    // Real data
#define AD5933_REG_IMAG_DATA        0x96    // Imaginary data

/* AD5933_REG_CONTROL_HB Bits */
#define AD5933_CONTROL_FUNCTION(x)  ((x) << 4)
#define AD5933_CONTROL_RANGE(x)     ((x) << 1)
#define AD5933_CONTROL_PGA_GAIN(x)  ((x) << 0)

/* AD5933_REG_CONTROL_LB Bits */
#define AD5933_CONTROL_RESET        (0x1 << 4)
#define AD5933_CONTROL_INT_SYSCLK   (0x0 << 3)
#define AD5933_CONTROL_EXT_SYSCLK   (0x1 << 3)

/* AD5933_CONTROL_FUNCTION(x) options */
#define AD5933_FUNCTION_NOP                 0x0
#define AD5933_FUNCTION_INIT_START_FREQ     0x1
#define AD5933_FUNCTION_START_SWEEP         0x2
#define AD5933_FUNCTION_INC_FREQ            0x3
#define AD5933_FUNCTION_REPEAT_FREQ         0x4
#define AD5933_FUNCTION_MEASURE_TEMP        0x9
#define AD5933_FUNCTION_POWER_DOWN          0xA
#define AD5933_FUNCTION_STANDBY             0xB

/* AD5933_CONTROL_RANGE(x) options */
#define AD5933_RANGE_2000mVpp       0x0
#define AD5933_RANGE_200mVpp        0x1
#define AD5933_RANGE_400mVpp        0x2
#define AD5933_RANGE_1000mVpp       0x3

/* AD5933_CONTROL_PGA_GAIN(x) options */
#define AD5933_GAIN_X5              0
#define AD5933_GAIN_X1              1

/* AD5933 Default number of settling cycles */
#define AD5933_15_CYCLES			15

/* AD5933 settling cycles mulitiplier */
#define AD5933_SETTLING_X1			0
#define AD5933_SETTLING_X2			1
#define AD5933_SETTLING_X4			3

/* AD5933_REG_STATUS Bits */
#define AD5933_STAT_TEMP_VALID      (0x1 << 0)
#define AD5933_STAT_DATA_VALID      (0x1 << 1)
#define AD5933_STAT_SWEEP_DONE      (0x1 << 2)

/* AD5933 Address */
#define AD5933_ADDRESS              0x0D

/* AD5933 Block Commands */
#define AD5933_BLOCK_WRITE          0xA0
#define AD5933_BLOCK_READ           0xA1
#define AD5933_ADDR_POINTER         0xB0

/* AD5933 Specifications */
#define AD5933_INTERNAL_SYS_CLK     16000000ul      // 16MHz
#define AD5933_MAX_INC_NUM          511             // Maximum increment number

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad5933_dev` structure is designed to encapsulate the state and
 * configuration of an AD5933 device, which is a high precision impedance
 * converter. It includes a pointer to an I2C descriptor for
 * communication, and fields to store the current system clock frequency,
 * clock source, gain, and range settings. This structure is essential
 * for managing the device's configuration and operation in applications
 * requiring impedance measurements.
 *
 * @param i2c_desc Pointer to a structure describing the I2C interface used for
 * communication.
 * @param current_sys_clk Stores the current system clock frequency in Hertz.
 * @param current_clock_source Indicates the current clock source being used by
 * the device.
 * @param current_gain Represents the current gain setting of the device.
 * @param current_range Specifies the current range setting of the device.
 ******************************************************************************/
struct ad5933_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
	/* Device Settings */
	uint32_t current_sys_clk;
	uint8_t current_clock_source;
	uint8_t current_gain;
	uint8_t current_range;
};

/***************************************************************************//**
 * @brief The `ad5933_init_param` structure is used to initialize the AD5933
 * device, a high-precision impedance converter. It contains parameters
 * for setting up the I2C communication interface and configuring the
 * device's operational settings, such as system clock frequency, clock
 * source, gain, and range. This structure is essential for ensuring the
 * device is correctly configured before operation.
 *
 * @param i2c_init Holds the initialization parameters for the I2C communication
 * interface.
 * @param current_sys_clk Stores the current system clock frequency in Hertz.
 * @param current_clock_source Indicates the current source of the system clock.
 * @param current_gain Represents the current gain setting of the device.
 * @param current_range Specifies the current range setting of the device.
 ******************************************************************************/
struct ad5933_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
	/* Device Settings */
	uint32_t current_sys_clk;
	uint8_t current_clock_source;
	uint8_t current_gain;
	uint8_t current_range;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the AD5933 device by allocating necessary
 * resources and initializing the communication interface using the
 * provided parameters. It must be called before any other operations on
 * the device to ensure proper configuration. The function requires valid
 * initialization parameters and will return an error if memory
 * allocation fails or if the communication interface cannot be
 * initialized.
 *
 * @param device A pointer to a pointer of type `struct ad5933_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct ad5933_init_param` containing
 * initialization parameters such as system clock, clock
 * source, gain, and range. These parameters must be correctly
 * set before calling the function.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a negative value indicates an error, such as memory
 * allocation failure or communication initialization failure.
 ******************************************************************************/
int32_t ad5933_init(struct ad5933_dev **device,
		    struct ad5933_init_param init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * AD5933 device when it is no longer needed. This function should be
 * called to clean up after the device has been initialized and used,
 * ensuring that any associated I2C resources are also freed. It is
 * important to call this function to prevent memory leaks and to ensure
 * that the I2C descriptor is properly removed. The function returns a
 * status code indicating the success or failure of the resource removal
 * process.
 *
 * @param dev A pointer to an ad5933_dev structure representing the device to be
 * removed. This pointer must not be null, and it is the caller's
 * responsibility to ensure that the device has been properly
 * initialized before calling this function. The function will handle
 * freeing the memory associated with this structure.
 * @return Returns an int32_t status code from the I2C removal process, where a
 * non-zero value indicates an error.
 ******************************************************************************/
int32_t ad5933_remove(struct ad5933_dev *dev);

/***************************************************************************//**
 * @brief Use this function to write a specified value into a register of the
 * AD5933 device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function allows
 * writing a multi-byte value to a register, with the number of bytes
 * specified by the user. This function is typically used to configure
 * the device by setting various control and configuration registers.
 * Care should be taken to provide the correct register address and the
 * appropriate number of bytes to write, as incorrect values may lead to
 * undefined behavior.
 *
 * @param dev A pointer to an initialized ad5933_dev structure representing the
 * device. Must not be null.
 * @param register_address The address of the register to write to. Must be a
 * valid register address for the AD5933.
 * @param register_value The value to write into the register. The value should
 * fit within the number of bytes specified by
 * bytes_number.
 * @param bytes_number The number of bytes to write to the register. Must be
 * between 1 and 4, inclusive.
 * @return None
 ******************************************************************************/
void ad5933_set_register_value(struct ad5933_dev *dev,
			       uint8_t register_address,
			       uint32_t register_value,
			       uint8_t bytes_number);

/***************************************************************************//**
 * @brief Use this function to retrieve the value stored in a specific register
 * of the AD5933 device. It is essential to ensure that the device has
 * been properly initialized before calling this function. The function
 * reads a specified number of bytes from the register, starting from the
 * given address, and combines them into a single 32-bit value. This
 * function is useful for obtaining configuration or status information
 * from the device. Ensure that the number of bytes requested does not
 * exceed the size of the register or the buffer limits.
 *
 * @param dev A pointer to an initialized ad5933_dev structure representing the
 * device. Must not be null.
 * @param register_address The starting address of the register to read from.
 * Must be a valid register address within the device's
 * address space.
 * @param bytes_number The number of bytes to read from the register. Must be
 * between 1 and the maximum number of bytes that can be
 * read from the register.
 * @return Returns a 32-bit unsigned integer representing the combined value of
 * the bytes read from the specified register.
 ******************************************************************************/
uint32_t ad5933_get_register_value(struct ad5933_dev *dev,
				   uint8_t register_address,
				   uint8_t bytes_number);

/***************************************************************************//**
 * @brief Use this function to reset the AD5933 device, which is typically
 * necessary when initializing the device or recovering from an error
 * state. This function should be called when the device needs to be
 * returned to a known default state, ensuring that all settings are
 * cleared and the device is ready for new configurations. It is
 * important to ensure that the device structure is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad5933_dev structure representing the
 * device to be reset. Must not be null. The caller retains ownership
 * of the structure.
 * @return None
 ******************************************************************************/
void ad5933_reset(struct ad5933_dev *dev);

/***************************************************************************//**
 * @brief This function configures the AD5933 device to use either its internal
 * system clock or an external clock source. It should be called when
 * there is a need to switch between clock sources, such as when using an
 * external clock for synchronization purposes. The function updates the
 * device's current clock source and system clock frequency based on the
 * provided parameters. It is important to ensure that the device is
 * properly initialized before calling this function. The function does
 * not perform validation on the clock source or frequency values, so the
 * caller must ensure they are valid.
 *
 * @param dev A pointer to an ad5933_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param clk_source An int8_t value indicating the desired clock source. It can
 * be AD5933_CONTROL_INT_SYSCLK for the internal clock or
 * AD5933_CONTROL_EXT_SYSCLK for an external clock.
 * @param ext_clk_freq A uint32_t value representing the frequency of the
 * external clock in Hz. This parameter is only used if
 * clk_source is AD5933_CONTROL_EXT_SYSCLK.
 * @return None
 ******************************************************************************/
void ad5933_set_system_clk(struct ad5933_dev *dev,
			   int8_t clk_source,
			   uint32_t ext_clk_freq);

/***************************************************************************//**
 * @brief This function configures the AD5933 device by setting its output
 * voltage range and programmable gain amplifier (PGA) gain. It should be
 * called whenever a change in the measurement configuration is required,
 * such as before starting a frequency sweep or measurement. The function
 * updates the device's control register with the specified range and
 * gain settings and stores these settings in the device structure for
 * future reference. It is important to ensure that the device has been
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad5933_dev structure representing the device. Must
 * not be null, and the device must be initialized prior to calling
 * this function. The caller retains ownership.
 * @param range An int8_t value specifying the desired output voltage range.
 * Valid values are defined by AD5933_CONTROL_RANGE(x) options,
 * such as AD5933_RANGE_2000mVpp, AD5933_RANGE_200mVpp, etc.
 * Invalid values may result in undefined behavior.
 * @param gain An int8_t value specifying the desired PGA gain. Valid values are
 * defined by AD5933_CONTROL_PGA_GAIN(x) options, such as
 * AD5933_GAIN_X5 or AD5933_GAIN_X1. Invalid values may result in
 * undefined behavior.
 * @return None
 ******************************************************************************/
void ad5933_set_range_and_gain(struct ad5933_dev *dev,
			       int8_t range,
			       int8_t gain);

/***************************************************************************//**
 * @brief Use this function to obtain the current temperature reading from the
 * AD5933 device. It must be called with a valid device structure that
 * has been properly initialized. The function communicates with the
 * device to measure the temperature and waits until the measurement is
 * valid before returning the result. This function is useful for
 * applications that require temperature compensation or monitoring of
 * the device's operating environment.
 *
 * @param dev A pointer to an initialized ad5933_dev structure representing the
 * device. Must not be null. The caller retains ownership of the
 * structure.
 * @return Returns the temperature in degrees Celsius as a float. The value is
 * derived from the device's temperature register and adjusted according
 * to the device's specifications.
 ******************************************************************************/
float ad5933_get_temperature(struct ad5933_dev *dev);

/***************************************************************************//**
 * @brief This function sets up the frequency sweep parameters for the AD5933
 * device, including the start frequency, frequency increment, and the
 * number of increments. It must be called after the device has been
 * initialized and before starting a frequency sweep. The function
 * ensures that the number of increments does not exceed the device's
 * maximum limit. This setup is crucial for applications requiring
 * precise frequency sweeps, such as impedance spectroscopy.
 *
 * @param dev A pointer to an initialized ad5933_dev structure representing the
 * device. Must not be null.
 * @param start_freq The starting frequency for the sweep in Hz. Must be a
 * positive integer.
 * @param inc_freq The frequency increment in Hz for each step of the sweep.
 * Must be a positive integer.
 * @param inc_num The number of frequency increments. Must be a positive integer
 * and will be clamped to AD5933_MAX_INC_NUM if it exceeds this
 * value.
 * @return None
 ******************************************************************************/
void ad5933_config_sweep(struct ad5933_dev *dev,
			 uint32_t  start_freq,
			 uint32_t  inc_freq,
			 uint16_t inc_num);

/***************************************************************************//**
 * @brief This function is used to start a frequency sweep operation on an
 * AD5933 device. It should be called after the device has been properly
 * initialized and configured with the desired sweep parameters. The
 * function sets the device to standby mode, resets it, and then
 * initiates the sweep starting from the initial frequency. It waits
 * until the data is valid before returning, ensuring that the sweep has
 * started correctly. This function is essential for applications
 * requiring frequency response analysis using the AD5933.
 *
 * @param dev A pointer to an initialized ad5933_dev structure representing the
 * device. Must not be null. The caller retains ownership and is
 * responsible for ensuring the device is properly configured before
 * calling this function.
 * @return None
 ******************************************************************************/
void ad5933_start_sweep(struct ad5933_dev *dev);

/***************************************************************************//**
 * @brief This function retrieves the real and imaginary data from the AD5933
 * device after a frequency function has been executed. It should be
 * called when the data is ready to be read, typically after a frequency
 * sweep or measurement operation. The function requires a valid device
 * structure and will not perform any operation if the device pointer is
 * null. It waits until the data is valid before reading and storing the
 * results in the provided pointers for real and imaginary data.
 *
 * @param dev A pointer to an ad5933_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param freq_function A uint8_t value representing the frequency function to
 * be executed. Valid values are defined by the
 * AD5933_CONTROL_FUNCTION macros.
 * @param imag_data A pointer to a short where the imaginary data will be
 * stored. Must not be null. The function writes the retrieved
 * imaginary data to this location.
 * @param real_data A pointer to a short where the real data will be stored.
 * Must not be null. The function writes the retrieved real
 * data to this location.
 * @return None
 ******************************************************************************/
void ad5933_get_data(struct ad5933_dev *dev,
		     uint8_t freq_function,
		     short *imag_data,
		     short *real_data);

/***************************************************************************//**
 * @brief Use this function to calculate the gain factor required for impedance
 * measurements with the AD5933 device. This function should be called
 * after initializing the device and configuring the frequency function.
 * The gain factor is calculated based on the real and imaginary data
 * obtained from the device and the provided calibration impedance.
 * Ensure that the device is properly configured and the frequency
 * function is set before calling this function to obtain accurate
 * results.
 *
 * @param dev A pointer to an initialized ad5933_dev structure representing the
 * device. Must not be null.
 * @param calibration_impedance The known impedance value used for calibration,
 * in ohms. Must be a positive non-zero value.
 * @param freq_function The frequency function to be used for data retrieval.
 * Must be a valid function code as defined by the device's
 * specifications.
 * @return Returns the calculated gain factor as a double, which is used in
 * subsequent impedance calculations.
 ******************************************************************************/
double ad5933_calculate_gain_factor(struct ad5933_dev *dev,
				    uint32_t calibration_impedance,
				    uint8_t freq_function);

/***************************************************************************//**
 * @brief This function calculates the impedance by reading the real and
 * imaginary data from the AD5933 device and using a provided gain
 * factor. It should be called after the device has been properly
 * initialized and configured for the desired frequency function. The
 * function requires a valid device structure and a gain factor that has
 * been previously calculated or determined. It is important to ensure
 * that the frequency function is correctly set to obtain accurate data.
 *
 * @param dev A pointer to an initialized ad5933_dev structure representing the
 * device. Must not be null.
 * @param gain_factor A double representing the gain factor used in the
 * impedance calculation. Must be a non-zero positive value
 * to avoid division by zero.
 * @param freq_function A uint8_t representing the frequency function to be used
 * for data acquisition. Must be a valid function code as
 * defined by the device specifications.
 * @return Returns a double representing the calculated impedance based on the
 * real and imaginary data and the provided gain factor.
 ******************************************************************************/
double ad5933_calculate_impedance(struct ad5933_dev *dev,
				  double gain_factor,
				  uint8_t freq_function);

/***************************************************************************//**
 * @brief Use this function to configure the settling time of the AD5933 device
 * by specifying a multiplier and the number of cycles. This function is
 * typically called after initializing the device and before starting a
 * frequency sweep to ensure the device has adequate time to settle
 * between frequency increments. The multiplier must be one of the
 * predefined constants (AD5933_SETTLING_X1, AD5933_SETTLING_X2,
 * AD5933_SETTLING_X4), and if an invalid multiplier is provided, it
 * defaults to AD5933_SETTLING_X1. The number of cycles is combined with
 * the multiplier to set the settling time.
 *
 * @param dev A pointer to an initialized ad5933_dev structure representing the
 * device. Must not be null.
 * @param multiplier A uint8_t value representing the settling time multiplier.
 * Valid values are AD5933_SETTLING_X1, AD5933_SETTLING_X2,
 * and AD5933_SETTLING_X4. Invalid values default to
 * AD5933_SETTLING_X1.
 * @param number_cycles A uint16_t value specifying the number of settling
 * cycles. Must be a valid cycle count for the device.
 * @return None
 ******************************************************************************/
void ad5933_set_settling_time(struct ad5933_dev *dev,
			      uint8_t mulitplier,
			      uint16_t number_cycles);

#endif /* __AD5933_H__ */
