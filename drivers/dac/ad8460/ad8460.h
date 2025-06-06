/***************************************************************************//**
 *   @file   ad8460.h
 *   @brief  Implementation of AD8460 Driver.
 *   @author John Erasmus Mari Geronimo (johnerasmusmari.geronimo@analog.com)
********************************************************************************
 * Copyright 2025(c) Analog Devices, Inc.
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

#ifndef __AD8460_H__
#define __AD8460_H__

#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"

#define AD8460_HVDAC_DATA_WORD(x)		(0x60 + (2 * (x)))

#define AD8460_HV_RESET_MSK			NO_OS_BIT(7)
#define AD8460_HV_SLEEP_MSK			NO_OS_BIT(4)
#define AD8460_WAVE_GEN_MODE_MSK		NO_OS_BIT(0)

#define AD8460_HVDAC_SLEEP_MSK			NO_OS_BIT(3)

#define AD8460_FAULT_ARM_MSK			NO_OS_BIT(7)
#define AD8460_FAULT_LIMIT_MSK			NO_OS_GENMASK(6, 0)

#define AD8460_APG_MODE_ENABLE_MSK		NO_OS_BIT(5)
#define AD8460_PATTERN_DEPTH_MSK		NO_OS_GENMASK(3, 0)

#define AD8460_QUIESCENT_CURRENT_MSK		NO_OS_GENMASK(7, 0)

#define AD8460_SHUTDOWN_FLAG_MSK		NO_OS_BIT(7)

#define AD8460_DATA_BYTE_LOW_MSK		NO_OS_GENMASK(7, 0)
#define AD8460_DATA_BYTE_HIGH_MSK		NO_OS_GENMASK(5, 0)
#define AD8460_DATA_BYTE_FULL_MSK		NO_OS_GENMASK(13, 0)

#define AD8460_DEFAULT_FAULT_PROTECT		0x00
#define AD8460_DATA_BYTE_WORD_LENGTH		2
#define AD8460_NUM_DATA_WORDS			16
#define AD8460_NOMINAL_VOLTAGE_SPAN		80
#define AD8460_MIN_EXT_RESISTOR_OHMS		2000
#define AD8460_MAX_EXT_RESISTOR_OHMS		20000
#define AD8460_MIN_VREFIO_UV			120000
#define AD8460_MAX_VREFIO_UV			1200000
#define AD8460_ABS_MAX_OVERVOLTAGE_UV		55000000
#define AD8460_ABS_MAX_OVERCURRENT_UA		1000000
#define AD8460_MAX_OVERTEMPERATURE_MC		150000
#define AD8460_MIN_OVERTEMPERATURE_MC		20000
#define AD8460_CURRENT_LIMIT_CONV(x)		((x) / 15625)
#define AD8460_VOLTAGE_LIMIT_CONV(x)		((x) / 1953000)
#define AD8460_TEMP_LIMIT_CONV(x)		(((x) + 266640) / 6510)

/***************************************************************************//**
 * @brief The `ad8460_device` structure is a descriptor for the AD8460 device,
 * encapsulating the necessary components for its operation and
 * configuration. It includes pointers to SPI and GPIO descriptors for
 * communication and control, as well as parameters for the reference
 * voltage and external resistor, which are critical for the device's
 * analog performance. This structure is essential for initializing and
 * managing the AD8460 device within a system.
 *
 * @param spi_desc Pointer to the SPI descriptor used for communication.
 * @param gpio_rstn Pointer to the GPIO descriptor for the reset pin.
 * @param refio_1p2v_mv The REFIO drive voltage specified in millivolts.
 * @param ext_resistor_ohms The resistance of the external resistor connected to
 * FS_ADJ, measured in ohms.
 ******************************************************************************/
struct ad8460_device {
	/** SPI Descriptor */
	struct no_os_spi_desc *spi_desc;
	/** Reset GPIO descriptor */
	struct no_os_gpio_desc *gpio_rstn;
	/** REFIO drive voltage in millivolts */
	int refio_1p2v_mv;
	/** External resistor connected to FS_ADJ in ohms */
	uint32_t ext_resistor_ohms;
};

/***************************************************************************//**
 * @brief The `ad8460_init_param` structure is used to initialize the AD8460
 * device, encapsulating the necessary parameters for SPI and GPIO
 * configurations, as well as specific electrical characteristics such as
 * the REFIO drive voltage and the external resistor value. This
 * structure is essential for setting up the device's communication and
 * operational parameters before it can be used in an application.
 *
 * @param spi_init_param Host processor SPI configuration.
 * @param gpio_rstn Reset GPIO configuration.
 * @param refio_1p2v_mv REFIO drive voltage in millivolts.
 * @param ext_resistor_ohms External resistor connected to FS_ADJ in ohms.
 ******************************************************************************/
struct ad8460_init_param {
	/** Host processor SPI configuration */
	struct no_os_spi_init_param spi_init_param;
	/** Reset GPIO configuration */
	struct no_os_gpio_init_param gpio_rstn;
	/** REFIO drive voltage in millivolts */
	int refio_1p2v_mv;
	/** External resistor connected to FS_ADJ in ohms */
	uint32_t ext_resistor_ohms;
};

/***************************************************************************//**
 * @brief This function reads a value from a specified register of the AD8460
 * device. It is typically used to retrieve configuration or status
 * information from the device. The function requires a valid device
 * descriptor and a register address to read from. The result is stored
 * in the provided output parameter. Ensure that the device has been
 * properly initialized before calling this function. The function
 * returns an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized ad8460_device structure. This must not
 * be null and should represent a valid, initialized device context.
 * @param addr The address of the register to read from. It is an 8-bit value
 * representing the register address within the device.
 * @param val A pointer to a uint8_t where the read value will be stored. This
 * must not be null, and the caller is responsible for providing a
 * valid memory location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ad8460_reg_read(struct ad8460_device *dev, uint8_t addr, uint8_t *val);

/***************************************************************************//**
 * @brief This function writes a given value to a specified register address on
 * the AD8460 device. It is typically used to configure or control the
 * device by setting specific register values. The function requires a
 * valid device descriptor, which must be initialized prior to calling
 * this function. It is important to ensure that the address and value
 * provided are within the valid range for the device's registers. The
 * function returns an integer status code indicating the success or
 * failure of the write operation.
 *
 * @param dev A pointer to an initialized ad8460_device structure. This must not
 * be null and should be properly configured before calling this
 * function.
 * @param addr The register address to which the value will be written. It
 * should be a valid register address for the AD8460 device.
 * @param val The value to write to the specified register. It should be within
 * the valid range for the register's data width.
 * @return Returns an integer status code. A value of 0 typically indicates
 * success, while a negative value indicates an error occurred during
 * the write operation.
 ******************************************************************************/
int ad8460_reg_write(struct ad8460_device *dev, uint8_t addr, uint8_t val);

/***************************************************************************//**
 * @brief This function is used to update specific bits of a register in the
 * AD8460 device by performing a read-modify-write operation. It is
 * useful when only certain bits of a register need to be changed without
 * affecting the other bits. The function first reads the current value
 * of the register, applies a mask to clear the bits to be updated, and
 * then sets the new value for those bits. It is important to ensure that
 * the device is properly initialized before calling this function. The
 * function returns an error code if the read or write operation fails.
 *
 * @param dev A pointer to an ad8460_device structure representing the device.
 * Must not be null, and the device must be initialized before use.
 * @param addr The address of the register to be updated. Must be a valid
 * register address for the AD8460 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected by the
 * operation.
 * @param val The new value to be written to the bits specified by the mask. The
 * value is prepared using the mask to ensure only the intended bits
 * are modified.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int ad8460_reg_update_bits(struct ad8460_device *dev, uint8_t addr,
			   uint8_t mask, uint8_t val);

/***************************************************************************//**
 * @brief This function initializes the AD8460 device by setting up the
 * necessary SPI and GPIO interfaces based on the provided initialization
 * parameters. It must be called before any other operations on the
 * device to ensure that the communication interfaces are properly
 * configured. The function allocates memory for the device descriptor
 * and initializes the SPI and optional GPIO reset line. If any step of
 * the initialization fails, the function will clean up any allocated
 * resources and return an error code. The caller is responsible for
 * freeing the allocated resources by calling `ad8460_remove` when the
 * device is no longer needed.
 *
 * @param dev A pointer to a pointer of type `struct ad8460_device`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A pointer to a `struct ad8460_init_param` containing the
 * initialization parameters for the SPI and GPIO interfaces,
 * as well as the reference voltage and external resistor
 * values. This must not be null, and the values should be
 * within the valid ranges specified in the structure's
 * documentation.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures no resources are left allocated.
 ******************************************************************************/
int ad8460_init(struct ad8460_device **dev,
		struct ad8460_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD8460 device when it is no longer needed. This function should be
 * called to clean up after a device has been initialized and used,
 * ensuring that any allocated memory and hardware resources are freed.
 * It is important to pass a valid device pointer that was previously
 * initialized; otherwise, the function will return an error. This
 * function must be called to prevent resource leaks.
 *
 * @param dev A pointer to an ad8460_device structure representing the device to
 * be removed. Must not be null. If null, the function returns
 * -ENODEV.
 * @return Returns 0 on successful removal of the device. If the device pointer
 * is null, returns -ENODEV. If an error occurs during the removal of
 * SPI resources, the function returns the corresponding error code.
 ******************************************************************************/
int ad8460_remove(struct ad8460_device *dev);

/***************************************************************************//**
 * @brief Use this function to reset the AD8460 device, which will bring all its
 * registers to their default state. This function should be called when
 * a reset of the device is required, such as during initialization or
 * when recovering from an error state. It is important to ensure that
 * the device is properly initialized before calling this function. The
 * function will return an error code if the reset operation fails, which
 * can occur if there is an issue with the GPIO or SPI communication.
 *
 * @param dev A pointer to an initialized `ad8460_device` structure. This
 * parameter must not be null, and the structure must be properly
 * configured with valid GPIO and SPI descriptors. The function will
 * return an error if the device is not correctly initialized.
 * @return Returns 0 on success or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ad8460_reset(struct ad8460_device *dev);

/***************************************************************************//**
 * @brief Use this function to obtain the current value of a high voltage DAC
 * word from the AD8460 device. It is essential to ensure that the device
 * has been properly initialized before calling this function. The
 * function reads the DAC word specified by the index and stores the
 * result in the provided memory location. This function is useful for
 * monitoring or verifying the DAC settings. Handle the return value to
 * check for successful execution or errors during the SPI communication.
 *
 * @param dev A pointer to an initialized ad8460_device structure. Must not be
 * null, and the device should be properly configured before use.
 * @param index An int8_t value specifying the DAC word index to read. Valid
 * indices depend on the device's configuration and should be
 * within the range supported by the device.
 * @param val A pointer to a uint16_t where the read DAC word value will be
 * stored. Must not be null, and the caller is responsible for
 * allocating memory for this pointer.
 * @return Returns 0 on success, or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int ad8460_get_hvdac_word(struct ad8460_device *dev, int8_t index,
			  uint16_t *val);

/***************************************************************************//**
 * @brief This function sets the value of a specified high voltage DAC word on
 * the AD8460 device. It should be used when you need to update the DAC
 * output to a new value. The function requires a valid device descriptor
 * and an index specifying which DAC word to set. The index must be
 * within the valid range for the device. The function prepares the data
 * and sends it over SPI to the device. Ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad8460_device structure representing the device.
 * Must not be null and should be properly initialized.
 * @param index An int8_t value specifying the DAC word index to set. Must be
 * within the valid range of DAC word indices for the device.
 * @param val A uint16_t value representing the new value to set for the
 * specified DAC word. The value is prepared and sent to the device.
 * @return Returns an integer status code. A non-negative value indicates
 * success, while a negative value indicates an error occurred during
 * the SPI communication.
 ******************************************************************************/
int ad8460_set_hvdac_word(struct ad8460_device *dev, int8_t index,
			  uint16_t val);

/***************************************************************************//**
 * @brief This function is used to control the APG (Automatic Pattern Generator)
 * mode of the AD8460 device. It should be called when you need to enable
 * or disable this mode, which is typically used for generating specific
 * signal patterns. The function requires a valid device descriptor and a
 * value indicating whether to enable or disable the mode. It returns an
 * error code if the operation fails, which can occur if the device is
 * not properly initialized or if there is a communication error.
 *
 * @param dev A pointer to an initialized ad8460_device structure. This must not
 * be null, and the device must be properly initialized before
 * calling this function. The caller retains ownership.
 * @param val An integer value where a non-zero value enables the APG mode and
 * zero disables it. The function does not perform validation on this
 * parameter, so it is the caller's responsibility to ensure it is
 * set correctly.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int ad8460_enable_apg_mode(struct ad8460_device *dev, int val);

/***************************************************************************//**
 * @brief This function retrieves the shutdown flag status from the AD8460
 * device and stores it in the provided memory location. It should be
 * called when the user needs to check if the device is in a shutdown
 * state. The function requires a valid device descriptor and a non-null
 * pointer to store the flag. It returns an error code if the read
 * operation fails.
 *
 * @param dev A pointer to an initialized ad8460_device structure. This must not
 * be null and should represent a valid, initialized device.
 * @param flag A pointer to a uint8_t where the shutdown flag will be stored.
 * This must not be null. The function writes 1 if the shutdown flag
 * is set, otherwise 0.
 * @return Returns 0 on success, or a negative error code if the register read
 * operation fails.
 ******************************************************************************/
int ad8460_read_shutdown_flag(struct ad8460_device *dev, uint8_t *flag);

/***************************************************************************//**
 * @brief This function is used to reset the high voltage driver of the AD8460
 * device. It should be called when a reset of the high voltage driver is
 * required, such as during initialization or error recovery. The
 * function performs a read-modify-write operation to set and then clear
 * the reset bit, with a delay in between to ensure proper reset timing.
 * It is important to ensure that the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad8460_device structure. This
 * parameter must not be null, and the device must be properly
 * initialized before calling this function. If the device is not
 * initialized, the behavior is undefined.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad8460_hv_reset(struct ad8460_device *dev);

/***************************************************************************//**
 * @brief This function sets a sample value on the AD8460 device by enabling the
 * APG mode, setting a high voltage DAC word, and updating specific
 * register bits. It should be called when a new sample value needs to be
 * configured on the device. The function requires a valid device
 * descriptor and a sample value to be provided. It returns an error code
 * if any of the operations fail, ensuring that the device state is not
 * partially updated.
 *
 * @param dev A pointer to an ad8460_device structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param val A 16-bit unsigned integer representing the sample value to be set.
 * The valid range is determined by the device's capabilities and
 * should be within the acceptable limits for the high voltage DAC.
 * @return Returns 0 on success or a negative error code if any operation fails.
 ******************************************************************************/
int ad8460_set_sample(struct ad8460_device *dev, uint16_t val);

#endif	/* __AD8460_H__ */
