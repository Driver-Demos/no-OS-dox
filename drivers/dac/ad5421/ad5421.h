/**************************************************************************//**
*   @file   AD5421.h
*   @brief  Header file of AD5421 Driver for Microblaze processor.
*   @author Lucian Sin (Lucian.Sin@analog.com)
*
*******************************************************************************
* Copyright 2013(c) Analog Devices, Inc.
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
******************************************************************************/
#ifndef _AD5421_H_
#define _AD5421_H_

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/*****************************************************************************/
/******************** Macros and Constants Definitions ***********************/
/*****************************************************************************/
/* COMMAND Bytes */
#define AD5421_CMDWRDAC         1
#define AD5421_CMDWRCTRL        2
#define AD5421_CMDWROFFSET      3
#define AD5421_CMDWRGAIN        4
#define AD5421_CMDRST			7
#define AD5421_CMDMEASVTEMP     8
#define AD5421_NO_OP			9
#define AD5421_CMDRDDAC         129
#define AD5421_CMDRDCTRL		130
#define AD5421_CMDRDOFFSET      131
#define AD5421_CMDRDGAIN        132
#define AD5421_CMDRDFAULT       133

/* AD5421 COMMAND mask */
#define AD5421_CMD(x)			((x & 0xFF) << 16)

/* AD5421 GPIO */
#define AD5421_LDAC_OUT			no_os_gpio_direction_output(dev->gpio_ldac,   \
			                NO_OS_GPIO_HIGH)
#define AD5421_LDAC_LOW			no_os_gpio_set_value(dev->gpio_ldac,          \
			                NO_OS_GPIO_LOW)
#define AD5421_LDAC_HIGH		no_os_gpio_set_value(dev->gpio_ldac,          \
			                NO_OS_GPIO_HIGH)
#define AD5421_FAULT_IN 		no_os_gpio_direction_input(dev->gpio_faultin)

/* CONTROL register bits */
#define CTRL_SPI_WATCHDOG		(1 << 12)
#define CTRL_AUTO_FAULT_RDBK    (1 << 11)
#define CTRL_SEL_ADC_INPUT      (1 << 8)
#define CTRL_ONCHIP_ADC         (1 << 7)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad5421_dev` structure is a data structure used to represent an
 * instance of the AD5421 device, which is a digital-to-analog converter
 * (DAC). It contains pointers to SPI and GPIO descriptors that
 * facilitate communication and control of the device's pins,
 * specifically for loading DAC values and handling fault conditions.
 * This structure is essential for managing the hardware interface and
 * operations of the AD5421 device in embedded systems.
 *
 * @param spi_desc Pointer to a SPI descriptor used for SPI communication.
 * @param gpio_ldac Pointer to a GPIO descriptor for the LDAC pin.
 * @param gpio_faultin Pointer to a GPIO descriptor for the FAULTIN pin.
 ******************************************************************************/
struct ad5421_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_ldac;
	struct no_os_gpio_desc	*gpio_faultin;
};

/***************************************************************************//**
 * @brief The `ad5421_init_param` structure is used to encapsulate the
 * initialization parameters required to set up the AD5421 device. It
 * includes configuration details for the SPI interface and two GPIO
 * pins, LDAC and FAULTIN, which are essential for the device's
 * operation. This structure is typically used during the device
 * initialization process to ensure that all necessary communication
 * interfaces are properly configured.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_ldac Contains the initialization parameters for the LDAC GPIO
 * pin.
 * @param gpio_faultin Contains the initialization parameters for the FAULTIN
 * GPIO pin.
 ******************************************************************************/
struct ad5421_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_ldac;
	struct no_os_gpio_init_param	gpio_faultin;
};

/*****************************************************************************/
/************************* Functions Declarations ****************************/
/*****************************************************************************/
/* Initialize the communication with the device. */
/***************************************************************************//**
 * @brief This function sets up the necessary communication interfaces and
 * initializes the AD5421 device for operation. It must be called before
 * any other operations are performed on the device. The function
 * configures the SPI interface and GPIO pins as specified in the
 * initialization parameters. It also writes to the control register to
 * set up the device's operational mode. If the initialization is
 * successful, a device handle is returned through the provided pointer.
 * The caller is responsible for managing the memory of the device handle
 * and must call `ad5421_remove` to free resources when the device is no
 * longer needed.
 *
 * @param device A pointer to a pointer where the initialized device handle will
 * be stored. Must not be null. The caller is responsible for
 * freeing the allocated memory using `ad5421_remove`.
 * @param init_param A structure containing initialization parameters for the
 * SPI and GPIO interfaces. Must be properly configured before
 * calling this function.
 * @return Returns 0 on successful initialization, or -1 if an error occurs
 * (e.g., memory allocation failure or communication setup failure).
 ******************************************************************************/
int32_t ad5421_init(struct ad5421_dev **device,
		    struct ad5421_init_param init_param);
/* Free the resources allocated by ad5421_init(). */
/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * allocated for an AD5421 device instance, typically after the device is
 * no longer needed. It ensures that the SPI and GPIO descriptors
 * associated with the device are removed and the memory allocated for
 * the device structure is freed. This function must be called after the
 * device has been initialized with `ad5421_init` to prevent resource
 * leaks. The function returns a status code indicating the success or
 * failure of the resource deallocation process.
 *
 * @param dev A pointer to an `ad5421_dev` structure representing the device
 * instance to be removed. This pointer must not be null and should
 * point to a valid device structure that was previously initialized
 * with `ad5421_init`. Passing an invalid or null pointer results in
 * undefined behavior.
 * @return Returns an `int32_t` status code. A value of 0 indicates success,
 * while a non-zero value indicates an error occurred during the removal
 * of resources.
 ******************************************************************************/
int32_t ad5421_remove(struct ad5421_dev *dev);
/* Set the value of DAC register. */
/***************************************************************************//**
 * @brief This function sets the digital-to-analog converter (DAC) register of
 * the AD5421 device to a specified value. It should be called when you
 * need to update the output of the DAC. Ensure that the device has been
 * properly initialized using `ad5421_init` before calling this function.
 * The function does not perform any validation on the `dac_value`, so it
 * is the caller's responsibility to ensure that the value is within the
 * valid range for the DAC.
 *
 * @param dev A pointer to an `ad5421_dev` structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param dac_value An integer representing the value to be set in the DAC
 * register. The caller must ensure this value is within the
 * valid range for the DAC.
 * @return None
 ******************************************************************************/
void ad5421_set_dac(struct ad5421_dev *dev,
		    int32_t dac_value);
/* Set the value of OFFSET register. */
/***************************************************************************//**
 * @brief This function is used to configure the offset register of the AD5421
 * device, which is essential for calibrating the output signal. It
 * should be called when an adjustment to the offset is required,
 * typically during device initialization or recalibration. The function
 * requires a valid device structure and an offset value to be provided.
 * It is important to ensure that the device has been properly
 * initialized before calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an ad5421_dev structure representing the device. This
 * must be a valid, initialized device structure and must not be
 * null. The caller retains ownership.
 * @param offset_value An integer representing the offset value to be set in the
 * device's offset register. The valid range is determined
 * by the device's specifications, and the function assumes
 * the provided value is within this range.
 * @return None
 ******************************************************************************/
void ad5421_set_offset(struct ad5421_dev *dev,
		       int32_t offset_value);
/* Set the value of GAIN register. */
/***************************************************************************//**
 * @brief Use this function to configure the gain setting of the AD5421 device
 * by writing a specified value to its GAIN register. This function
 * should be called when you need to adjust the gain for the device's
 * output signal. Ensure that the device has been properly initialized
 * before calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized ad5421_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before use.
 * @param gain_value An integer value representing the gain to be set in the
 * GAIN register. The valid range is determined by the
 * device's specifications, and invalid values may result in
 * incorrect device behavior.
 * @return None
 ******************************************************************************/
void ad5421_set_gain(struct ad5421_dev *dev,
		     int32_t gain_value);
/* Read the DAC register. */
/***************************************************************************//**
 * @brief This function retrieves the current digital-to-analog converter (DAC)
 * value from the AD5421 device. It should be used when you need to
 * obtain the current output setting of the DAC. The function requires a
 * valid device structure that has been initialized using `ad5421_init`.
 * It is important to ensure that the device is properly configured and
 * operational before calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an `ad5421_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @return Returns the current DAC value as a 32-bit integer. If the operation
 * fails, the return value may be an error code.
 ******************************************************************************/
int32_t ad5421_get_dac(struct ad5421_dev *dev);
/* Read OFFSET register. */
/***************************************************************************//**
 * @brief Use this function to retrieve the current value stored in the OFFSET
 * register of the AD5421 device. This function is typically called when
 * you need to read the offset calibration value that has been set in the
 * device. Ensure that the device has been properly initialized using
 * `ad5421_init` before calling this function. The function communicates
 * with the device over SPI to obtain the offset value.
 *
 * @param dev A pointer to an `ad5421_dev` structure representing the device
 * instance. This must be a valid, initialized device structure, and
 * must not be null.
 * @return Returns the 32-bit integer value read from the OFFSET register,
 * representing the offset calibration value.
 ******************************************************************************/
int32_t ad5421_get_offset(struct ad5421_dev *dev);
/* Read GAIN register. */
/***************************************************************************//**
 * @brief Use this function to retrieve the current gain setting from the AD5421
 * device. It is typically called when you need to verify or utilize the
 * gain configuration of the device in your application. Ensure that the
 * device has been properly initialized using `ad5421_init` before
 * calling this function. The function communicates with the device over
 * SPI to read the gain value, which is then returned as an integer.
 * Handle the return value appropriately, as it represents the gain
 * setting of the device.
 *
 * @param dev A pointer to an `ad5421_dev` structure representing the device
 * instance. This must be a valid, initialized device structure, and
 * must not be null. The caller retains ownership of this pointer.
 * @return Returns the gain value as a 32-bit integer. The value represents the
 * current setting of the GAIN register in the AD5421 device.
 ******************************************************************************/
int32_t ad5421_get_gain(struct ad5421_dev *dev);
/* Read FAULT register. */
/***************************************************************************//**
 * @brief Use this function to retrieve the current fault status from the AD5421
 * device. It is typically called to diagnose issues or verify the
 * operational status of the device. Ensure that the device has been
 * properly initialized using `ad5421_init` before calling this function.
 * The function communicates with the device over SPI to read the fault
 * register and returns the fault status as an integer. This function is
 * useful for error handling and system diagnostics.
 *
 * @param dev A pointer to an `ad5421_dev` structure representing the device.
 * This must be a valid, initialized device structure. The pointer
 * must not be null, and the device must be properly configured
 * before calling this function.
 * @return Returns an `int32_t` representing the fault status read from the
 * device's FAULT register. The value indicates the current fault
 * conditions, if any, on the device.
 ******************************************************************************/
int32_t ad5421_get_fault(struct ad5421_dev *dev);
/* Read the temperature from Fault register. */
/***************************************************************************//**
 * @brief Use this function to obtain the current temperature reading from the
 * AD5421 device. It must be called with a valid device structure that
 * has been properly initialized. The function configures the device to
 * measure temperature, initiates the measurement, and retrieves the
 * temperature value from the device's fault register. This function is
 * useful for monitoring the device's operating temperature, which can be
 * critical for ensuring proper operation and preventing overheating.
 * Ensure that the device is not in a fault state before calling this
 * function to get accurate readings.
 *
 * @param dev A pointer to an initialized ad5421_dev structure representing the
 * device. Must not be null. The caller retains ownership and is
 * responsible for ensuring the device is properly initialized before
 * calling this function.
 * @return Returns the temperature as an int32_t value, calculated based on the
 * device's internal measurement and datasheet formula.
 ******************************************************************************/
int32_t ad5421_get_temp(struct ad5421_dev *dev);
/* Read VLoop-COM from Fault register. */
/***************************************************************************//**
 * @brief This function retrieves the VLoop-COM voltage measurement from the
 * AD5421 device, which is useful for monitoring the loop voltage in
 * current loop applications. It must be called with a valid device
 * structure that has been properly initialized using `ad5421_init`. The
 * function communicates with the device over SPI to configure it for
 * VLoop measurement, initiates the measurement, and then reads the
 * result. The function returns the measured voltage as a floating-point
 * value. Ensure that the device is not in a reset state and is properly
 * powered before calling this function.
 *
 * @param dev A pointer to an `ad5421_dev` structure representing the device.
 * This must be a valid, non-null pointer to a device that has been
 * initialized with `ad5421_init`. The caller retains ownership of
 * this structure.
 * @return Returns the measured VLoop-COM voltage as a float, representing the
 * voltage in volts.
 ******************************************************************************/
float ad5421_get_vloop(struct ad5421_dev *dev);
/* Send command via SPI. */
/***************************************************************************//**
 * @brief This function is used to send a command to the AD5421 device using the
 * SPI interface. It should be called when a specific command needs to be
 * transmitted to the device. The function requires a valid device
 * structure and a pointer to the command value to be sent. It is
 * important to ensure that the device has been properly initialized
 * before calling this function. The function returns an error code if
 * the SPI communication fails.
 *
 * @param dev A pointer to an ad5421_dev structure representing the device. This
 * must be a valid, initialized device structure and must not be
 * null.
 * @param value A pointer to an int32_t containing the command value to be sent.
 * The value is expected to be a valid command for the AD5421
 * device. The pointer must not be null.
 * @return Returns 0 on success, or -1 if the SPI communication fails.
 ******************************************************************************/
int32_t ad5421_set(struct ad5421_dev *dev,
		   int32_t *i_value);
/* Receive value via SPI. */
/***************************************************************************//**
 * @brief This function is used to read a value from the AD5421 device using the
 * SPI communication protocol. It should be called when a value needs to
 * be retrieved from the device, typically after the device has been
 * properly initialized using `ad5421_init`. The function expects a valid
 * device structure and will return an error code if the SPI
 * communication fails. It is important to ensure that the device is in a
 * state ready for communication to avoid errors.
 *
 * @param dev A pointer to an `ad5421_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @return Returns the 16-bit value read from the device as an `int32_t`. If the
 * SPI communication fails, it returns -1 as an error code.
 ******************************************************************************/
int32_t ad5421_get(struct ad5421_dev *dev);
/* Resets the AD5421 device. */
/***************************************************************************//**
 * @brief Use this function to reset the AD5421 device, which is typically
 * necessary when you want to ensure the device is in a known state
 * before starting operations. This function should be called after the
 * device has been initialized and before any configuration or data
 * operations are performed. It sends a reset command to the device via
 * SPI, ensuring that all settings are returned to their default values.
 * This function does not return any value and does not provide feedback
 * on the success of the reset operation.
 *
 * @param dev A pointer to an initialized ad5421_dev structure representing the
 * device to be reset. This pointer must not be null, and the device
 * must have been successfully initialized prior to calling this
 * function.
 * @return None
 ******************************************************************************/
void ad5421_reset(struct ad5421_dev *dev);

#endif /* _AD5421_H_ */
