/**************************************************************************//**
*   @file   ad5110.h
*   @brief  Header file of ad5110 Driver for Microblaze processor.
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
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*****************************************************************************/
#ifndef _ad5110_H_
#define _ad5110_H_

/*****************************************************************************/
/****************************** Include Files ********************************/
/*****************************************************************************/
#include <stdint.h>
#include "no_os_i2c.h"

/******************************************************************************/
/************************* Input shift register *******************************/
/******************************************************************************/

/*!< Command position in transmitted bytes */
#define COMMAND                 8
/*!< Available Commands */
#define CMD_NOP                 0
#define CMD_WR_RDAC_EEPROM      1
#define CMD_WR_RDAC             2
#define CMD_SHUT_DOWN           3
#define CMD_RESET               4
#define CMD_RD_RDAC             5
#define CMD_RD_EEPROM           6
/*!< Shutdown modes */
#define SHUT_DOWN_OFF           0
#define SHUT_DOWN_ON            1
/*!< Read modes */
#define WIPER_POSITION          0
#define RESISTOR_TOLERANCE      1

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad5110_dev` structure is designed to encapsulate the necessary
 * components for interfacing with an AD5110 device over I2C. It includes
 * a pointer to an I2C descriptor, which facilitates communication with
 * the device, and a device address that uniquely identifies the AD5110
 * on the I2C bus. This structure is essential for initializing and
 * managing the communication with the AD5110 digital potentiometer,
 * allowing for operations such as reading and writing to the RDAC and
 * EEPROM, as well as performing device resets and shutdowns.
 *
 * @param i2c_desc A pointer to a no_os_i2c_desc structure, representing the I2C
 * descriptor for communication.
 * @param ad5110_dev_addr An 8-bit unsigned integer representing the device
 * address for the AD5110 on the I2C bus.
 ******************************************************************************/
struct ad5110_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
	/* Device Settings */
	uint8_t ad5110_dev_addr;
};

/***************************************************************************//**
 * @brief The `ad5110_init_param` structure is used to encapsulate the
 * initialization parameters required to set up communication with the
 * AD5110 device over an I2C interface. It includes the I2C
 * initialization parameters and the specific device address on the I2C
 * bus, allowing for proper configuration and communication with the
 * AD5110 digital potentiometer.
 *
 * @param i2c_init This member is a structure that holds the initialization
 * parameters for the I2C communication interface.
 * @param ad5110_dev_addr This member is an 8-bit unsigned integer representing
 * the device address for the AD5110 on the I2C bus.
 ******************************************************************************/
struct ad5110_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
	/* Device Settings */
	uint8_t ad5110_dev_addr;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the communication with the AD5110 device by
 * initializing the necessary I2C interface and allocating resources for
 * the device structure. It should be called before any other operations
 * on the AD5110 device to ensure that the device is properly configured
 * and ready for communication. The function requires valid
 * initialization parameters and will return an error code if the
 * initialization fails, such as when memory allocation is unsuccessful.
 *
 * @param device A pointer to a pointer of type `struct ad5110_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct ad5110_init_param` containing
 * the initialization parameters for the AD5110 device,
 * including the I2C initialization parameters and the device
 * address. The values must be valid and correctly configured
 * for the target device.
 * @return Returns an `int8_t` status code. A return value of 0 indicates
 * success, while a negative value indicates an error, such as memory
 * allocation failure.
 ******************************************************************************/
int8_t ad5110_init(struct ad5110_dev **device,
		   struct ad5110_init_param init_param);

/***************************************************************************//**
 * @brief Use this function to properly release resources associated with an
 * AD5110 device instance when it is no longer needed. This function
 * should be called to clean up after a successful call to ad5110_init,
 * ensuring that any allocated memory and I2C resources are freed. It is
 * important to call this function to prevent memory leaks and to
 * properly close the I2C communication channel associated with the
 * device.
 *
 * @param dev A pointer to an ad5110_dev structure representing the device
 * instance to be removed. This pointer must not be null and should
 * point to a valid device instance that was previously initialized
 * using ad5110_init. Passing an invalid or null pointer may result
 * in undefined behavior.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error occurred during the removal
 * process.
 ******************************************************************************/
int32_t ad5110_remove(struct ad5110_dev *dev);

/***************************************************************************//**
 * @brief This function writes a specified value to the RDAC register of an
 * AD5110 device over I2C. It should be called when you need to update
 * the RDAC register with a new value. Ensure that the device has been
 * properly initialized using `ad5110_init` before calling this function.
 * The function does not perform any validation on the `rdac_value`, so
 * it is the caller's responsibility to ensure that the value is within
 * the valid range for the device.
 *
 * @param dev A pointer to an `ad5110_dev` structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param rdac_value An 8-bit unsigned integer representing the value to be
 * written to the RDAC register. The caller must ensure this
 * value is within the valid range for the device.
 * @return None
 ******************************************************************************/
void ad5110_write_rdac(struct ad5110_dev *dev,
		       uint8_t rdac_value);

/***************************************************************************//**
 * @brief Use this function to retrieve the current value stored in the RDAC
 * register of the AD5110 device. This function is typically called when
 * you need to know the current wiper position of the digital
 * potentiometer. Ensure that the device has been properly initialized
 * using `ad5110_init` before calling this function. The function
 * communicates with the device over I2C and returns the RDAC value as an
 * 8-bit unsigned integer.
 *
 * @param dev A pointer to an `ad5110_dev` structure representing the device.
 * This must be a valid, non-null pointer to a properly initialized
 * device structure. The caller retains ownership of this pointer.
 * @return Returns an 8-bit unsigned integer representing the current value of
 * the RDAC register.
 ******************************************************************************/
uint8_t ad5110_read_rdac(struct ad5110_dev *dev);

/***************************************************************************//**
 * @brief Use this function to store the current RDAC register value into the
 * EEPROM of the AD5110 device, ensuring that the configuration is
 * retained across power cycles. This function should be called when the
 * device is properly initialized and configured, and the RDAC value
 * needs to be saved permanently. It is important to ensure that the
 * `dev` parameter is a valid pointer to an initialized `ad5110_dev`
 * structure.
 *
 * @param dev A pointer to an `ad5110_dev` structure representing the device.
 * This must be a valid, non-null pointer to a properly initialized
 * device structure. The function does not handle null pointers and
 * expects the device to be ready for I2C communication.
 * @return None
 ******************************************************************************/
void ad5110_write_rdac_eeprom(struct ad5110_dev *dev);

/***************************************************************************//**
 * @brief This function retrieves the current wiper position stored in the
 * EEPROM of the AD5110 digital potentiometer. It is typically used to
 * obtain the last saved wiper position, which can be useful for
 * restoring device settings after a power cycle. The function requires a
 * valid device structure that has been initialized with the appropriate
 * I2C descriptor. It is important to ensure that the device is properly
 * initialized before calling this function to avoid communication
 * errors.
 *
 * @param dev A pointer to an ad5110_dev structure representing the device. This
 * must be a valid, non-null pointer to a device that has been
 * initialized using ad5110_init. The function will not perform any
 * checks on the validity of this pointer, so passing an invalid
 * pointer may result in undefined behavior.
 * @return Returns an 8-bit unsigned integer representing the wiper position
 * read from the EEPROM.
 ******************************************************************************/
uint8_t ad5110_read_wiper(struct ad5110_dev *dev);

/***************************************************************************//**
 * @brief This function retrieves the resistor tolerance value stored in the
 * EEPROM of the AD5110 device. It is typically used to obtain
 * calibration or configuration data that has been previously stored in
 * the device's non-volatile memory. The function must be called with a
 * valid device structure that has been initialized using `ad5110_init`.
 * The function communicates with the device over I2C, so the I2C
 * interface must be properly configured and operational. It is important
 * to ensure that the device is not in a shutdown state when this
 * function is called.
 *
 * @param dev A pointer to an `ad5110_dev` structure representing the device.
 * This structure must be initialized and must not be null. The
 * caller retains ownership of this structure.
 * @return Returns an 8-bit unsigned integer representing the resistor tolerance
 * value read from the EEPROM.
 ******************************************************************************/
uint8_t ad5110_read_res_tolerance(struct ad5110_dev *dev);

/***************************************************************************//**
 * @brief Use this function to perform a software reset on the AD5110 device,
 * which refreshes the RDAC register with the values stored in EEPROM.
 * This function is typically called when you need to ensure that the
 * device is in a known state, particularly after initialization or when
 * recovering from an error state. It requires a valid device structure
 * that has been properly initialized. The function communicates with the
 * device over I2C, so the I2C interface must be correctly configured and
 * operational.
 *
 * @param dev A pointer to an ad5110_dev structure representing the device. This
 * structure must be initialized and must not be null. The function
 * assumes that the I2C descriptor within this structure is valid and
 * ready for communication.
 * @return None
 ******************************************************************************/
void ad5110_reset(struct ad5110_dev *dev);

/***************************************************************************//**
 * @brief This function is used to control the shutdown state of the AD5110
 * device via software. It should be called when you need to either
 * enable or disable the shutdown mode of the device. The function sends
 * a shutdown command to the device using I2C communication. It is
 * important to ensure that the device has been properly initialized
 * before calling this function to avoid communication errors. The
 * function does not return a value, and any errors during I2C
 * communication are not reported back to the caller.
 *
 * @param dev A pointer to an initialized ad5110_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before use.
 * @param value A uint8_t value indicating the desired shutdown state. Use
 * SHUT_DOWN_OFF (0) to disable shutdown and SHUT_DOWN_ON (1) to
 * enable shutdown. Other values are not valid and may result in
 * undefined behavior.
 * @return None
 ******************************************************************************/
void ad5110_shut_down(struct ad5110_dev *dev,
		      uint8_t value);

#endif  // _ad5110_H_
