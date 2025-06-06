/***************************************************************************//**
 *   @file   AD5628.h
 *   @brief  Header file of AD5628 Driver.
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

#ifndef __AD5628_H__
#define __AD5628_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/******************************** AD5628 **************************************/
/******************************************************************************/
/* AD5628 Input Register */
#define AD5628_CMD(x)              ((0x000F & (int32_t)(x)) << 24)
#define AD5628_ADDR(x)             ((0x000F & (int32_t)(x)) << 20)
#define AD5628_DATA_BITS(x)        ((0x0FFF & (int32_t)(x)) << 8)

/* Command Definitions (AD5628_COMMAND(x) options) */
#define AD5628_CMD_WRITE_INPUT_N             0 // Write to Input Register n.
#define AD5628_CMD_UPDATE_DAC_N              1 // Update DAC Register n.
#define AD5628_CMD_WRITE_INPUT_N_UPDATE_ALL  2 // Write to Input Register n, update all.
#define AD5628_CMD_WRITE_INPUT_N_UPDATE_N    3 // Write to and update DAC Channel n.
#define AD5628_CMD_POWERDOWN                 4 // Power down/power up DAC.
#define AD5628_CMD_LOAD_CLEAR_CODE           5 // Load clear code register.
#define AD5628_CMD_LOAD_LDAC_REG             6 // Load LDAC register.
#define AD5628_CMD_RESET                     7 // Reset (power-on reset)
#define AD5628_CMD_SET_INT_REF               8 // Set up internal REF register.

/* Address Commands (AD5628_ADDRESS(x) options) */
#define AD5628_ADDR_DAC_A      0x00 //DAC A
#define AD5628_ADDR_DAC_B      0x01 //DAC B
#define AD5628_ADDR_DAC_C      0x02 //DAC C
#define AD5628_ADDR_DAC_D      0x03 //DAC D
#define AD5628_ADDR_DAC_E      0x04 //DAC E
#define AD5628_ADDR_DAC_F      0x05 //DAC F
#define AD5628_ADDR_DAC_G      0x06 //DAC G
#define AD5628_ADDR_DAC_H      0x07 //DAC H
#define AD5628_ADDR_DAC_ALL    0x0F //All DACs

/* Internal Reference Register */
#define AD5628_INT_REF_OFF     0
#define AD5628_INT_REF_ON      1

/* Power-Down Modes of Operation */
#define AD5628_POWER_MODE(x)      ((0x03 & (uint16_t) (x)) << 8)

#define AD5628_PWRDN_NONE         0 // Normal operation
#define AD5628_PWRDN_1K           1 // 1 KOhm to GND    (Power-down mode)
#define AD5628_PWRDN_100K         2 // 100 KOhm to GND  (Power-down mode)
#define AD5628_PWRDN_3STATE       3 // Three-state      (Power-down mode)

/* Clear Code Function */
#define AD5628_CODE_0X0000        0
#define AD5628_CODE_0X8000        1
#define AD5628_CODE_0XFFFF        2
#define AD5628_CODE_NOP           3

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad5628_dev` structure is designed to represent a device instance
 * for the AD5628, a digital-to-analog converter (DAC). It contains a
 * single member, `spi_desc`, which is a pointer to a SPI descriptor that
 * facilitates communication with the device over the SPI protocol. This
 * structure is essential for managing the state and communication of the
 * AD5628 device within the software.
 *
 * @param spi_desc A pointer to a SPI descriptor used for SPI communication.
 ******************************************************************************/
struct ad5628_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
};

/***************************************************************************//**
 * @brief The `ad5628_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the AD5628 device,
 * specifically focusing on the SPI communication interface. It contains
 * a single member, `spi_init`, which is a structure itself that holds
 * the necessary configuration details for initializing the SPI
 * interface, ensuring proper communication with the AD5628 device.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 ******************************************************************************/
struct ad5628_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function initializes the AD5628 device by allocating necessary
 * resources and setting up the SPI communication interface. It performs
 * a reset on the device, enables the internal reference, and sets the
 * clear code to 0x0000. This function must be called before any other
 * operations on the AD5628 device to ensure proper configuration and
 * operation. The caller is responsible for providing valid
 * initialization parameters and handling the device pointer.
 *
 * @param device A pointer to a pointer of type `struct ad5628_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * The caller takes ownership of the allocated memory and is
 * responsible for freeing it using `ad5628_remove`.
 * @param init_param A structure of type `struct ad5628_init_param` containing
 * the initialization parameters for the SPI interface. Must
 * be properly initialized before calling this function.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a negative value indicates an error, such as memory allocation
 * failure.
 ******************************************************************************/
int32_t ad5628_init(struct ad5628_dev **device,
		    struct ad5628_init_param init_param);

/***************************************************************************//**
 * @brief Use this function to release all resources allocated for an AD5628
 * device instance when it is no longer needed. This function should be
 * called to clean up after using the device, ensuring that any
 * associated memory and hardware resources are properly freed. It is
 * important to call this function to prevent resource leaks. The
 * function must be called with a valid device instance that was
 * previously initialized using `ad5628_init`. After calling this
 * function, the device instance should not be used unless re-
 * initialized.
 *
 * @param dev A pointer to an `ad5628_dev` structure representing the device
 * instance to be removed. This must not be null and should point to
 * a valid device instance initialized by `ad5628_init`. Passing an
 * invalid or null pointer results in undefined behavior.
 * @return Returns an `int32_t` status code indicating the success or failure of
 * the operation. A non-zero value indicates an error occurred during
 * the removal process.
 ******************************************************************************/
int32_t ad5628_remove(struct ad5628_dev *dev);

/***************************************************************************//**
 * @brief Use this function to configure the power mode of a specific DAC
 * channel on the AD5628 device. It allows you to set the channel to
 * normal operation or various power-down modes. This function should be
 * called after the device has been initialized. If the special channel
 * identifier for all DACs is used, the power mode will be applied to all
 * channels simultaneously. Ensure that the device pointer is valid
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad5628_dev structure representing the
 * device. Must not be null.
 * @param pwr_mode An 8-bit unsigned integer representing the desired power
 * mode. Valid values are AD5628_PWRDN_NONE, AD5628_PWRDN_1K,
 * AD5628_PWRDN_100K, and AD5628_PWRDN_3STATE.
 * @param channel An 8-bit unsigned integer specifying the DAC channel to
 * configure. Valid values are AD5628_ADDR_DAC_A to
 * AD5628_ADDR_DAC_H for individual channels, or
 * AD5628_ADDR_DAC_ALL to apply the power mode to all channels.
 * @return None
 ******************************************************************************/
void ad5628_power_mode(struct ad5628_dev *dev,
		       uint8_t pwr_mode,
		       uint8_t channel);

/***************************************************************************//**
 * @brief This function is used to reset the AD5628 device, returning it to its
 * default power-on state. It should be called when a reset of the device
 * is necessary, such as during initialization or when recovering from an
 * error state. The function requires a valid device structure that has
 * been properly initialized. It does not return any value and does not
 * provide feedback on the success of the operation, so it is assumed to
 * succeed if the device is correctly set up.
 *
 * @param dev A pointer to an ad5628_dev structure representing the device to be
 * reset. This must not be null and should point to a valid,
 * initialized device structure. The caller retains ownership of this
 * structure.
 * @return None
 ******************************************************************************/
void ad5628_reset(struct ad5628_dev *dev);

/***************************************************************************//**
 * @brief Use this function to send a 32-bit command and data word to the input
 * register of the AD5628 device. This function is typically used to
 * configure the device or update its settings. It requires a valid
 * device structure that has been initialized and configured for SPI
 * communication. Ensure that the device is properly initialized before
 * calling this function to avoid communication errors.
 *
 * @param dev A pointer to an ad5628_dev structure representing the device. This
 * must be a valid, non-null pointer to a device that has been
 * initialized with SPI communication parameters. The caller retains
 * ownership.
 * @param register_value A 32-bit unsigned integer representing the command and
 * data to be written to the input register. The value
 * should be constructed using the appropriate macros to
 * ensure correct formatting.
 * @return None
 ******************************************************************************/
void ad5628_set_input_register(struct ad5628_dev *dev,
			       uint32_t register_value);

#endif /* __AD5628_H__ */
