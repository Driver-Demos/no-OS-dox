/***************************************************************************//**
 *   @file   AD5791.h
 *   @brief  Header file of AD5791 Driver.
 *   @author DNechita (Dan.Nechita@analog.com)
********************************************************************************
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
 *
*******************************************************************************/
#ifndef __AD5791_H__
#define __AD5791_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"
#include "no_os_error.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/* Supported devices */
/***************************************************************************//**
 * @brief The `ad5791_type` enumeration defines a set of constants representing
 * different types of AD57xx series devices, specifically the AD5760,
 * AD5780, AD5781, AD5790, and AD5791. This enumeration is used to
 * identify and differentiate between these supported devices within the
 * AD5791 driver, allowing the software to handle device-specific
 * configurations and operations.
 *
 * @param ID_AD5760 Represents the AD5760 device type.
 * @param ID_AD5780 Represents the AD5780 device type.
 * @param ID_AD5781 Represents the AD5781 device type.
 * @param ID_AD5790 Represents the AD5790 device type.
 * @param ID_AD5791 Represents the AD5791 device type.
 ******************************************************************************/
enum ad5791_type {
	ID_AD5760,
	ID_AD5780,
	ID_AD5781,
	ID_AD5790,
	ID_AD5791
};

/* Possible voltage spans for linearity error compensation */
/***************************************************************************//**
 * @brief The `ad5791_lin_comp_select` enumeration defines possible voltage
 * spans for linearity error compensation in the AD5791 device. Each
 * enumerator corresponds to a specific voltage range that the device can
 * handle, allowing for precise control and compensation of linearity
 * errors based on the selected voltage span. This is crucial for
 * applications requiring high precision and accuracy in voltage output.
 *
 * @param AD5781_SPAN_UPTO_10V Represents a voltage span up to 10V.
 * @param AD5781_SPAN_10V_TO_20V Represents a voltage span from 10V to 20V.
 * @param AD5791_SPAN_10V_TO_12V Represents a voltage span from 10V to 12V.
 * @param AD5791_SPAN_12V_TO_16V Represents a voltage span from 12V to 16V.
 * @param AD5791_SPAN_16V_TO_19V Represents a voltage span from 16V to 19V.
 * @param AD5791_SPAN_19V_TO_20V Represents a voltage span from 19V to 20V.
 ******************************************************************************/
enum ad5791_lin_comp_select {
	AD5781_SPAN_UPTO_10V = 0,
	AD5781_SPAN_10V_TO_20V = 4,
	AD5791_SPAN_10V_TO_12V = 1,
	AD5791_SPAN_12V_TO_16V = 2,
	AD5791_SPAN_16V_TO_19V = 3,
	AD5791_SPAN_19V_TO_20V = 4
};

/***************************************************************************//**
 * @brief The `ad5791_chip_info` structure is a simple data structure used to
 * store information about the AD5791 chip, specifically its resolution.
 * This structure is likely used in the context of configuring or
 * interfacing with the AD5791 digital-to-analog converter (DAC) to
 * ensure that operations are performed with the correct bit resolution.
 *
 * @param resolution Specifies the resolution of the AD5791 chip in bits.
 ******************************************************************************/
struct ad5791_chip_info {
	uint32_t resolution;
};

/***************************************************************************//**
 * @brief The `ad5791_dev` structure is designed to encapsulate the necessary
 * components for interfacing with an AD5791 digital-to-analog converter
 * (DAC) device. It includes SPI and GPIO descriptors for managing
 * communication and control signals, as well as device-specific settings
 * such as the active device type and output buffer configuration. This
 * structure is essential for initializing and controlling the AD5791 DAC
 * in embedded systems.
 *
 * @param spi_desc Pointer to an SPI descriptor for SPI communication.
 * @param gpio_reset Pointer to a GPIO descriptor for the reset pin.
 * @param gpio_clr Pointer to a GPIO descriptor for the clear pin.
 * @param gpio_ldac Pointer to a GPIO descriptor for the LDAC pin.
 * @param act_device Enumeration indicating the active device type.
 * @param rbuf_gain2 Boolean indicating if the output buffer gain is set to 2.
 ******************************************************************************/
struct ad5791_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_reset;
	struct no_os_gpio_desc	*gpio_clr;
	struct no_os_gpio_desc	*gpio_ldac;
	/* Device Settings */
	enum ad5791_type act_device;
	bool rbuf_gain2;
};

/***************************************************************************//**
 * @brief The `ad5791_init_param` structure is used to encapsulate all the
 * necessary initialization parameters required to set up the AD5791
 * device. It includes SPI and GPIO initialization parameters, as well as
 * device-specific settings such as the active device type and output
 * buffer gain configuration. This structure is essential for configuring
 * the AD5791 device before it can be used for digital-to-analog
 * conversion tasks.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_reset Contains the initialization parameters for the GPIO used
 * for the reset function.
 * @param gpio_clr Contains the initialization parameters for the GPIO used for
 * the clear function.
 * @param gpio_ldac Contains the initialization parameters for the GPIO used for
 * the load DAC function.
 * @param act_device Specifies the active device type from the supported AD5791
 * series.
 * @param rbuf_gain2 Indicates whether the output buffer gain is set to 2.
 ******************************************************************************/
struct ad5791_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param		gpio_reset;
	struct no_os_gpio_init_param		gpio_clr;
	struct no_os_gpio_init_param		gpio_ldac;
	/* Device Settings */
	enum ad5791_type act_device;
	bool					rbuf_gain2;
};

/******************************************************************************/
/*********************************** GPIO *************************************/
/******************************************************************************/
#define AD5791_RESET_OUT       no_os_gpio_direction_output(dev->gpio_reset, \
			       NO_OS_GPIO_HIGH);
#define AD5791_RESET_LOW       no_os_gpio_set_value(dev->gpio_reset,        \
			       NO_OS_GPIO_LOW)
#define AD5791_RESET_HIGH      no_os_gpio_set_value(dev->gpio_reset,        \
			       NO_OS_GPIO_HIGH)

#define AD5791_CLR_OUT         no_os_gpio_direction_output(dev->gpio_clr,  \
			       NO_OS_GPIO_HIGH);
#define AD5791_CLR_LOW         no_os_gpio_set_value(dev->gpio_clr,         \
			       NO_OS_GPIO_LOW)
#define AD5791_CLR_HIGH        no_os_gpio_set_value(dev->gpio_clr,         \
			       NO_OS_GPIO_HIGH)

#define AD5791_LDAC_OUT        no_os_gpio_direction_output(dev->gpio_ldac, \
			       NO_OS_GPIO_HIGH);
#define AD5791_LDAC_LOW        no_os_gpio_set_value(dev->gpio_ldac,        \
			       NO_OS_GPIO_LOW)
#define AD5791_LDAC_HIGH       no_os_gpio_set_value(dev->gpio_ldac,        \
			       NO_OS_GPIO_HIGH)

/******************************************************************************/
/********************************** AD5791 ************************************/
/******************************************************************************/

/* Maximum resolution */
#define MAX_RESOLUTION          20

/* Register Map */
#define AD5791_NOP                 0  // No operation (NOP).
#define AD5791_REG_DAC             1  // DAC register.
#define AD5791_REG_CTRL            2  // Control register.
#define AD5791_REG_CLR_CODE        3  // Clearcode register.
#define AD5791_CMD_WR_SOFT_CTRL    4  // Software control register(Write only).

/* Input Shift Register bit definition. */
#define AD5791_READ                (1ul << 23)
#define AD5791_WRITE               (0ul << 23)
#define AD5791_ADDR_REG(x)         (((uint32_t)(x) & 0x7) << 20)

/* Control Register bit Definition */
#define AD5791_CTRL_LINCOMP_MASK   NO_OS_GENMASK(9,6) // Linearity error compensation.
#define AD5791_CTRL_LINCOMP(x)     (((x) & 0xF) << 6)
#define AD5791_CTRL_SDODIS_MASK    NO_OS_BIT(5)       // SDO pin enable/disable control.
#define AD5791_CTRL_SDODIS(x)      ((x & 0x01) << 5)
#define AD5791_CTRL_BIN2SC_MASK    NO_OS_BIT(4)       // DAC register coding selection.
#define AD5791_CTRL_BIN2SC(x)      ((x & 0x01) << 4)
#define AD5791_CTRL_DACTRI         NO_OS_BIT(3)       // DAC tristate control.
#define AD5791_CTRL_OPGND          NO_OS_BIT(2)       // Output ground clamp control.
#define AD5791_CTRL_RBUF_MASK      NO_OS_BIT(1)       // Output amplifier configuration control.
#define AD5791_CTRL_RBUF(x)        ((x & 0x01) << 1)

/* Software Control Register bit definition */
#define AD5791_SOFT_CTRL_RESET      (1 << 2) // RESET function.
#define AD5791_SOFT_CTRL_CLR        (1 << 1) // CLR function.
#define AD5791_SOFT_CTRL_LDAC       (1 << 0) // LDAC function.

/* DAC OUTPUT STATES */
#define AD5791_OUT_NORMAL            0x0
#define AD5791_OUT_CLAMPED_6K        0x1
#define AD5791_OUT_TRISTATE          0x2

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the communication with an AD5791 device by
 * initializing the necessary SPI and GPIO interfaces based on the
 * provided initialization parameters. It must be called before any other
 * operations on the device to ensure that the device is properly
 * configured and ready for communication. The function allocates memory
 * for the device structure and configures the GPIO pins for reset,
 * clear, and load DAC operations. It returns a status code indicating
 * success or failure of the initialization process.
 *
 * @param device A pointer to a pointer of type `struct ad5791_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct ad5791_init_param` containing
 * initialization parameters for the device, including SPI and
 * GPIO configurations. The structure must be properly
 * populated before calling the function.
 * @return Returns an `int32_t` status code: 0 for success, or a negative error
 * code if initialization fails. The `device` pointer is updated to
 * point to the initialized device structure on success.
 ******************************************************************************/
int32_t ad5791_init(struct ad5791_dev **device,
		    struct ad5791_init_param init_param);

/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD5791
 * device instance when it is no longer needed. This function should be
 * called to clean up after a successful call to `ad5791_init`. It
 * ensures that all allocated memory and hardware resources, such as SPI
 * and GPIO descriptors, are properly freed. Failure to call this
 * function may result in resource leaks.
 *
 * @param dev A pointer to an `ad5791_dev` structure representing the device
 * instance to be removed. Must not be null. The function will handle
 * null pointers gracefully by not attempting to free resources.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a non-zero value indicates an error occurred during resource
 * deallocation.
 ******************************************************************************/
int32_t ad5791_remove(struct ad5791_dev *dev);

/***************************************************************************//**
 * @brief Use this function to write a 20-bit value to a specific register of
 * the AD5791 device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function
 * communicates with the device over SPI and expects a valid register
 * address and a value to write. If the SPI communication fails, the
 * function returns an error code. This function is typically used when
 * configuring the device or updating its settings.
 *
 * @param dev A pointer to an initialized ad5791_dev structure representing the
 * device. Must not be null.
 * @param register_address The address of the register to write to. Must be a
 * valid register address for the AD5791.
 * @param register_value The 20-bit value to write to the specified register.
 * The value is masked to ensure it fits within 20 bits.
 * @return Returns 0 on success, or -1 if the SPI communication fails.
 ******************************************************************************/
int32_t ad5791_set_register_value(struct ad5791_dev *dev,
				  uint8_t register_address,
				  uint32_t register_value);

/***************************************************************************//**
 * @brief Use this function to retrieve the current value stored in a specific
 * register of the AD5791 device. It is essential to ensure that the
 * device has been properly initialized before calling this function. The
 * function communicates with the device over SPI to read the register
 * value. If the operation is successful, the value is stored in the
 * provided memory location. In case of an error during communication,
 * the function returns a negative status code.
 *
 * @param dev A pointer to an initialized ad5791_dev structure representing the
 * device. Must not be null.
 * @param register_address The address of the register to be read. It should be
 * a valid register address for the AD5791 device.
 * @param value A pointer to a uint32_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int32_t ad5791_get_register_value(struct ad5791_dev *dev,
				  uint8_t register_address,
				  uint32_t *value);

/***************************************************************************//**
 * @brief This function configures the DAC output state of the AD5791 device to
 * one of the predefined modes: normal operation, clamped to 6k ohms, or
 * tristate. It should be called when there is a need to change the
 * output state of the DAC, such as during initialization or when
 * altering the device's operational mode. The function requires a valid
 * device structure and a state value that specifies the desired output
 * mode. It returns a status code indicating success or failure of the
 * operation.
 *
 * @param dev A pointer to an initialized ad5791_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param state A uint8_t value representing the desired DAC output state. Valid
 * values are 0 (normal), 1 (clamped to 6k ohms), and 2 (tristate).
 * Invalid values may result in undefined behavior.
 * @return Returns an int32_t status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t ad5791_dac_ouput_state(struct ad5791_dev *dev,
			       uint8_t state);

/***************************************************************************//**
 * @brief This function is used to set the digital-to-analog converter (DAC)
 * output value for a specified AD5791 device. It should be called when
 * you need to update the DAC output to a new value. The function
 * requires a valid device structure that has been initialized and
 * configured. The input value is adjusted according to the device's
 * resolution before being written to the DAC register. Ensure that the
 * device is properly initialized before calling this function to avoid
 * undefined behavior.
 *
 * @param dev A pointer to an initialized ad5791_dev structure representing the
 * target device. Must not be null, and the device must be properly
 * configured before use.
 * @param value A 32-bit unsigned integer representing the desired DAC value.
 * The value is adjusted based on the device's resolution before
 * being written. Ensure the value is within the valid range for
 * the device's resolution to avoid unexpected results.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad5791_set_dac_value(struct ad5791_dev *dev,
			     uint32_t value);

/***************************************************************************//**
 * @brief Use this function to send a software instruction to the AD5791 device,
 * such as asserting RESET, CLR, or LDAC. This function should be called
 * when a specific software-controlled action is required on the device.
 * It is important to ensure that the device has been properly
 * initialized before calling this function. The function will wait
 * briefly to allow the instruction to take effect, and it returns a
 * status code indicating success or failure.
 *
 * @param dev A pointer to an initialized ad5791_dev structure representing the
 * device. Must not be null.
 * @param instruction_bit A uint8_t value representing the specific instruction
 * to assert. Valid values are defined by the
 * AD5791_SOFT_CTRL_* macros.
 * @return Returns an int32_t status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t ad5791_soft_instruction(struct ad5791_dev *dev,
				uint8_t instruction_bit);

/***************************************************************************//**
 * @brief Use this function to configure the AD5791 device's output amplifier,
 * DAC coding, SDO state, and linearity error compensation settings. It
 * should be called after initializing the device with `ad5791_init`. The
 * function reads the current control register settings, modifies them
 * according to the provided setup word, and writes the new settings back
 * to the device. This allows for updating specific configuration bits
 * while preserving others. Ensure that the `dev` parameter is a valid,
 * initialized device structure.
 *
 * @param dev A pointer to an `ad5791_dev` structure representing the device.
 * Must not be null and should be initialized with `ad5791_init`
 * before calling this function. The caller retains ownership.
 * @param setup_word A 32-bit word containing the new configuration settings for
 * the device. The specific bits in this word determine the
 * new state of the device's control register.
 * @return Returns an integer status code. A non-negative value indicates
 * success, while a negative value indicates an error occurred during
 * the operation.
 ******************************************************************************/
int32_t ad5791_setup(struct ad5791_dev *dev,
		     uint32_t setup_word);

/***************************************************************************//**
 * @brief Use this function to modify specific bits of a register in the AD5791
 * device by applying a mask and a new value. This function is useful
 * when only certain bits of a register need to be updated without
 * affecting the other bits. It requires a valid device structure and a
 * register address to operate. The function first reads the current
 * value of the register, applies the mask and the new value, and then
 * writes the modified value back to the register. It must be called with
 * a properly initialized device structure, and the register address must
 * be valid for the AD5791 device. If the device structure is null, the
 * function returns an error.
 *
 * @param dev A pointer to an initialized ad5791_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param register_address The address of the register to be modified. Must be a
 * valid register address for the AD5791 device.
 * @param mask A 32-bit mask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected by the new
 * value.
 * @param value A 32-bit value containing the new bits to be written to the
 * register. Only the bits corresponding to the mask will be
 * written.
 * @return Returns 0 on success or a negative error code if the operation fails,
 * such as if the device structure is null or the register read/write
 * fails.
 ******************************************************************************/
int ad5791_spi_write_mask(struct ad5791_dev *dev,
			  uint8_t register_address,
			  uint32_t mask,
			  uint32_t value);

/***************************************************************************//**
 * @brief This function configures the linearity error compensation for an
 * AD5791 device based on the specified voltage span. It should be called
 * after the device has been initialized and is used to optimize the
 * device's performance for a given voltage range. The function checks
 * the compatibility of the voltage span with the active device type and
 * returns an error if the span is invalid for the device. It is
 * important to ensure that the device pointer is valid before calling
 * this function.
 *
 * @param dev A pointer to an initialized ad5791_dev structure representing the
 * device. Must not be null. The function returns -EINVAL if this
 * parameter is null.
 * @param v_span An enum value of type ad5791_lin_comp_select representing the
 * desired voltage span for linearity compensation. The value must
 * be compatible with the active device type, otherwise, the
 * function returns -EINVAL.
 * @return Returns 0 on success or -EINVAL if the device pointer is null or if
 * the voltage span is invalid for the active device type.
 ******************************************************************************/
int ad5791_set_lin_comp(struct ad5791_dev *dev,
			enum ad5791_lin_comp_select v_span);

/***************************************************************************//**
 * @brief This function triggers the Load DAC (LDAC) operation on the AD5791
 * device, which updates the DAC output with the value from its input
 * register. It should be called when a new DAC value needs to be
 * applied. The function first checks if a valid device structure is
 * provided. If a GPIO pin is configured for LDAC, it toggles the pin to
 * trigger the operation. If no GPIO is assigned, it uses a software
 * instruction to achieve the same effect. This function must be called
 * with a properly initialized device structure.
 *
 * @param dev A pointer to an ad5791_dev structure representing the device. Must
 * not be null. If null, the function returns an error code.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the input device is null.
 ******************************************************************************/
int ad5791_ldac_trigger(struct ad5791_dev *dev);

/***************************************************************************//**
 * @brief This function is used to clear the DAC channel output of an AD5791
 * device asynchronously. It should be called when the DAC output needs
 * to be reset to a predefined clear code. The function first checks if a
 * GPIO pin is assigned for the clear operation; if so, it uses this pin
 * to perform the clear operation with a necessary delay. If no GPIO pin
 * is assigned, it falls back to a software clear instruction. The
 * function must be called with a valid device structure that has been
 * properly initialized.
 *
 * @param dev A pointer to an ad5791_dev structure representing the device. This
 * must not be null. The structure should be initialized and
 * configured before calling this function. If the pointer is null,
 * the function returns an error code.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the input is invalid.
 ******************************************************************************/
int ad5791_clear_async(struct ad5791_dev *dev);

#endif /* __AD5791_H__ */
