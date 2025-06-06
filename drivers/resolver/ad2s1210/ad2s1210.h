/***************************************************************************//**
 *   @file   ad2s1210.h
 *   @brief  Header file for the ad2s1210 driver
 *   @author Axel Haslam (ahaslam@baylibre.com)
********************************************************************************
 * Copyright (c) 2023 Analog Devices, Inc.
 * Copyright (c) 2023 BayLibre, SAS.
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
#ifndef AD2S1210_H_
#define AD2S1210_H_

#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"

#define AD2S1210_REG_POSITION		0x80
#define AD2S1210_REG_VELOCITY		0x82
#define AD2S1210_REG_LOS_THRD		0x88
#define AD2S1210_REG_DOS_OVR_THRD	0x89
#define AD2S1210_REG_DOS_MIS_THRD	0x8A
#define AD2S1210_REG_DOS_RST_MAX_THRD	0x8B
#define AD2S1210_REG_DOS_RST_MIN_THRD	0x8C
#define AD2S1210_REG_LOT_HIGH_THRD	0x8D
#define AD2S1210_REG_LOT_LOW_THRD	0x8E
#define AD2S1210_REG_EXCIT_FREQ		0x91
#define AD2S1210_REG_CONTROL		0x92
#define AD2S1210_CONTROL_RES_MASK	NO_OS_GENMASK(1, 0)
#define AD2S1210_CONTROL_RES0_MASK	NO_OS_BIT(0)
#define AD2S1210_CONTROL_RES1_MASK	NO_OS_BIT(1)
#define AD2S1210_ENABLE_HYSTERESIS	NO_OS_BIT(4)

#define AD2S1210_REG_SOFT_RESET		0xF0
#define AD2S1210_REG_FAULT		0xFF

#define AD2S1210_REG_MIN		AD2S1210_REG_POSITION

#define AD2S1210_MIN_CLKIN	6144000
#define AD2S1210_MAX_CLKIN	10240000
#define AD2S1210_MIN_EXCIT	2000
#define AD2S1210_MAX_EXCIT	20000
#define AD2S1210_STEP_EXCIT	250
#define AD2S1210_MIN_FCW	0x4
#define AD2S1210_MAX_FCW	0x50

#define AD2S1210_POS_MASK	NO_OS_BIT(0)
#define AD2S1210_VEL_MASK	NO_OS_BIT(1)

/***************************************************************************//**
 * @brief The `ad2s1210_mode` enumeration defines the operational modes for the
 * AD2S1210 device, which include position, velocity, and configuration
 * modes, as well as a reserved mode for potential future use. This
 * enumeration is used to specify the current mode of operation for the
 * device, allowing it to perform different functions based on the
 * selected mode.
 *
 * @param MODE_POS Represents the position mode of the AD2S1210 device.
 * @param MODE_RESERVED A reserved mode, likely for future use or internal
 * purposes.
 * @param MODE_VEL Represents the velocity mode of the AD2S1210 device.
 * @param MODE_CONFIG Represents the configuration mode of the AD2S1210 device.
 ******************************************************************************/
enum ad2s1210_mode {
	MODE_POS,
	MODE_RESERVED,
	MODE_VEL,
	MODE_CONFIG,
};

/***************************************************************************//**
 * @brief The `ad2s1210_res` enumeration defines the possible resolution
 * settings for the AD2S1210 device, which is a resolver-to-digital
 * converter. Each enumerator corresponds to a specific bit resolution
 * (10, 12, 14, or 16 bits) that can be configured for the device,
 * allowing for different levels of precision in the conversion process.
 *
 * @param AD2S1210_RES_10BIT Represents a 10-bit resolution setting for the
 * AD2S1210 device.
 * @param AD2S1210_RES_12BIT Represents a 12-bit resolution setting for the
 * AD2S1210 device.
 * @param AD2S1210_RES_14BIT Represents a 14-bit resolution setting for the
 * AD2S1210 device.
 * @param AD2S1210_RES_16BIT Represents a 16-bit resolution setting for the
 * AD2S1210 device.
 ******************************************************************************/
enum ad2s1210_res {
	AD2S1210_RES_10BIT,
	AD2S1210_RES_12BIT,
	AD2S1210_RES_14BIT,
	AD2S1210_RES_16BIT,
};

/***************************************************************************//**
 * @brief The `ad2s1210_channel` enumeration defines the available channels for
 * the AD2S1210 device, specifically indicating whether the device is
 * operating in position or velocity mode. This enumeration is used to
 * specify the type of data being processed or accessed from the device,
 * allowing for clear and concise channel selection in the device's
 * operations.
 *
 * @param AD2S1210_POS Represents the position channel of the AD2S1210 device.
 * @param AD2S1210_VEL Represents the velocity channel of the AD2S1210 device.
 ******************************************************************************/
enum ad2s1210_channel {
	AD2S1210_POS,
	AD2S1210_VEL,
};

/***************************************************************************//**
 * @brief The `ad2s1210_init_param` structure is used to encapsulate all the
 * necessary initialization parameters required to set up the AD2S1210
 * device. This includes SPI and GPIO configurations, as well as device-
 * specific settings such as resolution and clock frequency. It serves as
 * a comprehensive configuration package that is passed to the
 * initialization function to properly configure the device for
 * operation.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_a0 Defines the initialization parameters for the GPIO pin A0.
 * @param gpio_a1 Defines the initialization parameters for the GPIO pin A1.
 * @param gpio_res0 Defines the initialization parameters for the GPIO pin RES0.
 * @param gpio_res1 Defines the initialization parameters for the GPIO pin RES1.
 * @param gpio_sample Defines the initialization parameters for the GPIO pin
 * SAMPLE.
 * @param resolution Specifies the resolution setting for the device,
 * represented as an 8-bit integer.
 * @param clkin_hz Specifies the clock input frequency in hertz.
 ******************************************************************************/
struct ad2s1210_init_param {
	struct no_os_spi_init_param spi_init;
	struct no_os_gpio_init_param gpio_a0;
	struct no_os_gpio_init_param gpio_a1;
	struct no_os_gpio_init_param gpio_res0;
	struct no_os_gpio_init_param gpio_res1;
	struct no_os_gpio_init_param gpio_sample;
	int8_t resolution;
	uint32_t clkin_hz;
};

/***************************************************************************//**
 * @brief The `ad2s1210_dev` structure is a comprehensive representation of the
 * AD2S1210 device, encapsulating its configuration and operational
 * parameters. It includes pointers to SPI and GPIO descriptors for
 * interfacing with the device, as well as fields for specifying the
 * device's mode and resolution. The structure also holds information
 * about the presence of mode and resolution pins, and the clock input
 * frequency, making it essential for managing the device's
 * initialization and operation in embedded systems.
 *
 * @param name A constant character pointer to the name of the device.
 * @param have_mode_pins A boolean indicating if the device has mode pins.
 * @param have_resolution_pins A boolean indicating if the device has resolution
 * pins.
 * @param mode An enumeration representing the mode of the device.
 * @param resolution An enumeration representing the resolution of the device.
 * @param spi_desc A pointer to a SPI descriptor structure for SPI
 * communication.
 * @param gpio_a1 A pointer to a GPIO descriptor for the A1 pin.
 * @param gpio_a0 A pointer to a GPIO descriptor for the A0 pin.
 * @param gpio_res0 A pointer to a GPIO descriptor for the RES0 pin.
 * @param gpio_res1 A pointer to a GPIO descriptor for the RES1 pin.
 * @param gpio_sample A pointer to a GPIO descriptor for the SAMPLE pin.
 * @param clkin_hz An unsigned 32-bit integer representing the clock input
 * frequency in hertz.
 ******************************************************************************/
struct ad2s1210_dev {
	const char *name;
	bool have_mode_pins;
	bool have_resolution_pins;
	enum ad2s1210_mode mode;
	enum ad2s1210_res resolution;
	struct no_os_spi_desc *spi_desc;
	struct no_os_gpio_desc *gpio_a1;
	struct no_os_gpio_desc *gpio_a0;
	struct no_os_gpio_desc *gpio_res0;
	struct no_os_gpio_desc *gpio_res1;
	struct no_os_gpio_desc *gpio_sample;
	uint32_t clkin_hz;
};

/***************************************************************************//**
 * @brief This function sets up an AD2S1210 device using the provided
 * initialization parameters, configuring the necessary GPIOs and SPI
 * interface. It must be called before any other operations on the
 * device. The function checks if the clock frequency is within the valid
 * range and allocates memory for the device structure. If any step
 * fails, it cleans up and returns an error code. Successful
 * initialization results in a pointer to the device structure being
 * returned via the provided pointer.
 *
 * @param dev A pointer to a pointer where the initialized device structure will
 * be stored. Must not be null. The caller takes ownership of the
 * allocated structure upon successful initialization.
 * @param init_param A pointer to a structure containing initialization
 * parameters, including SPI and GPIO configurations,
 * resolution, and clock frequency. Must not be null. The
 * clock frequency must be within the range defined by
 * AD2S1210_MIN_CLKIN and AD2S1210_MAX_CLKIN.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the device structure is allocated and initialized, and the
 * pointer to it is stored in the provided dev parameter.
 ******************************************************************************/
int ad2s1210_init(struct ad2s1210_dev **dev,
		  struct ad2s1210_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly release all resources and deallocate
 * memory associated with an AD2S1210 device when it is no longer needed.
 * This function should be called to prevent resource leaks after the
 * device has been initialized and used. It ensures that all associated
 * SPI and GPIO resources are removed and the memory allocated for the
 * device structure is freed. The function must be called with a valid
 * device pointer, and it will return an error code if the device pointer
 * is null or if any resource removal fails.
 *
 * @param dev A pointer to an ad2s1210_dev structure representing the device to
 * be removed. Must not be null. The function will return -EINVAL if
 * this parameter is null. The caller retains ownership of the
 * pointer, but the memory it points to will be freed by this
 * function.
 * @return Returns 0 on success, or a negative error code if any resource
 * removal fails or if the input parameter is invalid.
 ******************************************************************************/
int ad2s1210_remove(struct ad2s1210_dev *dev);
/***************************************************************************//**
 * @brief Use this function to write a value to a specific register of the
 * AD2S1210 device. It is essential to ensure that the device is properly
 * initialized before calling this function. The function requires the
 * device to be in configuration mode to perform the write operation. If
 * the provided register address is below the minimum allowed value, the
 * function will return an error. This function is useful for configuring
 * device settings or controlling its operation by writing to its
 * registers.
 *
 * @param dev A pointer to an initialized ad2s1210_dev structure representing
 * the device. Must not be null.
 * @param addr The address of the register to write to. Must be greater than or
 * equal to AD2S1210_REG_MIN. If less, the function returns an
 * error.
 * @param val The value to write to the specified register. This is an 8-bit
 * value.
 * @return Returns 0 on success or a negative error code on failure, such as if
 * the address is invalid or if there is a communication error.
 ******************************************************************************/
int ad2s1210_reg_write(struct ad2s1210_dev *dev, uint8_t addr,
		       uint8_t val);
/***************************************************************************//**
 * @brief Use this function to read a value from a specified register of the
 * AD2S1210 device. It is essential to ensure that the address provided
 * is valid and within the acceptable range, as defined by the device's
 * register map. The function must be called with a properly initialized
 * device structure. It will configure the device to the appropriate mode
 * for reading and then perform the read operation. If the address is
 * invalid, the function will return an error code. The function also
 * handles potential communication errors and returns corresponding error
 * codes.
 *
 * @param dev A pointer to an initialized ad2s1210_dev structure representing
 * the device. Must not be null.
 * @param addr The register address to read from. Must be greater than or equal
 * to AD2S1210_REG_MIN. Invalid addresses will result in an error.
 * @param val A pointer to a uint8_t where the read value will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * invalid address or communication errors.
 ******************************************************************************/
int ad2s1210_reg_read(struct ad2s1210_dev *dev, uint8_t addr,
		      uint8_t *val);
/***************************************************************************//**
 * @brief This function is used to perform a single SPI conversion on the
 * AD2S1210 device to read position and/or velocity data based on the
 * specified active mask. It should be called when a single conversion is
 * needed, and the device is properly initialized. The function requires
 * a buffer to store the conversion results, and the size of this buffer
 * must be sufficient to hold the requested data. If the buffer size is
 * insufficient or the active mask is invalid, the function will return
 * an error. The function manipulates GPIO pins to trigger the conversion
 * and reads the data into the provided buffer.
 *
 * @param dev A pointer to an initialized ad2s1210_dev structure representing
 * the device. Must not be null.
 * @param active_mask A bitmask indicating which data to read: position,
 * velocity, or both. Must be a valid combination of
 * AD2S1210_POS_MASK and AD2S1210_VEL_MASK.
 * @param data A pointer to a buffer where the conversion results will be
 * stored. Must not be null and must be large enough to hold the
 * requested data.
 * @param size The size of the data buffer in bytes. Must be at least 2 bytes,
 * and at least 4 bytes if both position and velocity data are
 * requested.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int ad2s1210_spi_single_conversion(struct ad2s1210_dev *dev,
				   uint32_t active_mask,
				   void *data, uint32_t size);
/***************************************************************************//**
 * @brief This function determines whether the hysteresis feature is currently
 * enabled on a specified AD2S1210 device. It should be called when you
 * need to verify the status of the hysteresis setting, which can affect
 * the device's response to input signal changes. The function requires a
 * valid device structure that has been properly initialized. It returns
 * a positive value if hysteresis is enabled, zero if it is not, or a
 * negative error code if the operation fails.
 *
 * @param dev A pointer to an initialized ad2s1210_dev structure representing
 * the device. Must not be null. The function will return an error if
 * the device is not properly initialized or if there is a
 * communication failure.
 * @return Returns 1 if hysteresis is enabled, 0 if it is not, or a negative
 * error code if an error occurs during the operation.
 ******************************************************************************/
int ad2s1210_hysteresis_is_enabled(struct ad2s1210_dev *dev);
/***************************************************************************//**
 * @brief This function is used to control the hysteresis feature of the
 * AD2S1210 device. It should be called when there is a need to enable or
 * disable hysteresis, which can affect the device's response to input
 * signals. The function must be called with a valid device structure
 * that has been properly initialized. It reads the current control
 * register, modifies the hysteresis bit according to the 'enable'
 * parameter, and writes the updated value back to the control register.
 * The function returns an error code if the read or write operation
 * fails.
 *
 * @param dev A pointer to an initialized 'ad2s1210_dev' structure representing
 * the device. Must not be null.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) hysteresis.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad2s1210_set_hysteresis(struct ad2s1210_dev *dev, bool enable);
/***************************************************************************//**
 * @brief This function sets a new excitation frequency for the AD2S1210 device
 * by calculating the frequency control word (FCW) based on the provided
 * frequency and the device's clock input frequency. It should be used
 * when there is a need to change the excitation frequency after the
 * device has been initialized. The function performs a software reset to
 * apply the new frequency, which does not affect other configuration
 * registers. It is important to ensure that the provided frequency is
 * within the valid range to avoid errors.
 *
 * @param dev A pointer to an initialized ad2s1210_dev structure representing
 * the device. Must not be null.
 * @param fexcit The desired excitation frequency in Hertz. Must be within the
 * valid range defined by the device specifications. If the
 * frequency is out of range, the function returns an error.
 * @return Returns 0 on success. If the frequency is out of range, returns
 * -ERANGE. If there is an error writing to the device registers,
 * returns a negative error code.
 ******************************************************************************/
int ad2s1210_reinit_excitation_frequency(struct ad2s1210_dev *dev,
		uint16_t fexcit);
/***************************************************************************//**
 * @brief Use this function to obtain the current excitation frequency
 * configured in the AD2S1210 device. This function should be called when
 * you need to verify or log the excitation frequency setting. It
 * requires a valid device structure and a pointer to store the frequency
 * value. Ensure that the device has been properly initialized before
 * calling this function. The function will return an error code if the
 * read operation fails.
 *
 * @param dev A pointer to an initialized 'ad2s1210_dev' structure representing
 * the device. Must not be null.
 * @param fexcit A pointer to a uint16_t variable where the excitation frequency
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails. On success, the excitation frequency is stored in the location
 * pointed to by 'fexcit'.
 ******************************************************************************/
int ad2s1210_get_excitation_frequency(struct ad2s1210_dev *dev,
				      uint16_t *fexcit);
#endif
