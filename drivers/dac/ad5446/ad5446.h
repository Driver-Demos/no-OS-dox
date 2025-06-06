/***************************************************************************//**
*   @file   AD5446.h
*   @brief  Header file of AD5446 Driver. This driver supporting the following
*              devices: AD5553, AD5543, AD5542A, AD5541A, AD5512A, AD5453,
*                       AD5452, AD5451, AD5450, AD5446, AD5444
*
*   @author Istvan Csomortani (istvan.csomortani@analog.com)
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
*******************************************************************************/

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* Custom type for active clock edge */
/***************************************************************************//**
 * @brief The `active_clk_t` is an enumeration that defines the active clock
 * edge types, specifically the negative edge (`negedge`) and the
 * positive edge (`posedge`). This enumeration is used to specify which
 * edge of the clock signal is active, which is crucial for timing and
 * synchronization in digital circuits and systems.
 *
 * @param negedge Represents the negative edge of a clock signal.
 * @param posedge Represents the positive edge of a clock signal.
 ******************************************************************************/
enum active_clk_t {
	negedge,
	posedge
};

/* Custom boolean type */
/***************************************************************************//**
 * @brief The `bool_t` data structure is an enumeration that defines a custom
 * boolean type with two possible values: `false` and `true`. This enum
 * is used to represent boolean logic in a more readable and type-safe
 * manner within the C programming language, particularly in the context
 * of the AD5446 driver code.
 *
 * @param false Represents the boolean value false.
 * @param true Represents the boolean value true.
 ******************************************************************************/
enum bool_t {
	false,
	true
};

/* Data structure for chip's attributes */
/***************************************************************************//**
 * @brief The `ad5446_chip_info` structure is used to encapsulate the attributes
 * of an AD5446 series chip, including its resolution, the active clock
 * edge for data input, and whether it has control features. This
 * structure is essential for configuring and managing the chip's
 * operational parameters in applications that utilize the AD5446 driver.
 *
 * @param resolution Specifies the resolution of the chip in bits.
 * @param data_clock_in Indicates the active clock edge for data input, using
 * the `active_clk_t` enum.
 * @param has_ctrl A boolean flag indicating if the chip has control features,
 * using the `bool_t` enum.
 ******************************************************************************/
struct ad5446_chip_info {
	uint8_t resolution;
	enum active_clk_t  data_clock_in;
	enum bool_t        has_ctrl;
};

/* Supported output types */
/***************************************************************************//**
 * @brief The `vout_type_t` enumeration defines the types of output voltage
 * ranges supported by the AD5446 series of devices. It includes three
 * possible values: `unipolar`, which allows for a voltage range from 0
 * to a positive reference voltage (Vref); `unipolar_inv`, which allows
 * for a range from 0 to a negative reference voltage (-Vref); and
 * `bipolar`, which allows for a symmetric range from -Vref to Vref. This
 * enumeration is used to configure the output voltage behavior of the
 * device.
 *
 * @param unipolar Represents an output voltage range from 0 to Vref.
 * @param unipolar_inv Represents an output voltage range from 0 to -Vref.
 * @param bipolar Represents an output voltage range from -Vref to Vref.
 ******************************************************************************/
enum vout_type_t {
	unipolar,       /* 0 .. Vref */
	unipolar_inv,   /* 0 .. -Vref */
	bipolar         /* -Vref .. Vref*/
};

/* Supported devices */
/***************************************************************************//**
 * @brief The `ad5446_type_t` is an enumeration that defines a set of constants
 * representing different device types supported by the AD5446 driver.
 * Each enumerator corresponds to a specific device model, allowing the
 * software to identify and handle various devices within the AD5446
 * family. This enumeration is crucial for configuring and managing
 * device-specific operations in the driver.
 *
 * @param ID_AD5600 Represents the AD5600 device type.
 * @param ID_AD5553 Represents the AD5553 device type.
 * @param ID_AD5543 Represents the AD5543 device type.
 * @param ID_AD5542A Represents the AD5542A device type.
 * @param ID_AD5541A Represents the AD5541A device type.
 * @param ID_AD5512A Represents the AD5512A device type.
 * @param ID_AD5453 Represents the AD5453 device type.
 * @param ID_AD5452 Represents the AD5452 device type.
 * @param ID_AD5451 Represents the AD5451 device type.
 * @param ID_AD5450 Represents the AD5450 device type.
 * @param ID_AD5446 Represents the AD5446 device type.
 * @param ID_AD5444 Represents the AD5444 device type.
 ******************************************************************************/
enum ad5446_type_t {
	ID_AD5600,
	ID_AD5553,
	ID_AD5543,
	ID_AD5542A,
	ID_AD5541A,
	ID_AD5512A,
	ID_AD5453,
	ID_AD5452,
	ID_AD5451,
	ID_AD5450,
	ID_AD5446,
	ID_AD5444,
};

/***************************************************************************//**
 * @brief The `ad5446_dev` structure is a compound data type used to represent a
 * device instance of the AD5446 series digital-to-analog converters
 * (DACs). It encapsulates the necessary SPI and GPIO descriptors
 * required for communication and control of the DAC, as well as an
 * enumeration to specify the particular device model being used. This
 * structure is essential for managing the hardware interface and
 * settings of the AD5446 devices within the driver.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_ladc Pointer to a GPIO descriptor for the LADC pin.
 * @param gpio_clrout Pointer to a GPIO descriptor for the CLR pin.
 * @param act_device Enumeration indicating the active device type from the
 * supported AD5446 series.
 ******************************************************************************/
struct ad5446_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_ladc;
	struct no_os_gpio_desc	*gpio_clrout;
	/* Device Settings */
	enum ad5446_type_t act_device;
};

/***************************************************************************//**
 * @brief The `ad5446_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up an AD5446 device. It
 * includes SPI initialization parameters, GPIO settings for LADC and CLR
 * output controls, and the specific device type being used. This
 * structure is essential for configuring the device's communication and
 * control interfaces before operation.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_ladc Contains the initialization parameters for the GPIO used for
 * LADC control.
 * @param gpio_clrout Contains the initialization parameters for the GPIO used
 * for CLR output control.
 * @param act_device Specifies the active device type from the supported AD5446
 * series.
 ******************************************************************************/
struct ad5446_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_ladc;
	struct no_os_gpio_init_param	gpio_clrout;
	/* Device Settings */
	enum ad5446_type_t act_device;
};

/* Control Bits */
#define AD5446_CTRL_LOAD_UPDATE     0x0
#define AD5446_CTRL_ACTIVE_POSEDGE  0x3

/* AD5446 GPIO */
#define AD5446_LDAC_OUT             no_os_gpio_direction_output(dev->gpio_ladc,   \
			            NO_OS_GPIO_HIGH)
#define AD5446_LDAC_LOW             no_os_gpio_set_value(dev->gpio_ladc,          \
			            NO_OS_GPIO_LOW)
#define AD5446_LDAC_HIGH            no_os_gpio_set_value(dev->gpio_ladc,          \
			            NO_OS_GPIO_HIGH)

#define AD5446_CLR_OUT              no_os_gpio_direction_output(dev->gpio_clrout, \
			            NO_OS_GPIO_HIGH)
#define AD5446_CLR_LOW              no_os_gpio_set_value(dev->gpio_clrout,        \
			            NO_OS_GPIO_LOW)
#define AD5446_CLR_HIGH             no_os_gpio_set_value(dev->gpio_clrout,        \
			            NO_OS_GPIO_HIGH)

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize SPI and Initial Values for AD5446 Board. */
/***************************************************************************//**
 * @brief This function sets up the AD5446 device by allocating necessary
 * resources and initializing the SPI and GPIO interfaces based on the
 * provided initialization parameters. It must be called before any other
 * operations on the AD5446 device to ensure that the device is properly
 * configured and ready for use. The function handles different device
 * types by configuring specific GPIO pins accordingly. If the
 * initialization fails at any step, the function returns an error code,
 * and the device pointer is not valid.
 *
 * @param device A pointer to a pointer of type `struct ad5446_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * The caller is responsible for freeing the allocated memory
 * using `ad5446_remove()`.
 * @param init_param A structure of type `struct ad5446_init_param` containing
 * initialization parameters for the SPI and GPIO interfaces,
 * as well as the specific AD5446 device type. All fields must
 * be properly initialized before calling this function.
 * @return Returns 0 on success, or a negative error code if initialization
 * fails. On success, the `device` pointer is set to point to a newly
 * allocated and initialized `ad5446_dev` structure.
 ******************************************************************************/
int8_t ad5446_init(struct ad5446_dev **device,
		   struct ad5446_init_param init_param);

/* Free the resources allocated by ad5446_init(). */
/***************************************************************************//**
 * @brief This function is used to release all resources associated with an
 * AD5446 device that were previously allocated during initialization. It
 * should be called when the device is no longer needed to ensure proper
 * cleanup and to prevent resource leaks. The function handles the
 * deallocation of SPI and GPIO descriptors and frees the memory
 * allocated for the device structure. It is important to ensure that the
 * device pointer is valid and was successfully initialized before
 * calling this function.
 *
 * @param dev A pointer to an ad5446_dev structure representing the device to be
 * removed. This pointer must not be null and should point to a
 * valid, initialized device structure. If the pointer is invalid,
 * the behavior is undefined.
 * @return Returns an int32_t value indicating the success or failure of the
 * resource deallocation process. A return value of 0 indicates success,
 * while a non-zero value indicates an error occurred during the removal
 * of resources.
 ******************************************************************************/
int32_t ad5446_remove(struct ad5446_dev *dev);

/* Write to shift register via SPI. */
/***************************************************************************//**
 * @brief This function is used to send a command and data to the AD5446 device
 * by writing to its shift register through the SPI interface. It should
 * be called when you need to update the device's configuration or output
 * settings. The function requires a valid device structure that has been
 * initialized with `ad5446_init`. The command and data parameters are
 * combined and sent over SPI, with the behavior potentially differing
 * based on the specific device's capabilities, such as control register
 * presence. Ensure that the device is properly configured and that the
 * SPI interface is correctly set up before calling this function.
 *
 * @param dev A pointer to an `ad5446_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @param command A 8-bit command value to be sent to the device. The valid
 * range is determined by the device's command set.
 * @param data A 16-bit data value to be sent to the device. The valid range is
 * determined by the device's resolution and configuration.
 * @return None
 ******************************************************************************/
void ad5446_set_register(struct ad5446_dev *dev,
			 uint8_t command,
			 uint16_t data);

/* Sets the output voltage. */
/***************************************************************************//**
 * @brief This function configures the output voltage of an AD5446 device based
 * on the specified voltage, reference voltage, and output type. It
 * should be called after the device has been initialized using
 * `ad5446_init`. The function calculates the appropriate register value
 * to achieve the desired output voltage and writes it to the device. It
 * handles different output types, including unipolar, unipolar inverted,
 * and bipolar, and ensures the register value does not exceed the
 * device's resolution limits. The actual voltage set is returned, which
 * may differ slightly from the requested voltage due to resolution
 * constraints.
 *
 * @param dev A pointer to an initialized `ad5446_dev` structure representing
 * the device. Must not be null.
 * @param voltage The desired output voltage to set. The valid range depends on
 * the `vout_type` and `vref`.
 * @param vref The reference voltage used for calculations. Must be a positive,
 * non-zero value.
 * @param vout_type Specifies the type of output voltage configuration:
 * `unipolar`, `unipolar_inv`, or `bipolar`. Determines how the
 * voltage is interpreted and set.
 * @return Returns the actual voltage set on the device, which may be slightly
 * different from the requested voltage due to resolution limits.
 ******************************************************************************/
float ad5446_set_voltage(struct ad5446_dev *dev,
			 float voltage,
			 float vref,
			 enum vout_type_t vout_type);
