/***************************************************************************//**
*   @file   AD7780.h
*   @brief  AD7780 header file.
*   @author DNechita (dan.nechita@analog.com)
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
#ifndef __AD7780_H__
#define __AD7780_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/************************** AD7780 Definitions ********************************/
/******************************************************************************/

/* DOUT/RDY pin */
#define AD7780_RDY_STATE(value) no_os_gpio_get_value(dev->gpio_miso,             \
		                &value)

/* PDRST pin */
#define AD7780_PDRST_PIN_OUT    no_os_gpio_direction_output(dev->gpio_pdrst,     \
					              NO_OS_GPIO_HIGH);
#define AD7780_PDRST_HIGH       no_os_gpio_set_value(dev->gpio_pdrst,            \
			        NO_OS_GPIO_HIGH)
#define AD7780_PDRST_LOW        no_os_gpio_set_value(dev->gpio_pdrst,            \
			        NO_OS_GPIO_LOW)

/* FILTER pin */
#define AD7780_FILTER_PIN_OUT   no_os_gpio_direction_output(dev->gpio_filter,    \
					              NO_OS_GPIO_HIGH);
#define AD7780_FILTER_HIGH      no_os_gpio_set_value(dev->gpio_filter,           \
			        NO_OS_GPIO_HIGH)
#define AD7780_FILTER_LOW       no_os_gpio_set_value(dev->gpio_filter,           \
			        NO_OS_GPIO_LOW)

/* GAIN pin */
#define AD7780_GAIN_PIN_OUT     no_os_gpio_direction_output(dev->gpio_gain,      \
					              NO_OS_GPIO_HIGH);
#define AD7780_GAIN_HIGH        no_os_gpio_set_value(dev->gpio_gain,             \
			        NO_OS_GPIO_HIGH)
#define AD7780_GAIN_LOW         no_os_gpio_set_value(dev->gpio_gain,             \
			        NO_OS_GPIO_LOW)

/* Status bits */
#define AD7780_STAT_RDY         (1 << 7) // Ready bit.
#define AD7780_STAT_FILTER      (1 << 6) // Filter bit.
#define AD7780_STAT_ERR         (1 << 5) // Error bit.
#define AD7780_STAT_ID1         (1 << 4) // ID bits.
#define AD7780_STAT_ID0         (1 << 3) // ID bits.
#define AD7780_STAT_GAIN        (1 << 2) // Gain bit.
#define AD7780_STAT_PAT1        (1 << 1) // Status pattern bits.
#define AD7780_STAT_PAT0        (1 << 0) // Status pattern bits.

#define AD7780_ID_NUMBER        0x08

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad7780_dev` structure is designed to encapsulate the necessary
 * components for interfacing with the AD7780 analog-to-digital
 * converter. It includes pointers to SPI and GPIO descriptors, which are
 * essential for managing the communication and control signals required
 * by the AD7780. The structure facilitates the initialization and
 * operation of the ADC by organizing the hardware interface components
 * in a coherent manner.
 *
 * @param spi_desc Pointer to a SPI descriptor for managing SPI communication.
 * @param gpio_pdrst Pointer to a GPIO descriptor for the power-down/reset pin.
 * @param gpio_miso Pointer to a GPIO descriptor for the MISO pin, used for data
 * output and ready signal.
 * @param gpio_filter Pointer to a GPIO descriptor for the filter pin.
 * @param gpio_gain Pointer to a GPIO descriptor for the gain pin.
 ******************************************************************************/
struct ad7780_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_pdrst;
	struct no_os_gpio_desc	*gpio_miso;
	struct no_os_gpio_desc	*gpio_filter;
	struct no_os_gpio_desc	*gpio_gain;
};

/***************************************************************************//**
 * @brief The `ad7780_init_param` structure is used to encapsulate the
 * initialization parameters required to set up the AD7780 device. It
 * includes SPI initialization parameters and several GPIO initialization
 * parameters for controlling various pins associated with the AD7780,
 * such as power-down/reset, MISO, filter, and gain. This structure is
 * essential for configuring the device before it can be used for data
 * acquisition.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_pdrst Holds the initialization parameters for the GPIO pin used
 * for power-down/reset control.
 * @param gpio_miso Holds the initialization parameters for the GPIO pin used
 * for the MISO line.
 * @param gpio_filter Holds the initialization parameters for the GPIO pin used
 * for filter control.
 * @param gpio_gain Holds the initialization parameters for the GPIO pin used
 * for gain control.
 ******************************************************************************/
struct ad7780_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_pdrst;
	struct no_os_gpio_init_param	gpio_miso;
	struct no_os_gpio_init_param	gpio_filter;
	struct no_os_gpio_init_param	gpio_gain;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the necessary communication peripherals for the
 * AD7780 device and verifies its presence. It should be called before
 * any other operations on the AD7780 to ensure that the device is
 * properly initialized and ready for communication. The function
 * allocates memory for the device structure and initializes GPIO and SPI
 * interfaces as specified in the initialization parameters. If the
 * initialization is successful, the device is placed in a known state
 * with specific pin configurations. The function returns an error code
 * if any step in the initialization process fails, ensuring that the
 * caller can handle initialization failures appropriately.
 *
 * @param device A pointer to a pointer of type `struct ad7780_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * The caller takes ownership of the allocated memory and is
 * responsible for freeing it using `ad7780_remove`.
 * @param init_param A structure of type `struct ad7780_init_param` containing
 * initialization parameters for the SPI and GPIO interfaces.
 * Must be properly populated with valid initialization data
 * before calling the function.
 * @return Returns 0 on successful initialization, or -1 if an error occurs
 * during the process.
 ******************************************************************************/
int8_t ad7780_init(struct ad7780_dev **device,
		   struct ad7780_init_param init_param);

/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD7780
 * device after it is no longer needed. This function should be called to
 * clean up and prevent resource leaks after the device has been
 * initialized and used. It handles the deallocation of SPI and GPIO
 * descriptors and frees the memory allocated for the device structure.
 * Ensure that the device pointer is valid and was previously initialized
 * using `ad7780_init` before calling this function.
 *
 * @param dev A pointer to an `ad7780_dev` structure representing the device to
 * be removed. Must not be null and should have been initialized by
 * `ad7780_init`. The function will handle invalid pointers
 * gracefully by returning an error code.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a non-zero value indicates an error occurred during resource
 * deallocation.
 ******************************************************************************/
int32_t ad7780_remove(struct ad7780_dev *dev);

/***************************************************************************//**
 * @brief This function is used to wait until the DOUT/RDY pin of the AD7780
 * device goes low, indicating that the device is ready for the next
 * operation. It should be called when the user needs to ensure that the
 * device is ready before proceeding with further data transactions. The
 * function will return immediately if the pin is already low, or it will
 * wait for a specified timeout period. If the timeout is reached and the
 * pin has not gone low, the function returns an error code. This
 * function must be called with a valid device structure that has been
 * properly initialized.
 *
 * @param dev A pointer to an ad7780_dev structure representing the device. This
 * must be a valid, initialized device structure. The function will
 * not modify the structure, but it must not be null.
 * @return Returns 0 if the DOUT/RDY pin goes low before the timeout, or -1 if
 * the timeout is reached without the pin going low.
 ******************************************************************************/
int8_t ad7780_wait_rdy_go_low(struct ad7780_dev *dev);

/***************************************************************************//**
 * @brief This function retrieves a 24-bit sample from the AD7780 analog-to-
 * digital converter (ADC) and provides the status byte associated with
 * the sample. It should be called when a new sample is ready to be read
 * from the ADC, typically after ensuring the DOUT/RDY pin is low. The
 * function requires a valid device structure that has been initialized
 * with `ad7780_init`. The status byte is returned through a pointer,
 * allowing the caller to check the ADC's status flags.
 *
 * @param dev A pointer to an `ad7780_dev` structure representing the device.
 * This must be a valid, initialized device structure. The function
 * does not modify this structure.
 * @param p_status A pointer to a uint8_t where the status byte will be stored.
 * This pointer must not be null, and the caller is responsible
 * for providing a valid memory location.
 * @return Returns the 24-bit signed integer sample read from the ADC.
 ******************************************************************************/
int32_t ad7780_read_sample(struct ad7780_dev *dev,
			   uint8_t* p_status);

/***************************************************************************//**
 * @brief Use this function to convert a raw ADC sample obtained from the AD7780
 * into a voltage value, based on a specified reference voltage and gain
 * setting. This function is typically called after reading a raw sample
 * from the ADC to interpret the data in terms of voltage. Ensure that
 * the reference voltage and gain are correctly set according to your ADC
 * configuration to obtain accurate results.
 *
 * @param raw_sample A 24-bit unsigned integer representing the raw ADC sample.
 * It should be a valid sample obtained from the AD7780 ADC.
 * @param v_ref A floating-point number representing the reference voltage used
 * by the ADC. This value must be positive and should match the
 * reference voltage used during ADC operation.
 * @param gain An 8-bit unsigned integer representing the gain setting of the
 * ADC. It must be greater than zero, as it is used as a divisor in
 * the conversion calculation.
 * @return Returns a floating-point number representing the converted voltage
 * value.
 ******************************************************************************/
float ad7780_convert_to_voltage(uint32_t raw_sample,
				float v_ref,
				uint8_t gain);

#endif /* __AD7780_H__ */
