/***************************************************************************//**
 *   @file   iio_ad5791.h
 *   @brief  Header of AD5791 IIO Driver.
 *   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
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
#ifndef IIO_AD5791_H_
#define IIO_AD5791_H_

#include "iio.h"
#include "no_os_util.h"
#include "ad5791.h"

#define AD5791_CH_NO 1

/***************************************************************************//**
 * @brief The `ad5791_iio_powerdown_modes` enumeration defines the power-down
 * modes available for the AD5791 device, which include connecting the
 * output to ground through a resistor or setting the output to a high-
 * impedance state. These modes are used to manage the power consumption
 * and output behavior of the AD5791 device when it is not actively
 * driving a signal.
 *
 * @param AD5791_6kOHMS_TO_GND Represents a power-down mode where the output is
 * connected to ground through a 6k ohm resistor.
 * @param AD5791_THREE_STATE Represents a power-down mode where the output is in
 * a high-impedance state.
 ******************************************************************************/
enum ad5791_iio_powerdown_modes {
	AD5791_6kOHMS_TO_GND,
	AD5791_THREE_STATE
};

/***************************************************************************//**
 * @brief The `ad5791_iio_desc` structure is a descriptor for the AD5791 IIO
 * driver, encapsulating the necessary components to manage and interface
 * with an AD5791 device through the IIO framework. It includes pointers
 * to the device and IIO structures, the current power-down mode, and
 * reference voltage values, both positive and negative, in millivolts.
 *
 * @param ad5791_handle A pointer to an AD5791 device structure.
 * @param ad5791_iio_dev A pointer to an IIO device structure.
 * @param curr_mode An enumeration indicating the current power-down mode of the
 * AD5791.
 * @param vref_mv A 32-bit unsigned integer representing the positive reference
 * voltage in millivolts.
 * @param vref_neg_mv A 32-bit unsigned integer representing the negative
 * reference voltage in millivolts.
 ******************************************************************************/
struct ad5791_iio_desc {
	struct ad5791_dev *ad5791_handle;
	struct iio_device *ad5791_iio_dev;
	enum ad5791_iio_powerdown_modes curr_mode;
	uint32_t vref_mv;
	uint32_t vref_neg_mv;
};

/***************************************************************************//**
 * @brief The `ad5791_iio_init_param` structure is used to initialize the AD5791
 * IIO driver, containing parameters for the initial setup of the AD5791
 * device, including pointers to initial configuration data and reference
 * voltage settings.
 *
 * @param ad5791_initial A pointer to an ad5791_init_param structure for initial
 * configuration.
 * @param vref_mv A 32-bit unsigned integer representing the positive reference
 * voltage in millivolts.
 * @param vref_neg_mv A 32-bit unsigned integer representing the negative
 * reference voltage in millivolts.
 ******************************************************************************/
struct ad5791_iio_init_param {
	struct ad5791_init_param *ad5791_initial;
	uint32_t vref_mv;
	uint32_t vref_neg_mv;
};

/* Initialize the AD5791 IIO driver. */
/***************************************************************************//**
 * @brief This function initializes the AD5791 IIO driver by allocating and
 * setting up the necessary resources for the driver to operate. It must
 * be called before any other operations are performed on the AD5791
 * device through the IIO interface. The function requires a valid
 * initialization parameter structure and will return an error if memory
 * allocation fails or if the underlying AD5791 initialization encounters
 * an issue. Upon successful initialization, the function provides a
 * descriptor for the initialized driver.
 *
 * @param iio_dev A pointer to a pointer of type struct ad5791_iio_desc. This
 * will be set to point to the newly allocated and initialized
 * descriptor. Must not be null.
 * @param init_param A pointer to a struct ad5791_iio_init_param containing
 * initialization parameters such as reference voltages and
 * initial AD5791 settings. Must not be null and should be
 * properly initialized before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code, and the iio_dev pointer is not set.
 ******************************************************************************/
int32_t ad5791_iio_init(struct ad5791_iio_desc **iio_dev,
			struct ad5791_iio_init_param *init_param);

/* Free memory allocated by ad5791_iio_init(). */
/***************************************************************************//**
 * @brief Use this function to release resources associated with an AD5791 IIO
 * driver instance when it is no longer needed. It should be called to
 * clean up after a successful initialization with `ad5791_iio_init`.
 * This function ensures that any memory allocated for the driver is
 * properly freed, preventing memory leaks. It is important to ensure
 * that the `desc` parameter is valid and was previously initialized;
 * otherwise, the behavior is undefined.
 *
 * @param desc A pointer to an `ad5791_iio_desc` structure representing the
 * driver instance to be removed. This must not be null and should
 * point to a valid, initialized descriptor. The caller retains
 * ownership of the pointer, but the memory it points to will be
 * freed.
 * @return Returns 0 on success, or a negative error code if the removal process
 * fails.
 ******************************************************************************/
int32_t ad5791_iio_remove(struct ad5791_iio_desc *desc);

/*****************************************************************************/
/***************************** Constant definition ***************************/
/***************************************************************************//**
 * @brief The `iio_ad5791_device` is a constant global variable of type `struct
 * iio_device` that represents the IIO (Industrial I/O) device interface
 * for the AD5791 digital-to-analog converter (DAC). This structure is
 * used to define the interface and operations for interacting with the
 * AD5791 device through the IIO subsystem.
 *
 * @details This variable is used to provide a standardized interface for the
 * AD5791 DAC within the IIO framework, facilitating communication and
 * control of the device.
 ******************************************************************************/
extern struct iio_device const iio_ad5791_device;

#endif /* IIO_AD5791_H_ */
