/***************************************************************************//**
 *   @file   iio_ad7091r8.h
 *   @brief  Header file of IIO AD7091R8 driver header file.
 *   @author Marcelo Schmitt (marcelo.schmitt@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
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
#ifndef IIO_AD7091R8_H
#define IIO_AD7091R8_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "iio.h"

/******************************************************************************/
/***************************** Define Section *********************************/
/******************************************************************************/
#define ad7091r8_iio_device(chans) {					\
	.num_ch = NO_OS_ARRAY_SIZE(chans),				\
	.channels = chans,						\
	.pre_enable = (int32_t (*)())ad7091r8_buffer_preenable,		\
	.trigger_handler = (int32_t (*)())ad7091r8_trigger_handler,	\
	.debug_reg_read = (int32_t (*)())ad7091r8_iio_read_reg,		\
	.debug_reg_write = (int32_t (*)())ad7091r8_iio_write_reg	\
}

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad7091r8_iio_timer_trig_desc` is an external variable of type
 * `struct iio_trigger`, which is part of the IIO (Industrial I/O)
 * subsystem. This structure is likely used to describe a trigger
 * mechanism for the AD7091R8 device, which is an analog-to-digital
 * converter (ADC).
 *
 * @details This variable is used to manage and describe the timer-based trigger
 * functionality for the AD7091R8 IIO device.
 ******************************************************************************/
extern struct iio_trigger ad7091r8_iio_timer_trig_desc;

/***************************************************************************//**
 * @brief The `ad7091r8_iio_dev` structure is a descriptor for an AD7091R8 IIO
 * device, encapsulating both the specific AD7091R8 device and its
 * associated IIO device interface. This structure is used to manage and
 * interface with the AD7091R8 device within the IIO framework, allowing
 * for operations such as initialization and removal of the device.
 *
 * @param ad7091r8_dev A pointer to an AD7091R8 device structure.
 * @param iio_dev A pointer to an IIO device structure.
 ******************************************************************************/
struct ad7091r8_iio_dev {
	struct ad7091r8_dev *ad7091r8_dev;
	struct iio_device *iio_dev;
};

/***************************************************************************//**
 * @brief The `ad7091r8_iio_dev_init_param` structure is designed to encapsulate
 * the initial parameters required for setting up an AD7091R8 IIO device.
 * It contains a single member, `ad7091r8_dev_init`, which is a pointer
 * to another structure that holds the specific initialization parameters
 * necessary for configuring the AD7091R8 device. This structure is used
 * during the initialization process to ensure that the device is set up
 * with the correct parameters.
 *
 * @param ad7091r8_dev_init A pointer to an ad7091r8_init_param structure, which
 * holds initialization parameters for the AD7091R8
 * device.
 ******************************************************************************/
struct ad7091r8_iio_dev_init_param {
	struct ad7091r8_init_param *ad7091r8_dev_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function initializes an AD7091R-2, AD7091R-4, or AD7091R-8 IIO
 * device based on the provided initialization parameters. It must be
 * called before any operations are performed on the device. The function
 * allocates memory for the device descriptor and sets up the device
 * according to the specified parameters. If the initialization
 * parameters are invalid or memory allocation fails, the function
 * returns an error code. The caller is responsible for ensuring that the
 * `init_param` is properly configured and non-null before calling this
 * function.
 *
 * @param iio_dev A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller takes ownership
 * of the allocated memory and is responsible for freeing it
 * using the appropriate remove function.
 * @param init_param A pointer to a structure containing the initialization
 * parameters for the device. Must not be null and must
 * include a valid `ad7091r8_dev_init` field. If invalid, the
 * function returns an error code.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -EINVAL for
 * invalid parameters, -ENOMEM for memory allocation failure).
 ******************************************************************************/
int ad7091r8_iio_init(struct ad7091r8_iio_dev **iio_dev,
		      struct ad7091r8_iio_dev_init_param *init_param);

/***************************************************************************//**
 * @brief This function is used to properly remove and deallocate resources
 * associated with an AD7091R-2/-4/-8 IIO device. It should be called
 * when the device is no longer needed to ensure that all associated
 * resources are freed and any necessary cleanup is performed. The
 * function expects a valid device descriptor and will return an error
 * code if the removal process encounters any issues. It is important to
 * ensure that the descriptor is not used after this function is called.
 *
 * @param desc A pointer to an ad7091r8_iio_dev structure representing the
 * device to be removed. Must not be null. The caller retains
 * ownership of the pointer, but the resources it points to will be
 * freed.
 * @return Returns 0 on successful removal, or a negative error code if an error
 * occurs during the removal process.
 ******************************************************************************/
int ad7091r8_iio_remove(struct ad7091r8_iio_dev *desc);

#endif /** IIO_AD7091R8_H */
