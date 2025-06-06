/***************************************************************************//**
 *   @file   AD7303.h
 *   @brief  Header file of AD7303 Driver.
 *   @author Mihai Bancisor(Mihai.Bancisor@analog.com)
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
#ifndef __AD7303_H__
#define __AD7303_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/******************************** AD7303 **************************************/
/******************************************************************************/
/* Control Bits */
#define AD7303_INT       (0 << 7)    // Selects internal reference.
#define AD7303_EXT       (1 << 7)    // Selects external reference.
#define AD7303_LDAC      (1 << 5)    // Load DAC bit.
#define AD7303_PDB       (1 << 4)    // Power-down DAC B.
#define AD7303_PDA       (1 << 3)    // Power-down DAC A.
#define AD7303_A         (0 << 2)    // Address bit to select DAC A.
#define AD7303_B         (1 << 2)    // Address bit to select DAC B.
#define AD7303_CR1       (1 << 1)    // Control Bit 1.
#define AD7303_CR0       (1 << 0)    // Control Bit 0.

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad7303_dev` structure is a simple data structure designed to
 * encapsulate the SPI communication descriptor for the AD7303 device. It
 * contains a single member, `spi_desc`, which is a pointer to a
 * `no_os_spi_desc` structure. This structure is essential for managing
 * the SPI interface required for communication with the AD7303 digital-
 * to-analog converter (DAC).
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for communication.
 ******************************************************************************/
struct ad7303_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
};

/***************************************************************************//**
 * @brief The `ad7303_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up SPI communication
 * with the AD7303 device. It contains a single member, `spi_init`, which
 * is a structure itself that holds the necessary configuration details
 * for initializing the SPI interface, ensuring that the AD7303 can be
 * properly interfaced with a microcontroller or other controlling
 * device.
 *
 * @param spi_init Holds the initialization parameters for SPI communication.
 ******************************************************************************/
struct ad7303_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the AD7303 device by allocating necessary
 * resources and initializing SPI communication based on the provided
 * initialization parameters. It must be called before any other
 * operations on the AD7303 device to ensure proper setup. The function
 * returns a status code indicating success or failure of the
 * initialization process. If the allocation of resources fails, the
 * function returns an error code and the device pointer is not set.
 *
 * @param device A pointer to a pointer of type `struct ad7303_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure upon successful initialization.
 * @param init_param A structure of type `struct ad7303_init_param` containing
 * the initialization parameters for the SPI communication.
 * This must be properly configured before calling the
 * function.
 * @return Returns an `int8_t` status code: `0` for successful initialization,
 * or `-1` if memory allocation fails. If SPI initialization fails, a
 * negative error code is returned.
 ******************************************************************************/
int8_t ad7303_init(struct ad7303_dev **device,
		   struct ad7303_init_param init_param);

/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * associated with an AD7303 device after it is no longer needed. It must
 * be called after the device has been initialized with `ad7303_init`.
 * The function handles the cleanup of the SPI descriptor and deallocates
 * the memory used by the device structure. It is important to ensure
 * that the device pointer is valid and was previously initialized;
 * otherwise, the behavior is undefined.
 *
 * @param dev A pointer to an `ad7303_dev` structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid, initialized device structure. The function will free the
 * memory associated with this structure.
 * @return Returns an integer status code from the SPI removal operation, where
 * 0 typically indicates success and a negative value indicates an
 * error.
 ******************************************************************************/
int32_t ad7303_remove(struct ad7303_dev *dev);

/***************************************************************************//**
 * @brief This function is used to send control and data register values to the
 * AD7303 device over an SPI interface. It should be called whenever
 * there is a need to update the control settings or data output of the
 * AD7303. The function requires a valid device structure that has been
 * initialized with the SPI descriptor. It is important to ensure that
 * the device has been properly initialized using `ad7303_init` before
 * calling this function. The function does not return any value and does
 * not provide feedback on the success of the operation, so it is assumed
 * that the SPI communication is reliable and correctly configured.
 *
 * @param dev A pointer to an `ad7303_dev` structure representing the device.
 * This must be initialized and must not be null. The caller retains
 * ownership.
 * @param control_reg A uint8_t value representing the control register settings
 * to be sent to the device. Valid values depend on the
 * specific control bits defined for the AD7303.
 * @param data_reg A uint8_t value representing the data register settings to be
 * sent to the device. This is typically the data to be output
 * by the DAC.
 * @return None
 ******************************************************************************/
void ad7303_write(struct ad7303_dev *dev,
		  uint8_t control_reg,
		  uint8_t data_reg);

#endif /* __AD7303_H__ */
