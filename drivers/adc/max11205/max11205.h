/***************************************************************************//**
 *   @file   max11205.h
 *   @brief  Implementation of max11205.h
 *   @author RBolboac (ramona.bolboaca@analog.com)
 *******************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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
#ifndef __MAX11205_H__
#define __MAX11205_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_gpio.h"
#include "no_os_irq.h"
#include "no_os_spi.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define MAX11205_VREF_MAX_MV 		3600
#define MAX11205_DATA_SIZE_BYTES 	2
#define MAX11205_SCALE 			NO_OS_GENMASK(14,0)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `max11205_dev` structure is designed to encapsulate the necessary
 * components for interfacing with the MAX11205 ADC device. It includes
 * descriptors for SPI communication, GPIO signaling, and interrupt
 * handling, as well as fields for storing the reference voltage and the
 * latest ADC data. The structure also contains a flag to indicate
 * whether the ADC data has been updated since the last read,
 * facilitating efficient data retrieval and processing.
 *
 * @param spi_desc Pointer to the SPI device descriptor.
 * @param gpio_rdy Pointer to the GPIO descriptor used to signal when ADC data
 * is available.
 * @param irq_ctrl Pointer to the IRQ device descriptor for handling interrupt
 * routines for GPIO RDY.
 * @param irq_cb IRQ callback descriptor for handling interrupt routines for
 * GPIO RDY.
 * @param vref_mv Reference voltage in millivolts.
 * @param adc_data_raw Stores the ADC raw data, updated at each interrupt.
 * @param data_updated Boolean flag indicating if data was updated since the
 * last read.
 ******************************************************************************/
struct max11205_dev {
	/** SPI device descriptor*/
	struct no_os_spi_desc   	*spi_desc;
	/** GPIO RDY descriptor used to signal when ADC data is available */
	struct no_os_gpio_desc  	*gpio_rdy;
	/** IRQ device descriptor used to handle interrupt routine for GPIO RDY */
	struct no_os_irq_ctrl_desc 	*irq_ctrl;
	/** IRQ callback used to handle interrupt routine for GPIO RDY */
	struct no_os_callback_desc	irq_cb;
	/** Reference voltage in millivolts */
	uint32_t			vref_mv;
	/** ADC raw data, updated at each interrupt */
	int16_t				adc_data_raw;
	/** True if data was updated since last read */
	bool				data_updated;
};

/***************************************************************************//**
 * @brief The `max11205_init_param` structure is used to initialize the MAX11205
 * device, which is an ADC (Analog-to-Digital Converter). It contains
 * parameters necessary for setting up the SPI communication, GPIO for
 * data readiness signaling, and interrupt handling. Additionally, it
 * specifies the reference voltage in millivolts, which is crucial for
 * accurate ADC operation.
 *
 * @param spi_init SPI device descriptor for initializing the SPI communication.
 * @param gpio_rdy Pointer to GPIO RDY descriptor used to signal when ADC data
 * is available.
 * @param irq_ctrl Pointer to IRQ device descriptor used to handle interrupt
 * routine for GPIO RDY.
 * @param vref_mv Reference voltage in millivolts.
 ******************************************************************************/
struct max11205_init_param {
	/** SPI device descriptor*/
	struct no_os_spi_init_param     spi_init;
	/** GPIO RDY descriptor used to signal when ADC data is available */
	struct no_os_gpio_init_param	*gpio_rdy;
	/** IRQ device descriptor used to handle interrupt routine for GPIO RDY */
	struct no_os_irq_ctrl_desc 	*irq_ctrl;
	/** Reference voltage in millivolts */
	uint32_t			vref_mv;
};

/***************************************************************************//**
 * @brief This function sets up and initializes a MAX11205 device using the
 * provided initialization parameters. It must be called before any other
 * operations on the device. The function configures the SPI interface,
 * sets up the GPIO for data ready signaling, and registers an interrupt
 * callback. The reference voltage must not exceed 3600 mV, and a valid
 * IRQ controller must be provided. If initialization fails, the function
 * returns an error code and no device is allocated.
 *
 * @param device A pointer to a pointer where the initialized device structure
 * will be stored. Must not be null. On success, the caller takes
 * ownership of the allocated device and is responsible for
 * freeing it.
 * @param init_param A structure containing initialization parameters, including
 * SPI initialization parameters, a GPIO descriptor for data
 * ready signaling, an IRQ controller descriptor, and the
 * reference voltage in millivolts. The reference voltage must
 * not exceed 3600 mV, and the IRQ controller must be valid.
 * Invalid parameters result in an error.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the device pointer is set to a newly allocated and
 * initialized device structure.
 ******************************************************************************/
int max11205_init(struct max11205_dev **device,
		  struct max11205_init_param init_param);
/***************************************************************************//**
 * @brief This function is used to obtain the latest raw ADC data from a
 * MAX11205 device and to check if new data has been made available since
 * the last read. It should be called when the user needs to access the
 * most recent ADC conversion result. The function requires a valid
 * device structure and pointers to store the data and status. It will
 * reset the new data availability flag after reading. Ensure that the
 * device has been properly initialized before calling this function.
 *
 * @param dev A pointer to a max11205_dev structure representing the device.
 * Must not be null. The device should be initialized before calling
 * this function.
 * @param new_data_avail A pointer to a boolean where the function will store
 * whether new data is available. Must not be null.
 * @param data_raw A pointer to an int16_t where the function will store the raw
 * ADC data. Must not be null.
 * @return Returns 0 on success. If any input pointers are null, returns
 * -EINVAL.
 ******************************************************************************/
int max11205_get_data_raw(struct max11205_dev *dev, bool *new_data_avail,
			  int16_t *data_raw);
/***************************************************************************//**
 * @brief This function converts raw ADC data to a voltage value in millivolts
 * using the reference voltage specified in the device structure. It
 * should be called when you need to interpret the raw ADC data as a
 * voltage measurement. The function requires a valid device structure
 * and a pointer to store the converted millivolt value. It returns an
 * error if the device structure or the output pointer is null.
 *
 * @param dev A pointer to a max11205_dev structure representing the device.
 * Must not be null. The structure should be properly initialized and
 * contain a valid reference voltage.
 * @param raw_data The raw ADC data to be converted. It is a 16-bit signed
 * integer representing the ADC output.
 * @param data_mv A pointer to an integer where the converted millivolt value
 * will be stored. Must not be null. The function writes the
 * result to this location.
 * @return Returns 0 on success. Returns a negative error code if the device
 * structure or data_mv pointer is null.
 ******************************************************************************/
int max11205_get_data_mv(struct max11205_dev *dev, int16_t raw_data,
			 int32_t *data_mv);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for a
 * MAX11205 device when it is no longer needed. This includes disabling
 * and unregistering any associated GPIO interrupts, removing the SPI and
 * GPIO descriptors, and freeing the device structure. It is important to
 * call this function to prevent resource leaks. Ensure that the device
 * pointer is valid and initialized before calling this function.
 *
 * @param dev A pointer to a max11205_dev structure representing the device to
 * be removed. Must not be null and should be a valid, initialized
 * device structure. The function will handle invalid pointers by
 * returning an error code.
 * @return Returns 0 on success, or a negative error code if any step in the
 * removal process fails.
 ******************************************************************************/
int max11205_remove(struct max11205_dev *dev);

#endif /* __MAX11205_H__ */
