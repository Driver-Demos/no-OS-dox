/***************************************************************************//**
 * @file adc_demo.h
 * @brief Header file of ADC Demo Driver.
 * @author RNechita (ramona.nechita@analog.com)
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
 *******************************************************************************/

#ifndef IIO_DEMO_ADC_
#define IIO_DEMO_ADC_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include "iio_types.h"
#include "no_os_irq.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

#define MAX_ADC_ADDR		16
/* For testing a maximum of 16 channels can be enabled */
#ifndef TOTAL_ADC_CHANNELS
#define TOTAL_ADC_CHANNELS 2
#endif

/***************************************************************************//**
 * @brief The `adc_demo_desc` structure is a descriptor for a demo ADC (Analog-
 * to-Digital Converter) device used primarily for testing purposes. It
 * includes an array of dummy registers, global and channel-specific
 * attributes, and manages multiple channels with an active channel
 * indicator. The structure also handles external buffers for each
 * channel, with a specified length for the number of samples in each
 * buffer, facilitating the simulation of ADC operations in a controlled
 * environment.
 *
 * @param reg An array of dummy registers used for testing, with a maximum size
 * defined by MAX_ADC_ADDR.
 * @param adc_global_attr A 32-bit unsigned integer representing a global
 * attribute of the ADC device.
 * @param adc_ch_attr An array of 32-bit unsigned integers representing channel-
 * specific attributes, with a size defined by MAX_ADC_ADDR.
 * @param active_ch A 32-bit unsigned integer indicating the currently active
 * channel.
 * @param ext_buff_len A 32-bit unsigned integer specifying the number of
 * samples in each buffer.
 * @param ext_buff A pointer to an array of buffers, each buffer being
 * associated with a channel.
 ******************************************************************************/
struct adc_demo_desc {
	/** Dummy registers of device for testing */
	uint8_t reg[MAX_ADC_ADDR];
	/** Demo global device attribute */
	uint32_t adc_global_attr;
	/** Demo device channel attribute */
	uint32_t adc_ch_attr[MAX_ADC_ADDR];
	/** Active channel**/
	uint32_t active_ch;
	/** Number of samples in each buffer */
	uint32_t ext_buff_len;
	/** Array of buffers for each channel*/
	uint16_t **ext_buff;
};

/***************************************************************************//**
 * @brief The `adc_demo_init_param` structure is used to initialize the ADC demo
 * driver with specific configuration parameters. It includes global and
 * channel-specific attributes for the DAC, the length of the external
 * buffer, and pointers to the buffers for each channel. This structure
 * is essential for setting up the ADC demo environment, allowing for the
 * configuration of attributes and management of data buffers.
 *
 * @param dev_global_attr A 32-bit unsigned integer representing a global
 * attribute for the demo DAC.
 * @param dev_ch_attr An array of 32-bit unsigned integers representing channel-
 * specific attributes for the demo DAC, with a maximum size
 * defined by MAX_ADC_ADDR.
 * @param ext_buff_len A 32-bit unsigned integer indicating the number of
 * samples in each buffer.
 * @param ext_buff A pointer to an array of buffers, each buffer being a pointer
 * to an array of 16-bit unsigned integers, for each channel.
 ******************************************************************************/
struct adc_demo_init_param {
	/** Demo global dac attribute */
	uint32_t dev_global_attr;
	/** Demo dac channel attribute */
	uint32_t dev_ch_attr[MAX_ADC_ADDR];
	/** Number of samples in each buffer */
	uint32_t ext_buff_len;
	/**Array of buffers for each channel*/
	uint16_t **ext_buff;
};

/***************************************************************************//**
 * @brief The `iio_adc_demo_attributes` enumeration defines two constants that
 * represent different types of attributes for an ADC (Analog-to-Digital
 * Converter) demo device. `ADC_CHANNEL_ATTR` is used to specify
 * attributes that are specific to individual ADC channels, while
 * `ADC_GLOBAL_ATTR` is used for attributes that apply to the ADC device
 * as a whole. This enumeration is likely used to differentiate between
 * channel-specific and global settings in the ADC demo driver.
 *
 * @param ADC_CHANNEL_ATTR Represents an attribute specific to an ADC channel.
 * @param ADC_GLOBAL_ATTR Represents a global attribute applicable to the entire
 * ADC device.
 ******************************************************************************/
enum iio_adc_demo_attributes {
	ADC_CHANNEL_ATTR,
	ADC_GLOBAL_ATTR,
};

/***************************************************************************//**
 * @brief The `sine_lut` is a constant array of 128 unsigned 16-bit integers. It
 * is likely used as a lookup table for sine wave values, providing
 * precomputed values to optimize performance in applications requiring
 * sine calculations.
 *
 * @details This variable is used to provide quick access to sine values, likely
 * for signal processing or waveform generation in the ADC demo driver.
 ******************************************************************************/
extern const uint16_t sine_lut[128];

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up an ADC demo descriptor using the provided
 * initialization parameters. It allocates memory for the descriptor and
 * configures it based on the attributes and buffer information specified
 * in the initialization parameters. This function must be called before
 * any other operations on the ADC demo descriptor to ensure it is
 * properly initialized. If memory allocation fails, the function returns
 * an error code.
 *
 * @param desc A pointer to a pointer where the initialized ADC demo descriptor
 * will be stored. Must not be null. The caller is responsible for
 * managing the memory of the descriptor after initialization.
 * @param param A pointer to an adc_demo_init_param structure containing the
 * initialization parameters, including global and channel
 * attributes, buffer length, and buffer pointers. Must not be
 * null. The structure should be fully populated with valid data
 * before calling this function.
 * @return Returns 0 on successful initialization. If memory allocation fails,
 * returns -ENOMEM.
 ******************************************************************************/
int32_t adc_demo_init(struct adc_demo_desc **desc,
		      struct adc_demo_init_param *param);

/***************************************************************************//**
 * @brief This function is used to clean up and release resources associated
 * with a given ADC demo descriptor. It should be called when the
 * descriptor is no longer needed to ensure that any allocated memory is
 * properly freed. The function expects a valid descriptor pointer and
 * will return an error if the pointer is null. It is important to ensure
 * that the descriptor has been initialized before calling this function.
 *
 * @param desc A pointer to an adc_demo_desc structure that represents the ADC
 * demo descriptor to be removed. Must not be null. If null, the
 * function returns an error code.
 * @return Returns 0 on success, or a negative error code if the input is
 * invalid (e.g., -EINVAL if desc is null).
 ******************************************************************************/
int32_t adc_demo_remove(struct adc_demo_desc *desc);

/***************************************************************************//**
 * @brief This function is used to set the active channels of an ADC device by
 * applying a bitmask. It is typically called when the configuration of
 * the ADC channels needs to be updated. The function requires a valid
 * device descriptor, and it will return an error if the descriptor is
 * null. The bitmask specifies which channels should be active, allowing
 * for flexible configuration of the ADC channels.
 *
 * @param dev A pointer to the ADC device descriptor. Must not be null. If null,
 * the function returns an error code (-ENODEV). The caller retains
 * ownership.
 * @param mask A 32-bit unsigned integer representing the bitmask for selecting
 * active channels. Each bit corresponds to a channel, and setting a
 * bit to 1 activates the corresponding channel.
 * @return Returns 0 on success, or -ENODEV if the device descriptor is null.
 ******************************************************************************/
int32_t update_adc_channels(void *dev, uint32_t mask);

/***************************************************************************//**
 * @brief Use this function to deactivate all currently active ADC channels
 * associated with the given device descriptor. It is typically called
 * when ADC operations are complete or when the channels need to be
 * reset. The function requires a valid device descriptor and will return
 * an error if the descriptor is null.
 *
 * @param dev A pointer to the device descriptor representing the ADC device.
 * Must not be null. If null, the function returns an error code
 * (-ENODEV). The caller retains ownership of the memory.
 * @return Returns 0 on success, indicating that all channels have been
 * successfully closed. Returns -ENODEV if the provided device
 * descriptor is null.
 ******************************************************************************/
int32_t close_adc_channels(void* dev);

/***************************************************************************//**
 * @brief This function is used to submit samples from an ADC device to an IIO
 * buffer for further processing or analysis. It should be called when
 * there is a need to transfer ADC data into the buffer specified in the
 * device data structure. The function requires a valid device data
 * structure, which includes a buffer and device descriptor. If the
 * device data is null, the function will return an error. The function
 * handles both internal and external buffer configurations, ensuring
 * that samples are correctly pushed to the IIO buffer.
 *
 * @param dev_data A pointer to an iio_device_data structure containing the
 * device descriptor and buffer information. Must not be null.
 * If null, the function returns an error code (-ENODEV). The
 * structure should be properly initialized before calling this
 * function.
 * @return Returns the number of samples successfully submitted to the buffer,
 * or an error code if the operation fails.
 ******************************************************************************/
int32_t adc_submit_samples(struct iio_device_data *dev_data);

/***************************************************************************//**
 * @brief This function is used to read the value stored at a specific register
 * index within an ADC demo descriptor. It is essential to ensure that
 * the descriptor is properly initialized before calling this function.
 * The function will return an error if the descriptor is null or if the
 * register index is out of bounds. This function is typically used when
 * there is a need to access the current state or configuration of the
 * ADC demo device as represented by its registers.
 *
 * @param desc A pointer to an adc_demo_desc structure representing the ADC demo
 * device. Must not be null.
 * @param reg_index An unsigned 8-bit integer representing the index of the
 * register to read. Must be less than the number of registers
 * available in the descriptor.
 * @param readval A pointer to an 8-bit unsigned integer where the read value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the descriptor is
 * null or the register index is out of bounds.
 ******************************************************************************/
int32_t adc_demo_reg_read(struct adc_demo_desc *desc, uint8_t reg_index,
			  uint8_t *readval);

/***************************************************************************//**
 * @brief This function is used to write a specified value to a register within
 * the ADC demo descriptor structure. It is essential to ensure that the
 * descriptor is properly initialized and that the register index is
 * within the valid range before calling this function. If the descriptor
 * is null or the register index is out of bounds, the function will
 * return an error code. This function is typically used in scenarios
 * where the configuration of the ADC demo needs to be modified by
 * writing to its registers.
 *
 * @param desc A pointer to an adc_demo_desc structure representing the ADC demo
 * descriptor. Must not be null, as a null pointer will result in an
 * error.
 * @param reg_index An unsigned 8-bit integer representing the index of the
 * register to write to. Must be less than the size of the
 * register array in the descriptor; otherwise, an error is
 * returned.
 * @param writeval An unsigned 8-bit integer representing the value to write to
 * the specified register. There are no restrictions on the
 * value itself.
 * @return Returns 0 on success, or a negative error code if the descriptor is
 * null or the register index is out of bounds.
 ******************************************************************************/
int32_t adc_demo_reg_write(struct adc_demo_desc *desc, uint8_t reg_index,
			   uint8_t writeval);

/***************************************************************************//**
 * @brief This function is used to handle ADC trigger events by processing the
 * active channels and pushing the resulting data to the IIO buffer. It
 * should be called whenever an ADC trigger event occurs. The function
 * requires a valid `iio_device_data` structure, which must not be null,
 * as it contains the necessary device descriptor and buffer information.
 * If the external buffer is not set, the function uses a sine lookup
 * table to generate data. The function manages the index for data
 * generation or retrieval, ensuring it wraps around appropriately.
 *
 * @param dev_data A pointer to an `iio_device_data` structure containing the
 * device descriptor and buffer. Must not be null. If null, the
 * function returns -EINVAL.
 * @return Returns an integer status code. A negative value indicates an error,
 * such as -EINVAL for invalid input. On success, it returns the result
 * of `iio_buffer_push_scan`, which indicates the success of pushing
 * data to the buffer.
 ******************************************************************************/
int32_t adc_demo_trigger_handler(struct iio_device_data *dev_data);

#endif /*IIO_DEMO_ADC_*/
