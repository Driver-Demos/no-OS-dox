/***************************************************************************//**
 * @file dac_demo.h
 * @brief Header file of DAC Demo Driver.
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

#ifndef IIO_DEMO_DAC_
#define IIO_DEMO_DAC_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include "iio_types.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

#define MAX_DAC_ADDR		16
/* For testing a maximum of 16 channels can be enabled */
#ifndef TOTAL_DAC_CHANNELS
#define TOTAL_DAC_CHANNELS 2
#endif

/***************************************************************************//**
 * @brief The `dac_demo_desc` structure is designed to simulate a Digital-to-
 * Analog Converter (DAC) device for testing purposes. It includes an
 * array of dummy registers, global and channel-specific attributes, and
 * manages multiple channels with a specified active channel. The
 * structure also supports loopback functionality with buffers for each
 * channel, allowing for testing of data flow and processing within the
 * simulated DAC environment.
 *
 * @param reg An array of dummy registers used for testing, with a maximum size
 * defined by MAX_DAC_ADDR.
 * @param dac_global_attr A 32-bit unsigned integer representing a global
 * attribute of the DAC device.
 * @param dac_ch_attr An array of 32-bit unsigned integers representing channel-
 * specific attributes, with a size defined by MAX_DAC_ADDR.
 * @param active_ch A 32-bit unsigned integer indicating the currently active
 * channel.
 * @param loopback_buffer_len A 32-bit unsigned integer specifying the number of
 * samples in each buffer.
 * @param loopback_buffers A pointer to an array of buffers for each channel,
 * where each buffer is a pointer to an array of 16-bit
 * unsigned integers.
 ******************************************************************************/
struct dac_demo_desc {
	/** Dummy registers of device for testing */
	uint8_t reg[MAX_DAC_ADDR];
	/** Demo global device attribute */
	uint32_t dac_global_attr;
	/** Demo device channel attribute */
	uint32_t dac_ch_attr[MAX_DAC_ADDR];
	/** Active channel**/
	uint32_t active_ch;
	/** Number of samples in each buffer */
	uint32_t loopback_buffer_len;
	/** Array of buffers for each channel*/
	uint16_t **loopback_buffers;
};

/***************************************************************************//**
 * @brief The `dac_demo_init_param` structure is used to initialize a DAC demo
 * device, encapsulating configuration parameters such as global and
 * channel-specific attributes, the length of loopback buffers, and the
 * buffers themselves for ADC/DAC communication. This structure is
 * essential for setting up the DAC device with the necessary attributes
 * and buffer configurations for testing and demonstration purposes.
 *
 * @param dev_global_attr A 32-bit unsigned integer representing a global
 * attribute for the DAC device.
 * @param dev_ch_attr An array of 32-bit unsigned integers representing channel-
 * specific attributes for the DAC device, with a maximum
 * size defined by MAX_DAC_ADDR.
 * @param loopback_buffer_len A 32-bit unsigned integer indicating the number of
 * samples in each buffer used for loopback
 * operations.
 * @param loopback_buffers A pointer to an array of pointers to 16-bit unsigned
 * integers, serving as buffers for ADC/DAC
 * communication.
 ******************************************************************************/
struct dac_demo_init_param {
	/** Demo global dac attribute */
	uint32_t dev_global_attr;
	/** Demo dac channel attribute */
	uint32_t dev_ch_attr[MAX_DAC_ADDR];
	/** Number of samples in each buffer */
	uint32_t loopback_buffer_len;
	/** Buffer for adc/dac communication*/
	uint16_t **loopback_buffers;
};

/***************************************************************************//**
 * @brief The `iio_dac_demo_attributes` enumeration defines two constants that
 * represent different types of attributes for a DAC (Digital-to-Analog
 * Converter) device in a demo context. `DAC_CHANNEL_ATTR` is used to
 * specify attributes that are specific to individual channels of the
 * DAC, while `DAC_GLOBAL_ATTR` is used for attributes that apply to the
 * DAC device as a whole. This enumeration is likely used to
 * differentiate between settings or configurations that are channel-
 * specific versus those that affect the entire device.
 *
 * @param DAC_CHANNEL_ATTR Represents a channel-specific attribute for the DAC.
 * @param DAC_GLOBAL_ATTR Represents a global attribute applicable to the entire
 * DAC device.
 ******************************************************************************/
enum iio_dac_demo_attributes {
	DAC_CHANNEL_ATTR,
	DAC_GLOBAL_ATTR,
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function initializes a DAC demo descriptor using the provided
 * initialization parameters. It allocates memory for the descriptor and
 * sets up the loopback buffers and channel attributes as specified in
 * the initialization parameters. This function must be called before any
 * other operations on the DAC demo descriptor. If memory allocation
 * fails, the function returns an error code.
 *
 * @param desc A pointer to a pointer where the initialized DAC demo descriptor
 * will be stored. Must not be null. The caller takes ownership of
 * the allocated descriptor and is responsible for freeing it.
 * @param param A pointer to a dac_demo_init_param structure containing the
 * initialization parameters, including loopback buffers and
 * channel attributes. Must not be null. The structure should be
 * properly initialized before calling this function.
 * @return Returns 0 on success or a negative error code if memory allocation
 * fails.
 ******************************************************************************/
int32_t dac_demo_init(struct dac_demo_desc **desc,
		      struct dac_demo_init_param *param);

/***************************************************************************//**
 * @brief Use this function to properly release and clean up resources
 * associated with a DAC demo descriptor when it is no longer needed.
 * This function should be called to prevent memory leaks after the
 * descriptor has been initialized and used. It is important to ensure
 * that the descriptor is valid and not null before calling this
 * function, as passing a null descriptor will result in an error.
 *
 * @param desc A pointer to a `dac_demo_desc` structure that represents the DAC
 * demo descriptor to be removed. Must not be null. If null, the
 * function returns an error code.
 * @return Returns 0 on successful removal of the descriptor. Returns a negative
 * error code if the descriptor is null.
 ******************************************************************************/
int32_t dac_demo_remove(struct dac_demo_desc *desc);

/***************************************************************************//**
 * @brief This function sets the active channels of a DAC device to those
 * specified by the mask. It should be called when you need to change
 * which channels are active on the DAC. The function requires a valid
 * device descriptor and a mask indicating the channels to activate. If
 * the device descriptor is null, the function returns an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to the DAC device descriptor. Must not be null. If null,
 * the function returns -ENODEV.
 * @param mask An integer representing the bitmask of channels to activate. Each
 * bit corresponds to a channel, and the function sets the active
 * channels based on this mask.
 * @return Returns 0 on success, or -ENODEV if the device descriptor is null.
 ******************************************************************************/
int32_t update_dac_channels(void *dev, int32_t mask);

/***************************************************************************//**
 * @brief This function is used to deactivate all currently active DAC channels
 * associated with the given device descriptor. It should be called when
 * the DAC channels are no longer needed or before shutting down the
 * device to ensure that all channels are properly closed. The function
 * requires a valid device descriptor and will return an error if the
 * descriptor is null.
 *
 * @param dev A pointer to the device descriptor representing the DAC device.
 * Must not be null. If null, the function returns an error code
 * indicating that the device is not found.
 * @return Returns 0 on success, indicating that all channels have been
 * successfully closed. Returns -ENODEV if the device descriptor is
 * null.
 ******************************************************************************/
int32_t close_dac_channels(void* dev);

/***************************************************************************//**
 * @brief This function is used to submit samples from a buffer to a DAC demo
 * device for processing. It should be called when you have a buffer of
 * samples ready to be processed by the DAC. The function requires a
 * valid `iio_device_data` structure, which must contain a properly
 * initialized buffer and a valid DAC descriptor. If the `dev_data` is
 * null or the DAC descriptor's loopback buffers are not initialized, the
 * function will return an error. This function processes the samples in
 * the buffer and writes them to the loopback buffers of the active
 * channels.
 *
 * @param dev_data A pointer to an `iio_device_data` structure containing the
 * device data and buffer. Must not be null. The buffer within
 * this structure should be initialized and contain samples to
 * be submitted. If null, the function returns -ENODEV.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error, such as -ENODEV if `dev_data` is null
 * or -EINVAL if the loopback buffers are not initialized.
 ******************************************************************************/
int32_t dac_submit_samples(struct iio_device_data *dev_data);

/***************************************************************************//**
 * @brief This function retrieves the value of a specified DAC attribute and
 * writes it as a string into the provided buffer. It is used to access
 * either a global attribute or a channel-specific attribute of a DAC
 * device. The function must be called with a valid device pointer, and
 * the buffer must be large enough to hold the resulting string. The
 * attribute to be retrieved is specified by the attr_id parameter, which
 * can indicate either a global or channel-specific attribute. If the
 * device pointer is null, the function returns an error. Similarly, if
 * an invalid attribute ID is provided, an error is returned.
 *
 * @param device A pointer to the DAC device descriptor. Must not be null. If
 * null, the function returns -ENODEV.
 * @param buf A pointer to a character buffer where the attribute value will be
 * written as a string. The buffer must be large enough to hold the
 * resulting string.
 * @param len The size of the buffer in bytes. It determines the maximum number
 * of characters that can be written to the buffer.
 * @param channel A pointer to an iio_ch_info structure that specifies the
 * channel information. This is used when retrieving channel-
 * specific attributes.
 * @param attr_id An integer that specifies the attribute to retrieve. It can be
 * DAC_GLOBAL_ATTR for global attributes or DAC_CHANNEL_ATTR for
 * channel-specific attributes.
 * @return Returns the number of characters written to the buffer, excluding the
 * null terminator, or a negative error code if the operation fails.
 ******************************************************************************/
int get_dac_demo_attr(void *device, char *buf, uint32_t len,
		      const struct iio_ch_info *channel, intptr_t priv);

/***************************************************************************//**
 * @brief This function is used to set a specific attribute of a DAC device,
 * either globally or for a specific channel, based on the attribute ID
 * provided. It requires a valid device descriptor and a buffer
 * containing the new attribute value as a string. The function should be
 * called when there is a need to update the DAC's configuration
 * attributes. It returns the length of the buffer on success or a
 * negative error code if the device is invalid or the attribute ID is
 * unrecognized.
 *
 * @param device A pointer to the DAC device descriptor. Must not be null. If
 * null, the function returns -ENODEV.
 * @param buf A character buffer containing the new attribute value as a string.
 * The buffer must be properly null-terminated.
 * @param len The length of the buffer. It is used as the return value on
 * successful attribute update.
 * @param channel A pointer to an iio_ch_info structure that specifies the
 * channel information. It is used when setting channel-specific
 * attributes.
 * @param attr_id An integer representing the attribute ID to set. Must be
 * either DAC_GLOBAL_ATTR or DAC_CHANNEL_ATTR. If an unrecognized
 * ID is provided, the function returns -EINVAL.
 * @return Returns the length of the buffer on success, -ENODEV if the device is
 * null, or -EINVAL if the attribute ID is invalid.
 ******************************************************************************/
int set_dac_demo_attr(void *device, char *buf, uint32_t len,
		      const struct iio_ch_info *channel, intptr_t priv);

/***************************************************************************//**
 * @brief Use this function to read the value stored in a specific register of a
 * DAC demo descriptor. It is essential to ensure that the descriptor is
 * properly initialized and that the register index is within the valid
 * range before calling this function. This function is useful for
 * retrieving configuration or status information from the DAC demo
 * device. If the descriptor is null or the register index is out of
 * bounds, the function will return an error code.
 *
 * @param desc A pointer to a dac_demo_desc structure representing the DAC demo
 * device. Must not be null.
 * @param reg_index An unsigned 8-bit integer specifying the index of the
 * register to read. Must be less than the number of registers
 * available in the descriptor.
 * @param readval A pointer to an unsigned 8-bit integer where the read value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the descriptor is
 * null or the register index is invalid.
 ******************************************************************************/
int32_t dac_demo_reg_read(struct dac_demo_desc *desc, uint8_t reg_index,
			  uint8_t *readval);

/***************************************************************************//**
 * @brief This function is used to write a value to a specific register within a
 * DAC demo descriptor. It is essential to ensure that the descriptor is
 * valid and that the register index is within the allowable range before
 * calling this function. This function is typically used when there is a
 * need to modify the state or configuration of the DAC demo device by
 * directly writing to its registers. It returns an error code if the
 * descriptor is null or if the register index is out of bounds.
 *
 * @param desc A pointer to a dac_demo_desc structure representing the DAC demo
 * device. Must not be null. The function will return an error if
 * this parameter is null.
 * @param reg_index An unsigned 8-bit integer representing the index of the
 * register to write to. Must be less than the size of the
 * register array in the descriptor. If the index is out of
 * bounds, the function returns an error.
 * @param writeval An unsigned 8-bit integer representing the value to write to
 * the specified register. There are no restrictions on the
 * value itself.
 * @return Returns 0 on success, or a negative error code if the descriptor is
 * null or the register index is out of bounds.
 ******************************************************************************/
int32_t dac_demo_reg_write(struct dac_demo_desc *desc, uint8_t reg_index,
			   uint8_t writeval);

/***************************************************************************//**
 * @brief This function processes a trigger event for a DAC demo device by
 * reading data from the device's buffer and writing it to the
 * appropriate loopback buffers for each active channel. It should be
 * called whenever a trigger event occurs that requires data processing
 * for the DAC demo. The function expects a valid device data structure
 * and will return an error if the device data is null or if the loopback
 * buffers are not initialized. It handles the case where no data is
 * available by returning zero, indicating no processing was needed.
 *
 * @param dev_data A pointer to an iio_device_data structure representing the
 * device data. Must not be null. The structure should contain a
 * valid buffer and initialized loopback buffers for the
 * function to operate correctly. If dev_data is null, the
 * function returns -ENODEV. If loopback_buffers is not
 * initialized, it returns -EINVAL.
 * @return Returns 0 on successful processing of data or if no data is available
 * to process. Returns -ENODEV if dev_data is null and -EINVAL if
 * loopback_buffers is not initialized.
 ******************************************************************************/
int32_t dac_demo_trigger_handler(struct iio_device_data *dev_data);

#endif /*IIO_DEMO_DAC_*/
