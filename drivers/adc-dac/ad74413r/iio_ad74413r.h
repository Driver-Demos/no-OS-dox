/***************************************************************************//**
 *   @file   iio_ad74413r.h
 *   @brief  Header file of the IIO AD74413r Driver.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
********************************************************************************
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
*******************************************************************************/
#ifndef IIO_AD74413R_H
#define IIO_AD74413R_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "iio.h"
#include "ad74413r.h"
#include "iio_trigger.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad74413r_diag_channel_config` structure is used to define the
 * configuration of a diagnostic channel in the AD74413R device. It
 * contains a boolean field `enabled` to indicate if the diagnostic
 * channel is active, and an `enum ad74413r_diag_mode` field `function`
 * to specify the diagnostic function or mode that the channel is set to
 * perform. This structure is part of the configuration setup for
 * managing diagnostic operations on the AD74413R device.
 *
 * @param enabled A boolean flag indicating whether the diagnostic channel is
 * enabled.
 * @param function An enumeration specifying the diagnostic mode of the channel.
 ******************************************************************************/
struct ad74413r_diag_channel_config {
	bool enabled;
	enum ad74413r_diag_mode function;
};

/***************************************************************************//**
 * @brief The `ad74413r_diag_available` is a static constant array of string
 * literals representing the available diagnostic modes for the AD74413R
 * device. Each element in the array corresponds to a specific diagnostic
 * mode, such as ground, temperature, or various voltage levels,
 * identified by predefined enumeration constants.
 *
 * @details This array is used to map diagnostic mode identifiers to their
 * corresponding string representations for easy reference and display.
 ******************************************************************************/
static const char * const ad74413r_diag_available[] = {
	[AD74413R_DIAG_AGND] = "agnd",
	[AD74413R_DIAG_TEMP] = "temp",
	[AD74413R_DIAG_AVDD] = "avdd",
	[AD74413R_DIAG_AVSS] = "avss",
	[AD74413R_DIAG_REFOUT] = "refout",
	[AD74413R_DIAG_ALDO_5V] = "aldo_5v",
	[AD74413R_DIAG_ALDO_1V8] = "aldo_1v8",
	[AD74413R_DIAG_DLDO_1V8] = "dldo_1v8",
	[AD74413R_DIAG_DVCC] = "dvcc",
	[AD74413R_DIAG_IOVDD] = "iovdd",
	[AD74413R_SENSEL_A] = "sensel_a",
	[AD74413R_SENSEL_B] = "sensel_b",
	[AD74413R_SENSEL_C] = "sensel_c",
	[AD74413R_SENSEL_D] = "sensel_d",
};

/***************************************************************************//**
 * @brief The `ad74413r_iio_desc` structure is designed to encapsulate the state
 * and configuration of an AD74413R device within an IIO (Industrial I/O)
 * context. It includes pointers to the device descriptor and IIO device,
 * as well as fields for managing active channels and their
 * configurations. Additionally, it holds a trigger for hardware events
 * and maintains the state of the conversion sequence. Diagnostic channel
 * configurations are also included to support various diagnostic modes.
 *
 * @param ad74413r_desc A pointer to an AD74413R descriptor structure.
 * @param iio_dev A pointer to an IIO device structure.
 * @param active_channels A 32-bit integer representing the active channels.
 * @param no_of_active_channels An 8-bit integer representing the number of
 * active channels.
 * @param channel_configs An array of channel configuration structures for each
 * channel.
 * @param trigger A pointer to an IIO hardware trigger structure.
 * @param conv_state An enumeration representing the conversion sequence state.
 * @param diag_channel_configs An array of diagnostic channel configuration
 * structures.
 ******************************************************************************/
struct ad74413r_iio_desc {
	struct ad74413r_desc *ad74413r_desc;
	struct iio_device *iio_dev;
	uint32_t active_channels;
	uint8_t no_of_active_channels;
	struct ad74413r_channel_config channel_configs[AD74413R_N_CHANNELS];
	struct iio_hw_trig *trigger;
	enum ad74413r_conv_seq conv_state;
	struct ad74413r_diag_channel_config
		diag_channel_configs[AD74413R_N_DIAG_CHANNELS];
};

/***************************************************************************//**
 * @brief The `ad74413r_iio_desc_init_param` structure is used to initialize an
 * IIO descriptor for the AD74413R device. It contains pointers and
 * arrays that define the initial configuration for the device, including
 * the main initialization parameters, channel-specific configurations, a
 * hardware trigger, and diagnostic channel configurations. This
 * structure is essential for setting up the device's operational
 * parameters before it is used in an IIO context.
 *
 * @param ad74413r_init_param A pointer to a structure containing initialization
 * parameters for the AD74413R device.
 * @param channel_configs An array of channel configuration structures for each
 * channel in the AD74413R device.
 * @param trigger A pointer to an IIO hardware trigger structure.
 * @param diag_channel_configs An array of diagnostic channel configuration
 * structures for each diagnostic channel in the
 * AD74413R device.
 ******************************************************************************/
struct ad74413r_iio_desc_init_param {
	struct ad74413r_init_param *ad74413r_init_param;
	struct ad74413r_channel_config channel_configs[AD74413R_N_CHANNELS];
	struct iio_hw_trig *trigger;
	struct ad74413r_diag_channel_config
		diag_channel_configs[AD74413R_N_DIAG_CHANNELS];
};

/***************************************************************************//**
 * @brief The `ad74413r_channel_map` structure is designed to store a collection
 * of IIO channels specific to a particular operation mode. It contains a
 * pointer to an array of `iio_channel` structures, which represent the
 * individual channels, and an integer that specifies the total number of
 * these channels. This structure is useful for managing and organizing
 * channels in applications that require interaction with multiple IIO
 * channels, such as in data acquisition systems.
 *
 * @param channels A pointer to an array of IIO channel structures.
 * @param num_channels An unsigned integer representing the number of channels
 * in the map.
 ******************************************************************************/
struct ad74413r_channel_map {
	struct iio_channel *channels;
	uint32_t num_channels;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function sets up an IIO descriptor for the AD74413R device,
 * preparing it for subsequent operations. It must be called before any
 * other operations on the device are performed. The function requires
 * valid initialization parameters and will allocate necessary resources.
 * If initialization fails, it returns an error code and ensures that no
 * resources are leaked. The caller is responsible for managing the
 * memory of the descriptor pointer, which will be populated upon
 * successful initialization.
 *
 * @param iio_desc A pointer to a pointer where the initialized IIO descriptor
 * will be stored. Must not be null. The caller retains
 * ownership and is responsible for freeing the memory using the
 * appropriate remove function.
 * @param init_param A pointer to a structure containing initialization
 * parameters for the AD74413R device. Must not be null and
 * must contain valid sub-parameters. If invalid, the function
 * returns an error code.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int ad74413r_iio_init(struct ad74413r_iio_desc **,
		      struct ad74413r_iio_desc_init_param *);
/***************************************************************************//**
 * @brief Use this function to properly release all resources and memory
 * associated with an AD74413R IIO descriptor when it is no longer
 * needed. This function should be called to prevent memory leaks after
 * the descriptor has been initialized and used. It is important to
 * ensure that the descriptor is not used after this function is called,
 * as it will be invalidated. The function returns an error code if the
 * underlying AD74413R descriptor removal fails, otherwise it returns 0.
 *
 * @param desc A pointer to the AD74413R IIO descriptor to be removed. Must not
 * be null. The caller retains ownership of the pointer, but the
 * memory it points to will be freed. If the descriptor is invalid
 * or null, the behavior is undefined.
 * @return Returns 0 on success, or a negative error code if the underlying
 * AD74413R descriptor removal fails.
 ******************************************************************************/
int ad74413r_iio_remove(struct ad74413r_iio_desc *);

#endif /** IIO_AD74413R_H */
