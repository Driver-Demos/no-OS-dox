/***************************************************************************//**
 *   @file   axi_jesd204_tx.h
 *   @brief  Driver for the Analog Devices AXI-JESD204-TX peripheral.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2018(c) Analog Devices, Inc.
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
#ifndef AXI_JESD204_TX_H_
#define AXI_JESD204_TX_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "jesd204.h"
#include "no_os_clk.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `jesd204_tx_config` structure is used to configure the JESD204
 * transmit peripheral, detailing various parameters such as device,
 * bank, and lane identifiers, as well as configuration specifics like
 * the number of lanes, octets per frame, frames per multiframe, and
 * converter resolution. It also includes settings for JESD204
 * versioning, scrambling, and high-density operation, providing a
 * comprehensive configuration for the JESD204 transmission process.
 *
 * @param device_id Identifies the device within the JESD204 system.
 * @param bank_id Identifies the bank within the device.
 * @param lane_id Identifies the lane within the bank.
 * @param lanes_per_device Specifies the number of lanes available per device.
 * @param octets_per_frame Indicates the number of octets in each frame.
 * @param frames_per_multiframe Specifies the number of frames in each
 * multiframe.
 * @param converters_per_device Indicates the number of converters available per
 * device.
 * @param resolution Specifies the resolution of the converters.
 * @param bits_per_sample Indicates the number of bits per sample.
 * @param samples_per_frame Specifies the number of samples in each frame.
 * @param jesd_version Indicates the version of the JESD204 standard being used.
 * @param subclass_version Specifies the subclass version of the JESD204
 * standard.
 * @param control_bits_per_sample Indicates the number of control bits per
 * sample.
 * @param version Specifies the version of the configuration structure.
 * @param enable_scrambling Indicates whether scrambling is enabled.
 * @param high_density Indicates whether high-density mode is enabled.
 ******************************************************************************/
struct jesd204_tx_config {
	uint8_t device_id;
	uint8_t bank_id;
	uint8_t lane_id;
	uint8_t lanes_per_device;
	uint8_t octets_per_frame;
	uint16_t frames_per_multiframe;
	uint8_t converters_per_device;
	uint8_t resolution;
	uint8_t bits_per_sample;
	uint8_t samples_per_frame;
	uint8_t jesd_version;
	uint8_t subclass_version;
	uint8_t control_bits_per_sample;
	uint32_t version;
	bool enable_scrambling;
	bool high_density;
};

/***************************************************************************//**
 * @brief The `axi_jesd204_tx` structure is designed to represent the
 * configuration and state of a JESD204B/C transmit peripheral device. It
 * includes fields for the device's name, base address, number of lanes,
 * and various clock settings. Additionally, it holds a configuration
 * structure for JESD204 TX settings, such as the number of lanes, data
 * path width, and encoder type. This structure is essential for managing
 * the JESD204 interface, which is used in high-speed data transmission
 * applications.
 *
 * @param name A pointer to a string representing the device name.
 * @param base The base address of the device.
 * @param num_lanes The number of lanes in the JESD204 interface.
 * @param data_path_width The width of the data path.
 * @param tpl_data_path_width The width of the TPL data path.
 * @param config A structure containing the JESD204 TX configuration.
 * @param device_clk_khz The device clock frequency in kilohertz.
 * @param lane_clk_khz The lane clock frequency in kilohertz.
 * @param encoder The selected JESD204 encoder type.
 * @param lane_clk A pointer to the lane clock descriptor.
 * @param jdev A pointer to the JESD204 FSM device structure.
 ******************************************************************************/
struct axi_jesd204_tx {
	/** Device Name */
	const char *name;
	/** Base address */
	uint32_t base;
	/** Number of lanes */
	uint32_t num_lanes;
	/** Data path width */
	uint32_t data_path_width;
	/** TPL data path width */
	uint32_t tpl_data_path_width;
	/** Tx coniguration */
	struct jesd204_tx_config config;
	/** Device Clock in KHz */
	uint32_t device_clk_khz;
	/** Lane Clock in KHz */
	uint32_t lane_clk_khz;
	/** Selected Encoder */
	enum jesd204_encoder encoder;
	/** Lane Clock */
	struct no_os_clk_desc *lane_clk;
	/** JESD204 FSM device */
	struct jesd204_dev *jdev;
};

/***************************************************************************//**
 * @brief The `jesd204_tx_init` structure is used to initialize the JESD204B/C
 * Transmit Peripheral, providing configuration parameters such as device
 * name, base address, and various JESD204B link parameters including
 * octets per frame, frames per multi-frame, converters per device, and
 * converter resolution. It also includes clock settings for the device
 * and lane, and options for high density operation and subclass
 * specification, making it essential for setting up the JESD204
 * interface.
 *
 * @param name A pointer to a constant character string representing the device
 * name.
 * @param base An unsigned 32-bit integer representing the base address of the
 * device.
 * @param octets_per_frame An unsigned 8-bit integer indicating the number of
 * octets per frame (F).
 * @param frames_per_multiframe An unsigned 16-bit integer specifying the number
 * of frames per multi-frame (K).
 * @param converters_per_device An unsigned 8-bit integer representing the
 * number of converters per device (M).
 * @param converter_resolution An unsigned 8-bit integer indicating the
 * converter resolution (N).
 * @param bits_per_sample An unsigned 8-bit integer specifying the number of
 * bits per sample (N').
 * @param high_density A boolean indicating if the JESD204B link is configured
 * for high density (HD) operation.
 * @param control_bits_per_sample An unsigned 8-bit integer representing the
 * number of control bits per conversion sample
 * (CS).
 * @param subclass An unsigned 8-bit integer indicating the JESD204B subclass.
 * @param device_clk_khz An unsigned 32-bit integer representing the device
 * clock in KHz for the JESD204 interface.
 * @param lane_clk_khz An unsigned 32-bit integer specifying the lane clock in
 * KHz.
 * @param lane_clk A pointer to a no_os_clk_desc structure representing the lane
 * clock.
 ******************************************************************************/
struct jesd204_tx_init {
	/** Device Name */
	const char *name;
	/** Base address */
	uint32_t base;
	/** Number of octets per frame (F) */
	uint8_t octets_per_frame;
	/** Number of frames per multi-frame (K) */
	uint16_t frames_per_multiframe;
	/**  Number of converter per device (M) */
	uint8_t converters_per_device;
	/** Converter resolution (N) */
	uint8_t converter_resolution;
	/** Number of bits per sample (N') */
	uint8_t bits_per_sample;
	/** If specified the JESD204B link is configured for high density (HD) operation. */
	bool high_density;
	/**  Number of control bits per conversion sample (CS) */
	uint8_t control_bits_per_sample;
	/** The JESD204B subclass */
	uint8_t subclass;
	/** Device Clock in KHz for the JESD204 interface */
	uint32_t device_clk_khz;
	/** Lane Clock in KHz */
	uint32_t lane_clk_khz;
	/** Lane Clock */
	struct no_os_clk_desc *lane_clk;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function is used to enable the lane clock for a JESD204 TX
 * peripheral device. It should be called when the lane clock needs to be
 * activated, typically after the device has been initialized and is
 * ready to transmit data. The function assumes that the provided device
 * structure is valid and properly initialized. It does not perform any
 * error checking on the input parameter and always returns 0, indicating
 * success.
 *
 * @param jesd A pointer to a struct axi_jesd204_tx representing the JESD204 TX
 * peripheral device. Must not be null and should be properly
 * initialized before calling this function.
 * @return Returns 0 to indicate successful execution.
 ******************************************************************************/
int32_t axi_jesd204_tx_lane_clk_enable(struct axi_jesd204_tx *jesd);
/***************************************************************************//**
 * @brief Use this function to disable the lane clock of a JESD204 TX peripheral
 * when it is no longer needed or before reconfiguring the device. This
 * function is typically called after the lane clock has been enabled and
 * the device is operational. It is important to ensure that the `jesd`
 * parameter is a valid pointer to an initialized `axi_jesd204_tx`
 * structure. The function returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param jesd A pointer to an `axi_jesd204_tx` structure representing the
 * JESD204 TX peripheral. Must not be null and should point to a
 * properly initialized structure. The function will return an error
 * if this parameter is invalid.
 * @return Returns an integer status code. A non-negative value indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int32_t axi_jesd204_tx_lane_clk_disable(struct axi_jesd204_tx *jesd);
/***************************************************************************//**
 * @brief This function retrieves and displays the current status of the JESD204
 * TX peripheral, including link status, clock rates, and SYSREF status.
 * It is typically used for diagnostic purposes to verify the operational
 * state of the JESD204 TX interface. The function must be called with a
 * valid and initialized `axi_jesd204_tx` structure. It outputs the
 * status information to the standard output, providing insights into the
 * link's operational parameters and any potential issues.
 *
 * @param jesd A pointer to an `axi_jesd204_tx` structure representing the
 * JESD204 TX peripheral. This must be a valid, non-null pointer to
 * an initialized structure. The function does not modify the
 * structure.
 * @return Returns 0 after printing the status information to the standard
 * output. The function does not modify the input structure or return
 * any other data.
 ******************************************************************************/
uint32_t axi_jesd204_tx_status_read(struct axi_jesd204_tx *jesd);
/***************************************************************************//**
 * @brief This function sets up the AXI-JESD204-TX peripheral using a legacy
 * configuration method. It should be called to initialize the peripheral
 * before any data transmission operations. The function allocates memory
 * for the device structure and configures it based on the provided
 * initialization parameters. It checks for compatibility with the
 * expected peripheral version and identifier, and applies the
 * configuration. If initialization fails, it returns an error code and
 * ensures that allocated resources are freed. This function must be
 * called before any other operations on the JESD204-TX peripheral.
 *
 * @param jesd204 A pointer to a pointer of type `struct axi_jesd204_tx`. This
 * will be allocated and initialized by the function. The caller
 * must ensure this pointer is valid and will receive ownership
 * of the allocated structure upon successful initialization.
 * @param init A pointer to a `struct jesd204_tx_init` containing the
 * initialization parameters. This includes device name, base
 * address, clock frequencies, and JESD204 configuration parameters.
 * The pointer must not be null, and the structure must be fully
 * populated with valid data.
 * @return Returns 0 on successful initialization. Returns -1 if an error
 * occurs, such as memory allocation failure or incompatible peripheral
 * version.
 ******************************************************************************/
int32_t axi_jesd204_tx_init_legacy(struct axi_jesd204_tx **jesd204,
				   const struct jesd204_tx_init *init);
/***************************************************************************//**
 * @brief This function initializes the AXI-JESD204-TX peripheral device using
 * the provided initialization parameters. It must be called before any
 * other operations on the device. The function allocates memory for the
 * device structure and configures it based on the initialization
 * parameters. It checks for valid peripheral identifiers and supported
 * versions, and sets up the JESD204 configuration. If initialization
 * fails, it returns an error code and ensures no resources are leaked.
 * The caller is responsible for managing the memory of the initialized
 * device structure.
 *
 * @param jesd204 A pointer to a pointer where the initialized device structure
 * will be stored. Must not be null. The caller takes ownership
 * of the allocated structure upon successful initialization.
 * @param init A pointer to a jesd204_tx_init structure containing
 * initialization parameters. Must not be null. The structure should
 * be fully populated with valid configuration values.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -ENOMEM for
 * memory allocation failure, -ENODEV for unsupported device).
 ******************************************************************************/
int32_t axi_jesd204_tx_init(struct axi_jesd204_tx **jesd204,
			    const struct jesd204_tx_init *init);
/***************************************************************************//**
 * @brief Use this function to release all resources associated with a JESD204
 * TX device when it is no longer needed. This function should be called
 * to clean up after a device has been initialized and used, ensuring
 * that any allocated memory is properly freed. It is important to ensure
 * that the pointer provided is valid and that the device is not in use
 * by any other operations at the time of calling this function.
 *
 * @param jesd A pointer to the JESD204 TX device structure to be deallocated.
 * Must not be null, and the device should not be in use when this
 * function is called.
 * @return Returns 0 on successful deallocation of resources.
 ******************************************************************************/
int32_t axi_jesd204_tx_remove(struct axi_jesd204_tx *jesd);
#endif
