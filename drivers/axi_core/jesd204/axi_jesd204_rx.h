/***************************************************************************//**
 *   @file   axi_jesd204_rx.h
 *   @brief  Driver for the Analog Devices AXI-JESD204-RX peripheral.
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
#ifndef AXI_JESD204_RX_H_
#define AXI_JESD204_RX_H_

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
 * @brief The `jesd204_rx_config` structure is used to configure the JESD204B/C
 * receive peripheral, specifying key parameters such as the number of
 * octets per frame, the number of frames per multiframe, and the
 * subclass version. These parameters are essential for setting up the
 * JESD204B/C link, ensuring proper data transmission and synchronization
 * between devices.
 *
 * @param octets_per_frame Specifies the number of octets per frame in the
 * JESD204B/C link.
 * @param frames_per_multiframe Indicates the number of frames per multiframe in
 * the JESD204B/C link.
 * @param subclass_version Defines the subclass version of the JESD204B/C
 * standard being used.
 ******************************************************************************/
struct jesd204_rx_config {
	uint8_t octets_per_frame;
	uint16_t frames_per_multiframe;
	uint8_t subclass_version;
};

/***************************************************************************//**
 * @brief The `axi_jesd204_rx` structure represents a JESD204B/C receive
 * peripheral device, encapsulating various configuration parameters and
 * state information necessary for managing the device's operation. It
 * includes fields for device identification, configuration settings such
 * as the number of lanes and data path widths, clock frequencies, and
 * pointers to related structures for clock management and JESD204 state
 * machine control. This structure is integral to the initialization and
 * management of the JESD204 RX interface in a system.
 *
 * @param name Device Name as a constant character pointer.
 * @param base Base address of the device as a 32-bit unsigned integer.
 * @param version Version of the peripheral as a 32-bit unsigned integer.
 * @param num_lanes Number of lanes of the peripheral as a 32-bit unsigned
 * integer.
 * @param data_path_width Data path width as a 32-bit unsigned integer.
 * @param tpl_data_path_width TPL data path width as a 32-bit unsigned integer.
 * @param config Rx configuration as a struct of type jesd204_rx_config.
 * @param device_clk_khz Device clock frequency in KHz as a 32-bit unsigned
 * integer.
 * @param lane_clk_khz Lane clock frequency in KHz as a 32-bit unsigned integer.
 * @param encoder Selected encoder as an enum of type jesd204_encoder.
 * @param lane_clk Lane clock as a pointer to a struct of type no_os_clk_desc.
 * @param jdev JESD204 FSM device as a pointer to a struct of type jesd204_dev.
 ******************************************************************************/
struct axi_jesd204_rx {
	/** Device Name */
	const char *name;
	/** Base address */
	uint32_t base;
	/** Version of the Peripheral */
	uint32_t version;
	/* Number of lanes of the peripheral */
	uint32_t num_lanes;
	/** Data path width */
	uint32_t data_path_width;
	/** TPL data path width */
	uint32_t tpl_data_path_width;
	/** Rx coniguration */
	struct jesd204_rx_config config;
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
 * @brief The `jesd204_rx_init` structure is used to initialize a JESD204B/C
 * receive peripheral device. It contains configuration parameters such
 * as the device name, base address, and clock frequencies, as well as
 * JESD204-specific settings like the number of octets per frame and
 * frames per multi-frame. This structure is essential for setting up the
 * JESD204 interface, ensuring that the device operates with the correct
 * timing and data format.
 *
 * @param name A pointer to a constant character string representing the device
 * name.
 * @param base A 32-bit unsigned integer representing the base address of the
 * device.
 * @param octets_per_frame An 8-bit unsigned integer indicating the number of
 * octets per frame (F).
 * @param frames_per_multiframe A 16-bit unsigned integer specifying the number
 * of frames per multi-frame (K).
 * @param subclass An 8-bit unsigned integer representing the JESD204B subclass.
 * @param device_clk_khz A 32-bit unsigned integer for the device clock
 * frequency in KHz for the JESD204 interface.
 * @param lane_clk_khz A 32-bit unsigned integer for the lane clock frequency in
 * KHz.
 * @param lane_clk A pointer to a no_os_clk_desc structure representing the lane
 * clock.
 ******************************************************************************/
struct jesd204_rx_init {
	/** Device Name */
	const char *name;
	/** Base address */
	uint32_t base;
	/** Number of octets per frame (F) */
	uint8_t octets_per_frame;
	/** Number of frames per multi-frame (K) */
	uint16_t frames_per_multiframe;
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
 * @brief This function is used to enable the lane clock of a JESD204 RX
 * peripheral, which is necessary for the proper operation of the data
 * link. It should be called when the peripheral is ready to start
 * receiving data and after any necessary configuration has been
 * completed. The function assumes that the provided JESD204 RX structure
 * is properly initialized and valid. It does not perform any error
 * checking on the input structure and always returns 0, indicating
 * success.
 *
 * @param jesd A pointer to a struct axi_jesd204_rx representing the JESD204 RX
 * peripheral. This structure must be properly initialized before
 * calling this function. The pointer must not be null, as the
 * function does not perform null checks.
 * @return Returns 0 to indicate successful execution.
 ******************************************************************************/
int32_t axi_jesd204_rx_lane_clk_enable(struct axi_jesd204_rx *jesd);
/***************************************************************************//**
 * @brief Use this function to disable the lane clock of a JESD204 RX peripheral
 * when it is no longer needed or before reconfiguring the peripheral.
 * This function should be called only after ensuring that the peripheral
 * is in a safe state to have its clock disabled, as disabling the clock
 * while the peripheral is active may lead to undefined behavior. It is
 * typically used in conjunction with the corresponding enable function
 * to manage the clock state of the peripheral.
 *
 * @param jesd A pointer to an initialized 'struct axi_jesd204_rx' representing
 * the JESD204 RX peripheral. Must not be null. The function will
 * attempt to disable the lane clock associated with this
 * peripheral.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation. A non-zero return value indicates an error.
 ******************************************************************************/
int32_t axi_jesd204_rx_lane_clk_disable(struct axi_jesd204_rx *jesd);
/***************************************************************************//**
 * @brief Use this function to obtain and display the current status of a
 * JESD204 RX peripheral, including link status, clock rates, and SYSREF
 * status. It is typically called to diagnose or verify the operational
 * state of the peripheral. The function requires a valid
 * `axi_jesd204_rx` structure, which must be properly initialized before
 * calling. The function outputs status information to the standard
 * output and returns a status code.
 *
 * @param jesd A pointer to an `axi_jesd204_rx` structure representing the
 * JESD204 RX peripheral. This must be a valid, non-null pointer to
 * a properly initialized structure. The function does not modify
 * the structure.
 * @return Returns a `uint32_t` status code, which is always 0 in this
 * implementation, indicating successful execution.
 ******************************************************************************/
uint32_t axi_jesd204_rx_status_read(struct axi_jesd204_rx *jesd);
/***************************************************************************//**
 * @brief Use this function to obtain and display the status of a specific lane
 * in a JESD204 RX peripheral. It should be called when you need to check
 * the operational status or error count of a lane. The function requires
 * a valid JESD204 RX device structure and a lane number within the range
 * of available lanes. It prints the lane status and, if applicable, the
 * error count to the standard output. Ensure that the JESD204 RX device
 * is properly initialized before calling this function.
 *
 * @param jesd A pointer to a valid 'struct axi_jesd204_rx' representing the
 * JESD204 RX device. Must not be null. The structure should be
 * properly initialized before use.
 * @param lane The lane number to read the status from. Must be a valid lane
 * number within the range of lanes supported by the device.
 * @return Returns 0 on success. The function prints the lane status and error
 * information to the standard output.
 ******************************************************************************/
int32_t axi_jesd204_rx_laneinfo_read(struct axi_jesd204_rx *jesd,
				     uint32_t lane);
/***************************************************************************//**
 * @brief This function is used to monitor the status of the JESD204 RX link and
 * restart it if any lane status issues are detected. It should be called
 * periodically to ensure the link remains operational. The function
 * checks if the link is disabled and returns immediately if so. If the
 * link is active but lane status issues are detected, it will disable
 * and then re-enable the link to attempt a recovery. This function
 * assumes that the JESD204 RX peripheral has been properly initialized
 * and configured before use.
 *
 * @param jesd A pointer to an initialized 'struct axi_jesd204_rx' representing
 * the JESD204 RX peripheral. Must not be null. The function assumes
 * the structure is properly initialized and configured.
 * @return Returns 0 on successful execution, indicating the function completed
 * its monitoring and potential restart operations.
 ******************************************************************************/
int32_t axi_jesd204_rx_watchdog(struct axi_jesd204_rx *jesd);
/***************************************************************************//**
 * @brief This function initializes a JESD204 RX peripheral using the provided
 * initialization parameters. It allocates memory for the device
 * structure and configures the peripheral based on the specified
 * settings. The function must be called before any other operations on
 * the JESD204 RX peripheral. It checks for compatibility with the
 * expected peripheral version and identifier, and applies the
 * configuration settings. If initialization fails, it returns an error
 * code and ensures that allocated resources are freed.
 *
 * @param jesd204 A pointer to a pointer where the initialized JESD204 RX
 * structure will be stored. Must not be null. The caller takes
 * ownership of the allocated structure upon successful
 * initialization.
 * @param init A pointer to a jesd204_rx_init structure containing
 * initialization parameters such as device name, base address,
 * clock settings, and JESD204 configuration. Must not be null and
 * should be properly initialized with valid values.
 * @return Returns 0 on successful initialization, or -1 if an error occurs
 * during initialization, such as memory allocation failure or
 * incompatible peripheral version.
 ******************************************************************************/
int32_t axi_jesd204_rx_init_legacy(struct axi_jesd204_rx **jesd204,
				   const struct jesd204_rx_init *init);
/***************************************************************************//**
 * @brief This function initializes a JESD204 RX peripheral device using the
 * provided initialization parameters. It must be called before any other
 * operations on the JESD204 RX device. The function allocates memory for
 * the device structure and sets up the device based on the
 * initialization parameters. It checks for valid peripheral identifiers
 * and supported versions, and configures the device's data path and
 * encoder settings. If initialization fails, it returns an error code
 * and ensures that allocated resources are freed.
 *
 * @param jesd204 A pointer to a pointer of type `struct axi_jesd204_rx`. This
 * will be set to point to the newly initialized device
 * structure. Must not be null.
 * @param init A pointer to a `struct jesd204_rx_init` containing the
 * initialization parameters for the JESD204 RX device. Must not be
 * null and should be properly populated with valid configuration
 * data.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered, such as
 * memory allocation failure or unsupported peripheral version.
 ******************************************************************************/
int32_t axi_jesd204_rx_init(struct axi_jesd204_rx **jesd204,
			    const struct jesd204_rx_init *init);
/***************************************************************************//**
 * @brief Use this function to release all resources associated with a JESD204
 * RX device when it is no longer needed. This function should be called
 * to clean up after a device has been initialized and used, ensuring
 * that any allocated memory is properly freed. It is important to ensure
 * that the pointer provided is valid and that the device is not in use
 * by any other operations at the time of calling this function.
 *
 * @param jesd A pointer to the JESD204 RX device structure to be removed. Must
 * not be null, and the caller retains ownership of the pointer.
 * Passing an invalid or null pointer may result in undefined
 * behavior.
 * @return Returns 0 on successful deallocation of resources. No other values
 * are returned.
 ******************************************************************************/
int32_t axi_jesd204_rx_remove(struct axi_jesd204_rx *jesd);
#endif
