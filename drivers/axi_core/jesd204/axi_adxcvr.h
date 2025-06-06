/***************************************************************************//**
 *   @file   axi_adxcvr.h
 *   @brief  Driver for the ADI AXI-ADXCVR Module.
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
#ifndef AXI_ADXCVR_H_
#define AXI_ADXCVR_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "xilinx_transceiver.h"

/******************************************************************************/
/********************** Macros and Types Declarations *************************/
/******************************************************************************/

// Selection of PLL reference clock source to drive the RXOUTCLK
#define ADXCVR_SYS_CLK_CPLL			0x00
#define ADXCVR_SYS_CLK_QPLL1		0x02
#define ADXCVR_SYS_CLK_QPLL0		0x03

// adi,out-clk-select
#define ADXCVR_OUTCLK_PCS		1
#define ADXCVR_OUTCLK_PMA		2
#define ADXCVR_REFCLK		3
#define ADXCVR_REFCLK_DIV2	4
#define ADXCVR_PROGDIV_CLK	5 /* GTHE3, GTHE4, GTYE4 only */

/***************************************************************************//**
 * @brief The `adxcvr` structure is designed to represent a high-speed
 * transceiver device configuration for the ADI JESD204B/C AXI_ADXCVR
 * module. It includes various configuration parameters such as device
 * name, base address, and enable flags for CPLL and QPLL, as well as
 * settings for transmission, lane configuration, and clock selection.
 * The structure also contains a nested structure for Xilinx transceiver
 * configuration and a pointer to a clock descriptor for managing output
 * clocks. This structure is essential for initializing and managing the
 * transceiver's operational parameters in a high-speed data
 * communication context.
 *
 * @param name Device Name.
 * @param base Base address.
 * @param cpll_enable Enable CPLL for the transceiver.
 * @param qpll_enable Enable QPLL for the transceiver.
 * @param tx_enable TX Enable.
 * @param lpm_enable Enable LPM mode for the transceiver; otherwise, use DFE.
 * @param num_lanes Number of lanes.
 * @param lane_rate_khz Lane rate in KHz.
 * @param ref_rate_khz Reference Clock rate.
 * @param sys_clk_sel Select the PLL reference clock source to be forwarded to
 * the OUTCLK MUX: 0-CPLL, 3-QPLL0.
 * @param out_clk_sel Controls the OUTCLKSEL multiplexer, controlling what will
 * be forwarded to OUTCLK pin.
 * @param xlx_xcvr Structure holding the configuration of the Xilinx
 * Transceiver.
 * @param clk_out Exported no-OS output clock.
 ******************************************************************************/
struct adxcvr {
	/** Device Name */
	const char *name;
	/** Base address */
	uint32_t base;
	/** Enable CPLL for the transceiver */
	bool cpll_enable;
	/** Enable QPLL for the transceiver */
	bool qpll_enable;
	/** TX Enable */
	bool tx_enable;
	/** Enable LPM mode for the transceiver. Otherwise use DFE. */
	bool lpm_enable;
	/** Number of lanes */
	uint32_t num_lanes;
	/** Lane rate in KHz */
	uint32_t lane_rate_khz;
	/** Reference Clock rate */
	uint32_t ref_rate_khz;
	/** Select the PLL reference clock source to be forwarded to the OUTCLK
	 * MUX: 0-CPLL, 3-QPLL0.
	 */
	uint32_t sys_clk_sel;
	/** Controls the OUTCLKSEL multiplexer, controlling what will be
	 * forwarded to OUTCLK pin.
	 */
	uint32_t out_clk_sel;
	/** Structure holding the configuration of the Xilinx Transceiver. */
	struct xilinx_xcvr xlx_xcvr;
	/** Exported no-OS output clock */
	struct no_os_clk_desc *clk_out;
};

/***************************************************************************//**
 * @brief The `adxcvr_init` structure is used to initialize the ADI AXI-ADXCVR
 * high-speed transceiver module. It contains configuration parameters
 * such as the device name, base address, clock source selection, and
 * lane rate. The structure also allows enabling low power mode (LPM) and
 * exporting a no-OS output clock, providing flexibility in configuring
 * the transceiver's operational characteristics.
 *
 * @param name Device Name.
 * @param base Base address.
 * @param sys_clk_sel Selects the PLL reference clock source to be forwarded to
 * the OUTCLK MUX.
 * @param out_clk_sel Controls the OUTCLKSEL multiplexer, determining what will
 * be forwarded to the OUTCLK pin.
 * @param lpm_enable Enables LPM mode for the transceiver; otherwise, DFE is
 * used.
 * @param lane_rate_khz Lane rate in KHz.
 * @param ref_rate_khz Reference Clock rate.
 * @param export_no_os_clk Indicates whether to export the no-OS output clock.
 ******************************************************************************/
struct adxcvr_init {
	/** Device Name */
	const char *name;
	/** Base address */
	uint32_t base;
	/** Select the PLL reference clock source to be forwarded to the OUTCLK
	 * MUX: 0-CPLL, 2-QPLL1 (GTH and GTY), 3-QPLL0.
	 */
	uint32_t sys_clk_sel;
	/** Controls the OUTCLKSEL multiplexer, controlling what will be
	 * forwarded to OUTCLK pin.
	 */
	uint32_t out_clk_sel;
	/** Enable LPM mode for the transceiver. Otherwise use DFE. */
	bool lpm_enable;
	/** Lane rate in KHz */
	uint32_t lane_rate_khz;
	/** Reference Clock rate */
	uint32_t ref_rate_khz;
	/** Export no-OS output clock */
	bool export_no_os_clk;
};

/***************************************************************************//**
 * @brief The `adxcvr_clk_ops` is a constant structure of type
 * `no_os_clk_platform_ops` that defines the clock operations for the ADI
 * AXI-ADXCVR module. This structure is used to interface with the clock
 * management functionalities of the transceiver, providing a
 * standardized way to perform clock-related operations.
 *
 * @details This variable is used to define and manage the clock operations for
 * the ADI AXI-ADXCVR module, facilitating clock control and
 * configuration.
 ******************************************************************************/
extern const struct no_os_clk_platform_ops adxcvr_clk_ops;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function reads a value from a specified Dynamic Reconfiguration
 * Port (DRP) register of the ADI AXI-ADXCVR module. It is used to access
 * configuration or status information from the transceiver. The function
 * requires a valid `adxcvr` structure, a DRP port identifier, and a
 * register address. The result is stored in the provided output
 * parameter. The function returns an error code if the operation fails,
 * which can occur if the DRP port is not ready or if other issues are
 * encountered during the read operation.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver.
 * Must not be null. The caller retains ownership.
 * @param drp_port An unsigned integer specifying the DRP port to read from.
 * Must be a valid port identifier.
 * @param reg An unsigned integer specifying the register address within the DRP
 * port to read from. Must be a valid register address.
 * @param val A pointer to an unsigned integer where the read value will be
 * stored. Must not be null. The caller provides the memory for this
 * output.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails. The read value is stored in the location pointed to by `val`
 * on success.
 ******************************************************************************/
int adxcvr_drp_read(struct adxcvr *xcvr,
		    unsigned int drp_port,
		    unsigned int reg,
		    unsigned int *val);
/***************************************************************************//**
 * @brief Use this function to write a value to a specific Dynamic
 * Reconfiguration Port (DRP) register of the ADXCVR module. This
 * function is typically used when configuring or modifying the behavior
 * of the transceiver at runtime. It requires a valid ADXCVR device
 * structure and the DRP port and register addresses to be specified. The
 * function will return an error code if the write operation fails, which
 * can occur if the DRP port is busy or invalid.
 *
 * @param xcvr A pointer to an initialized 'struct adxcvr' representing the
 * ADXCVR device. Must not be null.
 * @param drp_port An unsigned integer specifying the DRP port to write to. Must
 * be a valid port number.
 * @param reg An unsigned integer specifying the register address within the DRP
 * port to write to. Must be a valid register address.
 * @param val An unsigned integer specifying the value to write to the specified
 * register. Any 32-bit value is allowed.
 * @return Returns 0 on success or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int adxcvr_drp_write(struct adxcvr *xcvr,
		     unsigned int drp_port,
		     unsigned int reg,
		     unsigned int val);
/***************************************************************************//**
 * @brief This function is used to determine if there are any errors in the
 * status of the specified ADXCVR transceiver. It should be called when
 * you need to verify the operational status of the transceiver. The
 * function will attempt to read the status register multiple times, with
 * a delay between each attempt, until a valid status is obtained or a
 * timeout occurs. It is important to ensure that the `xcvr` parameter is
 * a valid pointer to an initialized `adxcvr` structure before calling
 * this function.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver
 * to be checked. Must not be null and should point to a properly
 * initialized transceiver instance. If the pointer is invalid, the
 * behavior is undefined.
 * @return Returns 0 if the transceiver status is OK, or -1 if an error is
 * detected or if the status could not be read within the timeout
 * period.
 ******************************************************************************/
int32_t adxcvr_status_error(struct adxcvr *xcvr);
/***************************************************************************//**
 * @brief This function is used to enable the clock for a given ADXCVR
 * transceiver device. It should be called when the transceiver is ready
 * to be used and requires its clock to be active. The function attempts
 * to reset the transceiver and checks for buffer status errors, retrying
 * the reset if necessary. It is important to ensure that the transceiver
 * structure is properly initialized before calling this function. The
 * function returns an error code if the clock cannot be enabled due to
 * persistent buffer status errors or other issues.
 *
 * @param xcvr A pointer to an initialized 'struct adxcvr' representing the
 * transceiver. Must not be null. The caller retains ownership of
 * the structure.
 * @return Returns 0 on success or a negative error code if the clock enable
 * operation fails.
 ******************************************************************************/
int adxcvr_clk_enable(struct adxcvr *xcvr);
/***************************************************************************//**
 * @brief Use this function to disable the clock of a specified ADXCVR
 * transceiver device. This function is typically called when the
 * transceiver is no longer needed or before reconfiguring it. It is
 * important to ensure that the `xcvr` parameter is a valid pointer to an
 * initialized `adxcvr` structure. The function does not perform any
 * error checking on the input parameter, so passing a null or invalid
 * pointer will result in undefined behavior.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver
 * whose clock is to be disabled. Must not be null and should point
 * to a valid, initialized `adxcvr` instance.
 * @return Returns 0 on success. The function does not provide error codes for
 * failure conditions.
 ******************************************************************************/
int adxcvr_clk_disable(struct adxcvr *xcvr);
/** AXI ADXCVR Device Initialization */
int32_t adxcvr_init(struct adxcvr **ad_xcvr,
		    const struct adxcvr_init *init);
/** AXI ADXCVR Resources Deallocation */
int32_t adxcvr_remove(struct adxcvr *xcvr);
/***************************************************************************//**
 * @brief This function configures the clock rate for the specified ADXCVR
 * transceiver instance. It should be called when a change in the
 * transceiver's operating frequency is required. The function calculates
 * and applies the necessary PLL configurations based on the provided
 * rate and parent rate. It handles both CPLL and QPLL configurations
 * depending on the transceiver's settings. The function must be called
 * with a valid `adxcvr` structure that has been properly initialized. It
 * returns an error code if the configuration fails, ensuring that the
 * caller can handle such cases appropriately.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver
 * instance. Must not be null and should be properly initialized
 * before calling this function.
 * @param rate The desired clock rate in Hz for the transceiver. Must be a
 * positive value.
 * @param parent_rate The parent clock rate in Hz. Must be a positive value and
 * typically represents the reference clock rate.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * that the clock rate could not be set.
 ******************************************************************************/
int adxcvr_clk_set_rate(struct adxcvr *xcvr,
			unsigned long rate,
			unsigned long parent_rate);
/** AXI ADXCVR Write */
int32_t adxcvr_write(struct adxcvr *xcvr, uint32_t reg_addr, uint32_t reg_val);
/** AXI ADXCVR Read */
int32_t adxcvr_read(struct adxcvr *xcvr, uint32_t reg_addr, uint32_t *reg_val);
#endif
