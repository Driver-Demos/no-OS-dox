/***************************************************************************//**
 *   @file   jesd204_clk.h
 *   @brief  Analog Devices JESD204 clock.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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
#ifndef JESD204_CLK_H_
#define JESD204_CLK_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "axi_adxcvr.h"
#include "axi_jesd204_rx.h"
#include "axi_jesd204_tx.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `jesd204_clk` structure is designed to encapsulate the
 * configuration and control of a JESD204 clock system, integrating
 * components for transceiver, receiver, and transmitter functionalities.
 * It serves as a central point for managing the clock settings and
 * operations related to the JESD204 interface, which is crucial for
 * high-speed data communication in digital signal processing
 * applications.
 *
 * @param xcvr A pointer to an adxcvr structure, representing the transceiver
 * configuration.
 * @param jesd_rx A pointer to an axi_jesd204_rx structure, representing the
 * JESD204 receiver configuration.
 * @param jesd_tx A pointer to an axi_jesd204_tx structure, representing the
 * JESD204 transmitter configuration.
 ******************************************************************************/
struct jesd204_clk {
	struct adxcvr *xcvr;
	struct axi_jesd204_rx *jesd_rx;
	struct axi_jesd204_tx *jesd_tx;
};

/***************************************************************************//**
 * @brief The `jesd204_clk_ops` is a constant structure of type
 * `no_os_clk_platform_ops` that defines the platform-specific operations
 * for the JESD204 clock. This structure is likely used to abstract the
 * clock operations for different hardware platforms, providing a uniform
 * interface for enabling, disabling, and setting the rate of the clock.
 *
 * @details This variable is used to define the platform-specific operations for
 * managing the JESD204 clock.
 ******************************************************************************/
extern const struct no_os_clk_platform_ops jesd204_clk_ops;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Start the clock. */
/***************************************************************************//**
 * @brief This function enables the clock components associated with a JESD204
 * interface, including the transceiver, JESD204 receiver, and JESD204
 * transmitter, if they are present. It should be called when the clock
 * components need to be activated for data transmission or reception.
 * The function will attempt to enable each component in turn, returning
 * an error code if any component fails to enable. It is important to
 * ensure that the `jesd204_clk` structure is properly initialized and
 * that any required hardware is ready before calling this function.
 *
 * @param clk A pointer to a `jesd204_clk` structure that contains pointers to
 * the clock components to be enabled. The structure must be
 * initialized, and any of its members can be null if the
 * corresponding component is not used. The function will handle null
 * pointers by skipping the enabling of that component.
 * @return Returns 0 on success, or a negative error code if any of the clock
 * components fail to enable.
 ******************************************************************************/
int32_t jesd204_clk_enable(struct jesd204_clk *clk);
/* Stop the clock. */
/***************************************************************************//**
 * @brief This function is used to disable the clock components associated with
 * a JESD204 interface. It should be called when the clock is no longer
 * needed or before reconfiguring the clock settings. The function
 * attempts to disable the transceiver clock, the JESD204 receiver lane
 * clock, and the JESD204 transmitter lane clock, in that order. If any
 * of these components are not present, they are simply skipped. The
 * function returns an error code if disabling any component fails,
 * allowing the caller to handle such cases appropriately.
 *
 * @param clk A pointer to a `jesd204_clk` structure representing the clock
 * components to be disabled. The structure must be properly
 * initialized, and the pointer must not be null. The function will
 * attempt to disable each non-null component within the structure.
 * @return Returns 0 on success, or a negative error code if disabling any clock
 * component fails.
 ******************************************************************************/
int32_t jesd204_clk_disable(struct jesd204_clk *clk);
/* Change the frequency of the clock. */
/***************************************************************************//**
 * @brief This function sets the frequency of a JESD204 clock to a specified
 * rate. It should be used when there is a need to adjust the clock
 * frequency for a specific channel. The function requires a valid
 * `jesd204_clk` structure, and it is expected that the clock has been
 * properly initialized before calling this function. If the `xcvr`
 * member of the `jesd204_clk` structure is non-null, the function will
 * attempt to set the rate using the associated transceiver. The function
 * returns an error code if the operation fails, or zero if it succeeds
 * without making any changes.
 *
 * @param clk A pointer to a `jesd204_clk` structure representing the clock to
 * be modified. Must not be null.
 * @param chan A `uint32_t` representing the channel for which the clock rate is
 * to be set. The specific channel value is not used in the current
 * implementation.
 * @param rate A `uint32_t` specifying the desired clock rate in Hz. The
 * function will attempt to set this rate if possible.
 * @return Returns an `int32_t` error code. Zero indicates success, while a non-
 * zero value indicates an error occurred during the rate setting
 * process.
 ******************************************************************************/
int32_t jesd204_clk_set_rate(struct jesd204_clk *clk, uint32_t chan,
			     uint32_t rate);
#endif
