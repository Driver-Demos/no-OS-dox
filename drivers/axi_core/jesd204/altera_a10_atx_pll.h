/***************************************************************************//**
 *   @file   altera_a10_atx_pll.c
 *   @brief  Driver for the Altera ATX PLL dynamic reconfiguration.
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
#ifndef ALTERA_A10_ATX_PLL_H_
#define ALTERA_A10_ATX_PLL_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "altera_adxcvr.h"

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function calculates the closest achievable output rate of the
 * Altera ATX PLL given a desired rate in kilohertz. It is useful for
 * determining the best possible rate that can be configured based on the
 * PLL's capabilities and constraints. The function should be called when
 * a specific output rate is desired, and it returns the closest possible
 * rate that can be achieved. It is important to ensure that the `xcvr`
 * parameter is properly initialized and represents a valid transceiver
 * configuration before calling this function. The function will return
 * an error if the calculated parameters are invalid.
 *
 * @param xcvr A pointer to a `struct adxcvr` representing the transceiver
 * configuration. Must not be null and should be properly
 * initialized before calling this function.
 * @param rate_khz The desired output rate in kilohertz. Must be a positive
 * integer representing the target rate for the PLL.
 * @return Returns the closest achievable rate in kilohertz as an `int32_t`.
 * Returns -1 if the calculation results in invalid parameters.
 ******************************************************************************/
int32_t altera_a10_atx_pll_round_rate(struct adxcvr *xcvr,
				      uint32_t rate_khz);
/***************************************************************************//**
 * @brief This function configures the Altera ATX PLL to achieve a specified
 * output rate in kilohertz. It should be called when a change in the PLL
 * output frequency is required. The function calculates the necessary
 * parameters to achieve the desired rate and updates the PLL
 * configuration accordingly. It is important to ensure that the `xcvr`
 * structure is properly initialized and that the desired `rate_khz` is
 * within the supported range of the PLL. The function handles parameter
 * validation internally and returns an error if the rate cannot be set.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver.
 * This structure must be initialized before calling this function.
 * The caller retains ownership.
 * @param rate_khz The desired output rate in kilohertz. Must be a positive
 * integer within the supported range of the PLL. If the rate is
 * invalid, the function returns an error.
 * @return Returns 0 on success, or -1 if the rate cannot be set due to invalid
 * parameters.
 ******************************************************************************/
int32_t altera_a10_atx_pll_set_rate(struct adxcvr *xcvr,
				    uint32_t rate_khz);
/***************************************************************************//**
 * @brief This function recalculates the output frequency of the Altera ATX PLL
 * based on the current configuration of the PLL dividers and the parent
 * clock rate. It should be used when the PLL configuration might have
 * changed, and an updated output rate is needed. The function requires
 * that the PLL arbitration is properly managed, as it acquires and
 * releases arbitration internally. It is important to ensure that the
 * `xcvr` structure is properly initialized and configured before calling
 * this function. The recalculated rate is returned, and if the initial
 * recalculation flag is set, the PLL rate is also updated to this new
 * value.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver
 * configuration. Must not be null. The structure should be properly
 * initialized and configured before calling this function. The
 * function assumes ownership of managing arbitration for the PLL.
 * @return Returns the recalculated PLL output rate in kHz as a 32-bit unsigned
 * integer. The value is clamped to the maximum representable 32-bit
 * unsigned integer if it exceeds this limit.
 ******************************************************************************/
uint32_t altera_a10_atx_pll_recalc_rate(struct adxcvr *xcvr);

#endif
