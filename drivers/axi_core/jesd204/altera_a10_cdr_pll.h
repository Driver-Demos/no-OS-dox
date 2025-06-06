/***************************************************************************//**
 *   @file   altera_a10_cdr_pll.h
 *   @brief  Driver for the Altera CDR/CMU PLL dynamic reconfiguration.
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
#ifndef ALTERA_A10_CDR_PLL_H_
#define ALTERA_A10_CDR_PLL_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "altera_adxcvr.h"

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function is used to determine the closest possible rate that the
 * Altera CDR/CMU PLL can achieve based on a desired target rate in
 * kilohertz. It is useful for applications where precise frequency
 * tuning is required, and the exact target rate may not be achievable
 * due to hardware limitations. The function should be called with a
 * valid `adxcvr` structure that represents the transceiver
 * configuration. It returns a rounded rate that is as close as possible
 * to the requested rate, or an error code if the calculation parameters
 * are invalid.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver
 * configuration. Must not be null. The caller retains ownership.
 * @param rate_khz The desired target rate in kilohertz. Must be a positive
 * integer.
 * @return Returns the closest achievable rate in kilohertz as a long integer.
 * If the calculation parameters are invalid, it returns -1.
 ******************************************************************************/
int32_t altera_a10_cdr_pll_round_rate(struct adxcvr *xcvr,
				      uint32_t rate_khz);
/***************************************************************************//**
 * @brief This function configures the Altera CDR/CMU PLL to operate at a
 * specified rate in kilohertz. It should be called when a change in the
 * PLL rate is required, ensuring that the PLL is dynamically
 * reconfigured to the new rate. The function must be called with a valid
 * `adxcvr` structure that represents the transceiver and a desired rate
 * in kilohertz. It performs necessary calculations and updates to the
 * PLL configuration registers. The function returns an error if the
 * calculated parameters are invalid, indicating that the requested rate
 * cannot be set.
 *
 * @param xcvr A pointer to an `adxcvr` structure representing the transceiver.
 * Must not be null. The caller retains ownership.
 * @param rate_khz The desired rate in kilohertz for the PLL. Must be a positive
 * integer. Invalid values result in an error return.
 * @return Returns 0 on success, or -1 if the rate cannot be set due to invalid
 * parameters.
 ******************************************************************************/
int32_t altera_a10_cdr_pll_set_rate(struct adxcvr *xcvr,
				    uint32_t rate_khz);
/***************************************************************************//**
 * @brief This function recalculates the current operating rate of the Altera
 * CDR/CMU PLL based on the configuration of the specified transceiver.
 * It should be used when there is a need to determine the current PLL
 * rate, especially after any dynamic reconfiguration. The function reads
 * configuration registers to compute the rate and may update the PLL
 * rate if certain conditions are met. It is important to ensure that the
 * transceiver structure is properly initialized and configured before
 * calling this function.
 *
 * @param xcvr A pointer to an adxcvr structure representing the transceiver.
 * This must be a valid, non-null pointer, and the structure should
 * be properly initialized before calling this function.
 * @return Returns the recalculated rate in kHz as a 32-bit unsigned integer.
 * The rate is clamped to ULONG_MAX if it exceeds this value.
 ******************************************************************************/
uint32_t altera_a10_cdr_pll_recalc_rate(struct adxcvr *xcvr);

#endif
