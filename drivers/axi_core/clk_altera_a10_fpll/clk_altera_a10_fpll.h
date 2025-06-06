/***************************************************************************//**
 *   @file   clk_altera_a10_fpll.h
 *   @brief  Driver for the Altera FPLL.
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
#ifndef CLK_ALTERA_A10_FPLL_H_
#define CLK_ALTERA_A10_FPLL_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `altera_a10_fpll` structure is used to represent the configuration
 * and state of an Altera Arria 10 FPGA Phase-Locked Loop (FPLL). It
 * includes fields for the name of the FPLL, its base address, the parent
 * clock rate, and a flag to indicate if an initial recalculation of the
 * clock settings is required. This structure is essential for managing
 * the FPLL's operation within the FPGA, allowing for initialization,
 * configuration, and control of the clock settings.
 *
 * @param name A pointer to a constant character string representing the name of
 * the FPLL.
 * @param base A 32-bit unsigned integer representing the base address of the
 * FPLL.
 * @param parent_rate A 32-bit unsigned integer representing the parent clock
 * rate.
 * @param initial_recalc A boolean indicating whether an initial recalculation
 * is needed.
 ******************************************************************************/
struct altera_a10_fpll {
	const char *name;
	uint32_t base;
	uint32_t parent_rate;
	bool initial_recalc;
};

/***************************************************************************//**
 * @brief The `altera_a10_fpll_init` structure is used to initialize an Altera
 * Arria 10 FPLL (Fractional Phase-Locked Loop) instance. It contains
 * essential configuration parameters such as the name of the FPLL, its
 * base address, and the parent clock rate, which are necessary for
 * setting up the FPLL in a system.
 *
 * @param name A constant character pointer representing the name of the FPLL
 * instance.
 * @param base A 32-bit unsigned integer representing the base address of the
 * FPLL.
 * @param parent_rate A 32-bit unsigned integer representing the parent clock
 * rate for the FPLL.
 ******************************************************************************/
struct altera_a10_fpll_init {
	const char *name;
	uint32_t base;
	uint32_t parent_rate;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
int32_t altera_a10_fpll_init(struct altera_a10_fpll **a10_fpll,
			     const struct altera_a10_fpll_init *init);
/***************************************************************************//**
 * @brief Use this function to release any resources associated with a
 * previously initialized Altera FPLL instance. It should be called when
 * the FPLL is no longer needed to ensure proper cleanup and avoid memory
 * leaks. The function assumes that the provided pointer is valid and was
 * previously allocated by a successful call to `altera_a10_fpll_init`.
 * It is the caller's responsibility to ensure that the FPLL is not in
 * use when this function is called.
 *
 * @param fpll A pointer to an `altera_a10_fpll` structure that was previously
 * initialized. Must not be null. The function does not perform any
 * checks on the validity of the pointer beyond assuming it was
 * allocated by `altera_a10_fpll_init`.
 * @return Returns 0 to indicate successful deallocation. No other values are
 * returned.
 ******************************************************************************/
int32_t altera_a10_fpll_remove(struct altera_a10_fpll *fpll);
/***************************************************************************//**
 * @brief Use this function to configure the output frequency of an Altera FPLL
 * instance. It must be called with a valid `altera_a10_fpll` structure
 * that has been properly initialized. The function calculates the
 * necessary parameters to achieve the desired frequency and updates the
 * FPLL configuration accordingly. If the parameters cannot be
 * calculated, the function returns an error. This function should be
 * used when you need to change the frequency output of the FPLL after
 * initialization.
 *
 * @param fpll A pointer to an `altera_a10_fpll` structure representing the FPLL
 * instance. Must not be null and should be initialized before
 * calling this function.
 * @param rate The desired output frequency in Hz. Must be a valid frequency
 * that the FPLL can achieve based on its configuration and
 * capabilities.
 * @return Returns 0 on success, or -1 if the parameters for the desired
 * frequency could not be calculated.
 ******************************************************************************/
int32_t altera_a10_fpll_set_rate(struct altera_a10_fpll *fpll, uint32_t rate);
/***************************************************************************//**
 * @brief This function enables the Altera FPLL (Fractional Phase-Locked Loop)
 * by writing a specific value to a control register. It should be called
 * when the FPLL needs to be activated, typically after it has been
 * initialized and configured. The function assumes that the provided
 * FPLL structure is valid and properly initialized. It does not perform
 * any error checking on the input parameter and always returns 0,
 * indicating success.
 *
 * @param fpll A pointer to an `altera_a10_fpll` structure representing the FPLL
 * to be enabled. This structure must be initialized and must not be
 * null. The caller retains ownership of the structure.
 * @return Returns 0 to indicate success.
 ******************************************************************************/
int32_t altera_a10_fpll_enable(struct altera_a10_fpll *fpll);
/***************************************************************************//**
 * @brief Use this function to disable the Altera A10 FPLL when it is no longer
 * needed or before reconfiguring it. This function should be called only
 * after the FPLL has been successfully initialized. Disabling the FPLL
 * will stop its operation, which may affect any dependent systems or
 * components. Ensure that any necessary precautions are taken before
 * disabling the FPLL to avoid unintended disruptions.
 *
 * @param fpll A pointer to an initialized 'struct altera_a10_fpll' representing
 * the FPLL to be disabled. Must not be null. The function does not
 * check for null pointers, so passing a null pointer will result in
 * undefined behavior.
 * @return None
 ******************************************************************************/
void altera_a10_fpll_disable(struct altera_a10_fpll *fpll);
#endif
