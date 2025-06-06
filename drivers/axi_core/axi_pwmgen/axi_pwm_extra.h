/*******************************************************************************
 *   @file   axi_pwm_extra.h
 *   @brief  Header containing types used in the axi_pwm driver.
 *   @author Cristian Pop (cristian.pop@analog.com)
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

#ifndef AXI_PWM_EXTRA_H_
#define AXI_PWM_EXTRA_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_pwm.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `axi_pwm_init_param` structure is used to define the
 * initialization parameters for an AXI PWM (Pulse Width Modulation)
 * controller. It includes the base address of the PWM core, the
 * reference clock frequency in Hertz, and the specific channel of the
 * PWM controller that is desired for operation. This structure is
 * essential for setting up the PWM controller with the correct
 * configuration before it is used in applications.
 *
 * @param base_addr PWM core base address.
 * @param ref_clock_Hz PWM reference clock in Hertz.
 * @param channel Desired channel of the PWM controller.
 ******************************************************************************/
struct axi_pwm_init_param {
	/** PWM core base address */
	uint32_t base_addr;
	/** PWM reference clock */
	uint32_t ref_clock_Hz;
	/** Desired channel of the pwm controller */
	uint32_t channel;
};

/***************************************************************************//**
 * @brief The `axi_pwm_desc` structure is a descriptor for an AXI PWM (Pulse
 * Width Modulation) controller, encapsulating essential configuration
 * parameters such as the base address of the PWM core, the reference
 * clock frequency, the specific channel to be used, and additional
 * details like the period of a disabled channel and the hardware version
 * for register offset validation. This structure is crucial for managing
 * and interfacing with the PWM hardware in a system.
 *
 * @param base_addr PWM core base address.
 * @param ref_clock_Hz PWM reference clock frequency in Hertz.
 * @param channel Desired channel of the PWM controller.
 * @param ch_period Stores the period when the channel is disabled.
 * @param hw_major_ver Hardware version for checking correct register offsets.
 ******************************************************************************/
struct axi_pwm_desc {
	/** PWM core base address */
	uint32_t base_addr;
	/** PWM reference clock */
	uint32_t ref_clock_Hz;
	/** Desired channel of the pwm controller */
	uint32_t channel;
	/** Used to store the period when the channel is disabled */
	uint32_t ch_period;
	/** Hardware version necessary for checking for the right register offsets  */
	uint32_t hw_major_ver;
};

/***************************************************************************//**
 * @brief The `axi_pwm_ops` is a constant structure of type
 * `no_os_pwm_platform_ops` that provides platform-specific operations
 * for the AXI PWM driver. It is used to define the function pointers and
 * operations that are specific to the AXI PWM hardware implementation.
 * This structure is essential for abstracting the hardware-specific
 * details and providing a uniform interface for PWM operations across
 * different platforms.
 *
 * @details This variable is used to interface with the AXI PWM hardware by
 * providing the necessary function pointers for platform-specific
 * operations.
 ******************************************************************************/
extern const struct no_os_pwm_platform_ops axi_pwm_ops;

#endif
