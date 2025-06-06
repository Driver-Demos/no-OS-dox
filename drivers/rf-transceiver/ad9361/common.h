/***************************************************************************//**
 *   @file   common.h
 *   @brief  Header file of Common Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
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
#ifndef COMMON_H_
#define COMMON_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_error.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
#if defined (__STDC__) && (__STDC_VERSION__ >= 199901L)
#include <stdbool.h>
#else
typedef enum { false, true } bool;
#endif

/***************************************************************************//**
 * @brief The `no_os_clk` structure is designed to represent a clock in a system
 * without an operating system. It contains two members: `name`, which is
 * a constant character pointer used to store the name of the clock, and
 * `rate`, which is a 32-bit unsigned integer that holds the clock's
 * rate. This structure is likely used in embedded systems or low-level
 * hardware interfacing where clocks need to be managed without the
 * overhead of an operating system.
 *
 * @param name A constant character pointer representing the name of the clock.
 * @param rate An unsigned 32-bit integer representing the rate of the clock.
 ******************************************************************************/
struct no_os_clk {
	const char	*name;
	uint32_t	rate;
};

/***************************************************************************//**
 * @brief The `no_os_clk_hw` structure is a simple data structure that
 * encapsulates a pointer to a `no_os_clk` structure, which represents a
 * hardware clock. This structure is used to manage and interface with
 * clock hardware in a system, allowing for the abstraction and
 * manipulation of clock properties through the `no_os_clk` structure it
 * points to.
 *
 * @param clk A pointer to a no_os_clk structure, representing a clock.
 ******************************************************************************/
struct no_os_clk_hw {
	struct no_os_clk *clk;
};

#endif
