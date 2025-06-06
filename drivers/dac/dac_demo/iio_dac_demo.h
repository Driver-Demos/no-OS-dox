/***************************************************************************//**
 *   @file   iio_dac_demo.h
 *   @brief  Header file of DAC Demo iio.
 *   @author RNechita (ramona.nechita@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
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

#ifndef IIO_DEMO_DAC
#define IIO_DEMO_DAC

#include <stdlib.h>
#include "iio_types.h"
#include "dac_demo.h"

/***************************************************************************//**
 * @brief The `dac_demo_iio_descriptor` is a global variable of type `struct
 * iio_device`, which is part of the IIO (Industrial I/O) framework. This
 * structure is used to describe a DAC (Digital-to-Analog Converter)
 * device in a demo setup, providing the necessary interface for IIO
 * operations.
 *
 * @details This variable is used to represent and manage the DAC device within
 * the IIO framework, facilitating interaction with the DAC in a demo
 * environment.
 ******************************************************************************/
extern struct iio_device dac_demo_iio_descriptor;
/***************************************************************************//**
 * @brief The variable `dac_iio_sw_trig_desc` is an external declaration of a
 * structure of type `iio_trigger`. This structure is likely used to
 * define a software trigger for a DAC (Digital-to-Analog Converter) in
 * an IIO (Industrial I/O) context. The `iio_trigger` structure typically
 * contains information necessary to manage and execute triggers in an
 * IIO device setup.
 *
 * @details This variable is used to manage a software-based trigger mechanism
 * for a DAC in an IIO system.
 ******************************************************************************/
extern struct iio_trigger dac_iio_sw_trig_desc;
/***************************************************************************//**
 * @brief The variable `dac_iio_timer_trig_desc` is an external declaration of a
 * structure of type `iio_trigger`. This structure is likely used to
 * describe a timer-based trigger for an IIO (Industrial I/O) device,
 * specifically for a DAC (Digital-to-Analog Converter) demo application.
 *
 * @details This variable is used to manage and configure a timer-based trigger
 * mechanism for the DAC demo IIO device.
 ******************************************************************************/
extern struct iio_trigger dac_iio_timer_trig_desc;

#endif /* IIO_DEMO_DAC */
