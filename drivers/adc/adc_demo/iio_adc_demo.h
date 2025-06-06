/***************************************************************************//**
 *   @file   iio_adc_demo.h
 *   @brief  Header file of ADC Demo iio.
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

#ifndef IIO_DEMO_ADC
#define IIO_DEMO_ADC

#include <stdlib.h>
#include "iio_types.h"
#include "adc_demo.h"

/***************************************************************************//**
 * @brief The `adc_demo_iio_descriptor` is a global variable of type `struct
 * iio_device`, which is part of the IIO (Industrial I/O) framework. This
 * structure is used to describe an ADC (Analog-to-Digital Converter)
 * device in a demo setup, providing the necessary interface for IIO
 * operations.
 *
 * @details This variable is used to represent and manage the ADC device within
 * the IIO framework for demonstration purposes.
 ******************************************************************************/
extern struct iio_device adc_demo_iio_descriptor;
/***************************************************************************//**
 * @brief The variable `adc_iio_sw_trig_desc` is an external declaration of a
 * structure of type `iio_trigger`. This structure is likely used to
 * define a software trigger for an ADC (Analog-to-Digital Converter) in
 * an IIO (Industrial I/O) context. The exact fields and functionality of
 * the `iio_trigger` structure are not detailed in the provided code, but
 * it typically involves configuration and control of triggering
 * mechanisms for data acquisition.
 *
 * @details This variable is used to manage and configure a software-based
 * trigger for ADC operations within the IIO framework.
 ******************************************************************************/
extern struct iio_trigger adc_iio_sw_trig_desc;
/***************************************************************************//**
 * @brief The `adc_iio_timer_trig_desc` is a global variable of type `struct
 * iio_trigger`. It is declared as an external variable, indicating that
 * its definition is located in another source file. This variable is
 * likely used to describe a timer-based trigger for an ADC (Analog-to-
 * Digital Converter) in an IIO (Industrial I/O) context.
 *
 * @details This variable is used to manage and configure a timer-based trigger
 * for ADC operations in an IIO system.
 ******************************************************************************/
extern struct iio_trigger adc_iio_timer_trig_desc;

#endif /* IIO_DEMO_ADC */
