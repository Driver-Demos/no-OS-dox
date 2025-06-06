/***************************************************************************//**
 *   @file   iio_adxrs290.h
 *   @brief  Implementation of ADXRS290 iio.
 *   @author Kister Genesis Jimenez (kister.jimenez@analog.com)
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

#ifndef IIO_ADXRS290_H
#define IIO_ADXRS290_H

#include "iio_types.h"
#include "iio_trigger.h"

/***************************************************************************//**
 * @brief The `adxrs290_iio_descriptor` is a global variable of type `struct
 * iio_device` that represents the IIO (Industrial I/O) device descriptor
 * for the ADXRS290 gyroscope sensor. This descriptor is used to define
 * the characteristics and capabilities of the ADXRS290 sensor within the
 * IIO framework, facilitating its integration and interaction with the
 * IIO subsystem.
 *
 * @details This variable is used to interface the ADXRS290 gyroscope sensor
 * with the IIO subsystem, allowing for data acquisition and control.
 ******************************************************************************/
extern struct iio_device adxrs290_iio_descriptor;
/***************************************************************************//**
 * @brief The variable `adxrs290_iio_trig_desc` is a global variable of type
 * `struct iio_trigger`. It is declared as an external variable,
 * indicating that its definition is located in another source file. This
 * structure is likely used to manage and configure trigger events for
 * the ADXRS290 device within the IIO (Industrial I/O) framework.
 *
 * @details This variable is used to handle trigger events for the ADXRS290
 * sensor in the IIO subsystem.
 ******************************************************************************/
extern struct iio_trigger adxrs290_iio_trig_desc;

#endif
