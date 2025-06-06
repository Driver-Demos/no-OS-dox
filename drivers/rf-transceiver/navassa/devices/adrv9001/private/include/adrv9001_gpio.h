/**
* \file
* \brief Contains ADRV9001 transmit related function prototypes for
*        adrv9001_gpio.c
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*
* Copyright 2015-2018 Analog Devices Inc.
* Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file.
*/

#ifndef _ADRV9001_GPIO_H_
#define _ADRV9001_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "adi_common_error.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_types.h"

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/***************************************************************************//**
 * @brief This function processes general purpose interrupts (GP_INT) for the
 * ADRV9001 device, determining the appropriate recovery action based on
 * the active interrupt sources. It should be called when a GP_INT is
 * detected to identify the source of the interrupt and to execute
 * necessary recovery actions. The function requires a valid device
 * structure and a status structure containing interrupt information. It
 * reports errors and recovery actions through the device's error
 * handling mechanism.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param gpIntStatus Pointer to a structure containing information about the
 * active GP interrupts. Must not be null. The caller retains
 * ownership.
 * @return Returns an integer indicating the recovery action to be taken, which
 * can be one of several predefined constants representing different
 * actions or errors.
 ******************************************************************************/
int32_t adrv9001_GpIntHandler(adi_adrv9001_Device_t *device, adi_adrv9001_gpIntStatus_t *gpIntStatus);

/***************************************************************************//**
 * @brief This function is used to obtain the current mask settings for general
 * purpose interrupts on pin B of the ADRV9001 device. It should be
 * called when the user needs to read the interrupt mask configuration.
 * The function requires a valid device pointer and a pointer to a
 * uint32_t variable where the mask value will be stored. It is important
 * to ensure that the device has been properly initialized before calling
 * this function. The function will return an error code if the device
 * pointer is null or if there is an issue reading the register values.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership and is responsible for
 * ensuring the device is initialized.
 * @param bfValue Pointer to a uint32_t variable where the mask value will be
 * stored. Must not be null if ADRV9001_BITFIELD_NULL_CHECK is
 * defined. The function writes the mask value to this location.
 * @return Returns an int32_t status code indicating success or the type of
 * error encountered. Possible return values include
 * ADI_COMMON_ACT_WARN_RESET_LOG, ADI_COMMON_ACT_ERR_CHECK_PARAM,
 * ADI_COMMON_ACT_ERR_RESET_INTERFACE, and ADI_COMMON_ACT_NO_ACTION.
 ******************************************************************************/
int32_t adrv9001_GpInterruptsMaskPinBfGet(adi_adrv9001_Device_t *device, uint32_t *bfValue);

/***************************************************************************//**
 * @brief This function is used to configure the interrupt mask for the ADRV9001
 * device, allowing specific interrupts to be masked based on the bit
 * positions in the provided value. It should be called when you need to
 * control which interrupts are output on gp_interrupt[0]. The function
 * requires a valid device pointer and a 32-bit value representing the
 * mask configuration. It returns an integer status code indicating
 * success or the type of error encountered.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param bfValue A 32-bit unsigned integer representing the interrupt mask
 * configuration. Each bit position corresponds to a specific
 * interrupt to be masked.
 * @return Returns an integer status code: 0 for success, or a non-zero error
 * code indicating the type of failure.
 ******************************************************************************/
int32_t adrv9001_GpInterruptsMaskPinBfSet(adi_adrv9001_Device_t *device, uint32_t bfValue);


/***************************************************************************//**
 * @brief This function reads the general purpose interrupt status word from the
 * ADRV9001 device and stores it in the provided buffer. It should be
 * called when the status of the general purpose interrupts needs to be
 * checked. The function requires a valid device pointer and a non-null
 * pointer to a uint32_t variable where the status word will be stored.
 * It returns a status code indicating the success or failure of the
 * operation, with specific codes for different types of errors or
 * required actions.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param bfValue Pointer to a uint32_t variable where the status word will be
 * stored. Must not be null. The function writes the status word
 * to this location.
 * @return Returns an int32_t status code indicating the result of the
 * operation. Possible values include ADI_COMMON_ACT_NO_ACTION for
 * success, and other codes indicating specific errors or required
 * recovery actions.
 ******************************************************************************/
int32_t adrv9001_GpInterruptsStatusWordBfGet(adi_adrv9001_Device_t *device, uint32_t *bfValue);

#ifdef __cplusplus
}
#endif

#endif /* _ADRV9001_GPIO_H_ */
