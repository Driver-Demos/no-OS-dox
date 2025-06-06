/**
* \file
* \brief Contains ADRV9001 API BBDC data types
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_BBDC_TYPES_H_
#define _ADI_ADRV9001_BBDC_TYPES_H_

/***************************************************************************//**
 * @brief The `adi_adrv9001_BbdcRejectionStatus_e` is an enumeration that
 * defines the status of the baseband DC rejection algorithm for the
 * ADRV9001 device. It includes three states: disabled, enabled, and
 * paused. When disabled, the algorithm does not apply any correction.
 * When enabled, the algorithm actively corrects the DC offset. The
 * paused state allows the algorithm to hold the last correction value
 * while not actively applying new corrections.
 *
 * @param ADI_ADRV9001_BBDC_REJECTION_DISABLED Rx/ORx baseband DC rejection
 * algorithm disabled; Disable does
 * not apply a correction.
 * @param ADI_ADRV9001_BBDC_REJECTION_ENABLED Rx/ORx baseband DC rejection
 * algorithm enabled.
 * @param ADI_ADRV9001_BBDC_REJECTION_PAUSED Rx/ORx baseband DC rejection
 * algorithm paused (but enabled),
 * holding the last correction value.
 ******************************************************************************/
typedef enum adi_adrv9001_BbdcRejectionStatus
{
    ADI_ADRV9001_BBDC_REJECTION_DISABLED,   //!< Rx/ORx baseband DC rejection algorithm disabled; Disable does not apply a correction
    ADI_ADRV9001_BBDC_REJECTION_ENABLED,    //!< Rx/ORx baseband DC rejection algorithm enabled
    /** Rx/ORx baseband DC rejection algorithm paused (but enabled)
     * \note Holds the last correction value
     */
    ADI_ADRV9001_BBDC_REJECTION_PAUSED
} adi_adrv9001_BbdcRejectionStatus_e;

#endif /* _ADI_ADRV9001_BBDC_TYPES_H_ */
