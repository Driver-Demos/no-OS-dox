/**
* \file
* \brief Contains ADRV9001 API ORx datapath data types
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_ORX_TYPES_H_
#define _ADI_ADRV9001_ORX_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
/* ADI specific header files */
//#include "adi_adrv9001_radioctrl_types.h"


/* Header files related to libraries */


/* System header files */
    
/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/

#define ADI_ADRV9001_ORX_GAIN_INDEX_MIN 2
#define ADI_ADRV9001_ORX_GAIN_INDEX_MAX 14

/*
*********************************************************************************************************
*                                             ENUMs
*********************************************************************************************************
*/

/***************************************************************************//**
 * @brief The `adi_adrv9001_OrxBbdcRejectionStatus_e` is an enumeration that
 * defines the status of the ORX baseband DC rejection algorithm in the
 * ADRV9001 API. It provides three possible states: disabled, enabled,
 * and paused, allowing for control over the algorithm's operation in the
 * ORX datapath.
 *
 * @param ADI_ADRV9001_ORX_BBDC_REJECTION_DISABLED ORX baseband DC rejection
 * algorithm disabled.
 * @param ADI_ADRV9001_ORX_BBDC_REJECTION_ENABLED ORX baseband DC rejection
 * algorithm enabled.
 * @param ADI_ADRV9001_ORX_BBDC_REJECTION_PAUSED ORX baseband DC rejection
 * algorithm paused (but enabled).
 ******************************************************************************/
typedef enum adi_adrv9001_OrxBbdcRejectionStatus
{
    ADI_ADRV9001_ORX_BBDC_REJECTION_DISABLED = 0,  /*!< ORX baseband DC rejection algorithm disabled */
    ADI_ADRV9001_ORX_BBDC_REJECTION_ENABLED  = 1,  /*!< ORX baseband DC rejection algorithm enabled */
    ADI_ADRV9001_ORX_BBDC_REJECTION_PAUSED   = 2   /*!< ORX baseband DC rejection algorithm paused (but enabled) */
} adi_adrv9001_OrxBbdcRejectionStatus_e;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_ORX_TYPES_H_ */
