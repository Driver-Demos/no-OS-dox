/**
 * \file
 * \brief Contains ADRV9001 Rx related private function prototypes for
 *        adrv9001_rx.c which help adi_adrv9001_rx.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADRV9001_RX_H_
#define _ADRV9001_RX_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "adi_adrv9001_rx_types.h"
#include "adrv9001_bf.h"

/***************************************************************************//**
 * @brief This function is used to format a gain table structure into a format
 * that is expected by the ARM DMA. It should be called when you need to
 * prepare gain table data for transmission to the ARM processor. The
 * function requires a valid device structure and non-null pointers for
 * both the input gain table and the output formatted gain table. The
 * number of gain indices must be greater than zero. If any of these
 * preconditions are not met, the function will return an error.
 *
 * @param device A pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param gainTablePtr A pointer to the input gain table. Must not be null. The
 * caller retains ownership.
 * @param formattedGainTablePtr A pointer to the memory where the formatted gain
 * table data will be stored. Must not be null. The
 * caller is responsible for ensuring sufficient
 * memory is allocated.
 * @param numGainIndicesInTable The number of gain table entries to format. Must
 * be greater than zero.
 * @return Returns an integer status code. Returns ADI_COMMON_ERR_NULL_PARAM if
 * any pointer is null, or ADI_COMMON_ACT_NO_ACTION if the function
 * completes successfully.
 ******************************************************************************/
int32_t adrv9001_RxGainTableFormat(adi_adrv9001_Device_t *device,
                                   adi_adrv9001_RxGainTableRow_t *gainTablePtr, 
                                   uint8_t *formattedGainTablePtr,
                                   uint16_t numGainIndicesInTable);

/***************************************************************************//**
 * @brief This function is used to convert gain table data formatted for ARM DMA
 * into a structured format that can be used by the application. It
 * should be called when you need to interpret the raw gain table data
 * received from ARM DMA into a more usable form. The function requires
 * valid pointers to the device structure, the destination gain table row
 * structure, and the source ARM DMA data. It also requires the number of
 * gain indices to be greater than zero. If any of these preconditions
 * are not met, the function will return an error.
 *
 * @param device A pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param gainTablePtr A pointer to the gain table row entry structure where the
 * parsed data will be stored. Must not be null.
 * @param armDmaDataGainTablePtr A pointer to the memory containing data from
 * ARM DMA. Must not be null.
 * @param numGainIndicesInTable The number of gain table entries to parse. Must
 * be greater than zero.
 * @return Returns an integer status code: ADI_COMMON_ERR_NULL_PARAM if any
 * pointer is null, ADI_COMMON_ACT_NO_ACTION if successful, or an error
 * code if numGainIndicesInTable is zero.
 ******************************************************************************/
int32_t adrv9001_RxGainTableParse(adi_adrv9001_Device_t *device,
                                  adi_adrv9001_RxGainTableRow_t *gainTablePtr, 
                                  uint8_t *armDmaDataGainTablePtr,
                                  uint16_t numGainIndicesInTable);

#ifdef __cplusplus
}
#endif

#endif