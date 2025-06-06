
/**
 * \file
 * \brief Type definitions for the ADRV9001 dynamic profile
 * \copyright Analog Devices Inc. 2021. All rights reserved.
 * Released under the ADRV9001 API license, for more information see "LICENSE.txt" in the SDK
 */

#ifndef _ADI_ADRV9001_DYNAMIC_PROFILE_TYPES_H_
#define _ADI_ADRV9001_DYNAMIC_PROFILE_TYPES_H_

#ifndef __KERNEL__
#include <stdint.h>
#endif

#include "adi_adrv9001_defines.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_txSettings_types.h"

/***************************************************************************//**
 * @brief The `adi_adrv9000_RxDynamicProfile_t` structure defines the dynamic
 * profile for a receiver channel in the ADRV9000 system. It includes
 * parameters for the receiver's sample rate, RF bandwidth, and a
 * wideband decimation top block, which are crucial for configuring the
 * receiver's dynamic behavior in response to changing conditions or
 * requirements.
 *
 * @param rxOutputate_Hz Receiver sample rate in Hz, with a fixed interface rate
 * during dynamic profile switching.
 * @param primaryBw_Hz Rx RF bandwidth.
 * @param rxWbDecTop Rx wideband decimation top block.
 ******************************************************************************/
typedef struct adi_adrv9000_RxDynamicProfile
{
    uint32_t rxOutputate_Hz;               /* Receiver sample rate in Hz. Note: in dynamic profile switching the interface rate is fixed. */
    uint32_t primaryBw_Hz;                 /* Rx RF bandwidth */
    adi_adrv9001_RxWbDecTop_t rxWbDecTop;  /* Rx wideband decimation top block */
} adi_adrv9000_RxDynamicProfile_t;

/***************************************************************************//**
 * @brief The `adi_adrv9000_TxDynamicProfile_t` structure defines the dynamic
 * profile for a transmitter channel in the ADRV9000 system. It includes
 * parameters for the transmitter's sample rate, RF bandwidth, and
 * wideband interpolation settings, which are crucial for configuring the
 * transmission characteristics dynamically. This structure is part of a
 * larger framework that allows for dynamic profile switching, enabling
 * flexible and efficient management of transmission parameters in
 * response to varying operational requirements.
 *
 * @param txInputRate_Hz Transmitter sample rate in Hz, with a fixed interface
 * rate during dynamic profile switching.
 * @param primaryBw_Hz Transmitter RF bandwidth.
 * @param txWbIntTop Tx wideband interpolation top block.
 ******************************************************************************/
typedef struct adi_adrv9000_TxDynamicProfile
{
    uint32_t txInputRate_Hz;               /* Transmitter sample rate in Hz. Note: in dynamic profile switching the interface rate is fixed. */
    uint32_t primaryBw_Hz;                 /* Tx RF bandwidth */
    adi_adrv9001_TxWbIntTop_t txWbIntTop;  /* Tx wideband interpolation top block */
} adi_adrv9000_TxDynamicProfile_t;

/***************************************************************************//**
 * @brief The `adi_adrv9000_DynamicProfileChannel_t` structure is designed to
 * encapsulate dynamic profile configurations for different channel types
 * in the ADRV9000 system. It includes separate dynamic profile
 * structures for the transmitter (Tx), receiver (Rx), observation
 * receiver (ORx), internal loopback (ILB), and external loopback (ELB)
 * channels. Each of these profiles is represented by its respective
 * dynamic profile structure, allowing for detailed configuration and
 * management of the channel's dynamic behavior in terms of sample rates
 * and bandwidths.
 *
 * @param txDynamicProfile Tx dynamic profile struct.
 * @param rxDynamicProfile Rx dynamic profile struct.
 * @param orxDynamicProfile ORx dynamic profile struct.
 * @param ilbDynamicProfile ILB dynamic profile struct.
 * @param elbDynamicProfile ELB dynamic profile struct.
 ******************************************************************************/
typedef struct adi_adrv9000_DynamicProfileChannel
{
    adi_adrv9000_TxDynamicProfile_t txDynamicProfile;  /*!< Tx dynamic profile struct */
    adi_adrv9000_RxDynamicProfile_t rxDynamicProfile;  /*!< Rx dynamic profile struct */
    adi_adrv9000_RxDynamicProfile_t orxDynamicProfile; /*!< ORx dynamic profile struct */
    adi_adrv9000_RxDynamicProfile_t ilbDynamicProfile; /*!< ILB dynamic profile struct */
    adi_adrv9000_RxDynamicProfile_t elbDynamicProfile; /*!< ELB dynamic profile struct */
} adi_adrv9000_DynamicProfileChannel_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DynamicProfileIndex_e` is an enumeration that
 * defines various dynamic profile indices for the ADRV9001 device, each
 * corresponding to a specific sampling rate. The indices range from 0 to
 * 5, with index 0 representing the slowest sampling rate and index 5
 * representing the fastest. This enumeration is used to select the
 * appropriate dynamic profile for the device's operation, ensuring that
 * the ratio between the interface rate and sampling rate remains a power
 * of 2.
 *
 * @param ADI_ADRV9001_DYNAMIC_PROFILE_INDEX0 Selects the dynamic profile index
 * = 0, slowest sampling rate.
 * @param ADI_ADRV9001_DYNAMIC_PROFILE_INDEX1 Selects the dynamic profile index
 * = 1.
 * @param ADI_ADRV9001_DYNAMIC_PROFILE_INDEX2 Selects the dynamic profile index
 * = 2.
 * @param ADI_ADRV9001_DYNAMIC_PROFILE_INDEX3 Selects the dynamic profile index
 * = 3.
 * @param ADI_ADRV9001_DYNAMIC_PROFILE_INDEX4 Selects the dynamic profile index
 * = 4.
 * @param ADI_ADRV9001_DYNAMIC_PROFILE_INDEX5 Selects the dynamic profile index
 * = 5, fastest sampling rate.
 ******************************************************************************/
typedef enum adi_adrv9001_DynamicProfileIndex
{
    ADI_ADRV9001_DYNAMIC_PROFILE_INDEX0 = 0,  /*!< Selects the dynamic profile index = 0, slowest sampling rate  */
    ADI_ADRV9001_DYNAMIC_PROFILE_INDEX1 = 1,  /*!< Selects the dynamic profile index = 1 */
    ADI_ADRV9001_DYNAMIC_PROFILE_INDEX2 = 2,  /*!< Selects the dynamic profile index = 2 */
    ADI_ADRV9001_DYNAMIC_PROFILE_INDEX3 = 3,  /*!< Selects the dynamic profile index = 3 */
    ADI_ADRV9001_DYNAMIC_PROFILE_INDEX4 = 4,  /*!< Selects the dynamic profile index = 4 */
    ADI_ADRV9001_DYNAMIC_PROFILE_INDEX5 = 5   /*!< Selects the dynamic profile index = 5, fastest sampling rate */
} adi_adrv9001_DynamicProfileIndex_e;

/***************************************************************************//**
 * @brief The `adi_adrv9000_DynamicProfile_t` structure is designed to manage
 * dynamic profiles for the ADRV9001 device, allowing for flexible
 * configuration of transmission and reception parameters. It includes a
 * `dynamicProfileIndex` to specify the current profile in use, and an
 * array `dynamicProfileChannels` that holds the dynamic profile
 * configurations for each channel, supporting multiple channels as
 * defined by `ADI_ADRV9001_MAX_NUM_CHANNELS`. This structure facilitates
 * dynamic switching between different profiles, optimizing the device's
 * performance for varying operational conditions.
 *
 * @param dynamicProfileIndex Current dynamic profile index.
 * @param dynamicProfileChannels Dynamic profiles array.
 ******************************************************************************/
typedef struct adi_adrv9000_DynamicProfile
{
    adi_adrv9001_DynamicProfileIndex_e dynamicProfileIndex; /*!< Current dynamic profile index */
    adi_adrv9000_DynamicProfileChannel_t dynamicProfileChannels[ADI_ADRV9001_MAX_NUM_CHANNELS];  /*!< Dynamic profiles array */
} adi_adrv9000_DynamicProfile_t;

#endif /* _ADI_ADRV9001_DYNAMIC_PROFILE_TYPES_H_ */
