/**
* \file
* \brief ADRV9001 Multi-Chip Synchronization (MCS) data types
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2020 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_MCS_TYPES_H_
#define _ADI_ADRV9001_MCS_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stdbool.h>
#endif

/***************************************************************************//**
 * @brief The `adi_adrv9001_McsSwStatus_e` is an enumeration that defines the
 * various software statuses for Multi-Chip Synchronization (MCS) in the
 * ADRV9001 transceiver. It includes states indicating readiness for
 * synchronization pulses and transitions through receiving specific
 * pulses, as well as a state indicating the switch to a high-speed
 * clock. This enumeration is crucial for managing and tracking the
 * synchronization process in the transceiver's operation.
 *
 * @param ADI_ADRV9001_MCSSWSTATUS_READY Waiting for pulse 1 and 2 if MCS
 * substate is MCS_READY.
 * @param ADI_ADRV9001_MCSSWSTATUS_PULSE2_RECEIVED Pulse 2 received if MCS
 * substate is MCS_TRANSITION.
 * @param ADI_ADRV9001_MCSSWSTATUS_PULSE3_RECEIVED Pulse 3 received if MCS
 * substate is MCS_TRANSITION.
 * @param ADI_ADRV9001_MCSSWSTATUS_PULSE4_RECEIVED Pulse 4 received if MCS
 * substate is MCS_TRANSITION.
 * @param ADI_ADRV9001_MCSSWSTATUS_PULSE5_RECEIVED Pulse 5 received if MCS
 * substate is MCS_TRANSITION.
 * @param ADI_ADRV9001_MCSSWSTATUS_DEVICE_SWITCHED_TO_HSCLK Transceiver has
 * switched to high
 * speed clock.
 ******************************************************************************/
typedef enum adi_adrv9001_McsSwStatus
{
    ADI_ADRV9001_MCSSWSTATUS_READY                       = 0,    /*!< Waiting for pulse 1 and 2 if MCS substate is MCS_READY */
    ADI_ADRV9001_MCSSWSTATUS_PULSE2_RECEIVED             = 1,    /*!< Pulse 2 received if MCS substate is MCS_TRANSITION */
    ADI_ADRV9001_MCSSWSTATUS_PULSE3_RECEIVED             = 2,    /*!< Pulse 3 received if MCS substate is MCS_TRANSITION */
    ADI_ADRV9001_MCSSWSTATUS_PULSE4_RECEIVED             = 3,    /*!< Pulse 4 received if MCS substate is MCS_TRANSITION */
    ADI_ADRV9001_MCSSWSTATUS_PULSE5_RECEIVED             = 4,    /*!< Pulse 5 received if MCS substate is MCS_TRANSITION */
    ADI_ADRV9001_MCSSWSTATUS_DEVICE_SWITCHED_TO_HSCLK    = 5,    /*!< Trancseiver has switched to high speed clock */
} adi_adrv9001_McsSwStatus_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PllSyncStatus_t` structure is used to represent the
 * synchronization status of various components in the ADRV9001 system.
 * It contains boolean flags that indicate whether synchronization has
 * been completed for JESD, digital clocks, clock generation dividers,
 * SDM clock dividers, and reference clock dividers. This structure is
 * crucial for ensuring that all parts of the system are properly
 * synchronized, which is essential for the correct operation of the
 * ADRV9001 device.
 *
 * @param jesdSyncComplete Indicates whether JESD synchronization is complete.
 * @param digitalClocksSyncComplete Indicates whether digital clock
 * synchronization is complete.
 * @param clockGenDividerSyncComplete Indicates whether clock generation divider
 * synchronization is complete.
 * @param sdmClockDividerSyncComplete Indicates whether CLK PLL SDM clock
 * divider synchronization is complete.
 * @param referenceClockDividerSyncComplete Indicates whether reference clock
 * divider synchronization is complete.
 ******************************************************************************/
typedef struct adi_adrv9001_PllSyncStatus {
    bool jesdSyncComplete;                    /*!< JESD synchronization complete */
    bool digitalClocksSyncComplete;           /*!< digital clock synchronization complete */
    bool clockGenDividerSyncComplete;         /*!< clock generation divider synchronization complete */
    bool sdmClockDividerSyncComplete;         /*!< CLK PLL SDM clock divider synchronization complete */
    bool referenceClockDividerSyncComplete;   /*!< Reference clock divider synchronization complete */
} adi_adrv9001_PllSyncStatus_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxLvdsSyncStatus_t` structure is used to represent
 * the synchronization status of the Rx LVDS divider in the ADRV9001
 * device. It contains two boolean fields that indicate whether the first
 * and second synchronization processes have been completed, which are
 * crucial for ensuring proper timing and data integrity in the LVDS
 * interface.
 *
 * @param lvdsFirstSyncComplete Indicates whether the first synchronization of
 * the Rx LVDS divider is complete.
 * @param lvdsSecondSyncComplete Indicates whether the second synchronization of
 * the Rx LVDS divider is complete.
 ******************************************************************************/
typedef struct adi_adrv9001_RxLvdsSyncStatus {
    bool lvdsFirstSyncComplete;   /*!< Rx LVDS divider first synchronization complete */
    bool lvdsSecondSyncComplete;  /*!< Rx LVDS divider second synchronization complete */
} adi_adrv9001_RxLvdsSyncStatus_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_McsStatus_t` structure is used to represent the
 * synchronization status of various components in the ADRV9001 system,
 * including RF PLLs, clock PLLs, and LVDS synchronization. It contains
 * fields for the synchronization status of RF1 and RF2 PLLs, clock PLLs,
 * and low-power clock PLLs, as well as the synchronization status of Rx1
 * and Rx2 LVDS. Additionally, it includes boolean flags to indicate the
 * completion of digital synchronization processes and fields to
 * represent the phase difference between local oscillators and the MCS
 * reference point, with data types varying based on the compilation
 * environment.
 *
 * @param rf1PllSyncStatus RF1 PLL synchronization status.
 * @param rf2PllSyncStatus RF2 PLL synchronization status.
 * @param clkPllSyncStatus CLK PLL synchronization status.
 * @param clkPllLpSyncStatus LP CLK PLL synchronization status.
 * @param rx1LvdsSyncStatus Rx1 digital LVDS synchronization status.
 * @param rx2LvdsSyncStatus Rx2 digital LVDS synchronization status.
 * @param firstDigitalSyncComplete Indicates if the first digital
 * synchronization is complete.
 * @param secondDigitalSyncComplete Indicates if the second digital
 * synchronization is complete.
 * @param rfPll1Phase_degrees The phase difference between LO1 and MCS reference
 * point in degrees.
 * @param rfPll2Phase_degrees The phase difference between LO2 and MCS reference
 * point in degrees.
 ******************************************************************************/
typedef struct adi_adrv9001_McsStatus {
    adi_adrv9001_PllSyncStatus_t rf1PllSyncStatus;     /*!< RF1 PLL synchronization status */
    adi_adrv9001_PllSyncStatus_t rf2PllSyncStatus;     /*!< RF1 PLL synchronization status */
    adi_adrv9001_PllSyncStatus_t clkPllSyncStatus;     /*!< CLK PLL synchronization status */
    adi_adrv9001_PllSyncStatus_t clkPllLpSyncStatus;   /*!< LP CLK PLL synchronization status */

    adi_adrv9001_RxLvdsSyncStatus_t rx1LvdsSyncStatus; /*!< Rx1 digital LVDS synchronization status */
    adi_adrv9001_RxLvdsSyncStatus_t rx2LvdsSyncStatus; /*!< Rx2 digital LVDS synchronization status */

    bool firstDigitalSyncComplete;  /*!< Digital synchronization status */
    bool secondDigitalSyncComplete; /*!< Digital synchronization status */
#ifdef __KERNEL__
    int32_t rfPll1Phase_degrees;    /*!< The phase difference between LO1 and MCS reference point in degrees */
    int32_t rfPll2Phase_degrees;    /*!< The phase difference between LO2 and MCS reference point in degrees */
#else
    float rfPll1Phase_degrees;      /*!< The phase difference between LO1 and MCS reference point in degrees */
    float rfPll2Phase_degrees;      /*!< The phase difference between LO2 and MCS reference point in degrees */
#endif
} adi_adrv9001_McsStatus_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_McsDelay_t` structure is used to define delay
 * parameters for the ADRV9001 Multi-Chip Synchronization (MCS) process.
 * It contains two members: `readDelay`, which specifies the delay for
 * the SSI FIFO read pointer, and `sampleDelay`, which indicates the
 * delay in terms of samples. This structure is crucial for managing
 * timing and synchronization in multi-chip setups, ensuring that data is
 * read and processed at the correct intervals.
 *
 * @param readDelay ADRV9001 SSI FIFO read pointer delay.
 * @param sampleDelay Delay specified in samples.
 ******************************************************************************/
typedef struct adi_adrv9001_McsDelay
{
    uint8_t  readDelay;     /*!< ADRV9001 SSI FIFO read pointer delay */
    uint16_t sampleDelay;   /*!< Delay specified in samples */
} adi_adrv9001_McsDelay_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_MCS_TYPES_H_ */
