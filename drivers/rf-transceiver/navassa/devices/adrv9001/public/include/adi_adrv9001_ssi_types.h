/*!
* \file
* \brief Contains ADRV9001 API SSI configuration and data type definitions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_SSI_TYPES_H_
#define _ADI_ADRV9001_SSI_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_common.h"
#include "adi_adrv9001_rx_types.h"
#include "adi_adrv9001_tx_types.h"

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiTestModeData_e` is an enumeration that defines
 * various types of data that can be transmitted over the SSI (Serial
 * Synchronous Interface) in the ADRV9001 system. Each enumerator
 * represents a specific test mode data type, such as normal data, fixed
 * pattern, ramp nibble, ramp 16-bit, PRBS15, and PRBS7, with some modes
 * being specific to either CSSI or LSSI configurations. This enumeration
 * is used to configure the type of test data transmitted or received
 * over the SSI interface, aiding in testing and validation of the data
 * transmission process.
 *
 * @param ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL No check available for this data
 * type.
 * @param ADI_ADRV9001_SSI_TESTMODE_DATA_FIXED_PATTERN Length of pattern is
 * dependent on SSI
 * configuration, refer to '
 * fixedDataPatternToTransmi
 * t' in adi_adrv9001_RxSsiT
 * estModeCfg_t for details.
 * @param ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE CSSI-Only.
 * @param ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_16_BIT CSSI and LSSI.
 * @param ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15 LSSI-Only.
 * @param ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7 LSSI-Only.
 ******************************************************************************/
typedef enum adi_adrv9001_SsiTestModeData
{
    ADI_ADRV9001_SSI_TESTMODE_DATA_NORMAL = 0,      /*!< No check available for this data type */
    ADI_ADRV9001_SSI_TESTMODE_DATA_FIXED_PATTERN,   /*!< Length of pattern is dependent on SSI configuration.
                                                         Refer to 'fixedDataPatternToTransmit' in adi_adrv9001_RxSsiTestModeCfg_t for details */
    ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_NIBBLE,     /*!< CSSI-Only */
    ADI_ADRV9001_SSI_TESTMODE_DATA_RAMP_16_BIT,     /*!< CSSI and LSSI */
    ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS15,          /*!< LSSI-Only */
    ADI_ADRV9001_SSI_TESTMODE_DATA_PRBS7,           /*!< LSSI-Only */
} adi_adrv9001_SsiTestModeData_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiPowerDown_e` is an enumeration that defines the
 * power down modes for the SSI (Serial Synchronous Interface) in LVDS
 * mode for the ADRV9001 device. It provides three levels of power down
 * states: 'DISABLED' where all SSI pads are powered up, 'MEDIUM' where
 * only RX_CLK and TX_REF_CLK pads are powered up while others are
 * powered down, and 'HIGH' where all SSI pads are powered down. This
 * allows for efficient power management by controlling the power state
 * of the SSI pads based on the operational requirements.
 *
 * @param ADI_ADRV9001_SSI_POWER_DOWN_DISABLED All SSI PADS powered up in
 * PRIMED.
 * @param ADI_ADRV9001_SSI_POWER_DOWN_MEDIUM RX_CLK and TX_REF_CLK SSI pads
 * powered up, TX_CLK and all STROBE
 * and DATA SSI pads powered down in
 * PRIMED.
 * @param ADI_ADRV9001_SSI_POWER_DOWN_HIGH All SSI pads powered down in PRIMED.
 ******************************************************************************/
typedef enum adi_adrv9001_SsiPowerDown
{
    ADI_ADRV9001_SSI_POWER_DOWN_DISABLED = 0, /*!< All SSI PADS powered up in PRIMED */
    ADI_ADRV9001_SSI_POWER_DOWN_MEDIUM = 1,   /*!< RX_CLK and TX_REF_CLK SSI pads powered up, 
                                                   TX_CLK and all STROBE and DATA SSI pads powered down in PRIMED */
    ADI_ADRV9001_SSI_POWER_DOWN_HIGH = 2,     /*!< All SSI pads powered down in PRIMED */
}adi_adrv9001_SsiPowerDown_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiRxGainSelect_e` is an enumeration that defines
 * the different modes for selecting the gain index and gain change bits
 * to be passed via the 32-bit Rx SSI interface in the ADRV9001 device.
 * It provides options to either pass zeroes, use the Rx Interface gain,
 * or utilize the Rx Automated Gain Control (AGC) for these bits,
 * allowing for flexible configuration of the gain control mechanism in
 * the device's signal processing chain.
 *
 * @param ADI_ADRV9001_RX_GAIN_INDEX_GAIN_CHANGE_ZEROES ADRV9001 passes zeroes
 * to gainIndex and
 * gainChange bits in
 * 32-bit Rx SSI.
 * @param ADI_ADRV9001_RX_GAIN_INDEX_GAIN_CHANGE_INTERFACE_GAIN ADRV9001 passes
 * Rx Interface
 * gain and
 * gainChange to
 * gainIndex and
 * gainChange bits
 * in 32-bit Rx
 * SSI.
 * @param ADI_ADRV9001_RX_GAIN_INDEX_GAIN_CHANGE_AGC ADRV9001 passes
 * RxAutomatedGainControl
 * gainIndex and gainChange to
 * gainIndex and gainChange
 * bits in 32-bit Rx SSI.
 ******************************************************************************/
typedef enum adi_adrv9001_SsiRxGainSelect
{
    ADI_ADRV9001_RX_GAIN_INDEX_GAIN_CHANGE_ZEROES = 0,         /*!< ADRV9001 passes zeroes to gainIndex and gainChange bits in 32-bit Rx SSI */
    ADI_ADRV9001_RX_GAIN_INDEX_GAIN_CHANGE_INTERFACE_GAIN = 1, /*!< ADRV9001 passes Rx Interface gain and gainChange to 
                                                                    gainIndex and gainChange bits in 32-bit Rx SSI */
    ADI_ADRV9001_RX_GAIN_INDEX_GAIN_CHANGE_AGC = 2,            /*!< ADRV9001 passes RxAutomatedGainControl gainIndex and gainChange to 
                                                                    gainIndex and gainChange bits in 32-bit Rx SSI */
} adi_adrv9001_SsiRxGainSelect_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxSsiTestModeCfg_t` structure is used to configure
 * the test mode for the SSI (Serial Synchronous Interface) on the Rx
 * channel of the ADRV9001 device. It includes a member to specify the
 * type of test data to be transmitted and another member to define a
 * fixed data pattern for transmission. The fixed data pattern is subject
 * to truncation based on the SSI data format, either CMOS or LVDS,
 * ensuring compatibility with the interface's requirements.
 *
 * @param testData Type of data to transmit over SSI.
 * @param fixedDataPatternToTransmit Value of fixed pattern to transmit over the
 * interface, with specific truncation for
 * CMOS and LVDS formats.
 ******************************************************************************/
typedef struct adi_adrv9001_RxSsiTestModeCfg
{
    adi_adrv9001_SsiTestModeData_e testData;   /*!< Type of data to transmit over SSI */
    uint32_t fixedDataPatternToTransmit;       /*!< Value of Fixed pattern to transmit over interface. For various SSI data format:
                                                CMOS: Pattern is truncated to bit3  - bit0 value is transmitted on RxSSI I and Q (where applicable)
                                                LVDS: Pattern is truncated to bit15 - bit0 value transmitted on RxSSI I and Q (where applicable) */
} adi_adrv9001_RxSsiTestModeCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxSsiTestModeCfg_t` structure is used to configure
 * the test mode for the ADRV9001's SSI (Serial Synchronous Interface) on
 * the transmit channel. It specifies the type of test data expected to
 * be received over the SSI and a fixed data pattern that should be
 * checked against the received pattern. This configuration is crucial
 * for ensuring data integrity and proper operation of the SSI in test
 * scenarios.
 *
 * @param testData Type of data to receive over SSI and check.
 * @param fixedDataPatternToCheck Value of Fixed pattern to check against
 * pattern received over interface.
 ******************************************************************************/
typedef struct adi_adrv9001_TxSsiTestModeCfg
{
    adi_adrv9001_SsiTestModeData_e testData;   /*!< Type of data to receive over SSI and check */
    uint32_t fixedDataPatternToCheck;          /*!< Value of Fixed pattern to check against pattern recevied over interface */
} adi_adrv9001_TxSsiTestModeCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxSsiTestModeStatus_t` structure is used to hold the
 * status of the ADRV9001 SSI test mode for the transmit channel. It
 * contains fields that indicate various error and status conditions such
 * as data errors, FIFO status, and clock alignment issues, which are
 * crucial for diagnosing and ensuring the proper functioning of the SSI
 * interface in test mode.
 *
 * @param dataError Non-zero if there is an error from Fixed Pattern, Ramp, or
 * PRBS checker.
 * @param fifoFull Non-zero if the FIFO is full.
 * @param fifoEmpty Non-zero if the FIFO is empty.
 * @param strobeAlignError Indicates a mismatch in external and internal clock
 * alignment.
 ******************************************************************************/
typedef struct adi_adrv9001_TxSsiTestModeStatus
{
    uint8_t dataError;                       /*!< non zero if error from Fixed Pattern, Ramp or PRBS checker */
    uint8_t fifoFull;                        /*!< non-zero if FIFO is full */
    uint8_t fifoEmpty;                       /*!< non-zero if FIFO is empty */
    uint8_t strobeAlignError;                /*!< Mismatch in external and internal clock */
} adi_adrv9001_TxSsiTestModeStatus_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiCalibrationCfg_t` structure is used to configure
 * the delay calibration settings for the ADRV9001's CMOS interface for
 * both receive (Rx) and transmit (Tx) channels. It contains arrays for
 * various delay parameters, such as clock, strobe, and data delays, for
 * both Rx and Tx channels. These settings are crucial for ensuring
 * proper timing alignment and signal integrity in the communication
 * between the ADRV9001 and other components in the system.
 *
 * @param rxClkDelay CLK delay for Rx1/Rx2 channels.
 * @param rxStrobeDelay Strobe delay for Rx1/Rx2 channels.
 * @param rxIDataDelay I data delay for Rx1/Rx2 channels.
 * @param rxQDataDelay Q data delay for Rx1/Rx2 channels.
 * @param txClkDelay CLK delay for Tx1/Tx2 channels.
 * @param txRefClkDelay Ref CLK delay for Tx1/Tx2 channels.
 * @param txStrobeDelay Strobe delay for Tx1/Tx2 channels.
 * @param txIDataDelay I data delay for Tx1/Tx2 channels.
 * @param txQDataDelay Q data delay for Tx1/Tx2 channels.
 ******************************************************************************/
typedef struct adi_adrv9001_SsiCalibrationCfg
{
    uint8_t rxClkDelay[ADI_ADRV9001_MAX_RX_ONLY];       /*!< CLK delay for Rx1/Rx2 channels */
    uint8_t rxStrobeDelay[ADI_ADRV9001_MAX_RX_ONLY];    /*!< Strobe delay for Rx1/Rx2 channels */
    uint8_t rxIDataDelay[ADI_ADRV9001_MAX_RX_ONLY];     /*!< I data delay for Rx1/Rx2 channels */
    uint8_t rxQDataDelay[ADI_ADRV9001_MAX_RX_ONLY];     /*!< Q data delay for Rx1/Rx2 channels */

    uint8_t txClkDelay[ADI_ADRV9001_MAX_TXCHANNELS];    /*!< CLK delay for Tx1/Tx2 channels */
    uint8_t txRefClkDelay[ADI_ADRV9001_MAX_TXCHANNELS]; /*!< Ref CLK delay for Tx1/Tx2 channels */
    uint8_t txStrobeDelay[ADI_ADRV9001_MAX_TXCHANNELS]; /*!< Strobe delay for Tx1/Tx2 channels */
    uint8_t txIDataDelay[ADI_ADRV9001_MAX_TXCHANNELS];  /*!< I data delay for Tx1/Tx2 channels */
    uint8_t txQDataDelay[ADI_ADRV9001_MAX_TXCHANNELS];  /*!< Q data delay for Tx1/Tx2 channels */

} adi_adrv9001_SsiCalibrationCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiRxGainSelectCfg_t` structure is used to configure
 * the gain selection for the SSI (Serial Synchronous Interface) in the
 * ADRV9001 device. It allows the user to specify which gain index and
 * gain change bits are passed through the 32-bit RxSSI interface. The
 * structure includes parameters for controlling the delay of the
 * Automated Gain Control (AGC) and the interface gain index FIFO,
 * providing flexibility in managing the timing of gain adjustments in
 * the receiver path.
 *
 * @param gainSelect Control of which gainIndex and gainChange bits to pass via
 * 32-bit-RxSSI.
 * @param agcGainDelay Automated Gain Control gainIndex FIFO Delay; Range: 0 ->
 * 255 Rx Sample Periods.
 * @param interfaceGainDelay Interface gainIndex FIFO Delay; Range: 0 -> 31 Rx
 * Sample Periods.
 ******************************************************************************/
typedef struct adi_adrv9001_SsiRxGainSelectCfg
{
    adi_adrv9001_SsiRxGainSelect_e gainSelect;   /*!< Control of which gainIndex and gainChange bits to pass via 32-bit-RxSSI */
    uint8_t agcGainDelay;                        /*!< Automated Gain Control gainIndex FIFO Delay; Range : 0 -> 255 Rx Sample Periods */
    uint8_t interfaceGainDelay;                  /*!< Interface gainIndex FIFO Delay; Range : 0 -> 31 Rx Sample Periods */
} adi_adrv9001_SsiRxGainSelectCfg_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_SSI_TYPES_H_ */
