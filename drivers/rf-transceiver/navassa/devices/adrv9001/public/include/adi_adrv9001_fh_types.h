/**
 * \file
 * \brief Contains ADRV9001 Frequency Hopping data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2020 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_FH_TYPES_H_
#define _ADI_ADRV9001_FH_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdint.h>
#endif
#include "adi_adrv9001_gpio_types.h"

/**
 * \brief Max number of GPIO pins for frequency selection. 
 */
#define ADI_ADRV9001_FH_MAX_NUM_FREQ_SELECT_PINS 6u /* 6 pins is enough to index all 64 frequencies in table */

/**
 * \brief Max number of hop tables.
 */
#define ADI_ADRV9001_FH_MAX_HOP_TABLE_SIZE 64u

/**
 * \brief Max number of GPIO pins for gain selection.
 */
#define  ADI_ADRV9001_FH_MAX_NUM_GAIN_SELECT_PINS 3u

/**
 * \brief Max number of entries in Rx gain and Tx attenuation tables.
 */
#define  ADI_ADRV9001_FH_MAX_NUM_GAIN_SELECT_ENTRIES 8u

/**
 * \brief Minimum carrier frequency supported in frequency hopping mode
 */
#define ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ 25000000llu    /* 25 MHz */

/**
 * \brief Maximum carrier frequency supported in frequency hopping mode
 */
#define ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ 6000000000llu  /* 6 GHz */

/**
 * \brief Maximum Tx analog front end delay in hop frames
 */
#define ADI_ADRV9001_FH_MAX_TX_FE_POWERON_FRAME_DELAY 64u

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhMode_e` is an enumeration that defines the
 * different modes of frequency hopping available in the ADRV9001 system.
 * Each mode specifies a unique method of handling frequency hopping,
 * including whether the local oscillator (LO) is muxed or retuned, and
 * whether the frequency hopping table is processed before or during the
 * operation. The modes also include options for dual-hop control,
 * allowing independent management of two channels.
 *
 * @param ADI_ADRV9001_FHMODE_LO_MUX_PREPROCESS Frequency hopping with LO
 * muxing, where the frequency
 * hopping table is processed
 * before the operation.
 * @param ADI_ADRV9001_FHMODE_LO_MUX_REALTIME_PROCESS Frequency hopping with LO
 * muxing, where the
 * frequency hopping table is
 * processed during the
 * operation.
 * @param ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS Frequency hopping with
 * LO retuning during
 * transition time, with
 * the table processed
 * during the operation.
 * @param ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP Two HOP
 * signals
 * control
 * channels
 * independently,
 * with LO
 * retuning
 * during
 * transition and
 * table
 * processing
 * during the
 * operation.
 ******************************************************************************/
typedef enum {
    ADI_ADRV9001_FHMODE_LO_MUX_PREPROCESS          = 0u,   /*!< Frequency hopping with LO muxing. Frequency hopping table is processed before the frequency hopping operation. */
    ADI_ADRV9001_FHMODE_LO_MUX_REALTIME_PROCESS    = 1u,   /*!< Frequency hopping with LO muxing. Frequency hopping table is processed during frequency hopping operation. */
    ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS = 2u,   /*!< Frequency hopping with LO retuning during transition time. Frequency hopping table is processed during frequency hopping operation. */
    ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP = 3u, /*!< 2 HOP signals are used to control Channel 1 and Channel 2 independently. Frequency hopping with LO retuning during transition time.
                                                                       Frequency hopping table is processed during frequency hopping operation. */
} adi_adrv9001_FhMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TableIndexCtrl_e` is an enumeration that defines the
 * control mechanisms for indexing through frequency hopping tables in
 * the ADRV9001 system. It provides options for automatic looping through
 * a table, a ping-pong mechanism between two tables, and GPIO-based
 * indexing, allowing for flexible control of frequency hopping
 * operations.
 *
 * @param ADI_ADRV9001_TABLEINDEXCTRL_AUTO_LOOP Automatically increment through
 * a frequency hopping table and
 * wrap around once the end has
 * been reached.
 * @param ADI_ADRV9001_TABLEINDEXCTRL_AUTO_PING_PONG Ping pong operation between
 * two frequency hopping
 * tables, automatically
 * switching once the end of
 * one table is reached.
 * @param ADI_ADRV9001_TABLEINDEXCTRL_GPIO Use GPIO pins to index entries within
 * a frequency hopping table.
 ******************************************************************************/
typedef enum {
    ADI_ADRV9001_TABLEINDEXCTRL_AUTO_LOOP      = 0u, /*!< Automatically increment through a frequency hopping table and wrap around once end has been reached */
    ADI_ADRV9001_TABLEINDEXCTRL_AUTO_PING_PONG = 1u, /*!< Ping pong operation between ADI_ADRV9001_FHHOPTABLE_A and ADI_ADRV9001_FHHOPTABLE_B. 
                                                          Automatically increment through one frequency hopping table and switch to the other 
                                                          once end has been reached */
    ADI_ADRV9001_TABLEINDEXCTRL_GPIO           = 2u, /*!< Use GPIO pins to index entries within a frequency hopping table */
} adi_adrv9001_TableIndexCtrl_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhHopTable_e` is an enumeration that defines
 * identifiers for frequency hopping tables used in the ADRV9001 device.
 * It provides two constants, `ADI_ADRV9001_FHHOPTABLE_A` and
 * `ADI_ADRV9001_FHHOPTABLE_B`, which are used to reference two distinct
 * hop tables stored in ARM memory. This enumeration is part of the
 * frequency hopping configuration, allowing the system to select between
 * different pre-defined frequency hopping tables.
 *
 * @param ADI_ADRV9001_FHHOPTABLE_A ID for hop table A in arm memory.
 * @param ADI_ADRV9001_FHHOPTABLE_B ID for hop table B in arm memory.
 ******************************************************************************/
typedef enum {
    ADI_ADRV9001_FHHOPTABLE_A = 0u, /*!< ID for hop table A in arm memory */
    ADI_ADRV9001_FHHOPTABLE_B = 1u, /*!< ID for hop table B in arm memory */
} adi_adrv9001_FhHopTable_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhFrameIndex_e` is an enumeration that defines
 * indices for hop frames in the context of frequency hopping operations.
 * It distinguishes between the current hop frame in progress and the
 * upcoming hop frame, which is relevant in specific frequency hopping
 * modes such as LO_MUX. This enumeration is part of the ADRV9001 API,
 * which is used for configuring and managing frequency hopping in radio
 * frequency applications.
 *
 * @param ADI_ADRV9001_FHFRAMEINDEX_CURRENT_FRAME Hop frame currently in
 * progress.
 * @param ADI_ADRV9001_FHFRAMEINDEX_UPCOMING_FRAME The upcoming hop frame after
 * the current, applicable only
 * in LO_MUX mode.
 ******************************************************************************/
typedef enum {
    ADI_ADRV9001_FHFRAMEINDEX_CURRENT_FRAME  = 0u, /*!< Hop frame currently in progress */
    ADI_ADRV9001_FHFRAMEINDEX_UPCOMING_FRAME = 1u, /*!< The upcoming hop frame after the current. 
                                                        Only applicable in LO_MUX, will match CURRENT in LO_RETUNE*/
} adi_adrv9001_FhFrameIndex_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhPerDynamicLoad_e` is an enumeration that defines
 * the number of frequency hops per dynamic load operation in the
 * ADRV9001 frequency hopping system. It is used to specify how many hops
 * should occur for each dynamic load, with options for 1, 4, or 8 hops.
 * This enumeration is particularly relevant in the context of FPGA
 * operations within the ADI evaluation system, although it is not
 * necessary for customer applications.
 *
 * @param ADI_ADRV9001_FH_HOP_PER_DYNAMIC_LOAD_ONE Represents 1 HOP per Dynamic
 * Load.
 * @param ADI_ADRV9001_FH_HOP_PER_DYNAMIC_LOAD_FOUR Represents 4 HOPs per
 * Dynamic Load.
 * @param ADI_ADRV9001_FH_HOP_PER_DYNAMIC_LOAD_EIGHT Represents 8 HOPs per
 * Dynamic Load.
 ******************************************************************************/
typedef enum adi_adrv9001_FhPerDynamicLoad
{
    ADI_ADRV9001_FH_HOP_PER_DYNAMIC_LOAD_ONE = 0,    /*!< 1 HOP per Dynamic Load */
    ADI_ADRV9001_FH_HOP_PER_DYNAMIC_LOAD_FOUR = 1,   /*!< 4 HOP per Dynamic Load */
    ADI_ADRV9001_FH_HOP_PER_DYNAMIC_LOAD_EIGHT = 2   /*!< 8 HOP per Dynamic Load */
} adi_adrv9001_FhPerDynamicLoad_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhHopSignal_e` is an enumeration that defines the
 * possible hop signals used in the ADRV9001 frequency hopping
 * configuration. It includes two members, `ADI_ADRV9001_FH_HOP_SIGNAL_1`
 * and `ADI_ADRV9001_FH_HOP_SIGNAL_2`, which represent different hop
 * signals that can be used to control frequency hopping operations. This
 * enumeration is part of the broader frequency hopping setup, allowing
 * for the selection and control of hop signals in the ADRV9001 system.
 *
 * @param ADI_ADRV9001_FH_HOP_SIGNAL_1 Hop 1 signal.
 * @param ADI_ADRV9001_FH_HOP_SIGNAL_2 Hop 2 signal.
 ******************************************************************************/
typedef enum adi_adrv9001_FhHopSignal
{
    ADI_ADRV9001_FH_HOP_SIGNAL_1 = 0, /*!< Hop 1 signal */
    ADI_ADRV9001_FH_HOP_SIGNAL_2 = 1  /*!< Hop 2 signal */
} adi_adrv9001_FhHopSignal_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhHopTableSelectMode_e` is an enumeration that
 * defines the modes for selecting frequency hopping tables in the
 * ADRV9001 device. It provides two modes: 'INDEPENDENT', where each GPIO
 * pin controls its respective channel, and 'COMMON', where a single GPIO
 * pin can control both channels simultaneously. This enumeration is used
 * to configure how the frequency hopping table selection is managed in
 * the device, allowing for flexible control based on the application
 * requirements.
 *
 * @param ADI_ADRV9001_FHHOPTABLESELECTMODE_INDEPENDENT Each assigned GPIO pin
 * controls the
 * corresponding channel.
 * @param ADI_ADRV9001_FHHOPTABLESELECTMODE_COMMON hopTableSelectGpioConfig[0]
 * is used to control both
 * channels, where applicable.
 ******************************************************************************/
typedef enum {
    ADI_ADRV9001_FHHOPTABLESELECTMODE_INDEPENDENT = 0u,     /*!< Each assigned GPIO pin controls the corresponding channel */
    ADI_ADRV9001_FHHOPTABLESELECTMODE_COMMON = 1u,          /*!< hopTableSelectGpioConfig[0] is used to control both channels, where applicable */
} adi_adrv9001_FhHopTableSelectMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhHopFrame_t` structure is used to define the
 * settings for a frequency hopping frame in the ADRV9001 system. It
 * includes fields for specifying offset frequencies for Rx1 and Rx2,
 * attenuation levels for Tx1 and Tx2, gain indices for Rx1 and Rx2, and
 * the operating frequency. The structure is designed to be flexible,
 * with certain fields being ignored depending on whether the frame is
 * designated for transmission (Tx) or reception (Rx). This allows for
 * efficient configuration of frequency hopping parameters in dynamic
 * radio environments.
 *
 * @param rx1OffsetFrequencyHz Rx1 Offset frequency, ignored by firmware if
 * frame is not Rx.
 * @param rx2OffsetFrequencyHz Rx2 Offset frequency, ignored by firmware if
 * frame is not Rx.
 * @param tx1Attenuation_fifthdB Tx1 attenuation in 0.2dBs, ignored if frame is
 * Rx.
 * @param tx2Attenuation_fifthdB Tx2 attenuation in 0.2dBs, ignored if frame is
 * Rx.
 * @param rx1GainIndex Starting Rx1 gain index for hop frame, ignored if frame
 * is Tx.
 * @param rx2GainIndex Starting Rx2 gain index for hop frame, ignored if frame
 * is Tx.
 * @param hopFrequencyHz Operating frequency in Hz.
 ******************************************************************************/
typedef struct {
    int32_t  rx1OffsetFrequencyHz;   /*!< Rx1 Offset frequency. This field is ignored by firmware if frame is not Rx */
    int32_t  rx2OffsetFrequencyHz;   /*!< Rx2 Offset frequency. This field is ignored by firmware if frame is not Rx */
    uint8_t tx1Attenuation_fifthdB;  /*!< Tx1 attenuation expressed in 0.2dBs. LSB = 0.2dB. Range 0 to 209 (0 to 41.8 dB). This field is ignored if frame is Rx */
	uint8_t tx2Attenuation_fifthdB;  /*!< Tx2 attenuation expressed in 0.2dBs. Range 0 to 209 (0 to 41.8 dB). This field is ignored if frame is Rx */
    uint8_t  rx1GainIndex;           /*!< Starting Rx1 gain index for hop frame. This field is ignored if frame is Tx */
	uint8_t  rx2GainIndex;           /*!< Starting Rx2 gain index for hop frame. This field is ignored if frame is Tx */
    uint64_t hopFrequencyHz; /*!< Operating frequency in Hz */
} adi_adrv9001_FhHopFrame_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhGainSetupByPinCfg_t` structure is used to
 * configure the gain setup for frequency hopping operations by utilizing
 * GPIO pins. It includes information about the number of entries and the
 * actual tables for Rx gain and Tx attenuation, as well as the
 * configuration of GPIO pins used for gain control. This structure is
 * particularly useful when gain levels need to be dynamically adjusted
 * during frequency hopping, allowing for up to 8 Rx gains and 8 Tx
 * attenuation levels to be indexed by up to 3 GPIO pins. The
 * configuration is ignored if the gainSetupByPin option is not enabled.
 *
 * @param numRxGainTableEntries Number of gain levels in the Rx gain table,
 * ignored if gainSetupByPin is false.
 * @param rxGainTable Array representing the Rx gain table used in frequency
 * hopping, ignored if gainSetupByPin is false.
 * @param numTxAttenTableEntries Number of attenuation levels in the Tx
 * attenuation table, ignored if gainSetupByPin is
 * false.
 * @param txAttenTable Array representing the Tx attenuation table used in
 * frequency hopping, ignored if gainSetupByPin is false.
 * @param numGainCtrlPins Number of pins assigned for gain control, ignored if
 * gainSetupByPin is false.
 * @param gainSelectGpioConfig Array of pin configurations for gain select,
 * ignored if gainSetupByPin is false.
 ******************************************************************************/
typedef struct { 
    /* Rx gain table information */
    uint8_t                          numRxGainTableEntries;         /*!< Number of gain levels in Rx gain table. Ignored if gainSetupByPin is false. */
    uint8_t                          rxGainTable[ADI_ADRV9001_FH_MAX_NUM_GAIN_SELECT_ENTRIES];  /*!< Pointer to Rx gain table used in frequency hopping. Ignored if gainSetupByPin is false. */
    /* Tx attenuation table information */
    uint8_t                          numTxAttenTableEntries;        /*!< Number of attenuation levels in Tx attenuation table. Ignored if gainSetupByPin is false. */
    uint16_t                         txAttenTable[ADI_ADRV9001_FH_MAX_NUM_GAIN_SELECT_ENTRIES]; /*!< Pointer to Tx attenuation table used in frequency hopping. Ignored if gainSetupByPin is false. */

    /* GPIO configuration */
    uint8_t                          numGainCtrlPins;               /*!< Number of pins assigned for gain control. Ignored if gainSetupByPin in adi_adrv9001_FhCfg_t is false. */
    adi_adrv9001_GpioCfg_t           gainSelectGpioConfig[ADI_ADRV9001_FH_MAX_NUM_GAIN_SELECT_PINS];  /*!< Pin configuration for gain select. Ignored if gainSetupByPin is false. */
} adi_adrv9001_FhGainSetupByPinCfg_t; 

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhhopTableSelectCfg_t` structure is used to
 * configure the selection mode and GPIO pin configuration for frequency
 * hopping table selection in the ADRV9001 device. It allows the user to
 * specify whether the hop table selection is controlled independently
 * for each channel or commonly for both channels, and provides the GPIO
 * configuration for the selection pins. This configuration is crucial
 * for setting up the frequency hopping operation in the device, ensuring
 * that the correct hop table is selected based on the desired control
 * mode.
 *
 * @param hopTableSelectMode Mode of control for hop table select, either
 * independent or common pin control.
 * @param hopTableSelectGpioConfig Array of pin configurations for hop table
 * select, ignored if unassigned.
 ******************************************************************************/
typedef struct {
    adi_adrv9001_FhHopTableSelectMode_e     hopTableSelectMode;             /*!< Mode of control for hop table select. Independent or common pin contorl */
    adi_adrv9001_GpioCfg_t                  hopTableSelectGpioConfig[2u];   /*!< Pin configuration for hop table select. Not an error if unassigned, just ignored.*/
} adi_adrv9001_FhhopTableSelectCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_FhCfg_t` structure is a comprehensive configuration
 * data structure for managing frequency hopping operations in the
 * ADRV9001 system. It includes settings for frequency hopping modes, hop
 * signals for RF ports, zero-IF operation configuration, GPIO pin
 * configurations for hop signals and table index selection, and control
 * types for frequency hopping tables. Additionally, it specifies the
 * range for Rx gain indices, Tx attenuation levels, and operating
 * frequencies, as well as frame duration and delay settings. The
 * structure also supports gain setup by pin and automatic gain control
 * (AGC) gain index seeding, providing flexibility and precision in
 * frequency hopping operations.
 *
 * @param mode Frequency hopping mode.
 * @param rxPortHopSignals Hop signals for each RF port for reception.
 * @param txPortHopSignals Hop signals for each RF port for transmission.
 * @param rxZeroIfEnable Configures the Rx ports for zero-IF operation.
 * @param hopSignalGpioConfig Pin configuration for HOP1 and HOP2 signals.
 * @param hopTableSelectConfig Configuration for hop table select signal.
 * @param tableIndexCtrl Control type for frequency hopping table selection.
 * @param minRxGainIndex Minimum Rx gain index for frequency hopping operation.
 * @param maxRxGainIndex Maximum Rx gain index for frequency hopping operation.
 * @param minTxAtten_mdB Minimum Tx attenuation level for frequency hopping
 * operation.
 * @param maxTxAtten_mdB Maximum Tx attenuation level for frequency hopping
 * operation.
 * @param minOperatingFrequency_Hz Minimum operating frequency for frequency
 * hopping operation.
 * @param maxOperatingFrequency_Hz Maximum operating frequency for frequency
 * hopping operation.
 * @param minFrameDuration_us Minimum frame duration supported during frequency
 * hopping operation.
 * @param txAnalogPowerOnFrameDelay Delay for Tx analog front end relative to
 * the first Tx setup rising edge.
 * @param numTableIndexPins Number of pins for table indexing.
 * @param tableIndexGpioConfig Pin configuration for table index selection.
 * @param gainSetupByPin Indicates if GPIO pins are used for Tx/Rx gain index
 * for the next hop frame.
 * @param gainSetupByPinConfig Configuration for gain setup by pin.
 * @param enableAGCGainIndexSeeding Enables seeding of the hardware value for
 * gain index with rxGainIndex from FHTable.
 ******************************************************************************/
typedef struct {
    adi_adrv9001_FhMode_e              mode;                        /*!< Frequency hopping mode */
    adi_adrv9001_FhHopSignal_e         rxPortHopSignals[ADI_ADRV9001_NUM_CHANNELS]; /*!< Hop Signals for each RF port [0] = Rx1, [1] = Rx2 */
    adi_adrv9001_FhHopSignal_e         txPortHopSignals[ADI_ADRV9001_NUM_CHANNELS]; /*!< Hop Signals for each RF port [0] = Tx1, [1] = Tx2 */
    bool                               rxZeroIfEnable;              /*!< Configure the Rx ports for zero-IF operation. 
                                                                         FALSE:  LO = hopFrequencyHz + rx(n)OffsetFrequencyHz.  
                                                                         TRUE:   LO = hopFrequencyHz */
    adi_adrv9001_GpioCfg_t             hopSignalGpioConfig[2u];     /*!< Pin configuration for HOP1 and HOP2 signals. If GPIO is unassigned, HOP signal source is defaulted to SPI mode*/                      
    adi_adrv9001_FhhopTableSelectCfg_t hopTableSelectConfig;        /*!< Hop table select signal configuration */ 
    adi_adrv9001_TableIndexCtrl_e      tableIndexCtrl;              /*!< Select control type for frequency hopping table */
    uint8_t                            minRxGainIndex;              /*!< Minimum Rx gain index for FH operation. Used for calibration over specified gain range. Valid range is from 0 to maxGainIndex  */
    uint8_t                            maxRxGainIndex;              /*!< Maximum Rx gain index for FH operation. Used for calibration over specified gain range. Valid range is from minGainIndex to 255 */
    uint16_t                           minTxAtten_mdB;              /*!< Minimum Tx attenuation level for FH operation. Used for calibration over specified attenuation range. Valid range is from 0 to maxTxAtten_mdB */
    uint16_t                           maxTxAtten_mdB;              /*!< Maximum Tx attenuation level for FH operation. Used for calibration over specified attenuation range. Valid range is from minTxAtten_mdB to 41950 */
    uint64_t                           minOperatingFrequency_Hz;    /*!< Minimum frequency used during FH operation. Used for calibration. Valid range is from 30Mhz to maxOperatingFrequency_Hz */
    uint64_t                           maxOperatingFrequency_Hz;    /*!< Maximum frequency used during FH operation. Used for calibration. Valid range is from minOperatingFrequency_Hz to 3.2GHz */
    uint32_t                           minFrameDuration_us;         /*!< Minimum frame duration to be supported during FH operation, in microseconds. */
    uint8_t                            txAnalogPowerOnFrameDelay;   /*!< A delay specified in terms of hop frames. Tx analog front end can be delayed relative to the first Tx setup rising edge. 
                                                                        Set this field to greater than 0 to support use cases where Tx propagation delay is longer than a single hop frame duration. */
    uint8_t                            numTableIndexPins;               /*!< Number of pins for table indexing */
    adi_adrv9001_GpioCfg_t             tableIndexGpioConfig[ADI_ADRV9001_FH_MAX_NUM_FREQ_SELECT_PINS]; /*!< Pin configuration for table index select. Ignored if tableIndexCtrl is not ADI_ADRV9001_TABLEINDEXCTRL_GPIO */
    
    bool                               gainSetupByPin;            /*!< Use GPIO Pins to provide a Tx/Rx gain index for next hop frame. If false, gain information is provided in hop table*/
	adi_adrv9001_FhGainSetupByPinCfg_t gainSetupByPinConfig[ADI_ADRV9001_NUM_CHANNELS];    /*!< Configuration information for gain setup by pin. This structure is ignored if gainSetupByPin is false */
	bool                               enableAGCGainIndexSeeding;       /*!< Enable seeding of the hardware value of adi_adrv9001_GainControlCfg_t->resetOnRxonGainIndex with rxGainIndex from FHTable,
                                                                             if mode is ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO then 
                                                                             gainIndex will begin tracking from this seeded gainIndex at the beginning of the frame */
} adi_adrv9001_FhCfg_t;

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_FH_TYPES_H_ */
