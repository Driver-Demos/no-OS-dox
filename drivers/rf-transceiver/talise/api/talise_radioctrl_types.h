/**
 * \file talise_radioctrl_types.h
 * \brief Contains Talise API Radio Control data types
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_RADIOCTRL_TYPES_H_
#define TALISE_RADIOCTRL_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief The `taliseGpioPinSel_t` is an enumeration that defines a set of
 * constants representing the selection of low voltage GPIO pins used by
 * the Talise API. It includes valid GPIO pin selections from 0 to 18, as
 * well as an option for an invalid selection, which can be used to
 * indicate an unassigned or erroneous state.
 *
 * @param TAL_GPIO_00 Represents GPIO pin 0.
 * @param TAL_GPIO_01 Represents GPIO pin 1.
 * @param TAL_GPIO_02 Represents GPIO pin 2.
 * @param TAL_GPIO_03 Represents GPIO pin 3.
 * @param TAL_GPIO_04 Represents GPIO pin 4.
 * @param TAL_GPIO_05 Represents GPIO pin 5.
 * @param TAL_GPIO_06 Represents GPIO pin 6.
 * @param TAL_GPIO_07 Represents GPIO pin 7.
 * @param TAL_GPIO_08 Represents GPIO pin 8.
 * @param TAL_GPIO_09 Represents GPIO pin 9.
 * @param TAL_GPIO_10 Represents GPIO pin 10.
 * @param TAL_GPIO_11 Represents GPIO pin 11.
 * @param TAL_GPIO_12 Represents GPIO pin 12.
 * @param TAL_GPIO_13 Represents GPIO pin 13.
 * @param TAL_GPIO_14 Represents GPIO pin 14.
 * @param TAL_GPIO_15 Represents GPIO pin 15.
 * @param TAL_GPIO_16 Represents GPIO pin 16.
 * @param TAL_GPIO_17 Represents GPIO pin 17.
 * @param TAL_GPIO_18 Represents GPIO pin 18.
 * @param TAL_GPIO_INVALID Represents an invalid GPIO pin selection.
 ******************************************************************************/
typedef enum {
	TAL_GPIO_00 = 0,
	TAL_GPIO_01,
	TAL_GPIO_02,
	TAL_GPIO_03,
	TAL_GPIO_04,
	TAL_GPIO_05,
	TAL_GPIO_06,
	TAL_GPIO_07,
	TAL_GPIO_08,
	TAL_GPIO_09,
	TAL_GPIO_10,
	TAL_GPIO_11,
	TAL_GPIO_12,
	TAL_GPIO_13,
	TAL_GPIO_14,
	TAL_GPIO_15,
	TAL_GPIO_16,
	TAL_GPIO_17,
	TAL_GPIO_18,
	TAL_GPIO_INVALID

} taliseGpioPinSel_t;

/***************************************************************************//**
 * @brief The `taliseExtLoDiv_t` is an enumerated type that defines various
 * division factors for the external Local Oscillator (LO) output
 * frequency, which is derived from the RFPLL VCO frequency. Each
 * enumerator corresponds to a specific frequency range, achieved by
 * dividing the RFPLL VCO frequency by a power of two, allowing the
 * selection of the desired frequency range for external LO output.
 *
 * @param TAL_EXTLO_RFPLLVCO_DIV2 External LO output in frequency range 3000 MHz
 * - 6000 MHz.
 * @param TAL_EXTLO_RFPLLVCO_DIV4 External LO output in frequency range 1500 MHz
 * - 3000 MHz.
 * @param TAL_EXTLO_RFPLLVCO_DIV8 External LO output in frequency range 750 MHz
 * - 1500 MHz.
 * @param TAL_EXTLO_RFPLLVCO_DIV16 External LO output in frequency range 375 MHz
 * - 750 MHz.
 * @param TAL_EXTLO_RFPLLVCO_DIV32 External LO output in frequency range 187.5
 * MHz - 375 MHz.
 * @param TAL_EXTLO_RFPLLVCO_DIV64 External LO output in frequency range 93.75
 * MHz - 187.5 MHz.
 ******************************************************************************/
typedef enum {
	TAL_EXTLO_RFPLLVCO_DIV2 =  0,  /*!< External LO output in frequency range 3000 MHz    - 6000 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV4 =  1,  /*!< External LO output in frequency range 1500 MHz    - 3000 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV8 =  2,  /*!< External LO output in frequency range  750 MHz    - 1500 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV16 = 3,  /*!< External LO output in frequency range  375 Mhz    -  750 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV32 = 4,  /*!< External LO output in frequency range  187.5 MHz  -  375 MHz   */
	TAL_EXTLO_RFPLLVCO_DIV64 = 5   /*!< External LO output in frequency range   93.75 MHz -  187.5 MHz */
} taliseExtLoDiv_t;
/***************************************************************************//**
 * @brief The `taliseArmGpioPinSettings_t` structure is used to configure the
 * settings for a GPIO pin in the Talise ARM system. It allows the
 * selection of a specific GPIO pin, the configuration of the signal
 * polarity, and the enabling or disabling of the ARM's use of the GPIO
 * signal. This structure is essential for managing how the Talise ARM
 * interacts with external signals through GPIO pins, providing
 * flexibility in signal handling and control.
 *
 * @param gpioPinSel Selects the desired GPIO pin to input into Talise, valid
 * range is 0-15.
 * @param polarity Indicates the signal polarity, where 0 is normal polarity and
 * 1 means Talise will invert the signal before using.
 * @param enable Determines if the Talise ARM uses the GPIO signal (1) or an ARM
 * command to set the signal value (0).
 ******************************************************************************/
typedef struct {
	taliseGpioPinSel_t
	gpioPinSel; /*!< Select desired GPIO pin to input into Talise (valid 0-15) */
	uint8_t polarity;           /*!< Signal polarity (0 = Normal polarity, 1=Talise will invert the signal before using) */
	uint8_t enable;             /*!< 1 = Enable Talise ARM use of GPIO signal, 0 = Talise ARM uses ARM command to set this signal value */

} taliseArmGpioPinSettings_t;


/***************************************************************************//**
 * @brief The `taliseArmGpioConfig_t` structure is designed to configure the ARM
 * GPIO pin assignments for the Talise device. It includes settings for
 * five GPIO pins, each represented by a `taliseArmGpioPinSettings_t`
 * structure, which specifies the pin selection, polarity, and enable
 * status. This configuration is crucial for managing the input GPIO pins
 * when the ORx pin mode is active, allowing for precise control over the
 * Talise's GPIO interactions.
 *
 * @param orx1TxSel0Pin Selects a GPIO pin for input into Talise with settings
 * for polarity and enable.
 * @param orx1TxSel1Pin Selects a GPIO pin for input into Talise with settings
 * for polarity and enable.
 * @param orx2TxSel0Pin Selects a GPIO pin for input into Talise with settings
 * for polarity and enable.
 * @param orx2TxSel1Pin Selects a GPIO pin for input into Talise with settings
 * for polarity and enable.
 * @param enTxTrackingCals Selects a GPIO pin for input into Talise with
 * settings for polarity and enable.
 ******************************************************************************/
typedef struct {
	/* Talise ARM input GPIO pins -- Only valid if orxPinMode = 1 */
	taliseArmGpioPinSettings_t
	orx1TxSel0Pin;      /*!< Select desired GPIO pin to input into Talise(valid 0-15), polarity, enable */
	taliseArmGpioPinSettings_t
	orx1TxSel1Pin;      /*!< Select desired GPIO pin to input into Talise(valid 0-15), polarity, enable */
	taliseArmGpioPinSettings_t
	orx2TxSel0Pin;      /*!< Select desired GPIO pin to input into Talise(valid 0-15), polarity, enable */
	taliseArmGpioPinSettings_t
	orx2TxSel1Pin;      /*!< Select desired GPIO pin to input into Talise(valid 0-15), polarity, enable */
	taliseArmGpioPinSettings_t
	enTxTrackingCals;   /*!< Select desired GPIO pin to input into Talise(valid 0-15), polarity, enable */

} taliseArmGpioConfig_t;

/***************************************************************************//**
 * @brief The `taliseRadioCtlCfg2_t` is an enumeration that defines the possible
 * GPIO pin pair selections for the ORx1 and ORx2 radio control
 * configuration in the Talise API. It allows the user to select between
 * different pairs of GPIO pins (0,1; 4,5; 8,9) or opt for no selection,
 * which is useful for configuring the hardware interface for radio
 * operations.
 *
 * @param TAL_ORX1ORX2_PAIR_01_SEL Radio Control Config 2 ORx1/ORx2 GPIO '0,1'
 * pin pair select.
 * @param TAL_ORX1ORX2_PAIR_45_SEL Radio Control Config 2 ORx1/ORx2 GPIO '4,5'
 * pin pair select.
 * @param TAL_ORX1ORX2_PAIR_89_SEL Radio Control Config 2 ORx1/ORx2 GPIO '8,9'
 * pin pair select.
 * @param TAL_ORX1ORX2_PAIR_NONE_SEL Radio Control Config 2 ORx1/ORx2 GPIO
 * 'none' pin pair select.
 ******************************************************************************/
typedef enum {
	TAL_ORX1ORX2_PAIR_01_SEL = 0x00,   /*!< Radio Control Config 2 ORx1/ORx2 GPIO '0,1' pin pair select */
	TAL_ORX1ORX2_PAIR_45_SEL,          /*!< Radio Control Config 2 ORx1/ORx2 GPIO '4,5' pin pair select */
	TAL_ORX1ORX2_PAIR_89_SEL,          /*!< Radio Control Config 2 ORx1/ORx2 GPIO '8,9' pin pair select */
	TAL_ORX1ORX2_PAIR_NONE_SEL         /*!< Radio Control Config 2 ORx1/ORx2 GPIO 'none' pin pair select */

} taliseRadioCtlCfg2_t;

/***************************************************************************//**
 * @brief The `taliseRadioCtlCfg1_t` is an enumerated type that defines various
 * bit masks for configuring radio control settings in the Talise API.
 * Each enumerator represents a specific configuration option, such as
 * selecting the pin mode for transmission and reception (TXRX),
 * observation receiver (ORX) pin mode, and enabling or selecting
 * specific channels or pins. These configurations are used to control
 * the behavior of the radio hardware in a Talise device.
 *
 * @param TAL_TXRX_PIN_MODE Radio Control Config 1 bit '0' mask.
 * @param TAL_ORX_PIN_MODE Radio Control Config 1 bit '1' mask.
 * @param TAL_ORX_USES_RX_PINS Radio Control Config 1 bit '2' mask.
 * @param TAL_ORX_SEL Radio Control Config 1 bit '4' mask.
 * @param TAL_ORX_SINGLE_CHANNEL Radio Control Config 1 bit '5' mask.
 * @param TAL_ORX_ENAB_SEL_PIN Radio Control Config 1 bit '6' mask.
 ******************************************************************************/
typedef enum {
	TAL_TXRX_PIN_MODE = 0x01,       /*!< Radio Control Config 1 bit '0' mask */
	TAL_ORX_PIN_MODE = 0x02,        /*!< Radio Control Config 1 bit '1' mask */
	TAL_ORX_USES_RX_PINS = 0x04,    /*!< Radio Control Config 1 bit '2' mask */
	TAL_ORX_SEL = 0x10,             /*!< Radio Control Config 1 bit '4' mask */
	TAL_ORX_SINGLE_CHANNEL = 0x20,  /*!< Radio Control Config 1 bit '5' mask */
	TAL_ORX_ENAB_SEL_PIN = 0x40     /*!< Radio Control Config 1 bit '6' mask */
} taliseRadioCtlCfg1_t;

/***************************************************************************//**
 * @brief The `taliseRxORxChannels_t` is an enumeration that defines the
 * possible states for enabling or disabling various combinations of Rx
 * and ORx channels in a Talise radio system. It provides options to
 * enable individual channels like Rx1, Rx2, ORx1, and ORx2, as well as
 * combinations of these channels, allowing for flexible configuration of
 * the radio's receive and observation paths.
 *
 * @param TAL_RXOFF_EN All Rx/ORx channels are turned off.
 * @param TAL_RX1_EN Enables the Rx1 channel.
 * @param TAL_RX2_EN Enables the Rx2 channel.
 * @param TAL_RX1RX2_EN Enables both Rx1 and Rx2 channels.
 * @param TAL_ORX1_EN Enables the ORx1 channel.
 * @param TAL_ORX2_EN Enables the ORx2 channel.
 * @param TAL_ORX1ORX2_EN Enables both ORx1 and ORx2 channels, only if ADC
 * stitching is not enabled.
 ******************************************************************************/
typedef enum {
	TAL_RXOFF_EN    = 0x00,  /*!< All Rx/ORx channels off */
	TAL_RX1_EN      = 0x01,  /*!< Rx1 channel enabled */
	TAL_RX2_EN      = 0x02,  /*!< Rx2 channel enabled */
	TAL_RX1RX2_EN   = 0x03,  /*!< Rx1 + Rx2 channels enabled */
	TAL_ORX1_EN     = 0x04,  /*!< ORx1 channel enabled */
	TAL_ORX2_EN     = 0x08,  /*!< ORx2 channel enabled */
	TAL_ORX1ORX2_EN = 0x0C	 /*!< ORx1 and ORx2 channels enabled - only allowed if ADC stitching is not enabled */

} taliseRxORxChannels_t;

/***************************************************************************//**
 * @brief The `taliseTxToOrxMapping_t` is an enumerated type that defines the
 * possible mappings between transmit (Tx) and observation receiver (ORx)
 * paths in a radio control system. It provides options to select no
 * mapping, map Tx1 to ORx, or map Tx2 to ORx, facilitating the
 * configuration of signal paths for monitoring and calibration purposes
 * in radio frequency applications.
 *
 * @param TAL_MAP_NONE No Tx to ORx mapping select.
 * @param TAL_MAP_TX1_ORX Tx1 to ORx mapping select.
 * @param TAL_MAP_TX2_ORX Tx2 to ORx mapping select.
 ******************************************************************************/
typedef enum {
	TAL_MAP_NONE = 0,           /*!< No Tx to ORx mapping select */
	TAL_MAP_TX1_ORX = 0x02,     /*!< Tx1 to ORx mapping select */
	TAL_MAP_TX2_ORX = 0x03      /*!< Tx2 to ORx mapping select */

} taliseTxToOrxMapping_t;

/***************************************************************************//**
 * @brief The `taliseRfPllName_t` is an enumeration that defines the possible
 * Phase-Locked Loop (PLL) selections for the Talise radio control API.
 * It allows the user to specify which PLL (Clock, RF, or Auxiliary) is
 * to be used for both the receive (Rx) and transmit (Tx) paths. This
 * selection is crucial for configuring the radio's frequency synthesis
 * and signal processing capabilities.
 *
 * @param TAL_CLK_PLL Selects CLK PLL for Rx and Tx.
 * @param TAL_RF_PLL Selects RF PLL for Rx and Tx.
 * @param TAL_AUX_PLL Selects AUX PLL for Rx and Tx.
 ******************************************************************************/
typedef enum {
	TAL_CLK_PLL = 0,                /*!< Selects CLK PLL for Rx and Tx */
	TAL_RF_PLL,                     /*!< Selects RF PLL for Rx and Tx */
	TAL_AUX_PLL                     /*!< Selects AUX PLL for Rx and tx*/

} taliseRfPllName_t;

/***************************************************************************//**
 * @brief The `taliseOrxLoCfg_t` structure is used to configure the ORx LO
 * selection feature in the Talise API. It allows the user to disable the
 * automatic relocking of the Auxiliary PLL by the ARM processor and to
 * select a specific GPIO pin for input. This configuration is crucial
 * when using the Auxiliary LO as the ORx LO source, providing
 * flexibility in the LO source selection and GPIO pin configuration.
 *
 * @param disableAuxPllRelocking Disables the ARM from automatically relocking
 * the Aux PLL, with a setting of 1 when using
 * AuxLO as ORx LO source and 0 as default when
 * RFPLL is used as ORx LO source.
 * @param gpioSelect Selects the desired GPIO pin for input, with
 * TAL_GPIO_INVALID disabling pin mode and GPIO0-15 being
 * valid selections.
 ******************************************************************************/
typedef struct {
	uint8_t disableAuxPllRelocking; /*!< Disables the ARM from automatically relocking the Aux PLL.
                                         Set to 1 when using AuxLO as ORx LO source, 0 = default when RFPLL used as ORx LO source */
	taliseGpioPinSel_t
	gpioSelect;  /*!< TAL_GPIO_INVALID = disable pin mode, GPIO0-15 valid */
} taliseOrxLoCfg_t;

/***************************************************************************//**
 * @brief The `taliseFhmExitMode_t` is an enumeration that defines the modes for
 * exiting frequency hopping in a Talise radio system. It provides two
 * options: a quick exit mode, which maintains the current RF PLL
 * bandwidth, and a full exit mode, which narrows the RF PLL bandwidth
 * and recalibrates the RF and Aux PLLs. This allows for flexibility in
 * managing the frequency hopping disable process, depending on the
 * desired system behavior.
 *
 * @param TAL_FHM_QUICK_EXIT Selects quick exit mode on frequency hopping
 * disable, leaving RF PLL bandwidth unchanged.
 * @param TAL_FHM_FULL_EXIT Selects full exit mode on frequency hopping disable,
 * restoring RF PLL Loop B/W to narrowband and
 * recalibrating RF and Aux PLLs.
 ******************************************************************************/
typedef enum {
	TAL_FHM_QUICK_EXIT = 0,  /*!< Selects quick exit mode on frequency hopping disable. In this case RF PLL bandwidth is left unchanded */
	TAL_FHM_FULL_EXIT        /*!< Selects full exit mode on frequency hopping disable. RF PLL Loop B/W is restored to narrowband.
                                  RF and Aux PLLs recalibrated and tracking cals resumed. */
} taliseFhmExitMode_t;

/***************************************************************************//**
 * @brief The `taliseFhmTriggerMode_t` is an enumeration that defines the
 * possible trigger modes for frequency hopping in the Talise API. It
 * includes modes for triggering via GPIO pulses or ARM commands, as well
 * as an option for an invalid mode, providing flexibility in how
 * frequency hopping can be initiated.
 *
 * @param TAL_FHM_GPIO_MODE Selects FHM trigger mode as GPIO, where a low to
 * high pulse triggers frequency hop.
 * @param TAL_FHM_NON_GPIO_MODE Selects FHM trigger mode as non-GPIO, where an
 * ARM command triggers frequency hop.
 * @param TAL_FHM_INVALID_TRIGGER_MODE Represents an invalid FHM trigger mode.
 ******************************************************************************/
typedef enum {
	TAL_FHM_GPIO_MODE = 0,       /*!< Selects FHM trigger mode as GPIO. A low to high pulse triggers frequency hop */
	TAL_FHM_NON_GPIO_MODE,       /*!< Selects FHM trigger mode as non-GPIO. An ARM command triggers frequency hop */
	TAL_FHM_INVALID_TRIGGER_MODE
} taliseFhmTriggerMode_t;

/***************************************************************************//**
 * @brief The `taliseFhmConfig_t` structure is used to configure frequency
 * hopping settings for the Talise radio. It includes a GPIO pin
 * selection for triggering frequency hops and defines the minimum and
 * maximum frequencies for the hopping range. This configuration allows
 * for dynamic frequency adjustments within specified limits, enhancing
 * the radio's adaptability to changing conditions.
 *
 * @param fhmGpioPin Maps the Talise ARM GPIO pin (TAL_GPIO_0 - TAL_GPIO_15) for
 * frequency hopping, with a low to high pulse triggering
 * frequency hopping.
 * @param fhmMinFreq_MHz Sets the minimum frequency for the frequency hopping
 * range.
 * @param fhmMaxFreq_MHz Sets the maximum frequency for the frequency hopping
 * range.
 ******************************************************************************/
typedef struct {
	taliseGpioPinSel_t
	fhmGpioPin; /*!< Maps the Talise ARM GPIO pin(TAL_GPIO_0 - TAL_GPIO_15) for frequency hopping. A low to high pulse on this pin triggers freq hopping
                                        Setting fhmGpioPin = TAL_GPIO_INVALID will unassign ARM GPIO pin mapped to Rf Pll Frequency hopping*/
	uint32_t fhmMinFreq_MHz;       /*!< Sets frequency hopping range minimum frequency */
	uint32_t fhmMaxFreq_MHz;       /*!< Sets frequency hopping range maximum frequency */
} taliseFhmConfig_t;

/***************************************************************************//**
 * @brief The `taliseFhmMode_t` structure is used to configure the frequency
 * hopping mode settings for the Talise radio. It includes fields to
 * enable or disable frequency hopping and MCS synchronization, select
 * the trigger mode for frequency hopping (either via GPIO or ARM
 * command), and choose the exit mode when frequency hopping is disabled.
 * Additionally, it specifies the initial frequency for the RF PLL when
 * frequency hopping is enabled. This structure is crucial for managing
 * the dynamic frequency hopping capabilities of the Talise radio system.
 *
 * @param fhmEnable Indicates whether Frequency Hopping is enabled (1) or
 * disabled (0).
 * @param enableMcsSync Indicates whether MCS Synchronization is enabled (1) or
 * disabled (0) when FHM is enabled, ignored if fhmEnable
 * is 0.
 * @param fhmTriggerMode Specifies the trigger mode for frequency hopping,
 * either via GPIO pulse or ARM command.
 * @param fhmExitMode Specifies the exit mode on frequency hopping disable,
 * either quick or full exit, ignored if fhmEnable is 1.
 * @param fhmInitFrequency_Hz The initial frequency in Hz that the RF PLL is
 * configured to when enabling FHM.
 ******************************************************************************/
typedef struct {
	uint8_t fhmEnable;                      /*!< 0 - Disables Frequency Hopping, 1 - Enables Frequency Hopping */
	uint8_t enableMcsSync;                  /*!< 0 - Disables MCS Synchronization on FHM enable, 1 - Enables MCS Synchronization on FHM enable. Ignored if fhmEnable = 0 */
	taliseFhmTriggerMode_t
	fhmTriggerMode;  /*!< TAL_FHM_GPIO_MODE - Frequency Hop triggered via GPIO low to high pulse
                                                 TAL_FHM_NON_GPIO_MODE - Frequency Hop triggered via ARM command*/
	taliseFhmExitMode_t
	fhmExitMode;        /*!< TAL_FHM_QUICK_EXIT = quick exit on frequency hopping disable,
                                                 TAL_FHM_FULL_EXIT = Full exit on frequency hopping disable. This is ignored if fhmEnable = 1*/
	uint64_t fhmInitFrequency_Hz;           /*!< First hop frequency that Rf Pll is configured to on enabling FHM */
} taliseFhmMode_t;

/***************************************************************************//**
 * @brief The `taliseFhmStatus_t` structure is designed to hold the status
 * information related to frequency hopping mode (FHM) in a radio control
 * system. It includes various fields to track error statuses, the number
 * of hops, and specific frequencies associated with errors or events.
 * This structure is crucial for monitoring and diagnosing the
 * performance and issues encountered during frequency hopping
 * operations, providing detailed insights into the operational status
 * and any anomalies that may occur.
 *
 * @param currentFhmCmdErrorStatus Current FHM Enter Command Error Status, same
 * as the mailbox command error status for FHM.
 * @param currentFhmHopErrorStatus Currently active FHM errors during frequency
 * hopping.
 * @param numFhmHops Total number of hops since entering FHM.
 * @param numFhmNoErrorEvents Total number of NO FHM error events.
 * @param lastFhmNoErrorFreq_Hz Last frequency for which NO Error was
 * encountered.
 * @param numFhmHopsOutsideScanRange Total number of hops outside FHM scan
 * range.
 * @param lastFreqOutsideScanRange_Hz Last frequency which was outside FHM scan
 * range.
 * @param numInvalidFhmHopFrequencies Number of invalid hop frequencies.
 * @param lastInvalidHopFreq_Hz Last invalid hop frequency.
 * @param compPllError PLL LO Computation Error.
 * @param compPllErrorFreq_Hz PLL LO Computation Error frequency.
 * @param rfPllLockFailed RF PLL Lock failed.
 * @param rfPllLockFailedFreq_Hz RF PLL Lock failed frequency.
 ******************************************************************************/
typedef struct {
	uint16_t currentFhmCmdErrorStatus;     /*!< Current FHM Enter Command Error Status. This is same as the mailbox command error status for FHM */
	uint16_t currentFhmHopErrorStatus;     /*!< Currently active FHM errors during frequency hopping */
	uint32_t numFhmHops;                   /*!< Total no. of Hops since entering FHM */
	uint32_t numFhmNoErrorEvents;          /*!< Total no. of NO FHM error events */
	uint64_t lastFhmNoErrorFreq_Hz;        /*!< Last frequency for which NO Error was encountered */
	uint32_t numFhmHopsOutsideScanRange;   /*!< Total no. of Hops outside FHM scan range */
	uint64_t lastFreqOutsideScanRange_Hz;  /*!< Last frequency which was outside FHM scan range*/
	uint32_t numInvalidFhmHopFrequencies;  /*!< Invalid Hop Freq */
	uint64_t lastInvalidHopFreq_Hz;        /*!< Last Invalid Hop Freq */
	uint32_t compPllError;                 /*!< PLL LO Computation Error */
	uint64_t compPllErrorFreq_Hz;          /*!< PLL LO Computation Error frequency */
	uint32_t rfPllLockFailed;              /*!< RF PLL Lock failed */
	uint64_t rfPllLockFailedFreq_Hz;       /*!< RF PLL Lock failed frequency*/
} taliseFhmStatus_t;

#ifdef __cplusplus
}
#endif

#endif /* TALISE_RADIOCTRL_TYPES_H_ */
