/**
 * \file
 * \brief Contains functions to allow control of the General Purpose IO functions on the ADRV9001 device
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_GPIO_TYPES_H_
#define _ADI_ADRV9001_GPIO_TYPES_H_

#include "adi_adrv9001_user.h"
#include "adi_adrv9001_types.h"

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioPin_e` is an enumeration that defines a set of
 * constants representing the various GPIO pins available on the ADRV9001
 * device. It includes both digital and analog GPIO pins, as well as an
 * unassigned state. This enumeration is used to specify which GPIO pin
 * is being referenced or configured in the context of the ADRV9001 API,
 * facilitating the control and management of the device's general-
 * purpose input/output functionalities.
 *
 * @param ADI_ADRV9001_GPIO_UNASSIGNED No GPIO pin selected.
 * @param ADI_ADRV9001_GPIO_DIGITAL_00 Digital GPIO Pin 0.
 * @param ADI_ADRV9001_GPIO_DIGITAL_01 Digital GPIO Pin 1.
 * @param ADI_ADRV9001_GPIO_DIGITAL_02 Digital GPIO Pin 2.
 * @param ADI_ADRV9001_GPIO_DIGITAL_03 Digital GPIO Pin 3.
 * @param ADI_ADRV9001_GPIO_DIGITAL_04 Digital GPIO Pin 4.
 * @param ADI_ADRV9001_GPIO_DIGITAL_05 Digital GPIO Pin 5.
 * @param ADI_ADRV9001_GPIO_DIGITAL_06 Digital GPIO Pin 6.
 * @param ADI_ADRV9001_GPIO_DIGITAL_07 Digital GPIO Pin 7.
 * @param ADI_ADRV9001_GPIO_DIGITAL_08 Digital GPIO Pin 8.
 * @param ADI_ADRV9001_GPIO_DIGITAL_09 Digital GPIO Pin 9.
 * @param ADI_ADRV9001_GPIO_DIGITAL_10 Digital GPIO Pin 10.
 * @param ADI_ADRV9001_GPIO_DIGITAL_11 Digital GPIO Pin 11.
 * @param ADI_ADRV9001_GPIO_DIGITAL_12 Digital GPIO Pin 12.
 * @param ADI_ADRV9001_GPIO_DIGITAL_13 Digital GPIO Pin 13.
 * @param ADI_ADRV9001_GPIO_DIGITAL_14 Digital GPIO Pin 14.
 * @param ADI_ADRV9001_GPIO_DIGITAL_15 Digital GPIO Pin 15.
 * @param ADI_ADRV9001_GPIO_ANALOG_00 Analog GPIO Pin 0.
 * @param ADI_ADRV9001_GPIO_ANALOG_01 Analog GPIO Pin 1.
 * @param ADI_ADRV9001_GPIO_ANALOG_02 Analog GPIO Pin 2.
 * @param ADI_ADRV9001_GPIO_ANALOG_03 Analog GPIO Pin 3.
 * @param ADI_ADRV9001_GPIO_ANALOG_04 Analog GPIO Pin 4.
 * @param ADI_ADRV9001_GPIO_ANALOG_05 Analog GPIO Pin 5.
 * @param ADI_ADRV9001_GPIO_ANALOG_06 Analog GPIO Pin 6.
 * @param ADI_ADRV9001_GPIO_ANALOG_07 Analog GPIO Pin 7.
 * @param ADI_ADRV9001_GPIO_ANALOG_08 Analog GPIO Pin 8.
 * @param ADI_ADRV9001_GPIO_ANALOG_09 Analog GPIO Pin 9.
 * @param ADI_ADRV9001_GPIO_ANALOG_10 Analog GPIO Pin 10.
 * @param ADI_ADRV9001_GPIO_ANALOG_11 Analog GPIO Pin 11.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioPin
{
    ADI_ADRV9001_GPIO_UNASSIGNED,       /*!< No GPIO pin selected */
    ADI_ADRV9001_GPIO_DIGITAL_00,       /*!< Digital GPIO Pin 0 */
    ADI_ADRV9001_GPIO_DIGITAL_01,       /*!< Digital GPIO Pin 1 */
    ADI_ADRV9001_GPIO_DIGITAL_02,       /*!< Digital GPIO Pin 2 */
    ADI_ADRV9001_GPIO_DIGITAL_03,       /*!< Digital GPIO Pin 3 */
    ADI_ADRV9001_GPIO_DIGITAL_04,       /*!< Digital GPIO Pin 4 */
    ADI_ADRV9001_GPIO_DIGITAL_05,       /*!< Digital GPIO Pin 5 */
    ADI_ADRV9001_GPIO_DIGITAL_06,       /*!< Digital GPIO Pin 6 */
    ADI_ADRV9001_GPIO_DIGITAL_07,       /*!< Digital GPIO Pin 7 */
    ADI_ADRV9001_GPIO_DIGITAL_08,       /*!< Digital GPIO Pin 8 */
    ADI_ADRV9001_GPIO_DIGITAL_09,       /*!< Digital GPIO Pin 9 */
    ADI_ADRV9001_GPIO_DIGITAL_10,       /*!< Digital GPIO Pin 10 */
    ADI_ADRV9001_GPIO_DIGITAL_11,       /*!< Digital GPIO Pin 11 */
    ADI_ADRV9001_GPIO_DIGITAL_12,       /*!< Digital GPIO Pin 12 */
    ADI_ADRV9001_GPIO_DIGITAL_13,       /*!< Digital GPIO Pin 13 */
    ADI_ADRV9001_GPIO_DIGITAL_14,       /*!< Digital GPIO Pin 14 */
    ADI_ADRV9001_GPIO_DIGITAL_15,       /*!< Digital GPIO Pin 15 */
    ADI_ADRV9001_GPIO_ANALOG_00,        /*!< Analog GPIO Pin 0 */
    ADI_ADRV9001_GPIO_ANALOG_01,        /*!< Analog GPIO Pin 1 */
    ADI_ADRV9001_GPIO_ANALOG_02,        /*!< Analog GPIO Pin 2 */
    ADI_ADRV9001_GPIO_ANALOG_03,        /*!< Analog GPIO Pin 3 */
    ADI_ADRV9001_GPIO_ANALOG_04,        /*!< Analog GPIO Pin 4 */
    ADI_ADRV9001_GPIO_ANALOG_05,        /*!< Analog GPIO Pin 5 */
    ADI_ADRV9001_GPIO_ANALOG_06,        /*!< Analog GPIO Pin 6 */
    ADI_ADRV9001_GPIO_ANALOG_07,        /*!< Analog GPIO Pin 7 */
    ADI_ADRV9001_GPIO_ANALOG_08,        /*!< Analog GPIO Pin 8 */
    ADI_ADRV9001_GPIO_ANALOG_09,        /*!< Analog GPIO Pin 9 */
    ADI_ADRV9001_GPIO_ANALOG_10,        /*!< Analog GPIO Pin 10 */
    ADI_ADRV9001_GPIO_ANALOG_11,        /*!< Analog GPIO Pin 11 */
} adi_adrv9001_GpioPin_e;
    
/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioPinCrumbSel_e` is an enumeration that defines
 * various selections for GPIO pin crumbs on the ADRV9001 device. Each
 * enumerator represents a specific pair of GPIO pins that can be
 * selected for configuration or control purposes. This enumeration is
 * used to facilitate the management and assignment of GPIO pins in
 * pairs, allowing for efficient control and configuration of the
 * device's GPIO functionalities.
 *
 * @param ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED Represents an unassigned GPIO
 * pin crumb.
 * @param ADI_ADRV9001_GPIO_PIN_CRUMB_01_00 Represents GPIO pin crumb selection
 * for pins 01 and 00.
 * @param ADI_ADRV9001_GPIO_PIN_CRUMB_03_02 Represents GPIO pin crumb selection
 * for pins 03 and 02.
 * @param ADI_ADRV9001_GPIO_PIN_CRUMB_05_04 Represents GPIO pin crumb selection
 * for pins 05 and 04.
 * @param ADI_ADRV9001_GPIO_PIN_CRUMB_07_06 Represents GPIO pin crumb selection
 * for pins 07 and 06.
 * @param ADI_ADRV9001_GPIO_PIN_CRUMB_09_08 Represents GPIO pin crumb selection
 * for pins 09 and 08.
 * @param ADI_ADRV9001_GPIO_PIN_CRUMB_11_10 Represents GPIO pin crumb selection
 * for pins 11 and 10.
 * @param ADI_ADRV9001_GPIO_PIN_CRUMB_13_12 Represents GPIO pin crumb selection
 * for pins 13 and 12.
 * @param ADI_ADRV9001_GPIO_PIN_CRUMB_15_14 Represents GPIO pin crumb selection
 * for pins 15 and 14.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioPinCrumbSel
{
    ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED,
    ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
    ADI_ADRV9001_GPIO_PIN_CRUMB_03_02,
    ADI_ADRV9001_GPIO_PIN_CRUMB_05_04,
    ADI_ADRV9001_GPIO_PIN_CRUMB_07_06,
    ADI_ADRV9001_GPIO_PIN_CRUMB_09_08,
    ADI_ADRV9001_GPIO_PIN_CRUMB_11_10,
    ADI_ADRV9001_GPIO_PIN_CRUMB_13_12,
    ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
} adi_adrv9001_GpioPinCrumbSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioTx1SsiPinSel_e` is an enumeration that defines
 * the selection of GPIO pins for the Tx1 SSI (Serial Synchronous
 * Interface) on the ADRV9001 device. It provides options to select from
 * a range of GPIO pins (00 to 07) specifically for the Tx1 SSI, as well
 * as an option to indicate an invalid selection. This enumeration is
 * used within the API to configure and control the GPIO pins associated
 * with the Tx1 SSI functionality.
 *
 * @param ADI_ADRV9001_GPIO_TX1_SSI_00 Select GPIO_Tx1_SSI_00.
 * @param ADI_ADRV9001_GPIO_TX1_SSI_01 Select GPIO_Tx1_SSI_01.
 * @param ADI_ADRV9001_GPIO_TX1_SSI_02 Select GPIO_Tx1_SSI_02.
 * @param ADI_ADRV9001_GPIO_TX1_SSI_03 Select GPIO_Tx1_SSI_03.
 * @param ADI_ADRV9001_GPIO_TX1_SSI_04 Select GPIO_Tx1_SSI_04.
 * @param ADI_ADRV9001_GPIO_TX1_SSI_05 Select GPIO_Tx1_SSI_05.
 * @param ADI_ADRV9001_GPIO_TX1_SSI_06 Select GPIO_Tx1_SSI_06.
 * @param ADI_ADRV9001_GPIO_TX1_SSI_07 Select GPIO_Tx1_SSI_07.
 * @param ADI_ADRV9001_GPIO_TX1_SSI_INVALID Invalid Tx1 SSI GPIO.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioTx1SsiPinSel
{
    ADI_ADRV9001_GPIO_TX1_SSI_00 = 0,  /*!< Select GPIO_Tx1_SSI_00*/
    ADI_ADRV9001_GPIO_TX1_SSI_01,      /*!< Select GPIO_Tx1_SSI_01*/
    ADI_ADRV9001_GPIO_TX1_SSI_02,      /*!< Select GPIO_Tx1_SSI_02*/
    ADI_ADRV9001_GPIO_TX1_SSI_03,      /*!< Select GPIO_Tx1_SSI_03*/
    ADI_ADRV9001_GPIO_TX1_SSI_04,      /*!< Select GPIO_Tx1_SSI_04*/
    ADI_ADRV9001_GPIO_TX1_SSI_05,      /*!< Select GPIO_Tx1_SSI_05*/
    ADI_ADRV9001_GPIO_TX1_SSI_06,      /*!< Select GPIO_Tx1_SSI_06*/
    ADI_ADRV9001_GPIO_TX1_SSI_07,      /*!< Select GPIO_Tx1_SSI_07*/    
    ADI_ADRV9001_GPIO_TX1_SSI_INVALID  /*!< Invalid Tx1 SSI GPIO*/
} adi_adrv9001_GpioTx1SsiPinSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioTx2SsiPinSel_e` is an enumeration that defines
 * the selection of GPIO pins for the Tx2 SSI interface on the ADRV9001
 * device. It provides specific identifiers for each GPIO pin option,
 * ranging from `ADI_ADRV9001_GPIO_TX2_SSI_00` to
 * `ADI_ADRV9001_GPIO_TX2_SSI_07`, and includes an
 * `ADI_ADRV9001_GPIO_TX2_SSI_INVALID` option to represent an invalid
 * selection. This enumeration is used to configure which GPIO pin is
 * used for the Tx2 SSI functionality.
 *
 * @param ADI_ADRV9001_GPIO_TX2_SSI_00 Select GPIO_Tx2_SSI_00.
 * @param ADI_ADRV9001_GPIO_TX2_SSI_01 Select GPIO_Tx2_SSI_01.
 * @param ADI_ADRV9001_GPIO_TX2_SSI_02 Select GPIO_Tx2_SSI_02.
 * @param ADI_ADRV9001_GPIO_TX2_SSI_03 Select GPIO_Tx2_SSI_03.
 * @param ADI_ADRV9001_GPIO_TX2_SSI_04 Select GPIO_Tx2_SSI_04.
 * @param ADI_ADRV9001_GPIO_TX2_SSI_05 Select GPIO_Tx2_SSI_05.
 * @param ADI_ADRV9001_GPIO_TX2_SSI_06 Select GPIO_Tx2_SSI_06.
 * @param ADI_ADRV9001_GPIO_TX2_SSI_07 Select GPIO_Tx2_SSI_07.
 * @param ADI_ADRV9001_GPIO_TX2_SSI_INVALID Invalid TxX2 SSI GPIO.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioTx2SsiPinSel
{
    ADI_ADRV9001_GPIO_TX2_SSI_00 = 0,  /*!< Select GPIO_Tx2_SSI_00*/
    ADI_ADRV9001_GPIO_TX2_SSI_01,      /*!< Select GPIO_Tx2_SSI_01*/
    ADI_ADRV9001_GPIO_TX2_SSI_02,      /*!< Select GPIO_Tx2_SSI_02*/
    ADI_ADRV9001_GPIO_TX2_SSI_03,      /*!< Select GPIO_Tx2_SSI_03*/
    ADI_ADRV9001_GPIO_TX2_SSI_04,      /*!< Select GPIO_Tx2_SSI_04*/
    ADI_ADRV9001_GPIO_TX2_SSI_05,      /*!< Select GPIO_Tx2_SSI_05*/
    ADI_ADRV9001_GPIO_TX2_SSI_06,      /*!< Select GPIO_Tx2_SSI_06*/
    ADI_ADRV9001_GPIO_TX2_SSI_07,      /*!< Select GPIO_Tx2_SSI_07*/    
    ADI_ADRV9001_GPIO_TX2_SSI_INVALID  /*!< Invalid TxX2 SSI GPIO*/
} adi_adrv9001_GpioTx2SsiPinSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioRx1SsiPinSel_e` is an enumeration that defines
 * the selection of GPIO pins for the Rx1 SSI interface on the ADRV9001
 * device. It provides options to select from a range of GPIO pins (00 to
 * 07) specifically for the Rx1 SSI, as well as an option to indicate an
 * invalid selection. This enumeration is used within the API to
 * configure and manage the GPIO settings for the Rx1 SSI interface.
 *
 * @param ADI_ADRV9001_GPIO_RX1_SSI_00 Select GPIO_Rx1_SSI_00.
 * @param ADI_ADRV9001_GPIO_RX1_SSI_01 Select GPIO_Rx1_SSI_01.
 * @param ADI_ADRV9001_GPIO_RX1_SSI_02 Select GPIO_Rx1_SSI_02.
 * @param ADI_ADRV9001_GPIO_RX1_SSI_03 Select GPIO_Rx1_SSI_03.
 * @param ADI_ADRV9001_GPIO_RX1_SSI_04 Select GPIO_Rx1_SSI_04.
 * @param ADI_ADRV9001_GPIO_RX1_SSI_05 Select GPIO_Rx1_SSI_05.
 * @param ADI_ADRV9001_GPIO_RX1_SSI_06 Select GPIO_Rx1_SSI_06.
 * @param ADI_ADRV9001_GPIO_RX1_SSI_07 Select GPIO_Rx1_SSI_07.
 * @param ADI_ADRV9001_GPIO_RX1_SSI_INVALID Invalid Rx1 SSI GPIO.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioRx1SsiPinSel
{
    ADI_ADRV9001_GPIO_RX1_SSI_00 = 0,  /*!< Select GPIO_Rx1_SSI_00*/
    ADI_ADRV9001_GPIO_RX1_SSI_01,      /*!< Select GPIO_Rx1_SSI_01*/
    ADI_ADRV9001_GPIO_RX1_SSI_02,      /*!< Select GPIO_Rx1_SSI_02*/
    ADI_ADRV9001_GPIO_RX1_SSI_03,      /*!< Select GPIO_Rx1_SSI_03*/
    ADI_ADRV9001_GPIO_RX1_SSI_04,      /*!< Select GPIO_Rx1_SSI_04*/
    ADI_ADRV9001_GPIO_RX1_SSI_05,      /*!< Select GPIO_Rx1_SSI_05*/
    ADI_ADRV9001_GPIO_RX1_SSI_06,      /*!< Select GPIO_Rx1_SSI_06*/
    ADI_ADRV9001_GPIO_RX1_SSI_07,      /*!< Select GPIO_Rx1_SSI_07*/    
    ADI_ADRV9001_GPIO_RX1_SSI_INVALID  /*!< Invalid Rx1 SSI GPIO*/
} adi_adrv9001_GpioRx1SsiPinSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioRx2SsiPinSel_e` is an enumeration that defines
 * the selection of GPIO pins for the Rx2 SSI interface on the ADRV9001
 * device. Each enumerator corresponds to a specific GPIO pin, allowing
 * the user to select which pin to use for the Rx2 SSI function. The
 * enumeration also includes an option for an invalid selection, which
 * can be used to indicate an error or uninitialized state.
 *
 * @param ADI_ADRV9001_GPIO_RX2_SSI_00 Select GPIO_Rx2_SSI_00.
 * @param ADI_ADRV9001_GPIO_RX2_SSI_01 Select GPIO_Rx2_SSI_01.
 * @param ADI_ADRV9001_GPIO_RX2_SSI_02 Select GPIO_Rx2_SSI_02.
 * @param ADI_ADRV9001_GPIO_RX2_SSI_03 Select GPIO_Rx2_SSI_03.
 * @param ADI_ADRV9001_GPIO_RX2_SSI_04 Select GPIO_Rx2_SSI_04.
 * @param ADI_ADRV9001_GPIO_RX2_SSI_05 Select GPIO_Rx2_SSI_05.
 * @param ADI_ADRV9001_GPIO_RX2_SSI_06 Select GPIO_Rx2_SSI_06.
 * @param ADI_ADRV9001_GPIO_RX2_SSI_07 Select GPIO_Rx2_SSI_07.
 * @param ADI_ADRV9001_GPIO_RX2_SSI_INVALID Invalid Rx2 SSI GPIO.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioRx2SsiPinSel
{
    ADI_ADRV9001_GPIO_RX2_SSI_00 = 0,  /*!< Select GPIO_Rx2_SSI_00*/
    ADI_ADRV9001_GPIO_RX2_SSI_01,      /*!< Select GPIO_Rx2_SSI_01*/
    ADI_ADRV9001_GPIO_RX2_SSI_02,      /*!< Select GPIO_Rx2_SSI_02*/
    ADI_ADRV9001_GPIO_RX2_SSI_03,      /*!< Select GPIO_Rx2_SSI_03*/
    ADI_ADRV9001_GPIO_RX2_SSI_04,      /*!< Select GPIO_Rx2_SSI_04*/
    ADI_ADRV9001_GPIO_RX2_SSI_05,      /*!< Select GPIO_Rx2_SSI_05*/
    ADI_ADRV9001_GPIO_RX2_SSI_06,      /*!< Select GPIO_Rx2_SSI_06*/
    ADI_ADRV9001_GPIO_RX2_SSI_07,      /*!< Select GPIO_Rx2_SSI_07*/    
    ADI_ADRV9001_GPIO_RX2_SSI_INVALID  /*!< Invalid Rx2 SSI GPIO*/
} adi_adrv9001_GpioRx2SsiPinSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioPinDirection_e` is an enumeration that defines
 * the possible directions for a GPIO pin on the ADRV9001 device,
 * specifically indicating whether a pin is set as an input or an output.
 * This enumeration is used to configure the directionality of GPIO pins,
 * which is crucial for ensuring correct data flow and control in
 * hardware interfacing.
 *
 * @param ADI_ADRV9001_GPIO_PIN_DIRECTION_INPUT Represents a GPIO pin configured
 * as an input.
 * @param ADI_ADRV9001_GPIO_PIN_DIRECTION_OUTPUT Represents a GPIO pin
 * configured as an output.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioPinDirection
{
    ADI_ADRV9001_GPIO_PIN_DIRECTION_INPUT,
    ADI_ADRV9001_GPIO_PIN_DIRECTION_OUTPUT,
} adi_adrv9001_GpioPinDirection_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioPinLevel_e` is an enumeration that defines the
 * possible levels for a GPIO pin on the ADRV9001 device, specifically
 * indicating whether the pin is in a low or high state. This enumeration
 * is used to control and configure the state of GPIO pins, which are
 * essential for interfacing and communication with other components in a
 * system.
 *
 * @param ADI_ADRV9001_GPIO_PIN_LEVEL_LOW Represents a low level state for a
 * GPIO pin.
 * @param ADI_ADRV9001_GPIO_PIN_LEVEL_HIGH Represents a high level state for a
 * GPIO pin.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioPinLevel
{
    ADI_ADRV9001_GPIO_PIN_LEVEL_LOW,
    ADI_ADRV9001_GPIO_PIN_LEVEL_HIGH,
} adi_adrv9001_GpioPinLevel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioSignal_e` is an enumeration that defines various
 * GPIO signals used in the ADRV9001 device for controlling and
 * interfacing with different functionalities. These signals include
 * digital GPIO functions for enabling and controlling channels, power
 * saving, frequency hopping, and ADC switching, as well as analog GPIO
 * functions for external frontend control, PLL lock and chip enable
 * signals, and auxiliary DAC and ADC controls. The enumeration provides
 * a comprehensive list of signals that can be mapped to GPIO pins,
 * facilitating the configuration and management of the device's GPIO
 * functionalities.
 *
 * @param ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1 ORx Enable signal channel 1.
 * @param ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_2 ORx Enable signal channel 2.
 * @param ADI_ADRV9001_GPIO_SIGNAL_MON_ENABLE_SPS Monitor mode enable and System
 * Power Saving request signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_MON_BBIC_WAKEUP Monitor mode signal to wake
 * up the BBIC.
 * @param ADI_ADRV9001_GPIO_SIGNAL_POWER_SAVING_CHANNEL1 Power saving signal for
 * channel 1.
 * @param ADI_ADRV9001_GPIO_SIGNAL_POWER_SAVING_CHANNEL2 Power saving signal for
 * channel 2.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_HOP Frequency hopping hop request signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_CH1_GAIN_ATTEN_SEL_0 Frequency hopping
 * Channel1 gain/atten
 * select bit 0.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_CH1_GAIN_ATTEN_SEL_1 Frequency hopping
 * Channel1 gain/atten
 * select bit 1.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_CH1_GAIN_ATTEN_SEL_2 Frequency hopping
 * Channel1 gain/atten
 * select bit 2.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_CH2_GAIN_ATTEN_SEL_0 Frequency hopping
 * Channel2 gain/atten
 * select bit 0.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_CH2_GAIN_ATTEN_SEL_1 Frequency hopping
 * Channel2 gain/atten
 * select bit 1.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_CH2_GAIN_ATTEN_SEL_2 Frequency hopping
 * Channel2 gain/atten
 * select bit 2.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_0 Frequency hopping frequency
 * index select bit 0.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_1 Frequency hopping frequency
 * index select bit 1.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_2 Frequency hopping frequency
 * index select bit 2.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_3 Frequency hopping frequency
 * index select bit 3.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_4 Frequency hopping frequency
 * index select bit 4.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_5 Frequency hopping frequency
 * index select bit 5.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_HOP_TABLE_SELECT Frequency hopping Hop
 * table select signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_TX1_PA_RAMP_CTRL Tx1 Aux DAC ramp control
 * request signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_TX2_PA_RAMP_CTRL Tx2 Aux DAC ramp control
 * request signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_ADC_SWITCHING_CHANNEL1 ADC switching gpio
 * based data capture for
 * Channel 1.
 * @param ADI_ADRV9001_GPIO_SIGNAL_ADC_SWITCHING_CHANNEL2 ADC switching gpio
 * based data capture for
 * Channel 2.
 * @param ADI_ADRV9001_GPIO_SIGNAL_TX1_EXT_FRONTEND_CONTROL Tx1 FrontEnd control
 * signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_TX2_EXT_FRONTEND_CONTROL Tx2 FrontEnd control
 * signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_RX1_EXT_FRONTEND_CONTROL Rx1 FrontEnd control
 * signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_RX2_EXT_FRONTEND_CONTROL Rx2 FrontEnd control
 * signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_1_LOCK External PLL 1 lock signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_2_LOCK External PLL 2 lock signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_1_CE External PLL 1 Chip Enable
 * signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_2_CE External PLL 2 Chip Enable
 * signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_RX_VCO_1_CE Rx VCO 1 Chip Enable signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_RX_VCO_2_CE Rx VCO 2 Chip Enable signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_0 Aux DAC control 0 signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_1 Aux DAC control 1 signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_2 Aux DAC control 2 signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_3 Aux DAC control 3 signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_0 Aux ADC control 0 signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_1 Aux ADC control 1 signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_2 Aux ADC control 2 signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_3 Aux ADC control 3 signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_HOP_2 Frequency hopping hop request
 * signal.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_HOP_2_TABLE_SELECT Frequency hopping table
 * select for HOP 2.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_HOP1_NCO_ASYNC_CHANGE Asynchronously
 * change NCO for Hop1.
 * @param ADI_ADRV9001_GPIO_SIGNAL_FH_HOP2_NCO_ASYNC_CHANGE Asynchronously
 * change NCO for Hop2.
 * @param ADI_ADRV9001_GPIO_SIGNAL_RX1_INTERFACEGAIN_SEED_SAVE Seed (rising
 * edge) or save
 * (falling edge)
 * the RX1
 * InterfaceGain.
 * @param ADI_ADRV9001_GPIO_SIGNAL_RX2_INTERFACEGAIN_SEED_SAVE Seed (rising
 * edge) or save
 * (falling edge)
 * the RX2
 * InterfaceGain.
 * @param ADI_ADRV9001_GPIO_NUM_SIGNALS Total Number of signals from BBIC.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioSignal
{
    /* Digital GPIO Functions */
    ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_1,                  /*!< ORx Enable signal channel 1 */
    ADI_ADRV9001_GPIO_SIGNAL_ORX_ENABLE_2,                  /*!< ORx Enable signal channel 2 */
    ADI_ADRV9001_GPIO_SIGNAL_MON_ENABLE_SPS,                /*!< Monitor mode enable and System Power Saving request signal */
    ADI_ADRV9001_GPIO_SIGNAL_MON_BBIC_WAKEUP,               /*!< Monitor mode signal to wake up the BBIC */
    ADI_ADRV9001_GPIO_SIGNAL_POWER_SAVING_CHANNEL1,         /*!< Power saving signal for channel 1 */
    ADI_ADRV9001_GPIO_SIGNAL_POWER_SAVING_CHANNEL2,         /*!< Power saving signal for channel 2 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_HOP,                        /*!< Frequency hopping hop request signal */
    ADI_ADRV9001_GPIO_SIGNAL_FH_CH1_GAIN_ATTEN_SEL_0,       /*!< Frequency hopping Channel1 gain/atten select bit 0 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_CH1_GAIN_ATTEN_SEL_1,       /*!< Frequency hopping Channel1 gain/atten select bit 1 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_CH1_GAIN_ATTEN_SEL_2,       /*!< Frequency hopping Channel1 gain/atten select bit 2 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_CH2_GAIN_ATTEN_SEL_0,       /*!< Frequency hopping Channel2 gain/atten select bit 0 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_CH2_GAIN_ATTEN_SEL_1,       /*!< Frequency hopping Channel2 gain/atten select bit 1 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_CH2_GAIN_ATTEN_SEL_2,       /*!< Frequency hopping Channel2 gain/atten select bit 2 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_0,              /*!< Frequency hopping frequency index select bit 0 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_1,              /*!< Frequency hopping frequency index select bit 1 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_2,              /*!< Frequency hopping frequency index select bit 2 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_3,              /*!< Frequency hopping frequency index select bit 3 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_4,              /*!< Frequency hopping frequency index select bit 4 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_TABLE_INDEX_5,              /*!< Frequency hopping frequency index select bit 5 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_HOP_TABLE_SELECT,           /*!< Frequency hopping Hop table select signal */
    ADI_ADRV9001_GPIO_SIGNAL_TX1_PA_RAMP_CTRL,              /*!< Tx1 Aux DAC ramp control request signal*/
    ADI_ADRV9001_GPIO_SIGNAL_TX2_PA_RAMP_CTRL,              /*!< Tx2 Aux DAC ramp control request signal*/

    ADI_ADRV9001_GPIO_SIGNAL_ADC_SWITCHING_CHANNEL1,        /*!< ADC switching gpio based data capture for Channel 1 */
    ADI_ADRV9001_GPIO_SIGNAL_ADC_SWITCHING_CHANNEL2,        /*!< ADC switching gpio based data capture for Channel 2 */

    /* Analog GPIO Functions */
    ADI_ADRV9001_GPIO_SIGNAL_TX1_EXT_FRONTEND_CONTROL,      /*!< Tx1 FrontEnd control signal */
    ADI_ADRV9001_GPIO_SIGNAL_TX2_EXT_FRONTEND_CONTROL,      /*!< Tx2 FrontEnd control signal */
    ADI_ADRV9001_GPIO_SIGNAL_RX1_EXT_FRONTEND_CONTROL,      /*!< Rx1 FrontEnd control signal */
    ADI_ADRV9001_GPIO_SIGNAL_RX2_EXT_FRONTEND_CONTROL,      /*!< Rx2 FrontEnd control signal */
    ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_1_LOCK,                /*!< External PLL 1 lock signal */
    ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_2_LOCK,                /*!< External PLL 2 lock signal */
    ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_1_CE,                  /*!< External PLL 1 Chip Enable signal */
    ADI_ADRV9001_GPIO_SIGNAL_EXT_PLL_2_CE,                  /*!< External PLL 2 Chip Enable signal */
    ADI_ADRV9001_GPIO_SIGNAL_RX_VCO_1_CE,                   /*!< Rx VCO 1 Chip Enable signal */
    ADI_ADRV9001_GPIO_SIGNAL_RX_VCO_2_CE,                   /*!< Rx VCO 2 Chip Enable signal */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_0,                     /*!< Aux DAC control 0 signal */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_1,                     /*!< Aux DAC control 1 signal */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_2,                     /*!< Aux DAC control 2 signal */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_DAC_3,                     /*!< Aux DAC control 3 signal */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_0,                     /*!< Aux ADC control 0 signal */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_1,                     /*!< Aux ADC control 1 signal */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_2,                     /*!< Aux ADC control 2 signal */
    ADI_ADRV9001_GPIO_SIGNAL_AUX_ADC_3,                     /*!< Aux ADC control 3 signal */

    ADI_ADRV9001_GPIO_SIGNAL_FH_HOP_2,                      /*!< Frequency hopping hop request signal   */
    ADI_ADRV9001_GPIO_SIGNAL_FH_HOP_2_TABLE_SELECT,         /*!< Frequency hopping table select for HOP 2 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_HOP1_NCO_ASYNC_CHANGE,      /*!< Asynchronously change NCO for Hop1 */
    ADI_ADRV9001_GPIO_SIGNAL_FH_HOP2_NCO_ASYNC_CHANGE,      /*!< Asynchronously change NCO for Hop2 */
    ADI_ADRV9001_GPIO_SIGNAL_RX1_INTERFACEGAIN_SEED_SAVE = 52,   /*!< Seed (rising edge) or save (falling edge) the RX1 InterfaceGain */
    ADI_ADRV9001_GPIO_SIGNAL_RX2_INTERFACEGAIN_SEED_SAVE = 53,   /*!< Seed (rising edge) or save (falling edge) the RX2 InterfaceGain  */
	
    ADI_ADRV9001_GPIO_NUM_SIGNALS = 54,                     /*!< Total Number of signals from BBIC*/
} adi_adrv9001_GpioSignal_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_gpIntStatus_t` structure is used to manage and track
 * the status of general-purpose interrupts in the ADRV9001 device. It
 * contains fields to hold the current interrupt status, mask specific
 * interrupts, identify active interrupt sources, and save interrupt
 * request masks. This structure is essential for handling interrupt-
 * driven events and ensuring that only the desired interrupts are
 * processed by the system.
 *
 * @param gpIntStatus Holds the current status of the general-purpose interrupt.
 * @param gpIntMask Specifies which interrupts are masked.
 * @param gpIntActiveSources Indicates the active sources of the interrupt.
 * @param gpIntSaveIrqMask Stores the mask for saving interrupt requests.
 ******************************************************************************/
typedef struct adi_adrv9001_gpIntStatus
{
    uint32_t gpIntStatus;
    uint32_t gpIntMask;
    uint32_t gpIntActiveSources;
    uint32_t gpIntSaveIrqMask;
} adi_adrv9001_gpIntStatus_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioPolarity_e` is an enumeration that defines the
 * polarity settings for GPIO pins on the ADRV9001 device. It provides
 * two options: normal polarity and inverted polarity, allowing users to
 * configure the signal behavior of the GPIO pins according to their
 * application needs.
 *
 * @param ADI_ADRV9001_GPIO_POLARITY_NORMAL Represents normal polarity with a
 * value of 0x00.
 * @param ADI_ADRV9001_GPIO_POLARITY_INVERTED Represents inverted polarity with
 * a value of 0x01.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioPolarity
{
    ADI_ADRV9001_GPIO_POLARITY_NORMAL = 0x00,      /*!< Normal polarity */
    ADI_ADRV9001_GPIO_POLARITY_INVERTED = 0x01,    /*!< Inverted polarity */
} adi_adrv9001_GpioPolarity_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioMaster_e` is an enumeration that defines the
 * possible masters for controlling the GPIO pins on the ADRV9001 device.
 * It allows the selection between the Baseband IC (BBIC) and the
 * ADRV9001 itself as the controlling entity for the GPIO operations,
 * providing flexibility in GPIO management and configuration.
 *
 * @param ADI_ADRV9001_GPIO_MASTER_BBIC Represents the BBIC as the GPIO master
 * with a value of 0x00.
 * @param ADI_ADRV9001_GPIO_MASTER_ADRV9001 Represents the ADRV9001 as the GPIO
 * master with a value of 0x02.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioMaster
{
    ADI_ADRV9001_GPIO_MASTER_BBIC = 0x00,
    ADI_ADRV9001_GPIO_MASTER_ADRV9001 = 0x02
} adi_adrv9001_GpioMaster_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioCfg_t` structure is used to configure a GPIO pin
 * on the ADRV9001 device. It includes fields to specify the pin, its
 * polarity, and the controlling master (either BBIC or ADRV9001). This
 * configuration is essential for setting up the GPIO pins to function
 * correctly according to the desired control and signal requirements.
 *
 * @param pin The GPIO pin.
 * @param polarity Polarity of the GPIO pin (normal or inverted).
 * @param master Indicates whether BBIC or ADRV9001 controls this pin.
 ******************************************************************************/
typedef struct adi_adrv9001_GpioCfg
{
    adi_adrv9001_GpioPin_e pin;                 /*!< The GPIO pin */
    adi_adrv9001_GpioPolarity_e polarity;       /*!< Polarity of the GPIO pin (normal or inverted) */
    adi_adrv9001_GpioMaster_e master;           /*!< Whether BBIC or ADRV9001 controls this pin
    adi_adrv9001_GpioMaster_e master;            *   TODO: JS: Can't this be inferred from the signal and/or direction (in/out)? */
} adi_adrv9001_GpioCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioCtrlInitCfg_t` structure is used to initialize
 * the GPIO control configurations for the ADRV9001 device. It includes
 * arrays of GPIO configurations for various control signals such as Tx
 * and Rx FrontEnd controls, external PLL chip enables, VCO chip enables,
 * and PLL locks, each corresponding to a specific port. Additionally, it
 * contains configurations for channel power saving, system power saving
 * and monitoring, frequency hop frame updates, and interface gain
 * seed/save operations, each tailored to specific channels or system-
 * wide functions.
 *
 * @param tx_ext_frontend_ctrl An array of GPIO configurations for Tx FrontEnd
 * controls, one for each port.
 * @param rx_ext_frontend_ctrl An array of GPIO configurations for Rx FrontEnd
 * controls, one for each port.
 * @param ext_pll_chip_enable An array of GPIO configurations for enabling
 * external PLL chips, one for each port.
 * @param vco_chip_enable An array of GPIO configurations for enabling Rx VCO
 * chips, one for each port.
 * @param ext_pll_lock An array of GPIO configurations for external PLL lock
 * signals, one for each port.
 * @param channelPowerSaving An array of GPIO configurations for channel power
 * saving enables, one for each channel.
 * @param systemPowerSavingAndMonitorEnable A GPIO configuration for system
 * power saving and monitor enable.
 * @param systemPowerSavingAndMonitorWakeUp A GPIO configuration for monitor
 * wake-up.
 * @param fh_update_rx_nco An array of GPIO configurations to trigger updates of
 * Rx NCO in frequency hop frames, one for each channel.
 * @param rx_interfaceGain_seed_save An array of GPIO configurations to trigger
 * seed or save of interface gain, one for
 * each channel.
 ******************************************************************************/
typedef struct adi_adrv9001_GpioCtrlInitCfg
{
    adi_adrv9001_GpioCfg_t tx_ext_frontend_ctrl[ADI_ADRV9001_NUM_PORTS];            /*!< (AGPIO) Tx FrontEnd controls */
    adi_adrv9001_GpioCfg_t rx_ext_frontend_ctrl[ADI_ADRV9001_NUM_PORTS];            /*!< (AGPIO) Rx FrontEnd controls */
    adi_adrv9001_GpioCfg_t ext_pll_chip_enable[ADI_ADRV9001_NUM_PORTS];             /*!< (AGPIO) External PLL Chip Enables */
    adi_adrv9001_GpioCfg_t vco_chip_enable[ADI_ADRV9001_NUM_PORTS];                 /*!< (AGPIO) Rx VCO Chip Enables */
    adi_adrv9001_GpioCfg_t ext_pll_lock[ADI_ADRV9001_NUM_PORTS];                    /*!< (AGPIO) External PLL locks */
    adi_adrv9001_GpioCfg_t channelPowerSaving[ADI_ADRV9001_NUM_CHANNELS];           /*!< (DGPIO) Channel Power Saving Enables */
    adi_adrv9001_GpioCfg_t systemPowerSavingAndMonitorEnable;                       /*!< (DGPIO) System Power Saving and Monitor Enable */
    adi_adrv9001_GpioCfg_t systemPowerSavingAndMonitorWakeUp;                       /*!< (DGPIO) Monitor WakeUp */
	adi_adrv9001_GpioCfg_t fh_update_rx_nco[ADI_ADRV9001_NUM_CHANNELS];             /*!< (DGPIO) Trigger Update of Rx NCO in Frequency Hop Frame */
    adi_adrv9001_GpioCfg_t rx_interfaceGain_seed_save[ADI_ADRV9001_NUM_CHANNELS];   /*!< (DGPIO) Trigger Seed (RisingEdge) or Save (FallingEdge) of interfaceGain */

} adi_adrv9001_GpioCtrlInitCfg_t;

#endif /* _ADI_ADRV9001_GPIO_TYPES_H_ */
