/*!
* \file
* \brief Contains ADRV9001 API configuration and run-time type definitions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_TYPES_H_
#define _ADI_ADRV9001_TYPES_H_

#include "adi_adrv9001_defines.h"
#include "adi_adrv9001_profile_types.h"

#include "adi_adrv9001_common.h"
#include "adi_common.h"

#define ADI_ADRV9001_NUM_TXRX_CHANNELS         0x4
#define ADI_ADRV9001_NUM_RX_CHANNELS           0x2
#define ADI_ADRV9001_NUM_TX_CHANNELS           0x2

#define ADI_ADRV9001_TX_PROFILE_VALID		  0x01
#define ADI_ADRV9001_RX_PROFILE_VALID		  0x02
#define ADI_ADRV9001_ORX_PROFILE_VALID		  0x04
#define ADI_ADRV9001_ELB_PROFILE_VALID		  0x08

#define ADI_ADRV9001_NUM_CHANNELS 2
#define ADI_ADRV9001_NUM_PORTS 2

#define ADI_ADRV9001_MAX_AUXDACS              4U

/* TODO: Determine a reasonable value */
#define ADI_ADRV9001_READY_FOR_MCS_DELAY_US 100U

#define ADI_ADRV9001_WB_MAX_NUM_UNIQUE_CALS 156
#define ADI_ADRV9001_WB_MAX_NUM_VECTOR_TABLE_WORDS 624
#define ADI_ADRV9001_WB_MAX_NUM_VECTOR_TABLE_BYTES 2496
#define ADI_ADRV9001_WB_MAX_NUM_ENTRY 16384
#define ADI_ADRV9001_WB_MAX_NUM_COEFF 6000

/***************************************************************************//**
 * @brief The `adi_adrv9001_PartNumber_e` is an enumeration that defines the
 * possible part numbers for the ADRV9001 series of devices. It includes
 * specific identifiers for each part number, such as ADRV9002, ADRV9003,
 * and ADRV9004, as well as a value for unknown part numbers. This
 * enumeration is used to specify and differentiate between the various
 * models within the ADRV9001 series.
 *
 * @param ADI_ADRV9001_PART_NUMBER_UNKNOWN Represents an unknown part number
 * with a value of -1.
 * @param ADI_ADRV9001_PART_NUMBER_ADRV9002 Represents the ADRV9002 part number
 * with a value of 0x0.
 * @param ADI_ADRV9001_PART_NUMBER_ADRV9003 Represents the ADRV9003 part number
 * with a value of 0xC.
 * @param ADI_ADRV9001_PART_NUMBER_ADRV9004 Represents the ADRV9004 part number
 * with a value of 0x8.
 ******************************************************************************/
typedef enum adi_adrv9001_PartNumber
{
    ADI_ADRV9001_PART_NUMBER_UNKNOWN    = -1,
    ADI_ADRV9001_PART_NUMBER_ADRV9002   = 0x0,
    ADI_ADRV9001_PART_NUMBER_ADRV9003   = 0xC,
    ADI_ADRV9001_PART_NUMBER_ADRV9004   = 0x8,
} adi_adrv9001_PartNumber_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_MailboxChannel_e` is an enumeration that defines
 * various channel identifiers for the ADRV9001 device, each associated
 * with a unique hexadecimal value. These channels include RX, TX, ORX,
 * ILB, and ELB channels, which are used to specify different
 * communication paths within the device. The enumeration allows for easy
 * reference and manipulation of these channels in the device's API,
 * facilitating operations such as configuration and data transmission.
 *
 * @param ADI_ADRV9001_RX1 Represents the RX1 channel with a value of 0x0001.
 * @param ADI_ADRV9001_RX2 Represents the RX2 channel with a value of 0x0002.
 * @param ADI_ADRV9001_TX1 Represents the TX1 channel with a value of 0x0004.
 * @param ADI_ADRV9001_TX2 Represents the TX2 channel with a value of 0x0008.
 * @param ADI_ADRV9001_ORX1 Represents the ORX1 channel with a value of 0x0010.
 * @param ADI_ADRV9001_ORX2 Represents the ORX2 channel with a value of 0x0020.
 * @param ADI_ADRV9001_ILB1 Represents the ILB1 channel with a value of 0x0040.
 * @param ADI_ADRV9001_ILB2 Represents the ILB2 channel with a value of 0x0080.
 * @param ADI_ADRV9001_ELB1 Represents the ELB1 channel with a value of 0x0100.
 * @param ADI_ADRV9001_ELB2 Represents the ELB2 channel with a value of 0x0200.
 ******************************************************************************/
typedef enum adi_adrv9001_MailboxChannel
{
    ADI_ADRV9001_RX1  = 0x0001,
    ADI_ADRV9001_RX2  = 0x0002,
    ADI_ADRV9001_TX1  = 0x0004,
    ADI_ADRV9001_TX2  = 0x0008,
    ADI_ADRV9001_ORX1 = 0x0010,
    ADI_ADRV9001_ORX2 = 0x0020,
    ADI_ADRV9001_ILB1 = 0x0040,
    ADI_ADRV9001_ILB2 = 0x0080,
    ADI_ADRV9001_ELB1 = 0x0100,
    ADI_ADRV9001_ELB2 = 0x0200
} adi_adrv9001_MailboxChannel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DeviceClockDivisor_e` is an enumeration that defines
 * possible values for the device clock divisor in the ADRV9001 API. It
 * provides options to set the clock divisor to various powers of two,
 * ranging from bypass (no division) to 64, as well as an option to
 * disable the device clock output entirely. This enumeration is used to
 * configure the clock settings of the ADRV9001 device, allowing for
 * flexible clock management based on application requirements.
 *
 * @param ADI_ADRV9001_DEVICECLOCKDIVISOR_BYPASS Represents a bypass mode for
 * the device clock divisor.
 * @param ADI_ADRV9001_DEVICECLOCKDIVISOR_2 Sets the device clock divisor to 2.
 * @param ADI_ADRV9001_DEVICECLOCKDIVISOR_4 Sets the device clock divisor to 4.
 * @param ADI_ADRV9001_DEVICECLOCKDIVISOR_8 Sets the device clock divisor to 8.
 * @param ADI_ADRV9001_DEVICECLOCKDIVISOR_16 Sets the device clock divisor to
 * 16.
 * @param ADI_ADRV9001_DEVICECLOCKDIVISOR_32 Sets the device clock divisor to
 * 32.
 * @param ADI_ADRV9001_DEVICECLOCKDIVISOR_64 Sets the device clock divisor to
 * 64.
 * @param ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED Disables the device clock
 * output.
 ******************************************************************************/
typedef enum adi_adrv9001_DeviceClockDivisor
{
    ADI_ADRV9001_DEVICECLOCKDIVISOR_BYPASS   = 0,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_2        = 1,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_4        = 2,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_8        = 3,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_16       = 4,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_32       = 5,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_64       = 6,
    ADI_ADRV9001_DEVICECLOCKDIVISOR_DISABLED = 7 /* Arbitrary value, just to select in case to disable device clock output */
} adi_adrv9001_DeviceClockDivisor_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ApiStates_e` is an enumeration that defines the
 * various operational states of the ADRV9001 device API. Each state
 * represents a specific stage in the device's initialization and
 * operational lifecycle, from power-on reset to standby mode. This
 * enumeration is crucial for managing and understanding the device's
 * current status and ensuring that the device transitions through its
 * states correctly during operation.
 *
 * @param ADI_ADRV9001_STATE_POWERON_RESET Represents the state of the device
 * immediately after power-on reset.
 * @param ADI_ADRV9001_STATE_ANA_INITIALIZED Indicates that the analog
 * components of the device have been
 * initialized.
 * @param ADI_ADRV9001_STATE_DIG_INITIALIZED Indicates that the digital
 * components of the device have been
 * initialized.
 * @param ADI_ADRV9001_STATE_STREAM_LOADED Represents the state where the stream
 * processor has been loaded.
 * @param ADI_ADRV9001_STATE_ARM_DEBUG_LOADED Indicates that the ARM debug
 * components have been loaded.
 * @param ADI_ADRV9001_STATE_ARM_LOADED Indicates that the ARM processor has
 * been loaded.
 * @param ADI_ADRV9001_STATE_INITCALS_RUN Represents the state where initial
 * calibrations have been run.
 * @param ADI_ADRV9001_STATE_PRIMED Indicates that the device is primed and
 * ready for operation.
 * @param ADI_ADRV9001_STATE_IDLE Represents the idle state of the device.
 * @param ADI_ADRV9001_STATE_STANDBY Indicates that the device is in standby
 * mode.
 ******************************************************************************/
typedef enum adi_adrv9001_ApiStates
{
    ADI_ADRV9001_STATE_POWERON_RESET    = 0x00,
    ADI_ADRV9001_STATE_ANA_INITIALIZED  = 0x01,
    ADI_ADRV9001_STATE_DIG_INITIALIZED  = 0x02,
    ADI_ADRV9001_STATE_STREAM_LOADED    = 0x04,
    ADI_ADRV9001_STATE_ARM_DEBUG_LOADED = 0x08,
    ADI_ADRV9001_STATE_ARM_LOADED       = 0x10,
    ADI_ADRV9001_STATE_INITCALS_RUN     = 0x20,
    ADI_ADRV9001_STATE_PRIMED           = 0x40,
    ADI_ADRV9001_STATE_IDLE             = 0x80,
    ADI_ADRV9001_STATE_STANDBY          = 0x100,
} adi_adrv9001_ApiStates_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_CmosPadDrvStr_e` is an enumeration that defines the
 * drive strength options for the CMOS pads used in the ADRV9001 device,
 * specifically for the SPI_DO signal. It provides two options: a weak
 * drive strength for lighter loads and higher frequencies, and a strong
 * drive strength for heavier loads and lower frequencies. This allows
 * for flexibility in configuring the device to match the electrical
 * characteristics of the connected components.
 *
 * @param ADI_ADRV9001_CMOSPAD_DRV_WEAK Represents a weak drive strength
 * suitable for a 5pF load at 75MHz.
 * @param ADI_ADRV9001_CMOSPAD_DRV_STRONG Represents a strong drive strength
 * suitable for a 100pF load at 20MHz.
 ******************************************************************************/
typedef enum adi_adrv9001_CmosPadDrvStr
{
    ADI_ADRV9001_CMOSPAD_DRV_WEAK   = 0,	/*!<    5pF load @ 75MHz */
    ADI_ADRV9001_CMOSPAD_DRV_STRONG = 1		/*!<  100pF load @ 20MHz */
} adi_adrv9001_CmosPadDrvStr_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxRxLoSel_e` is an enumeration that defines the
 * options for selecting the local oscillator (LO) source for the
 * transmit (Tx) and receive (Rx) paths in the ADRV9001 device. It
 * includes options for selecting between two receive LOs, two transmit
 * LOs, and an auxiliary LO. This enumeration is used to configure the LO
 * source for the mixers in the device, which is crucial for setting the
 * correct frequency for signal transmission and reception.
 *
 * @param ADI_ADRV9001_TXRXLOSEL_RX_LO1 Represents the selection of the first
 * receive local oscillator (LO1) for the
 * Rx path.
 * @param ADI_ADRV9001_TXRXLOSEL_RX_LO2 Represents the selection of the second
 * receive local oscillator (LO2) for the
 * Rx path.
 * @param ADI_ADRV9001_TXRXLOSEL_TX_LO1 Represents the selection of the first
 * transmit local oscillator (LO1) for the
 * Tx path.
 * @param ADI_ADRV9001_TXRXLOSEL_TX_LO2 Represents the selection of the second
 * transmit local oscillator (LO2) for the
 * Tx path.
 * @param ADI_ADRV9001_TXRXLOSEL_AUXLO Represents the selection of an auxiliary
 * local oscillator (AUXLO).
 ******************************************************************************/
typedef enum adi_adrv9001_TxRxLoSel
{
    ADI_ADRV9001_TXRXLOSEL_RX_LO1 = 1,
    ADI_ADRV9001_TXRXLOSEL_RX_LO2 = 1,
    ADI_ADRV9001_TXRXLOSEL_TX_LO1 = 1,
    ADI_ADRV9001_TXRXLOSEL_TX_LO2 = 1,
    ADI_ADRV9001_TXRXLOSEL_AUXLO  = 2
} adi_adrv9001_TxRxLoSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_FirGain_e` is an enumeration that defines various
 * gain levels for a Finite Impulse Response (FIR) filter in the ADRV9001
 * device. Each enumerator represents a specific gain value in decibels
 * (dB), ranging from -12 dB to 26 dB, allowing for precise control over
 * the filter's amplification or attenuation.
 *
 * @param ADRV9001_FIR_GAIN_NEG_12_DB FIR gain set to -12 dB.
 * @param ADRV9001_FIR_GAIN_NEG_6_DB FIR gain set to -6 dB.
 * @param ADRV9001_FIR_GAIN_ZERO_DB FIR gain set to 0 dB.
 * @param ADRV9001_FIR_GAIN_POS_6_DB FIR gain set to 6 dB.
 * @param ADRV9001_FIR_GAIN_POS_9P54_DB FIR gain set to 9.54 dB.
 * @param ADRV9001_FIR_GAIN_POS_12_DB FIR gain set to 12 dB.
 * @param ADRV9001_FIR_GAIN_POS_14_DB FIR gain set to 14 dB.
 * @param ADRV9001_FIR_GAIN_POS_20_DB FIR gain set to 20 dB.
 * @param ADRV9001_FIR_GAIN_POS_24_DB FIR gain set to 24 dB.
 * @param ADRV9001_FIR_GAIN_POS_26_DB FIR gain set to 26 dB.
 ******************************************************************************/
typedef enum adi_adrv9001_FirGain
{
    ADRV9001_FIR_GAIN_NEG_12_DB   = -12, /*!< FIR gain -12 */
    ADRV9001_FIR_GAIN_NEG_6_DB    = -6,  /*!< FIR gain -6 */
    ADRV9001_FIR_GAIN_ZERO_DB     = 0,   /*!< FIR gain 0 */
    ADRV9001_FIR_GAIN_POS_6_DB    = 6,   /*!< FIR gain 6 */
    ADRV9001_FIR_GAIN_POS_9P54_DB = 9,   /*!< FIR gain 9.54 */
    ADRV9001_FIR_GAIN_POS_12_DB   = 12,  /*!< FIR gain 12 */
    ADRV9001_FIR_GAIN_POS_14_DB   = 14,  /*!< FIR gain 14 */
    ADRV9001_FIR_GAIN_POS_20_DB   = 20,  /*!< FIR gain 20 */
    ADRV9001_FIR_GAIN_POS_24_DB   = 24,  /*!< FIR gain 24 */
    ADRV9001_FIR_GAIN_POS_26_DB   = 26   /*!< FIR gain 26 */
} adi_adrv9001_FirGain_e;

/*
*********************************************************************************************************
*                                             Structure definition
*********************************************************************************************************
*/

/***************************************************************************//**
 * @brief The `adi_adrv9001_SpiSettings_t` structure is designed to configure
 * the SPI settings for ADRV9001 devices. It includes options for setting
 * the bit order of SPI transactions, enabling or disabling SPI streaming
 * mode, and determining the address increment direction during
 * streaming. Additionally, it allows the selection between 3-wire and
 * 4-wire SPI modes and specifies the drive strength of CMOS pads when
 * they are used as outputs. This structure is crucial for ensuring
 * proper communication and configuration of the ADRV9001 device via SPI.
 *
 * @param msbFirst Determines the bit order for SPI transactions, with 1 for MSB
 * first and 0 for LSB first.
 * @param enSpiStreaming Enables (1) or disables (0) ADRV9001 SPI streaming
 * mode.
 * @param autoIncAddrUp Sets the address increment direction for SPI streaming,
 * with 1 for incrementing and 0 for decrementing.
 * @param fourWireMode Specifies the SPI mode, with 1 for 4-wire and 0 for
 * 3-wire (SDIO pin is bidirectional).
 * @param cmosPadDrvStrength Defines the drive strength of CMOS pads when used
 * as outputs.
 ******************************************************************************/
typedef struct adi_adrv9001_SpiSettings
{
    uint8_t msbFirst;                           		/*!< 1 = MSB First, 0 = LSB First Bit order for SPI transaction */
    uint8_t enSpiStreaming;                     		/*!< 1 = ADRV9001 SPI streaming mode; 0 = Standard mode */
    uint8_t autoIncAddrUp;                      		/*!< For SPI Streaming, set address increment direction. 1= next addr = addr+1, 0:addr = addr-1 */
    uint8_t fourWireMode;                       		/*!< 1: Use 4-wire SPI, 0: 3-wire SPI (SDIO pin is bidirectional). NOTE: ADI's FPGA platform always uses 4-wire mode */
    adi_adrv9001_CmosPadDrvStr_e cmosPadDrvStrength;   	/*!< Drive strength of CMOS pads when used as outputs (SDIO, SDO, GP_INTERRUPT, GPIO 1, GPIO 0) */
} adi_adrv9001_SpiSettings_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ClkDivideRatios_t` structure is used to define
 * various clock divide ratios for different modules within the ADRV9001
 * device. Each field in the structure represents a specific module's
 * clock divide ratio, which is crucial for configuring the timing and
 * synchronization of the device's internal operations. These ratios are
 * relative to the high-speed digital clock (hsDigClk) or other reference
 * clocks, ensuring that each module operates at the correct frequency
 * for optimal performance.
 *
 * @param devClkDivideRatio Dev clock divide ratio.
 * @param refClkDivideRatio Ref clock divide ratio.
 * @param armClkDivideRatio ARM clock divide ratio with respect to hsDigClk.
 * @param agcClkDivideRatio AGC module clock divide ratio with respect to
 * hsDigClk.
 * @param regClkDivideRatio Register bus clock divide ratio with respect to
 * hsDigClk.
 * @param txAttenDeviceClockDivideRatio Tx Atten module clock divide ratio with
 * respect to hsDigClk.
 * @param anaRefClockRatio Analog clock divide ratio.
 ******************************************************************************/
typedef struct adi_adrv9001_ClkDivideRatios
{
    uint8_t devClkDivideRatio; /*!< Dev clock divide ratio */
    uint8_t refClkDivideRatio; /*!< Ref clock divide ratio */
    uint8_t armClkDivideRatio; /*!< ARM clock divide ratio w.r.t hsDigClk*/
    uint8_t agcClkDivideRatio; /*!< AGC module clock divide ratio w.r.t hsDigClk*/
    uint8_t regClkDivideRatio; /*!< Register bus clock divide ratio w.r.t hsDigClk*/
    uint8_t txAttenDeviceClockDivideRatio; /*!< Tx Atten module clock divide ratio w.r.t hsDigClk*/
    uint8_t anaRefClockRatio;  /*!< Analog clock divide ratio */
} adi_adrv9001_ClkDivideRatios_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SiliconVersion_t` structure is used to represent the
 * version of the ADRV9001 silicon. It contains two fields, `major` and
 * `minor`, which store the major and minor version numbers of the
 * silicon, respectively. This structure is useful for identifying the
 * specific version of the silicon being used, which can be important for
 * compatibility and feature support.
 *
 * @param major Major silicon version (0xA, 0xB, etc).
 * @param minor Minor silicon version.
 ******************************************************************************/
typedef struct adi_adrv9001_SiliconVersion
{
    uint8_t major; /*!< Major silicon version (0xA, 0xB, etc) */
    uint8_t minor; /*!< Minor silicon version */
} adi_adrv9001_SiliconVersion_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_Warmboot_Coeff_t` structure is designed to hold the
 * warmboot initialization calibration coefficients for the ADRV9001
 * device. It contains a two-dimensional array `calValue` that stores
 * calibration values, where the first dimension corresponds to the
 * maximum number of unique calibrations and the second dimension
 * corresponds to the maximum number of coefficients for each
 * calibration. This structure is essential for managing and applying
 * calibration data during the warmboot process of the device.
 *
 * @param calValue A 2D array storing calibration values with dimensions defined
 * by ADI_ADRV9001_WB_MAX_NUM_UNIQUE_CALS and
 * ADI_ADRV9001_WB_MAX_NUM_COEFF.
 ******************************************************************************/
typedef struct adi_adrv9001_Warmboot_Coeff
{
	uint8_t calValue[ADI_ADRV9001_WB_MAX_NUM_UNIQUE_CALS][ADI_ADRV9001_WB_MAX_NUM_COEFF];
} adi_adrv9001_Warmboot_Coeff_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_Warmboot_CalNumbers_t` structure is designed to
 * manage the initialization calibrations for the ADRV9001 device during
 * a warm boot process. It keeps track of the number of unique
 * calibrations enabled, the specific calibration numbers that are
 * active, and the memory required to handle the warm boot coefficients.
 * This structure is crucial for ensuring that the device can efficiently
 * resume operation with the correct calibration settings after a warm
 * boot.
 *
 * @param numberUniqueEnabledCals The number of unique initialization
 * calibrations enabled for this device
 * configuration.
 * @param calNumbersEnabled Array indicating the calibration number of each
 * enabled unique initialization calibration.
 * @param warmbootMemoryNumBytes Memory allocation required to store or retrieve
 * the Warmboot coefficients.
 ******************************************************************************/
typedef struct adi_adrv9001_Warmboot_CalNumbers {
	uint8_t numberUniqueEnabledCals;                                    /*!< The Number of unique initCals enabled for this device configuration */   
	uint8_t calNumbersEnabled[ADI_ADRV9001_WB_MAX_NUM_UNIQUE_CALS];     /*!< Array indicating the calNumber of each enabled unique initCal */   
	uint32_t warmbootMemoryNumBytes;                                    /*!< Memory allocation required to store/retrieve the Warmboot coefficients */   
} adi_adrv9001_Warmboot_CalNumbers_t;

#ifndef CLIENT_IGNORE
/***************************************************************************//**
 * @brief The `adi_adrv9001_Info_t` structure is a comprehensive data structure
 * used to hold the state and configuration information of an ADRV9001
 * device instance. It includes fields for device identification, state,
 * and configuration settings such as initialized channels, tracking
 * calibration masks, and profile validity. Additionally, it contains
 * information about signal types, clock settings, and frequency hopping
 * configurations. This structure is essential for managing the runtime
 * state and settings of the ADRV9001 device, facilitating operations
 * such as initialization, calibration, and profile management.
 *
 * @param deviceSiRev ADRV9001 silicon revision read during initialization.
 * @param deviceProductId ADRV9001 Product ID read during hardware open.
 * @param devState Current device state, defined by deviceState enum.
 * @param initializedChannels Channels that were initialized and calibrated for
 * the current device.
 * @param trackingCalEnableMask Tracking calibration mask of Rx and ORx as set
 * by the customer.
 * @param profilesValid Bit field indicating valid profiles for use
 * notification.
 * @param rxOutputSignaling Rx output to analog signal type.
 * @param txOutputSignaling Tx output to analog signal type.
 * @param swTest Software test mode signal.
 * @param hsDigClk_Hz Digital clock frequency used throughout API functions.
 * @param deviceClock_kHz Device clock frequency in kHz.
 * @param clkPllMode CLKPLL mode.
 * @param clkDivideRatios Clock divide ratios for various modules in the device.
 * @param profileAddr Address to load profile.
 * @param adcProfileAddr Address to load ADC profile.
 * @param pfirProfileAddr Address to load PFIR coefficients.
 * @param fhHopTableA1Addr Address to load hop table A1 in frequency hopping
 * mode.
 * @param fhHopTableB1Addr Address to load hop table B1 in frequency hopping
 * mode.
 * @param fhHopTableA2Addr Address to load hop table A2 in frequency hopping
 * mode.
 * @param fhHopTableB2Addr Address to load hop table B2 in frequency hopping
 * mode.
 * @param fhHopTableBufferA1Addr Address to load hop table A1 buffer in
 * frequency hopping mode.
 * @param fhHopTableBufferB1Addr Address to load hop table B1 buffer in
 * frequency hopping mode.
 * @param fhHopTableBufferA2Addr Address to load hop table A2 buffer in
 * frequency hopping mode.
 * @param fhHopTableBufferB2Addr Address to load hop table B2 buffer in
 * frequency hopping mode.
 * @param txInputRate_kHz Tx input sample rate from currently loaded profile.
 * @param rxOutputRate_kHz Rx output sample rate from currently loaded profile.
 * @param rx1InterfaceSampleRate_kHz Rx1 interface sample rate from currently
 * loaded profile.
 * @param chunkStreamImageSize Stream image size.
 * @param chunkStreamImageOffset Stream image offset.
 * @param currentStreamBinBaseAddr Address to load current stream binary.
 * @param currentStreamBaseAddr Address to load current stream base.
 * @param currentStreamNumberStreams Number of streams for current stream.
 * @param currentStreamImageIndex Index of current stream.
 * @param currentStreamImageSize Image size of current stream.
 * @param frequencyHoppingEnabled Flag indicating if frequency hopping is
 * enabled.
 * @param gainTableType Type of gain table loaded during initialization.
 * @param txAttenMode TX attenuation mode.
 * @param chProfEnMask Mask for enabled channels and profiles in firmware
 * format.
 ******************************************************************************/
typedef struct adi_adrv9001_Info
{
    uint8_t deviceSiRev;												/*!< ADRV9001 silicon rev read during ADRV9001_initialize */
    uint8_t deviceProductId;                                            /*!< ADRV9001 Product ID read during HWOpen */
    uint16_t devState;									                /*!< Current device state of the part, i.e., radio on, radio off, arm loaded,
                                                                             etc., defined by deviceState enum */
    uint32_t initializedChannels;										/*!< Holds Rx/ORx/Tx channels that were initialized and calibrated for the
                                                                             current device */
    uint32_t trackingCalEnableMask;										/*!< Keeps track of tracking calibration mask of Rx and ORx as set by the
                                                                             customer through the api */
    uint32_t profilesValid;												/*!< Current device profilesValid bit field for use notification, i.e.,
                                                                             Tx = 0x01, Rx = 0x02, Orx = 0x04 */
    adi_adrv9001_RxSignalType_e rxOutputSignaling[ADI_ADRV9001_MAX_RX_ONLY];      /*!< Rx Output to Analog signal type */
    adi_adrv9001_TxSignalType_e txOutputSignaling[ADI_ADRV9001_MAX_TXCHANNELS];   /*!< Tx Output to Analog signal type */
    uint32_t swTest;													/*!< Software testmode signal */
    uint32_t hsDigClk_Hz;												/*!< Calculated in initialize() digital clock used throughout API functions */
    uint32_t deviceClock_kHz;                                           /*!< Device clock frequency in kHz (copy from adi_adrv9001_ClockSettings_t struct) */
    adi_adrv9001_ClkPllMode_e  clkPllMode;                              /*!< CLKPLL Mode */
    adi_adrv9001_ClkDivideRatios_t clkDivideRatios;						/*!< Clock divide ratios w.r.t hsDigClk for various modules in the device */
    uint32_t  profileAddr;												/*!< Address to load Profile */
    uint32_t  adcProfileAddr;											/*!< Address to load ADC Profile */
    uint32_t  pfirProfileAddr;                                          /*!< Address to load PFIR coefficients */
    uint32_t  fhHopTableA1Addr;                                          /*!< Address to load hop table A1 in frequency hopping mode */
    uint32_t  fhHopTableB1Addr;                                          /*!< Address to load hop table B1 in frequency hopping mode */
	uint32_t  fhHopTableA2Addr;                                          /*!< Address to load hop table A2 in frequency hopping mode */
	uint32_t  fhHopTableB2Addr;                                          /*!< Address to load hop table B2 in frequency hopping mode */
	uint32_t  fhHopTableBufferA1Addr;                                    /*!< Address to load hop table A1 Buffer in frequency hopping mode */
	uint32_t  fhHopTableBufferB1Addr;                                    /*!< Address to load hop table B1 Buffer in frequency hopping mode */
	uint32_t  fhHopTableBufferA2Addr;                                    /*!< Address to load hop table A2 Buffer in frequency hopping mode */
	uint32_t  fhHopTableBufferB2Addr;                                   /*!< Address to load hop table B2 Buffer in frequency hopping mode */
    uint32_t txInputRate_kHz[ADI_ADRV9001_MAX_TXCHANNELS];				/*!< Tx Input sample rate from currently loaded profile */
    uint32_t rxOutputRate_kHz[ADI_ADRV9001_MAX_RXCHANNELS];				/*!< Rx Output sample rate from currently loaded profile */
    uint32_t rx1InterfaceSampleRate_kHz;                                /*!< Rx1 Interface sample rate from currently loaded profile */
    uint16_t chunkStreamImageSize[12];									/*!< Stream Image Size */
    uint16_t chunkStreamImageOffset[12];								/*!< Stream Image Offset */
    uint32_t currentStreamBinBaseAddr;									/*!< Address to load current stream */
    uint32_t currentStreamBaseAddr;										/*!< Address to load current stream base */
    uint8_t currentStreamNumberStreams;									/*!< Number of Streams for current stream  */
    uint8_t currentStreamImageIndex;									/*!< Index of current stream  */
    uint32_t currentStreamImageSize;									/*!< Image size of current stream */
    uint8_t frequencyHoppingEnabled;                                    /*!< Frequency hopping enabled flag from currently loaded profile */
    adi_adrv9001_RxGainTableType_e gainTableType[ADI_ADRV9001_MAX_RX_ONLY]; /*!< type of gain table loaded during ADRV9001 initialization */
	uint8_t txAttenMode[ADI_ADRV9001_MAX_TXCHANNELS];					/* TX Attenuation Mode*/
	uint32_t chProfEnMask[ADI_ADRV9001_MAX_NUM_CHANNELS];				/* Mask for enabled channels and profiles in FW format - used for warmboot*/
} adi_adrv9001_Info_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_Device_t` structure is designed to encapsulate the
 * state and configuration settings of an ADRV9001 device instance. It
 * includes a common layer structure for shared device attributes, a
 * container for runtime state information specific to the ADRV9001, and
 * a pointer to the SPI settings used for communication with the device.
 * This structure is essential for managing the device's operational
 * state and ensuring proper configuration and communication.
 *
 * @param common Common layer structure.
 * @param devStateInfo ADRV9001 run time state information container.
 * @param spiSettings Pointer to ADRV9001 SPI Settings.
 ******************************************************************************/
typedef struct adi_adrv9001_Device
{
    adi_common_Device_t			common;        /*!< Common layer structure */
    adi_adrv9001_Info_t			devStateInfo;  /*!< ADRV9001 run time state information container */
    adi_adrv9001_SpiSettings_t	spiSettings;   /*!< Pointer to ADRV9001 SPI Settings */
} adi_adrv9001_Device_t;
#endif /* CLIENT_IGNORE */

#endif /* _ADI_ADRV9001_TYPES_H_ */
