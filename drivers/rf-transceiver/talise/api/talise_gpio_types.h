/**
 * \file talise_gpio_types.h
 * \brief Contains functions to allow control of the General Purpose IO functions on the Talise device
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_GPIO_TYPES_H_
#define TALISE_GPIO_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief The `taliseGpioMode_t` is an enumeration that defines the different
 * modes available for configuring the GPIO pins on a Talise device. Each
 * mode specifies a distinct operational behavior for the GPIO pins, such
 * as monitoring device state, manual control via API, ARM processor
 * output, or slicer configuration. This enumeration allows for flexible
 * control and configuration of GPIO functionalities to suit various
 * application needs.
 *
 * @param TAL_GPIO_MONITOR_MODE Allows a choice of debug signals to output from
 * Talise to monitor the state of the device.
 * @param TAL_GPIO_BITBANG_MODE Manual mode, API function sets output pin levels
 * and reads input pin levels.
 * @param TAL_GPIO_ARM_OUT_MODE Allows internal ARM processor to output on GPIO
 * pins.
 * @param TAL_GPIO_SLICER_OUT_MODE Allows Slicer active configuration to the
 * GPIO output pins.
 ******************************************************************************/
typedef enum {
	TAL_GPIO_MONITOR_MODE = 0,      /*!< Allows a choice of debug signals to output from Talise to monitor the state of the device */
	TAL_GPIO_BITBANG_MODE = 3,      /*!< Manual mode, API function sets output pin levels and reads input pin levels */
	TAL_GPIO_ARM_OUT_MODE = 9,      /*!< Allows internal ARM processor to output on GPIO pins */
	TAL_GPIO_SLICER_OUT_MODE = 10   /*!< Allows Slicer active configuration to the GPIO output  pins */
} taliseGpioMode_t;

/***************************************************************************//**
 * @brief The `taliseGpio3v3Mode_t` is an enumeration that defines the various
 * modes available for controlling the GPIO3v3 pins on the Talise device.
 * Each mode specifies a different behavior for the GPIO3v3 pins, such as
 * level translation, inverted level translation, manual bit-banging, or
 * following an external attenuation lookup table. This allows for
 * flexible configuration of the GPIO3v3 pins to suit different
 * application needs.
 *
 * @param TAL_GPIO3V3_LEVELTRANSLATE_MODE Level translate mode, signal level on
 * low voltage GPIO output on GPIO3v3
 * pins.
 * @param TAL_GPIO3V3_INVLEVELTRANSLATE_MODE Inverted Level translate mode,
 * inverse of signal level on low
 * voltage GPIO output on GPIO3v3
 * pins.
 * @param TAL_GPIO3V3_BITBANG_MODE Manual mode, API function sets output pin
 * levels and reads input pin levels.
 * @param TAL_GPIO3V3_EXTATTEN_LUT_MODE GPIO3v3 output level follows Rx1/Rx2
 * gain table external control 6bit field.
 ******************************************************************************/
typedef enum {
	TAL_GPIO3V3_LEVELTRANSLATE_MODE     = 1,    /*!< Level translate mode, signal level on low voltage GPIO output on GPIO3v3 pins */
	TAL_GPIO3V3_INVLEVELTRANSLATE_MODE  = 2,    /*!< Inverted Level translate mode, inverse of signal level on low voltage GPIO output on GPIO3v3 pins */
	TAL_GPIO3V3_BITBANG_MODE            = 3,    /*!< Manual mode, API function sets output pin levels and reads input pin levels */
	TAL_GPIO3V3_EXTATTEN_LUT_MODE       = 4     /*!< GPIO3v3 output level follows Rx1/Rx2 gain table external control 6bit field. */
} taliseGpio3v3Mode_t;

/***************************************************************************//**
 * @brief The `taliseGpioErr_t` is an enumeration that defines a comprehensive
 * set of error codes for the Talise GPIO API functions. Each error code
 * corresponds to a specific error condition that can occur during the
 * operation of GPIO functions, such as invalid parameters, null
 * parameters, or specific hardware-related issues like PLL unlocks or
 * watchdog timeouts. This enumeration facilitates debugging by providing
 * unique identifiers for each error scenario, allowing developers to
 * quickly identify and address issues within the GPIO control and
 * monitoring processes of the Talise device.
 *
 * @param TALISE_ERR_GPIO_OK Indicates no error occurred.
 * @param TALISE_ERR_MONITOR_OUT_INDEX_RANGE Error for monitor output index out
 * of range.
 * @param TALISE_ERR_GETGPIOMON_INDEX_NULL_PARM Error for null parameter in GPIO
 * monitor index.
 * @param TALISE_ERR_GETGPIOMON_MONITORMASK_NULL_PARM Error for null parameter
 * in GPIO monitor mask.
 * @param TALISE_ERR_GETGPIO_OE_NULL_PARM Error for null parameter in GPIO
 * output enable.
 * @param TALISE_ERR_GPIO_OE_INV_PARM Error for invalid parameter in GPIO output
 * enable.
 * @param TALISE_ERR_GPIO_SRC_INV_PARM Error for invalid parameter in GPIO
 * source.
 * @param TALISE_ERR_GETGPIO_SRC_NULL_PARM Error for null parameter in GPIO
 * source.
 * @param TALISE_ERR_GPIO_LEVEL_INV_PARM Error for invalid parameter in GPIO
 * level.
 * @param TALISE_ERR_GETGPIO_LEVEL_NULL_PARM Error for null parameter in GPIO
 * level.
 * @param TALISE_ERR_GETGPIO_SETLEVEL_NULL_PARM Error for null parameter in GPIO
 * set level.
 * @param TALISE_ERR_SETUPAUXDAC_NULL_PARM Error for null parameter in setup
 * auxiliary DAC.
 * @param TALISE_ERR_SETUPAUXDAC_INV_10BIT_AUXDACCODE Error for invalid 10-bit
 * auxiliary DAC code.
 * @param TALISE_ERR_SETUPAUXDAC_INV_12BIT_AUXDACCODE Error for invalid 12-bit
 * auxiliary DAC code.
 * @param TALISE_ERR_WRITEAUXDAC_INV_10BIT_AUXDACCODE Error for invalid 10-bit
 * auxiliary DAC code during
 * write.
 * @param TALISE_ERR_WRITEAUXDAC_INV_12BIT_AUXDACCODE Error for invalid 12-bit
 * auxiliary DAC code during
 * write.
 * @param TALISE_ERR_WRITEAUXDAC_INV_AUXDACINDEX Error for invalid auxiliary DAC
 * index during write.
 * @param TALISE_ERR_SETUPAUXDAC_INV_RESOLUTION Error for invalid resolution in
 * setup auxiliary DAC.
 * @param TALISE_ERR_GPIO3V3_OE_INV_PARM Error for invalid parameter in GPIO3V3
 * output enable.
 * @param TALISE_ERR_GETGPIO3V3_OE_NULL_PARM Error for null parameter in GPIO3V3
 * output enable.
 * @param TALISE_ERR_GPIO3V3_SRC_INV_PARM Error for invalid parameter in GPIO3V3
 * source.
 * @param TALISE_ERR_GETGPIO3V3_SRC_NULL_PARM Error for null parameter in
 * GPIO3V3 source.
 * @param TALISE_ERR_GPIO3V3_LEVEL_INV_PARM Error for invalid parameter in
 * GPIO3V3 level.
 * @param TALISE_ERR_GETGPIO3V3_LEVEL_NULL_PARM Error for null parameter in
 * GPIO3V3 level.
 * @param TALISE_ERR_GETGPIO3V3_SETLEVEL_NULL_PARM Error for null parameter in
 * GPIO3V3 set level.
 * @param TALISE_ERR_GPINT_OK Indicates no error occurred in general purpose
 * interrupt.
 * @param TALISE_ERR_GPINT_STATUS_NULL_PARM Error for null parameter in general
 * purpose interrupt status.
 * @param TALISE_ERR_GPINT_GPINTDIAG_NULL_PARM Error for null parameter in
 * general purpose interrupt
 * diagnostic.
 * @param TALISE_ERR_GPINT_NO_SOURCE_FOUND Error for no source found in general
 * purpose interrupt.
 * @param TALISE_ERR_GPINT_SOURCE_NOT_IMPLEMENTED Error for source not
 * implemented in general purpose
 * interrupt.
 * @param TALISE_ERR_GPINT_CLKPLL_UNLOCKED Error for clock PLL unlocked in
 * general purpose interrupt.
 * @param TALISE_ERR_GPINT_RFPLL_UNLOCKED Error for RF PLL unlocked in general
 * purpose interrupt.
 * @param TALISE_ERR_GPINT_AUXPLL_UNLOCKED Error for auxiliary PLL unlocked in
 * general purpose interrupt.
 * @param TALISE_ERR_GPINT_ARM_WATCHDOG_TIMEOUT Error for ARM watchdog timeout
 * in general purpose interrupt.
 * @param TALISE_ERR_GPINT_ARM_FORCE_GPINT Error for ARM force general purpose
 * interrupt.
 * @param TALISE_ERR_GPINT_ARM_SYSTEM_ERROR Error for ARM system error in
 * general purpose interrupt.
 * @param TALISE_ERR_GPINT_ARM_DATA_PARITY_ERROR Error for ARM data parity error
 * in general purpose interrupt.
 * @param TALISE_ERR_GPINT_ARM_PROG_PARITY_ERROR Error for ARM program parity
 * error in general purpose
 * interrupt.
 * @param TALISE_ERR_GPINT_ARM_CALIBRATION_ERROR Error for ARM calibration error
 * in general purpose interrupt.
 * @param TALISE_ERR_GPINT_FRAMERA Error for framer A in general purpose
 * interrupt.
 * @param TALISE_ERR_GPINT_DEFRAMERA Error for deframer A in general purpose
 * interrupt.
 * @param TALISE_ERR_GPINT_FRAMERB Error for framer B in general purpose
 * interrupt.
 * @param TALISE_ERR_GPINT_DEFRAMERB Error for deframer B in general purpose
 * interrupt.
 * @param TALISE_ERR_GPINT_PA_PROTECT_CH1 Error for PA protection channel 1 in
 * general purpose interrupt.
 * @param TALISE_ERR_GPINT_PA_PROTECT_CH2 Error for PA protection channel 2 in
 * general purpose interrupt.
 * @param TALISE_ERR_GPINT_STREAM_ERROR Error for stream error in general
 * purpose interrupt.
 * @param TALISE_ERR_SETAUXADCPINMODEGPIO_INV_GPIO Error for invalid GPIO in set
 * auxiliary ADC pin mode.
 * @param TALISE_ERR_SETAUXADCPINMODEGPIO_GPIO_IN_USE Error for GPIO in use in
 * set auxiliary ADC pin
 * mode.
 * @param TALISE_ERR_STARTAUXADC_INV_CHANNEL Error for invalid channel in start
 * auxiliary ADC.
 * @param TALISE_ERR_STARTAUXADC_INV_MODE Error for invalid mode in start
 * auxiliary ADC.
 * @param TALISE_ERR_STARTAUXADC_INV_NUM_SAMPLES Error for invalid number of
 * samples in start auxiliary ADC.
 * @param TALISE_ERR_STARTAUXADC_INV_SAMPLING_PERIOD Error for invalid sampling
 * period in start auxiliary
 * ADC.
 * @param TALISE_ERR_STARTAUXADC_NULL_PARAM Error for null parameter in start
 * auxiliary ADC.
 * @param TALISE_ERR_READAUXADC_NULL_PARAM Error for null parameter in read
 * auxiliary ADC.
 * @param TAL_ERR_GPIO_NUMBER_OF_ERRORS Reference to the total number of error
 * enum values.
 ******************************************************************************/
typedef enum {
	TALISE_ERR_GPIO_OK = 0,
	TALISE_ERR_MONITOR_OUT_INDEX_RANGE,
	TALISE_ERR_GETGPIOMON_INDEX_NULL_PARM,
	TALISE_ERR_GETGPIOMON_MONITORMASK_NULL_PARM,
	TALISE_ERR_GETGPIO_OE_NULL_PARM,
	TALISE_ERR_GPIO_OE_INV_PARM,
	TALISE_ERR_GPIO_SRC_INV_PARM,
	TALISE_ERR_GETGPIO_SRC_NULL_PARM,
	TALISE_ERR_GPIO_LEVEL_INV_PARM,
	TALISE_ERR_GETGPIO_LEVEL_NULL_PARM,
	TALISE_ERR_GETGPIO_SETLEVEL_NULL_PARM,
	TALISE_ERR_SETUPAUXDAC_NULL_PARM,
	TALISE_ERR_SETUPAUXDAC_INV_10BIT_AUXDACCODE,
	TALISE_ERR_SETUPAUXDAC_INV_12BIT_AUXDACCODE,
	TALISE_ERR_WRITEAUXDAC_INV_10BIT_AUXDACCODE,
	TALISE_ERR_WRITEAUXDAC_INV_12BIT_AUXDACCODE,
	TALISE_ERR_WRITEAUXDAC_INV_AUXDACINDEX,
	TALISE_ERR_SETUPAUXDAC_INV_RESOLUTION,
	TALISE_ERR_GPIO3V3_OE_INV_PARM,
	TALISE_ERR_GETGPIO3V3_OE_NULL_PARM,
	TALISE_ERR_GPIO3V3_SRC_INV_PARM,
	TALISE_ERR_GETGPIO3V3_SRC_NULL_PARM,
	TALISE_ERR_GPIO3V3_LEVEL_INV_PARM,
	TALISE_ERR_GETGPIO3V3_LEVEL_NULL_PARM,
	TALISE_ERR_GETGPIO3V3_SETLEVEL_NULL_PARM,
	TALISE_ERR_GPINT_OK,
	TALISE_ERR_GPINT_STATUS_NULL_PARM,
	TALISE_ERR_GPINT_GPINTDIAG_NULL_PARM,
	TALISE_ERR_GPINT_NO_SOURCE_FOUND,
	TALISE_ERR_GPINT_SOURCE_NOT_IMPLEMENTED,
	TALISE_ERR_GPINT_CLKPLL_UNLOCKED,
	TALISE_ERR_GPINT_RFPLL_UNLOCKED,
	TALISE_ERR_GPINT_AUXPLL_UNLOCKED,
	TALISE_ERR_GPINT_ARM_WATCHDOG_TIMEOUT,
	TALISE_ERR_GPINT_ARM_FORCE_GPINT,
	TALISE_ERR_GPINT_ARM_SYSTEM_ERROR,
	TALISE_ERR_GPINT_ARM_DATA_PARITY_ERROR,
	TALISE_ERR_GPINT_ARM_PROG_PARITY_ERROR,
	TALISE_ERR_GPINT_ARM_CALIBRATION_ERROR,
	TALISE_ERR_GPINT_FRAMERA,
	TALISE_ERR_GPINT_DEFRAMERA,
	TALISE_ERR_GPINT_FRAMERB,
	TALISE_ERR_GPINT_DEFRAMERB,
	TALISE_ERR_GPINT_PA_PROTECT_CH1,
	TALISE_ERR_GPINT_PA_PROTECT_CH2,
	TALISE_ERR_GPINT_STREAM_ERROR,
	TALISE_ERR_SETAUXADCPINMODEGPIO_INV_GPIO,
	TALISE_ERR_SETAUXADCPINMODEGPIO_GPIO_IN_USE,
	TALISE_ERR_STARTAUXADC_INV_CHANNEL,
	TALISE_ERR_STARTAUXADC_INV_MODE,
	TALISE_ERR_STARTAUXADC_INV_NUM_SAMPLES,
	TALISE_ERR_STARTAUXADC_INV_SAMPLING_PERIOD,
	TALISE_ERR_STARTAUXADC_NULL_PARAM,
	TALISE_ERR_READAUXADC_NULL_PARAM,
	TAL_ERR_GPIO_NUMBER_OF_ERRORS /* Keep this ENUM last as a reference to the total number of error enum values */
} taliseGpioErr_t;

/***************************************************************************//**
 * @brief The `taliseAuxDacVref_t` is an enumeration that defines the possible
 * reference voltage levels for the auxiliary DAC (Digital-to-Analog
 * Converter) on the Talise device. Each enumerator represents a specific
 * voltage level that can be used as a reference for the DAC, allowing
 * for precise control over the output voltage range of the DAC. This
 * enumeration is crucial for configuring the DAC to operate at the
 * desired voltage reference, which is essential for applications
 * requiring specific voltage outputs.
 *
 * @param TAL_AUXDACVREF_1V AuxDAC reference at 1V.
 * @param TAL_AUXDACVREF_1P5V AuxDAC reference at 1.5V.
 * @param TAL_AUXDACVREF_2V AuxDAC reference at 2V.
 * @param TAL_AUXDACVREF_2P5V AuxDAC reference at 2.5V.
 ******************************************************************************/
typedef enum {
	TAL_AUXDACVREF_1V = 0, /*!< AuxDAC reference at 1V */
	TAL_AUXDACVREF_1P5V = 1, /*!< AuxDAC reference at 1.5V */
	TAL_AUXDACVREF_2V = 2, /*!< AuxDAC reference at 2V */
	TAL_AUXDACVREF_2P5V = 3 /*!< AuxDAC reference at 2.5V */
} taliseAuxDacVref_t;

/***************************************************************************//**
 * @brief The `taliseAuxDacResolution_t` is an enumeration that defines the
 * resolution modes for the auxiliary DAC (Digital-to-Analog Converter)
 * on the Talise device. It specifies three different resolution
 * settings: 12-bit, 11-bit, and 10-bit, each corresponding to different
 * ranges and precision levels of the DAC output voltage. This
 * enumeration is used to configure the DAC resolution based on the
 * desired output characteristics, allowing for flexibility in the
 * precision and range of the DAC's output.
 *
 * @param TAL_AUXDACRES_12BIT 12bit DAC resolution for a subset of the output
 * voltage range centered around VREF.
 * @param TAL_AUXDACRES_11BIT 11bit DAC resolution for a subset of the output
 * voltage range centered around VREF.
 * @param TAL_AUXDACRES_10BIT 10bit DAC resolution for 100mv to 3v range.
 ******************************************************************************/
typedef enum {
	TAL_AUXDACRES_12BIT = 0, /*!< 12bit DAC resolution for a subset of the output voltage range centered around VREF */
	TAL_AUXDACRES_11BIT = 1, /*!< 11bit DAC resolution for a subset of the output voltage range centered around VREF */
	TAL_AUXDACRES_10BIT = 2  /*!< 10bit DAC resolution for 100mv to 3v range */
} taliseAuxDacResolution_t;

/***************************************************************************//**
 * @brief The `taliseAuxDac_t` structure is designed to manage the settings for
 * auxiliary DACs in the Talise device. It includes fields to enable or
 * disable each DAC, set the voltage reference for each 10-bit DAC,
 * define the resolution for voltage changes, and store the DAC values
 * for both 10-bit and 12-bit DACs. This structure allows for detailed
 * configuration and control of the auxiliary DACs, facilitating precise
 * voltage output adjustments.
 *
 * @param auxDacEnables Aux DAC enable bit for each DAC, where the first ten
 * bits correspond to the 10-bit DACs, and the next
 * consecutive two bits enable the 12-bit DACs.
 * @param auxDacVref Aux DAC voltage reference value for each of the 10-bit
 * DACs.
 * @param auxDacResolution Aux DAC slope (resolution of voltage change per
 * AuxDAC code) - only applies to 10bit DACs (0-9).
 * @param auxDacValues Aux DAC values for each 10-bit DAC correspond to the
 * first 10 array elements, the next consecutive array
 * elements correspond to the two 12-bit DAC values.
 ******************************************************************************/
typedef struct {
	uint16_t
	auxDacEnables;         /*!< Aux DAC enable bit for each DAC, where the first ten bits correspond to the 10-bit DACs, and the next consecutive two bits enable the 12-bit DACs */
	taliseAuxDacVref_t
	auxDacVref[10];        /*!< Aux DAC voltage reference value for each of the 10-bit DACs */
	taliseAuxDacResolution_t
	auxDacResolution[10];  /*!< Aux DAC slope (resolution of voltage change per AuxDAC code) - only applies to 10bit DACs (0-9) */
	uint16_t
	auxDacValues[12];      /*!< Aux DAC values for each 10-bit DAC correspond to the first 10 array elements, the next consecutive array elements correspond to the two 12-bit DAC values */
} taliseAuxDac_t;

/***************************************************************************//**
 * @brief The `taliseGpIntMask_t` is an enumeration that defines various
 * general-purpose interrupt mask bits for the Talise device. Each
 * enumerator represents a specific type of interrupt condition, such as
 * errors related to stream processing, ARM calibration, system errors,
 * watchdog timeouts, PA protection errors, JESD204B framer and deframer
 * IRQs, and PLL unlock conditions. These mask bits are used to enable or
 * disable specific interrupt sources, allowing for fine-grained control
 * over the handling of interrupt events in the device.
 *
 * @param TAL_GP_MASK_STREAM_ERROR Stream processor error GP Interrupt mask bit.
 * @param TAL_GP_MASK_ARM_CALIBRATION_ERROR ARM calibration error GP Interrupt
 * mask bit.
 * @param TAL_GP_MASK_ARM_SYSTEM_ERROR ARM System error GP Interrupt mask bit.
 * @param TAL_GP_MASK_ARM_FORCE_INTERRUPT ARM force GP Interrupt mask bit.
 * @param TAL_GP_MASK_WATCHDOG_TIMEOUT Watchdog GP Interrupt mask bit.
 * @param TAL_GP_MASK_PA_PROTECTION_TX2_ERROR Tx2 PA protection error GP
 * Interrupt mask bit.
 * @param TAL_GP_MASK_PA_PROTECTION_TX1_ERROR Tx1 PA protection error GP
 * Interrupt mask bit.
 * @param TAL_GP_MASK_JESD_DEFRMER_IRQ JESD204B Deframer IRQ error GP Interrupt
 * mask bit.
 * @param TAL_GP_MASK_JESD_FRAMER_IRQ JESD204B Framer IRQ error GP Interrupt
 * mask bit.
 * @param TAL_GP_MASK_CLK_SYNTH_UNLOCK Device clock PLL non-lock error GP
 * Interrupt mask bit.
 * @param TAL_GP_MASK_AUX_SYNTH_UNLOCK Auxiliary PLL non-lock error GP Interrupt
 * mask bit.
 * @param TAL_GP_MASK_RF_SYNTH_UNLOCK RF PLL non-lock error GP Interrupt mask
 * bit.
 ******************************************************************************/
typedef enum {
	TAL_GP_MASK_STREAM_ERROR            = 0x1000,   /*!< Stream processor error GP Interrupt mask bit */
	TAL_GP_MASK_ARM_CALIBRATION_ERROR   = 0x0800,   /*!< ARM calibration error GP Interrupt mask bit */
	TAL_GP_MASK_ARM_SYSTEM_ERROR        = 0x0400,   /*!< ARM System error GP Interrupt mask bit */
	TAL_GP_MASK_ARM_FORCE_INTERRUPT     = 0x0200,   /*!< ARM force GP Interrupt mask bit */
	TAL_GP_MASK_WATCHDOG_TIMEOUT        = 0x0100,   /*!< Watchdog GP Interrupt mask bit */
	TAL_GP_MASK_PA_PROTECTION_TX2_ERROR = 0x0080,   /*!< Tx2 PA protection error GP Interrupt mask bit */
	TAL_GP_MASK_PA_PROTECTION_TX1_ERROR = 0x0040,   /*!< Tx1 PA protection error GP Interrupt mask bit */
	TAL_GP_MASK_JESD_DEFRMER_IRQ        = 0x0020,   /*!< JESD204B Deframer IRQ error GP Interrupt mask bit */
	TAL_GP_MASK_JESD_FRAMER_IRQ         = 0x0010,   /*!< JESD204B Framer IRQ error GP Interrupt mask bit */
	TAL_GP_MASK_CLK_SYNTH_UNLOCK        = 0x0008,   /*!< Device clock PLL non-lock error GP Interrupt mask bit */
	TAL_GP_MASK_AUX_SYNTH_UNLOCK        = 0x0004,   /*!< Auxiliary PLL non-lock error GP Interrupt mask bit */
	TAL_GP_MASK_RF_SYNTH_UNLOCK         = 0x0002    /*!< RF PLL non-lock error GP Interrupt mask bit */
} taliseGpIntMask_t;

#define TAL_GPMASK_MSB (uint16_t)(TAL_GP_MASK_STREAM_ERROR | \
                                  TAL_GP_MASK_ARM_CALIBRATION_ERROR | \
                                  TAL_GP_MASK_ARM_SYSTEM_ERROR | \
                                  TAL_GP_MASK_ARM_FORCE_INTERRUPT | \
                                  TAL_GP_MASK_WATCHDOG_TIMEOUT)

#define TAL_GPMASK_LSB (uint16_t)(TAL_GP_MASK_PA_PROTECTION_TX1_ERROR | \
                                  TAL_GP_MASK_PA_PROTECTION_TX2_ERROR | \
                                  TAL_GP_MASK_JESD_DEFRMER_IRQ | \
                                  TAL_GP_MASK_JESD_FRAMER_IRQ | \
                                  TAL_GP_MASK_CLK_SYNTH_UNLOCK | \
                                  TAL_GP_MASK_AUX_SYNTH_UNLOCK | \
                                  TAL_GP_MASK_RF_SYNTH_UNLOCK)
/***************************************************************************//**
 * @brief The `taliseSpi2TxAttenGpioSel_t` is an enumeration that defines the
 * possible GPIO selections for controlling the SPI2 transmit attenuation
 * on the Talise device. It provides options to select specific GPIO pins
 * (GPIO4, GPIO8, GPIO14) or to disable the GPIO control for SPI2 Tx
 * attenuation, allowing for flexible configuration of the device's
 * attenuation settings.
 *
 * @param TAL_SPI2_TXATTEN_GPIO4 Selects GPIO4 for SPI2 Tx Attenuation.
 * @param TAL_SPI2_TXATTEN_GPIO8 Selects GPIO8 for SPI2 Tx Attenuation.
 * @param TAL_SPI2_TXATTEN_GPIO14 Selects GPIO14 for SPI2 Tx Attenuation.
 * @param TAL_SPI2_TXATTEN_DISABLE Disables GPIO for SPI2 Tx Attenuation.
 ******************************************************************************/
typedef enum {
	TAL_SPI2_TXATTEN_GPIO4   = 0x00,    /*!< Select GPIO4 for SPI2 Tx Attenuation select */
	TAL_SPI2_TXATTEN_GPIO8   = 0x01,    /*!< Select GPIO8 for SPI2 Tx Attenuation select */
	TAL_SPI2_TXATTEN_GPIO14  = 0x02,    /*!< Select GPIO14 for SPI2 Tx Attenuation select */
	TAL_SPI2_TXATTEN_DISABLE = 0x03     /*!< Disable GPIO for SPI2 Tx Attenuation select */
} taliseSpi2TxAttenGpioSel_t;


/***************************************************************************//**
 * @brief The `taliseGpIntInformation_t` structure is designed to encapsulate
 * information related to general-purpose interrupts (GP_INT) in the
 * Talise device. It includes an array to store all GP_INT sources, and
 * fields to specify the interrupting framer and deframer, which are only
 * valid for their respective sources. Additionally, it contains a mask
 * for deframer inputs, which is used to identify specific deframer lanes
 * after crossbar swapping, providing a detailed view of the interrupt
 * sources and their configurations.
 *
 * @param data An array of 9 bytes representing all GP_INT sources.
 * @param framer An enumerated type indicating the interrupting framer, valid
 * only for framer sources.
 * @param deframer An enumerated type indicating the interrupting deframer,
 * valid only for deframer sources.
 * @param deframerInputsMask An integer representing the interrupting deframer
 * input mask, valid only for deframer sources, with a
 * valid range of 0x0-0xF.
 ******************************************************************************/
typedef struct {
	uint8_t data[9];                /*!< All GP_INT sources */
	taliseFramerSel_t
	framer;       /*!< Interrupting framer, valid only for framer sources */
	taliseDeframerSel_t
	deframer;   /*!< Interrupting deframer, valid only for deframer sources */
	int32_t deframerInputsMask;     /*!< Interrupting deframer input mask (bit per deframer input), valid only for deframer sources (valid 0x0-0xF)
                                         deframerInputsMask is the deframer lane after the deframer lane crossbar swapping (lane input of the deframer) */
} taliseGpIntInformation_t;

/***************************************************************************//**
 * @brief The `taliseAuxAdcChannels_t` is an enumeration that defines the
 * available auxiliary ADC channels on the Talise device. Each enumerator
 * corresponds to a specific channel (0 to 3) that can be selected for
 * sampling and conversion operations. This enumeration is used to
 * specify which channel should be used when configuring or initiating
 * ADC operations on the device.
 *
 * @param TAL_AUXADC_CH0 Select Aux ADC Channel 0 for sampling and conversion.
 * @param TAL_AUXADC_CH1 Select Aux ADC Channel 1 for sampling and conversion.
 * @param TAL_AUXADC_CH2 Select Aux ADC Channel 2 for sampling and conversion.
 * @param TAL_AUXADC_CH3 Select Aux ADC Channel 3 for sampling and conversion.
 ******************************************************************************/
typedef enum {
	TAL_AUXADC_CH0 = 0,  /*!< Select Aux ADC Channel 0 for sampling and conversion*/
	TAL_AUXADC_CH1 = 1,  /*!< Select Aux ADC Channel 1 for sampling and conversion*/
	TAL_AUXADC_CH2 = 2,  /*!< Select Aux ADC Channel 2 for sampling and conversion*/
	TAL_AUXADC_CH3 = 3   /*!< Select Aux ADC Channel 3 for sampling and conversion*/
} taliseAuxAdcChannels_t;

/***************************************************************************//**
 * @brief The `taliseAuxAdcModes_t` is an enumeration that defines the modes for
 * sampling and conversion of the auxiliary ADC on the Talise device. It
 * provides two modes: Non-Pin mode, where the ARM internal timer is used
 * for scheduling, and Pin mode, where GPIO input pin pulses are used to
 * schedule the sampling and conversion process. This allows for flexible
 * configuration of the ADC operation based on the desired timing and
 * control mechanism.
 *
 * @param TAL_AUXADC_NONPIN_MODE Selects Aux ADC sampling and conversion in Non-
 * Pin mode using ARM Internal timer.
 * @param TAL_AUXADC_PIN_MODE Selects Aux ADC sampling and conversion in Pin
 * mode using pulses on ARM GPIO Input pins.
 ******************************************************************************/
typedef enum {
	TAL_AUXADC_NONPIN_MODE = 0,  /*!< Select Aux ADC sampling and conversion in Non-Pin mode (ARM Internal timer is used for sampling and conversion)*/
	TAL_AUXADC_PIN_MODE    = 1   /*!< Select Aux ADC sampling and conversion in Pin mode (Pulses on ARM GPIO Input pins are used to schedule sampling and conversion)*/
} taliseAuxAdcModes_t;

/***************************************************************************//**
 * @brief The `taliseAuxAdcConfig_t` structure is used to configure the
 * auxiliary ADC (Analog-to-Digital Converter) settings on the Talise
 * device. It allows the user to select the ADC channel, set the mode for
 * storing conversion results, specify the number of samples for
 * conversion, and define the sampling period in microseconds. This
 * configuration is crucial for setting up the ADC to perform accurate
 * and timely conversions based on the application's requirements.
 *
 * @param auxAdcChannelSel Selects the channel which is supposed to sample
 * AuxADC input for A/D conversion.
 * @param auxAdcMode Selects mode to latch and store conversion results.
 * @param numSamples No. of A/D conversions to be performed in range 1 - 1000.
 * @param samplingPeriod_us Sampling interval time in microseconds (Minimum
 * 15us) NOTE: Valid only for non pin mode. Ignored for
 * pin mode.
 ******************************************************************************/
typedef struct {
	taliseAuxAdcChannels_t
	auxAdcChannelSel;  /*!< Selects the channel which is supposed to sample AuxADC input for A/D conversion */
	taliseAuxAdcModes_t
	auxAdcMode;        /*!< Selects mode to latch and store conversion results */
	uint16_t
	numSamples;        /*!< No. of A/D conversions to be performed in range 1 - 1000 */
	uint16_t
	samplingPeriod_us; /*!< Sampling interval time in microseconds (Minimum 15us) NOTE: Valid only for non pin mode. Ignored for pin mode. */
} taliseAuxAdcConfig_t;

/***************************************************************************//**
 * @brief The `taliseAuxAdcResult_t` structure is designed to store the results
 * of an auxiliary ADC conversion process. It includes the average ADC
 * code from the conversion samples, the number of samples that were
 * averaged to obtain this result, and a completion indicator flag to
 * denote whether the conversion process has been completed. This
 * structure is useful for managing and interpreting the results of ADC
 * operations in the Talise device.
 *
 * @param auxAdcCodeAvg 12-bit Average of AuxADC A/D conversion samples.
 * @param numSamples Number of samples averaged in auxAdcCodeAvg.
 * @param completeIndicator Flag indicating if a scheduled AuxADC conversion
 * completed; 1 for complete, 0 for incomplete.
 ******************************************************************************/
typedef struct {
	uint16_t auxAdcCodeAvg;     /*!< 12-bit Average of AuxADC A/D conversion samples */
	uint16_t numSamples;        /*!< No. of samples averaged in AuxAdcCodeAvg */
	uint8_t  completeIndicator; /*!< Flag to indicate if a scheduled AuxADC conversion completed. 1 - AuxADC Conversion Complete, 0 - AuxADC Conversion Incomplete */
} taliseAuxAdcResult_t;


#ifdef __cplusplus
}
#endif

#endif /* TALISE_GPIO_TYPES_H_ */
