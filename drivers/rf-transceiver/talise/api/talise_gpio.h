/**
 * \file talise_gpio.h
 * \brief Talise GPIO header file
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_GPIO_H_
#define TALISE_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_types.h"
#include "talise_gpio_types.h"

/***************************************************************************//**
 * @brief This function configures the direction of the low voltage GPIO pins on
 * a Talise device, allowing each pin to be set as either input or
 * output. It uses a mask to specify which pins should be affected by the
 * direction setting. This function should be called when you need to
 * change the GPIO pin directions, and it is important to ensure that the
 * `device` is properly initialized before calling this function. The
 * function handles invalid `gpioOutEn` values by returning an error
 * code.
 *
 * @param device Pointer to the Talise data structure. Must not be null and
 * should be properly initialized before use.
 * @param gpioOutEn A 32-bit value where each bit represents the direction of a
 * GPIO pin (0 for input, 1 for output). Valid range is 0 to
 * 0x07FFFF. Values outside this range will result in an error.
 * @param gpioUsedMask A 32-bit mask that specifies which GPIO pins' directions
 * should be modified. If a bit is set to 1, the
 * corresponding pin's direction will be updated based on
 * `gpioOutEn`.
 * @return Returns a 32-bit value indicating the result of the operation.
 * Possible return values include success or various error codes
 * indicating specific issues encountered during execution.
 ******************************************************************************/
uint32_t TALISE_setGpioOe(taliseDevice_t *device, uint32_t gpioOutEn,
			  uint32_t gpioUsedMask);

/***************************************************************************//**
 * @brief Use this function to obtain the current direction configuration of the
 * GPIO pins on the Talise device. It provides a bitmask indicating which
 * pins are set as inputs and which are set as outputs. This function is
 * useful for verifying the current GPIO setup or for debugging purposes.
 * Ensure that the `gpioOutEn` pointer is not null before calling this
 * function, as a null pointer will result in an error.
 *
 * @param device Pointer to the Talise device structure. Must not be null and
 * should be properly initialized before calling this function.
 * @param gpioOutEn Pointer to a uint32_t variable where the function will store
 * the GPIO direction bitmask. Each bit represents a GPIO pin
 * direction: 0 for input, 1 for output. Must not be null.
 * @return Returns a uint32_t value indicating the status of the operation.
 * Possible return values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for a null pointer error, and other values
 * indicating specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_getGpioOe(taliseDevice_t *device, uint32_t *gpioOutEn);

/***************************************************************************//**
 * @brief This function configures the GPIO output source for the Talise device,
 * affecting only the GPIO pins set to output. It allows each group of
 * four GPIO pins to share the same output source, which is specified by
 * the `gpioSrcCtrl` parameter. This function should be used when you
 * need to change the output source configuration of the GPIO pins.
 * Ensure that the `gpioSrcCtrl` value is within the valid range before
 * calling this function to avoid parameter errors.
 *
 * @param device Pointer to the Talise data structure. Must not be null, and the
 * device should be properly initialized before use.
 * @param gpioSrcCtrl A 32-bit value containing 5 nibbles that set the output
 * source control for each set of four GPIO pins. Valid range
 * is 0 to 0x000FFFFF. Values outside this range will result
 * in an error.
 * @return Returns a status code indicating the result of the operation, which
 * can be a success or an error code requiring specific recovery
 * actions.
 ******************************************************************************/
uint32_t TALISE_setGpioSourceCtrl(taliseDevice_t *device, uint32_t gpioSrcCtrl);

/***************************************************************************//**
 * @brief Use this function to retrieve the current GPIO output source
 * configuration for the Talise device. It provides the source control
 * settings for each set of four GPIO pins, which are organized in
 * nibbles. This function is useful for verifying the current GPIO
 * configuration, especially after changes have been made. Ensure that
 * the `device` is properly initialized before calling this function. The
 * function will return an error if the `gpioSrcCtrl` pointer is null.
 *
 * @param device Pointer to the Talise device structure. Must not be null and
 * should be properly initialized before use.
 * @param gpioSrcCtrl Pointer to a uint32_t variable where the GPIO source
 * control configuration will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the success or type of error
 * encountered. The GPIO source control configuration is written to the
 * location pointed to by gpioSrcCtrl.
 ******************************************************************************/
uint32_t TALISE_getGpioSourceCtrl(taliseDevice_t *device,
				  uint32_t *gpioSrcCtrl);

/***************************************************************************//**
 * @brief Use this function to set the output level of the Talise GPIO pins that
 * are configured as outputs. The function requires a valid device
 * structure and a gpioPinLevel value that specifies the desired output
 * level for each GPIO pin. The gpioPinLevel parameter must be within the
 * range of 0 to 0x07FFFF, where each bit represents the level of a
 * corresponding GPIO pin (0 for low, 1 for high). If the gpioPinLevel
 * exceeds this range, the function will return an error. This function
 * should be called only after the GPIO pins have been properly
 * configured for output.
 *
 * @param device Pointer to the Talise device structure. Must not be null and
 * should be properly initialized before calling this function.
 * @param gpioPinLevel A 32-bit unsigned integer representing the desired output
 * level for each GPIO pin. Valid range is 0 to 0x07FFFF.
 * Each bit corresponds to a GPIO pin, with 0 indicating a
 * low level and 1 indicating a high level. Values outside
 * this range will result in an error.
 * @return Returns a 32-bit unsigned integer indicating the result of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success, TALACT_ERR_CHECK_PARAM for invalid parameters, and
 * TALACT_ERR_RESET_SPI for SPI-related errors.
 ******************************************************************************/
uint32_t TALISE_setGpioPinLevel(taliseDevice_t *device, uint32_t gpioPinLevel);

/***************************************************************************//**
 * @brief Use this function to read the current levels of the Talise GPIO pins
 * configured as inputs. It returns the levels in a single 32-bit word,
 * where each bit represents the state of a corresponding GPIO pin. This
 * function should be called when you need to determine the input state
 * of the GPIO pins. Ensure that the `gpioPinLevel` pointer is not null
 * before calling this function, as a null pointer will result in an
 * error. The function returns a status code indicating the success or
 * type of error encountered during execution.
 *
 * @param device Pointer to the Talise device structure. Must not be null. The
 * caller retains ownership.
 * @param gpioPinLevel Pointer to a uint32_t variable where the GPIO pin levels
 * will be stored. Must not be null. The function writes the
 * pin levels to this location.
 * @return Returns a uint32_t status code indicating the result of the
 * operation. Possible values include success and various error codes.
 ******************************************************************************/
uint32_t TALISE_getGpioPinLevel(taliseDevice_t *device, uint32_t *gpioPinLevel);

/***************************************************************************//**
 * @brief Use this function to retrieve the current output levels set on the
 * Talise GPIO pins when operating in BITBANG mode. This function is
 * useful for verifying the output state of GPIO pins configured as
 * outputs. It must be called with a valid device structure and a non-
 * null pointer to store the output levels. The function will return a
 * recovery action code indicating the success or type of error
 * encountered during the operation.
 *
 * @param device Pointer to the Talise data structure. Must not be null and
 * should be properly initialized before calling this function.
 * @param gpioPinSetLevel Pointer to a single uint32_t variable where the
 * function will store the output levels of the GPIO
 * pins. Each bit represents the level of a corresponding
 * GPIO pin. Must not be null.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_NO_ACTION for success,
 * TALACT_WARN_RESET_LOG for log reset warnings, TALACT_ERR_CHECK_PARAM
 * for parameter errors, and TALACT_ERR_RESET_SPI for SPI reset
 * requirements.
 ******************************************************************************/
uint32_t TALISE_getGpioSetLevel(taliseDevice_t *device,
				uint32_t *gpioPinSetLevel);

/***************************************************************************//**
 * @brief This function configures the monitor output function for the GPIOs on
 * a Talise device, allowing visibility to internal signals. It should be
 * used when you need to route specific internal signals to the GPIO pins
 * for monitoring purposes. Before calling this function, ensure that the
 * GPIO pins are set to the correct direction and source control to allow
 * the monitor signals to be routed. The function requires a valid
 * monitor index and mask to specify which signals to output. It returns
 * a status code indicating the success or type of error encountered.
 *
 * @param device Pointer to the Talise data structure. Must not be null, and the
 * device should be properly initialized before calling this
 * function.
 * @param monitorIndex The index specifying which set of 8 monitor outputs to
 * use. Valid values are within the range defined by the
 * device, and values outside this range will result in an
 * error.
 * @param monitorMask A bitmask indicating which of the 8 monitor outputs are
 * active. Each bit corresponds to a monitor output, with bit
 * 0 representing monitor out 0 and bit 7 representing
 * monitor out 7.
 * @return Returns a uint32_t status code indicating the result of the
 * operation. Possible values include success or specific error codes
 * related to parameter validation or hardware communication issues.
 ******************************************************************************/
uint32_t TALISE_setGpioMonitorOut(taliseDevice_t *device, uint8_t monitorIndex,
				  uint8_t monitorMask);

/***************************************************************************//**
 * @brief Use this function to retrieve the current GPIO monitor index and mask
 * settings from a Talise device. It is essential to ensure that the
 * pointers provided for the monitor index and monitor mask are valid and
 * not null before calling this function. This function is typically used
 * to verify or log the current GPIO monitor configuration. It must be
 * called after the device has been properly initialized and configured.
 *
 * @param device Pointer to the Talise data structure. Must not be null. The
 * caller retains ownership.
 * @param monitorIndex Pointer to a single uint8_t variable where the current
 * monitor signal selection index will be stored. Must not
 * be null. If null, the function returns an error.
 * @param monitorMask Pointer to a single uint8_t variable where the monitor out
 * signal masking will be stored. Must not be null. If null,
 * the function returns an error.
 * @return Returns a uint32_t value indicating the status of the operation.
 * Possible return values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for null parameter errors, and other error
 * codes for specific issues encountered during execution.
 ******************************************************************************/
uint32_t TALISE_getGpioMonitorOut(taliseDevice_t *device, uint8_t *monitorIndex,
				  uint8_t *monitorMask);

/***************************************************************************//**
 * @brief Use this function to configure which signals can assert the General
 * Purpose (GP) interrupt pin by setting a bit mask. Each bit in the mask
 * corresponds to a specific interrupt source, and setting a bit to 1
 * disables the corresponding interrupt from asserting the GP interrupt
 * pin. This function should be called after device initialization and
 * can be used to selectively enable or disable specific interrupt
 * sources. Care should be taken to ensure that the mask is valid, as
 * invalid masks will result in an error.
 *
 * @param device Pointer to the Talise data structure. Must not be null, and the
 * device should be properly initialized before calling this
 * function.
 * @param gpIntMask A 16-bit bit-mask where each bit represents a specific
 * interrupt source. Valid bits are defined by the
 * taliseGpIntMask_t enumeration. Invalid bits will result in
 * an error.
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for invalid parameters, and other error codes
 * indicating specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_setGpIntMask(taliseDevice_t *device, uint16_t gpIntMask);

/***************************************************************************//**
 * @brief Use this function to obtain the current mask settings for the General
 * Purpose (GP) interrupts on the Talise device. This function can be
 * called at any time after the device has been initialized. It provides
 * a bit-mask indicating which interrupt sources are currently masked
 * (disabled) from asserting the GP interrupt pin. Ensure that the
 * `gpIntMask` pointer is not null before calling this function to avoid
 * parameter errors.
 *
 * @param device Pointer to the Talise device structure. Must not be null. The
 * caller retains ownership.
 * @param gpIntMask Pointer to a uint16_t variable where the function will store
 * the current GP interrupt mask. Must not be null. If null,
 * the function returns an error.
 * @return Returns a uint32_t value indicating the success or type of error
 * encountered. Possible return values include TALACT_NO_ACTION for
 * success, TALACT_ERR_CHECK_PARAM for invalid parameters, and other
 * error codes for specific issues.
 ******************************************************************************/
uint32_t TALISE_getGpIntMask(taliseDevice_t *device, uint16_t *gpIntMask);

/***************************************************************************//**
 * @brief This function retrieves the current status of the General Purpose (GP)
 * interrupt from the Talise device, allowing the user to determine the
 * source of an interrupt. It should be called after the device has been
 * initialized and the interrupt mask bits have been set. The function
 * requires a valid pointer to a `taliseDevice_t` structure and a non-
 * null pointer to a `uint16_t` variable where the status will be stored.
 * If the `gpIntStatus` pointer is null, the function will handle the
 * error and return an appropriate recovery action. The status word
 * returned will indicate the current state of all interrupt sources,
 * even if they are masked, but the GP Interrupt pin will only assert for
 * enabled sources.
 *
 * @param device Pointer to the Talise data structure. Must not be null. The
 * caller retains ownership.
 * @param gpIntStatus Pointer to a uint16_t variable where the interrupt status
 * will be stored. Must not be null. If null, the function
 * will handle the error and return a recovery action.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_NO_ACTION for successful completion,
 * TALACT_ERR_CHECK_PARAM for parameter errors, and other recovery
 * actions for specific error conditions.
 ******************************************************************************/
uint32_t TALISE_getGpIntStatus(taliseDevice_t *device, uint16_t *gpIntStatus);

/***************************************************************************//**
 * @brief This function retrieves the current temperature from the device's
 * temperature sensor, converting the raw sensor data into degrees
 * Celsius. It should be called after the device has been fully
 * initialized and the ARM processor is configured. The function requires
 * a valid pointer to store the temperature value. If the pointer is
 * null, the function will handle the error and return an appropriate
 * recovery action. This function is useful for monitoring the device's
 * operating temperature during runtime.
 *
 * @param device Pointer to the Talise device structure. Must not be null and
 * should point to a properly initialized device.
 * @param temperatureDegC Pointer to an int16_t variable where the temperature
 * in degrees Celsius will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the recovery action required.
 * TALACT_NO_ACTION indicates success, while other values indicate
 * specific error handling actions.
 ******************************************************************************/
uint32_t TALISE_getTemperature(taliseDevice_t *device,
			       int16_t *temperatureDegC);


/***************************************************************************//**
 * @brief This function sets up the auxiliary DACs on the Talise device using
 * the configuration provided in the `auxDac` data structure. It can be
 * called any time after the device has been initialized to configure,
 * enable, or disable the DAC outputs. The function operates the DACs in
 * manual control mode, and it is possible to change a specific DAC code
 * later using a separate function. The auxiliary DACs share GPIO pins,
 * so ensure the corresponding GPIO pins are set to input mode to avoid
 * conflicts. The function handles invalid input values by clamping them
 * to the maximum allowable values and returns an error code for invalid
 * configurations.
 *
 * @param device Pointer to the Talise device structure. Must not be null. The
 * caller retains ownership.
 * @param auxDac Pointer to the Talise auxiliary DAC data structure. Must not be
 * null. Contains configuration for each DAC, including enable
 * flags, resolution, and values. Invalid values are clamped to
 * the maximum allowable values for the respective DAC resolution.
 * @return Returns a uint32_t indicating the result of the operation. Possible
 * return values include success, parameter check errors, and SPI reset
 * requirements.
 ******************************************************************************/
uint32_t TALISE_setupAuxDacs(taliseDevice_t *device, taliseAuxDac_t *auxDac);

/***************************************************************************//**
 * @brief Use this function to set the output voltage of a specific auxiliary
 * DAC on the Talise device by writing a code to it. This function should
 * be called after the device has been initialized and the auxiliary DACs
 * have been set up. The function supports writing to both 10-bit and
 * 12-bit DACs, with indices 0-9 corresponding to 10-bit DACs and indices
 * 10-11 corresponding to 12-bit DACs. Ensure that the `auxDacIndex` and
 * `auxDacCode` parameters are within their valid ranges to avoid errors.
 *
 * @param device Pointer to the Talise device structure. Must not be null, and
 * the device should be properly initialized before calling this
 * function.
 * @param auxDacIndex Selects the desired DAC to load the code. Valid values are
 * 0-11, where 0-9 correspond to 10-bit DACs and 10-11
 * correspond to 12-bit DACs. If the index is out of range,
 * an error is returned.
 * @param auxDacCode The DAC code to write to the selected DAC. For indices 0-9,
 * valid range is 0-1023; for indices 10-11, valid range is
 * 0-4095. If the code exceeds the valid range for the
 * selected DAC, an error is returned.
 * @return Returns a uint32_t indicating the result of the operation. Possible
 * return values include success or specific error codes related to
 * parameter validation or SPI communication issues.
 ******************************************************************************/
uint32_t TALISE_writeAuxDac(taliseDevice_t *device, uint8_t auxDacIndex,
			    uint16_t auxDacCode);

/***************************************************************************//**
 * @brief This function configures the second SPI port (SPI2) on the Talise
 * device, allowing it to be enabled or disabled. It also sets the GPIO
 * pin used to toggle between two Tx attenuation settings when SPI2 is
 * enabled. This function should be used when you need to control Tx
 * attenuation via a secondary SPI interface. Ensure that the specified
 * GPIO pins are not already in use by other features, as the function
 * will return an error if the GPIOs are unavailable. The function must
 * be called after the device has been initialized.
 *
 * @param device Pointer to the Talise data structure. Must not be null.
 * @param spi2Enable A uint8_t value where 1 enables and 0 disables the SPI2
 * protocol on the Talise device.
 * @param spi2TxAttenGpioSel An enumeration value of type
 * taliseSpi2TxAttenGpioSel_t that specifies the GPIO
 * pin used to select between two Tx attenuation
 * values. Must be a valid enumeration value;
 * otherwise, the function returns an error.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation, with specific error codes for invalid parameters or GPIO
 * conflicts.
 ******************************************************************************/
uint32_t TALISE_setSpi2Enable(taliseDevice_t *device, uint8_t spi2Enable,
			      taliseSpi2TxAttenGpioSel_t spi2TxAttenGpioSel);

/***************************************************************************//**
 * @brief This function retrieves the current status of the second SPI port
 * (SPI2) on the Talise device, indicating whether it is enabled or
 * disabled. It also provides the GPIO pin selected for the TxAttenuation
 * switch, which is used when SPI2 is enabled. This function should be
 * called when you need to verify the current configuration of SPI2,
 * especially before performing operations that depend on its state.
 * Ensure that the device is properly initialized before calling this
 * function.
 *
 * @param device Pointer to the Talise data structure. Must not be null. The
 * caller retains ownership.
 * @param spi2Enable Pointer to a uint8_t variable where the function will store
 * the status of SPI2 (1 if enabled, 0 if disabled). Must not
 * be null.
 * @param spi2TxAttenGpioSel Pointer to a taliseSpi2TxAttenGpioSel_t variable
 * where the function will store the GPIO pin used for
 * TxAttenuation switch. Must not be null.
 * @return Returns a uint32_t indicating the recovery action status. Possible
 * values include TALACT_WARN_RESET_LOG, TALACT_ERR_RESET_SPI, and
 * TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_getSpi2Enable(taliseDevice_t *device, uint8_t *spi2Enable,
			      taliseSpi2TxAttenGpioSel_t *spi2TxAttenGpioSel);

/***************************************************************************//**
 * @brief This function configures the direction of the 3.3V GPIO pins on the
 * Talise device, allowing each pin to be set as either input or output.
 * The function uses a mask to specify which pins should be affected,
 * ensuring that only the desired pins are modified. It is important to
 * ensure that the `gpio3v3OutEn` parameter is within the valid range
 * before calling this function. The function should be used when there
 * is a need to change the direction of the GPIO pins, and it must be
 * called after the device has been properly initialized.
 *
 * @param device Pointer to the Talise data structure. Must not be null, and the
 * device must be initialized before use.
 * @param gpio3v3OutEn A 16-bit value where each bit represents the direction of
 * a GPIO pin (0 for input, 1 for output). Valid range is 0
 * to 0x0FFF. Values outside this range will result in an
 * error.
 * @param gpio3v3UsedMask A 16-bit mask indicating which GPIO pins' directions
 * should be modified. If a bit is set to 1, the
 * corresponding pin's direction will be updated based on
 * `gpio3v3OutEn`.
 * @return Returns a uint32_t indicating the result of the operation. Possible
 * values include TALACT_NO_ACTION for success, TALACT_ERR_CHECK_PARAM
 * for invalid parameters, and TALACT_ERR_RESET_SPI for SPI errors.
 ******************************************************************************/
uint32_t TALISE_setGpio3v3Oe(taliseDevice_t *device, uint16_t gpio3v3OutEn,
			     uint16_t gpio3v3UsedMask);

/***************************************************************************//**
 * @brief Use this function to obtain the current direction configuration of the
 * 3.3V GPIO pins on the Talise device. This function is useful for
 * verifying the current GPIO setup, especially after configuration
 * changes. It must be called with a valid device pointer and a non-null
 * pointer to store the GPIO direction data. The function will return an
 * error if the provided pointer for storing the output is null.
 *
 * @param device Pointer to the Talise device structure. Must not be null and
 * should be properly initialized before calling this function.
 * @param gpio3v3OutEn Pointer to a uint16_t variable where the function will
 * store the current GPIO direction settings. Each bit
 * represents a GPIO pin's direction: 0 for input, 1 for
 * output. Must not be null.
 * @return Returns a uint32_t value indicating the status of the operation.
 * Possible return values include success or specific error codes
 * related to parameter validation or SPI communication issues.
 ******************************************************************************/
uint32_t TALISE_getGpio3v3Oe(taliseDevice_t *device, uint16_t *gpio3v3OutEn);

/***************************************************************************//**
 * @brief This function configures the output source control for the 3.3V GPIO
 * pins on a Talise device. It is used to assign a specific source to
 * each group of four GPIO pins, known as a nibble. The function should
 * be called when the GPIO pins are set to output mode. The source
 * control value is a 12-bit number, with each nibble controlling a set
 * of four pins. Ensure that the device is properly initialized before
 * calling this function. The function returns a status code indicating
 * the success or type of error encountered during execution.
 *
 * @param device Pointer to the Talise device structure. Must not be null. The
 * caller retains ownership.
 * @param gpio3v3SrcCtrl A 12-bit value specifying the source control for each
 * set of four GPIO pins. Valid range is 0 to 0x0FFF.
 * Values outside this range will result in an error.
 * @return Returns a uint32_t status code indicating the result of the
 * operation: TALACT_NO_ACTION for success, or an error code for
 * failure.
 ******************************************************************************/
uint32_t TALISE_setGpio3v3SourceCtrl(taliseDevice_t *device,
				     uint16_t gpio3v3SrcCtrl);

/***************************************************************************//**
 * @brief Use this function to obtain the current source control configuration
 * for the 3.3V GPIO pins on a Talise device. This function is typically
 * called to verify or log the current GPIO source settings. It requires
 * a valid device structure and a non-null pointer to store the retrieved
 * source control value. If the pointer is null, the function will handle
 * the error and return an appropriate error code. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param device Pointer to the Talise device structure. Must not be null and
 * should be properly initialized before use.
 * @param gpio3v3SrcCtrl Pointer to a uint16_t variable where the GPIO source
 * control settings will be stored. Must not be null. If
 * null, the function will return an error.
 * @return Returns a uint32_t value indicating the status of the operation.
 * Possible return values include success or specific error codes
 * related to parameter validation or SPI communication issues.
 ******************************************************************************/
uint32_t TALISE_getGpio3v3SourceCtrl(taliseDevice_t *device,
				     uint16_t *gpio3v3SrcCtrl);

/***************************************************************************//**
 * @brief Use this function to set the output level of the 3.3V GPIO pins on the
 * Talise device. It is applicable only to GPIO pins configured as
 * outputs and with the correct source control settings. Ensure that the
 * GPIO pins are set to output mode before calling this function. The
 * function checks if the provided pin level is within the valid range
 * and returns an error if it is not. This function should be called
 * after the device has been properly initialized and configured.
 *
 * @param device Pointer to the Talise device structure. Must not be null, and
 * the device should be initialized before use.
 * @param gpio3v3PinLevel A 16-bit value representing the desired output level
 * for each GPIO pin, with each bit corresponding to a
 * pin. Valid range is 0 to 0x0FFF. Values outside this
 * range will result in an error.
 * @return Returns a uint32_t indicating the status of the operation. Possible
 * values include TALACT_NO_ACTION for success, TALACT_ERR_CHECK_PARAM
 * for invalid parameters, and other error codes for specific failures.
 ******************************************************************************/
uint32_t TALISE_setGpio3v3PinLevel(taliseDevice_t *device,
				   uint16_t gpio3v3PinLevel);

/***************************************************************************//**
 * @brief Use this function to read the current levels of the 3.3V GPIO pins on
 * the Talise device. It is useful for determining the state of GPIO pins
 * configured as inputs. Ensure that the device is properly initialized
 * before calling this function. The function will return an error if the
 * provided pointer for the pin level is null.
 *
 * @param device Pointer to the Talise data structure. Must not be null and
 * should be properly initialized before use.
 * @param gpio3v3PinLevel Pointer to a uint16_t variable where the GPIO pin
 * levels will be stored. Must not be null. The function
 * will return an error if this parameter is null.
 * @return Returns a uint32_t value indicating the status of the operation. The
 * value can represent success or various error conditions, such as
 * parameter errors or SPI communication issues.
 ******************************************************************************/
uint32_t TALISE_getGpio3v3PinLevel(taliseDevice_t *device,
				   uint16_t *gpio3v3PinLevel);

/***************************************************************************//**
 * @brief Use this function to obtain the current output levels of the 3.3V GPIO
 * pins configured in BITBANG mode. It is essential to ensure that the
 * `device` is properly initialized before calling this function. The
 * function will populate the provided `gpio3v3PinSetLevel` with the
 * output levels of each GPIO pin, where each bit represents the level of
 * a corresponding pin. This function is useful for verifying the current
 * state of GPIO outputs in applications where GPIO pin levels are
 * critical.
 *
 * @param device Pointer to the Talise device structure. Must not be null and
 * should be initialized before use.
 * @param gpio3v3PinSetLevel Pointer to a uint16_t variable where the function
 * will store the output levels of the 3.3V GPIO pins.
 * Must not be null.
 * @return Returns a uint32_t indicating the status of the operation, with
 * specific values representing different recovery actions or success.
 ******************************************************************************/
uint32_t TALISE_getGpio3v3SetLevel(taliseDevice_t *device,
				   uint16_t *gpio3v3PinSetLevel);

/***************************************************************************//**
 * @brief This function is used to determine the source of a General Purpose
 * Interrupt (GP_INT) on the Talise device, clear the interrupt if
 * possible, and provide a recovery action. It should be called whenever
 * a GP_INT assertion is detected. The function requires the device to be
 * initialized and the interrupt mask bits to be set. It provides
 * detailed diagnostic information if requested, and handles various
 * error conditions by returning appropriate recovery actions.
 *
 * @param device Pointer to the Talise data structure. Must not be null and
 * should be properly initialized before calling this function.
 * @param gpIntStatus Pointer to a uint32_t variable where the status of the
 * GP_INT source registers will be stored. Must not be null.
 * If null, the function will return an error.
 * @param gpIntDiag Pointer to a taliseGpIntInformation_t structure for
 * returning detailed diagnostic information about the GP_INT
 * source. Can be null if diagnostic information is not
 * required.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_WARN_RESET_LOG,
 * TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_ARM,
 * TALACT_ERR_RERUN_INIT_CALS, TALACT_ERR_RESET_JESD204FRAMERA,
 * TALACT_ERR_RESET_JESD204FRAMERB, TALACT_ERR_RESET_FULL,
 * TALACT_ERR_RESET_JESD204DEFRAMERA, TALACT_ERR_RESET_JESD204DEFRAMERB,
 * TALACT_ERR_REDUCE_TXSAMPLE_PWR, TALACT_ERR_BBIC_LOG_ERROR, and
 * TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_gpIntHandler(taliseDevice_t *device, uint32_t *gpIntStatus,
			     taliseGpIntInformation_t *gpIntDiag);

/***************************************************************************//**
 * @brief This function assigns a specified GPIO pin to the Aux ADC start
 * signal, allowing the ARM processor to initiate an ADC conversion upon
 * detecting a low-to-high pulse on the assigned pin. It is essential to
 * ensure that the GPIO pin is not already in use by another feature. The
 * function can also unassign the GPIO pin by passing a specific invalid
 * GPIO value. This operation is only supported when the ARM processor is
 * in the IDLE state, requiring the radio to be turned off before setting
 * the GPIO pin. Only GPIO pins 0 to 15 are supported for this purpose.
 *
 * @param device Pointer to the Talise data structure. Must not be null.
 * @param pinModeGpio Specifies the GPIO pin to be assigned or unassigned from
 * the Aux ADC start signal. Valid values are TAL_GPIO0 to
 * TAL_GPIO15 for assignment, and TAL_GPIO_INVALID for
 * unassignment. If the pin is already in use, the function
 * will handle it as an error.
 * @return Returns a uint32_t indicating the success or type of error
 * encountered. Possible return values include TALACT_NO_ACTION for
 * success, TALACT_ERR_CHECK_PARAM for invalid parameters, and other
 * error codes for specific issues.
 ******************************************************************************/
uint32_t TALISE_setAuxAdcPinModeGpio(taliseDevice_t *device,
				     taliseGpioPinSel_t pinModeGpio);

/***************************************************************************//**
 * @brief Use this function to determine which GPIO pin is currently assigned to
 * trigger the Aux ADC start signal in pin mode operations. This function
 * should be called after the device has been initialized and the ARM
 * processor is in the appropriate state to handle GPIO control commands.
 * It is important to ensure that the GPIO pin is not being used by
 * another feature. The function will return a valid GPIO pin number if
 * one is assigned, or TAL_GPIO_INVALID if no valid GPIO is associated.
 *
 * @param device Pointer to the Talise device structure. Must not be null and
 * should be properly initialized before calling this function.
 * @param pinModeGpio Pointer to a taliseGpioPinSel_t variable where the
 * function will store the GPIO pin currently associated with
 * the Aux ADC start signal. Must not be null.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation, with specific error codes for different failure modes.
 ******************************************************************************/
uint32_t TALISE_getAuxAdcPinModeGpio(taliseDevice_t *device,
				     taliseGpioPinSel_t *pinModeGpio);

/***************************************************************************//**
 * @brief This function is used to configure and initiate the auxiliary ADC on
 * the Talise device for sampling and A/D conversion. It should be called
 * after the device has been initialized and can be used in both Radio ON
 * and Radio OFF states. The function requires a valid configuration
 * structure to specify the ADC channel, mode, number of samples, and
 * sampling period. It performs parameter validation and returns an error
 * code if any parameter is out of range. This function is essential for
 * setting up the ADC for external use, and the results can be retrieved
 * using a separate API call.
 *
 * @param device Pointer to the Talise device structure. Must not be null.
 * @param auxAdcConfig Pointer to the configuration structure for the auxiliary
 * ADC. Must not be null. The structure should specify a
 * valid channel (0-3), mode (pin or non-pin), number of
 * samples (1-1000), and a sampling period of at least 15
 * microseconds for non-pin mode.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. Possible return values include error codes for invalid
 * parameters or successful completion.
 ******************************************************************************/
uint32_t TALISE_startAuxAdc(taliseDevice_t *device,
			    taliseAuxAdcConfig_t *auxAdcConfig);

/***************************************************************************//**
 * @brief Use this function to obtain the result of an auxiliary ADC conversion
 * that was previously initiated. It should be called after the device
 * has been initialized and a conversion has been started using the
 * appropriate API. The function updates the provided data structure with
 * the conversion result, which includes the average ADC code, the number
 * of samples, and a completion indicator. Ensure that the pointer to the
 * result structure is not null before calling this function.
 *
 * @param device Pointer to the Talise device structure. Must not be null. The
 * caller retains ownership.
 * @param auxAdcResult Pointer to a taliseAuxAdcResult_t structure where the
 * conversion result will be stored. Must not be null. If
 * null, the function returns an error.
 * @return Returns a uint32_t value indicating the status of the operation. A
 * successful operation returns TALACT_NO_ACTION, while errors are
 * indicated by other return values.
 ******************************************************************************/
uint32_t TALISE_readAuxAdc(taliseDevice_t *device,
			   taliseAuxAdcResult_t *auxAdcResult);


/***************************************************************************//**
 * @brief Use this function to obtain a human-readable error message
 * corresponding to a specific GPIO error source and error code. This is
 * useful for debugging and logging purposes when working with the Talise
 * API. The function should be called whenever an error code is returned
 * from a GPIO-related API function to understand the nature of the
 * error. The function returns an empty string if verbose error messages
 * are not enabled.
 *
 * @param errSrc A value representing the error source from the Talise API. It
 * must be a valid error source identifier related to GPIO
 * operations.
 * @param errCode An error code that, in conjunction with the error source,
 * identifies a specific error. It must be a valid error code
 * associated with the provided error source.
 * @return Returns a constant character pointer to a string containing the error
 * message. If verbose error messages are not enabled, an empty string
 * is returned.
 ******************************************************************************/
const char* talGetGpioErrorMessage(uint32_t errSrc, uint32_t errCode);

#ifdef __cplusplus
}
#endif

#endif /* TALISE_GPIO_H_ */
