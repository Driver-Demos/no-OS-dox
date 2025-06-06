/**
 * \file
 * \brief ADRV9001 GPIO header file
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_GPIO_H_
#define _ADI_ADRV9001_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_common_error.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_types.h"

/***************************************************************************//**
 * @brief This function sets the bit mask for the General Purpose (GP) interrupt
 * register, allowing the user to specify which interrupts should be
 * enabled or disabled. It should be called after the device has been
 * initialized. The function provides direct register access to configure
 * the interrupt mask, which determines the sources that can trigger the
 * GP_INT pin. This is useful for managing interrupt-driven events in the
 * ADRV9001 device.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null. The caller retains
 * ownership.
 * @param gpIntMask The GP interrupt masks to write. It is a 32-bit unsigned
 * integer where each bit represents a different interrupt
 * source. The caller is responsible for setting the
 * appropriate bits to enable or disable specific interrupts.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_GpIntMask_Set(adi_adrv9001_Device_t *adrv9001, uint32_t gpIntMask);

/***************************************************************************//**
 * @brief This function is used to obtain the current bit mask of the General
 * Purpose (GP) interrupt register for the GP_INT pin. It should be
 * called after the device has been initialized to ensure that the device
 * context is valid. The function provides the current interrupt mask
 * settings, which can be useful for understanding which interrupts are
 * currently enabled. It is important to ensure that the `gpIntMask`
 * pointer is valid and not null before calling this function, as it will
 * be used to store the retrieved mask value.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null. The caller retains
 * ownership.
 * @param gpIntMask Pointer to a uint32_t where the GP interrupt masks will be
 * stored. Must not be null. The function writes the retrieved
 * mask value to this location.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_GpIntMask_Get(adi_adrv9001_Device_t *adrv9001, uint32_t *gpIntMask);

/***************************************************************************//**
 * @brief Use this function to determine the source of a General Purpose
 * interrupt after detecting a rising edge on the GP_INT pin. It should
 * be called any time after device initialization and after setting the
 * interrupt mask bits. Note that reading the status will clear the
 * status bits, so it should not be called before handling the interrupt
 * with adi_adrv9001_gpio_GpIntHandler. This function provides a status
 * word indicating the interrupt sources, even if they are masked.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param gpIntStatus Pointer to a variable where the status read-back word will
 * be stored. Must not be null. The status word will indicate
 * the current value for all interrupt sources.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_GpIntStatus_Get(adi_adrv9001_Device_t *adrv9001, uint32_t *gpIntStatus);

/***************************************************************************//**
 * @brief This function sets a specified GPIO pin on the ADRV9001 device to a
 * desired logic level, either high or low. It is applicable to both
 * digital and analog GPIO pins, provided they are configured as outputs
 * and have the correct source control settings. The function should be
 * called after the device has been initialized. It returns a status code
 * indicating success or the necessary action to recover from an error.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param pin The GPIO pin to set to the desired level. Valid values are within
 * the range of defined digital and analog GPIO pins.
 * @param level The logic level to set the pin to, either high or low. Invalid
 * levels will result in an error.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_OutputPinLevel_Set(adi_adrv9001_Device_t *adrv9001,
                                             adi_adrv9001_GpioPin_e pin, 
                                             adi_adrv9001_GpioPinLevel_e level);

/***************************************************************************//**
 * @brief Use this function to retrieve the current output level of a specified
 * GPIO pin on the ADRV9001 device. It is applicable for both digital and
 * analog GPIO pins, provided they are configured for output. This
 * function should be called after the device has been initialized and
 * the GPIO pins have been configured appropriately. Ensure that the pin
 * specified is within the valid range for digital or analog pins, as the
 * function will not execute for invalid pin numbers.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, as it provides the context for the operation.
 * @param pin The GPIO pin for which to get the output level. Must be a valid
 * digital or analog GPIO pin identifier within the defined range.
 * @param gpioOutPinLevel Pointer to a variable where the current output level
 * of the specified pin will be stored. Must not be null.
 * @return Returns an integer code indicating success or the required action to
 * recover. The output level of the specified pin is stored in the
 * variable pointed to by gpioOutPinLevel.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_OutputPinLevel_Get(adi_adrv9001_Device_t *adrv9001,
                                             adi_adrv9001_GpioPin_e pin,
                                             adi_adrv9001_GpioPinLevel_e *gpioOutPinLevel);

/***************************************************************************//**
 * @brief This function is used to read the current input level of a specified
 * GPIO pin on the ADRV9001 device. It can be called any time after the
 * device has been initialized. The function supports both digital and
 * analog GPIO pins, and it returns the logic level of the specified pin
 * through the output parameter. The caller must ensure that the device
 * pointer and the output parameter are not null. If the specified pin is
 * not within the valid range for digital or analog pins, the function
 * will not execute as expected.
 *
 * @param device A pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param pin The GPIO pin for which to get the input level. Valid values are
 * within the range of digital pins (ADI_ADRV9001_GPIO_DIGITAL_00 to
 * ADI_ADRV9001_GPIO_DIGITAL_15) or analog pins
 * (ADI_ADRV9001_GPIO_ANALOG_00 to ADI_ADRV9001_GPIO_ANALOG_11).
 * @param gpioInPinLevel A pointer to a variable where the input level of the
 * specified pin will be stored. Must not be null. The
 * function writes the logic level (0 or 1) to this
 * location.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. The input level of the specified pin is written to the
 * location pointed to by gpioInPinLevel.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_InputPinLevel_Get(adi_adrv9001_Device_t *adrv9001, 
                                            adi_adrv9001_GpioPin_e pin,
                                            adi_adrv9001_GpioPinLevel_e *gpioInPinLevels);

/***************************************************************************//**
 * @brief Use this function to determine whether a specified GPIO pin on the
 * ADRV9001 device is configured as an input or output. This function
 * should be called after the device has been initialized. It supports
 * both digital and analog GPIO pins, and the pin must be within the
 * valid range for its type. The function will return an error code if
 * the pin is not valid or if any other issue occurs during execution.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null, and the device must be initialized before calling
 * this function.
 * @param pin The GPIO pin for which to get the direction. Must be a valid pin
 * identifier within the range of digital
 * (ADI_ADRV9001_GPIO_DIGITAL_00 to ADI_ADRV9001_GPIO_DIGITAL_15) or
 * analog (ADI_ADRV9001_GPIO_ANALOG_00 to
 * ADI_ADRV9001_GPIO_ANALOG_11) pins.
 * @param direction Pointer to a variable where the current direction of the pin
 * will be stored. Must not be null. The direction will be set
 * to indicate input or output.
 * @return Returns an integer code indicating success or the required action to
 * recover from an error. The direction of the specified pin is written
 * to the location pointed to by the 'direction' parameter.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_PinDirection_Get(adi_adrv9001_Device_t *device,
                                          adi_adrv9001_GpioPin_e pin,
                                          adi_adrv9001_GpioPinDirection_e *direction);

/***************************************************************************//**
 * @brief This function configures a specified GPIO pin on the ADRV9001 device
 * to operate as a manual input. It should be called after the device has
 * been initialized. The function modifies the direction of the specified
 * GPIO pin to input mode, allowing it to read external signals. This is
 * useful in applications where the GPIO pin needs to be used for input
 * purposes, such as reading sensor data or external control signals.
 * Ensure that the device context and pin parameters are valid before
 * calling this function.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * This must not be null and should point to a properly
 * initialized device context.
 * @param pin The GPIO pin to configure as a manual input. It must be a valid
 * pin identifier as defined by the adi_adrv9001_GpioPin_e
 * enumeration.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_ManualInput_Configure(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_GpioPin_e pin);
    
/***************************************************************************//**
 * @brief This function configures a specified GPIO pin crumb on the ADRV9001
 * device to operate as a manual output. It should be called after the
 * device has been initialized. The function sets the direction of the
 * specified GPIO pins to output and configures the source control for
 * the GPIO crumbs. This is useful for applications requiring direct
 * control over GPIO outputs. Ensure that the device context is valid and
 * that the crumb parameter is within the valid range before calling this
 * function.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null. The caller retains
 * ownership.
 * @param crumb The GPIO pin crumb to configure. Must be a valid value of type
 * adi_adrv9001_GpioPinCrumbSel_e. Invalid values may result in
 * undefined behavior.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_ManualOutput_Configure(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_GpioPinCrumbSel_e crumb);

/***************************************************************************//**
 * @brief This function configures a specified analog GPIO pin on the ADRV9001
 * device to operate as a manual input. It should be called after the
 * device has been initialized and is used when there is a need to
 * manually control the input state of an analog GPIO pin. The function
 * modifies the direction control register to ensure the pin is set as an
 * input, allowing it to receive signals. This function is part of the
 * direct register access operations and is essential for applications
 * requiring precise control over GPIO pin configurations.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null, as it provides the
 * necessary context for the operation.
 * @param pin The analog GPIO pin to configure. Must be a valid pin identifier
 * as defined in the adi_adrv9001_GpioPin_e enumeration. Invalid pin
 * values will result in an error.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_ManualAnalogInput_Configure(adi_adrv9001_Device_t *adrv9001, 
                                                      adi_adrv9001_GpioPin_e pin);
/***************************************************************************//**
 * @brief This function configures a specified analog GPIO pin nibble on the
 * ADRV9001 device to operate as a manual output. It should be called
 * after the device has been initialized. The function sets the direction
 * of the pins in the selected nibble to output and configures the source
 * for the output signal. This is useful for applications requiring
 * direct control over the analog GPIO outputs. Ensure that the device
 * context is valid and properly initialized before calling this
 * function.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null. The caller retains
 * ownership.
 * @param nibble The analog GPIO pin nibble to configure. Must be a valid value
 * of type adi_adrv9001_GpioAnalogPinNibbleSel_e. Invalid values
 * may result in undefined behavior.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_ManualAnalogOutput_Configure(adi_adrv9001_Device_t *adrv9001, 
                                                       adi_adrv9001_GpioAnalogPinNibbleSel_e nibble);

/***************************************************************************//**
 * @brief This function is used to configure the GPIO pins of the ADRV9001
 * device for ARM functionality. It should be called after the device has
 * been initialized and is in a state where GPIO configuration is
 * permissible. The function takes a configuration structure that
 * specifies the desired settings for various GPIO signals. Only the pins
 * that are not marked as unassigned in the configuration will be
 * configured. This function returns a status code indicating success or
 * the necessary action to recover from an error.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure containing
 * settings. Must not be null. The caller retains ownership.
 * @param initCfg A pointer to a structure containing the GPIO signal
 * configuration for ARM functionality. Must not be null. The
 * structure should specify the desired configuration for each
 * GPIO signal, with unassigned pins being ignored.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_ControlInit_Configure(adi_adrv9001_Device_t *adrv9001,
                                                adi_adrv9001_GpioCtrlInitCfg_t *gpioCtrlInitCfg);
    
/***************************************************************************//**
 * @brief This function configures a GPIO pin on the ADRV9001 device to handle a
 * specified signal, using the provided GPIO configuration settings. It
 * should be called when the channel is in any of the following states:
 * STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. The function communicates
 * with the ARM processor to apply the configuration, and it returns a
 * status code indicating success or the necessary recovery action.
 * Ensure that the device is properly initialized before calling this
 * function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param signal The signal to configure, specified as an enumeration of type
 * adi_adrv9001_GpioSignal_e.
 * @param gpioConfig Pointer to the desired GPIO configuration of type
 * adi_adrv9001_GpioCfg_t. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_Configure(adi_adrv9001_Device_t *adrv9001,
                                    adi_adrv9001_GpioSignal_e signal,
                                    adi_adrv9001_GpioCfg_t *gpioConfig);
    
/***************************************************************************//**
 * @brief Use this function to obtain the current configuration of a GPIO signal
 * on the ADRV9001 device. It should be called when the channel is in any
 * of the following states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED.
 * This function is useful for verifying the current GPIO settings or for
 * debugging purposes. Ensure that the device pointer is valid and that
 * the gpioConfig pointer is not null before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param signal The GPIO signal for which the configuration is to be retrieved.
 * Must be a valid adi_adrv9001_GpioSignal_e value.
 * @param gpioConfig Pointer to a structure where the current GPIO configuration
 * will be stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_gpio_Inspect(adi_adrv9001_Device_t *adrv9001,
                                  adi_adrv9001_GpioSignal_e signal,
                                  adi_adrv9001_GpioCfg_t *gpioConfig);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_GPIO_H_ */
