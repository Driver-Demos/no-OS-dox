/**
 * \file
 * \brief ADRV9001 AUX ADC header file
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2020 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_AUXADC_H_
#define _ADI_ADRV9001_AUXADC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_common_error.h"
#include "adi_adrv9001_auxadc_types.h"
#include "adi_adrv9001_types.h"

/***************************************************************************//**
 * @brief Use this function to enable or disable a specific auxiliary ADC on the
 * ADRV9001 device. It should be called when the device is in any of the
 * following states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. This
 * function allows for direct register access to configure the power
 * state of the selected auxiliary ADC, either powering it up or down
 * based on the provided parameters. Ensure that the device context is
 * properly initialized before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, and the device should be properly initialized.
 * @param auxAdc The auxiliary ADC to be configured. It must be a valid value of
 * type adi_adrv9001_AuxAdc_e, representing one of the available
 * auxiliary ADCs on the device.
 * @param enable Boolean value indicating whether to power up (true) or power
 * down (false) the selected auxiliary ADC.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_AuxAdc_Configure(adi_adrv9001_Device_t *adrv9001,
                                      adi_adrv9001_AuxAdc_e auxAdc,
                                      bool enable);

/***************************************************************************//**
 * @brief Use this function to determine whether a specific auxiliary ADC
 * (AuxADC) on the ADRV9001 device is currently powered up or down. It is
 * essential to call this function when the channel state is in any of
 * the following states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. This
 * function provides direct register access to read the configuration
 * status of the selected AuxADC. Ensure that the `enabled` pointer is
 * valid and non-null to receive the status result.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, and the device should be properly initialized.
 * @param auxAdc The specific auxiliary ADC to inspect. Must be a valid value of
 * type `adi_adrv9001_AuxAdc_e`.
 * @param enabled Pointer to a boolean where the function will store the power
 * status of the selected AuxADC. Must not be null.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The `enabled` parameter is set to
 * true if the AuxADC is powered up, false if powered down.
 ******************************************************************************/
int32_t adi_adrv9001_AuxAdc_Inspect(adi_adrv9001_Device_t *adrv9001,
                                    adi_adrv9001_AuxAdc_e auxAdc,
                                    bool *enabled);

/***************************************************************************//**
 * @brief Use this function to obtain the voltage in millivolts from a specified
 * auxiliary ADC on the ADRV9001 device. It is essential to ensure that
 * the device is in one of the valid states: STANDBY, CALIBRATED, PRIMED,
 * or RF_ENABLED before calling this function. The function reads the ADC
 * code from the selected AuxADC and converts it to a voltage value in
 * millivolts, which is then stored in the provided output parameter.
 * This function is useful for applications requiring precise voltage
 * measurements from the ADRV9001's auxiliary ADCs.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should point to a valid device context.
 * @param auxAdc Specifies which AuxADC to read from. Must be a valid value of
 * type adi_adrv9001_AuxAdc_e.
 * @param auxAdc_mV Pointer to a uint16_t where the converted voltage in
 * millivolts will be stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_AuxAdc_Voltage_Get(adi_adrv9001_Device_t *adrv9001,
                                        adi_adrv9001_AuxAdc_e auxAdc,
                                        uint16_t *auxAdc_mV);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_AUXADC_H_ */
