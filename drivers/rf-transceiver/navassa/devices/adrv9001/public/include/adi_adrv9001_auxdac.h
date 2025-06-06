/**
 * \file
 * \brief ADRV9001 AUX DAC header file
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_AUXDAC_H_
#define _ADI_ADRV9001_AUXDAC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_adrv9001_auxdac_types.h"
#include "adi_adrv9001_types.h"

/***************************************************************************//**
 * @brief Use this function to enable or disable one of the four available
 * AuxDACs on the ADRV9001 device. Each AuxDAC corresponds to a specific
 * GPIO analog pin, and this function allows you to control whether the
 * selected AuxDAC is active. This function should be called when you
 * need to change the operational state of an AuxDAC, such as during
 * initialization or when modifying the device's configuration. Ensure
 * that the device context is properly initialized before calling this
 * function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, and the device should be properly initialized
 * before use.
 * @param auxDac The specific AuxDAC to configure, represented by an
 * adi_adrv9001_AuxDac_e enumeration. Valid values are
 * ADI_ADRV9001_AUXDAC0 through ADI_ADRV9001_AUXDAC3.
 * @param enable Boolean value indicating whether to enable (true) or disable
 * (false) the selected AuxDAC.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_AuxDac_Configure(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_AuxDac_e auxDac, bool enable);

/***************************************************************************//**
 * @brief Use this function to check whether a specific AuxDAC on the ADRV9001
 * device is currently enabled or disabled. This function is useful for
 * verifying the current configuration of the AuxDACs, especially after
 * configuration changes. It must be called with a valid device context
 * and a specific AuxDAC identifier. The function will output the enable
 * status through the provided pointer, indicating whether the selected
 * AuxDAC is active. Ensure that the `enabled` pointer is not null before
 * calling this function.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param auxDac Select AuxDAC to read back the configuration. Must be a valid
 * AuxDAC identifier (e.g., ADI_ADRV9001_AUXDAC0 to
 * ADI_ADRV9001_AUXDAC3).
 * @param enabled Pointer to a boolean where the enable status will be stored.
 * Must not be null. The function writes 'true' if the AuxDAC is
 * enabled, 'false' otherwise.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_AuxDac_Inspect(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_AuxDac_e auxDac, bool *enabled);

/***************************************************************************//**
 * @brief Use this function to set a 12-bit digital-to-analog converter (DAC)
 * code for one of the four available AuxDACs on the ADRV9001 device.
 * This function is typically called when you need to configure the
 * output voltage of a specific AuxDAC channel, which corresponds to a
 * GPIO analog pin. Ensure that the device context is properly
 * initialized before calling this function. The function will validate
 * the inputs and perform the necessary register writes to set the DAC
 * code. It returns a status code indicating success or the type of error
 * encountered.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be properly initialized before calling
 * this function. The caller retains ownership.
 * @param auxDac The selected AuxDAC to configure. Must be one of the valid
 * AuxDAC enumerations (e.g., ADI_ADRV9001_AUXDAC0 to
 * ADI_ADRV9001_AUXDAC3). Invalid values will result in no
 * operation.
 * @param code The 12-bit DAC code to set for the selected AuxDAC. Valid range
 * is 0 to 4095. Values outside this range may result in undefined
 * behavior.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adi_adrv9001_AuxDac_Code_Set(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_AuxDac_e auxDac, uint16_t code);

/***************************************************************************//**
 * @brief Use this function to retrieve the current 12-bit DAC code from a
 * specified AuxDAC on the ADRV9001 device. This function is useful for
 * verifying the DAC output settings of the device. It must be called
 * with a valid device context and a valid AuxDAC identifier. The
 * function will populate the provided pointer with the DAC code if
 * successful. Ensure that the device is properly initialized before
 * calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be properly initialized before calling
 * this function.
 * @param auxDac Specifies which AuxDAC to read from. Must be a valid value of
 * type adi_adrv9001_AuxDac_e, corresponding to one of the
 * available AuxDACs (AuxDAC0 to AuxDAC3).
 * @param code Pointer to a uint16_t where the DAC code will be stored. Must not
 * be null. The function writes the DAC code of the specified AuxDAC
 * to this location.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The DAC code is written to the
 * location pointed to by the 'code' parameter if successful.
 ******************************************************************************/
int32_t adi_adrv9001_AuxDac_Code_Get(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_AuxDac_e auxDac, uint16_t *code);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_AUXDAC_H_ */
