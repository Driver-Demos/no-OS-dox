/**
 * \file
 * \brief Contains functions to configure AUX DAC channels on the ADRV9001 device
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_AUXDAC_TYPES_H_
#define _ADI_ADRV9001_AUXDAC_TYPES_H_

/***************************************************************************//**
 * @brief The `adi_adrv9001_AuxDac_e` is an enumeration that defines constants
 * for selecting one of the four auxiliary DAC channels available on the
 * ADRV9001 device. Each enumerator corresponds to a specific DAC
 * channel, allowing for easy configuration and selection within the
 * device's API.
 *
 * @param ADI_ADRV9001_AUXDAC0 Represents the first auxiliary DAC channel.
 * @param ADI_ADRV9001_AUXDAC1 Represents the second auxiliary DAC channel.
 * @param ADI_ADRV9001_AUXDAC2 Represents the third auxiliary DAC channel.
 * @param ADI_ADRV9001_AUXDAC3 Represents the fourth auxiliary DAC channel.
 ******************************************************************************/
typedef enum adi_adrv9001_AuxDac
{
    ADI_ADRV9001_AUXDAC0, /*!< AuxDAC0 */
    ADI_ADRV9001_AUXDAC1, /*!< AuxDAC1 */
    ADI_ADRV9001_AUXDAC2, /*!< AuxDAC2 */
    ADI_ADRV9001_AUXDAC3, /*!< AuxDAC3 */
} adi_adrv9001_AuxDac_e;

#endif /* _ADI_ADRV9001_AUXDAC_TYPES_H_ */
