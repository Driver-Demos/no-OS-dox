/**
 * \file
 * \brief Contains ADRV9001 init related private function prototypes for
 *        adrv9001_init.c that helps adi_adrv9001_init.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADRV9001_INIT_H_
#define _ADRV9001_INIT_H_

#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C" {
#endif

//#define ADRV9001_INIT_DEBUG 1
#ifdef ADRV9001_INIT_DEBUG
#include <stdio.h>
#define ADRV9001_DEBUG_INFO(x) printf("MESSAGE: %s ******************************* \n", (x));
#define ADRV9001_DEBUG_INFO_NUM(x,n) printf("MESSAGE: %s: %d 0x%08x \n", (x),(n),(n));
#else
#define ADRV9001_DEBUG_INFO(x)
#define ADRV9001_DEBUG_INFO_NUM(x,n)
#endif

//#define ADRV9001_INIT_DMAINFO_DEBUG 1
#ifdef ADRV9001_INIT_DMAINFO_DEBUG
#include <stdio.h>
#define ADRV9001_DMAINFO(text, addr, count) printf("MESSAGE: DMA: %30s: addr=0x%08x, count=%d \n", (text), (addr), (count));
#else
#define ADRV9001_DMAINFO(text, addr, count)
#endif

//#define ADRV9001_INIT_DMA_DEBUG 1
#ifdef ADRV9001_INIT_DMA_DEBUG
#include <stdio.h>
#define ADRV9001_SPIDMAINFO(s,a,b,c) printf((s),(a),(b),(c));
#else
#define ADRV9001_SPIDMAINFO(s,a,b,c)
#endif

#define ADRV9001_SPIWRITEBYTESDMA(devicePtr, text, addr, addrArray, dataArray, count) \
{\
    int32_t recoveryAction = 0; \
    recoveryAction = adi_adrv9001_spi_Bytes_Write(devicePtr, (addrArray), (dataArray), (count)); \
    ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
    ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Error while writing bytes to DMA over SPI"); \
    ADRV9001_DMAINFO((text), (addr), (count)); \
}


#define ADRV9001_SPIWRITEBYTEDMA(devicePtr, text, addr, data) \
    {\
        int32_t recoveryAction = 0; \
        recoveryAction = adi_adrv9001_spi_Byte_Write(devicePtr, (addr), (data)); \
        ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Error while writing byte to DMA over SPI"); \
        ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
        ADRV9001_SPIDMAINFO("MESSAGE: WRITE: %30s: addr=0x%04x, data=0x%02x \n", (text), (addr), (data)); \
    }

#define ADRV9001_SPIWRITEBYTESTREAMDMA(devicePtr, text, addr, dataArray, count) \
    {\
        int32_t recoveryAction = 0; \
        recoveryAction = adi_adrv9001_spi_Bytes_Stream_Write(devicePtr, (addr), (dataArray), count); \
        ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Error while writing byte to DMA over SPI"); \
        ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
        ADRV9001_DMAINFO((text), (addr), (count)); \
    }

#define ADRV9001_SPIREADBYTEDMA(devicePtr, text, addr, data) \
    {\
        recoveryAction = adi_adrv9001_spi_Byte_Read(devicePtr, (addr), (data)); \
        ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Error while Reading byte from DMA over SPI"); \
        ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
        ADRV9001_SPIDMAINFO("MESSAGE:  READ: %30s: addr=0x%04x, data=0x%02x \n", (text), (addr), (*(uint8_t*)(data))); \
    }


//#define ADRV9001_INIT_SPI_DEBUG 1
#ifdef ADRV9001_INIT_SPI_DEBUG
#include <stdio.h>
#define ADRV9001_SPIINFO(s,a,b,c) printf((s),(a),(b),(c));
#define ADRV9001_SPI_FIELD_INFO(s,a,b,c,d) printf((s),(a),(b),(c), (d));
#else
#define ADRV9001_SPIINFO(s,a,b,c)
#define ADRV9001_SPI_FIELD_INFO(s,a,b,c,d)
#endif

#define ADRV9001_SPIINFO_NO(s,a,b,c)

#define ADRV9001_SPIWRITEBYTE(devicePtr, text, addr, data) \
    {\
        int32_t recoveryAction = 0; \
        recoveryAction = adi_adrv9001_spi_Byte_Write(devicePtr, (addr), (data)); \
        ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Error while writing byte to SPI"); \
        ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
        ADRV9001_DEBUG_INFO(__FUNCTION__); \
        ADRV9001_SPIINFO("MESSAGE: WRITE: %30s: addr=0x%04x, data=0x%02x \n", (text), (addr), (data)); \
    }

#define ADRV9001_SPIREADBYTE(devicePtr, text, addr, data) \
    {\
        int32_t recoveryAction = 0;\
        recoveryAction = adi_adrv9001_spi_Byte_Read(devicePtr, (addr), (data)); \
        ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Error while reading byte from SPI"); \
        ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
        ADRV9001_DEBUG_INFO(__FUNCTION__); \
        ADRV9001_SPIINFO("MESSAGE:  READ: %30s: addr=0x%04x, data=0x%02x \n", (text), (addr), (*(uint8_t*)(data))); \
    }

#define ADRV9001_SPIFIELDWRITE(devicePtr, addr, fieldVal, mask, startBit, text) \
{ \
int32_t recoveryAction = 0;\
recoveryAction = adi_adrv9001_spi_Field_Write(devicePtr, addr, fieldVal, mask, startBit); \
ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Error on SPI field write"); \
ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
ADRV9001_SPI_FIELD_INFO("MESSAGE: WRITE FIELD: %30s: addr=0x%04x, data=0x%02x \n", (text), (addr), (fieldVal), (startBit)); \
}


#define ADRV9001_SPIFIELDREAD(devicePtr, addr, fieldVal, mask, startBit, text) \
{ \
int32_t recoveryAction = 0;\
recoveryAction = adi_adrv9001_spi_Field_Read(devicePtr, addr, (fieldVal), mask, startBit); \
    ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, ADI_COMMON_ERR_API_FAIL, recoveryAction, NULL, "Error on SPI field read"); \
ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
ADRV9001_SPI_FIELD_INFO("MESSAGE: READ: %30s: addr=0x%04x, data=0x%02x \n", (text), (addr), (fieldVal), (startBit)); \
}

/***************************************************************************//**
 * @brief This function configures the analog registers of the ADRV9001 device
 * based on the provided initialization settings. It should be used as
 * part of the device initialization process to ensure that the device's
 * analog components are correctly configured. The function requires
 * valid device and initialization structures and a clock output divisor.
 * It performs validation and configuration tasks, including setting
 * master bias, enabling reference clocks, and verifying profiles. The
 * function handles errors related to invalid clock divisors and ensures
 * that the internal reference clock and device clock outputs do not
 * exceed specified maximum frequencies.
 *
 * @param device A pointer to an adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. Must not be null. The caller retains
 * ownership.
 * @param init A pointer to an adi_adrv9001_Init_t structure containing
 * initialization settings for the ADRV9001 device. Must not be
 * null. The caller retains ownership.
 * @param adrv9001DeviceClockOutDivisor An enum value of type
 * adi_adrv9001_DeviceClockDivisor_e,
 * representing the device clock output
 * divisor. Valid values range from 0 to 6,
 * corresponding to divisors of 1, 2, 4, 8,
 * 16, 32, and 64.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. Possible return values include
 * ADI_COMMON_ACT_ERR_CHECK_PARAM for parameter errors and
 * ADI_COMMON_ACT_NO_ACTION for successful completion.
 ******************************************************************************/
int32_t adrv9001_InitAnalog(adi_adrv9001_Device_t *device,
                            adi_adrv9001_Init_t *init,
                            adi_adrv9001_DeviceClockDivisor_e adrv9001DeviceClockOutDivisor);
    
/***************************************************************************//**
 * @brief This function checks the validity of the Rx, Tx, and ORx profiles
 * within the provided initialization structure to ensure they have
 * compatible clock rates for operation. It verifies that the profiles
 * can share a common high-speed digital clock and returns an error if
 * any invalid profile combinations are detected. Profiles with an IQ
 * rate of zero are considered disabled. This function should be called
 * with a properly initialized device and initialization structure, and
 * it will update the device state information based on the verification
 * results.
 *
 * @param device A pointer to the ADRV9001 device data structure. Must not be
 * null. The function updates the device's state information based
 * on the profile verification.
 * @param init A pointer to the ADRV9001 initialization settings structure. Must
 * not be null. The function checks the profiles within this
 * structure for validity.
 * @return Returns an integer status code. A return value of
 * ADI_COMMON_ACT_NO_ACTION indicates success, while
 * ADI_COMMON_ACT_ERR_CHECK_PARAM indicates a parameter error.
 ******************************************************************************/
int32_t adrv9001_ProfilesVerify(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init);

/***************************************************************************//**
 * @brief This function configures the analog clock settings for the ADRV9001
 * device based on the initialization parameters provided. It should be
 * used during the device initialization process to ensure that the clock
 * settings are correctly applied. The function requires valid clock PLL
 * mode and high-speed divider settings in the initialization structure.
 * If invalid parameters are provided, the function will report an error
 * and return a recovery action. This function is intended for internal
 * use and is not called directly by the user.
 *
 * @param device Structure pointer to ADRV9001 device data structure. Must not
 * be null. Caller retains ownership.
 * @param init Structure pointer to ADRV9001 initialization settings. Must
 * contain valid clock PLL mode and high-speed divider settings.
 * Caller retains ownership.
 * @return Returns an integer indicating the success or failure of the
 * operation. Possible return values include
 * ADI_COMMON_ACT_ERR_CHECK_PARAM for parameter errors and
 * ADI_COMMON_ACT_NO_ACTION for successful completion.
 ******************************************************************************/
int32_t adrv9001_AnalogClockSet(adi_adrv9001_Device_t *device, adi_adrv9001_Init_t *init);

#ifdef __cplusplus
}
#endif

#endif