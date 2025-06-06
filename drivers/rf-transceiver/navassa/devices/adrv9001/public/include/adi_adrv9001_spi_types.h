/**
 * \file
 * \brief Contains prototypes and macro definitions for ADI HAL wrapper
 *        functions implemented in adi_adrv9001_hal.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef ADRV9001_SPI_TYPES_H_
#define ADRV9001_SPI_TYPES_H_

#ifndef __KERNEL__
#include <stdint.h>
#include <stddef.h>
#endif

#include "adi_adrv9001.h"
#include "adi_common_hal.h"

#include "adi_adrv9001_gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_ADDR_SIZE 16
#define SPI_DATA_SIZE 8
#define SPI_MASK_SIZE 8
#define HW_RMW_DATA_BYTES 12
#define SPI_MASTER_TOTAL_BYTES_MAX 30

/***************************************************************************//**
 * @brief The `adrv9001_Hal_Err_e` is an enumeration that defines a set of error
 * codes for the Hardware Abstraction Layer (HAL) of the ADRV9001 device.
 * Each enumerator represents a specific type of error or status that can
 * occur during HAL operations, such as SPI failures, GPIO failures,
 * timer failures, and general software errors. This enumeration is used
 * to standardize error reporting and handling within the HAL,
 * facilitating debugging and ensuring consistent error management across
 * different HAL functions.
 *
 * @param ADRV9001HAL_OK HAL function successful, indicating no error was
 * detected.
 * @param ADRV9001HAL_SPI_FAIL Indicates a failure in the HAL SPI operation,
 * suggesting the SPI controller is down.
 * @param ADRV9001HAL_GPIO_FAIL Represents a failure in the HAL GPIO function.
 * @param ADRV9001HAL_TIMER_FAIL Denotes a failure in the HAL Timer function.
 * @param ADRV9001HAL_WAIT_TIMEOUT Indicates a timeout occurred in a HAL
 * function.
 * @param ADRV9001HAL_LOG_FAIL Represents a failure in the logging function of
 * the HAL.
 * @param ADRV9001HAL_LOG_LEVEL_FAIL Indicates a failure related to the logging
 * level in the HAL.
 * @param ADRV9001HAL_HAL_MODE_FAIL Denotes a failure in the HAL mode operation.
 * @param ADRV9001HAL_GEN_SW Indicates a general software failure due to invalid
 * HAL data.
 * @param ADRV9001HAL_WARNING Represents a warning for a non-critical error
 * detected by the HAL.
 * @param ADRV9001HAL_BUFFER_OVERFLOW Indicates a buffer overflow condition in
 * the HAL.
 ******************************************************************************/
typedef enum adrv9001_Hal_Err
{
    ADRV9001HAL_OK = 0,        /*!< HAL function successful. No error Detected */
    ADRV9001HAL_SPI_FAIL,      /*!< HAL SPI operation failure. SPI controller Down */
    ADRV9001HAL_GPIO_FAIL,     /*!< HAL GPIO function Failure */
    ADRV9001HAL_TIMER_FAIL,    /*!< HAL Timer function Failure */
    ADRV9001HAL_WAIT_TIMEOUT,  /*!< HAL function Timeout */
    ADRV9001HAL_LOG_FAIL,
    ADRV9001HAL_LOG_LEVEL_FAIL,
    ADRV9001HAL_HAL_MODE_FAIL,
    ADRV9001HAL_GEN_SW,        /*!< HAL function failed due to general invalid  HAL data*/
    ADRV9001HAL_WARNING,       /*!< HAL function warning that non critical error was detected*/
    ADRV9001HAL_BUFFER_OVERFLOW
} adrv9001_Hal_Err_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_spiMasterSlaveDevices_e` is an enumeration that
 * defines the configuration of SPI slave devices connected to the
 * ADRV9001 device. It currently supports a single SPI slave device
 * configuration, with a reserved option for future expansion to multiple
 * SPI slave devices. This enumeration is used to specify how the SPI
 * master interacts with connected slave devices, ensuring compatibility
 * and future scalability.
 *
 * @param ADI_ADRV9001_SPI_MASTER_SLAVE_SINGLE Represents a single SPI slave
 * device connected to the ADRV9001
 * device.
 * @param ADI_ADRV9001_SPI_MASTER_SLAVE_RESERVED Reserved for future use,
 * indicating more than one SPI
 * slave device connected.
 ******************************************************************************/
typedef enum
{
	ADI_ADRV9001_SPI_MASTER_SLAVE_SINGLE = 0u,		/* Single SPI slave device connected to ADRV9001 device */
	ADI_ADRV9001_SPI_MASTER_SLAVE_RESERVED = 1u     /* More than one SPI slave device connected; Reserved for future */
} adi_adrv9001_spiMasterSlaveDevices_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_spiMasterSource_e` is an enumeration that defines
 * the source of the SPI master chip select (CS) signal for the ADRV9001
 * device. It provides two options: using a digital GPIO or an analog
 * GPIO for the SPI CS signal, allowing for flexibility in hardware
 * configuration depending on the specific requirements of the
 * application.
 *
 * @param ADI_ADRV9001_SPI_MASTER_CS_SOURCE_GPIO_ANALOG Selects digital GPIO for
 * SPI chip select (CS).
 * @param ADI_ADRV9001_SPI_MASTER_CS_SOURCE_GPIO_DIGITAL Selects analog GPIO for
 * SPI chip select (CS).
 ******************************************************************************/
typedef enum
{
	ADI_ADRV9001_SPI_MASTER_CS_SOURCE_GPIO_ANALOG = 0u,		/* Select digital GPIO for SPI CS */
	ADI_ADRV9001_SPI_MASTER_CS_SOURCE_GPIO_DIGITAL = 1u		/* Select analog GPIO for SPI CS */
} adi_adrv9001_spiMasterSource_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_spiMasterTriggerSource_e` is an enumeration that
 * defines the possible sources for triggering SPI transactions in the
 * ADRV9001 device. It includes options for triggering from ARM, monitor
 * logic, reference timer, and GPIO, with some options reserved for
 * future use. This enumeration is used to configure the source of SPI
 * transaction triggers in the device's SPI master configuration.
 *
 * @param ADI_ADRV9001_SPI_MASTER_TRIGGER_SOURCE_ARM Reserved for future use to
 * trigger SPI transaction
 * directly from ARM.
 * @param ADI_ADRV9001_SPI_MASTER_TRIGGER_SOURCE_MONITOR Triggers SPI
 * transaction from
 * monitor logic.
 * @param ADI_ADRV9001_SPI_MASTER_TRIGGER_SOURCE_REF_TIMER Reserved for future
 * use to trigger SPI
 * transaction directly
 * by reference timer.
 * @param ADI_ADRV9001_SPI_MASTER_TRIGGER_SOURCE_GPIO Reserved for future use to
 * trigger SPI transaction
 * from GPIO.
 ******************************************************************************/
typedef enum
{
	ADI_ADRV9001_SPI_MASTER_TRIGGER_SOURCE_ARM = 0u,		/* Reserved for future - Trigger SPI transaction directly from ARM */
	ADI_ADRV9001_SPI_MASTER_TRIGGER_SOURCE_MONITOR = 1u,	/* Trigger SPI transaction from monitor logic */
	ADI_ADRV9001_SPI_MASTER_TRIGGER_SOURCE_REF_TIMER = 2u,	/* Reserved for future - Trigger SPI transaction directly by reference timer */
	ADI_ADRV9001_SPI_MASTER_TRIGGER_SOURCE_GPIO = 3u		/* Reserved for future - Trigger SPI transaction from GPIO */
} adi_adrv9001_spiMasterTriggerSource_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_spiMasterTransferMode_e` is an enumeration that
 * defines the modes of SPI master transfer in terms of how the Chip
 * Select (CS) signal is asserted. It provides two modes: one where the
 * CS is asserted for a specific number of transaction bytes, and another
 * where the CS is asserted for the total number of bytes. This allows
 * for flexibility in handling SPI transactions based on the specific
 * requirements of the data being transferred.
 *
 * @param ADI_ADRV9001_SPI_MASTER_CS_TRANSFER_TRANSACTION_BYTES CS assert for
 * transactionBytes
 * and only sends
 * transactionBytes
 * bytes.
 * @param ADI_ADRV9001_SPI_MASTER_CS_TRANSFER_NUM_BYTES CS assert for numBytes
 * and send all numBytes.
 ******************************************************************************/
typedef enum
{
	ADI_ADRV9001_SPI_MASTER_CS_TRANSFER_TRANSACTION_BYTES = 0u, /* CS assert for transactionBytes and only sends transactionBytes bytes */
	ADI_ADRV9001_SPI_MASTER_CS_TRANSFER_NUM_BYTES = 1u			/* CS assert for numBytes and send all numBytes */
} adi_adrv9001_spiMasterTransferMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_spiMasterConfig_t` structure is used to configure
 * the SPI master settings for the ADRV9001 device. It includes
 * parameters for the number of bytes to write, baud rate division,
 * transaction bytes per CS assertion, and the transfer mode. It also
 * specifies the connected SPI slave devices, the GPIO type for SPI pins,
 * the CS pin, and the trigger source for transactions. Additionally, it
 * includes a wakeup timer for monitor mode and an array for the data to
 * be written to the SPI slave device.
 *
 * @param numBytes Number of bytes to write, with a valid range of 1 to 30.
 * @param baudRateDiv Determines the SPI baud rate as DEV_CLK_IN divided by this
 * value, with a valid range of 0 to 31.
 * @param transactionBytes Number of bytes per CS assertion, with a valid range
 * of 1 to 30.
 * @param transferMode Specifies the CS transfer mode during frame duration.
 * @param spiSlaveDevicesConnected Indicates whether a single or multiple SPI
 * slaves are connected to the ADRV9001.
 * @param csSource Specifies the GPIO type for all SPI pins, either Analog or
 * Digital.
 * @param pin CS pin used, with other SPI signals having dedicated pins.
 * @param triggerSource Defines the trigger source for SPI master transactions.
 * @param wakeupTimer_us Early wakeup timer for SPI master transactions in
 * monitor mode, applicable if triggerSource is set to
 * MONITOR.
 * @param spiData Array of bytes to be written to the SPI slave device, with a
 * maximum size defined by SPI_MASTER_TOTAL_BYTES_MAX.
 ******************************************************************************/
typedef struct
{
	uint8_t                       numBytes;                               /* Number of bytes to write. Valid range is 1 - 30 */
	uint8_t                       baudRateDiv;                            /* SPI baudRate = DEV_CLK_IN / baudRateDiv; Valid range is 0 - 31 */
	uint8_t                       transactionBytes;                       /* Number of bytes per CS assertion.Valid range is 1 - 30 */
	adi_adrv9001_spiMasterTransferMode_e       transferMode;              /* CS transfer during frame duration */
	adi_adrv9001_spiMasterSlaveDevices_e       spiSlaveDevicesConnected;  /* Single or multiple(only in future) SPI slaves connected with ADRV9001 */
	adi_adrv9001_spiMasterSource_e			   csSource;                  /* GPIO type for all SPI pins: Analog or Digital */
	adi_adrv9001_GpioPin_e                     pin;                       /* CS pin - Other SPI signals have dedicated pins: Pin9 = SPI_CLK, 
																										Pin10 = SPI_MISO, Pin11= SPI_MOSI*/
	adi_adrv9001_spiMasterTriggerSource_e      triggerSource;             /* SPI master transaction trigger source */
	uint32_t                      wakeupTimer_us;                         /* Monitor mode early wakeup timer for SPI master transaction. Only available 
                                                                                     if triggerSource = ADI_ADRV9001_SPI_MASTER_TRIGGER_SOURCE_MONITOR */
	uint8_t                      spiData[SPI_MASTER_TOTAL_BYTES_MAX];	  /* Bytes to be written in SPI slave device */
} adi_adrv9001_spiMasterConfig_t;

#ifdef __cplusplus
}
#endif

#endif /* ADRV9001_HAL_TYPES_H_ */
