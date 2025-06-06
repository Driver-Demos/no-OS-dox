/***************************************************************************//**
 *   @file   adgs5412.h
 *   @brief  Header file of ADGS5412 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2016(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef ADGS5412_H_
#define ADGS5412_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define ADGS5412_REG_SW_DATA		0x01
#define ADGS5412_REG_ERR_CONFIG		0x02
#define ADGS5412_REG_ERR_FLAGS		0x03
#define ADGS5412_REG_BURST_EN		0x05
#define ADGS5412_REG_SOFT_RESETB	0x0B

/* ADGS5412_REG_SW_DATA for ADGS5412 */
#define ADGS5412_SW4_EN			(1 << 3)
#define ADGS5412_SW3_EN			(1 << 2)
#define ADGS5412_SW2_EN			(1 << 1)
#define ADGS5412_SW1_EN			(1 << 0)

/* ADGS5412_REG_SW_DATA for ADGS5414 */
#define ADGS5414_SW7_EN			(1 << 7)
#define ADGS5414_SW6_EN			(1 << 6)
#define ADGS5414_SW5_EN			(1 << 5)
#define ADGS5414_SW4_EN			(1 << 4)
#define ADGS5414_SW3_EN			(1 << 3)
#define ADGS5414_SW2_EN			(1 << 2)
#define ADGS5414_SW1_EN			(1 << 1)
#define ADGS5414_SW0_EN			(1 << 0)

/* ADGS5412_REG_ERR_CONFIG */
#define ADGS5412_RW_ERR_EN		(1 << 2)
#define ADGS5412_SCLK_ERR_EN		(1 << 1)
#define ADGS5412_CRC_ERR_EN		(1 << 0)

/* ADGS5412_REG_ERR_FLAGS */
#define ADGS5412_RW_ERR_FLAG		(1 << 2)
#define ADGS5412_SCLK_ERR_FLAG		(1 << 1)
#define ADGS5412_CRC_ERR_FLAG		(1 << 0)
#define ADGS5412_CLR_1			0x6C
#define ADGS5412_CLR_2			0xA9

/* ADGS5412_REG_BURST_EN */
#define ADGS5412_BURST_MODE_EN		(1 << 0)

/* ADGS5412_REG_SOFT_RESETB */
#define ADGS5412_SOFT_RESETB(x)		(((x) & 0xFF) << 0)
#define ADGS5412_RESET_1		0xA3
#define ADGS5412_RESET_2		0x05

#define ADGS5412_DAISY_CHAIN_1		0x25
#define ADGS5412_DAISY_CHAIN_2		0x00

#define ADGS5412_ALIGNMENT		0x25

#define ADGS5412_CRC8_POLY		0x07

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `adgs5412_state` is an enumeration that defines the possible
 * states of the ADGS5412 device, specifically whether it is enabled or
 * disabled. This enumeration is used to manage and control the
 * operational state of the device within the driver, allowing for clear
 * and concise state management.
 *
 * @param ADGS5412_ENABLE Represents the enabled state of the ADGS5412 device.
 * @param ADGS5412_DISABLE Represents the disabled state of the ADGS5412 device.
 ******************************************************************************/
typedef enum {
	ADGS5412_ENABLE,
	ADGS5412_DISABLE,
} adgs5412_state;

/***************************************************************************//**
 * @brief The `adgs5412_dev` structure is a data structure used to represent the
 * state and configuration of an ADGS5412 device. It includes a pointer
 * to an SPI descriptor for communication, and three state variables that
 * control the enabling or disabling of CRC, burst mode, and daisy chain
 * mode. This structure is essential for managing the device's settings
 * and ensuring proper communication and operation.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for communication.
 * @param crc_en An adgs5412_state enum indicating whether CRC is enabled or
 * disabled.
 * @param burst_mode_en An adgs5412_state enum indicating whether burst mode is
 * enabled or disabled.
 * @param daisy_chain_en An adgs5412_state enum indicating whether daisy chain
 * mode is enabled or disabled.
 ******************************************************************************/
typedef struct {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* Device Settings */
	adgs5412_state	crc_en;
	adgs5412_state	burst_mode_en;
	adgs5412_state	daisy_chain_en;
} adgs5412_dev;

/***************************************************************************//**
 * @brief The `adgs5412_init_param` structure is used to initialize the ADGS5412
 * device, encapsulating the necessary parameters for SPI communication
 * and device-specific settings. It includes a `spi_init` member for SPI
 * initialization parameters and three `adgs5412_state` members to enable
 * or disable CRC error checking, burst mode, and daisy chain mode,
 * respectively. This structure is essential for configuring the device
 * before use.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param crc_en Indicates whether CRC error checking is enabled.
 * @param burst_mode_en Indicates whether burst mode is enabled.
 * @param daisy_chain_en Indicates whether daisy chain mode is enabled.
 ******************************************************************************/
typedef struct {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* Device Settings */
	adgs5412_state	crc_en;
	adgs5412_state	burst_mode_en;
	adgs5412_state	daisy_chain_en;
} adgs5412_init_param;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Compute CRC8 checksum. */
/***************************************************************************//**
 * @brief This function calculates the CRC8 checksum for a given array of data
 * bytes, which is useful for error-checking purposes in data
 * transmission or storage. It should be called with a valid pointer to
 * the data array and the correct size of the data. The function
 * processes each byte of the data to compute the checksum, which can
 * then be used to verify data integrity. Ensure that the data pointer is
 * not null and that the data size accurately reflects the number of
 * bytes to be processed.
 *
 * @param data A pointer to the array of data bytes for which the CRC8 checksum
 * is to be computed. Must not be null, and the caller retains
 * ownership of the data.
 * @param data_size The number of bytes in the data array to be processed. Must
 * be a non-negative integer.
 * @return Returns the computed CRC8 checksum as an 8-bit unsigned integer.
 ******************************************************************************/
uint8_t adgs5412_compute_crc8(uint8_t *data,
			      uint8_t data_size);
/* SPI register read from device. */
/***************************************************************************//**
 * @brief Use this function to read a specific register from the ADGS5412 device
 * using SPI communication. It is important to ensure that the device is
 * not in Daisy-Chain mode, as this feature is unavailable in that mode.
 * The function reads the register data and performs error checking,
 * including alignment and CRC validation if CRC is enabled. It is
 * typically called when you need to retrieve configuration or status
 * information from the device.
 *
 * @param dev A pointer to an initialized adgs5412_dev structure representing
 * the device. Must not be null. The device should not be in Daisy-
 * Chain mode.
 * @param reg_addr The address of the register to read. It is an 8-bit value,
 * and only the lower 7 bits are used.
 * @param reg_data A pointer to a uint8_t where the read register data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or -1 if an error occurs (e.g., if the device
 * is in Daisy-Chain mode, alignment error, or CRC error).
 ******************************************************************************/
int32_t adgs5412_spi_reg_read(adgs5412_dev *dev,
			      uint8_t reg_addr,
			      uint8_t *reg_data);
/* SPI register write to device. */
/***************************************************************************//**
 * @brief Use this function to write a value to a specific register on the
 * ADGS5412 device using SPI communication. This function should not be
 * called when the device is in Daisy-Chain mode, as it is not supported
 * in that configuration. Ensure that the device is properly initialized
 * and not in Daisy-Chain mode before calling this function. The function
 * handles CRC if enabled and checks for alignment errors, returning an
 * error code if any issues are detected.
 *
 * @param dev A pointer to an initialized adgs5412_dev structure representing
 * the device. Must not be null and the device must not be in Daisy-
 * Chain mode.
 * @param reg_addr The address of the register to write to. It is a 7-bit value,
 * and the function will mask it appropriately.
 * @param reg_data The data to write to the specified register. It is an 8-bit
 * value.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as when the device is in Daisy-Chain mode or if an
 * alignment error occurs.
 ******************************************************************************/
int32_t adgs5412_spi_reg_write(adgs5412_dev *dev,
			       uint8_t reg_addr,
			       uint8_t reg_data);
/* SPI register read from device using a mask. */
/***************************************************************************//**
 * @brief This function reads a value from a specified register of the ADGS5412
 * device, applies a mask to the read value, and stores the result in the
 * provided data pointer. It is important to note that this function
 * cannot be used when the device is in Daisy-Chain mode, as it will
 * return an error in such cases. The function should be called when the
 * device is properly initialized and not in Daisy-Chain mode. It is
 * useful for reading specific bits from a register by applying a mask.
 *
 * @param dev A pointer to an initialized adgs5412_dev structure representing
 * the device. Must not be null and the device must not be in Daisy-
 * Chain mode.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the ADGS5412 device.
 * @param mask A bitmask to apply to the read register value. Determines which
 * bits of the register value are of interest.
 * @param data A pointer to a uint8_t where the masked register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is in
 * Daisy-Chain mode or if the read operation fails.
 ******************************************************************************/
int32_t adgs5412_spi_reg_read_mask(adgs5412_dev *dev,
				   uint8_t reg_addr,
				   uint8_t mask,
				   uint8_t *data);
/* SPI internal register write to device using a mask. */
/***************************************************************************//**
 * @brief Use this function to modify specific bits of a register on the
 * ADGS5412 device by applying a mask. It is essential to ensure that the
 * device is not in Daisy-Chain mode before calling this function, as it
 * is not supported in that mode. The function reads the current register
 * value, applies the mask to clear specific bits, and then sets the
 * desired bits according to the provided data. This function is useful
 * for updating only certain bits of a register without affecting others.
 *
 * @param dev A pointer to an adgs5412_dev structure representing the device.
 * Must not be null and the device must not be in Daisy-Chain mode.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the ADGS5412 device.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The data to be written to the register, after applying the mask.
 * Only the bits corresponding to the mask will be written.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the device is in Daisy-Chain mode.
 ******************************************************************************/
int32_t adgs5412_spi_reg_write_mask(adgs5412_dev *dev,
				    uint8_t reg_addr,
				    uint8_t mask,
				    uint8_t data);
/* Do a software reset. */
/***************************************************************************//**
 * @brief This function performs a software reset on the ADGS5412 device, which
 * is useful for reinitializing the device to its default state. It must
 * not be called when the device is in Daisy-Chain mode, as the reset
 * feature is unavailable in this mode. The function should be used when
 * a reset of the device is required, such as after a configuration
 * change or to recover from an error state. Ensure that the device is
 * not in Daisy-Chain mode before calling this function to avoid errors.
 *
 * @param dev A pointer to an adgs5412_dev structure representing the device.
 * The structure must be properly initialized and must not be null.
 * The function will return an error if the device is in Daisy-Chain
 * mode.
 * @return Returns 0 on success or a negative error code if the device is in
 * Daisy-Chain mode or if the SPI write operations fail.
 ******************************************************************************/
int32_t adgs5412_do_soft_reset(adgs5412_dev *dev);
/* Clear the Error Flags Register. */
/***************************************************************************//**
 * @brief Use this function to clear the error flags register of an ADGS5412
 * device, which is necessary to reset any error conditions that may have
 * been flagged. This function should not be called when the device is in
 * Daisy-Chain mode, as it is not supported in that configuration. Ensure
 * that the device is properly initialized and not in Daisy-Chain mode
 * before calling this function. The function communicates with the
 * device over SPI and may return an error code if the operation fails.
 *
 * @param dev A pointer to an adgs5412_dev structure representing the device.
 * This must be a valid, initialized device structure. The function
 * will return an error if the device is in Daisy-Chain mode.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error, such as when the device is in Daisy-Chain
 * mode.
 ******************************************************************************/
int32_t adgs5412_clear_err_flags(adgs5412_dev *dev);
/* Enter Daisy-Chain Mode. */
/***************************************************************************//**
 * @brief Use this function to enable Daisy-Chain mode on an ADGS5412 device. It
 * should be called when the device is not already in Daisy-Chain mode,
 * as indicated by the `daisy_chain_en` field of the `adgs5412_dev`
 * structure. If the device is already in Daisy-Chain mode, the function
 * will print an error message and return an error code. This function
 * communicates with the device via SPI to configure it for Daisy-Chain
 * operation.
 *
 * @param dev A pointer to an `adgs5412_dev` structure representing the device.
 * The `daisy_chain_en` field must be set to `ADGS5412_DISABLE`
 * before calling this function. The caller retains ownership of this
 * pointer.
 * @return Returns 0 on success, or a negative error code if the device is
 * already in Daisy-Chain mode or if there is an SPI communication
 * error.
 ******************************************************************************/
int32_t adgs5412_enter_daisy_chain(adgs5412_dev *dev);
/* Send Daisy-Chain commands. */
/***************************************************************************//**
 * @brief This function is used to send a series of commands to the ADGS5412
 * device when it is operating in Daisy-Chain mode. It is essential to
 * ensure that the device is configured in Daisy-Chain mode before
 * calling this function, as it will return an error if the mode is not
 * enabled. This function is typically used in applications where
 * multiple devices are connected in a daisy-chain configuration,
 * allowing for efficient command transmission across the chain.
 *
 * @param dev A pointer to an adgs5412_dev structure representing the device.
 * The device must be initialized and configured to operate in Daisy-
 * Chain mode. Must not be null.
 * @param cmds A pointer to an array of uint8_t containing the commands to be
 * sent. The caller retains ownership of the memory, and it must not
 * be null.
 * @param cmds_size The number of commands in the cmds array. It must be greater
 * than zero.
 * @return Returns 0 on success or a negative error code if the device is not in
 * Daisy-Chain mode or if there is a failure in sending the commands.
 ******************************************************************************/
int32_t adgs5412_send_daisy_chain_cmds(adgs5412_dev *dev,
				       uint8_t *cmds,
				       uint8_t cmds_size);
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the ADGS5412 device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * device. The function sets up the SPI communication and configures
 * device settings such as CRC, burst mode, and daisy chain mode based on
 * the input parameters. If initialization is successful, the device
 * pointer is updated to point to the newly allocated device structure.
 * The function returns an error code if memory allocation fails or if
 * there is an issue with SPI initialization.
 *
 * @param device A pointer to a pointer of type adgs5412_dev. This parameter
 * will be updated to point to the initialized device structure.
 * Must not be null.
 * @param init_param A structure of type adgs5412_init_param containing
 * initialization parameters for the device, including SPI
 * settings and device configuration options such as CRC,
 * burst mode, and daisy chain mode. The values must be valid
 * states defined by the adgs5412_state enumeration.
 * @return Returns an int32_t value indicating success (0) or an error code (<0)
 * if initialization fails.
 ******************************************************************************/
int32_t adgs5412_init(adgs5412_dev **device,
		      adgs5412_init_param init_param);
/* Free the resources allocated by adgs5412_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an ADGS5412
 * device when it is no longer needed. This function should be called to
 * clean up after a device has been initialized and used, ensuring that
 * any allocated memory and associated SPI resources are properly freed.
 * It is important to call this function to prevent memory leaks and to
 * ensure that the SPI descriptor is correctly removed. The function
 * returns an error code if the SPI removal fails, which should be
 * handled by the caller.
 *
 * @param dev A pointer to an adgs5412_dev structure representing the device to
 * be removed. Must not be null. The caller retains ownership of the
 * pointer, but the memory it points to will be freed by this
 * function.
 * @return Returns an int32_t error code from the SPI removal operation. A non-
 * zero value indicates an error occurred during the SPI descriptor
 * removal.
 ******************************************************************************/
int32_t adgs5412_remove(adgs5412_dev *dev);
#endif // ADGS5412_H_
