/***************************************************************************//**
 *   @file   adgs1408.h
 *   @brief  Header file of ADGS1408 Driver.
 *   @author Mircea Caprioru (mircea.caprioru@analog.com)
********************************************************************************
 * Copyright 2018(c) Analog Devices, Inc.
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
#ifndef ADGS1408_H_
#define ADGS1408_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define ADGS1408_REG_SW_DATA		        0x01
#define ADGS1408_REG_ERR_CONFIG		        0x02
#define ADGS1408_REG_ERR_FLAGS		        0x03
#define ADGS1408_REG_BURST_EN		        0x05
#define ADGS1408_REG_ROUND_ROBIN_EN	        0x06
#define ADGS1408_REG_RROBIN_CHANNEL_CONFIG	0x07
#define ADGS1408_REG_CNV_EDGE_SEL	        0x09
#define ADGS1408_REG_SOFT_RESETB	        0x0B

/* ADGS1408_REG_SW_DATA for ADGS1408 */
#define ADGS1408_SW1_EN                 0x01
#define ADGS1408_SW2_EN                 0x03
#define ADGS1408_SW3_EN                 0x05
#define ADGS1408_SW4_EN                 0x07
#define ADGS1408_SW5_EN                 0x09
#define ADGS1408_SW6_EN                 0x0B
#define ADGS1408_SW7_EN                 0x0D
#define ADGS1408_SW8_EN                 0x0F

/* ADGS1408_REG_SW_DATA for ADGS1409 */
#define ADGS1409_SW1_EN			0x01
#define ADGS1409_SW2_EN			0x03
#define ADGS1409_SW3_EN			0x05
#define ADGS1409_SW4_EN			0x07

/* ADGS1408_REG_ERR_CONFIG */
#define ADGS1408_RW_ERR_EN		(1 << 2)
#define ADGS1408_SCLK_ERR_EN		(1 << 1)
#define ADGS1408_CRC_ERR_EN		(1 << 0)

/* ADGS1408_REG_ERR_FLAGS */
#define ADGS1408_RW_ERR_FLAG		(1 << 2)
#define ADGS1408_SCLK_ERR_FLAG		(1 << 1)
#define ADGS1408_CRC_ERR_FLAG		(1 << 0)
#define ADGS1408_CLR_1			0x6C
#define ADGS1408_CLR_2			0xA9

/* ADGS1408_REG_BURST_EN */
#define ADGS1408_BURST_MODE_EN		(1 << 0)

/* ADGS1408_REG_ROUND_ROBIN_EN */
#define ADGS1408_ROUND_ROBIN_EN		(1 << 0)

/* ADGS1408_REG_RROBIN_CHANNEL_CONFIG  for ADGS1408 */
#define ADGS1408_RROBIN_SW1(x)	(((x) & 0x1) << 0)
#define ADGS1408_RROBIN_SW2(x)	(((x) & 0x1) << 1)
#define ADGS1408_RROBIN_SW3(x)	(((x) & 0x1) << 2)
#define ADGS1408_RROBIN_SW4(x)	(((x) & 0x1) << 3)
#define ADGS1408_RROBIN_SW5(x)	(((x) & 0x1) << 4)
#define ADGS1408_RROBIN_SW6(x)	(((x) & 0x1) << 5)
#define ADGS1408_RROBIN_SW7(x)	(((x) & 0x1) << 6)
#define ADGS1408_RROBIN_SW8(x)	(((x) & 0x1) << 7)

/* ADGS1408_REG_RROBIN_CHANNEL_CONFIG  for ADGS1409 */
#define ADGS1408_RROBIN_SW1(x)	(((x) & 0x1) << 0)
#define ADGS1408_RROBIN_SW2(x)	(((x) & 0x1) << 1)
#define ADGS1408_RROBIN_SW3(x)	(((x) & 0x1) << 2)
#define ADGS1408_RROBIN_SW4(x)	(((x) & 0x1) << 3)

/* ADGS1408_REG_CNV_EDGE_SEL */
#define ADGS1408_CNV_EDGE_RISING        (1 << 0)

/* ADGS1408_REG_SOFT_RESETB */
#define ADGS1408_SOFT_RESETB(x)		(((x) & 0xFF) << 0)
#define ADGS1408_RESET_1		0xA3
#define ADGS1408_RESET_2		0x05

#define ADGS1408_DAISY_CHAIN_1		0x25
#define ADGS1408_DAISY_CHAIN_2		0x00

/* ADGS1408 exit Round Robin*/
#define ADGS1408_RROBIN_EXIT_1		0xA3
#define ADGS1408_RROBIN_EXIT_2		0x18
#define ADGS1408_RROBIN_EXIT_3		0xE3
#define ADGS1408_RROBIN_EXIT_4		0xB4

#define ADGS1408_ALIGNMENT		0x25

#define ADGS1408_CRC8_POLY		0x07

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `adgs1408_type` enumeration defines identifiers for different
 * device types, specifically the ADGS1408 and ADGS1409, which are likely
 * different models or configurations of a similar device family. This
 * enumeration is used to distinguish between these device types within
 * the driver code, allowing for device-specific operations and
 * configurations.
 *
 * @param ID_ADGS1408 Represents the identifier for the ADGS1408 device type.
 * @param ID_ADGS1409 Represents the identifier for the ADGS1409 device type.
 ******************************************************************************/
enum adgs1408_type {
	ID_ADGS1408,
	ID_ADGS1409,
};

/***************************************************************************//**
 * @brief The `adgs1408_state` enumeration defines two possible states for the
 * ADGS1408 device: enabled and disabled. This enumeration is used to
 * manage and represent the operational state of the device, allowing for
 * easy toggling between active and inactive modes.
 *
 * @param ADGS1408_ENABLE Represents the enabled state of the ADGS1408 device.
 * @param ADGS1408_DISABLE Represents the disabled state of the ADGS1408 device.
 ******************************************************************************/
enum adgs1408_state {
	ADGS1408_ENABLE,
	ADGS1408_DISABLE,
};

/***************************************************************************//**
 * @brief The `adgs1408_rrobin_config` structure is used to configure the round-
 * robin mode of the ADGS1408 device, with each boolean member
 * representing the enable state of one of the eight switches in the
 * round-robin sequence. This configuration allows for the dynamic
 * selection of switches in a predefined order, facilitating automated
 * switching operations in the device.
 *
 * @param rrobin_sw1 Indicates the state of the first switch in the round-robin
 * configuration.
 * @param rrobin_sw2 Indicates the state of the second switch in the round-robin
 * configuration.
 * @param rrobin_sw3 Indicates the state of the third switch in the round-robin
 * configuration.
 * @param rrobin_sw4 Indicates the state of the fourth switch in the round-robin
 * configuration.
 * @param rrobin_sw5 Indicates the state of the fifth switch in the round-robin
 * configuration.
 * @param rrobin_sw6 Indicates the state of the sixth switch in the round-robin
 * configuration.
 * @param rrobin_sw7 Indicates the state of the seventh switch in the round-
 * robin configuration.
 * @param rrobin_sw8 Indicates the state of the eighth switch in the round-robin
 * configuration.
 ******************************************************************************/
struct adgs1408_rrobin_config {
	bool rrobin_sw1;
	bool rrobin_sw2;
	bool rrobin_sw3;
	bool rrobin_sw4;
	bool rrobin_sw5;
	bool rrobin_sw6;
	bool rrobin_sw7;
	bool rrobin_sw8;
};

/***************************************************************************//**
 * @brief The `adgs1408_dev` structure is used to represent the configuration
 * and state of an ADGS1408 device, which is a type of analog switch. It
 * includes a SPI descriptor for communication, various state flags to
 * enable or disable features like CRC, burst mode, daisy chain, and
 * round robin mode, as well as a configuration structure for round robin
 * switch settings. The structure also specifies the active device type,
 * allowing for flexible control and communication with the ADGS1408
 * device.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param crc_en Indicates if CRC is enabled or disabled.
 * @param burst_mode_en Indicates if burst mode is enabled or disabled.
 * @param daisy_chain_en Indicates if daisy chain mode is enabled or disabled.
 * @param round_robin_en Indicates if round robin mode is enabled or disabled.
 * @param rrobin_sw_config Configuration for round robin switch settings.
 * @param act_device Specifies the active device type.
 ******************************************************************************/
struct adgs1408_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* Device Settings */
	enum adgs1408_state	crc_en;
	enum adgs1408_state	burst_mode_en;
	enum adgs1408_state	daisy_chain_en;
	enum adgs1408_state	round_robin_en;
	struct adgs1408_rrobin_config rrobin_sw_config;
	enum adgs1408_type act_device;
};

/***************************************************************************//**
 * @brief The `adgs1408_init_param` structure is used to initialize the ADGS1408
 * device with specific settings. It includes parameters for SPI
 * communication, as well as various device settings such as CRC, burst
 * mode, daisy chain, and round robin configurations. This structure
 * allows for the configuration of the device's operational modes and
 * active device type, facilitating the setup process for the ADGS1408
 * driver.
 *
 * @param spi_init Holds the SPI initialization parameters.
 * @param crc_en Indicates whether CRC is enabled or disabled.
 * @param burst_mode_en Indicates whether burst mode is enabled or disabled.
 * @param daisy_chain_en Indicates whether daisy chain mode is enabled or
 * disabled.
 * @param round_robin_en Indicates whether round robin mode is enabled or
 * disabled.
 * @param rrobin_sw_config Contains the round robin switch configuration.
 * @param act_device Specifies the active device type.
 ******************************************************************************/
struct adgs1408_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* Device Settings */
	enum adgs1408_state	crc_en;
	enum adgs1408_state	burst_mode_en;
	enum adgs1408_state	daisy_chain_en;
	enum adgs1408_state	round_robin_en;
	struct adgs1408_rrobin_config rrobin_sw_config;
	enum adgs1408_type act_device;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Compute CRC8 checksum. */
/***************************************************************************//**
 * @brief This function calculates the CRC8 checksum for a specified buffer of
 * data, which is useful for error-checking in data transmission or
 * storage. It should be called with a valid data buffer and the correct
 * size of the data to ensure accurate computation. The function does not
 * modify the input data and returns the computed CRC8 value. It is
 * important to ensure that the data pointer is not null and that the
 * data size is correctly specified to avoid undefined behavior.
 *
 * @param data A pointer to the data buffer for which the CRC8 checksum is to be
 * computed. Must not be null, and the caller retains ownership of
 * the data.
 * @param data_size The size of the data buffer in bytes. Must accurately
 * reflect the number of bytes in the data buffer to ensure
 * correct CRC8 computation.
 * @return Returns the computed CRC8 checksum as an 8-bit unsigned integer.
 ******************************************************************************/
uint8_t adgs1408_compute_crc8(uint8_t *data,
			      uint8_t data_size);
/* SPI register read from device. */
/***************************************************************************//**
 * @brief This function reads a specified register from the ADGS1408 device
 * using SPI communication. It should be called when you need to retrieve
 * the current value of a register from the device. The function requires
 * that the device is not in Daisy-Chain mode, as this feature is not
 * supported in that mode. Ensure that the device has been properly
 * initialized and configured before calling this function. The function
 * handles CRC checks if enabled, and it will return an error if the CRC
 * check fails or if there is an alignment error.
 *
 * @param dev A pointer to an initialized adgs1408_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to read. Must be a valid register
 * address for the ADGS1408 device.
 * @param reg_data A pointer to a uint8_t where the read register data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or -1 on failure due to Daisy-Chain mode being
 * enabled, alignment error, or CRC error.
 ******************************************************************************/
int32_t adgs1408_spi_reg_read(struct adgs1408_dev *dev,
			      uint8_t reg_addr,
			      uint8_t *reg_data);
/* SPI register write to device. */
/***************************************************************************//**
 * @brief This function writes a given data value to a specified register
 * address on the ADGS1408 device using SPI communication. It must be
 * called with a valid device structure that has been properly
 * initialized. The function is not available in Daisy-Chain mode, and it
 * will return an error if attempted. Additionally, if CRC is enabled, a
 * CRC8 checksum is computed and included in the transmission. The
 * function checks for alignment errors after the write operation and
 * returns an error if any are detected. It is important to ensure that
 * the device is not in Daisy-Chain mode before calling this function.
 *
 * @param dev A pointer to an initialized adgs1408_dev structure representing
 * the device. Must not be null. The function will return an error if
 * the device is in Daisy-Chain mode.
 * @param reg_addr The address of the register to write to. It is a 7-bit value,
 * and the function will mask it appropriately.
 * @param reg_data The data to be written to the specified register. It is an
 * 8-bit value.
 * @return Returns 0 on success, or -1 if an error occurs, such as when the
 * device is in Daisy-Chain mode or if an alignment error is detected.
 ******************************************************************************/
int32_t adgs1408_spi_reg_write(struct adgs1408_dev *dev,
			       uint8_t reg_addr,
			       uint8_t reg_data);
/* SPI register read from device using a mask. */
int32_t adgs1408_spi_reg_read_mask(struct adgs1408_dev *dev,
				   uint8_t reg_addr,
				   uint8_t mask,
				   uint8_t *data);
/* SPI internal register write to device using a mask. */
/***************************************************************************//**
 * @brief This function allows writing specific bits to a register of the
 * ADGS1408 device by applying a mask. It is useful when only certain
 * bits of a register need to be modified without affecting the others.
 * The function must not be used when the device is in Daisy-Chain mode,
 * as it will return an error in such cases. It reads the current
 * register value, applies the mask to clear the bits, and then sets the
 * new data before writing it back. This function should be called only
 * when the device is properly initialized and not in Daisy-Chain mode.
 *
 * @param dev A pointer to an initialized adgs1408_dev structure representing
 * the device. Must not be null. The device must not be in Daisy-
 * Chain mode.
 * @param reg_addr The address of the register to be written to. Must be a valid
 * register address for the ADGS1408 device.
 * @param mask A bitmask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The data to be written to the register, masked by the provided
 * mask. Only the bits corresponding to the mask will be written.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as when the device is in Daisy-Chain mode.
 ******************************************************************************/
int32_t adgs1408_spi_reg_write_mask(struct adgs1408_dev *dev,
				    uint8_t reg_addr,
				    uint8_t mask,
				    uint8_t data);
/* Do a software reset. */
/***************************************************************************//**
 * @brief This function performs a software reset on the ADGS1408 device, which
 * is useful for reinitializing the device to its default state. It
 * should be called when the device is not in Daisy-Chain mode, as the
 * reset feature is unavailable in that mode. The function writes
 * specific reset commands to the device's registers to achieve the
 * reset. It is important to ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized adgs1408_dev structure representing
 * the device. The device must not be in Daisy-Chain mode, as
 * indicated by the daisy_chain_en field being set to
 * ADGS1408_DISABLE. The caller retains ownership of the structure.
 * @return Returns 0 on success, or a negative error code if the device is in
 * Daisy-Chain mode or if the SPI register writes fail.
 ******************************************************************************/
int32_t adgs1408_do_soft_reset(struct adgs1408_dev *dev);
/* Clear the Error Flags Register. */
/***************************************************************************//**
 * @brief Use this function to clear the error flags register of an ADGS1408
 * device, which is necessary to reset any error states that may have
 * been set. This function should not be called when the device is in
 * Daisy-Chain mode, as it is not supported in that configuration. Ensure
 * that the device is properly initialized and not in Daisy-Chain mode
 * before calling this function. The function communicates with the
 * device over SPI and may include a CRC check if enabled.
 *
 * @param dev A pointer to an initialized adgs1408_dev structure representing
 * the device. The structure must not be null, and the device must
 * not be in Daisy-Chain mode. The caller retains ownership of the
 * structure.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the device is in Daisy-Chain mode.
 ******************************************************************************/
int32_t adgs1408_clear_err_flags(struct adgs1408_dev *dev);
/* Enter Daisy-Chain Mode. */
/***************************************************************************//**
 * @brief This function is used to enable Daisy-Chain mode on an ADGS1408
 * device. It should be called when the device is not already in Daisy-
 * Chain mode, as indicated by the `daisy_chain_en` field of the
 * `adgs1408_dev` structure. If the device is already in Daisy-Chain
 * mode, the function will print an error message and return an error
 * code. This function requires a valid device structure that has been
 * properly initialized and configured. It communicates with the device
 * via SPI to set the necessary registers for entering Daisy-Chain mode.
 *
 * @param dev A pointer to an `adgs1408_dev` structure representing the device.
 * This must not be null and should be properly initialized. The
 * function checks the `daisy_chain_en` field to determine if the
 * device is already in Daisy-Chain mode.
 * @return Returns 0 on success, indicating that the device has successfully
 * entered Daisy-Chain mode. Returns -1 if the device is already in
 * Daisy-Chain mode, or if there is an error in SPI communication.
 ******************************************************************************/
int32_t adgs1408_enter_daisy_chain(struct adgs1408_dev *dev);
/* Send Daisy-Chain commands. */
/***************************************************************************//**
 * @brief This function is used to send a series of commands to the ADGS1408
 * device when it is operating in Daisy-Chain mode. It is essential to
 * ensure that the device is configured in Daisy-Chain mode before
 * calling this function, as it will return an error if the mode is not
 * enabled. The function communicates with the device using SPI, sending
 * the specified commands. It is typically used in applications where
 * multiple devices are connected in a daisy-chain configuration,
 * allowing for synchronized control.
 *
 * @param dev A pointer to an initialized adgs1408_dev structure representing
 * the device. The device must be in Daisy-Chain mode; otherwise, the
 * function will return an error.
 * @param cmds A pointer to an array of uint8_t values representing the commands
 * to be sent. The caller retains ownership of the memory, and it
 * must not be null.
 * @param cmds_size The number of commands in the cmds array. It must accurately
 * reflect the size of the cmds array to ensure correct
 * operation.
 * @return Returns 0 on success, or -1 if the device is not in Daisy-Chain mode.
 * On success, the commands are sent to the device via SPI.
 ******************************************************************************/
int32_t adgs1408_send_daisy_chain_cmds(struct adgs1408_dev *dev,
				       uint8_t *cmds,
				       uint8_t cmds_size);
/* Enter round robin mode. */
/***************************************************************************//**
 * @brief This function enables the round robin mode on the ADGS1408 device,
 * allowing it to cycle through its channels automatically. It should be
 * called when the device is not already in round robin mode, as
 * indicated by the `round_robin_en` field in the device structure. If
 * the device is already in round robin mode, the function will print a
 * message and return an error code. This function requires a valid
 * device structure with an initialized SPI descriptor.
 *
 * @param dev A pointer to an `adgs1408_dev` structure representing the device.
 * This structure must be properly initialized and must not be null.
 * The function checks the `round_robin_en` field to determine if the
 * device is already in round robin mode.
 * @return Returns 0 on success, or -1 if the device is already in round robin
 * mode. On success, the SPI write operation is performed to enable
 * round robin mode.
 ******************************************************************************/
int32_t adgs1408_enter_round_robin(struct adgs1408_dev *dev);
/* Configure Round Robin Mode. */
/***************************************************************************//**
 * @brief This function sets up the round robin mode for an ADGS1408 device by
 * configuring the channel selection and conversion edge polarity. It
 * should be called when the device is in a state ready to accept
 * configuration changes, typically after initialization. The function
 * requires a valid device structure and a conversion polarity setting.
 * It communicates with the device over SPI to apply the configuration.
 * The function returns an error code if the SPI communication fails.
 *
 * @param dev A pointer to an adgs1408_dev structure representing the device to
 * be configured. Must not be null and should be properly initialized
 * before calling this function.
 * @param cnv_polarity A uint8_t value representing the conversion edge
 * polarity. Valid values depend on the device's expected
 * configuration for conversion polarity.
 * @return Returns an int32_t value indicating success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error occurred during SPI communication.
 ******************************************************************************/
int32_t adgs1408_configure_round_robin(struct adgs1408_dev *dev,
				       uint8_t cnv_polarity);
/* Exit Round Robin Mode. */
/***************************************************************************//**
 * @brief This function is used to exit the round robin mode on an ADGS1408
 * device. It should be called when the device is currently in round
 * robin mode and you wish to disable this mode. The function
 * communicates with the device over SPI to send the necessary commands
 * to exit round robin mode. It is important to ensure that the device is
 * properly initialized and in a state where it can accept these commands
 * before calling this function. The function returns an error code if
 * the operation fails, which can be used for error handling.
 *
 * @param dev A pointer to an initialized adgs1408_dev structure representing
 * the device. This must not be null, and the structure should be
 * properly initialized with a valid SPI descriptor. The function
 * will use this descriptor to communicate with the device.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error occurred during the SPI communication.
 ******************************************************************************/
int32_t adgs1408_exit_round_robin(struct adgs1408_dev *dev);
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up the ADGS1408 device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * device. The function handles SPI initialization, performs a soft
 * reset, and configures optional features such as CRC, burst mode, daisy
 * chain, and round robin mode based on the input parameters. It returns
 * an error code if initialization fails, and the caller is responsible
 * for managing the allocated device structure.
 *
 * @param device A pointer to a pointer of type `struct adgs1408_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and is responsible for freeing the
 * allocated memory using `adgs1408_remove`.
 * @param init_param A structure of type `struct adgs1408_init_param` containing
 * initialization parameters such as SPI settings and feature
 * enable flags. The structure must be properly populated
 * before calling the function.
 * @return Returns an `int32_t` indicating success (0) or an error code if
 * initialization fails. On success, the `device` pointer is set to
 * point to the initialized device structure.
 ******************************************************************************/
int32_t adgs1408_init(struct adgs1408_dev **device,
		      struct adgs1408_init_param init_param);
/* Free the resources allocated by adgs1408_init(). */
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * ADGS1408 device when it is no longer needed. This function should be
 * called to clean up after a device has been initialized and used,
 * ensuring that any allocated memory and SPI resources are freed. It is
 * important to call this function to prevent memory leaks and to ensure
 * that the SPI descriptor is properly removed. The function returns an
 * error code if the SPI removal fails.
 *
 * @param dev A pointer to an adgs1408_dev structure representing the device to
 * be removed. Must not be null. The caller retains ownership of the
 * pointer, but the memory it points to will be freed by this
 * function.
 * @return Returns an int32_t error code from the SPI removal process. A non-
 * zero value indicates an error occurred during the SPI descriptor
 * removal.
 ******************************************************************************/
int32_t adgs1408_remove(struct adgs1408_dev *dev);
#endif // ADGS1408_H_
