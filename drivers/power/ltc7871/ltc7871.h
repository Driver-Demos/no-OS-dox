/***************************************************************************//**
 *   @file   LTC7871.h
 *   @brief  Header file for the LTC7871 Driver
 *   @author Aldrin Abacan (aldrin.abacan@analog.com)
 *******************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
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
 ******************************************************************************/

#ifndef __LTC7871_H__
#define __LTC7871_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_units.h"
#include "no_os_util.h"

#define SET 1
#define CLEAR 0

#define LTC7871_CRC_POLYNOMIAL          0x07

#define LTC7871_FRAME_SIZE       		3
#define LTC7871_NUM_REGISTERS           8

/* Miscellaneous Definitions*/
#define LTC7871_SPI_READ			0x01
#define LTC7871_SPI_WRITE			0x00

// LTC7871 SPI Command
#define LTC7871_STATUS_ACQUISITION_COMMAND    0xF0
#define LTC7871_DATA_WRITE_COMMAND            0xF2
#define LTC7871_DATA_READ_COMMAND             0xF4

// LTC7871 SPI Command Size in Bytes
#define LTC7871_STATUS_ACQUISITION_COMMAND_SIZE   4
#define LTC7871_DATA_WRITE_COMMAND_SIZE           8
#define LTC7871_DATA_READ_COMMAND_SIZE            8

// LTC7871 SPI Register Address
#define LTC7871_MFR_FAULT                   0x01
#define LTC7871_MFR_OC_FAULT                0x02
#define LTC7871_MFR_NOC_FAULT               0x03
#define LTC7871_MFR_STATUS                  0x04
#define LTC7871_MFR_CONFIG1                 0x05
#define LTC7871_MFR_CONFIG2                 0x06
#define LTC7871_CHIP_CTRL                   0x07
#define LTC7871_IDAC_VLOW                   0x08
#define LTC7871_IDAC_VHIGH                  0x09
#define LTC7871_IDAC_SETCUR                 0x0A
#define LTC7871_MFR_SSFM                    0x0B

// LTC7871 SPI Register Mask
#define LTC7871_MFR_FAULT_MASK         		  NO_OS_GENMASK(6, 0)
#define LTC7871_MFR_OC_FAULT_MASK             NO_OS_GENMASK(6, 0)
#define LTC7871_MFR_NOC_FAULT_MASK            NO_OS_GENMASK(6, 0)
#define LTC7871_MFR_STATUS_MASK               NO_OS_GENMASK(2, 0)
#define LTC7871_MFR_CONFIG1_MASK              NO_OS_GENMASK(5, 0)
#define LTC7871_MFR_CONFIG2_MASK              NO_OS_GENMASK(4, 0)
#define LTC7871_MFR_CHIP_CTRL_MASK            NO_OS_GENMASK(2, 0)
#define LTC7871_MFR_SSFM_MASK           	  NO_OS_GENMASK(8, 0)


// LTC7871 MFR_FAULT register bits
#define LTC7871_VLOW_OV_MASK                  NO_OS_BIT(6)
#define LTC7871_VHIGH_OV_MASK                 NO_OS_BIT(5)
#define LTC7871_VHIGH_UV_MASK                 NO_OS_BIT(4)
#define LTC7871_DRVCC_UV_MASK                 NO_OS_BIT(3)
#define LTC7871_V5_UV_MASK                    NO_OS_BIT(2)
#define LTC7871_VREF_BAD_MASK                 NO_OS_BIT(1)
#define LTC7871_OVER_TEMP_MASK                NO_OS_BIT(0)

// LTC7871 MFR_OC_FAULT register bits
#define LTC7871_OC_FAULT_6_MASK               NO_OS_BIT(5)
#define LTC7871_OC_FAULT_5_MASK               NO_OS_BIT(4)
#define LTC7871_OC_FAULT_4_MASK               NO_OS_BIT(3)
#define LTC7871_OC_FAULT_3_MASK               NO_OS_BIT(2)
#define LTC7871_OC_FAULT_2_MASK               NO_OS_BIT(1)
#define LTC7871_OC_FAULT_1_MASK               NO_OS_BIT(0)

// LTC7871 MFR_NOC_FAULT register bits
#define LTC7871_NOC_FAULT_6_MASK               NO_OS_BIT(5)
#define LTC7871_NOC_FAULT_5_MASK               NO_OS_BIT(4)
#define LTC7871_NOC_FAULT_4_MASK               NO_OS_BIT(3)
#define LTC7871_NOC_FAULT_3_MASK               NO_OS_BIT(2)
#define LTC7871_NOC_FAULT_2_MASK               NO_OS_BIT(1)
#define LTC7871_NOC_FAULT_1_MASK               NO_OS_BIT(0)

// LTC7871 MFR_STATUS register bits
#define LTC7871_SS_DONE_MASK                   NO_OS_BIT(2)
#define LTC7871_MAX_CURRENT_MASK               NO_OS_BIT(1)
#define LTC7871_PGOOD_MASK                     NO_OS_BIT(0)

// LTC7871 MFR_CONFIG1 register bits
#define LTC7871_SERCUR_WARNING_MASK            NO_OS_BIT(5)
#define LTC7871_DRVCC_SET_MASK                 NO_OS_GENMASK(4, 3)
#define LTC7871_ILIM_SET_MASK                  NO_OS_GENMASK(2, 0)

// LTC7871 MFR_CONFIG2 register bits
#define LTC7871_BURST_MASK                     NO_OS_BIT(4)
#define LTC7871_DCM_MASK                       NO_OS_BIT(3)
#define LTC7871_HIZ_MASK                       NO_OS_BIT(2)
#define LTC7871_SPRD_MASK                      NO_OS_BIT(1)
#define LTC7871_BUCK_BOOST_MASK                NO_OS_BIT(0)

// LTC7871 MFR_CHIP_CTRL register bits
#define LTC7871_CML_MASK                       NO_OS_BIT(2)
#define LTC7871_RESET_MASK                     NO_OS_BIT(1)
#define LTC7871_WP_MASK                        NO_OS_BIT(0)

// LTC7871 MFR_IDAC_VLOW register bits
#define LTC7871_MFR_IDAC_VLOW_MASK             NO_OS_GENMASK(6, 0)

// LTC7871 MFR_IDAC_VHIGH register bits
#define LTC7871_MFR_IDAC_VHIGH_MASK            NO_OS_GENMASK(6, 0)

// LTC7871 MFR_IDAC_SETCUR register bits
#define LTC7871_MFR_IDAC_SETCUR_MASK           NO_OS_GENMASK(4, 0)

// LTC7871 MFR_SSFM register bits
#define LTC7871_MFR_SSFM_FSR_MASK              NO_OS_GENMASK(4, 3)
#define LTC7871_MFR_SSFM_MSF_MASK              NO_OS_GENMASK(2, 0)


/***************************************************************************//**
 * @brief The `ltc7871_ssfm_fsr` enumeration defines the frequency spread range
 * control bits for the LTC7871 device, allowing selection of different
 * spread ranges for frequency modulation. This is used to control the
 * spread spectrum frequency modulation feature of the device, which can
 * help reduce electromagnetic interference by spreading the energy of
 * the signal over a wider frequency band.
 *
 * @param LTC7871_SSFM_FSR_12 Represents a frequency spread range of +/-12%.
 * @param LTC7871_SSFM_FSR_15 Represents a frequency spread range of +/-15%.
 * @param LTC7871_SSFM_FSR_10 Represents a frequency spread range of +/-10%.
 * @param LTC7871_SSFM_FSR_8 Represents a frequency spread range of +/-8%.
 ******************************************************************************/
enum ltc7871_ssfm_fsr {
	/** +/-12%*/
	LTC7871_SSFM_FSR_12,
	/** +/-15%*/
	LTC7871_SSFM_FSR_15,
	/** +/-10%*/
	LTC7871_SSFM_FSR_10,
	/** +/-8%*/
	LTC7871_SSFM_FSR_8
};

/***************************************************************************//**
 * @brief The `ltc7871_ssfm_msf` enumeration defines a set of constants used to
 * control the modulation signal frequency for the LTC7871 device. Each
 * constant represents a specific division of the controller's switching
 * frequency, allowing for precise control over the modulation signal
 * frequency. This is crucial for applications requiring specific
 * frequency modulation settings to optimize performance or meet design
 * specifications.
 *
 * @param LTC7871_SSFM_MSF_512 Represents the modulation signal frequency as the
 * controller switching frequency divided by 512.
 * @param LTC7871_SSFM_MSF_1024 Represents the modulation signal frequency as
 * the controller switching frequency divided by
 * 1024.
 * @param LTC7871_SSFM_MSF_2048 Represents the modulation signal frequency as
 * the controller switching frequency divided by
 * 2048.
 * @param LTC7871_SSFM_MSF_4096 Represents the modulation signal frequency as
 * the controller switching frequency divided by
 * 4096.
 * @param LTC7871_SSFM_MSF_256 Represents the modulation signal frequency as the
 * controller switching frequency divided by 256.
 * @param LTC7871_SSFM_MSF_128 Represents the modulation signal frequency as the
 * controller switching frequency divided by 128.
 * @param LTC7871_SSFM_MSF_64 Represents the modulation signal frequency as the
 * controller switching frequency divided by 64.
 * @param LTC7871_SSFM_MSF_512U Represents the modulation signal frequency as
 * the controller switching frequency divided by
 * 512, similar to LTC7871_SSFM_MSF_512.
 ******************************************************************************/
enum ltc7871_ssfm_msf {
	/** Controller Switching Frequency/512*/
	LTC7871_SSFM_MSF_512,
	/** Controller Switching Frequency/1024*/
	LTC7871_SSFM_MSF_1024,
	/** Controller Switching Frequency/2048*/
	LTC7871_SSFM_MSF_2048,
	/** Controller Switching Frequency/4096*/
	LTC7871_SSFM_MSF_4096,
	/** Controller Switching Frequency/256*/
	LTC7871_SSFM_MSF_256,
	/** Controller Switching Frequency/128*/
	LTC7871_SSFM_MSF_128,
	/** Controller Switching Frequency/64*/
	LTC7871_SSFM_MSF_64,
	/** Controller Switching Frequency/512*/
	LTC7871_SSFM_MSF_512U
};

/***************************************************************************//**
 * @brief The `ltc7871_ctrl_wp` enumeration defines two states for controlling
 * the write protection feature of the LTC7871 device. It provides
 * symbolic names for enabling and disabling write protection, which can
 * be used to manage access to certain registers or settings within the
 * device, ensuring that critical configurations are protected from
 * unintended modifications.
 *
 * @param LTC7871_CTRL_WP_ENABLE Represents the state to enable write
 * protection.
 * @param LTC7871_CTRL_WP_DISABLE Represents the state to disable write
 * protection.
 ******************************************************************************/
enum ltc7871_ctrl_wp {
	/** enable write protect*/
	LTC7871_CTRL_WP_ENABLE,
	/** disable write protect*/
	LTC7871_CTRL_WP_DISABLE
};

/***************************************************************************//**
 * @brief The `ltc7871_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the LTC7871 device.
 * It includes pointers to SPI and GPIO initialization parameter
 * structures, which are essential for configuring the communication
 * interface and control signals of the device. This structure is
 * typically used during the initialization process to ensure that the
 * device is properly configured for operation.
 *
 * @param spi Pointer to a structure containing SPI initialization parameters.
 * @param gpio_pwmen Pointer to a structure containing GPIO initialization
 * parameters for the PWM enable pin.
 ******************************************************************************/
struct ltc7871_init_param {
	struct no_os_spi_init_param *spi;
	struct no_os_gpio_init_param *gpio_pwmen;
};

/***************************************************************************//**
 * @brief The `ltc7871_dev` structure serves as a device descriptor for the
 * LTC7871, a power management IC. It contains pointers to SPI and GPIO
 * descriptors, which are used to facilitate communication and control of
 * the device's PWM enable functionality. This structure is essential for
 * interfacing with the LTC7871, allowing for configuration and operation
 * through SPI communication and GPIO control.
 *
 * @param spi Pointer to a SPI descriptor for communication.
 * @param gpio_pwmen Pointer to a GPIO descriptor for the PWM enable pin.
 ******************************************************************************/
struct ltc7871_dev {
	struct no_os_spi_desc *spi;
	struct no_os_gpio_desc *gpio_pwmen;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief Use this function to compute the PEC byte for a sequence of data
 * bytes, which is typically used for error checking in communication
 * protocols. The function processes each byte of the input data array
 * and applies a specific algorithm to generate the PEC byte. This
 * function should be called when you need to append a PEC byte to a data
 * packet for transmission or verify the integrity of received data.
 * Ensure that the data pointer is valid and the length accurately
 * represents the number of bytes to process.
 *
 * @param data A pointer to an array of bytes for which the PEC is to be
 * calculated. Must not be null, and the caller retains ownership of
 * the data.
 * @param len The number of bytes in the data array to process. Must be non-
 * negative and should not exceed the actual size of the data array.
 * @return Returns the calculated PEC byte as an unsigned 8-bit integer.
 ******************************************************************************/
uint8_t ltc7871_get_pec_byte(uint8_t *data, uint8_t len);

/***************************************************************************//**
 * @brief Use this function to read a specific register from the LTC7871 device.
 * It requires a valid device descriptor and a register address to read
 * from. The function will store the read value in the provided data
 * pointer. Ensure that the device has been properly initialized before
 * calling this function. The function performs error checking on the
 * device descriptor and data pointer, returning an error code if either
 * is invalid. It also verifies the integrity of the read data using a
 * PEC (Packet Error Code) check, returning an error if the check fails.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param reg The register address to read from. Must be a valid register
 * address for the LTC7871 device.
 * @param data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null. The caller retains ownership.
 * @return Returns 0 on success, a negative error code on failure, such as
 * -EINVAL for invalid parameters or -EBADMSG for a PEC check failure.
 ******************************************************************************/
int ltc7871_reg_read(struct ltc7871_dev *dev, uint8_t reg, uint8_t *data);

/***************************************************************************//**
 * @brief Use this function to write a byte of data to a specific register on
 * the LTC7871 device. It is essential to ensure that the device
 * descriptor is properly initialized and not null before calling this
 * function. This function is typically used when configuring or updating
 * the settings of the LTC7871 device. The function returns an error code
 * if the device descriptor is null, indicating invalid input. Successful
 * execution results in the data being written to the device via SPI
 * communication.
 *
 * @param dev A pointer to an initialized ltc7871_dev structure representing the
 * device. Must not be null. The function returns an error if this
 * parameter is null.
 * @param reg The register address to which the data will be written. It should
 * be a valid register address defined for the LTC7871 device.
 * @param data The byte of data to be written to the specified register. It
 * should be a valid data byte for the register being accessed.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if the device descriptor is null.
 ******************************************************************************/
int ltc7871_reg_write(struct ltc7871_dev *dev, uint8_t reg, uint8_t data);

/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * LTC7871 device by applying a mask and writing the new data. It is
 * useful when only certain bits of a register need to be updated without
 * affecting the other bits. The function first reads the current value
 * of the register, applies the mask to clear the bits to be modified,
 * and then sets the new data using the provided mask. It must be called
 * with a valid device descriptor, and the address must correspond to a
 * valid register in the LTC7871. The function returns an error code if
 * the device descriptor is null or if the read or write operations fail.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param address The register address to be written to. Must be a valid
 * register address for the LTC7871.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Only bits set in the mask will be affected.
 * @param data The new data to be written to the register, masked by the
 * provided mask. Only the bits corresponding to the mask will be
 * written.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if the device is null or other error codes from read/write
 * operations.
 ******************************************************************************/
int ltc7871_reg_write_mask(struct ltc7871_dev *dev, uint8_t address,
			   uint8_t mask, uint8_t data);

/***************************************************************************//**
 * @brief This function is used to check the most critical fault status of the
 * LTC7871 device by reading the manufacturer fault register. It should
 * be called when you need to determine if a specific fault condition,
 * indicated by the status parameter, is present. The function requires a
 * valid device descriptor and a non-null pointer to store the result. It
 * returns an error code if the device descriptor or the result pointer
 * is null, or if the register read operation fails.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param status A uint8_t value representing the specific fault status to
 * check. Valid values depend on the fault conditions defined for
 * the device.
 * @param value A pointer to a bool where the result will be stored. Must not be
 * null. The function writes true if the fault is present, false
 * otherwise.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ltc7871_get_mfr_fault(struct ltc7871_dev *dev, uint8_t status, bool *value);

/***************************************************************************//**
 * @brief This function checks the overcurrent fault status of the LTC7871
 * device and stores the result in the provided boolean pointer. It
 * should be called when you need to determine if an overcurrent fault
 * condition has been detected by the device. Ensure that the device has
 * been properly initialized before calling this function. The function
 * requires valid pointers for both the device descriptor and the output
 * boolean; otherwise, it returns an error.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null.
 * @param status A uint8_t value representing the status bits to be checked.
 * Valid values depend on the specific status bits defined for the
 * device.
 * @param value A pointer to a boolean where the function will store the
 * overcurrent fault status. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor or output pointer is null, or if there is an error reading
 * the device register.
 ******************************************************************************/
int ltc7871_get_mfr_oc_fault(struct ltc7871_dev *dev, uint8_t status,
			     bool *value);

/***************************************************************************//**
 * @brief This function checks the negative overcurrent fault status of the
 * LTC7871 device by reading the relevant register and extracting the
 * specified status bit. It should be called when you need to determine
 * if a negative overcurrent condition has been detected. Ensure that the
 * device has been properly initialized before calling this function. The
 * function requires valid pointers for the device descriptor and the
 * output value; otherwise, it returns an error.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null.
 * @param status A uint8_t value representing the specific status bit to check
 * within the negative overcurrent fault register.
 * @param value A pointer to a bool where the function will store the result of
 * the status check. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor or output pointer is null, or if there is an error reading
 * the register.
 ******************************************************************************/
int ltc7871_get_mfr_noc_fault(struct ltc7871_dev *dev, uint8_t status,
			      bool *value);

/***************************************************************************//**
 * @brief Use this function to obtain a specific configuration setting from the
 * LTC7871 device's MFR_CONFIG1 register. It is essential to ensure that
 * the device descriptor is properly initialized before calling this
 * function. The function reads the register and extracts the desired
 * configuration field specified by the config parameter. It is important
 * to provide valid pointers for both the device descriptor and the
 * output value to avoid errors. This function is useful when you need to
 * verify or utilize specific configuration settings of the LTC7871
 * device.
 *
 * @param dev A pointer to an initialized ltc7871_dev structure representing the
 * device. Must not be null.
 * @param config A uint8_t value specifying the configuration field to retrieve
 * from the MFR_CONFIG1 register.
 * @param value A pointer to a uint8_t where the retrieved configuration value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as when invalid pointers are provided.
 ******************************************************************************/
int ltc7871_get_mfr_config1_setting(struct ltc7871_dev *dev,
				    uint8_t config, uint8_t *value);

/***************************************************************************//**
 * @brief Use this function to obtain a specific configuration setting from the
 * LTC7871 device's MFR_CONFIG2 register. It is essential to ensure that
 * the device descriptor is properly initialized before calling this
 * function. The function reads the register and extracts the desired
 * configuration field specified by the config parameter. It is important
 * to provide valid pointers for both the device descriptor and the
 * output value parameter, as the function will return an error if either
 * is null.
 *
 * @param dev A pointer to an initialized ltc7871_dev structure representing the
 * device. Must not be null.
 * @param config A uint8_t value specifying the configuration field to retrieve
 * from the MFR_CONFIG2 register. Valid values depend on the
 * specific configuration fields defined for the device.
 * @param value A pointer to a uint8_t where the retrieved configuration value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor or value pointer is null, or if the register read
 * operation fails.
 ******************************************************************************/
int ltc7871_get_mfr_config2_setting(struct ltc7871_dev *dev,
				    uint8_t config, uint8_t *value);

/***************************************************************************//**
 * @brief Use this function to obtain the manufacturer status of the LTC7871
 * device by reading the relevant register and applying a status mask. It
 * is essential to ensure that the device has been properly initialized
 * before calling this function. The function requires valid pointers for
 * both the device descriptor and the status output parameter. If either
 * pointer is null, the function will return an error. This function is
 * useful for checking specific status bits as defined by the status
 * mask.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null.
 * @param status_mask A bitmask indicating which status bits to check. The mask
 * should correspond to the bits of interest in the
 * manufacturer status register.
 * @param status A pointer to a boolean where the result will be stored. Must
 * not be null. The boolean will be set to true if the masked
 * status bits are set, otherwise false.
 * @return Returns 0 on success, or a negative error code if the device or
 * status pointer is null, or if there is an error reading the register.
 ******************************************************************************/
int ltc7871_get_mfr_status(struct ltc7871_dev *dev, uint8_t status_mask,
			   bool *status);

/***************************************************************************//**
 * @brief Use this function to reset all the read/write registers of the LTC7871
 * device to their default states. This function should be called when a
 * reset of the device's configuration is required, such as during
 * initialization or error recovery. It is important to ensure that the
 * device descriptor is properly initialized and not null before calling
 * this function, as passing a null pointer will result in an error. The
 * function returns an error code if the reset operation fails.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null. If null, the function returns an
 * error code.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the input is invalid.
 ******************************************************************************/
int ltc7871_reset(struct ltc7871_dev *dev);

/***************************************************************************//**
 * @brief Use this function to clear the PEC (Packet Error Code) fault bit in
 * the LTC7871 device, which may be set due to communication errors. This
 * function should be called when you need to reset the PEC fault status
 * after handling or acknowledging the fault condition. Ensure that the
 * device descriptor is properly initialized before calling this
 * function. The function will return an error code if the device
 * descriptor is null or if the operation fails.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null. If null, the function returns an
 * error code.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor is null or if the operation fails.
 ******************************************************************************/
int ltc7871_clear_pec_fault(struct ltc7871_dev *dev);

/***************************************************************************//**
 * @brief This function checks the PEC (Packet Error Code) fault status of the
 * LTC7871 device and stores the result in the provided boolean pointer.
 * It should be called when the user needs to verify if a PEC fault has
 * occurred. The function requires a valid device descriptor and a non-
 * null pointer to store the result. It returns an error code if the
 * device descriptor is null or if there is an issue reading the
 * register.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null. If null, the function returns an
 * error.
 * @param value A pointer to a boolean where the PEC fault status will be
 * stored. Must not be null. The function writes the fault status
 * to this location.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor is null or if there is an error reading the register.
 ******************************************************************************/
int ltc7871_read_pec_fault(struct ltc7871_dev *dev, bool *value);

/***************************************************************************//**
 * @brief Use this function to enable or disable the write protection feature of
 * the LTC7871 device. This function should be called when you need to
 * control the write access to certain registers of the device, typically
 * for safety or security reasons. Ensure that the device descriptor is
 * properly initialized before calling this function. If the device
 * descriptor is null, the function will return an error. The function
 * returns an integer indicating success or failure of the operation.
 *
 * @param dev A pointer to an initialized ltc7871_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param value A boolean value indicating the desired state of write
 * protection. True to enable write protection, false to disable
 * it.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ltc7871_set_write_protect(struct ltc7871_dev *dev, bool value);

/***************************************************************************//**
 * @brief Use this function to determine whether the LTC7871 device is currently
 * write-protected. This is useful for ensuring that write operations are
 * allowed before attempting to modify device settings. The function must
 * be called with a valid device descriptor, and it will output the write
 * protection status through the provided boolean pointer. Ensure that
 * the device has been properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null. If null, the function returns an
 * error code.
 * @param value A pointer to a boolean variable where the write protection
 * status will be stored. Must not be null. The function writes
 * `true` if the device is write-protected, otherwise `false`.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor is null or if there is an error reading the device
 * register.
 ******************************************************************************/
int ltc7871_is_write_protected(struct ltc7871_dev *dev, bool *value);

/***************************************************************************//**
 * @brief This function is used to obtain the current DAC value that is used to
 * program the VLOW voltage of the LTC7871 device. It should be called
 * when the user needs to read the VLOW voltage setting from the device.
 * The function requires a valid device descriptor and a pointer to an
 * int8_t variable where the retrieved value will be stored. It is
 * important to ensure that both the device descriptor and the pointer to
 * the value are not null before calling this function, as null pointers
 * will result in an error. The function returns an error code if the
 * read operation fails.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param value A pointer to an int8_t variable where the function will store
 * the retrieved DAC value. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ltc7871_get_mfr_idac_vlow(struct ltc7871_dev *dev, int8_t *value);

/***************************************************************************//**
 * @brief This function sets the IDAC VLOW value for the specified LTC7871
 * device, which is used to program the VLOW voltage. It should be called
 * when you need to update the VLOW voltage setting. The function checks
 * if the device is write-protected and will not perform the write
 * operation if it is. Ensure that the device is properly initialized
 * before calling this function. The function returns an error code if
 * the device is null or if any operation fails.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param value An int8_t value representing the new IDAC VLOW setting. The
 * valid range is determined by the device's specifications.
 * @return Returns 0 on success, or a negative error code if the device is null,
 * write-protected, or if any operation fails.
 ******************************************************************************/
int ltc7871_set_mfr_idac_vlow(struct ltc7871_dev *dev, int8_t value);

/***************************************************************************//**
 * @brief This function is used to obtain the current digital-to-analog
 * converter (DAC) value that is used to program the VHIGH voltage
 * setting of the LTC7871 device. It should be called when you need to
 * read the VHIGH voltage configuration. The function requires a valid
 * device descriptor and a pointer to store the retrieved value. It
 * returns an error code if the device descriptor or the output pointer
 * is null, or if the read operation fails.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null. The function will return an error if
 * this parameter is invalid.
 * @param value A pointer to an int8_t variable where the retrieved DAC value
 * will be stored. Must not be null. The function will return an
 * error if this parameter is invalid.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the input parameters are invalid.
 ******************************************************************************/
int ltc7871_get_mfr_idac_vhigh(struct ltc7871_dev *dev, int8_t *value);

/***************************************************************************//**
 * @brief This function sets the manufacturer IDAC VHIGH value for the specified
 * LTC7871 device. It should be called when you need to configure the
 * VHIGH voltage level through the IDAC. The function checks if the
 * device is write-protected and only performs the write operation if it
 * is not. Ensure that the device descriptor is valid before calling this
 * function. The function returns an error code if the device is null, if
 * there is an error checking the write protection status, or if the
 * write operation fails.
 *
 * @param dev A pointer to the ltc7871_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param value An int8_t value representing the desired IDAC VHIGH setting. The
 * valid range is determined by the device's specifications.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL if the device is null or other error codes for write
 * protection or write operation failures.
 ******************************************************************************/
int ltc7871_set_mfr_idac_vhigh(struct ltc7871_dev *dev, int8_t value);

/***************************************************************************//**
 * @brief This function is used to obtain the current digital-to-analog
 * converter (DAC) value that programs the sourcing current of the SETCUR
 * pin on the LTC7871 device. It should be called when you need to read
 * the current setting of the SETCUR pin. Ensure that the device has been
 * properly initialized before calling this function. The function will
 * return an error if the device descriptor or the output pointer is
 * null.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null.
 * @param value A pointer to an int8_t where the retrieved DAC value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if any input parameter is invalid.
 ******************************************************************************/
int ltc7871_get_mfr_idac_setcur(struct ltc7871_dev *dev, int8_t *value);

/***************************************************************************//**
 * @brief This function is used to store a specified DAC value to program the
 * sourcing current of the SETCUR pin on an LTC7871 device. It should be
 * called when you need to adjust the current setting for the SETCUR pin.
 * The function requires a valid device descriptor and a value to set. It
 * checks if the device is write-protected and will not perform the write
 * operation if write protection is enabled. Ensure that the device
 * descriptor is properly initialized before calling this function.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The function returns -EINVAL if this parameter
 * is null.
 * @param value An int8_t value representing the DAC setting for the SETCUR pin.
 * The function does not specify a range, but it is expected to be
 * within the valid range for the device's DAC settings.
 * @return Returns 0 on success, a negative error code if the device is write-
 * protected or if there is an error in communication.
 ******************************************************************************/
int ltc7871_set_mfr_idac_setcur(struct ltc7871_dev *dev, int8_t value);

/***************************************************************************//**
 * @brief This function is used to obtain the current frequency spread range
 * setting from an LTC7871 device. It should be called when you need to
 * know the spread range configuration of the device. The function
 * requires a valid device descriptor and a pointer to a uint8_t variable
 * where the result will be stored. It returns an error code if the
 * device descriptor or the output pointer is null, or if there is an
 * issue reading from the device.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param value A pointer to a uint8_t where the frequency spread range will be
 * stored. Must not be null. The caller retains ownership.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor or value pointer is null, or if there is a failure in
 * reading the register.
 ******************************************************************************/
int ltc7871_get_freq_spread_range(struct ltc7871_dev *dev, uint8_t *value);

/***************************************************************************//**
 * @brief This function configures the frequency spread range of the LTC7871
 * device to the specified value. It should be called when you need to
 * adjust the frequency spread range for the device's operation. The
 * function requires a valid device descriptor and a frequency spread
 * range value. It checks if the device is write-protected and only
 * performs the write operation if it is not. If the device descriptor is
 * null, the function returns an error. Ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param value An enum ltc7871_ssfm_fsr value representing the desired
 * frequency spread range. Valid values are defined in the
 * ltc7871_ssfm_fsr enumeration.
 * @return Returns 0 on success, a negative error code if the device is null, or
 * if there is an error in checking write protection or writing the
 * register.
 ******************************************************************************/
int ltc7871_set_freq_spread_range(struct ltc7871_dev *dev,
				  enum ltc7871_ssfm_fsr value);

/***************************************************************************//**
 * @brief This function is used to obtain the current modulation signal
 * frequency setting from an LTC7871 device. It should be called when you
 * need to know the modulation frequency configuration of the device. The
 * function requires a valid device descriptor and a pointer to a uint8_t
 * variable where the frequency value will be stored. It returns an error
 * code if the device descriptor or the output pointer is null, or if
 * there is an issue reading from the device.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param value A pointer to a uint8_t where the modulation frequency value will
 * be stored. Must not be null. The function writes the result to
 * this location.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor or value pointer is null, or if there is a read error.
 ******************************************************************************/
int ltc7871_get_mod_freq(struct ltc7871_dev *dev, uint8_t *value);

/***************************************************************************//**
 * @brief This function sets the modulation signal frequency of the LTC7871
 * device to a specified value. It should be called when you need to
 * configure the modulation frequency for the device. The function
 * requires a valid device descriptor and a modulation frequency value
 * from the predefined enumeration. It checks if the device is write-
 * protected and only performs the write operation if it is not. Ensure
 * the device descriptor is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device.
 * Must not be null. The function returns -EINVAL if this parameter
 * is null.
 * @param value An enum ltc7871_ssfm_msf value representing the desired
 * modulation signal frequency. Must be a valid enumeration value.
 * @return Returns 0 on success, a negative error code on failure, or -EINVAL if
 * the device pointer is null.
 ******************************************************************************/
int ltc7871_set_mod_freq(struct ltc7871_dev *dev, enum ltc7871_ssfm_msf value);

/***************************************************************************//**
 * @brief This function sets the PWMEN pin of the LTC7871 device to the
 * specified value. It should be called when you need to control the PWM
 * enable state of the device. Ensure that the device descriptor is
 * properly initialized before calling this function. The function will
 * return an error if the device descriptor is null.
 *
 * @param dev A pointer to an initialized ltc7871_dev structure. Must not be
 * null. If null, the function returns an error.
 * @param value A uint8_t value representing the desired state of the PWMEN pin.
 * Typically, 1 to set and 0 to clear the pin.
 * @return Returns 0 on success or a negative error code if the device
 * descriptor is null or if setting the pin value fails.
 ******************************************************************************/
int ltc7871_set_pwmen_pin(struct ltc7871_dev *dev, uint8_t value);

/***************************************************************************//**
 * @brief Use this function to obtain the current state of the PWMEN pin
 * associated with the LTC7871 device. It is essential to ensure that
 * both the device descriptor and the output pointer are valid before
 * calling this function. The function will return an error if either of
 * these pointers is null. This function is typically used to monitor or
 * verify the state of the PWMEN pin in applications where the pin's
 * state affects device operation.
 *
 * @param dev A pointer to an initialized `ltc7871_dev` structure representing
 * the device. Must not be null.
 * @param value A pointer to a uint8_t where the state of the PWMEN pin will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid.
 ******************************************************************************/
int ltc7871_get_pwmen_pin(struct ltc7871_dev *dev, uint8_t *value);

/***************************************************************************//**
 * @brief This function sets up the LTC7871 device by allocating necessary
 * resources and configuring the device according to the provided
 * initialization parameters. It must be called before any other
 * operations on the device to ensure proper setup. The function handles
 * resource allocation and initialization of SPI and GPIO interfaces as
 * specified in the initialization parameters. If any step fails, it
 * cleans up allocated resources and returns an error code. Ensure that
 * the `device` pointer is valid and that `init_param` contains valid
 * configuration data before calling this function.
 *
 * @param device A pointer to a pointer of type `struct ltc7871_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ltc7871_init_param` containing
 * initialization parameters for the device. Must not be null
 * and should contain valid SPI and GPIO configuration data.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, `device` points to a newly allocated and initialized device
 * descriptor.
 ******************************************************************************/
int ltc7871_init(struct ltc7871_dev **device,
		 struct ltc7871_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * LTC7871 device when it is no longer needed. This function should be
 * called to clean up after the device has been initialized and used,
 * ensuring that any associated GPIO and SPI resources are also released.
 * It is important to pass a valid device descriptor to avoid undefined
 * behavior. If the provided device descriptor is null, the function will
 * return an error code indicating that the device is not available.
 *
 * @param dev A pointer to an ltc7871_dev structure representing the device to
 * be removed. Must not be null. If null, the function returns
 * -ENODEV.
 * @return Returns 0 on successful removal of the device resources, or -ENODEV
 * if the device descriptor is null.
 ******************************************************************************/
int ltc7871_remove(struct ltc7871_dev *dev);

#endif /** __LTC7871_H__ */
