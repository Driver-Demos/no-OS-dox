/***************************************************************************//**
 *   @file   ade7913.h
 *   @brief  Header file of ADE7913 Driver.
 *   @author Radu Etz (radu.etz@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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
#ifndef __ADE7913_H__
#define __ADE7913_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_irq.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* SPI commands */
#define ADE7913_SPI_READ				NO_OS_BIT(2)

/* ENABLE and DISABLE */
#define ENABLE						1u
#define DISABLE						0u

/* ADE7913 Register Map */
#define ADE7913_REG_IWV					0x00
#define ADE7913_REG_V1WV				0x01
#define ADE7913_REG_V2WV				0x02
#define ADE7913_REG_ADC_CRC				0x04
#define ADE7913_REG_CTRL_CRC				0x05
#define ADE7913_REG_CNT_SNAPSHOT			0x07
#define ADE7913_REG_CONFIG				0x08
#define ADE7913_REG_STATUS0				0x09
#define ADE7913_REG_LOCK				0x0A
#define ADE7913_REG_SYNC_SNAP				0x0B
#define ADE7913_REG_COUNTER0				0x0C
#define ADE7913_REG_COUNTER1				0x0D
#define ADE7913_REG_EMI_CTRL				0x0E
#define ADE7913_REG_STATUS1				0x0F
#define ADE7913_REG_TEMPOS				0x08

/* ADE7913_REG_CNT_SNAPSHOT Bit Definition */
#define ADE7913_CNT_VAL_MSK				NO_OS_GENMASK(11, 0)

/* ADE7913_REG_CONFIG Bit Definition */
#define ADE7913_CLKOUT_EN_MSK				NO_OS_BIT(0)
#define ADE7913_PWRDWN_EN_MSK				NO_OS_BIT(2)
#define ADE7913_TEMP_EN_MSK				NO_OS_BIT(3)
#define ADE7913_ADC_FREQ_MSK				NO_OS_GENMASK(5, 4)
#define ADE7913_SWRST_MSK				NO_OS_BIT(6)
#define ADE7913_BW_MSK					NO_OS_BIT(7)

/* ADE7913_REG_STATUS0 Bit Definition */
#define ADE7913_RESET_ON_MSK				NO_OS_BIT(0)
#define ADE7913_CRC_STAT_MSK				NO_OS_BIT(1)
#define ADE7913_IC_PROT_MSK				NO_OS_BIT(2)

/* ADE7913_REG_LOCK Bit Definition */
#define ADE7913_LOCK_KEY_MSK				NO_OS_GENMASK(5, 4)

/* ADE7913_REG_SYNC_SNAP Bit Definition */
#define ADE7913_SYNC_MSK				NO_OS_BIT(0)
#define ADE7913_SNAP_MSK				NO_OS_BIT(1)

/* ADE7913_REG_EMI_CTRL Bit Definition */
#define ADE7913_SLOT0_MSK				NO_OS_BIT(0)
#define ADE7913_SLOT1_MSK				NO_OS_BIT(1)
#define ADE7913_SLOT2_MSK				NO_OS_BIT(2)
#define ADE7913_SLOT3_MSK				NO_OS_BIT(3)
#define ADE7913_SLOT4_MSK				NO_OS_BIT(4)
#define ADE7913_SLOT5_MSK				NO_OS_BIT(5)
#define ADE7913_SLOT6_MSK				NO_OS_BIT(6)
#define ADE7913_SLOT7_MSK				NO_OS_BIT(7)

/* ADE7913_REG_STATUS1 Bit Definition */
#define ADE7913_VERSION_MSK				NO_OS_GENMASK(2, 0)
#define ADE7913_ADC_NA_MSK				NO_OS_BIT(6)

/* Configuration Register Write Lock */
#define ADE7913_LOCK_KEY				0XCA
#define ADE7913_UNLOCK_KEY				0X9C

/* Version Product */
#define ADE7913_3_CHANNEL_ADE7913			3U
#define ADE7913_2_CHANNEL_ADE7912			2U

/* Nominal reference voltage */
#define ADE7913_VREF_V					(788)
#define ADE7913_VREF_I					(49)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ade7913_adc_freq_e` enumeration defines the possible ADC output
 * frequencies for the ADE7913 device, each associated with a specific
 * sampling period. This allows the user to select the desired frequency
 * for the ADC operation, which can be crucial for applications requiring
 * specific timing and data acquisition rates.
 *
 * @param ADE7913_ADC_FREQ_8KHZ Represents an ADC frequency with a period of 125
 * microseconds.
 * @param ADE7913_ADC_FREQ_4KHZ Represents an ADC frequency with a period of 250
 * microseconds.
 * @param ADE7913_ADC_FREQ_2KHZ Represents an ADC frequency with a period of 500
 * microseconds.
 * @param ADE7913_ADC_FREQ_1KHZ Represents an ADC frequency with a period of 1
 * millisecond.
 ******************************************************************************/
enum ade7913_adc_freq_e {
	/* 125 us period */
	ADE7913_ADC_FREQ_8KHZ,
	/* 250 us period */
	ADE7913_ADC_FREQ_4KHZ,
	/* 500 us period */
	ADE7913_ADC_FREQ_2KHZ,
	/* 1 ms period */
	ADE7913_ADC_FREQ_1KHZ
};

/***************************************************************************//**
 * @brief The `ade7913_init_param` structure is used to define the
 * initialization parameters for up to three ADE7913 devices. It includes
 * the number of devices to be initialized, a flag for enabling burst
 * mode, and pointers to SPI initialization parameters for each device.
 * This structure is essential for setting up the communication and
 * operational parameters of the ADE7913 devices in a system.
 *
 * @param no_devs Specifies the number of ADE7913 devices to be initialized.
 * @param burst_mode Indicates whether burst mode is enabled for the devices.
 * @param spi_init0 Pointer to the SPI initialization parameters for the first
 * device.
 * @param spi_init1 Pointer to the SPI initialization parameters for the second
 * device.
 * @param spi_init2 Pointer to the SPI initialization parameters for the third
 * device.
 ******************************************************************************/
struct ade7913_init_param {
	/* Number of devices */
	uint8_t no_devs;
	/* burst mode */
	uint8_t burst_mode;
	/* Device 1 communication descriptor */
	struct no_os_spi_init_param 	*spi_init0;
	/* Device 2 communication descriptor */
	struct no_os_spi_init_param 	*spi_init1;
	/* Device 3 communication descriptor */
	struct no_os_spi_init_param 	*spi_init2;
};

/***************************************************************************//**
 * @brief The `ade7913_dev` structure is designed to manage and interface with
 * ADE7913 devices, which are used for energy measurement and monitoring.
 * It contains pointers to SPI descriptors for up to three devices,
 * allowing for communication and data exchange. The structure also holds
 * waveform data for current and voltage, both for single and multiple
 * devices, as well as various status and control information such as ADC
 * CRC, device status, and count snapshots. Additionally, it includes
 * fields for managing the number of devices, burst mode operation, and
 * an IRQ control descriptor for handling interrupts.
 *
 * @param spi_desc Pointer to the SPI communication descriptor for the device.
 * @param spi_desc0 Pointer to the SPI communication descriptor for Device 1.
 * @param spi_desc1 Pointer to the SPI communication descriptor for Device 2.
 * @param spi_desc2 Pointer to the SPI communication descriptor for Device 3.
 * @param ver_product Pointer to the version product information.
 * @param i_wav Pointer to the current waveform data.
 * @param v1_wav Pointer to the voltage 1 waveform data.
 * @param v2_wav Pointer to the voltage 2 waveform data.
 * @param i_wav_m Pointer to the current waveform data for multiple devices.
 * @param v1_wav_m Pointer to the voltage 1 waveform data for multiple devices.
 * @param v2_wav_m Pointer to the voltage 2 waveform data for multiple devices.
 * @param adc_crc Pointer to the ADC CRC data.
 * @param status0 Pointer to the status 0 information.
 * @param cnt_snapshot Pointer to the count snapshot data.
 * @param no_devs Number of devices connected.
 * @param burst_mode Indicates whether burst mode is enabled.
 * @param irq_ctrl Pointer to the IRQ control descriptor for handling GPIO RDY
 * interrupts.
 ******************************************************************************/
struct ade7913_dev {
	/** Device communication descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** Device 1 communication descriptor */
	struct no_os_spi_desc		*spi_desc0;
	/** Device 2 communication descriptor */
	struct no_os_spi_desc		*spi_desc1;
	/** Device 3 communication descriptor */
	struct no_os_spi_desc		*spi_desc2;
	/* Version product */
	uint8_t 			*ver_product;
	/* I_WAV */
	int32_t				*i_wav;
	/* V1_WAV */
	int32_t				*v1_wav;
	/* V2_WAV */
	int32_t				*v2_wav;
	/* I_WAV multiple devices */
	int32_t				*i_wav_m;
	/* V1_WAV multiple devices */
	int32_t				*v1_wav_m;
	/* V2_WAV multiple devices */
	int32_t				*v2_wav_m;
	/* ADC_CRC */
	uint16_t            		*adc_crc;
	/* Status 0 */
	uint8_t             		*status0;
	/* CNT_SNAPSHOT */
	uint16_t            		*cnt_snapshot;
	/* number of devices */
	uint8_t 			no_devs;
	/* burst mode */
	uint8_t 			burst_mode;
	/** IRQ device descriptor used to handle interrupt routine for GPIO RDY */
	struct no_os_irq_ctrl_desc 	*irq_ctrl;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

// Read device register.
/***************************************************************************//**
 * @brief This function reads data from a specified register of the ADE7913
 * device. It requires a valid device structure and a register address to
 * read from. The function supports both burst mode and individual
 * register reads, adjusting the number of bytes read accordingly. It
 * must be called with a properly initialized device structure and a non-
 * null pointer for storing the read data. The function returns an error
 * code if the device structure or data pointer is invalid, or if the
 * read operation fails.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. If null, the function returns -ENODEV.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address defined for the ADE7913.
 * @param reg_data A pointer to a uint8_t where the read data will be stored.
 * Must not be null. If null, the function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure. The read
 * data is stored in the location pointed to by reg_data.
 ******************************************************************************/
int ade7913_read(struct ade7913_dev *dev, uint8_t reg_addr, uint8_t *reg_data);

// Read multiple devices
/***************************************************************************//**
 * @brief This function reads waveform data from one or more ADE7913 devices
 * connected to the system. It should be called when waveform data is
 * needed from the devices, and it supports configurations with up to
 * three devices. The function requires a valid device structure and a
 * buffer to store the read data. It returns an error if the device
 * structure is null, the data buffer is null, or if the number of
 * devices is outside the supported range of 1 to 3. The function updates
 * the waveform data in the provided buffer for each connected device.
 *
 * @param dev A pointer to an ade7913_dev structure representing the device
 * configuration. Must not be null. The structure should be properly
 * initialized and configured with the number of devices (1 to 3)
 * before calling this function.
 * @param reg_addr The register address from which to read the waveform data. It
 * is a uint8_t value representing the specific register in the
 * ADE7913 device.
 * @param reg_data A pointer to a uint8_t buffer where the read waveform data
 * will be stored. Must not be null. The buffer should be large
 * enough to hold the data from the specified register for all
 * connected devices.
 * @return Returns 0 on success. On failure, returns a negative error code:
 * -ENODEV if the device structure is null, -EINVAL if the data buffer
 * is null or the number of devices is invalid.
 ******************************************************************************/
int ade7913_read_waveforms(struct ade7913_dev *dev, uint8_t reg_addr,
			   uint8_t *reg_data);

// Write device register.
/***************************************************************************//**
 * @brief Use this function to write a byte of data to a specific register of
 * the ADE7913 device. It is essential to ensure that the device
 * structure is properly initialized and not null before calling this
 * function. This function is typically used when configuring the device
 * or updating its settings. If the device structure is null, the
 * function will return an error code indicating that the device is not
 * available. The function communicates with the device over SPI, and any
 * errors during this communication will be reflected in the return
 * value.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg_addr The address of the register to write to. It should be a valid
 * register address as defined by the device's register map.
 * @param reg_data The data byte to write to the specified register. It should
 * be a valid byte value (0-255).
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if there is an SPI communication error.
 ******************************************************************************/
int ade7913_write(struct ade7913_dev *dev, uint8_t reg_addr, uint8_t reg_data);

// Write broadcast.
/***************************************************************************//**
 * @brief Use this function to broadcast a write operation to the same register
 * on multiple ADE7913 devices connected to the system. It is
 * particularly useful when you need to configure or update the same
 * setting across all devices simultaneously. The function requires a
 * valid device structure and data to write. It handles up to three
 * devices, writing to each in sequence. Ensure that the device structure
 * is properly initialized and that the number of devices is correctly
 * set before calling this function. The function returns an error code
 * if the device structure or data pointer is null, or if any write
 * operation fails.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device configuration. Must not be null. The structure should
 * correctly reflect the number of devices to be written to.
 * @param reg_addr The address of the register to write to. It should be a valid
 * register address for the ADE7913 device.
 * @param reg_data A pointer to the data to be written to the register. Must not
 * be null. The data is broadcast to all devices specified in
 * the device structure.
 * @return Returns 0 on success, or a negative error code if the device
 * structure is null, the data pointer is null, or if any write
 * operation fails.
 ******************************************************************************/
int ade7913_write_broadcast(struct ade7913_dev *dev, uint8_t reg_addr,
			    uint8_t *reg_data);

// Update specific register bits.
/***************************************************************************//**
 * @brief This function updates specific bits in a register of the ADE7913
 * device. It is used when only certain bits of a register need to be
 * modified without affecting the other bits. The function reads the
 * current value of the register, applies a mask to clear the bits to be
 * updated, and then sets the new bits as specified by the input. It must
 * be called with a valid device structure pointer. If the device pointer
 * is null, the function returns an error. This function is typically
 * used in scenarios where precise control over register bits is
 * required, such as configuring device settings or enabling specific
 * features.
 *
 * @param dev A pointer to an ade7913_dev structure representing the device.
 * Must not be null. The function returns an error if this parameter
 * is null.
 * @param reg_addr The address of the register to be updated. It specifies which
 * register in the device will have its bits modified.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask are the ones that will be
 * modified.
 * @param reg_data The new data to be written to the specified bits in the
 * register. Only the bits specified by the mask will be updated
 * with this data.
 * @return Returns 0 on success or a negative error code if the operation fails,
 * such as when the device pointer is null or if reading or writing the
 * register fails.
 ******************************************************************************/
static int ade7913_update_bits(struct ade7913_dev *dev, uint8_t reg_addr,
			       uint8_t mask, uint8_t reg_data);

// Initialize the device.
/***************************************************************************//**
 * @brief This function sets up the ADE7913 device by allocating necessary
 * resources and initializing communication parameters based on the
 * provided initialization parameters. It supports up to three devices,
 * defaulting to one if not specified. The function must be called before
 * any other operations on the device, and it handles memory allocation
 * and SPI initialization for each device. If initialization fails at any
 * point, it cleans up allocated resources and returns an error code.
 *
 * @param device A pointer to a pointer of ade7913_dev structure where the
 * initialized device instance will be stored. Must not be null.
 * @param init_param A structure containing initialization parameters such as
 * the number of devices, burst mode, and SPI initialization
 * parameters for each device. The number of devices must be
 * between 1 and 3.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures that no resources are left allocated.
 ******************************************************************************/
int ade7913_init(struct ade7913_dev **device,
		 struct ade7913_init_param init_param);

// Remove the device and release resources.
/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADE7913 device when it is no longer needed. This includes closing SPI
 * communication descriptors and freeing memory associated with the
 * device structure and its components. It is important to call this
 * function to prevent resource leaks. Ensure that the device pointer is
 * valid and initialized before calling this function.
 *
 * @param dev A pointer to an initialized `ade7913_dev` structure representing
 * the device to be removed. Must not be null. The function will
 * handle invalid pointers by returning an error code.
 * @return Returns 0 on successful removal of the device and its resources, or a
 * negative error code if any SPI descriptor removal fails.
 ******************************************************************************/
int ade7913_remove(struct ade7913_dev *dev);

// Reset the device using SW reset.
/***************************************************************************//**
 * @brief Use this function to reset the ADE7913 device via software, which is
 * necessary when you need to reinitialize the device or clear its
 * current state. This function should be called when the device is in a
 * known state and ready to be reset. It waits for the device to complete
 * its initialization process after the reset and updates the device
 * structure with the version product information. Ensure that the device
 * structure is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized `ade7913_dev` structure representing
 * the device to reset. Must not be null. If null, the function
 * returns an error code indicating the device is not available.
 * @return Returns 0 on success, or a negative error code if the reset fails or
 * if the device is not available.
 ******************************************************************************/
int ade7913_sw_reset(struct ade7913_dev *dev);

// Lock device.
/***************************************************************************//**
 * @brief Use this function to lock the configuration registers of the ADE7913
 * device, preventing any further modifications until the device is
 * unlocked. This is typically done to ensure the device settings remain
 * unchanged during operation. The function must be called with a valid
 * device structure that has been properly initialized. If the device
 * pointer is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. If null, the function returns -ENODEV.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade7913_wr_lock(struct ade7913_dev *dev);

// Unlock device.
/***************************************************************************//**
 * @brief Use this function to unlock the ADE7913 device, allowing writes to its
 * configuration registers. This function should be called when you need
 * to modify the device's configuration settings. Ensure that the device
 * has been properly initialized before calling this function. If the
 * device pointer is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. If null, the function returns -ENODEV.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade7913_wr_unlock(struct ade7913_dev *dev);

// Get synchronization counter value.
/***************************************************************************//**
 * @brief Use this function to obtain the current synchronization counter value
 * from an ADE7913 device. It is essential to ensure that the device has
 * been properly initialized before calling this function. The function
 * requires a valid device structure and a pointer to a uint16_t variable
 * where the counter value will be stored. If the device structure or the
 * counter pointer is null, the function will return an error code. This
 * function is useful for applications that need to monitor or
 * synchronize operations based on the counter value.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. If null, the function returns -ENODEV.
 * @param counter A pointer to a uint16_t variable where the synchronization
 * counter value will be stored. Must not be null. If null, the
 * function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code if the device is not
 * initialized or if there is a communication error.
 ******************************************************************************/
int ade7913_get_sync_cnt_val(struct ade7913_dev *dev, uint16_t *counter);

// Set clkout enable.
/***************************************************************************//**
 * @brief This function is used to control the CLKOUT signal of an ADE7913
 * device by enabling or disabling it based on the provided parameter. It
 * should be called when there is a need to configure the CLKOUT signal
 * for synchronization or other purposes. The function requires a valid
 * device structure and will return an error if the device is not
 * properly initialized. It is important to ensure that the device is
 * unlocked if necessary before calling this function to allow
 * configuration changes.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. The function will return -ENODEV if this
 * parameter is null.
 * @param clkout_en A uint8_t value indicating whether to enable (1) or disable
 * (0) the CLKOUT signal. Values outside this range may lead to
 * undefined behavior.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -ENODEV if the device pointer is null.
 ******************************************************************************/
int ade7913_set_clkout_en(struct ade7913_dev *dev,
			  uint8_t clkout_en);

// Power down enable.
/***************************************************************************//**
 * @brief This function is used to control the power-down mode of an ADE7913
 * device. It should be called when you need to enable or disable the
 * power-down feature of the device, which can be useful for power
 * management in your application. The function requires a valid device
 * structure pointer and a power-down control flag. It returns an error
 * code if the device pointer is null, indicating that the device is not
 * available.
 *
 * @param dev A pointer to an ade7913_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param pwrdwn A uint8_t value indicating whether to enable (1) or disable (0)
 * the power-down mode. Values outside this range may lead to
 * undefined behavior.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade7913_pwrdwn(struct ade7913_dev *dev,
		   uint8_t pwrdwn);

// Temperature enable.
/***************************************************************************//**
 * @brief Use this function to control the temperature sensor feature of the
 * ADE7913 device. It allows enabling or disabling the temperature sensor
 * by updating the appropriate configuration register. This function
 * should be called only after the device has been properly initialized.
 * If the device pointer is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null.
 * @param temp_en A uint8_t value indicating whether to enable (1) or disable
 * (0) the temperature sensor. Values outside this range may lead
 * to undefined behavior.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null or if there is an issue updating the register.
 ******************************************************************************/
int ade7913_temp_en(struct ade7913_dev *dev,
		    uint8_t temp_en);

// Sync enable.
/***************************************************************************//**
 * @brief This function enables the synchronization feature on the ADE7913
 * device, which is typically used to align operations across multiple
 * devices or to synchronize with an external event. It should be called
 * when synchronization is required for the device's operation. The
 * function must be called with a valid device structure that has been
 * properly initialized. If the device pointer is null, the function will
 * return an error code indicating that the device is not available.
 *
 * @param dev A pointer to an initialized `ade7913_dev` structure representing
 * the device. Must not be null. If null, the function returns
 * -ENODEV.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if the operation fails.
 ******************************************************************************/
int ade7913_sync_en(struct ade7913_dev *dev);

// Set ADC frequency
/***************************************************************************//**
 * @brief Use this function to configure the ADC output frequency of an ADE7913
 * device to one of the predefined frequency settings. This function
 * should be called after the device has been properly initialized. It
 * requires a valid device structure and a frequency enumeration value.
 * If the device pointer is null, the function will return an error. The
 * function modifies the device's configuration register to set the
 * desired frequency.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. If null, the function returns an error
 * code.
 * @param frequency An enumeration value of type ade7913_adc_freq_e representing
 * the desired ADC output frequency. Valid values are
 * ADE7913_ADC_FREQ_1KHZ, ADE7913_ADC_FREQ_2KHZ,
 * ADE7913_ADC_FREQ_4KHZ, and ADE7913_ADC_FREQ_8KHZ.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null or if there is an issue updating the device's configuration.
 ******************************************************************************/
int ade7913_adc_freq(struct ade7913_dev *dev,
		     enum ade7913_adc_freq_e frequency);

// Digital lpf bandwith select.
/***************************************************************************//**
 * @brief This function is used to configure the bandwidth of the digital low-
 * pass filter in the ADE7913 device. It should be called when there is a
 * need to adjust the filter settings for specific application
 * requirements. The function requires a valid device structure pointer
 * and a bandwidth setting. It returns an error code if the device
 * pointer is null, indicating that the device is not available.
 *
 * @param dev A pointer to an ade7913_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param bw An 8-bit unsigned integer representing the desired bandwidth
 * setting. The valid range is determined by the device's
 * specifications.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade7913_lfp_bw(struct ade7913_dev *dev, uint8_t bw);

// CRC of config registers status
/***************************************************************************//**
 * @brief Use this function to check the CRC status of the ADE7913 device, which
 * indicates the integrity of the configuration registers. It should be
 * called when you need to verify the correctness of the device's
 * configuration. Ensure that the device has been properly initialized
 * before calling this function. The function requires valid pointers for
 * both the device structure and the status output parameter. If the
 * device pointer is null, the function returns an error indicating the
 * device is not available. If the status pointer is null, it returns an
 * invalid argument error.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. If null, the function returns -ENODEV.
 * @param status A pointer to a uint8_t where the CRC status will be stored.
 * Must not be null. If null, the function returns -EINVAL.
 * @return Returns 0 on success, with the CRC status stored in the provided
 * status pointer. Returns a negative error code on failure.
 ******************************************************************************/
int ade7913_crc_status(struct ade7913_dev *dev, uint8_t *status);

// IC config regs protection status.
/***************************************************************************//**
 * @brief This function checks the integrated circuit (IC) protection status of
 * the ADE7913 device and stores the result in the provided status
 * variable. It should be called when you need to verify the protection
 * status of the device. The function requires a valid device structure
 * and a non-null pointer for the status output. It returns an error code
 * if the device is not initialized or if the status pointer is null.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. If null, the function returns -ENODEV.
 * @param status A pointer to a uint8_t variable where the IC protection status
 * will be stored. Must not be null. If null, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure. The status
 * variable is updated with the IC protection status if the function
 * succeeds.
 ******************************************************************************/
int ade7913_ic_prot_status(struct ade7913_dev *dev, uint8_t *status);

// Set EMI CTRL register.
/***************************************************************************//**
 * @brief This function configures the EMI control register of the ADE7913
 * device, which is used to manage electromagnetic interference settings.
 * It should be called when you need to update the EMI control settings
 * of the device. The function requires a valid device structure pointer
 * and an 8-bit value representing the desired EMI control configuration.
 * It returns an error code if the device pointer is null, indicating
 * that the device is not available.
 *
 * @param dev A pointer to an ade7913_dev structure representing the device.
 * Must not be null. The function will return an error if this
 * parameter is null.
 * @param emi_ctrl An 8-bit unsigned integer representing the EMI control
 * settings to be written to the device. The specific bits and
 * their meanings are defined by the device's EMI control
 * register.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null or if writing to the device fails.
 ******************************************************************************/
int ade7913_emi_ctrl(struct ade7913_dev *dev,
		     uint8_t emi_ctrl);

// ADC not accesed during one period status.
/***************************************************************************//**
 * @brief This function checks the status of the ADC to determine if it was not
 * accessed during a specific period. It should be called when you need
 * to verify the ADC's access status. The function requires a valid
 * device structure and a pointer to store the status result. It returns
 * an error code if the device structure or status pointer is null, or if
 * there is an issue reading the device register.
 *
 * @param dev A pointer to an initialized `ade7913_dev` structure representing
 * the device. Must not be null. If null, the function returns
 * -ENODEV.
 * @param status A pointer to a uint8_t where the ADC not accessed status will
 * be stored. Must not be null. If null, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure. The status
 * of the ADC not accessed is stored in the provided `status` pointer on
 * success.
 ******************************************************************************/
int ade7913_adc_na_status(struct ade7913_dev *dev, uint8_t *status);

// Cnt snapshot.
/***************************************************************************//**
 * @brief This function is used to obtain the current snapshot value from the
 * ADE7913 device's counter register. It should be called when a snapshot
 * of the counter value is needed for monitoring or processing purposes.
 * The function requires a valid device structure and a pointer to store
 * the retrieved value. It returns an error code if the device is not
 * initialized or if the provided pointer is null.
 *
 * @param dev A pointer to an initialized `ade7913_dev` structure representing
 * the device. Must not be null. If null, the function returns
 * -ENODEV.
 * @param val A pointer to a `uint16_t` where the snapshot value will be stored.
 * Must not be null. If null, the function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade7913_cnt_snapshot_val(struct ade7913_dev *dev, uint16_t *val);

// Get version product value.
/***************************************************************************//**
 * @brief Use this function to obtain the version product identifier of an
 * ADE7913 device, which indicates the specific model or configuration of
 * the device. This function should be called when you need to verify the
 * device type or ensure compatibility with specific features. It
 * requires a valid device structure and a pointer to store the version
 * product. The function will return an error if the device structure or
 * the output pointer is null, or if the device cannot be accessed.
 *
 * @param dev A pointer to an initialized ade7913_dev structure representing the
 * device. Must not be null. If null, the function returns -ENODEV.
 * @param ver_product A pointer to a uint8_t where the version product
 * identifier will be stored. Must not be null. If null, the
 * function returns -EINVAL.
 * @return Returns 0 on success, with the version product identifier stored in
 * the location pointed to by ver_product. Returns a negative error code
 * on failure, such as -ENODEV if the device is not accessible or
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int ade7913_get_version_product(struct ade7913_dev *dev,
				uint8_t *ver_product);

#endif // __ADE7913_H__
