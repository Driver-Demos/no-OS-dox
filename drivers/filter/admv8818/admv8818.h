/***************************************************************************//**
 *   @file   admv8818.h
 *   @brief  Header file for admv8818 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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

#ifndef ADMV8818_H_
#define ADMV8818_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* ADMV8818 Register Map */
#define ADMV8818_REG_SPI_CONFIG_A		0x0
#define ADMV8818_REG_SPI_CONFIG_B		0x1
#define ADMV8818_REG_CHIPTYPE			0x3
#define ADMV8818_REG_PRODUCT_ID_L		0x4
#define ADMV8818_REG_PRODUCT_ID_H		0x5
#define ADMV8818_REG_FAST_LATCH_POINTER		0x10
#define ADMV8818_REG_FAST_LATCH_STOP		0x11
#define ADMV8818_REG_FAST_LATCH_START		0x12
#define ADMV8818_REG_FAST_LATCH_DIRECTION	0x13
#define ADMV8818_REG_FAST_LATCH_STATE		0x14
#define ADMV8818_REG_WR0_SW			0x20
#define ADMV8818_REG_WR0_FILTER			0x21
#define ADMV8818_REG_WR1_SW			0x22
#define ADMV8818_REG_WR1_FILTER			0x23
#define ADMV8818_REG_WR2_SW			0x24
#define ADMV8818_REG_WR2_FILTER			0x25
#define ADMV8818_REG_WR3_SW			0x26
#define ADMV8818_REG_WR3_FILTER			0x27
#define ADMV8818_REG_WR4_SW			0x28
#define ADMV8818_REG_WR4_FILTER			0x29
#define ADMV8818_REG_LUT0_SW			0x100
#define ADMV8818_REG_LUT0_FILTER		0x101
#define ADMV8818_REG_LUT127_SW			0x1FE
#define ADMV8818_REG_LUT127_FILTER		0x1FF

/* ADMV8818_REG_SPI_CONFIG_A Map */
#define ADMV8818_SOFTRESET_N_MSK		NO_OS_BIT(7)
#define ADMV8818_LSB_FIRST_N_MSK		NO_OS_BIT(6)
#define ADMV8818_ENDIAN_N_MSK			NO_OS_BIT(5)
#define ADMV8818_SDOACTIVE_N_MSK		NO_OS_BIT(4)
#define ADMV8818_SDOACTIVE_MSK			NO_OS_BIT(3)
#define ADMV8818_ENDIAN_MSK			NO_OS_BIT(2)
#define ADMV8818_LSBFIRST_MSK			NO_OS_BIT(1)
#define ADMV8818_SOFTRESET_MSK			NO_OS_BIT(0)

/* ADMV8818_REG_SPI_CONFIG_B Map */
#define ADMV8818_SINGLE_INSTRUCTION_MSK		NO_OS_BIT(7)
#define ADMV8818_CSB_STALL_MSK			NO_OS_BIT(6)
#define ADMV8818_MASTER_SLAVE_RB_MSK		NO_OS_BIT(5)
#define ADMV8818_MASTER_SLAVE_TRANSFER_MSK	NO_OS_BIT(0)

/* ADMV8818_REG_WR0_SW Map */
#define ADMV8818_SW_IN_SET_WR0_MSK		NO_OS_BIT(7)
#define ADMV8818_SW_OUT_SET_WR0_MSK		NO_OS_BIT(6)
#define ADMV8818_SW_IN_WR0_MSK			NO_OS_GENMASK(5, 3)
#define ADMV8818_SW_OUT_WR0_MSK			NO_OS_GENMASK(2, 0)

/* ADMV8818_REG_WR0_FILTER Map */
#define ADMV8818_HPF_WR0_MSK			NO_OS_GENMASK(7, 4)
#define ADMV8818_LPF_WR0_MSK			NO_OS_GENMASK(3, 0)

/* Specifications */
#define ADMV8818_BUFF_SIZE_BYTES		3
#define ADMV8818_CHIP_ID			NO_OS_BIT(0)
#define ADMV8818_SPI_READ_CMD			NO_OS_BIT(7)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `admv8818_filter_mode` is an enumeration that defines the filter
 * modes available for the ADMV8818 device, allowing it to operate in
 * either automatic or manual mode. This enum is used to configure the
 * filter mode setting within the ADMV8818 device, which is a part of its
 * initialization and operational parameters.
 *
 * @param ADMV8818_AUTO Represents the automatic filter mode for the ADMV8818.
 * @param ADMV8818_MANUAL Represents the manual filter mode for the ADMV8818.
 ******************************************************************************/
enum admv8818_filter_mode {
	ADMV8818_AUTO,
	ADMV8818_MANUAL,
};

/***************************************************************************//**
 * @brief The `admv8818_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the ADMV8818 device.
 * It includes a pointer to SPI initialization parameters, the local
 * oscillator input frequency, and the filter mode, which can be either
 * automatic or manual. This structure is essential for configuring the
 * device before use, ensuring that the SPI communication and frequency
 * settings are correctly established.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param rf_in Specifies the local oscillator input frequency.
 * @param mode Defines the filter mode of the ADMV8818.
 ******************************************************************************/
struct admv8818_init_param {
	/** SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
	/** LO Input Frequency */
	unsigned long long		rf_in;
	/* Filter Mode */
	enum admv8818_filter_mode	mode;
};

/***************************************************************************//**
 * @brief The `admv8818_dev` structure is a device descriptor for the ADMV8818,
 * a wideband microwave downconverter. It encapsulates the necessary
 * information for interfacing with the device, including a pointer to
 * the SPI descriptor for communication, the local oscillator input
 * frequency, and the filter mode setting. This structure is used to
 * manage the state and configuration of the ADMV8818 device within the
 * driver.
 *
 * @param spi_desc A pointer to the SPI descriptor used for communication with
 * the device.
 * @param rf_in The local oscillator input frequency for the device.
 * @param mode The filter mode setting for the device, defined by the
 * admv8818_filter_mode enum.
 ******************************************************************************/
struct admv8818_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** LO Input Frequency */
	unsigned long long		rf_in;
	/* Filter Mode */
	enum admv8818_filter_mode	mode;
};

/***************************************************************************//**
 * @brief Use this function to write a byte of data to a specific register of
 * the ADMV8818 device. It requires a valid device descriptor and the
 * register address to which the data will be written. This function is
 * typically used to configure or control the ADMV8818 device by updating
 * its registers. Ensure that the device has been properly initialized
 * before calling this function. The function returns an integer status
 * code indicating the success or failure of the SPI write operation.
 *
 * @param dev A pointer to an initialized 'admv8818_dev' structure representing
 * the device. Must not be null.
 * @param reg_addr A 16-bit unsigned integer specifying the address of the
 * register to write to. Valid register addresses are defined by
 * the device's register map.
 * @param data An 8-bit unsigned integer representing the data to be written to
 * the specified register.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int admv8818_spi_write(struct admv8818_dev *dev, uint16_t reg_addr,
		       uint8_t data);

/* ADMV8818 Register Update */
/***************************************************************************//**
 * @brief Use this function to modify specific bits in a register of the
 * ADMV8818 device by applying a mask and new data. This function is
 * typically used when only certain bits of a register need to be changed
 * without affecting the other bits. It reads the current value of the
 * register, applies the mask to clear the bits to be updated, and then
 * sets the new data. This function should be called only after the
 * device has been properly initialized. It returns an error code if the
 * read or write operation fails.
 *
 * @param dev A pointer to an initialized admv8818_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADMV8818 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the register, after applying the
 * mask. Only the bits specified by the mask will be updated.
 * @return Returns 0 on success, or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int admv8818_spi_update_bits(struct admv8818_dev *dev, uint16_t reg_addr,
			     uint8_t mask, uint8_t data);

/***************************************************************************//**
 * @brief Use this function to read a specific register from the ADMV8818 device
 * using SPI communication. It is essential to ensure that the device has
 * been properly initialized and that the SPI interface is correctly
 * configured before calling this function. The function reads the value
 * from the specified register address and stores it in the provided data
 * pointer. This function is typically used when you need to retrieve
 * configuration or status information from the device.
 *
 * @param dev A pointer to an initialized admv8818_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The 16-bit address of the register to read from. Valid
 * register addresses are defined by the device's register map.
 * @param data A pointer to a uint8_t variable where the read register value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI read
 * operation fails.
 ******************************************************************************/
int admv8818_spi_read(struct admv8818_dev *dev, uint16_t reg_addr,
		      uint8_t *data);

/***************************************************************************//**
 * @brief This function configures the high-pass filter (HPF) of the ADMV8818
 * device to the closest available frequency band and step based on the
 * provided frequency. It should be called when the HPF frequency needs
 * to be adjusted, typically after initializing the device. The function
 * handles frequencies outside the supported range by setting the HPF to
 * its maximum or minimum configuration. It returns an error code if the
 * SPI communication fails during the update process.
 *
 * @param dev A pointer to an initialized `admv8818_dev` structure representing
 * the device. Must not be null, and the device should be properly
 * initialized before calling this function.
 * @param freq The desired frequency in hertz for the high-pass filter. It
 * should be within the operational range of the device, but the
 * function will handle frequencies outside this range by clamping
 * to the nearest supported configuration.
 * @return Returns 0 on success or a negative error code if the SPI update
 * operation fails.
 ******************************************************************************/
int admv8818_hpf_select(struct admv8818_dev *dev, unsigned long long freq);

/***************************************************************************//**
 * @brief This function is used to obtain the current frequency setting of the
 * high-pass filter (HPF) in the ADMV8818 device. It should be called
 * when the user needs to know the HPF frequency that is currently
 * configured. The function requires a valid device descriptor and a
 * pointer to store the frequency value. It reads the necessary registers
 * to determine the HPF band and state, and calculates the frequency
 * accordingly. If the band is invalid, the frequency is set to zero. The
 * function returns an error code if any SPI read operation fails.
 *
 * @param dev A pointer to an initialized `admv8818_dev` structure representing
 * the device. Must not be null.
 * @param freq A pointer to an unsigned long long where the frequency will be
 * stored. Must not be null. If the function fails, the value
 * pointed to by `freq` is set to zero.
 * @return Returns 0 on success, or a negative error code if a SPI read
 * operation fails.
 ******************************************************************************/
int admv8818_read_hpf_freq(struct admv8818_dev *dev, unsigned long long *freq);

/***************************************************************************//**
 * @brief This function configures the low-pass filter (LPF) of the ADMV8818
 * device to operate at a specified frequency. It should be called when
 * the user needs to adjust the LPF settings based on the desired
 * frequency range. The function determines the appropriate LPF band and
 * step based on the input frequency and updates the device's
 * configuration registers accordingly. It is important to ensure that
 * the device is properly initialized before calling this function. The
 * function handles frequencies outside the supported range by selecting
 * the closest available band.
 *
 * @param dev A pointer to an initialized `admv8818_dev` structure representing
 * the device. Must not be null.
 * @param freq The desired frequency for the LPF in hertz. The function will
 * select the closest available band if the frequency is outside the
 * supported range.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int admv8818_lpf_select(struct admv8818_dev *dev, unsigned long long freq);

/***************************************************************************//**
 * @brief This function is used to obtain the current frequency setting of the
 * low-pass filter (LPF) on the ADMV8818 device. It should be called when
 * the user needs to know the LPF frequency that the device is currently
 * configured to use. The function requires a valid device descriptor and
 * a pointer to store the frequency value. It reads the necessary
 * registers to determine the LPF frequency and writes the result to the
 * provided pointer. If the LPF band is invalid, the frequency is set to
 * zero. The function returns an error code if any SPI read operation
 * fails.
 *
 * @param dev A pointer to an initialized 'admv8818_dev' structure representing
 * the device. Must not be null.
 * @param freq A pointer to an unsigned long long where the frequency will be
 * stored. Must not be null. If the LPF band is invalid, the
 * frequency will be set to zero.
 * @return Returns 0 on success, or a negative error code if a SPI read
 * operation fails.
 ******************************************************************************/
int admv8818_read_lpf_freq(struct admv8818_dev *dev, unsigned long long *freq);

/***************************************************************************//**
 * @brief This function configures the RF input band of the ADMV8818 device by
 * setting both the high-pass filter (HPF) and low-pass filter (LPF)
 * frequencies based on the device's current configuration. It should be
 * called when the RF input band needs to be adjusted, typically after
 * initializing the device or changing the input frequency. The function
 * requires a valid device descriptor and returns an error code if the
 * operation fails.
 *
 * @param dev A pointer to an initialized 'admv8818_dev' structure representing
 * the device. Must not be null. The function will return an error if
 * the device is not properly initialized.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int admv8818_rfin_select(struct admv8818_dev *dev);

/***************************************************************************//**
 * @brief This function initializes an ADMV8818 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function sets up the SPI communication
 * and configures the device according to the specified parameters. If
 * the initialization is successful, a device descriptor is returned
 * through the provided pointer. The function handles errors by cleaning
 * up resources and returning an appropriate error code.
 *
 * @param device A pointer to a pointer of type `struct admv8818_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * The caller is responsible for deallocating the device using
 * `admv8818_remove`.
 * @param init_param A pointer to a `struct admv8818_init_param` containing the
 * initialization parameters. Must not be null. The structure
 * should be properly initialized with valid SPI parameters,
 * RF input frequency, and filter mode before calling this
 * function.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the `device` pointer is set to point to a newly allocated
 * and initialized `admv8818_dev` structure.
 ******************************************************************************/
int admv8818_init(struct admv8818_dev **device,
		  struct admv8818_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADMV8818 device when it is no longer needed. This function should be
 * called to clean up after the device has been initialized and used,
 * ensuring that any associated SPI resources are also released. It is
 * important to call this function to prevent resource leaks in your
 * application.
 *
 * @param dev A pointer to an initialized `admv8818_dev` structure representing
 * the device to be removed. Must not be null. The function will
 * handle invalid pointers by returning an error code.
 * @return Returns 0 on successful removal of the device resources, or a
 * negative error code if the SPI resources could not be released.
 ******************************************************************************/
int admv8818_remove(struct admv8818_dev *dev);

#endif /* ADMV8818_H_ */
