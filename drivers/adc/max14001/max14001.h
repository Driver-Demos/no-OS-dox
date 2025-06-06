/***************************************************************************//**
 *   @file   max14001.h
 *   @brief  Implementation of max14001.h
 *   @author NAlteza (nathaniel.alteza@analog.com)
 *******************************************************************************
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
 ******************************************************************************/
#ifndef __MAX14001_H__
#define __MAX14001_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define MAX14001_REG_READ(x)		(((x) << 11) & NO_OS_GENMASK(15, 11))
#define MAX14001_REG_WRITE(x,y)		(MAX14001_REG_READ(x) | \
					NO_OS_BIT(10) | \
					(y & MAX14001_REG_DATA_MASK))
#define MAX14001_REG_DATA_MASK		NO_OS_GENMASK(9, 0)
#define VERIFICATION_REG(x)		((x) + 0x10)
#define MAX14001_SPI_REG_WRITE_ENABLE	0x294
#define MAX14001_SPI_REG_WRITE_DISABLE	0x000

/*
 * MAX14001 registers definition
 */
#define MAX14001_ADC_REG        	0x00
#define MAX14001_FADC_REG       	0x01
#define MAX14001_FLAGS_REG      	0x02
#define MAX14001_FLTEN_REG      	0x03
#define MAX14001_THL_REG        	0x04
#define MAX14001_THU_REG        	0x05
#define MAX14001_INRR_REG       	0x06
#define MAX14001_INRT_REG       	0x07
#define MAX14001_INRP_REG       	0x08
#define MAX14001_CFG_REG        	0x09
#define MAX14001_ENBL_REG       	0x0A
#define MAX14001_ACT_REG        	0x0B
#define MAX14001_WEN_REG        	0x0C
#define MAX14001_FLTV_REG       	0x13
#define MAX14001_THLV_REG       	0x14
#define MAX14001_THUV_REG       	0x15
#define MAX14001_INRRV_REG      	0x16
#define MAX14001_INRTV_REG      	0x17
#define MAX14001_INRPV_REG      	0x18
#define MAX14001_CFGV_REG       	0x19
#define MAX14001_ENBLV_REG      	0x1A

/*
 * MAX14001_ACT_REG
 */

/** Reset. Has the same effect as a power on reset. */
#define MAX14001_RSET_MASK		NO_OS_BIT(7)

/** Software reset. Restores all registers to their POR value. */
#define MAX14001_SRES_MASK              NO_OS_BIT(6)

/** Trigger an inrush current pulse. Has no effect when ENA = 0. */
#define MAX14001_INPLS_MASK             NO_OS_BIT(9)

/*
 * MAX14001_FLAGS_REG
 */
#define MAX14001_ADC_FLAG_MASK          NO_OS_BIT(1)
#define MAX14001_INRD_FLAG_MASK         NO_OS_BIT(2)
#define MAX14001_SPI_FLAG_MASK          NO_OS_BIT(3)
#define MAX14001_COM_FLAG_MASK          NO_OS_BIT(4)
#define MAX14001_CRCL_FLAG_MASK         NO_OS_BIT(5)
#define MAX14001_CRCF_FLAG_MASK         NO_OS_BIT(6)
#define MAX14001_FET_FLAG_MASK          NO_OS_BIT(7)
#define MAX14001_MV_FLAG_MASK           NO_OS_BIT(8)

/*
 * MAX14001_FLTEN_REG
 */
#define MAX14001_DYEN_FLTEN_MASK        NO_OS_BIT(0)
#define MAX14001_EADC_FLTEN_MASK        NO_OS_BIT(1)
#define MAX14001_EINRD_FLTEN_MASK       NO_OS_BIT(2)
#define MAX14001_ESPI_FLTEN_MASK        NO_OS_BIT(3)
#define MAX14001_ECOM_FLTEN_MASK        NO_OS_BIT(4)
#define MAX14001_ECRCL_FLTEN_MASK       NO_OS_BIT(5)
#define MAX14001_ECRCF_FLTEN_MASK       NO_OS_BIT(6)
#define MAX14001_EFET_FLTEN_MASK        NO_OS_BIT(7)
#define MAX14001_EMV_FLTEN_MASK         NO_OS_BIT(8)

/*
 * MAX14001_INRP_REG
 */
#define MAX14001_IINR_INRP_MASK         NO_OS_GENMASK(9, 6)
#define MAX14001_IINR_INRP_MODE(a)      ((MAX14001_IINR(a)) << 6)
#define MAX14001_TINR_INRP_MASK         NO_OS_GENMASK(5, 2)
#define MAX14001_TINR_INRP_MODE(a)	((MAX14001_TINR(a)) << 2)
#define MAX14001_DU_INRP_MASK           NO_OS_GENMASK(1, 0)
#define MAX14001_DU_INRP_MODE(a)  	(((a) & 0x3) << 0)

/*
 * MAX14001_CFG_REG
 */
#define MAX14001_IBIAS_CFG_MASK         NO_OS_GENMASK(9, 6)
#define MAX14001_IBIAS_CFG_MODE(a)	((MAX14001_IBIAS(a)) << 6)
#define MAX14001_EXRF_CFG_MASK          NO_OS_BIT(5)
#define MAX14001_EXTI_CFG_MASK          NO_OS_BIT(4)
#define MAX14001_FT_CFG_MASK            NO_OS_GENMASK(3, 2)
#define MAX14001_FT_CFG_MODE(a)		(((a) & 0x3) << 2)
#define MAX14001_FAST_CFG_MASK          NO_OS_BIT(1)
#define MAX14001_IRAW_CFG_MASK          NO_OS_BIT(0)

/*
 * MAX14001_ENBL_REG
 */
#define MAX14001_ENA_ENBL_MASK          NO_OS_BIT(4)

/*
 * 16-bit bitswap
 */
#define REVERSE_UINT16(x)		((((x >> 0) & 1) << 15) | \
					(((x >> 1) & 1 ) << 14) | \
					(((x >> 2) & 1 ) << 13) | \
					(((x >> 3) & 1 ) << 12) | \
					(((x >> 4) & 1 ) << 11) | \
					(((x >> 5) & 1 ) << 10) | \
					(((x >> 6) & 1 ) << 9) | \
					(((x >> 7) & 1 ) << 8) | \
					(((x >> 8) & 1 ) << 7) | \
					(((x >> 9) & 1 ) << 6) | \
					(((x >> 10) & 1) << 5) | \
					(((x >> 11) & 1) << 4) | \
					(((x >> 12) & 1) << 3) | \
					(((x >> 13) & 1) << 2) | \
					(((x >> 14) & 1) << 1) | \
					(((x >> 15) & 1) << 0))

/*
 * MAX14001 Quantization of configuration inputs
 */
#define MAX14001_CFG_MIN              	0
#define MAX14001_CFG_MAX              	0xF
#define MAX14001_CFG_Q(x,y)		(no_os_clamp( \
					NO_OS_DIV_ROUND_CLOSEST(x,y), \
                    			(MAX14001_CFG_MIN),(MAX14001_CFG_MAX)))
/*
 * MAX14001 TINR input to bits equivelent
 */
#define MAX14001_TINR_INC		8	//in terms of ms
#define MAX14001_TINR(x)		MAX14001_CFG_Q(x, MAX14001_TINR_INC)

/*
 * MAX14001 IINR input to bits equivelent
 */
#define MAX14001_IINR_INC		7	//in terms of mA
#define MAX14001_IINR(x)		MAX14001_CFG_Q(x, MAX14001_IINR_INC)

/*
 * MAX14001 IBIAS input to bits equivelent
 */
#define MAX14001_MUL			100
#define MAX14001_IBIAS_INC		25	//increment of 0.25mA
#define MAX14001_IBIAS(x)		MAX14001_CFG_Q((int)(x*MAX14001_MUL), \
					MAX14001_IBIAS_INC)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `max14001_du` enumeration defines a set of constants representing
 * different duty cycle modes for the MAX14001 device. These modes are
 * used to configure the maximum duty cycle for inrush current over a
 * specified period, with options ranging from turning the function off
 * to setting specific percentage limits. This allows for precise control
 * over the duty cycle behavior of the device.
 *
 * @param DUTY_OFF Represents the default state where the duty cycle limiting
 * function is off.
 * @param DUTY_1P6 Represents a duty cycle mode with a 1.6% limit.
 * @param DUTY_3P1 Represents a duty cycle mode with a 3.1% limit.
 * @param DUTY_6P3 Represents a duty cycle mode with a 6.3% limit.
 ******************************************************************************/
enum max14001_du {
	DUTY_OFF,	//default, duty cycle limiting function off
	DUTY_1P6,
	DUTY_3P1,
	DUTY_6P3,
};

/***************************************************************************//**
 * @brief The `max14001_ft` enumeration defines the possible filtering modes for
 * the MAX14001 device, allowing the user to select between no filtering
 * or averaging a specified number of readings (2, 4, or 8) to smooth out
 * the ADC data.
 *
 * @param FILTER_OFF Default mode where filtering is off.
 * @param AVERAGE_2_READINGS Mode to average 2 readings for filtering.
 * @param AVERAGE_4_READINGS Mode to average 4 readings for filtering.
 * @param AVERAGE_8_READINGS Mode to average 8 readings for filtering.
 ******************************************************************************/
enum max14001_ft {
	FILTER_OFF,	//default, filtering is off
	AVERAGE_2_READINGS,
	AVERAGE_4_READINGS,
	AVERAGE_8_READINGS,
};

/***************************************************************************//**
 * @brief The `max14001_dev` structure is designed to encapsulate the necessary
 * components for interfacing with a MAX14001 device via SPI
 * communication. It contains a single member, `spi_desc`, which is a
 * pointer to a `no_os_spi_desc` structure, providing the necessary SPI
 * layer handler for communication with the device. This structure is
 * essential for managing the SPI interactions required to configure and
 * operate the MAX14001 device.
 *
 * @param spi_desc A pointer to a SPI descriptor used for handling SPI
 * communication.
 ******************************************************************************/
struct max14001_dev {
	/** SPI layer handler*/
	struct no_os_spi_desc   	*spi_desc;
};

/***************************************************************************//**
 * @brief The `max14001_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up a MAX14001 device,
 * specifically focusing on the SPI communication interface. It contains
 * a single member, `spi_init`, which is a structure that holds the
 * necessary SPI initialization parameters, ensuring that the device can
 * be properly configured and communicated with over an SPI bus.
 *
 * @param spi_init This member is a structure of type no_os_spi_init_param,
 * which holds the initialization parameters for an SPI device.
 ******************************************************************************/
struct max14001_init_param {
	/** SPI device descriptor*/
	struct no_os_spi_init_param     spi_init;
};

/***************************************************************************//**
 * @brief This function reads a 16-bit value from a specified register of the
 * MAX14001 device. It should be called when you need to retrieve data
 * from a specific register. The function requires a valid device
 * structure and a register address to read from. It is important to
 * ensure that the register address is within the valid range and not one
 * of the restricted addresses, as invalid addresses will result in an
 * error. The function will store the read value in the provided pointer
 * if successful.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address within the allowed range and not a
 * restricted address. Invalid addresses will result in an
 * error.
 * @param reg_data A pointer to a uint16_t where the read register data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the register
 * address is invalid or if there is a communication error.
 ******************************************************************************/
int max14001_read(struct max14001_dev *dev,
		  uint8_t reg_addr,
		  uint16_t *reg_data);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specific
 * register of the MAX14001 device via SPI communication. It should be
 * called when there is a need to configure or update the settings of the
 * device. The function requires a valid device structure and a register
 * address within the permissible range. If the register address is
 * invalid, the function returns an error code. It is important to ensure
 * that the device is properly initialized before calling this function.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param reg_addr The address of the register to write to. Must be within the
 * valid range of register addresses defined for the MAX14001
 * device. Invalid addresses result in an error.
 * @param reg_data The 16-bit data to be written to the specified register. The
 * data is processed and sent to the device.
 * @return Returns 0 on success or a negative error code if the register address
 * is invalid or if the SPI communication fails.
 ******************************************************************************/
int max14001_write(struct max14001_dev *dev,
		   uint8_t reg_addr,
		   uint16_t reg_data);

/***************************************************************************//**
 * @brief This function is used to update a specific register on the MAX14001
 * device by applying a mask and new data to the current register value.
 * It should be called when you need to modify specific bits of a
 * register without affecting other bits. The function reads the current
 * value of the register, applies the mask to clear specific bits, and
 * then sets the new data. It is important to ensure that the register
 * address is within the valid range for writable registers; otherwise,
 * the function will return an error. This function requires the device
 * to be properly initialized before use.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null, and the device must be initialized.
 * @param reg_addr The address of the register to update. Must be within the
 * valid range of writable registers; otherwise, the function
 * returns an error.
 * @param mask A 16-bit mask indicating which bits of the register should be
 * cleared before writing the new data.
 * @param data A 16-bit value containing the new data to be written to the
 * register after applying the mask.
 * @return Returns 0 on success, or a negative error code if the register
 * address is invalid or if there is a failure in reading or writing the
 * register.
 ******************************************************************************/
int max14001_reg_update(struct max14001_dev *dev,
			uint8_t reg_addr,
			uint16_t mask,
			uint16_t data);

/***************************************************************************//**
 * @brief This function is used to write data to a specified configuration
 * register of the MAX14001 device and verify the write operation by
 * checking the corresponding verification register. It is essential to
 * ensure that the device is properly initialized before calling this
 * function. The function performs a read-modify-write operation using a
 * mask to update specific bits in the register. It is important to
 * provide a valid register address within the allowed range; otherwise,
 * the function will return an error. This function is useful when you
 * need to ensure that the configuration changes have been successfully
 * applied to the device.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null, and the device must be initialized before use.
 * @param reg_addr The address of the register to be written to. Must be within
 * the valid range of configuration registers; otherwise, the
 * function returns an error.
 * @param mask A 16-bit mask used to specify which bits in the register should
 * be modified. The mask is applied to clear specific bits before
 * writing the new data.
 * @param data The 16-bit data to be written to the register. The data is
 * combined with the existing register value using the mask to
 * update specific bits.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as when an invalid register address is provided.
 ******************************************************************************/
int max14001_write_config_verify(struct max14001_dev *dev,
				 uint8_t reg_addr,
				 uint16_t mask,
				 uint16_t data);

/***************************************************************************//**
 * @brief This function sets up a MAX14001 device by allocating necessary
 * resources and initializing it with the provided parameters. It must be
 * called before any other operations on the device to ensure proper
 * setup. The function performs a delay to allow for any necessary
 * hardware stabilization, allocates memory for the device structure, and
 * initializes the SPI interface using the provided initialization
 * parameters. If any step fails, the function ensures that allocated
 * resources are freed and returns an error code. This function is
 * essential for preparing the device for further configuration and
 * operation.
 *
 * @param device A double pointer to a max14001_dev structure where the
 * initialized device instance will be stored. Must not be null.
 * The caller takes ownership of the allocated memory and is
 * responsible for freeing it using max14001_remove().
 * @param init_param A structure containing initialization parameters for the
 * device, including SPI configuration. Must be properly
 * initialized before calling this function.
 * @return Returns 0 on success, or a negative error code if initialization
 * fails. On success, the device pointer is updated to point to the
 * newly allocated and initialized device structure.
 ******************************************************************************/
int max14001_init(struct max14001_dev **device,
		  struct max14001_init_param init_param);

/***************************************************************************//**
 * @brief This function sets up the initial configuration for a MAX14001 device.
 * It should be called after the device has been initialized but before
 * it is used for any operations. The function performs a series of
 * register reads and writes to configure the device, enabling and
 * disabling write access as needed. It handles any errors encountered
 * during these operations by returning immediately with the error code.
 * This function is essential for preparing the device for normal
 * operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device to
 * be configured. Must not be null. The caller retains ownership of
 * the memory.
 * @return Returns 0 on success or a negative error code if any of the
 * configuration steps fail.
 ******************************************************************************/
int max14001_init_config(struct max14001_dev *dev);

/***************************************************************************//**
 * @brief This function is used to control the write access to the registers of
 * a MAX14001 device. It should be called when there is a need to enable
 * or disable the ability to write to the device's registers, typically
 * during configuration changes. The function requires a valid device
 * structure and a boolean flag indicating whether to enable or disable
 * write access. It returns an integer status code indicating success or
 * failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param enable A boolean value where 'true' enables write access and 'false'
 * disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_wen(struct max14001_dev *dev, bool write_enable);

/***************************************************************************//**
 * @brief This function is used to perform a full reset on the MAX14001 device,
 * which has the same effect as a power-on reset. It should be called
 * when a complete reinitialization of the device is required, such as
 * after a configuration change or to recover from an error state. The
 * function requires a valid device structure that has been properly
 * initialized. It returns an integer status code indicating the success
 * or failure of the reset operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device to
 * be reset. This must not be null and should be initialized before
 * calling this function. If the pointer is invalid, the behavior is
 * undefined.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int max14001_full_reset(struct max14001_dev *dev);

/***************************************************************************//**
 * @brief This function is used to perform a software reset on the MAX14001
 * device, restoring all its registers to their power-on reset (POR)
 * values. It should be called when a reset of the device's configuration
 * is required without cycling power. The function requires a valid
 * device structure that has been initialized. It is important to ensure
 * that the device is in a state where a reset will not disrupt ongoing
 * operations.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure. The function
 * will not perform any action if this pointer is null or invalid.
 * @return Returns 0 on success or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int max14001_reg_reset(struct max14001_dev *dev);

/***************************************************************************//**
 * @brief This function is used to trigger an inrush current pulse on a MAX14001
 * device, which is typically used in applications where a sudden
 * increase in current is required. It should be called when the device
 * is properly initialized and configured, and when the inrush current
 * pulse is needed. The function requires a valid device structure and
 * will return an error code if the operation fails.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null. The caller retains ownership and is responsible
 * for ensuring the device is initialized before calling this
 * function.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int max14001_inpls_reset(struct max14001_dev *dev);

/***************************************************************************//**
 * @brief This function is used to configure the fast inrush mode of a MAX14001
 * device. It should be called when there is a need to toggle the fast
 * inrush mode, which can be useful in applications requiring rapid
 * current changes. The function requires a valid device structure and a
 * boolean indicating whether to enable or disable the mode. It returns
 * an integer status code indicating success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param fast A boolean value where 'true' enables fast inrush mode and 'false'
 * disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_fast_config(struct max14001_dev *dev,
			 bool fast);

/***************************************************************************//**
 * @brief This function is used to configure the inrush comparator input
 * multiplexer of the MAX14001 device. It should be called when there is
 * a need to switch between raw data input and processed data input for
 * the inrush comparator. The function must be called with a valid device
 * structure that has been properly initialized. It modifies the
 * configuration register of the device to either enable or disable the
 * raw data input based on the boolean parameter provided.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param raw_data A boolean value indicating whether to enable (true) or
 * disable (false) the raw data input for the inrush comparator.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int max14001_iraw_config(struct max14001_dev *dev,
			 bool raw_data);

/***************************************************************************//**
 * @brief This function is used to configure the MV fault detection feature of
 * the MAX14001 device. It should be called when you need to enable or
 * disable the MV fault detection, which is part of the device's fault
 * management system. The function requires a valid device structure that
 * has been properly initialized. It returns an integer status code
 * indicating success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param mode A boolean value where 'true' enables the MV fault detection and
 * 'false' disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_emv_config(struct max14001_dev *dev,
			bool mode);

/***************************************************************************//**
 * @brief This function is used to configure the FET fault detection feature of
 * the MAX14001 device. It should be called when you need to enable or
 * disable the FET fault detection, which is part of the device's fault
 * management system. The function requires a valid device structure that
 * has been properly initialized. It modifies the configuration register
 * associated with FET fault detection based on the mode parameter. This
 * function returns an integer status code indicating success or failure
 * of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param mode A boolean value where 'true' enables the FET fault detection and
 * 'false' disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_efet_config(struct max14001_dev *dev,
			 bool mode);

/***************************************************************************//**
 * @brief This function is used to configure the CRCF fault detection mode on a
 * MAX14001 device. It should be called when you need to enable or
 * disable the CRCF fault detection feature, which is part of the
 * device's fault management system. The function requires a valid device
 * structure that has been properly initialized. It returns an integer
 * status code indicating success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param mode A boolean value where 'true' enables the CRCF fault detection and
 * 'false' disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_ecrcf_config(struct max14001_dev *dev,
			  bool mode);

/***************************************************************************//**
 * @brief This function is used to configure the CRCL fault detection feature on
 * a MAX14001 device. It should be called when you need to enable or
 * disable the CRCL fault detection, which is part of the device's fault
 * management system. The function requires a valid device structure that
 * has been properly initialized. It returns an integer status code
 * indicating success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param mode A boolean value where 'true' enables the CRCL fault detection and
 * 'false' disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_ecrcl_config(struct max14001_dev *dev,
			  bool mode);

/***************************************************************************//**
 * @brief This function is used to enable or disable the ECOM fault detection
 * mode on a MAX14001 device. It should be called when there is a need to
 * configure the fault detection settings related to communication
 * errors. The function requires a valid device structure that has been
 * properly initialized. It modifies the device's configuration register
 * to reflect the desired mode. This function is typically used in
 * scenarios where fault detection is critical for maintaining
 * communication integrity.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param mode A boolean value indicating whether to enable (true) or disable
 * (false) the ECOM fault detection mode.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int max14001_ecom_config(struct max14001_dev *dev,
			 bool mode);

/***************************************************************************//**
 * @brief This function is used to configure the SPI fault detection feature on
 * a MAX14001 device. It should be called when you need to enable or
 * disable the SPI fault detection, which is part of the device's fault
 * management capabilities. The function requires a valid device
 * structure that has been properly initialized. It modifies the device's
 * configuration register to reflect the desired state of the SPI fault
 * detection. This function returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @param mode A boolean value indicating whether to enable (true) or disable
 * (false) the SPI fault detection feature.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_espi_config(struct max14001_dev *dev,
			 bool mode);

/***************************************************************************//**
 * @brief This function is used to configure the INRD fault detection mode on a
 * MAX14001 device. It should be called when you need to enable or
 * disable the INRD fault detection feature, which is part of the
 * device's fault management system. Ensure that the device has been
 * properly initialized before calling this function. The function
 * modifies the device's configuration register to reflect the desired
 * mode.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param mode A boolean value indicating whether to enable (true) or disable
 * (false) the INRD fault detection.
 * @return Returns an integer status code. A value of 0 typically indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int max14001_einrd_config(struct max14001_dev *dev,
			  bool mode);

/***************************************************************************//**
 * @brief This function is used to configure the ADC fault detection feature on
 * a MAX14001 device. It should be called when you need to enable or
 * disable the ADC fault detection, which is part of the device's fault
 * management system. The function requires a valid device structure that
 * has been properly initialized. It returns an integer status code
 * indicating success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param mode A boolean value where 'true' enables ADC fault detection and
 * 'false' disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_eadc_config(struct max14001_dev *dev,
			 bool mode);

/***************************************************************************//**
 * @brief This function is used to configure the dynamic FAULT signal on a
 * MAX14001 device, which can be enabled or disabled based on the
 * provided mode. It should be called when there is a need to control the
 * dynamic FAULT signal behavior, typically during device setup or
 * configuration changes. The function requires a valid device structure
 * and a boolean mode to specify the desired state of the dynamic FAULT
 * signal. It returns an integer status code indicating success or
 * failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param mode A boolean value where true enables the dynamic FAULT signal and
 * false disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_dyen_config(struct max14001_dev *dev,
			 bool mode);

/***************************************************************************//**
 * @brief This function is used to control the field-side current sink of the
 * MAX14001 device. It should be called when there is a need to enable or
 * disable this feature, typically as part of the device configuration
 * process. The function requires a valid device structure that has been
 * properly initialized. It returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the field-side current sink.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_ena_config(struct max14001_dev *dev,
			bool enable);

/***************************************************************************//**
 * @brief This function configures the voltage reference source for the ADC in
 * the MAX14001 device. It should be called when you need to switch
 * between different voltage reference modes. The function requires a
 * valid device structure and a boolean mode to determine the reference
 * source. Ensure the device is properly initialized before calling this
 * function. The function returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null, and the device must be initialized before use.
 * @param mode A boolean value where 'true' enables the external reference mode
 * and 'false' disables it. The function will configure the ADC
 * reference based on this mode.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_exrf_config(struct max14001_dev *dev,
			 bool mode);

/***************************************************************************//**
 * @brief This function is used to enable or disable the external current source
 * connection on a MAX14001 device. It should be called when there is a
 * need to control the current source connection based on the
 * application's requirements. The function requires a valid device
 * structure that has been properly initialized. It modifies the
 * configuration register of the device to reflect the desired state of
 * the external current source connection. The function returns an
 * integer status code indicating success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param mode A boolean value where 'true' enables the external current source
 * connection and 'false' disables it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_exti_config(struct max14001_dev *dev,
			 bool mode);

/***************************************************************************//**
 * @brief This function sets the inrush time configuration for a MAX14001
 * device. It should be used when you need to adjust the inrush time
 * settings of the device, typically during initialization or when
 * changing operational parameters. The function requires a valid device
 * structure and a mode value representing the desired inrush time
 * configuration. It is important to ensure that the device has been
 * properly initialized before calling this function. The function will
 * return an error code if the configuration fails.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * This must not be null and should be properly initialized before
 * calling this function. The caller retains ownership.
 * @param mode An integer representing the inrush time configuration mode. The
 * valid range for this parameter is determined by the device's
 * specifications. Invalid values may result in an error return.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int max14001_tinr_config(struct max14001_dev *dev,
			 int mode);

/***************************************************************************//**
 * @brief This function sets the inrush current mode for a MAX14001 device by
 * writing to the appropriate configuration register. It should be called
 * when you need to adjust the inrush current settings of the device.
 * Ensure that the device has been properly initialized before calling
 * this function. The function returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param mode An integer representing the desired inrush current mode. The
 * valid range and specific values depend on the device's
 * configuration requirements.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_iinr_config(struct max14001_dev *dev,
			 int mode);

/***************************************************************************//**
 * @brief This function configures the maximum duty cycle for the inrush current
 * over a 10-second period for a MAX14001 device. It should be called
 * when you need to adjust the duty cycle limiting function of the
 * device. The function requires a valid device structure and a duty
 * cycle mode from the predefined enumeration. It returns an error if the
 * mode is outside the valid range.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param mode An enum max14001_du value representing the desired duty cycle
 * mode. Valid values are DUTY_OFF, DUTY_1P6, DUTY_3P1, and
 * DUTY_6P3. If the mode is outside this range, the function returns
 * an error.
 * @return Returns 0 on success or a negative error code if the mode is invalid.
 ******************************************************************************/
int max14001_du_config(struct max14001_dev *dev,
		       enum max14001_du mode);

/***************************************************************************//**
 * @brief This function sets the bias current configuration for a MAX14001
 * device. It should be called when you need to adjust the bias current
 * settings of the device. The function requires a valid device structure
 * and a mode value representing the desired bias current configuration.
 * Ensure that the device has been properly initialized before calling
 * this function. The function returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param mode A float representing the desired bias current configuration. The
 * value should be within the valid range for the device's bias
 * current settings.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max14001_ibias_config(struct max14001_dev *dev,
			  float mode);

/***************************************************************************//**
 * @brief Use this function to configure the filtering mode of the MAX14001
 * device, which determines how many readings are averaged in the ADC
 * filter. This function should be called after the device has been
 * initialized. It validates the mode parameter to ensure it is within
 * the acceptable range of filtering modes. If the mode is invalid, the
 * function returns an error code. This function does not modify the
 * device state if an invalid mode is provided.
 *
 * @param dev A pointer to a max14001_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function. The caller retains ownership.
 * @param mode An enum max14001_ft value representing the desired filtering
 * mode. Valid values are FILTER_OFF, AVERAGE_2_READINGS,
 * AVERAGE_4_READINGS, and AVERAGE_8_READINGS. If the mode is
 * outside this range, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the mode is
 * invalid.
 ******************************************************************************/
int max14001_ft_config(struct max14001_dev *dev,
		       enum max14001_ft mode);

/***************************************************************************//**
 * @brief This function is used to obtain raw ADC data from a MAX14001 device.
 * It should be called when raw, unprocessed ADC readings are required
 * from the device. The function requires a valid device structure, which
 * must be initialized prior to calling this function. If the device
 * structure is null, the function will return an error. The retrieved
 * data is stored in the location pointed to by the data parameter.
 *
 * @param dev A pointer to an initialized max14001_dev structure representing
 * the device. Must not be null. If null, the function returns an
 * error.
 * @param data A pointer to a uint16_t variable where the raw ADC data will be
 * stored. The caller must ensure this pointer is valid and points
 * to a writable memory location.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is an error in reading the data.
 ******************************************************************************/
int max14001_get_data_raw(struct max14001_dev *dev, uint16_t *data);

/***************************************************************************//**
 * @brief This function is used to obtain filtered ADC data from a MAX14001
 * device. It should be called when filtered data is required for further
 * processing or analysis. The function requires a valid device
 * structure, which must be initialized prior to calling this function.
 * If the device structure is null, the function will return an error.
 * The filtered data is stored in the provided data pointer, which must
 * not be null.
 *
 * @param dev A pointer to an initialized max14001_dev structure representing
 * the device. Must not be null. If null, the function returns an
 * error.
 * @param data A pointer to a uint16_t variable where the filtered ADC data will
 * be stored. Must not be null. The caller retains ownership of the
 * memory.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is an error in reading the data.
 ******************************************************************************/
int max14001_get_data_filtered(struct max14001_dev *dev, uint16_t *data);

/***************************************************************************//**
 * @brief This function is used to release all resources associated with a
 * MAX14001 device that were previously allocated during initialization.
 * It should be called when the device is no longer needed to ensure
 * proper cleanup and to prevent resource leaks. The function must be
 * called with a valid device pointer that was successfully initialized.
 * If the provided device pointer is null, the function will return an
 * error code indicating invalid input. After successful execution, the
 * device pointer should not be used unless reinitialized.
 *
 * @param dev A pointer to a max14001_dev structure representing the device to
 * be removed. Must not be null. If null, the function returns
 * -EINVAL. The caller retains ownership of the pointer, but it
 * should not be used after this function is called unless
 * reinitialized.
 * @return Returns 0 on successful removal of the device resources. If the
 * device pointer is null, returns -EINVAL. If an error occurs during
 * the SPI removal process, the function returns the corresponding error
 * code.
 ******************************************************************************/
int max14001_remove(struct max14001_dev *dev);

#endif /* __MAX14001_H__ */
