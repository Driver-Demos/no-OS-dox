/***************************************************************************//**
 *   @file   AD9739A.h
 *   @brief  Header file of AD9739A Driver.
 *   @author Bancisor Mihai
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
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
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/
#ifndef __AD9739A_H__
#define __AD9739A_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/***************************** AD9739A ****************************************/
/******************************************************************************/

#define AD9739A_READ				(1 << 7)
#define AD9739A_WRITE				(0 << 7)

/* Registers */
#define AD9739A_REG_MODE			0x00
#define AD9739A_REG_POWER_DOWN			0x01
#define AD9739A_REG_CNT_CLK_DIS			0x02
#define AD9739A_REG_IRQ_EN			0x03
#define AD9739A_REG_IRQ_REQ			0x04
#define AD9739A_REG_FSC_1			0x06
#define AD9739A_REG_FSC_2			0x07
#define AD9739A_REG_DEC_CNT			0x08
#define AD9739A_REG_LVDS_STAT1			0x0C
#define AD9739A_REG_LVDS_REC_CNT1		0x10
#define AD9739A_REG_LVDS_REC_CNT2		0x11
#define AD9739A_REG_LVDS_REC_CNT3		0x12
#define AD9739A_REG_LVDS_REC_CNT4		0x13
#define AD9739A_REG_LVDS_REC_CNT5		0x14
#define AD9739A_REG_LVDS_REC_STAT1		0x19
#define AD9739A_REG_LVDS_REC_STAT2		0x1A
#define AD9739A_REG_LVDS_REC_STAT3		0x1B
#define AD9739A_REG_LVDS_REC_STAT4		0x1C
#define AD9739A_REG_LVDS_REC_STAT9		0x21
#define AD9739A_REG_CROSS_CNT1			0x22
#define AD9739A_REG_CROSS_CNT2			0x23
#define AD9739A_REG_PHS_DET			0x24
#define AD9739A_REG_MU_DUTY			0x25
#define AD9739A_REG_MU_CNT1			0x26
#define AD9739A_REG_MU_CNT2			0x27
#define AD9739A_REG_MU_CNT3			0x28
#define AD9739A_REG_MU_CNT4			0x29
#define AD9739A_REG_MU_STAT1			0x2A
#define AD9739A_REG_PART_ID			0x35
#define AD9739A_CHIP_ID				0x24

/* AD9739A_REG_MODE definitions, address 0x00 */
#define AD9739A_MODE_SDIO_DIR			((1 << 7) | (1 << 0))
#define AD9739A_MODE_LSB			((1 << 6) | (1 << 1))
#define AD9739A_MODE_RESET			((1 << 5) | (1 << 2))

/* AD9739A_REG_POWER_DOWN definitions, address 0x01 */
#define AD9739A_POWER_DOWN_LVDS_DRVR_PD		(1 << 5)
#define AD9739A_POWER_DOWN_LVDS_RCVR_PD		(1 << 4)
#define AD9739A_POWER_DOWN_CLK_RCVR_PD		(1 << 1)
#define AD9739A_POWER_DOWN_DAC_BIAS_PD		(1 << 0)

/* AD9739A_REG_CNT_CLK_DIS definitions, address 0x02 */
#define AD9739A_CNT_CLK_DIS_CLKGEN_PD		(1 << 3)
#define AD9739A_CNT_CLK_DIS_REC_CNT_CLK		(1 << 1)
#define AD9739A_CNT_CLK_DIS_MU_CNT_CLK		(1 << 0)

/* AD9739A_REG_IRQ_EN definitions, address 0x03 */
#define AD9739A_IRQ_EN_MU_LST_EN		(1 << 3)
#define AD9739A_IRQ_EN_MU_LCK_EN		(1 << 2)
#define AD9739A_IRQ_EN_RCV_LST_EN		(1 << 1)
#define AD9739A_IRQ_EN_RCV_LCK_EN		(1 << 0)

/* AD9739A_REG_IRQ_REQ definitions, address 0x04 */
#define AD9739A_IRQ_REQ_MU_LST_IRQ		(1 << 3)
#define AD9739A_IRQ_REQ_MU_LCK_IRQ		(1 << 2)
#define AD9739A_IRQ_REQ_RCV_LST_IRQ		(1 << 1)
#define AD9739A_IRQ_REQ_RCV_LCK_IRQ		(1 << 0)

/* AD9739A_REG_FSC_1 definitions, address 0x06 */
#define AD9739A_FSC_1_FSC_1(x)			(((x) & 0xFF) << 0)

/* AD9739A_REG_FSC_2 definitions, address 0x07 */
#define AD9739A_FSC_2_FSC_2(x)			(((x) & 0x3) << 0)
#define AD9739A_FSC_2_Sleep			(1 << 7)

/* AD9739A_REG_DEC_CNT definitions, address 0x08 */
#define AD9739A_DEC_CNT_DAC_DEC(x)		(((x) & 0x3) << 0)
/* AD9739A_DEC_CNT_DAC_DEC(x) option. */
#define AD9739A_DAC_DEC_NORMAL_BASEBAND		0
/* AD9739A_DEC_CNT_DAC_DEC(x) option. */
#define AD9739A_DAC_DEC_MIX_MODE		2

/* AD9739A_REG_LVDS_STAT1 definitions, address 0x0C */
#define AD9739A_LVDS_STAT1_DCI_PRE_PH0		(1 << 2)
#define AD9739A_LVDS_STAT1_DCI_PST_PH0		(1 << 0)

/* AD9739A_REG_LVDS_REC_CNT1 definitions, address 0x10 */
#define AD9739A_LVDS_REC_CNT1_RCVR_FLG_RST	(1 << 2)
#define AD9739A_LVDS_REC_CNT1_RCVR_LOOP_ON	(1 << 1)
#define AD9739A_LVDS_REC_CNT1_RCVR_CNT_ENA	(1 << 0)

/* AD9739A_REG_LVDS_REC_CNT2 definitions, address 0x11 */
#define AD9739A_LVDS_REC_CNT2_SMP_DEL(x)		(((x) & 0x3) << 6)

/* AD9739A_REG_LVDS_REC_CNT3 definitions, address 0x12 */
#define AD9739A_LVDS_REC_CNT3_SMP_DEL(x)		(((x) & 0xFF) << 0)

/* AD9739A_REG_LVDS_REC_CNT4 definitions, address 0x13 */
#define AD9739A_LVDS_REC_CNT4_DCI_DEL(x)		(((x) & 0xF) << 4)
#define AD9739A_LVDS_REC_CNT4_FINE_DEL_SKEW(x)		(((x) & 0xF) << 0)

/* AD9739A_REG_LVDS_REC_CNT5 definitions, address 0x14 */
#define AD9739A_LVDS_REC_CNT5_DCI_DEL(x)		(((x) & 0x3F) << 0)

/* AD9739A_REG_LVDS_REC_STAT1 definitions, address 0x19 */
#define AD9739A_LVDS_REC_STAT1_SMP_DEL(x)		(((x) & 0x3) << 6)

/* AD9739A_REG_LVDS_REC_STAT2 definitions, address 0x1A */
#define AD9739A_LVDS_REC_STAT2_SMP_DEL(x)		(((x) & 0xFF) << 0)

/* AD9739A_REG_LVDS_REC_STAT3 definitions, address 0x1B */
#define AD9739A_LVDS_REC_STAT3_DCI_DEL(x)		(((x) & 0x3) << 6)

/* AD9739A_REG_LVDS_REC_STAT4 definitions, address 0x1C */
#define AD9739A_LVDS_REC_STAT4_DCI_DEL(x)		(((x) & 0xFF) << 0)

/* AD9739A_REG_LVDS_REC_STAT9 definitions, address 0x21 */
#define AD9739A_LVDS_REC_STAT9_RCVR_TRK_ON		(1 << 3)
#define AD9739A_LVDS_REC_STAT9_RCVR_FE_ON	   	(1 << 2)
#define AD9739A_LVDS_REC_STAT9_RCVR_LST			(1 << 1)
#define AD9739A_LVDS_REC_STAT9_RCVR_LCK			(1 << 0)

/* AD9739A_REG_CROSS_CNT1 definitions, address 0x22 */
#define AD9739A_CROSS_CNT1_DIR_P			(1 << 4)
#define AD9739A_CROSS_CNT1_CLKP_OFFSET(x)		(((x) & 0xF) << 0)

/* AD9739A_REG_CROSS_CNT2 definitions, address 0x23 */
#define AD9739A_CROSS_CNT2_DIR_N			(1 << 4)
#define AD9739A_CROSS_CNT2_CLKN_OFFSET(x)		(((x) & 0xF) << 0)

/* AD9739A_REG_PHS_DET definitions, address 0x24 */
#define AD9739A_PHS_DET_CMP_BST				(1 << 5)
#define AD9739A_PHS_DET_PHS_DET_AUTO_EN			(1 << 4)

/* AD9739A_REG_MU_DUTY definitions, address 0x25 */
#define AD9739A_MU_DUTY_MU_DUTY_AUTO_EN			(1 << 7)

/* AD9739A_REG_MU_CNT1 definitions, address 0x26 */
#define AD9739A_MU_CNT1_SLOPE				(1 << 6)
#define AD9739A_MU_CNT1_MODE(x)				(((x) & 0x3) << 4)
#define AD9739A_MU_CNT1_READ				(1 << 3)
#define AD9739A_MU_CNT1_GAIN(x)				(((x) & 0x3) << 1)
#define AD9739A_MU_CNT1_ENABLE				(1 << 0)

/* AD9739A_REG_MU_CNT2 definitions, address 0x27 */
#define AD9739A_MU_CNT2_MUDEL				(1 << 7)
#define AD9739A_MU_CNT2_SRCH_MODE(x)			((((x) & 0x3) << 5))
#define AD9739A_MU_CNT2_SET_PHS(x)			((((x) & 0x1F) << 0))

/* AD9739A_REG_MU_CNT3 definitions, address 0x28 */
#define AD9739A_MU_CNT3_MUDEL(x)			((((x) & 0xFF) << 0))

/* AD9739A_REG_MU_CNT4 definitions, address 0x29 */
#define AD9739A_MU_CNT4_SEARCH_TOL			(1 << 7)
#define AD9739A_MU_CNT4_RETRY				(1 << 6)
#define AD9739A_MU_CNT4_CONTRST				(1 << 5)
#define AD9739A_MU_CNT4_GUARD(x)			((((x) & 0x1F) << 0))

/* AD9739A_REG_MU_STAT1 definitions, address 0x2A */
#define AD9739A_MU_STAT1_MU_LST				(1 << 1)
#define AD9739A_MU_STAT1_MU_LKD				(1 << 0)

/* AD9739A_REG_PART_ID definitions, address 0x35 */
#define AD9739A_PART_ID_PART_ID(x)			((((x) & 0xFF) << 0))

/******************************************************************************/
/************************ Types Definitions ***********************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad9739a_dev` structure is a simple data structure used to
 * encapsulate the SPI descriptor necessary for communication with the
 * AD9739A device. It serves as a container for the SPI interface,
 * allowing functions to interact with the device through SPI
 * communication. This structure is essential for initializing and
 * managing the SPI connection to the AD9739A, which is a high-speed
 * digital-to-analog converter (DAC).
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for communication.
 ******************************************************************************/
struct ad9739a_dev {
	/* SPI */
	struct no_os_spi_desc *spi_desc;
};

/***************************************************************************//**
 * @brief The `ad9739a_init_param` structure is used to define the
 * initialization parameters for the AD9739A device driver. It includes
 * SPI initialization parameters, offsets for the differential clock
 * signals DACCLK_P and DACCLK_N, and the full-scale current setting for
 * the DAC. This structure is essential for configuring the device to
 * operate correctly with the desired settings.
 *
 * @param spi_init SPI Initialization parameters.
 * @param common_mode_voltage_dacclk_p Magnitude of the offset for the DACCLK_P.
 * @param common_mode_voltage_dacclk_n Magnitude of the offset for the DACCLK_N.
 * @param full_scale_current Full-scale current.
 ******************************************************************************/
struct ad9739a_init_param {
	/** SPI Initialization parameters */
	struct no_os_spi_init_param	spi_init;
	/** magnitude of the offset for the DACCLK_P. */
	uint8_t		common_mode_voltage_dacclk_p;
	/** magnitude of the offset for the DACCLK_N. */
	uint8_t		common_mode_voltage_dacclk_n;
	/** Full-scale current. */
	float		full_scale_current;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to write a specific value to a designated
 * register within the AD9739A device. It is essential for configuring
 * the device's operational parameters. The function should be called
 * when a register needs to be updated with a new value. It requires a
 * valid device structure that has been properly initialized. The
 * function communicates with the device over SPI, and the caller should
 * ensure that the SPI interface is correctly set up before invoking this
 * function. The function returns an integer status code indicating the
 * success or failure of the write operation.
 *
 * @param dev A pointer to an ad9739a_dev structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param register_address The address of the register to which the value will
 * be written. It is a 7-bit value, and the function
 * will mask it to ensure it fits within this range.
 * @param register_value The value to be written to the specified register. It
 * is an 8-bit value.
 * @return Returns an int32_t status code. A value of 0 typically indicates
 * success, while a negative value indicates an error occurred during
 * the write operation.
 ******************************************************************************/
int32_t ad9739a_write(struct ad9739a_dev *dev,
		      uint8_t register_address,
		      uint8_t register_value);
/***************************************************************************//**
 * @brief Use this function to retrieve the current value stored in a specific
 * register of the AD9739A device. It is essential to ensure that the
 * device has been properly initialized and configured before calling
 * this function. The function communicates with the device over SPI to
 * perform the read operation. The caller must provide a valid device
 * structure and a pointer to store the read value. This function returns
 * an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized ad9739a_dev structure representing the
 * device. Must not be null.
 * @param register_address The address of the register to read from. Must be a
 * valid register address as defined by the device.
 * @param register_value A pointer to a uint8_t where the read register value
 * will be stored. Must not be null.
 * @return Returns an int32_t error code indicating the success or failure of
 * the read operation. The read value is stored in the location pointed
 * to by register_value if successful.
 ******************************************************************************/
int32_t ad9739a_read(struct ad9739a_dev *dev,
		     uint8_t register_address,
		     uint8_t *register_value);
/***************************************************************************//**
 * @brief Use this function to reset the AD9739A device to its default SPI
 * register values. This is typically done to ensure the device is in a
 * known state before configuration or after an error condition. The
 * function must be called with a valid device structure that has been
 * properly initialized. It performs a software reset by writing to the
 * device's mode register and then clears the reset bit. If the reset
 * operation fails, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad9739a_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if communication with the
 * device fails.
 * @return Returns 0 on success or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int32_t ad9739a_reset(struct ad9739a_dev *dev);
/***************************************************************************//**
 * @brief This function is used to control the power state of the AD9739A device
 * by writing a power configuration to the power down register. It should
 * be called when there is a need to change the power settings of the
 * device, such as powering down certain components to save energy. If
 * the provided power configuration is invalid, the function will instead
 * return the current power down register value. This function requires a
 * valid device structure and should be used after the device has been
 * properly initialized.
 *
 * @param dev A pointer to an ad9739a_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param pwr_config A uint8_t value representing the power configuration to be
 * written to the device. Valid configurations must not set
 * bits 7, 6, 3, or 2. If these bits are set, the function
 * will return the current power down register value instead
 * of writing.
 * @return Returns an int32_t value. If a valid power configuration is provided,
 * it returns the result of the write operation. If the configuration is
 * invalid, it returns the current power down register value.
 ******************************************************************************/
int32_t ad9739a_power_down(struct ad9739a_dev *dev,
			   uint8_t pwr_config);
/*! Sets the normal baseband mode or mix-mode. */
int32_t ad9739a_operation_mode(struct ad9739a_dev *dev,
			       uint8_t mode);
/***************************************************************************//**
 * @brief This function configures the full-scale output current of the AD9739A
 * DAC based on the provided current value. It should be used when there
 * is a need to adjust the DAC's output current to a specific level
 * within the valid range. The function can also be used to put the DAC
 * into a sleep mode by setting the current value to zero. It is
 * important to ensure that the device is properly initialized before
 * calling this function. The function handles values outside the valid
 * range by returning the current full-scale setting without making
 * changes.
 *
 * @param dev A pointer to an initialized ad9739a_dev structure representing the
 * device. Must not be null.
 * @param fs_val The desired full-scale current value in mA. Valid range is 8.7
 * to 31.7 mA, or 0 to put the DAC into sleep mode. Values outside
 * this range will result in the function returning the current
 * full-scale setting without modification.
 * @return Returns the actual full-scale current set, or a negative error code
 * if a write operation fails.
 ******************************************************************************/
float ad9739a_dac_fs_current(struct ad9739a_dev *dev,
			     float fs_val);
/***************************************************************************//**
 * @brief This function introduces a delay in the program execution based on the
 * number of DAC clock cycles specified by the user. It is useful in
 * timing-sensitive applications where precise delays are required. The
 * function calculates the delay in microseconds and adjusts it slightly
 * to ensure accuracy. It should be used when a delay corresponding to a
 * specific number of DAC cycles is needed, and it assumes that the FDATA
 * constant is defined and represents the DAC clock frequency.
 *
 * @param cycles The number of DAC clock cycles to delay. It must be a non-
 * negative integer. The function does not handle invalid values
 * explicitly, so the caller must ensure the input is valid.
 * @return Returns 0 after the delay is completed. The function does not modify
 * any input parameters or produce any other output.
 ******************************************************************************/
int32_t delay_fdata_cycles(uint32_t cycles);
/***************************************************************************//**
 * @brief This function sets up the AD9739A device by initializing it with the
 * provided parameters and configuring it for operation. It must be
 * called before any other operations on the device to ensure proper
 * setup. The function allocates memory for the device structure and
 * initializes the SPI interface. It also verifies the device ID to
 * ensure the correct device is being configured. If the initialization
 * is successful, the device is ready for use; otherwise, an error code
 * is returned. The caller is responsible for freeing the allocated
 * resources using `ad9739a_remove` when the device is no longer needed.
 *
 * @param device A pointer to a pointer of type `struct ad9739a_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `ad9739a_init_param` containing
 * initialization parameters such as SPI settings, common mode
 * voltages, and full-scale current. All fields must be
 * properly set before calling the function.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * `device` is set to point to the initialized device structure.
 ******************************************************************************/
int32_t ad9739a_setup(struct ad9739a_dev **device,
		      struct ad9739a_init_param init_param);
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD9739A
 * device instance when it is no longer needed. This function should be
 * called after the device is no longer in use to ensure proper cleanup
 * and to prevent resource leaks. It is important to ensure that the
 * device pointer provided is valid and was previously initialized using
 * the appropriate setup function.
 *
 * @param dev A pointer to an ad9739a_dev structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid device instance that was previously initialized.
 * @return Returns an integer status code from the underlying SPI removal
 * operation, where 0 typically indicates success and a negative value
 * indicates an error.
 ******************************************************************************/
int32_t ad9739a_remove(struct ad9739a_dev *dev);

#endif /* __AD9739A_H__ */
