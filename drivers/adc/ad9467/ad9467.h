/***************************************************************************//**
 *   @file   AD9467.h
 *   @brief  Header file of AD9467 Driver.
 *   @author DNechita (Dan.Nechita@analog.com)
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
*******************************************************************************/
#ifndef __AD9467_H__
#define __AD9467_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/*********************************** AD9467 ***********************************/
/******************************************************************************/
#define AD9467_READ                         (1 << 15)
#define AD9467_WRITE                        (0 << 15)
#define AD9467_CNT(x)                       (((x) - 1) << 13)
#define AD9467_ADDR(x)                      ((x) & 0X1FFF)

/* Chip configuration registers */
#define AD9467_REG_CHIP_PORT_CFG            0x00
#define AD9467_REG_CHIP_ID                  0x01
#define AD9467_REG_CHIP_GRADE               0x02

/* Channel index and transfer registers */
#define AD9467_REG_DEVICE_UPDATE            0xFF

/* ADC functions registers */
#define AD9467_REG_MODES                    0x08
#define AD9467_REG_TEST_IO                  0x0D
#define AD9467_REG_ADC_INPUT                0x0F
#define AD9467_REG_OFFSET                   0x10
#define AD9467_REG_OUT_MODE                 0x14
#define AD9467_REG_OUT_ADJ                  0x15
#define AD9467_REG_OUT_PHASE                0x16
#define AD9467_REG_OUT_DELAY                0x17
#define AD9467_REG_V_REF                    0x18
#define AD9467_REG_ANALOG_INPUT             0x2C
#define AD9467_REG_BUFF_CURRENT_1           0x36
#define AD9467_REG_BUFF_CURRENT_2           0x107

/* AD9467_REG_CHIP_PORT_CFG */
#define AD9467_CHIP_PORT_CGF_LSB_FIRST      (1 << 6)
#define AD9467_CHIP_PORT_CGF_SOFT_RST       (1 << 5)

/* AD9467_REG_CHIP_GRADE */
#define AD9467_CHIP_GRADE_BITS(x)           (((x) & 0x3) << 4)

/* AD9467_REG_DEVICE_UPDATE */
#define AD9467_DEVICE_UPDATE_SW             (1 << 0)

/* AD9467_REG_MODES */
#define AD9467_MODES_INT_PD_MODE(x)         (((x) & 0x3) << 0)

/* AD9467_REG_TEST_IO */
#define AD9467_TEST_IO_RST_PN_LONG          (1 << 5)
#define AD9467_TEST_IO_RST_PN_SHORT         (1 << 4)
#define AD9467_TEST_IO_OUT_TEST(x)          (((x) & 0xF) << 0)

/* AD9467_REG_ADC_INPUT */
#define AD9467_ADC_INPUT_XVREF              (1 << 7)
#define AD9467_ADC_INPUT_ANALOG_DSCN        (1 << 2)

/* AD9467_REG_OUT_MODE */
#define AD9467_OUT_MODE_DOUT_DISBL          (1 << 4)
#define AD9467_OUT_MODE_OUT_INV             (1 << 2)
#define AD9467_OUT_MODE_DATA_FORMAT(x)      (((x) & 0x3) << 0)

/* AD9467_REG_OUT_ADJ */
#define AD9467_OUT_ADJ_LVDS                 (1 << 3)
#define AD9467_OUT_ADJ_OUT_CURRENT(x)       (((x) & 0x7) << 0)

/* AD9467_REG_OUT_PHASE */
#define AD9467_OUT_PHASE_DCO_INV            (1 << 7)

/* AD9467_REG_OUT_DELAY */
#define AD9467_OUT_DELAY_DCO_DLY_EN         (1 << 7)
#define AD9467_OUT_DELAY_OUT_DLY(x)         (((x) & 0x1F) << 0)

/* AD9467_REG_V_REF */
#define AD9467_V_REF_IN_FS_RANGE(x)         (((x) & 0xF) << 0)

/* AD9467_REG_ANALOG_INPUT */
#define AD9467_ANALOG_INPUT_COUPLING        (1 << 2)

/* AD9467_REG_BUFF_CURRENT_1 */
#define AD9467_BUFF_CURRENT_1(x)            (((x) & 0x3F) << 2)

/* AD9467_REG_BUFF_CURRENT_2 */
#define AD9467_BUFF_CURRENT_2(x)            (((x) & 0x3F) << 2)

/******************************************************************************/
/************************ Types Definitions ***********************************/
/***************************************************************************//**
 * @brief The `ad9467_dev` structure is a simple data structure designed to
 * encapsulate the SPI communication descriptor for the AD9467 device. It
 * contains a single member, `spi_desc`, which is a pointer to a
 * `no_os_spi_desc` structure. This structure is used to manage and
 * facilitate SPI communication with the AD9467, a high-speed analog-to-
 * digital converter. The `ad9467_dev` structure is essential for
 * initializing and managing the SPI interface required for interacting
 * with the AD9467 device.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for communication.
 ******************************************************************************/
struct ad9467_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
};

/***************************************************************************//**
 * @brief The `ad9467_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the AD9467 device,
 * specifically focusing on the SPI interface configuration. This
 * structure is essential for initializing the device communication
 * settings before any operations can be performed on the AD9467,
 * ensuring that the SPI interface is correctly configured for subsequent
 * data transactions.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 ******************************************************************************/
struct ad9467_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function sets up the AD9467 device by initializing the SPI
 * interface and configuring the device to a default state. It must be
 * called before any other operations on the AD9467 device to ensure
 * proper communication and functionality. The function allocates memory
 * for the device structure and configures the test and output modes. If
 * the setup is successful, the device pointer is updated to point to the
 * initialized device structure. The function returns an error code if
 * memory allocation fails or if any configuration step encounters an
 * error.
 *
 * @param device A pointer to a pointer of type struct ad9467_dev. This
 * parameter will be updated to point to the newly allocated and
 * initialized device structure. Must not be null.
 * @param init_param A struct of type ad9467_init_param containing
 * initialization parameters for the SPI interface. The
 * structure must be properly populated before calling this
 * function.
 * @return Returns 0 on success, or a negative error code if memory allocation
 * fails or if any configuration step encounters an error.
 ******************************************************************************/
int32_t ad9467_setup(struct ad9467_dev **device,
		     struct ad9467_init_param init_param);
/***************************************************************************//**
 * @brief Use this function to release all resources allocated for an AD9467
 * device instance when it is no longer needed. This function should be
 * called after all operations with the device are complete to ensure
 * proper cleanup and to prevent resource leaks. It is important to
 * ensure that the device pointer is valid and was previously initialized
 * by a successful call to `ad9467_setup`. The function will handle the
 * deallocation of the SPI descriptor and the device structure itself.
 *
 * @param dev A pointer to an `ad9467_dev` structure representing the device
 * instance to be removed. Must not be null and should point to a
 * valid, initialized device structure. The function will free the
 * memory associated with this structure.
 * @return Returns an integer status code from the SPI removal operation, where
 * 0 indicates success and a negative value indicates an error.
 ******************************************************************************/
int32_t ad9467_remove(struct ad9467_dev *dev);
/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * within the AD9467 device. It is essential for configuring the device's
 * settings by updating its registers. The function requires a valid
 * device structure, a register address, and the value to be written. It
 * is important to ensure that the device has been properly initialized
 * before calling this function. The function returns an integer status
 * code indicating the success or failure of the write operation.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null and should be properly initialized before use.
 * @param reg_addr The address of the register to which the data will be
 * written. It is a 16-bit unsigned integer.
 * @param reg_val The 8-bit value to be written to the specified register.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9467_write(struct ad9467_dev *dev,
		     uint16_t reg_addr,
		     uint8_t reg_val);
/***************************************************************************//**
 * @brief Use this function to read a byte of data from a specific register of
 * the AD9467 device. It requires a valid device structure and a register
 * address to read from. The function will store the read value in the
 * provided memory location pointed to by reg_val. Ensure that the device
 * has been properly initialized before calling this function. The
 * function returns an integer status code indicating the success or
 * failure of the read operation.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null and should be properly initialized before use.
 * @param reg_addr The address of the register to read from. It is a 16-bit
 * unsigned integer representing the register address within the
 * device.
 * @param reg_val A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns an int32_t status code. A value of 0 typically indicates
 * success, while a negative value indicates an error occurred during
 * the read operation.
 ******************************************************************************/
int32_t ad9467_read(struct ad9467_dev *dev,
		    uint16_t reg_addr,
		    uint8_t *reg_val);
/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * AD9467 device while preserving the other bits. It is useful when only
 * a subset of bits in a register needs to be changed. The function reads
 * the current value of the register, applies a mask to clear the bits to
 * be modified, and then sets the new bits as specified. It must be
 * called with a valid device structure and register address. The
 * function returns an error code if the read or write operation fails.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null.
 * @param register_address The address of the register to modify. Must be a
 * valid register address for the AD9467 device.
 * @param bits_value The new bit values to set in the register. Only the bits
 * specified by the mask will be affected.
 * @param mask A bitmask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be updated with the
 * corresponding bits from bits_value.
 * @return Returns 0 on success, or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
uint32_t ad9467_set_bits_to_reg(struct ad9467_dev *dev,
				uint16_t register_address,
				uint8_t bits_value,
				uint8_t mask);
/***************************************************************************//**
 * @brief This function sets or retrieves the power mode of the AD9467 device.
 * It should be called when you need to change the power mode to either
 * normal or power-down, or to check the current power mode. The function
 * requires a valid device structure and a mode value indicating the
 * desired power mode. If the mode is set to 0 or 1, the function will
 * attempt to set the power mode accordingly. If the mode is any other
 * value, the function will return the current power mode without making
 * changes. The function updates the ret_mode parameter with the
 * resulting power mode, whether set or retrieved.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null.
 * @param mode An integer representing the desired power mode. Valid values are
 * 0 (normal mode) and 1 (power-down mode). Any other value will
 * cause the function to return the current power mode without
 * changing it.
 * @param ret_mode A pointer to an integer where the resulting power mode will
 * be stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. The
 * ret_mode parameter is updated with the current or set power mode.
 ******************************************************************************/
int32_t ad9467_pwr_mode(struct ad9467_dev *dev,
			int32_t mode,
			int32_t *ret_mode);
/***************************************************************************//**
 * @brief This function is used to configure the test mode of the AD9467 ADC or
 * to retrieve the current test mode setting. It should be called when
 * you need to either set a new test mode or check the current mode. The
 * function requires a valid device structure and a mode value within the
 * specified range if setting a mode. If the mode is outside the valid
 * range, the function will instead return the current mode. Ensure the
 * device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param mode An integer specifying the test mode to set, ranging from 0 to 7.
 * If the value is outside this range, the function will return the
 * current mode instead of setting a new one.
 * @param ret_mode A pointer to an integer where the function will store the
 * current or newly set test mode. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. The
 * ret_mode parameter is updated with the current or newly set mode.
 ******************************************************************************/
int32_t ad9467_test_mode(struct ad9467_dev *dev,
			 int32_t mode,
			 int32_t *ret_mode);
/***************************************************************************//**
 * @brief This function is used to set or query the reset state of the short PN
 * sequence (PN9) in the AD9467 device. It can be used to either enable
 * or disable the reset state by passing 1 or 0, respectively, as the
 * `rst` parameter. If the `rst` parameter is neither 0 nor 1, the
 * function will query the current state instead. The function must be
 * called with a valid device structure, and it will update the
 * `ret_stat` parameter to reflect the current or newly set state. It
 * returns an error code if the operation fails.
 *
 * @param dev A pointer to an initialized `ad9467_dev` structure representing
 * the device. Must not be null.
 * @param rst An integer indicating the desired reset state: 1 to set the reset,
 * 0 to clear it, or any other value to query the current state.
 * @param ret_stat A pointer to an integer where the function will store the
 * current or newly set reset state. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. Updates
 * `ret_stat` with the reset state.
 ******************************************************************************/
int32_t ad9467_reset_pn9(struct ad9467_dev *dev,
			 int32_t rst,
			 int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function is used to control the reset state of the long pseudo-
 * random noise (PN) sequence bit (PN23) in the AD9467 device. It can
 * either set or clear the reset bit based on the provided input. If the
 * input is 0 or 1, the function will set or clear the bit accordingly
 * and update the status. If the input is any other value, the function
 * will read the current state of the reset bit and return it. This
 * function should be used when you need to manage the PN23 sequence
 * reset state, and it requires a valid device structure to operate.
 *
 * @param dev A pointer to an initialized ad9467_dev structure. Must not be
 * null. The caller retains ownership.
 * @param rst An integer value indicating the desired state of the reset bit.
 * Valid values are 0 (clear) or 1 (set). If any other value is
 * provided, the function will read the current state instead.
 * @param ret_stat A pointer to an integer where the function will store the
 * resulting state of the reset bit. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. The
 * ret_stat parameter is updated with the current or new state of the
 * reset bit.
 ******************************************************************************/
int32_t ad9467_reset_pn23(struct ad9467_dev *dev,
			  int32_t rst,
			  int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function is used to control the external voltage reference of the
 * AD9467 device. It can either enable or disable the external reference
 * based on the provided parameter. The function must be called with a
 * valid device structure and is typically used when configuring the ADC
 * for specific input reference requirements. If the enable parameter is
 * set to 0 or 1, the function will attempt to set the external reference
 * accordingly and update the status. If the parameter is outside this
 * range, the function will read the current status of the external
 * reference instead. The function returns an error code if the operation
 * fails, and updates the status parameter with the current state of the
 * external reference.
 *
 * @param dev A pointer to an initialized ad9467_dev structure representing the
 * device. Must not be null.
 * @param en An integer indicating whether to enable (1) or disable (0) the
 * external reference. If not 0 or 1, the function reads the current
 * status instead.
 * @param ret_stat A pointer to an integer where the function will store the
 * current status of the external reference. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. Updates the
 * ret_stat parameter with the current status of the external reference.
 ******************************************************************************/
int32_t ad9467_external_ref(struct ad9467_dev *dev,
			    int32_t en,
			    int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function is used to either disconnect or connect the analog input
 * to the ADC channel based on the specified enable parameter. It should
 * be called when there is a need to control the connection state of the
 * analog input. The function requires a valid device structure and
 * updates the status of the connection in the provided status pointer.
 * It handles invalid enable values by reading the current connection
 * state instead of modifying it.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null.
 * @param en An integer indicating whether to disconnect (1) or connect (0) the
 * analog input. Values other than 0 or 1 will result in the function
 * reading the current state instead of changing it.
 * @param ret_stat A pointer to an integer where the function will store the
 * resulting connection status. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. The
 * ret_stat parameter is updated with the current connection status.
 ******************************************************************************/
int32_t ad9467_analog_input_disconnect(struct ad9467_dev *dev,
				       int32_t en,
				       int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function adjusts the offset of the AD9467 device to a specified
 * value within the range of -128 to 127. It should be used when there is
 * a need to calibrate or modify the offset setting of the device. If the
 * specified adjustment value is within the valid range, the function
 * writes this value to the device and updates the status with the new
 * offset. If the value is outside the valid range, the function reads
 * the current offset from the device and returns it in the status. This
 * function must be called with a valid device structure, and the status
 * pointer must not be null.
 *
 * @param dev A pointer to an initialized ad9467_dev structure representing the
 * device. Must not be null.
 * @param adj The desired offset adjustment value, which must be between -128
 * and 127 inclusive. Values outside this range will result in the
 * current offset being read instead.
 * @param ret_stat A pointer to an integer where the function will store the
 * resulting offset value. Must not be null.
 * @return Returns 0 on success, or a negative error code if the write operation
 * fails. The ret_stat parameter is updated with the new or current
 * offset value.
 ******************************************************************************/
int32_t ad9467_offset_adj(struct ad9467_dev *dev,
			  int32_t adj,
			  int32_t *ret_stat);
/***************************************************************************//**
 * @brief Use this function to control the data output state of the AD9467
 * device, either disabling or enabling it based on the provided
 * parameter. This function should be called when you need to manage the
 * data output state, such as during configuration changes or power
 * management. It requires a valid device structure and will update the
 * status of the output state in the provided status pointer. Ensure that
 * the device has been properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad9467_dev structure representing the
 * device. Must not be null.
 * @param en An integer value where 1 disables the data output and 0 enables it.
 * Any other value will result in the function returning the current
 * output state without making changes.
 * @param ret_stat A pointer to an integer where the function will store the
 * current output state after execution. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during register access. The current output state is stored in the
 * ret_stat parameter.
 ******************************************************************************/
int32_t ad9467_output_disable(struct ad9467_dev *dev,
			      int32_t en,
			      int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function is used to set the output mode of the AD9467 device to
 * either inverted or normal. It should be called when there is a need to
 * change the output signal inversion state. The function accepts a
 * parameter to specify the desired mode and updates the device
 * configuration accordingly. If the provided mode is invalid, the
 * function will instead return the current inversion state without
 * making changes. It is important to ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null, and the device must be initialized.
 * @param invert An integer indicating the desired output mode: 0 for normal
 * output, 1 for inverted output. Other values will not change the
 * mode but will return the current state.
 * @param ret_stat A pointer to an integer where the function will store the
 * resulting output mode state. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure. The
 * current or new output mode state is stored in the location pointed to
 * by ret_stat.
 ******************************************************************************/
int32_t ad9467_output_invert(struct ad9467_dev *dev,
			     int32_t invert,
			     int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function sets or retrieves the output data format of the AD9467
 * device. It should be called when you need to configure the data format
 * for the device's output or to check the current format. The function
 * accepts a format value to set the output format, or it can be used to
 * read the current format if an invalid format value is provided. The
 * function must be called with a valid device structure, and the output
 * format can be one of the predefined values (0, 1, or 2). If an invalid
 * format is provided, the function will return the current format
 * instead.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param format An integer specifying the desired output format. Valid values
 * are 0, 1, or 2. If an invalid value is provided, the function
 * will return the current format instead.
 * @param ret_stat A pointer to an integer where the function will store the
 * resulting format status. Must not be null. The caller retains
 * ownership.
 * @return Returns 0 on success or a negative error code on failure. The
 * ret_stat parameter is updated with the format value.
 ******************************************************************************/
int32_t ad9467_output_format(struct ad9467_dev *dev,
			     int32_t format,
			     int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function is used to configure the LVDS output properties of the
 * AD9467 device by setting or clearing the LVDS adjustment bit. It
 * should be called when there is a need to modify the LVDS output
 * settings. The function can also be used to query the current LVDS
 * adjustment status if an invalid adjustment value is provided. It
 * requires a valid device structure and a pointer to store the status of
 * the LVDS adjustment. The function must be called with a properly
 * initialized device structure.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param lvds_adj An integer indicating the desired LVDS adjustment setting.
 * Valid values are 0 or 1. If an invalid value is provided, the
 * function will return the current LVDS adjustment status
 * instead of making changes.
 * @param ret_stat A pointer to an integer where the function will store the
 * status of the LVDS adjustment after execution. Must not be
 * null.
 * @return Returns 0 on success or a negative error code on failure. The
 * ret_stat parameter is updated with the LVDS adjustment status.
 ******************************************************************************/
int32_t ad9467_coarse_lvds_adj(struct ad9467_dev *dev,
			       int32_t lvds_adj,
			       int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function adjusts the output current setting of the AD9467 device
 * based on the provided adjustment value. It should be used when there
 * is a need to modify the output current configuration. The function
 * requires a valid device structure and an adjustment value within the
 * range of 1 to 7. If the adjustment value is outside this range, the
 * function will instead return the current output setting without making
 * changes. The function must be called with a valid device pointer and a
 * non-null pointer for the return status to store the result of the
 * operation.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null.
 * @param adj An integer representing the desired output current adjustment.
 * Valid values are between 1 and 7 inclusive. If outside this range,
 * the function will not adjust the setting.
 * @param ret_stat A pointer to an integer where the function will store the
 * current or adjusted output current setting. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. The current
 * or adjusted output current setting is stored in the location pointed
 * to by ret_stat.
 ******************************************************************************/
int32_t ad9467_output_current_adj(struct ad9467_dev *dev,
				  int32_t adj,
				  int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function is used to set the DCO clock mode of the AD9467 device
 * to either normal or inverted. It should be called when there is a need
 * to change the clock phase for synchronization purposes. The function
 * can also be used to query the current DCO clock mode by passing a
 * value other than 0 or 1 for the `invert` parameter. It is important to
 * ensure that the device is properly initialized before calling this
 * function. The function updates the `ret_stat` parameter to reflect the
 * current DCO clock mode after execution.
 *
 * @param dev A pointer to an `ad9467_dev` structure representing the device.
 * Must not be null.
 * @param invert An integer indicating the desired DCO clock mode: 0 for normal,
 * 1 for inverted. Any other value will query the current mode
 * without changing it.
 * @param ret_stat A pointer to an integer where the current DCO clock mode will
 * be stored after the function call. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. The
 * `ret_stat` parameter is updated with the current DCO clock mode.
 ******************************************************************************/
int32_t ad9467_dco_clock_invert(struct ad9467_dev *dev,
				int32_t invert,
				int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function sets the delay for the Data Clock Output (DCO) of the
 * AD9467 device. It should be used when there is a need to adjust the
 * timing of the DCO relative to the data output. The function accepts a
 * delay value in picoseconds, which must be between 0 and 3200
 * inclusive. If the delay is set to 0, the delay is disabled. The
 * function also provides feedback on the actual delay set via the
 * ret_stat parameter. It is important to ensure that the device is
 * properly initialized before calling this function. The function
 * returns an error code if the operation fails, which should be checked
 * by the caller.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param delay An integer specifying the desired delay in picoseconds. Valid
 * values are 0 to 3200 inclusive. If the value is 0, the delay is
 * disabled.
 * @param ret_stat A pointer to an integer where the function will store the
 * actual delay set. Must not be null. The caller provides the
 * memory for this output.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t ad9467_dco_output_clock_delay(struct ad9467_dev *dev,
				      int32_t delay,
				      int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function sets the full-scale input voltage range of the AD9467
 * ADC to a specified value or retrieves the current setting if the
 * specified value is not within the valid range. It should be called
 * when the ADC's input range needs to be configured or verified. The
 * function requires a valid device structure and a pointer to store the
 * resulting status. It handles specific voltage ranges and returns an
 * error code if the operation fails.
 *
 * @param dev A pointer to an initialized ad9467_dev structure representing the
 * device. Must not be null.
 * @param v_fs A float representing the desired full-scale voltage range. Valid
 * values are 2.0 or between 2.1 and 2.5 inclusive. If outside these
 * ranges, the function retrieves the current setting instead.
 * @param ret_stat A pointer to a float where the function will store the
 * resulting full-scale voltage range. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. The
 * ret_stat parameter is updated with the current or set full-scale
 * voltage range.
 ******************************************************************************/
int32_t ad9467_full_scale_range(struct ad9467_dev *dev,
				float v_fs,
				float *ret_stat);
/***************************************************************************//**
 * @brief This function sets the analog input coupling mode of the AD9467 device
 * to either AC or DC based on the provided coupling_mode parameter. It
 * should be called when there is a need to change the coupling mode of
 * the device. The function requires a valid device structure and a
 * coupling mode value of either 0 (for AC coupling) or 7 (for DC
 * coupling). If an invalid coupling mode is provided, the function will
 * not change the mode but will return the current mode status. The
 * function updates the ret_stat parameter with the current coupling mode
 * after execution.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null.
 * @param coupling_mode An integer specifying the desired coupling mode. Valid
 * values are 0 for AC coupling and 7 for DC coupling.
 * Other values will result in the function returning the
 * current mode without making changes.
 * @param ret_stat A pointer to an integer where the function will store the
 * current coupling mode after execution. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. Updates
 * ret_stat with the current coupling mode.
 ******************************************************************************/
int32_t ad9467_analog_input_coupling(struct ad9467_dev *dev,
				     int32_t coupling_mode,
				     int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function sets or retrieves the input buffer current setting for
 * the AD9467 device based on the provided percentage value. It should be
 * used when there is a need to configure the buffer current to a
 * specific level or to read the current setting. The function expects a
 * valid device structure and a percentage value within the range of -100
 * to 530. If the percentage is within this range, the function sets the
 * buffer current accordingly; otherwise, it retrieves the current
 * setting. The function must be called with a valid device pointer and a
 * non-null pointer for the return status.
 *
 * @param dev A pointer to an initialized ad9467_dev structure representing the
 * device. Must not be null.
 * @param percentage An integer representing the desired buffer current setting
 * as a percentage. Valid range is -100 to 530. Values outside
 * this range will trigger a read of the current setting
 * instead of a write.
 * @param ret_stat A pointer to an integer where the function will store the
 * resulting buffer current setting. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. The
 * ret_stat parameter is updated with the current or set buffer current
 * percentage.
 ******************************************************************************/
int32_t ad9467_buffer_current_1(struct ad9467_dev *dev,
				int32_t percentage,
				int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function sets or retrieves the input buffer current setting for
 * the AD9467 device based on the specified percentage. It should be used
 * when there is a need to adjust the buffer current to a specific level
 * or to query the current setting. The function accepts a percentage
 * value within the range of -100 to 530, where valid values will update
 * the device's setting, and out-of-range values will result in the
 * current setting being read back. The function must be called with a
 * valid device structure, and the caller is responsible for ensuring
 * that the device has been properly initialized before use.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null, and the device must be initialized before calling
 * this function.
 * @param percentage An integer representing the desired buffer current setting
 * as a percentage. Valid values range from -100 to 530.
 * Values outside this range will cause the function to return
 * the current setting instead of updating it.
 * @param ret_stat A pointer to an integer where the function will store the
 * resulting buffer current setting. Must not be null.
 * @return Returns 0 on success or a negative error code on failure. The
 * ret_stat parameter is updated with the current or set buffer current
 * percentage.
 ******************************************************************************/
int32_t ad9467_buffer_current_2(struct ad9467_dev *dev,
				int32_t percentage,
				int32_t *ret_stat);
/***************************************************************************//**
 * @brief This function is used to initiate a transfer operation on the AD9467
 * device by writing to the device update register and then polling it
 * until the transfer is complete. It should be called when a register
 * update needs to be applied to the device. The function ensures that
 * the update is fully completed before returning, making it suitable for
 * use in scenarios where subsequent operations depend on the completion
 * of this transfer. It is important to ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad9467_dev structure representing the device. Must
 * not be null. The caller retains ownership and is responsible for
 * ensuring the device is initialized.
 * @return Returns 0 on success, or a negative error code if the transfer fails.
 ******************************************************************************/
int32_t ad9467_transfer(struct ad9467_dev *dev);

#endif /* __AD9467_H__ */
