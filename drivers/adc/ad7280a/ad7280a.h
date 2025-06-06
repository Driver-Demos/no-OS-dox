/**************************************************************************//**
*   @file   ad7280a.c
*   @brief  Driver's header for the AD7280A Lithium Ion Battery Monitoring System
*   @author Lucian Sin (Lucian.Sin@analog.com)
*
*******************************************************************************
* Copyright 2014(c) Analog Devices, Inc.
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

#ifndef _AD7280A_H_
#define _AD7280A_H_


/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* GPIOs */
#define AD7280A_PD_OUT              no_os_gpio_direction_output(dev->gpio_pd,    \
			            NO_OS_GPIO_HIGH)
#define AD7280A_PD_HIGH             no_os_gpio_set_value(dev->gpio_pd,           \
			            NO_OS_GPIO_HIGH)
#define AD7280A_PD_LOW              no_os_gpio_set_value(dev->gpio_pd,           \
			            NO_OS_GPIO_LOW)
#define AD7280A_CNVST_OUT           no_os_gpio_direction_output(dev->gpio_cnvst, \
			            NO_OS_GPIO_HIGH)
#define AD7280A_CNVST_HIGH          no_os_gpio_set_value(dev->gpio_cnvst,        \
			            NO_OS_GPIO_HIGH)
#define AD7280A_CNVST_LOW           no_os_gpio_set_value(dev->gpio_cnvst,        \
			            NO_OS_GPIO_LOW)

#define AD7280A_ALERT_IN            no_os_gpio_direction_input(dev->gpio_alert)
#define AD7280_ALERT                (1 << 6)

/* Acquisition time */
#define AD7280A_ACQ_TIME_400ns      0
#define AD7280A_ACQ_TIME_800ns      1
#define AD7280A_ACQ_TIME_1200ns     2
#define AD7280A_ACQ_TIME_1600ns     3

/* Conversion averaging */
#define AD7280A_CONV_AVG_DIS        0
#define AD7280A_CONV_AVG_2          1
#define AD7280A_CONV_AVG_4          2
#define AD7280A_CONV_AVG_8          3

/* Alert register bits */
#define AD7280A_ALERT_REMOVE_VIN5       (1 << 2)
#define AD7280A_ALERT_REMOVE_VIN4_VIN5  (2 << 2)
#define AD7280A_ALERT_REMOVE_AUX5       (1 << 0)
#define AD7280A_ALERT_REMOVE_AUX4_AUX5  (2 << 0)

/* Registers */
#define AD7280A_CELL_VOLTAGE_1          0x0  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_2          0x1  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_3          0x2  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_4          0x3  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_5          0x4  /* D11 to D0, Read only */
#define AD7280A_CELL_VOLTAGE_6          0x5  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_1               0x6  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_2               0x7  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_3               0x8  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_4               0x9  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_5               0xA  /* D11 to D0, Read only */
#define AD7280A_AUX_ADC_6               0xB  /* D11 to D0, Read only */
#define AD7280A_SELF_TEST               0xC  /* D11 to D0, Read only */
#define AD7280A_CONTROL_HB              0xD  /* D15 to D8, Read/write */
#define AD7280A_CONTROL_LB              0xE  /* D7 to D0, Read/write */
#define AD7280A_CELL_OVERVOLTAGE        0xF  /* D7 to D0, Read/write */
#define AD7280A_CELL_UNDERVOLTAGE       0x10 /* D7 to D0, Read/write */
#define AD7280A_AUX_ADC_OVERVOLTAGE     0x11 /* D7 to D0, Read/write */
#define AD7280A_AUX_ADC_UNDERVOLTAGE    0x12 /* D7 to D0, Read/write */
#define AD7280A_ALERT                   0x13 /* D7 to D0, Read/write */
#define AD7280A_CELL_BALANCE            0x14 /* D7 to D0, Read/write */
#define AD7280A_CB1_TIMER               0x15 /* D7 to D0, Read/write */
#define AD7280A_CB2_TIMER               0x16 /* D7 to D0, Read/write */
#define AD7280A_CB3_TIMER               0x17 /* D7 to D0, Read/write */
#define AD7280A_CB4_TIMER               0x18 /* D7 to D0, Read/write */
#define AD7280A_CB5_TIMER               0x19 /* D7 to D0, Read/write */
#define AD7280A_CB6_TIMER               0x1A /* D7 to D0, Read/write */
#define AD7280A_PD_TIMER                0x1B /* D7 to D0, Read/write */
#define AD7280A_READ                    0x1C /* D7 to D0, Read/write */
#define AD7280A_CNVST_N_CONTROL         0x1D /* D7 to D0, Read/write */

/* Bits and Masks */
#define AD7280A_CTRL_HB_CONV_INPUT_ALL                  (0 << 6)
#define AD7280A_CTRL_HB_CONV_INPUT_6CELL_AUX1_3_4       (1 << 6)
#define AD7280A_CTRL_HB_CONV_INPUT_6CELL                (2 << 6)
#define AD7280A_CTRL_HB_CONV_INPUT_SELF_TEST            (3 << 6)
#define AD7280A_CTRL_HB_CONV_RES_READ_ALL               (0 << 4)
#define AD7280A_CTRL_HB_CONV_RES_READ_6CELL_AUX1_3_4    (1 << 4)
#define AD7280A_CTRL_HB_CONV_RES_READ_6CELL             (2 << 4)
#define AD7280A_CTRL_HB_CONV_RES_READ_NO                (3 << 4)
#define AD7280A_CTRL_HB_CONV_START_CNVST                (0 << 3)
#define AD7280A_CTRL_HB_CONV_START_CS                   (1 << 3)
#define AD7280A_CTRL_HB_CONV_AVG_DIS                    (0 << 1)
#define AD7280A_CTRL_HB_CONV_AVG_2                      (1 << 1)
#define AD7280A_CTRL_HB_CONV_AVG_4                      (2 << 1)
#define AD7280A_CTRL_HB_CONV_AVG_8                      (3 << 1)
#define AD7280A_CTRL_HB_CONV_AVG(x)                     ((x) << 1)
#define AD7280A_CTRL_HB_PWRDN_SW                        (1 << 0)

#define AD7280A_CTRL_LB_SWRST                           (1 << 7)
#define AD7280A_CTRL_LB_ACQ_TIME_400ns                  (0 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME_800ns                  (1 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME_1200ns                 (2 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME_1600ns                 (3 << 5)
#define AD7280A_CTRL_LB_ACQ_TIME(x)                     ((x) << 5)
#define AD7280A_CTRL_LB_MUST_SET                        (1 << 4)
#define AD7280A_CTRL_LB_THERMISTOR_EN                   (1 << 3)
#define AD7280A_CTRL_LB_LOCK_DEV_ADDR                   (1 << 2)
#define AD7280A_CTRL_LB_INC_DEV_ADDR                    (1 << 1)
#define AD7280A_CTRL_LB_DAISY_CHAIN_RB_EN               (1 << 0)

#define AD7280A_ALERT_GEN_STATIC_HIGH                   (1 << 6)
#define AD7280A_ALERT_RELAY_SIG_CHAIN_DOWN              (3 << 6)

#define AD7280A_ALL_CELLS                               (0xAD << 16)

#define AD7280A_DEVADDR_MASTER                  0
#define AD7280A_DEVADDR_ALL                     0x1F

/* Value to be sent when readings are performed */
#define AD7280A_READ_TXVAL                      0xF800030A

#define NUMBITS_READ        22   // Number of bits for CRC when reading
#define NUMBITS_WRITE       21   // Number of bits for CRC when writing

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad7280a_dev` structure is designed to encapsulate the necessary
 * components for interfacing with the AD7280A Lithium Ion Battery
 * Monitoring System. It includes SPI and GPIO descriptors for
 * communication and control, as well as arrays to store both raw and
 * processed data from the device, such as cell voltages and auxiliary
 * ADC readings. This structure is central to managing the device's
 * operations and data handling in a software application.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_pd Pointer to the GPIO descriptor for power down control.
 * @param gpio_cnvst Pointer to the GPIO descriptor for conversion start
 * control.
 * @param gpio_alert Pointer to the GPIO descriptor for alert signal.
 * @param read_data Array to store raw data read from the device.
 * @param cell_voltage Array to store converted cell voltage values.
 * @param aux_adc Array to store auxiliary ADC converted values.
 ******************************************************************************/
struct ad7280a_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_pd;
	struct no_os_gpio_desc	*gpio_cnvst;
	struct no_os_gpio_desc	*gpio_alert;
	/* Device Settings */
	uint32_t		read_data[24];
	float			cell_voltage[12];
	float			aux_adc[12];
};

/***************************************************************************//**
 * @brief The `ad7280a_init_param` structure is used to encapsulate the
 * initialization parameters required to set up the AD7280A device, which
 * is a Lithium Ion Battery Monitoring System. It includes SPI
 * initialization parameters and GPIO initialization parameters for
 * power-down, conversion start, and alert functionalities, ensuring that
 * the device is properly configured for communication and operation.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_pd Contains the initialization parameters for the power-down GPIO
 * pin.
 * @param gpio_cnvst Contains the initialization parameters for the conversion
 * start GPIO pin.
 * @param gpio_alert Contains the initialization parameters for the alert GPIO
 * pin.
 ******************************************************************************/
struct ad7280a_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_pd;
	struct no_os_gpio_init_param	gpio_cnvst;
	struct no_os_gpio_init_param	gpio_alert;
};

/*****************************************************************************/
/************************ Functions Declarations *****************************/
/*****************************************************************************/
/* Initializes the communication with the device. */
/***************************************************************************//**
 * @brief This function sets up the necessary communication interfaces and
 * configurations to interact with the AD7280A Lithium Ion Battery
 * Monitoring System. It must be called before any other operations on
 * the device to ensure proper initialization. The function allocates
 * memory for the device structure and initializes GPIO and SPI
 * interfaces based on the provided parameters. It also configures the
 * device's control registers for operation. If initialization fails at
 * any step, the function returns an error code, and the device pointer
 * is not valid.
 *
 * @param device A pointer to a pointer of type struct ad7280a_dev. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A struct of type ad7280a_init_param containing
 * initialization parameters for SPI and GPIO interfaces. Must
 * be properly configured before calling the function.
 * @return Returns an int8_t status code: 0 for success, or a negative value for
 * failure. On success, the device pointer is set to a newly allocated
 * and initialized device structure.
 ******************************************************************************/
int8_t ad7280a_init(struct ad7280a_dev **device,
		    struct ad7280a_init_param init_param);

/* Free the resources allocated by AD7280A_Init(). */
/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * associated with an AD7280A device instance when it is no longer
 * needed. It ensures that the SPI and GPIO descriptors are removed and
 * the memory allocated for the device structure is freed. This function
 * must be called after the device has been initialized and used, to
 * prevent resource leaks. It is important to ensure that the `dev`
 * parameter is valid and properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an `ad7280a_dev` structure representing the device
 * instance to be removed. This pointer must not be null and should
 * point to a valid, initialized device structure. The function will
 * handle invalid pointers by returning an error code.
 * @return Returns an `int32_t` value indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a non-zero
 * value indicates an error occurred during the removal process.
 ******************************************************************************/
int32_t ad7280a_remove(struct ad7280a_dev *dev);

/* Reads/transmits 32 data bits from/to AD7280A. */
/***************************************************************************//**
 * @brief This function is used to send a 32-bit data word to the AD7280A device
 * and simultaneously receive a 32-bit response. It is typically used for
 * communication with the AD7280A over an SPI interface, where both
 * transmission and reception occur in a single operation. The function
 * requires a valid device structure that has been properly initialized.
 * It is important to ensure that the device is ready for communication
 * before calling this function to avoid data corruption or communication
 * errors.
 *
 * @param dev A pointer to an ad7280a_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param data A 32-bit unsigned integer representing the data to be sent to the
 * device. There are no specific constraints on the value of this
 * parameter.
 * @return Returns a 32-bit unsigned integer representing the data received from
 * the device during the transfer.
 ******************************************************************************/
uint32_t ad7280a_transfer_32bits(struct ad7280a_dev *dev,
				 uint32_t data);

/* Computes the CRC value for a write transmission, and prepares the complete
 write codeword */
/***************************************************************************//**
 * @brief This function is used to compute the CRC value for a given write
 * message intended for the AD7280A device and prepares the complete
 * 32-bit codeword to be transmitted. It is typically called when
 * preparing data to be sent to the device to ensure data integrity. The
 * function processes the input message by shifting it and applying a CRC
 * algorithm, then combines the original message with the computed CRC
 * and a fixed pattern to form the final codeword.
 *
 * @param message A 32-bit unsigned integer representing the message to be sent
 * to the AD7280A device. The message should be formatted
 * according to the device's communication protocol. Invalid or
 * improperly formatted messages may result in incorrect CRC
 * computation.
 * @return Returns a 32-bit unsigned integer representing the complete codeword,
 * which includes the original message, the computed CRC, and a fixed
 * pattern.
 ******************************************************************************/
uint32_t ad7280a_crc_write(uint32_t message);

/* Checks the received message if the received CRC and computed CRC are
the same. */
/***************************************************************************//**
 * @brief This function checks if the received CRC value in a message matches
 * the computed CRC value, ensuring data integrity. It is used to
 * validate messages received from the AD7280A device, which is crucial
 * for reliable communication. The function should be called whenever a
 * message is received to confirm its correctness before processing. It
 * assumes the message is formatted correctly with the CRC in the
 * expected position.
 *
 * @param message A 32-bit unsigned integer representing the message received
 * from the AD7280A device. The message must include a CRC value
 * in the correct position for validation.
 * @return Returns 1 if the received CRC matches the computed CRC, indicating
 * the message is valid; otherwise, returns 0.
 ******************************************************************************/
int32_t ad7280a_crc_read(uint32_t message);

/* Performs a read from all registers on 2 devices. */
/***************************************************************************//**
 * @brief This function is used to initiate a read and conversion process for
 * all registers on two AD7280A devices, typically used in a battery
 * monitoring system. It configures the necessary control registers,
 * triggers the conversion process, and reads the resulting data into the
 * provided device structure. The function must be called with a valid
 * and initialized `ad7280a_dev` structure. It is important to ensure
 * that the device is properly initialized and that the SPI and GPIO
 * interfaces are correctly configured before calling this function. The
 * function assumes that the device is in a state ready to perform
 * conversions and reads.
 *
 * @param dev A pointer to an `ad7280a_dev` structure representing the device.
 * This structure must be initialized and must not be null. The
 * caller retains ownership of this structure.
 * @return Returns an int8_t value, which is always 1, indicating the function
 * executed the read and conversion process.
 ******************************************************************************/
int8_t ad7280a_convert_read_all(struct ad7280a_dev *dev);

/* Converts acquired data to float values. */
/***************************************************************************//**
 * @brief This function processes raw data from the AD7280A device, converting
 * it into meaningful voltage and auxiliary ADC values. It should be
 * called after data has been read into the `read_data` array of the
 * `ad7280a_dev` structure. The function updates the `cell_voltage` and
 * `aux_adc` arrays within the device structure with the converted float
 * values. It is essential to ensure that the `dev` parameter is properly
 * initialized and contains valid data before calling this function.
 *
 * @param dev A pointer to an `ad7280a_dev` structure. This must be a valid,
 * initialized device structure with the `read_data` array populated
 * with raw data. The caller retains ownership and must ensure it is
 * not null.
 * @return Returns 1 to indicate successful conversion. The `cell_voltage` and
 * `aux_adc` arrays in the `ad7280a_dev` structure are updated with the
 * converted values.
 ******************************************************************************/
int8_t ad7280a_convert_data_all(struct ad7280a_dev *dev);

/* Reads the register content of one selected register. */
/***************************************************************************//**
 * @brief This function is used to read the content of a specific register from
 * a designated AD7280A device in a multi-device setup. It is essential
 * to ensure that the device has been properly initialized before calling
 * this function. The function handles enabling and disabling reading on
 * the devices and performs the necessary data transfer to retrieve the
 * register content. If the read operation is unsuccessful, the function
 * returns -1. This function is useful for monitoring and diagnostics in
 * battery management systems.
 *
 * @param dev A pointer to an initialized ad7280a_dev structure representing the
 * device context. Must not be null.
 * @param dev_addr The address of the device from which the register is to be
 * read. Valid values depend on the specific device addressing
 * scheme used in the setup.
 * @param read_reg The register address to be read. Must be a valid register
 * address as defined by the AD7280A device specifications.
 * @return Returns the content of the specified register as an int16_t. If the
 * read operation fails, returns -1.
 ******************************************************************************/
int16_t ad7280a_read_register(struct ad7280a_dev *dev,
			      uint8_t dev_addr,
			      uint8_t read_reg);

/* Reads the conversion of one channel. */
/***************************************************************************//**
 * @brief Use this function to obtain the conversion result from a specific
 * register of the AD7280A device. It is essential to ensure that the
 * device has been properly initialized and configured before calling
 * this function. The function communicates with the device to initiate a
 * conversion and then reads the result from the specified register. It
 * handles the necessary timing and control signals required for the
 * conversion process. If the read operation is successful, the function
 * returns the conversion result; otherwise, it returns -1 to indicate an
 * error.
 *
 * @param dev A pointer to an initialized ad7280a_dev structure representing the
 * device. Must not be null.
 * @param dev_addr The address of the device from which to read. Must be a valid
 * device address.
 * @param read_reg The register from which to read the conversion result. Must
 * be a valid register address for conversion results.
 * @return Returns the conversion result as a 12-bit signed integer. Returns -1
 * if an error occurs during the read operation.
 ******************************************************************************/
int16_t ad7280a_read_conversion(struct ad7280a_dev *dev,
				uint8_t dev_addr,
				uint8_t read_reg);

/* Converts the input data to a voltage value. */
/***************************************************************************//**
 * @brief This function is used to convert raw 12-bit ADC data into a floating-
 * point voltage value based on the specified type. It is typically
 * called after acquiring raw data from the AD7280A device to interpret
 * the data as a voltage. The function expects a valid type and data
 * input, where the data is masked to 12 bits. The conversion type
 * determines the scaling factor applied to the data, with type 0 and
 * type 1 being supported. The function returns a floating-point value
 * representing the converted voltage.
 *
 * @param type Specifies the conversion type. Valid values are 0 and 1. If the
 * value is not 0 or 1, the function will return 0.0 as the result.
 * @param data The raw ADC data to be converted. It is a 16-bit unsigned
 * integer, but only the lower 12 bits are used for conversion.
 * @return Returns a floating-point value representing the converted voltage
 * based on the input type and data.
 ******************************************************************************/
float ad7280a_convert_data(uint8_t type,
			   uint16_t data);

/* Writes the content of one selected register from the selected device. */
/***************************************************************************//**
 * @brief This function is used to write a specified value to a register on the
 * AD7280A device, identified by the device address and register address.
 * It is typically used to configure the device or set specific
 * operational parameters. The function must be called with a valid
 * device structure and appropriate register and value parameters. It is
 * important to ensure that the device has been properly initialized
 * before calling this function. The function does not return a value,
 * and it assumes that the provided register address is valid for
 * writing.
 *
 * @param dev A pointer to an ad7280a_dev structure representing the device.
 * Must not be null, and the device must be initialized before use.
 * @param dev_addr The address of the device to which the register belongs. It
 * is an 8-bit value representing the device address.
 * @param read_reg The address of the register to be written to. It is an 8-bit
 * value, and only specific register addresses are valid for
 * writing.
 * @param reg_val The value to be written to the specified register. It is an
 * 8-bit value representing the data to be written.
 * @return None
 ******************************************************************************/
void ad7280a_write_register(struct ad7280a_dev *dev,
			    uint8_t dev_addr,
			    uint8_t read_reg,
			    uint8_t reg_val);

/* Performs the self test for two devices(one master and one slave). */
/***************************************************************************//**
 * @brief This function is used to perform a self-test on an AD7280A device,
 * which is a lithium-ion battery monitoring system. It should be called
 * when you need to verify the functionality of the device's self-test
 * feature. The function requires a valid device structure and two
 * pointers to store the results of the self-test for the master and
 * slave devices. It is important to ensure that the device is properly
 * initialized before calling this function. The function will populate
 * the provided pointers with the self-test results, which are
 * represented as floating-point values.
 *
 * @param dev A pointer to an ad7280a_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param self_test_reg_a A pointer to a float where the self-test result for
 * the master device will be stored. This pointer must
 * not be null.
 * @param self_test_reg_b A pointer to a float where the self-test result for
 * the slave device will be stored. This pointer must not
 * be null.
 * @return None
 ******************************************************************************/
void ad7280a_selftest_all(struct ad7280a_dev *dev,
			  float *self_test_reg_a,
			  float *self_test_reg_b);

/* Reads the value of Alert Pin from the device. */
/***************************************************************************//**
 * @brief This function retrieves the current state of the alert pin from the
 * AD7280A device, which is part of a battery monitoring system. It is
 * typically used to check for alert conditions that the device may
 * signal through this pin. The function requires a valid device
 * structure that has been properly initialized. It is important to
 * ensure that the device is correctly configured and that the GPIO for
 * the alert pin is set up before calling this function.
 *
 * @param dev A pointer to an ad7280a_dev structure representing the device.
 * This must be a valid, initialized device structure, and must not
 * be null. The caller retains ownership of this structure.
 * @return Returns a uint8_t value representing the state of the alert pin,
 * where a non-zero value indicates an active alert condition.
 ******************************************************************************/
uint8_t ad7280a_alert_pin(struct ad7280a_dev *dev);

#endif /*_AD7280A_H_*/
