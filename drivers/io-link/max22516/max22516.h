/***************************************************************************//**
 *   @file   max22516.h
 *   @brief  Header file for max22516 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
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

#ifndef MAX22516_H_
#define MAX22516_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define REG_CHIP_ID			0x00
#define REG_REV_ID			0x01
#define REG_IOL_STAT			0x02
#define REG_DEV_STAT1			0x03
#define REG_DEV_STAT2			0x04
#define REG_ISDU_STAT			0x05
#define REG_IOL_ERR_CNT			0x06
#define REG_FRM_ERR_CNT			0x07
#define REG_IOL_INT			0x08
#define REG_DEV_INT			0x09
#define REG_ISDU_INT			0x0A
#define REG_IOL_INT_EN			0x0E
#define REG_DEV_INT_EN			0x0F
#define REG_ISDU_INT_EN			0x10
#define REG_IOL_CFG			0x14
#define REG_WATCHDOG			0x15
#define REG_WDGCLR			0x16
#define REG_MISC_CFG			0x17
#define REG_CLK_CFG			0x18
#define REG_CLK_TRIM			0x19
#define REG_PG1_MSTCMD			0x1A
#define REG_PG1_MSTCYCTM		0x1B
#define REG_PG1_MINCYCTM		0x1C
#define REG_PG1_MSEQCAP			0x1D
#define REG_PG1_REVID			0x1E
#define REG_PG1_PDIN			0x1F
#define REG_PG1_PDOUT			0x20
#define REG_PG1_VID1			0x21
#define REG_PG1_VID2			0x22
#define REG_PG1_DEVID1			0x23
#define REG_PG1_DEVID2			0x24
#define REG_PG1_DEVID3			0x25
#define REG_PG1_FUNCID1			0x26
#define REG_PG1_FUNCID2			0x27
#define REG_PG1_RES1			0x28
#define REG_PG1_RES2			0x29
#define REG_WDG_EVENT			0x2A
#define REG_STATUS_CODE_DEF		0x2B
#define REG_STATUS_CODE			0x2C
#define REG_EVENT_QUAL			0x2D
#define REG_EVENT_CODE_MSB		0x2E
#define REG_EVENT_CODE_LSB		0x2F
#define REG_EVENT_FLAG			0x30
#define REG_PDIN_FIFO			0x35
#define REG_PDIN_DATA_RDY		0x36
#define REG_PDOUT_FIFO			0x37
#define REG_ISDU_OFFSET			0x3F
#define REG_ISDU_INFIFO			0x40
#define REG_ISDU_DATARDY		0x41
#define REG_ISDU_OUTFIFO		0x42
#define REG_ISDU_LEVEL			0x43
#define REG_LED1_CTRL_MSB		0x50
#define REG_LED1_CTRL_LSB		0x51
#define REG_LED2_CTRL_MSB		0x52
#define REG_LED2_CTRL_LSB		0x53
#define REG_GPIO1_CTRL			0x54
#define REG_GPIO2_CTRL			0x55
#define REG_CQ_CTRL1			0x56
#define REG_CQ_CTRL2			0x57
#define REG_DO_CTRL1			0x58
#define REG_DO_CTRL2			0x59
#define REG_TX_CTRL			0x5A
#define REG_RX_CTRL			0x5B
#define REG_MISC_CTRL			0x5C

/* REG_DEV_STAT2 */
#define DEV_STAT2_SET_DOFAULT		NO_OS_BIT(5)
#define DEV_STAT2_SET_CQFAULT		NO_OS_BIT(4)
#define DEV_STAT2_SET_V24ERR		NO_OS_BIT(3)
#define DEV_STAT2_SET_VMWERR		NO_OS_BIT(2)
#define DEV_STAT2_SET_THWARN		NO_OS_BIT(1)
#define DEV_STAT2_SET_TSHD		NO_OS_BIT(0)

/* REG_CQ_CTRL1 */
#define BIT_CQCTRL1_CQ_SLEW0		0x00
#define BIT_CQCTRL1_CQ_SLEW1		NO_OS_BIT(6)
#define BIT_CQCTRL1_CQ_SLEW2		NO_OS_BIT(7)
#define BIT_CQCTRL1_CQ_SLEW3		NO_OS_BIT(7, 6)
#define BIT_CQCTRL1_CQ_PD		NO_OS_BIT(5)
#define BIT_CQCTRL1_CQ_PU		NO_OS_BIT(4)
#define BIT_CQCTRL1_CQ_NPN		NO_OS_BIT(3)
#define BIT_CQCTRL1_CQ_PP		NO_OS_BIT(2)
#define BIT_CQCTRL1_CQ_INV		NO_OS_BIT(1)
#define BIT_CQCTRL1_CQ_EN		NO_OS_BIT(0)

/* REG_CQ_CTRL2 */
#define BIT_CQ_CL_50MA			0x00
#define BIT_CQ_CL_100MA			NO_OS_BIT(6)
#define BIT_CQ_CL_200MA			NO_OS_BIT(7)
#define BIT_CQ_CL_250MA			NO_OS_BIT(7, 6)
#define BIT_CQ_CLBL_128US		0x00
#define BIT_CQ_CLBL_500US		NO_OS_BIT(3)
#define BIT_CQ_CLBL_1000US		NO_OS_BIT(4)
#define BIT_CQ_CLBL_5000US		NO_OS_BIT(4, 3)
#define BIT_CQ_AUTORTY_TIME_50MS	0x00
#define BIT_CQ_AUTORTY_TIME_100MS	NO_OS_BIT(1)
#define BIT_CQ_AUTORTY_TIME_200MS	NO_OS_BIT(2)
#define BIT_CQ_AUTORTY_TIME_500MS	NO_OS_BIT(2, 1)
#define BIT_CQ_AUTORTY			NO_OS_BIT(0)

/* REG_DO_CTRL1 */
#define BIT_DOCTRL1_DO_SLEW0		0x00
#define BIT_DOCTRL1_DO_SLEW1		NO_OS_BIT(6)
#define BIT_DOCTRL1_DO_SLEW2		NO_OS_BIT(7)
#define BIT_DOCTRL1_DO_SLEW3		NO_OS_BIT(7, 6)
#define BIT_DOCTRL1_DO_PD		NO_OS_BIT(5)
#define BIT_DOCTRL1_DO_PU		NO_OS_BIT(4)
#define BIT_DOCTRL1_DO_NPN		NO_OS_BIT(3)
#define BIT_DOCTRL1_DO_PP		NO_OS_BIT(2)
#define BIT_DOCTRL1_DO_INV		NO_OS_BIT(1)
#define BIT_DOCTRL1_DO_EN		NO_OS_BIT(0)

/* REG_DO_CTRL2 */
#define BIT_DO_CL_50MA			0x00
#define BIT_DO_CL_100MA			NO_OS_BIT(6)
#define BIT_DO_CL_200MA			NO_OS_BIT(7)
#define BIT_DO_CL_250MA			NO_OS_BIT(7, 6)
#define BIT_DO_CLBL_128US		0x00
#define BIT_DO_CLBL_500US		NO_OS_BIT(3)
#define BIT_DO_CLBL_1000US		NO_OS_BIT(4)
#define BIT_DO_CLBL_5000US		NO_OS_BIT(4, 3)
#define BIT_DO_AUTORTY_TIME_50MS	0x00
#define BIT_DO_AUTORTY_TIME_100MS	NO_OS_BIT(1)
#define BIT_DO_AUTORTY_TIME_200MS	NO_OS_BIT(2)
#define BIT_DO_AUTORTY_TIME_500MS	NO_OS_BIT(2, 1)
#define BIT_DO_AUTORTY			NO_OS_BIT(0)

/* REG_TX_CTRL */
#define BIT_TXC_CQTX			NO_OS_BIT(7)
#define BIT_TXC_CQTXEN			NO_OS_BIT(6)
#define BIT_TXC_CQDRVSEL		NO_OS_BIT(5)
#define BIT_TXC_DOTX			NO_OS_BIT(4)
#define BIT_TXC_DODRVSEL		NO_OS_BIT(3)
#define BIT_TXC_CQDOPAR			NO_OS_BIT(1)
#define BIT_TXC_DO_AV			NO_OS_BIT(0)

/* REG_DEV_STAT2 */
#define DEV_STAT2_SET_V24ERR		NO_OS_BIT(3)
#define DEV_STAT2_SET_VMERR		NO_OS_BIT(2)
#define DEV_STAT2_SET_THWARN		NO_OS_BIT(1)
#define DEV_STAT2_SET_TSHD		NO_OS_BIT(0)

/* REG_EVENT_FLAG */
#define EVF_EVENT_FLG			NO_OS_BIT(0)

/* MAX22516 EVENT CODE Masks */
#define REG_EVENT_CODE_MSB_MSK		NO_OS_GENMASK(15, 8)
#define REG_EVENT_CODE_LSB_MSK		NO_OS_GENMASK(7, 0)

/* MAX22516 VID Masks*/
#define PG1_VID1_MSK			NO_OS_GENMASK(15, 8)
#define PG1_VID2_MSK			NO_OS_GENMASK(7, 0)

/* MAX22516 DEVID Masks*/
#define PG1_DEVID1_MSK			NO_OS_GENMASK(23, 16)
#define PG1_DEVID2_MSK			NO_OS_GENMASK(15, 8)
#define PG1_DEVID3_MSK			NO_OS_GENMASK(7, 0)

/* MAX22516 FUNCID1 Masks */
#define PG1_FUNCID1_MSB_MSK		NO_OS_GENMASK(15, 8)
#define PG1_FUNCID1_LSB_MSK		NO_OS_GENMASK(7, 0)

/* MAX22516 LED1_CTRL Masks */
#define REG_LED1_CTRL_MSB_MSK		NO_OS_GENMASK(15, 8)
#define REG_LED1_CTRL_LSB_MSK		NO_OS_GENMASK(7, 0)

/* MAX22516 LED2_CTRL Masks */
#define REG_LED2_CTRL_MSB_MSK		NO_OS_GENMASK(15, 8)
#define REG_LED2_CTRL_LSB_MSK		NO_OS_GENMASK(7, 0)

/* MAX22516 Extra Definitions */
#define MAX22516_SPI_DUMMY_DATA		0x00
#define MAX22516_BUFF_SIZE_BYTES     	64
#define MAX22516_SPI_READ_CMD		NO_OS_BIT(7)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `max22516_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the MAX22516 device,
 * specifically focusing on the SPI (Serial Peripheral Interface)
 * configuration. It contains a single member, `spi_init`, which is a
 * pointer to a `no_os_spi_init_param` structure, providing the necessary
 * details for SPI communication setup.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 ******************************************************************************/
struct max22516_init_param {
	/* SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
};

/***************************************************************************//**
 * @brief The `max22516_dev` structure is a device descriptor for the MAX22516,
 * which is a component that communicates over SPI. It contains a pointer
 * to an SPI descriptor (`spi_desc`) for managing SPI communication and a
 * communication buffer (`comm_buff`) used to store data being sent to or
 * received from the MAX22516. This structure is essential for
 * initializing and managing the communication with the MAX22516 device.
 *
 * @param spi_desc Pointer to the SPI descriptor used for SPI communication.
 * @param comm_buff Buffer for communication with the MAX22516, sized by
 * MAX22516_BUFF_SIZE_BYTES.
 ******************************************************************************/
struct max22516_dev {
	/* SPI Initialization parameters */
	struct no_os_spi_desc	*spi_desc;
	/** Buffer used for communication with MAX22516 */
	uint8_t comm_buff[MAX22516_BUFF_SIZE_BYTES];
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* MAX22516 SPI write */
/***************************************************************************//**
 * @brief Use this function to write a single byte of data to a specific
 * register on the MAX22516 device via SPI communication. This function
 * is typically called when you need to configure or update the settings
 * of the MAX22516 device. Ensure that the device has been properly
 * initialized before calling this function. The function returns an
 * integer status code indicating the success or failure of the write
 * operation.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param reg_addr The address of the register to which the data will be
 * written. It is an 8-bit unsigned integer.
 * @param data The data byte to be written to the specified register. It is an
 * 8-bit unsigned integer.
 * @return Returns an integer status code from the underlying SPI write
 * operation, where 0 typically indicates success and a negative value
 * indicates an error.
 ******************************************************************************/
int max22516_write(struct max22516_dev *dev, uint8_t reg_addr,
		   uint8_t data);

/* MAX22516 SPI Read */
/***************************************************************************//**
 * @brief This function is used to read a single byte of data from a specified
 * register of the MAX22516 device via SPI communication. It is essential
 * to ensure that the device has been properly initialized before calling
 * this function. The function requires a valid device descriptor and a
 * register address to read from. The read data is stored in the location
 * pointed to by the data parameter. If the SPI communication fails, the
 * function returns a non-zero error code.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param reg_addr The address of the register to read from. It is an 8-bit
 * unsigned integer, and valid register addresses are defined by
 * the device's specification.
 * @param data A pointer to an 8-bit unsigned integer where the read data will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a non-zero error code if the SPI
 * communication fails.
 ******************************************************************************/
int max22516_read(struct max22516_dev *dev, uint8_t reg_addr,
		  uint8_t *data);

/* MAX22516 Register Update */
/***************************************************************************//**
 * @brief This function is used to update a specific register on the MAX22516
 * device by first reading the current value, applying a mask to clear
 * specific bits, and then setting new data on those bits. It is
 * typically used when only certain bits of a register need to be
 * modified without affecting the others. The function must be called
 * with a valid device descriptor and register address. It handles
 * reading and writing operations internally and returns an error code if
 * any operation fails.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the MAX22516 device.
 * @param mask A bitmask indicating which bits in the register should be cleared
 * before setting new data. Only the bits set to 1 in the mask will
 * be affected.
 * @param data The new data to be written to the register, after applying the
 * mask. Only the bits corresponding to the mask will be updated.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if the read or write operation fails.
 ******************************************************************************/
int max22516_update(struct max22516_dev *dev, uint8_t reg_addr,
		    uint8_t mask, uint8_t data);

/* MAX22516 Burst Write */
/***************************************************************************//**
 * @brief This function is used to write multiple bytes of data to a specified
 * register of a MAX22516 device in a single operation. It is typically
 * called when there is a need to update a sequence of register values
 * efficiently. The function requires a valid device descriptor and
 * assumes that the SPI interface has been properly initialized. The
 * caller must ensure that the data buffer contains at least 'count'
 * bytes of data to be written. The function returns an integer status
 * code indicating the success or failure of the operation.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param reg_addr The starting register address where the data will be written.
 * It is an 8-bit unsigned integer.
 * @param count The number of bytes to write. It is an 8-bit unsigned integer
 * and should not exceed the buffer capacity of the device.
 * @param data A pointer to an array of bytes containing the data to be written.
 * Must not be null and should have at least 'count' elements.
 * @return Returns an integer status code from the SPI write operation, where 0
 * typically indicates success and a negative value indicates an error.
 ******************************************************************************/
int max22516_burst_write_register(struct max22516_dev *dev, uint8_t reg_addr,
				  uint8_t count, uint8_t *data);

/* MAX22516 Burst Read */
/***************************************************************************//**
 * @brief Use this function to read a sequence of bytes from a specific register
 * of the MAX22516 device. It is essential to ensure that the device has
 * been properly initialized and that the SPI communication is correctly
 * set up before calling this function. The function reads the specified
 * number of bytes starting from the given register address and stores
 * them in the provided data buffer. This function is useful for
 * retrieving multiple consecutive register values in a single operation.
 * Ensure that the data buffer is large enough to hold the requested
 * number of bytes. The function returns an error code if the read
 * operation fails.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized.
 * @param reg_addr The starting register address from which to begin reading.
 * Must be a valid register address for the MAX22516 device.
 * @param count The number of bytes to read from the device. Must be a positive
 * integer and should not exceed the buffer size.
 * @param data A pointer to a buffer where the read data will be stored. Must
 * not be null and should have enough space to store 'count' bytes.
 * @return Returns 0 on success or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max22516_burst_read_register(struct max22516_dev *dev, uint8_t reg_addr,
				 uint8_t count, uint8_t *data);

/* MAX22516 build tcyc */
/***************************************************************************//**
 * @brief This function is used to convert a given cycle time, specified in
 * units of 100 microseconds, into a format suitable for a timer
 * register. It is useful when configuring timing parameters for the
 * MAX22516 device. The function handles cycle times up to 132.8
 * milliseconds, encoding them into different step sizes based on the
 * range they fall into. The caller must ensure that the pointer to the
 * timer register is valid and that the cycle time is within the
 * supported range.
 *
 * @param t The cycle time in units of 100 microseconds. Valid values range from
 * 0 to 1327, corresponding to 0 to 132.7 milliseconds.
 * @param tmr A pointer to a uint8_t where the encoded timer value will be
 * stored. Must not be null.
 * @return None
 ******************************************************************************/
void max22516_build_tcyc(int16_t t, uint8_t *tmr);

/* MAX22516 rebuild min cyct to microseconds */
/***************************************************************************//**
 * @brief This function is used to convert a coded time value, represented by
 * the parameter `t`, into microseconds and store the result in the
 * location pointed to by `tmr`. The function interprets the input `t`
 * based on its most significant bits to determine the time increment:
 * 100µs, 400µs, or 1.6ms. It is essential to ensure that `tmr` is a
 * valid pointer to a `uint8_t` memory location where the result can be
 * stored. This function does not perform any error checking on the input
 * values, so the caller must ensure that `t` is within a valid range for
 * the expected operation.
 *
 * @param t An `int16_t` value representing a coded time. The function
 * interprets this value based on its most significant bits to
 * determine the time increment. The caller must ensure that `t` is
 * within a valid range for the expected operation.
 * @param tmr A pointer to a `uint8_t` where the converted time in microseconds
 * will be stored. Must not be null, and the caller is responsible
 * for ensuring it points to a valid memory location.
 * @return The function writes the converted time in microseconds to the
 * location pointed to by `tmr`. It does not return a value.
 ******************************************************************************/
void max22516_rebuild_min_cyct_to_us(int16_t t, uint8_t *tmr);

/* MAX22516 set min ctmr */
/***************************************************************************//**
 * @brief This function configures the minimum cycle time for a MAX22516 device,
 * which is specified in units of 100 microseconds. It should be used
 * when there is a need to adjust the timing parameters of the device to
 * meet specific application requirements. The function must be called
 * with a valid device descriptor that has been properly initialized. It
 * returns an integer status code indicating success or failure of the
 * operation.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * This must be a valid, initialized device descriptor and must not
 * be null.
 * @param min_t A 16-bit unsigned integer representing the minimum cycle time in
 * units of 100 microseconds. The value should be within the valid
 * range for the device's cycle time settings.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max22516_set_min_ctmr(struct max22516_dev *dev, uint16_t min_t);

/* MAX22516 set id */
/***************************************************************************//**
 * @brief This function configures the MAX22516 device by setting its vendor ID,
 * device ID, and function ID. It should be called when these identifiers
 * need to be updated or initialized. The function writes the provided
 * IDs to specific registers on the device. It is important to ensure
 * that the device is properly initialized before calling this function.
 * The function returns an error code if any of the write operations
 * fail, allowing the caller to handle such errors appropriately.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param vid A 16-bit unsigned integer representing the vendor ID to be set.
 * Valid range is from 0 to 65535.
 * @param id A 32-bit unsigned integer representing the device ID to be set.
 * Valid range is from 0 to 4294967295.
 * @param fid A 16-bit unsigned integer representing the function ID to be set.
 * Valid range is from 0 to 65535.
 * @return Returns 0 on success or a negative error code if a write operation
 * fails.
 ******************************************************************************/
int max22516_set_id(struct max22516_dev *dev, uint16_t vid, uint32_t id,
		    uint16_t fid);

/* MAX22516 decode tcyc */
/***************************************************************************//**
 * @brief This function is used to decode a timer value, represented by an 8-bit
 * unsigned integer, into a cycle time, which is stored as a 16-bit
 * signed integer. The function interprets the two most significant bits
 * of the timer value to determine a base multiplier and applies it to
 * the lower six bits to compute the cycle time. It is essential to
 * ensure that the pointer provided for the output cycle time is valid
 * and non-null before calling this function.
 *
 * @param tmr An 8-bit unsigned integer representing the timer value to be
 * decoded. The two most significant bits determine the base
 * multiplier, and the lower six bits are used in the calculation.
 * @param t A pointer to a 16-bit signed integer where the decoded cycle time
 * will be stored. Must not be null, as the function writes the result
 * to this location.
 * @return None
 ******************************************************************************/
void max22516_decode_tcyc(uint8_t tmr, int16_t *t);

/* MAX22516 get mst ctmr */
/***************************************************************************//**
 * @brief This function is used to obtain the master cycle time from a MAX22516
 * device, expressed in units of 100 microseconds. It should be called
 * when the master cycle time is needed for further processing or
 * configuration. The function requires a valid device descriptor and a
 * pointer to store the retrieved cycle time. It reads the cycle time
 * from the device and decodes it into a more usable format. Ensure that
 * the device has been properly initialized before calling this function.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param min_t A uint16_t value representing the minimum cycle time. This
 * parameter is not used in the function and can be ignored.
 * @param c_tmr A pointer to an int16_t where the decoded master cycle time will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max22516_get_mst_ctmr(struct max22516_dev *dev, uint16_t min_t,
			  int16_t *c_tmr);

/* MAX22516 get dl mode */
/***************************************************************************//**
 * @brief This function is used to obtain the current download mode setting from
 * a MAX22516 device. It should be called when the user needs to check or
 * verify the download mode configuration of the device. The function
 * requires a valid device descriptor and a pointer to a uint8_t variable
 * where the mode will be stored. It is important to ensure that the
 * device has been properly initialized before calling this function. The
 * function will return an error code if the operation fails, which
 * should be handled appropriately by the caller.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * This must be a valid, initialized device descriptor and must not
 * be null.
 * @param mode A pointer to a uint8_t variable where the download mode will be
 * stored. This pointer must not be null, and the caller is
 * responsible for ensuring it points to a valid memory location.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the operation.
 ******************************************************************************/
int max22516_get_dl_mode(struct max22516_dev *dev, uint8_t *mode);

/* MAX22516 get iol err cnt */
/***************************************************************************//**
 * @brief Use this function to obtain the current count of IOL errors from a
 * MAX22516 device. This function is typically called when monitoring the
 * device for communication errors. It requires a valid device descriptor
 * and a pointer to store the error count. Ensure that the device has
 * been properly initialized before calling this function. The function
 * will return an error code if the read operation fails.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param cnt A pointer to a uint8_t variable where the error count will be
 * stored. Must not be null. The caller is responsible for providing
 * a valid memory location.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the read operation.
 ******************************************************************************/
int max22516_get_iol_err_cnt(struct max22516_dev *dev, uint8_t *cnt);

/* MAX22516 get frm err cnt */
/***************************************************************************//**
 * @brief Use this function to obtain the current frame error count from a
 * MAX22516 device. It is typically called when monitoring or diagnosing
 * communication issues with the device. Ensure that the device has been
 * properly initialized before calling this function. The function will
 * store the retrieved error count in the provided memory location. It is
 * important to handle the return value to check for successful execution
 * or any errors during the read operation.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param cnt A pointer to a uint8_t variable where the frame error count will
 * be stored. Must not be null, and the caller is responsible for
 * managing the memory.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error during the read operation.
 ******************************************************************************/
int max22516_get_frm_err_cnt(struct max22516_dev *dev, uint8_t *cnt);

/* MAX22516 clr iol err cnt */
/***************************************************************************//**
 * @brief Use this function to reset the IO-Link error counter of a MAX22516
 * device to zero. This is typically done to clear any accumulated error
 * counts after handling or acknowledging errors. The function should be
 * called when the device is in a state where it can safely reset its
 * error count, such as after initialization or error handling. Ensure
 * that the device pointer is valid and properly initialized before
 * calling this function.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * The function will not modify the ownership of this pointer.
 * @return Returns an integer status code. A return value of 0 typically
 * indicates success, while a negative value indicates an error occurred
 * during the operation.
 ******************************************************************************/
int max22516_clr_iol_err_cnt(struct max22516_dev *dev);

/* MAX22516 clr frm err cnt */
/***************************************************************************//**
 * @brief Use this function to reset the frame error count register of a
 * MAX22516 device to zero. This is typically done to clear any
 * accumulated error counts after they have been read or when starting a
 * new operation where previous error counts are no longer relevant. The
 * function must be called with a valid device descriptor that has been
 * properly initialized. It returns an integer status code indicating the
 * success or failure of the operation.
 *
 * @param dev A pointer to a `max22516_dev` structure representing the device.
 * This must be a valid, initialized device descriptor. The function
 * will not perform any operation if this pointer is null or invalid.
 * @return Returns an integer status code. A return value of 0 typically
 * indicates success, while a non-zero value indicates an error occurred
 * during the operation.
 ******************************************************************************/
int max22516_clr_frm_err_cnt(struct max22516_dev *dev);

/* MAX22516 set led1 */
/***************************************************************************//**
 * @brief This function configures the LED1 control registers of the MAX22516
 * device with a specified timer value. It should be called when you need
 * to set or update the LED1 behavior based on a timer. The function
 * requires a valid device descriptor and a timer value, and it returns
 * an error code if the operation fails. Ensure the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param ltmr A 16-bit unsigned integer representing the timer value to set for
 * LED1. The value is used to configure the LED1 control registers.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int max22516_set_led1(struct max22516_dev *dev, uint16_t ltmr);

/* MAX22516 set led2 */
/***************************************************************************//**
 * @brief This function configures the LED2 control registers of the MAX22516
 * device with a specified timer value. It should be called when you need
 * to update the LED2 settings on the device. The function requires a
 * valid device descriptor and a timer value to be provided. It writes
 * the timer value to both the MSB and LSB control registers for LED2.
 * Ensure that the device has been properly initialized before calling
 * this function. The function returns an error code if the write
 * operation fails.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ltmr A 16-bit unsigned integer representing the timer value to set for
 * LED2. The valid range is from 0 to 65535.
 * @return Returns 0 on success or a negative error code if the write operation
 * fails.
 ******************************************************************************/
int max22516_set_led2(struct max22516_dev *dev, uint16_t ltmr);

/* MAX22516 get v24 */
/***************************************************************************//**
 * @brief This function is used to obtain the V24 status from a MAX22516 device,
 * which indicates specific error conditions. It should be called when
 * the status of the V24 line needs to be checked. The function reads the
 * device status register and updates the provided status variable based
 * on the error flags present. It is important to ensure that the device
 * is properly initialized before calling this function. The function
 * returns an error code if the read operation fails.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param status3 A pointer to a uint8_t variable where the status will be
 * stored. Must not be null. The function writes 0, 1, or 2 to
 * this variable based on the error flags.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max22516_get_v24(struct max22516_dev *dev, uint8_t *status3);

/* MAX22516 get THD */
/***************************************************************************//**
 * @brief This function checks the thermal status of the MAX22516 device and
 * updates the provided status variable accordingly. It should be called
 * when the thermal status needs to be monitored or logged. The function
 * reads a specific register to determine if a thermal shutdown or
 * warning condition is present. It requires a valid device descriptor
 * and a non-null pointer to a status variable. The function returns an
 * error code if the read operation fails, otherwise it returns 0.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param status3 A pointer to a uint8_t variable where the thermal status will
 * be stored. Must not be null. The function writes 2 if a
 * thermal shutdown is detected, 1 if a thermal warning is
 * detected, and 0 if neither condition is present.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max22516_get_thd(struct max22516_dev *dev, uint8_t *status3);

/* MAX22516 setup cq dis */
/***************************************************************************//**
 * @brief This function is used to disable the CQ feature on a MAX22516 device
 * by writing to the appropriate control register. It should be called
 * when the CQ feature is no longer needed or needs to be turned off for
 * the device. The function requires a valid device descriptor and will
 * return an error code if the operation fails.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * This must be a valid, initialized device descriptor. The function
 * will not perform any operation if this pointer is null.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int max22516_setup_cq_dis(struct max22516_dev *dev);

/* MAX22516 setup cq pp */
/***************************************************************************//**
 * @brief This function sets up the MAX22516 device to operate in push-pull mode
 * by configuring the appropriate control registers. It should be called
 * when the device needs to be set to this specific mode, typically
 * during initialization or mode switching. The function requires a valid
 * device descriptor and returns an error code if the configuration
 * fails. Ensure the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership and is responsible
 * for ensuring the device is initialized.
 * @return Returns 0 on success or a negative error code if the configuration
 * fails.
 ******************************************************************************/
int max22516_setup_cq_pp(struct max22516_dev *dev);

/* MAX22516 setup cq pnp */
/***************************************************************************//**
 * @brief This function sets up the MAX22516 device by enabling the CQ (Current
 * and Voltage) control and configuring various parameters such as auto-
 * retry time, current limit, and clamping behavior. It should be called
 * when the device needs to be configured for specific current and
 * voltage settings. The function must be called with a valid device
 * descriptor that has been properly initialized. It returns an error
 * code if the configuration fails, which can be used to diagnose issues
 * with the device setup.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * This must be a valid, initialized device descriptor. The function
 * will return an error if this parameter is null or if the device is
 * not properly initialized.
 * @return Returns 0 on success, or a negative error code if the configuration
 * fails.
 ******************************************************************************/
int max22516_setup_cq_pnp(struct max22516_dev *dev);

/* MAX22516 setup cq npn */
/***************************************************************************//**
 * @brief This function sets up the MAX22516 device to operate in current
 * control mode with NPN configuration. It should be called when the
 * device needs to be configured for NPN mode operation, typically during
 * initialization or reconfiguration. The function writes specific
 * control values to the device's registers to enable this mode. It is
 * important to ensure that the device is properly initialized before
 * calling this function. The function returns an error code if the
 * configuration fails, which can be used for error handling.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership and is responsible
 * for ensuring the device is initialized before calling this
 * function.
 * @return Returns 0 on success or a negative error code if the configuration
 * fails.
 ******************************************************************************/
int max22516_setup_cq_npn(struct max22516_dev *dev);

/* MAX22516 tx set */
/***************************************************************************//**
 * @brief This function is used to set the transmission control settings of a
 * MAX22516 device. It should be called when you need to configure the
 * transmission parameters, specifically the CQTX and CQDRVSEL bits, of
 * the device. The function requires a valid device descriptor and a
 * control value to determine the low or high state of the transmission.
 * It returns an error code if the operation fails, which can be used to
 * diagnose issues with the device communication or configuration.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param low_high A uint8_t value representing the desired state of the
 * transmission control. Typically, 0 for low and 1 for high.
 * Values outside this range may lead to undefined behavior.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int max22516_tx_set(struct max22516_dev *dev, uint8_t low_high);

/* MAX22516 txen set */
/***************************************************************************//**
 * @brief This function configures the transmission enable level of the MAX22516
 * device by updating specific control registers. It should be called
 * when the transmission settings need to be adjusted. The function
 * requires a valid device descriptor and a level value to set. It
 * returns an error code if the operation fails, which can occur if the
 * device descriptor is invalid or if there is a communication error.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param lvl An 8-bit unsigned integer representing the level to set. Valid
 * values depend on the device's expected configuration for the TX
 * enable level.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int max22516_txen_set(struct max22516_dev *dev, uint8_t lvl);

/* MAX22516 set cq */
/***************************************************************************//**
 * @brief This function configures the CQ pin of the MAX22516 device to one of
 * three states: low, high, or high impedance (highZ). It should be used
 * when there is a need to control the output state of the CQ pin,
 * typically in applications involving digital signaling or
 * communication. The function requires a valid device descriptor and a
 * level value indicating the desired state. It returns an error code if
 * the operation fails, which can occur if the device is not properly
 * initialized or if there is a communication error.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param lvl An unsigned 8-bit integer specifying the desired level of the CQ
 * pin. Valid values are 0 (CQ low), 1 (CQ high), and 2 (CQ highZ).
 * Values outside this range are not explicitly handled and may
 * result in undefined behavior.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int max22516_set_cq(struct max22516_dev *dev, uint8_t lvl);

/* MAX22516 tx get */
/***************************************************************************//**
 * @brief This function is used to obtain the current transmission state from a
 * MAX22516 device. It should be called when the user needs to check the
 * transmission status, specifically the low or high state, of the
 * device. The function requires a valid device descriptor and a pointer
 * to store the result. It returns an error code if the read operation
 * fails, otherwise it returns 0 indicating success.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param low_high A pointer to a uint8_t where the transmission state will be
 * stored. Must not be null. The function writes the result to
 * this location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max22516_tx_get(struct max22516_dev *dev, uint8_t *low_high);

/* MAX22516 get cq */
/***************************************************************************//**
 * @brief This function is used to obtain the current status of the CQ (current
 * quality) from a MAX22516 device. It should be called when the user
 * needs to check the CQ status, which is represented as either 0 or 1.
 * The function requires a valid device descriptor and a pointer to a
 * uint8_t variable where the CQ status will be stored. It returns an
 * error code if the operation fails, otherwise it returns 0 indicating
 * success. Ensure that the device has been properly initialized before
 * calling this function.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param cq A pointer to a uint8_t variable where the CQ status will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int max22516_get_cq(struct max22516_dev *dev, uint8_t *cq);

/* MAX22516 get cq stat */
/***************************************************************************//**
 * @brief This function is used to obtain the CQ (Current Quality) status from a
 * MAX22516 device. It should be called when the CQ status needs to be
 * checked or monitored. The function reads the device status register
 * and extracts the relevant CQ status information. It is important to
 * ensure that the device is properly initialized before calling this
 * function. The function will return an error code if the read operation
 * fails, and the status3 parameter will not be modified in such cases.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param status3 A pointer to a uint8_t where the CQ status will be stored.
 * Must not be null. The function writes the CQ status to this
 * location if successful.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max22516_get_cq_stat(struct max22516_dev *dev, uint8_t *status3);

/* MAX22516 get cq */
/***************************************************************************//**
 * @brief This function is used to disable the digital output of a MAX22516
 * device by writing a specific control bit to the device's control
 * register. It should be called when the digital output needs to be
 * turned off, for instance, to save power or to reset the output state.
 * The function requires a valid device descriptor that has been properly
 * initialized. It returns an integer status code indicating the success
 * or failure of the operation.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * This must be a valid, non-null pointer to a properly initialized
 * device descriptor. If the pointer is null, the behavior is
 * undefined.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int max22516_setup_do_dis(struct max22516_dev *dev);

/* MAX22516 setup do pp */
/***************************************************************************//**
 * @brief This function sets up the MAX22516 device to operate in push-pull mode
 * for digital output. It should be called when the device needs to be
 * configured for this specific output mode, typically during
 * initialization or reconfiguration. The function writes specific
 * control values to the device's registers to enable push-pull operation
 * and configure automatic retry and current limit settings. It is
 * important to ensure that the device is properly initialized before
 * calling this function. The function returns an error code if the
 * configuration fails, which should be handled by the caller.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership and is responsible
 * for ensuring the device is initialized before use.
 * @return Returns 0 on success or a negative error code if the configuration
 * fails.
 ******************************************************************************/
int max22516_setup_do_pp(struct max22516_dev *dev);

/* MAX22516 setup do pnp */
/***************************************************************************//**
 * @brief This function is used to configure the MAX22516 device's digital
 * output for push-pull operation with predefined settings. It should be
 * called when the device needs to be set up for this specific mode,
 * typically during initialization or reconfiguration. The function
 * writes to two control registers to enable the digital output and set
 * parameters such as automatic retry, retry time, current limit, and
 * clamp time. It is important to ensure that the device is properly
 * initialized before calling this function. The function returns an
 * error code if the configuration fails, which should be handled by the
 * caller.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership and is responsible
 * for ensuring the device is initialized before use.
 * @return Returns 0 on success or a negative error code if the configuration
 * fails.
 ******************************************************************************/
int max22516_setup_do_pnp(struct max22516_dev *dev);

/* MAX22516 setup do npn */
/***************************************************************************//**
 * @brief This function sets up the MAX22516 device to operate in NPN digital
 * output mode by writing specific configuration values to the device's
 * control registers. It should be called when the device needs to be
 * configured for NPN mode, typically during initialization or
 * reconfiguration. The function requires a valid device descriptor and
 * returns an error code if the configuration fails. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership and is responsible
 * for ensuring the device is initialized.
 * @return Returns 0 on success or a negative error code if the configuration
 * fails.
 ******************************************************************************/
int max22516_setup_do_npn(struct max22516_dev *dev);

/* MAX22516 do set */
/***************************************************************************//**
 * @brief This function sets the digital output level of the MAX22516 device to
 * the specified level. It should be used when you need to control the
 * digital output state of the device. The function requires a valid
 * device descriptor and a level value to be provided. It is important to
 * ensure that the device has been properly initialized before calling
 * this function. The function returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param lvl An 8-bit unsigned integer representing the desired digital output
 * level. Valid values depend on the specific configuration and
 * capabilities of the device.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max22516_do_set(struct max22516_dev *dev, uint8_t lvl);

/* MAX22516 do get */
/***************************************************************************//**
 * @brief This function is used to obtain the current digital output level from
 * a MAX22516 device. It should be called when the user needs to know the
 * state of the digital output. The function requires a valid device
 * descriptor and a pointer to store the output level. It reads the
 * relevant register from the device and extracts the digital output
 * level, storing it in the provided pointer. The function returns an
 * error code if the read operation fails, otherwise it returns 0.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param lvl A pointer to a uint8_t where the digital output level will be
 * stored. Must not be null. The function writes the output level to
 * this location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max22516_do_get(struct max22516_dev *dev, uint8_t *lvl);

/* MAX22516 get do stat */
/***************************************************************************//**
 * @brief This function is used to obtain the status of the digital output from
 * a MAX22516 device. It should be called when the digital output status
 * needs to be checked. The function requires a valid device descriptor
 * and a pointer to store the status. It reads the status from the device
 * and extracts the relevant information, storing it in the provided
 * pointer. Ensure that the device has been properly initialized before
 * calling this function.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param status3 A pointer to a uint8_t where the digital output status will be
 * stored. Must not be null. The function writes the status to
 * this location.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int max22516_get_do_stat(struct max22516_dev *dev, uint8_t *status3);

/* MAX22516 set event */
/***************************************************************************//**
 * @brief This function configures an event on the MAX22516 device by writing
 * the specified event qualifier and event code to the appropriate
 * registers. It should be called when an event needs to be set on the
 * device, ensuring that the device is properly initialized before use.
 * The function performs multiple register writes and returns an error
 * code if any write operation fails, allowing the caller to handle such
 * errors appropriately.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null, and the device must be initialized before
 * calling this function.
 * @param ev_qual An 8-bit unsigned integer representing the event qualifier.
 * Valid values depend on the specific application requirements.
 * @param ev_code A 16-bit unsigned integer representing the event code. Valid
 * values depend on the specific application requirements.
 * @return Returns 0 on success or a negative error code if any register write
 * operation fails.
 ******************************************************************************/
int max22516_set_event(struct max22516_dev *dev, uint8_t ev_qual,
		       uint16_t ev_code);

/* MAX22516 setup watchdog */
/***************************************************************************//**
 * @brief This function sets up the watchdog timer on a MAX22516 device,
 * allowing for configuration of the timeout, clear, event enable, and
 * event flag settings. It should be called when the watchdog
 * functionality needs to be initialized or reconfigured. The function
 * requires a valid device descriptor and specific configuration
 * parameters. It handles enabling or disabling event flags based on the
 * provided parameters. The function returns an error code if any of the
 * operations fail, ensuring that the caller can handle such cases
 * appropriately.
 *
 * @param dev A pointer to a max22516_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param wd_timeout A uint8_t value representing the watchdog timeout setting.
 * The valid range and specific values depend on the device's
 * capabilities.
 * @param wd_clr A uint8_t value used to clear the watchdog timer. The specific
 * value required depends on the device's configuration.
 * @param wd_event_en A uint8_t value indicating whether the watchdog event is
 * enabled (1) or disabled (0). Any other value is considered
 * invalid and will not change the event enable state.
 * @param wd_event_flag A uint8_t value representing the event flag to be set
 * when the watchdog event is enabled. The valid range and
 * specific values depend on the device's capabilities.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if an operation fails.
 ******************************************************************************/
int max22516_setup_watchdog(struct max22516_dev *dev, uint8_t wd_timeout,
			    uint8_t wd_clr, uint8_t wd_event_en,
			    uint8_t wd_event_flag);

/* MAX22516 Initialization */
/***************************************************************************//**
 * @brief This function initializes a MAX22516 device using the provided
 * initialization parameters. It allocates memory for the device
 * structure and sets up the SPI interface as specified in the
 * initialization parameters. This function must be called before any
 * other operations on the MAX22516 device. If the initialization fails,
 * the function returns an error code and no device is created. Ensure
 * that the device pointer is valid and that the initialization
 * parameters are correctly set before calling this function.
 *
 * @param device A pointer to a pointer of a max22516_dev structure. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a max22516_init_param structure containing the
 * initialization parameters for the device. Must not be null
 * and should be properly initialized with valid SPI
 * parameters.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and the device pointer is not set.
 ******************************************************************************/
int max22516_init(struct max22516_dev **device,
		  struct max22516_init_param *init_param);

/* MAX22516 Resources Deallocation */
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with a
 * MAX22516 device when it is no longer needed. This function should be
 * called to clean up after a device has been initialized and used,
 * ensuring that any allocated memory and associated SPI resources are
 * freed. It is important to call this function to prevent memory leaks
 * and to ensure that the SPI descriptor is properly removed. The
 * function returns an error code if the SPI removal fails, otherwise it
 * returns 0.
 *
 * @param dev A pointer to a max22516_dev structure representing the device to
 * be removed. Must not be null. The caller relinquishes ownership of
 * the memory, which will be freed by this function.
 * @return Returns 0 on success, or a negative error code if the SPI removal
 * fails.
 ******************************************************************************/
int max22516_remove(struct max22516_dev *dev);

#endif /* MAX22516_H_ */
