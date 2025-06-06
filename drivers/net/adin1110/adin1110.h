/******************************************************************************
 *   @file   adin1110.h
 *   @brief  Header file of the ADIN1110 driver.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
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

#ifndef _ADIN1110_H
#define _ADIN1110_H

#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_util.h"

#define ADIN1110_BUFF_LEN			1530
#define ADIN1110_ETH_ALEN			6
#define ADIN1110_ETHERTYPE_LEN			2
#define ADIN1110_ETH_HDR_LEN			14
#define ADIN1110_ADDR_FILT_LEN			16

#define ADIN1110_FCS_LEN			4
#define ADIN1110_MAC_LEN			6

#define ADIN1110_ADDR_MASK			NO_OS_GENMASK(12, 0)
#define ADIN1110_RD_FRAME_SIZE			7
#define ADIN1110_WR_FRAME_SIZE			6
#define ADIN1110_RD_HDR_SIZE			3
#define ADIN1110_WR_HDR_SIZE			2
#define ADIN1110_PHY_ID_REG			1

#define ADIN1110_PHY_ID				0x0283BC91
#define ADIN2111_PHY_ID				0x0283BCA1

#define ADIN1110_PORTS				1
#define ADIN2111_PORTS				2

#define ADIN1110_CD_MASK			NO_OS_BIT(15)
#define ADIN1110_RW_MASK			NO_OS_BIT(13)

#define ADIN1110_SOFT_RST_REG			0x3C
#define ADIN1110_RESET_REG			0x03
#define ADIN1110_SWRESET			NO_OS_BIT(0)
#define ADIN1110_SWRESET_KEY1			0x4F1C
#define ADIN1110_SWRESET_KEY2			0xC1F4
#define ADIN1110_SWRELEASE_KEY1			0x6F1A
#define ADIN1110_SWRELEASE_KEY2			0xA1F6

#define ADIN1110_SPI_CD				NO_OS_BIT(7)
#define ADIN1110_SPI_RW             		NO_OS_BIT(5)

#define ADIN1110_CONFIG1_REG			0x04
#define ADIN1110_CONFIG1_SYNC			NO_OS_BIT(15)

#define ADIN1110_CONFIG2_REG			0x06
#define ADIN2111_P2_FWD_UNK2HOST_MASK		NO_OS_BIT(12)
#define ADIN2111_PORT_CUT_THRU_EN		NO_OS_BIT(11)
#define ADIN1110_CRC_APPEND			NO_OS_BIT(5)
#define ADIN1110_FWD_UNK2HOST_MASK		NO_OS_BIT(2)

#define ADIN1110_STATUS0_REG			0x08
#define ADIN1110_STATUS0_TXPE_MASK		NO_OS_BIT(0)
#define ADIN1110_RESETC_MASK			NO_OS_BIT(6)

#define ADIN1110_STATUS1_REG			0x09
#define ADIN1110_LINK_STATE_MASK		NO_OS_BIT(0)
#define ADIN2111_P2_RX_RDY			NO_OS_BIT(17)
#define ADIN1110_SPI_ERR			NO_OS_BIT(10)
#define ADIN1110_RX_RDY				NO_OS_BIT(4)

#define ADIN1110_IMASK1_REG			0x0D
#define ADIN2111_RX_RDY_IRQ			NO_OS_BIT(17)
#define ADIN1110_SPI_ERR_IRQ			NO_OS_BIT(10)
#define ADIN1110_RX_RDY_IRQ			NO_OS_BIT(4)
#define ADIN1110_TX_RDY_IRQ			NO_OS_BIT(3)

#define ADIN1110_MDIOACC(x)			(0x20 + (x))
#define ADIN1110_MDIO_TRDONE			NO_OS_BIT(31)
#define ADIN1110_MDIO_TAERR			NO_OS_BIT(30)
#define ADIN1110_MDIO_ST			NO_OS_GENMASK(29, 28)
#define ADIN1110_MDIO_OP			NO_OS_GENMASK(27, 26)
#define ADIN1110_MDIO_PRTAD			NO_OS_GENMASK(25, 21)
#define ADIN1110_MDIO_DEVAD			NO_OS_GENMASK(20, 16)
#define ADIN1110_MDIO_DATA			NO_OS_GENMASK(15, 0)

#define ADIN1110_MMD_ACR_DEVAD_MASK		NO_OS_GENMASK(4, 0)
#define ADIN1110_MMD_ACR_FUNCTION_MASK		NO_OS_GENMASK(15, 14)
#define ADIN1110_MMD_ACCESS_MASK		NO_OS_GENMASK(15, 0)
#define ADIN1110_MMD_ACCESS_CTRL_REG		0x0D
#define ADIN1110_MMD_ACCESS_REG			0x0E

#define ADIN1110_MI_SFT_PD_MASK			NO_OS_BIT(11)
#define ADIN1110_MDIO_PHY_ID(x)			((x) + 1)
#define ADIN1110_MI_CONTROL_REG			0x0

#define ADIN1110_CRSM_SFT_PD_CNTRL_REG		0x8812
#define ADIN1110_CRSM_SFT_PD_MASK		NO_OS_BIT(0)

#define ADIN1110_TX_FSIZE_REG			0x30
#define ADIN1110_TX_REG				0x31
#define ADIN1110_TX_SPACE_REG			0x32

#define ADIN1110_FIFO_CLR_REG			0x36
#define ADIN1110_FIFO_CLR_RX_MASK		NO_OS_BIT(0)
#define ADIN1110_FIFO_CLR_TX_MASK		NO_OS_BIT(1)

#define ADIN1110_MAC_RST_STATUS_REG		0x3B

#define ADIN2111_MAC_ADDR_APPLY2PORT2		NO_OS_BIT(31)
#define ADIN1110_MAC_ADDR_APPLY2PORT		NO_OS_BIT(30)
#define ADIN2111_MAC_ADDR_TO_OTHER_PORT		NO_OS_BIT(17)
#define ADIN1110_MAC_ADDR_TO_HOST		NO_OS_BIT(16)

#define ADIN1110_MAC_ADDR_FILT_UPR_REG(x)	(0x50 + 2 * (x))
#define ADIN1110_MAC_ADDR_FILT_LWR_REG(x)	(0x51 + 2 * (x))

#define ADIN1110_MAC_ADDR_UPR_MASK		NO_OS_GENMASK(15, 0)
#define ADIN1110_MAC_ADDR_LWR_MASK		NO_OS_GENMASK(31, 0)

#define ADIN1110_MAC_ADDR_MASK_UPR_REG		0x70
#define ADIN1110_MAC_ADDR_MASK_LWR_REG		0x71

#define ADIN1110_RX_FRM_CNT_REG			0xA0
#define ADIN1110_RX_CRC_ERR_CNT_REG		0xA4
#define ADIN1110_RX_ALGN_ERR_CNT_REG		0xA5
#define ADIN1110_RX_LS_ERR_CNT_REG		0xA6
#define ADIN1110_RX_PHY_ERR_CNT_REG		0xA7
#define ADIN1110_TX_FRM_CNT_REG			0xA8
#define ADIN1110_TX_BCAST_CNT_REG		0xA9
#define ADIN1110_TX_MCAST_CNT_REG		0xAA
#define ADIN1110_TX_UCAST_CNT_REG		0xAB
#define ADIN1110_RX_BCAST_CNT_REG		0xA1
#define ADIN1110_RX_MCAST_CNT_REG		0xA2
#define ADIN1110_RX_UCAST_CNT_REG		0xA3

#define ADIN1110_RX_DROP_FULL_CNT_REG		0xAC
#define ADIN1110_RX_DROP_FILT_CNT_REG		0xAD

#define ADIN1110_RX_FSIZE_REG			0x90
#define ADIN1110_RX_REG				0x91

#define ADIN2111_RX_P2_FSIZE_REG		0xC0
#define ADIN2111_RX_P2_REG			0xC1

#define ADIN1110_CLEAR_STATUS0			0xFFF

/* MDIO_OP codes */
#define ADIN1110_MDIO_OP_ADDR			0x0
#define ADIN1110_MDIO_OP_WR			0x1
#define ADIN1110_MDIO_OP_RD			0x3

#define ADIN1110_WR_HEADER_LEN			2
#define ADIN1110_FRAME_HEADER_LEN		2
#define ADIN1110_RD_HEADER_LEN			3
#define ADIN1110_REG_LEN			4
#define ADIN1110_CRC_LEN			1
#define ADIN1110_FEC_LEN			4

#define ADIN_MAC_MULTICAST_ADDR_SLOT		0
#define ADIN_MAC_BROADCAST_ADDR_SLOT		1
#define ADIN_MAC_P1_ADDR_SLOT			2
#define ADIN_MAC_P2_ADDR_SLOT			3
#define ADIN_MAC_FDB_ADDR_SLOT			4

/***************************************************************************//**
 * @brief The `adin1110_chip_id` enumeration defines identifiers for different
 * chip models supported by the ADIN1110 driver, specifically the
 * ADIN1110 and ADIN2111 chips. This enumeration is used to distinguish
 * between the two chip types when initializing or configuring the
 * device, ensuring that the correct settings and operations are applied
 * for each specific chip model.
 *
 * @param ADIN1110 Represents the ADIN1110 chip identifier.
 * @param ADIN2111 Represents the ADIN2111 chip identifier.
 ******************************************************************************/
enum adin1110_chip_id {
	ADIN1110,
	ADIN2111,
};

/***************************************************************************//**
 * @brief The `adin1110_desc` structure is a device descriptor for the ADIN1110
 * Ethernet device, encapsulating essential configuration and state
 * information. It includes the chip type, communication descriptor for
 * SPI, MAC address, data buffer, reset GPIO descriptor, and a flag for
 * CRC appending. This structure is central to managing the device's
 * operation and communication, providing a comprehensive representation
 * of the device's configuration and state.
 *
 * @param chip_type Specifies the type of ADIN1110 chip being used.
 * @param comm_desc Pointer to a SPI descriptor for communication.
 * @param mac_address Array holding the MAC address of the device.
 * @param data Buffer for storing data with a length defined by
 * ADIN1110_BUFF_LEN.
 * @param reset_gpio Pointer to a GPIO descriptor for the reset pin.
 * @param append_crc Boolean flag indicating whether to append CRC to frames.
 ******************************************************************************/
struct adin1110_desc {
	enum adin1110_chip_id chip_type;
	struct no_os_spi_desc *comm_desc;
	uint8_t mac_address[ADIN1110_ETH_ALEN];
	uint8_t data[ADIN1110_BUFF_LEN];
	struct no_os_gpio_desc *reset_gpio;
	bool append_crc;
};

/***************************************************************************//**
 * @brief The `adin1110_init_param` structure is used to initialize an ADIN1110
 * device descriptor, specifying the chip type, communication parameters,
 * reset parameters, MAC address, and whether to append a CRC to frames.
 * This structure is essential for setting up the device with the correct
 * configuration before it can be used for network communication.
 *
 * @param chip_type Specifies the type of ADIN1110 chip being used.
 * @param comm_param Holds the SPI communication parameters for initialization.
 * @param reset_param Contains the GPIO parameters for the reset operation.
 * @param mac_address Stores the MAC address of the device as an array of bytes.
 * @param append_crc Indicates whether a CRC should be appended to transmitted
 * frames.
 ******************************************************************************/
struct adin1110_init_param {
	enum adin1110_chip_id chip_type;
	struct no_os_spi_init_param comm_param;
	struct no_os_gpio_init_param reset_param;
	uint8_t mac_address[ADIN1110_ETH_ALEN];
	bool append_crc;
};

/***************************************************************************//**
 * @brief The `adin1110_eth_buff` structure is used to represent an Ethernet
 * frame for transmission and reception in the ADIN1110 driver. It
 * contains fields for the frame's length, destination and source MAC
 * addresses, the ethertype, and a pointer to the payload data. This
 * structure is essential for handling Ethernet frames in network
 * communication, allowing the driver to manage data packets effectively.
 *
 * @param len Stores the length of the Ethernet frame.
 * @param mac_dest Holds the destination MAC address of the Ethernet frame.
 * @param mac_source Holds the source MAC address of the Ethernet frame.
 * @param ethertype Indicates the protocol type encapsulated in the payload.
 * @param payload Pointer to the data payload of the Ethernet frame.
 ******************************************************************************/
struct adin1110_eth_buff {
	uint32_t len;
	uint8_t mac_dest[ADIN1110_ETH_ALEN];
	uint8_t mac_source[ADIN1110_ETH_ALEN];
	uint8_t ethertype[2];
	uint8_t *payload;
};

/* Reset both the MAC and PHY. */
/***************************************************************************//**
 * @brief Use this function to reset both the MAC and PHY components of an
 * ADIN1110 device. This operation is typically performed to restore the
 * device to a known state, especially after configuration changes or
 * error conditions. Ensure that the device descriptor is properly
 * initialized before calling this function. The function will attempt to
 * write a reset command to the device and return a status code
 * indicating the success or failure of the operation.
 *
 * @param desc A pointer to an initialized `adin1110_desc` structure
 * representing the device to be reset. This parameter must not be
 * null, and the structure should be properly configured before use.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the reset operation fails.
 ******************************************************************************/
int adin1110_sw_reset(struct adin1110_desc *);

/* Update a register's value based on a mask */
/***************************************************************************//**
 * @brief This function is used to update a specific register in the ADIN1110
 * device by applying a mask and new data to the current register value.
 * It is typically called when a specific bit or set of bits in a
 * register needs to be modified without affecting other bits. The
 * function first reads the current value of the register, applies the
 * mask to clear the bits to be updated, and then sets the new bits
 * according to the provided data. It is important to ensure that the
 * device descriptor is properly initialized before calling this
 * function.
 *
 * @param desc A pointer to an initialized adin1110_desc structure representing
 * the device. Must not be null.
 * @param addr The address of the register to be updated. Must be a valid
 * register address for the ADIN1110 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected by the data
 * parameter.
 * @param data The new data to be written to the register, masked by the mask
 * parameter. Only bits corresponding to 1s in the mask will be
 * written.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adin1110_reg_update(struct adin1110_desc *, uint16_t, uint32_t, uint32_t);

/* Write a register's value */
/***************************************************************************//**
 * @brief Use this function to write a 32-bit value to a specific register of
 * the ADIN1110 device. This operation is typically performed when
 * configuring the device or updating its settings. The function requires
 * a valid device descriptor and a register address. It supports optional
 * CRC appending for data integrity, which is controlled by the device
 * descriptor. Ensure the device is properly initialized before calling
 * this function.
 *
 * @param desc A pointer to an adin1110_desc structure representing the device.
 * Must not be null. The structure should be properly initialized
 * and configured before use.
 * @param addr A 16-bit unsigned integer representing the register address to
 * write to. The address is masked and modified internally to fit
 * the device's addressing scheme.
 * @param data A 32-bit unsigned integer representing the data to be written to
 * the specified register.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a negative value indicates an error during the SPI transfer.
 ******************************************************************************/
int adin1110_reg_write(struct adin1110_desc *, uint16_t, uint32_t);

/* Read a register's value */
/***************************************************************************//**
 * @brief This function retrieves the value of a specified register from an
 * ADIN1110 device using SPI communication. It should be called when you
 * need to read a register value from the device. Ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function supports optional CRC validation if the descriptor is
 * configured to append CRC. If CRC validation is enabled and fails, the
 * function returns an error.
 *
 * @param desc A pointer to an initialized `adin1110_desc` structure
 * representing the device. Must not be null.
 * @param addr The 16-bit address of the register to read. Must be a valid
 * register address for the ADIN1110 device.
 * @param data A pointer to a 32-bit unsigned integer where the read register
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI transfer
 * fails or if CRC validation fails (when enabled).
 ******************************************************************************/
int adin1110_reg_read(struct adin1110_desc *, uint16_t, uint32_t *);

/* Write a frame to the TX FIFO */
/***************************************************************************//**
 * @brief This function is used to transmit an Ethernet frame by writing it to
 * the transmit FIFO of the ADIN1110 device. It should be called when
 * there is a need to send a frame over the network. The function checks
 * if there is enough space in the TX FIFO before attempting to write the
 * frame. If the frame is smaller than the minimum Ethernet frame size,
 * it will be padded to meet the minimum size requirement. The function
 * also aligns the frame length to a 4-byte boundary. It is important to
 * ensure that the `desc` and `eth_buff` parameters are properly
 * initialized and valid before calling this function. The function
 * returns an error code if the operation fails, such as when there is
 * insufficient space in the FIFO or if the specified port is invalid.
 *
 * @param desc A pointer to an `adin1110_desc` structure that describes the
 * ADIN1110 device. Must not be null and should be properly
 * initialized before calling this function.
 * @param port A 32-bit unsigned integer specifying the port on which to send
 * the frame. Must be less than the number of ports supported by the
 * device; otherwise, the function returns an error.
 * @param eth_buff A pointer to an `adin1110_eth_buff` structure containing the
 * Ethernet frame to be transmitted. Must not be null and should
 * contain a valid frame with a length specified in the `len`
 * field.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters or -EAGAIN if there is insufficient
 * space in the TX FIFO.
 ******************************************************************************/
int adin1110_write_fifo(struct adin1110_desc *, uint32_t,
			struct adin1110_eth_buff *);

/* Read a frame from the RX FIFO */
/***************************************************************************//**
 * @brief This function reads a frame from the RX FIFO of a specified port on an
 * ADIN1110 device and stores it in the provided Ethernet buffer. It
 * should be called when a frame needs to be retrieved from the device's
 * FIFO. The function requires a valid device descriptor and a port
 * number within the range supported by the device. The Ethernet buffer
 * must be prepared to receive the frame data. If the port number is
 * invalid or an error occurs during the read operation, the function
 * returns a negative error code. The function does not modify the device
 * descriptor or the port configuration.
 *
 * @param desc A pointer to an adin1110_desc structure representing the device
 * descriptor. Must not be null and should be properly initialized
 * before calling this function.
 * @param port A uint32_t representing the port number from which to read the
 * frame. Must be within the range of available ports for the
 * device; otherwise, the function returns -EINVAL.
 * @param eth_buff A pointer to an adin1110_eth_buff structure where the read
 * frame will be stored. Must not be null and should have
 * sufficient space to store the frame data.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the Ethernet buffer is populated with the frame data.
 ******************************************************************************/
int adin1110_read_fifo(struct adin1110_desc *, uint32_t,
		       struct adin1110_eth_buff *);

/* Write a PHY register using clause 22 */
/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specified
 * register of a PHY device identified by its PHY ID, using the MDIO
 * clause 22 protocol. It is typically called when there is a need to
 * configure or modify the settings of a PHY device. The function
 * requires a valid device descriptor and specific register and data
 * values. It ensures the completion of the write operation by polling
 * the transaction done bit. The function should be used in environments
 * where the ADIN1110 device is properly initialized and configured.
 *
 * @param desc A pointer to an adin1110_desc structure representing the device
 * descriptor. Must not be null and should be properly initialized
 * before calling this function.
 * @param phy_id A 32-bit unsigned integer representing the PHY ID of the target
 * device. It should be within the valid range for PHY IDs.
 * @param reg A 32-bit unsigned integer specifying the register address within
 * the PHY to write to. Must be a valid register address for the
 * target PHY.
 * @param data A 16-bit unsigned integer containing the data to be written to
 * the specified register. The data should be formatted according to
 * the register's requirements.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adin1110_mdio_write(struct adin1110_desc *, uint32_t, uint32_t, uint16_t);

/* Read a PHY register using clause 22 */
/***************************************************************************//**
 * @brief This function reads a PHY register specified by the given register
 * address from a PHY device identified by the PHY ID using the MDIO
 * clause 22 protocol. It should be called when you need to retrieve the
 * value of a specific register from a PHY device. Ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function will block until the read operation is complete, and it will
 * return an error code if the operation fails.
 *
 * @param desc A pointer to an initialized `adin1110_desc` structure
 * representing the device. Must not be null.
 * @param phy_id The PHY ID of the target device. It should be a valid
 * identifier for the PHY device you intend to communicate with.
 * @param reg The register address within the PHY to read from. It should be a
 * valid register address for the target PHY.
 * @param data A pointer to a `uint16_t` where the read data will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adin1110_mdio_read(struct adin1110_desc *, uint32_t, uint32_t, uint16_t *);

/* Write a PHY register using clause 45 */
/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specified
 * register of a PHY device using the MDIO clause 45 protocol. It is
 * typically called when there is a need to configure or modify the
 * settings of a PHY device. The function requires a valid device
 * descriptor and specific identifiers for the PHY, device, and register.
 * It ensures the completion of the write transaction before returning.
 * The function should be called only after the device has been properly
 * initialized.
 *
 * @param desc A pointer to an `adin1110_desc` structure representing the device
 * descriptor. Must not be null. The caller retains ownership.
 * @param phy_id A 32-bit identifier for the PHY device. Must be a valid PHY ID.
 * @param dev_id A 32-bit identifier for the device within the PHY. Must be a
 * valid device ID.
 * @param reg A 32-bit register address within the device. Must be a valid
 * register address.
 * @param data A 16-bit data value to be written to the specified register. Any
 * 16-bit value is valid.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adin1110_mdio_write_c45(struct adin1110_desc *, uint32_t, uint32_t,
			    uint32_t,
			    uint16_t);

/* Read a PHY register using clause 45 */
/***************************************************************************//**
 * @brief This function is used to read a PHY register from a device using the
 * MDIO clause 45 protocol. It requires a valid device descriptor and
 * specific identifiers for the PHY and device, as well as the register
 * address to be read. The function will store the read data in the
 * provided data pointer. It is important to ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function will return an error code if the read operation fails.
 *
 * @param desc A pointer to an initialized adin1110_desc structure representing
 * the device. Must not be null.
 * @param phy_id The PHY identifier. Must be a valid identifier for the target
 * PHY.
 * @param dev_id The device identifier within the PHY. Must be a valid
 * identifier for the target device.
 * @param reg The register address to read from. Must be a valid register
 * address for the target device.
 * @param data A pointer to a uint16_t where the read data will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code on failure. The read
 * data is stored in the location pointed to by the data parameter.
 ******************************************************************************/
int adin1110_mdio_read_c45(struct adin1110_desc *, uint32_t, uint32_t, uint16_t,
			   uint16_t *);

/* Get the link state for a given port */
/***************************************************************************//**
 * @brief This function is used to obtain the current link state of an ADIN1110
 * device. It should be called when you need to check the connectivity
 * status of the device. The function requires a valid device descriptor
 * and a pointer to store the link state. It reads the link state from
 * the device and updates the provided state variable. Ensure that the
 * device descriptor is properly initialized before calling this
 * function. The function returns an error code if the operation fails,
 * otherwise it returns 0 indicating success.
 *
 * @param desc A pointer to an initialized `adin1110_desc` structure
 * representing the device. This must not be null and should be
 * properly configured before calling the function.
 * @param state A pointer to a `uint32_t` variable where the link state will be
 * stored. This must not be null. The function will update the
 * value pointed to by this parameter with the current link state.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adin1110_link_state(struct adin1110_desc *, uint32_t *);

/* Set a port in promiscuous mode. All MAC filters are dropped */
/***************************************************************************//**
 * @brief This function configures a specified port on the ADIN1110 device to
 * operate in promiscuous mode, where all incoming frames are forwarded
 * to the host, bypassing MAC address filtering. It should be used when
 * it is necessary to receive all network traffic on a port, such as for
 * network monitoring or debugging purposes. The function must be called
 * with a valid device descriptor and a port number within the range
 * supported by the device. If the port number is invalid, the function
 * returns an error code.
 *
 * @param desc A pointer to an adin1110_desc structure representing the device.
 * Must not be null, and must be properly initialized before calling
 * this function.
 * @param port The port number to configure. Must be less than the number of
 * ports supported by the device. If the port number is invalid, the
 * function returns -EINVAL.
 * @param promisc A boolean value indicating whether to enable (true) or disable
 * (false) promiscuous mode on the specified port.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if the port number is invalid.
 ******************************************************************************/
int adin1110_set_promisc(struct adin1110_desc *, uint32_t, bool);

/*
 * Set a MAC filter. The frames with destination MAC addresses matching this will be
 * forwarded to the host.
 */
/***************************************************************************//**
 * @brief This function configures a MAC address filter on the ADIN1110 device,
 * allowing frames with the specified MAC address to be forwarded to the
 * host. It should be used when you need to ensure that frames with a
 * specific destination MAC address are processed by the host. The
 * function searches for an available filter slot and writes the MAC
 * address into it. It must be called with a valid device descriptor and
 * a properly formatted MAC address. The function returns an error code
 * if it fails to find a free slot or encounters a read/write error.
 *
 * @param desc A pointer to an initialized `adin1110_desc` structure
 * representing the device. Must not be null.
 * @param mac_address An array of 6 bytes representing the MAC address to be
 * set. Must be a valid MAC address and not null.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * issues such as no available filter slots or read/write errors.
 ******************************************************************************/
int adin1110_set_mac_addr(struct adin1110_desc *desc,
			  uint8_t mac_address[ADIN1110_ETH_ALEN]);

/* Enable/disable the forwarding (to host) of broadcast frames */
/***************************************************************************//**
 * @brief This function is used to control whether broadcast frames are
 * forwarded to the host by enabling or disabling the broadcast filter.
 * It should be called when there is a need to manage the handling of
 * broadcast frames in the network. The function requires a valid device
 * descriptor and a boolean flag indicating whether to enable or disable
 * the filter. It returns an integer status code indicating success or
 * failure of the operation.
 *
 * @param desc A pointer to a valid 'adin1110_desc' structure representing the
 * device descriptor. Must not be null.
 * @param enabled A boolean value where 'true' enables the broadcast filter and
 * 'false' disables it.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int adin1110_broadcast_filter(struct adin1110_desc *, bool);

/* Reset the MAC device */
/***************************************************************************//**
 * @brief Use this function to reset the MAC device associated with the given
 * descriptor. This operation is typically performed to ensure the MAC is
 * in a known state, especially after initialization or when recovering
 * from an error condition. The function must be called with a valid
 * device descriptor that has been properly initialized. It attempts to
 * reset the MAC by writing specific reset keys to the appropriate
 * register and checks the reset status to confirm success. If the reset
 * is unsuccessful, an error code is returned.
 *
 * @param desc A pointer to a valid 'adin1110_desc' structure representing the
 * device to be reset. This parameter must not be null, and the
 * descriptor should be properly initialized before calling this
 * function. If the descriptor is invalid, the function will return
 * an error code.
 * @return Returns 0 on successful reset. If the reset fails, a negative error
 * code is returned, such as -EBUSY if the MAC reset status indicates
 * the reset did not complete successfully.
 ******************************************************************************/
int adin1110_mac_reset(struct adin1110_desc *);

/* Reset the PHY device */
/***************************************************************************//**
 * @brief This function is used to reset the PHY device associated with the
 * ADIN1110 descriptor. It should be called when a PHY reset is required,
 * such as during initialization or when recovering from an error state.
 * The function performs a reset sequence by toggling the reset GPIO and
 * verifies the PHY ID to ensure the reset was successful. It is
 * important to ensure that the `desc` parameter is properly initialized
 * and that the reset GPIO is correctly configured before calling this
 * function. The function returns an error code if the reset fails or if
 * the PHY ID does not match the expected value.
 *
 * @param desc A pointer to an `adin1110_desc` structure that must be properly
 * initialized. This structure contains the reset GPIO configuration
 * and other necessary device information. The pointer must not be
 * null, and the function will return an error if the reset GPIO
 * operations fail or if the PHY ID verification fails.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails or if the PHY ID does not match the expected value.
 ******************************************************************************/
int adin1110_phy_reset(struct adin1110_desc *);

/* Initialize the device */
/***************************************************************************//**
 * @brief This function sets up and initializes an ADIN1110 device using the
 * provided initialization parameters. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * descriptor, configures the reset GPIO if provided, initializes the SPI
 * communication, and sets up the MAC and PHY layers. If the reset GPIO
 * is not provided, a software reset is performed. The function handles
 * errors by cleaning up resources and returning an appropriate error
 * code. It is essential to ensure that the `mac_address` in the
 * initialization parameters is not null before calling this function.
 *
 * @param desc A pointer to a pointer of type `struct adin1110_desc`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated descriptor.
 * @param param A pointer to a `struct adin1110_init_param` containing
 * initialization parameters such as the MAC address, SPI, and GPIO
 * configurations. The `mac_address` must not be null, and the
 * caller retains ownership of this structure.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as `-EINVAL`
 * for invalid parameters or `-ENOMEM` for memory allocation failures.
 ******************************************************************************/
int adin1110_init(struct adin1110_desc **, struct adin1110_init_param *);

/* Free a device descriptor */
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * ADIN1110 device descriptor when it is no longer needed. This function
 * should be called to prevent resource leaks after the device has been
 * initialized and used. It handles the cleanup of SPI and GPIO resources
 * if they were allocated, and frees the memory associated with the
 * descriptor. Ensure that the descriptor is valid and initialized before
 * calling this function.
 *
 * @param desc A pointer to an adin1110_desc structure representing the device
 * descriptor to be removed. Must not be null. If the pointer is
 * null, the function returns an error code.
 * @return Returns 0 on successful removal of the descriptor and its resources.
 * If the descriptor is null, returns -EINVAL. If an error occurs during
 * the removal of SPI or GPIO resources, the function returns the
 * corresponding error code.
 ******************************************************************************/
int adin1110_remove(struct adin1110_desc *);

#endif