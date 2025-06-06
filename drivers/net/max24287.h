#ifndef _MAX24287_H_
#define _MAX24287_H_

#include <stdint.h>
#include "no_os_util.h"
#include "no_os_mdio.h"

#define MAX24287_REG(page, addr)	(((page) << 5) | ((addr) & 0x1f))
#define MAX24287_PAGE(addr)		((addr) >> 5)
#define MAX24287_ADDR(addr)		((addr) & 0x1f)

#define MAX24287_BMCR		0
#define MAX24287_DP_RST_MASK	NO_OS_BIT(15)
#define MAX24287_AN_EN_MASK	NO_OS_BIT(12)
#define MAX24287_AN_START_MASK	NO_OS_BIT(9)

#define MAX24287_BMSR		1
#define MAX24287_LINK_ST_MASK	NO_OS_BIT(2)

#define MAX24287_ID1		2
#define MAX24287_ID2		3

#define MAX24287_AN_ADV		4
#define MAX24287_AN_ADV_W_MASK	NO_OS_BIT(0)
#define MAX24287_LK_MASK	NO_OS_BIT(15)
#define MAX24287_DPLX_MASK	NO_OS_BIT(12)
#define MAX24287_SPD_MASK	NO_OS_GENMASK(11, 10)

#define MAX24287_AN_RX		5

#define MAX24287_AN_EXP		6
#define MAX24287_AN_RX_PAGE	NO_OS_BIT(1)

#define MAX24287_EXT_STAT	15
#define MAX24287_JIT_DIAG	16

#define MAX24287_PCSCR		17
#define MAX24287_WD_DIS_MASK	NO_OS_BIT(6)
#define MAX24287_BASEX_MASK	NO_OS_BIT(4)
#define MAX24287_EN_CDET_MASK	NO_OS_BIT(0)

#define MAX24287_GMIICR			18
#define MAX24287_GMIICR_W_MASK		NO_OS_BIT(7)
#define MAX24287_GMIICR_SPD_MASK	NO_OS_GENMASK(15, 14)
#define MAX24287_GMIICR_DTE_DCE_MASK	NO_OS_BIT(12)
#define MAX24287_GMIICR_DDR_MASK	NO_OS_BIT(11)
#define MAX24287_GMIICR_TXCLK_EN_MASK	NO_OS_BIT(10)

#define MAX24287_CR		19

#define MAX24287_IR		20
#define MAX24287_PAGE_IE	NO_OS_BIT(10)

#define MAX24287_ID		MAX24287_REG(1, 16)

#define MAX24287_GPIOCR1	MAX24287_REG(1, 17)
#define MAX24287_RST_MASK	NO_OS_BIT(15)
#define MAX24287_GPO1_SEL	NO_OS_GENMASK(14, 12)
#define MAX24287_GPO2_SEL	NO_OS_GENMASK(11, 9)
#define MAX24287_GPIO1_SEL	NO_OS_GENMASK(8, 6)
#define MAX24287_GPIO2_SEL	NO_OS_GENMASK(5, 3)
#define MAX24287_GPIO3_SEL	NO_OS_GENMASK(2, 0)

#define MAX24287_GPIOCR2	MAX24287_REG(1, 18)
#define MAX24287_GPIOSR		MAX24287_REG(1, 19)

#define MAX24287_PTPCR1		MAX24287_REG(2, 16)
#define MAX24287_PTPCR1_W_MASK	NO_OS_BIT(14)
#define MAX24287_PLL_PWDN_MASK	NO_OS_BIT(5)
#define MAX24287_TX_PWDN_MASK	NO_OS_BIT(3)
#define MAX24287_RX_PWDN_MASK	NO_OS_BIT(2)

#define MAX24287_PAGESEL	31
#define MAX24287_PAGE_MASK	NO_OS_GENMASK(1, 0)

/***************************************************************************//**
 * @brief The `max24287_parallel` enumeration defines various parallel interface
 * modes supported by the MAX24287 device, which is a type of Ethernet
 * PHY. Each enumerator corresponds to a specific interface standard,
 * allowing the device to be configured for different types of network
 * connections. This enumeration is used to specify the desired parallel
 * interface mode when initializing or configuring the MAX24287 device.
 *
 * @param MAX24287_TBI Represents the TBI (Ten Bit Interface) mode.
 * @param MAX24287_RTBI Represents the RTBI (Reduced Ten Bit Interface) mode.
 * @param MAX24287_GMII Represents the GMII (Gigabit Media Independent
 * Interface) mode.
 * @param MAX24287_RGMII Represents the RGMII (Reduced Gigabit Media Independent
 * Interface) mode.
 * @param MAX24287_MII Represents the MII (Media Independent Interface) mode.
 ******************************************************************************/
enum max24287_parallel {
	MAX24287_TBI,
	MAX24287_RTBI,
	MAX24287_GMII,
	MAX24287_RGMII,
	MAX24287_MII
};

/***************************************************************************//**
 * @brief The `max24287_serial` enumeration defines the serial interface modes
 * supported by the MAX24287 device, specifically SGMII and 1000BASE-X.
 * These modes are used to configure the serial communication interface
 * of the device, allowing it to operate in different network
 * environments and with various types of physical media.
 *
 * @param MAX24287_SGMII Represents the SGMII (Serial Gigabit Media Independent
 * Interface) mode.
 * @param MAX24287_1000BASEX Represents the 1000BASE-X mode, a standard for
 * gigabit Ethernet over fiber optics.
 ******************************************************************************/
enum max24287_serial {
	MAX24287_SGMII,
	MAX24287_1000BASEX
};

/***************************************************************************//**
 * @brief The `max24287_speed` enumeration defines various network speeds and
 * duplex modes supported by the MAX24287 device. It includes options for
 * 10 Mbps, 100 Mbps, 1000 Mbps (1 Gbps), and 1250 Mbps, each available
 * in both half-duplex and full-duplex modes. This enumeration is used to
 * configure and manage the speed settings of the MAX24287 network
 * interface.
 *
 * @param MAX24287_10_HALFDUPLEX Represents a speed of 10 Mbps in half-duplex
 * mode.
 * @param MAX24287_10_FULLDUPLEX Represents a speed of 10 Mbps in full-duplex
 * mode.
 * @param MAX24287_100_HALFDUPLEX Represents a speed of 100 Mbps in half-duplex
 * mode.
 * @param MAX24287_100_FULLDUPLEX Represents a speed of 100 Mbps in full-duplex
 * mode.
 * @param MAX24287_1000_HALFDUPLEX Represents a speed of 1000 Mbps (1 Gbps) in
 * half-duplex mode.
 * @param MAX24287_1000_FULLDUPLEX Represents a speed of 1000 Mbps (1 Gbps) in
 * full-duplex mode.
 * @param MAX24287_1250_HALFDUPLEX Represents a speed of 1250 Mbps in half-
 * duplex mode.
 * @param MAX24287_1250_FULLDUPLEX Represents a speed of 1250 Mbps in full-
 * duplex mode.
 ******************************************************************************/
enum max24287_speed {
	MAX24287_10_HALFDUPLEX,
	MAX24287_10_FULLDUPLEX,
	MAX24287_100_HALFDUPLEX,
	MAX24287_100_FULLDUPLEX,
	MAX24287_1000_HALFDUPLEX,
	MAX24287_1000_FULLDUPLEX,
	MAX24287_1250_HALFDUPLEX,
	MAX24287_1250_FULLDUPLEX
};

/***************************************************************************//**
 * @brief The `max24287_init_param` structure is used to initialize the MAX24287
 * device, which is a network interface component. It contains parameters
 * for configuring both the parallel and serial interfaces, including
 * their types and speeds, as well as GPIO and MDIO initialization
 * parameters for hardware control and communication.
 *
 * @param reset_param Pointer to a GPIO initialization parameter structure for
 * reset configuration.
 * @param mdio_param MDIO initialization parameter structure for MDIO interface
 * configuration.
 * @param parallel Enumeration specifying the parallel interface type.
 * @param parspeed Enumeration specifying the speed for the parallel interface.
 * @param serial Enumeration specifying the serial interface type.
 * @param serspeed Enumeration specifying the speed for the serial interface.
 ******************************************************************************/
struct max24287_init_param {
	struct no_os_gpio_init_param *reset_param;
	struct no_os_mdio_init_param mdio_param;
	enum max24287_parallel parallel;
	enum max24287_speed parspeed;
	enum max24287_serial serial;
	enum max24287_speed serspeed;
};

/***************************************************************************//**
 * @brief The `max24287_desc` structure is a descriptor for the MAX24287 device,
 * encapsulating the necessary hardware interface components. It includes
 * pointers to a GPIO descriptor for handling reset operations and an
 * MDIO descriptor for managing MDIO communication, which are essential
 * for configuring and controlling the MAX24287 device.
 *
 * @param reset_gpio Pointer to a GPIO descriptor used for reset operations.
 * @param mdio Pointer to an MDIO descriptor for MDIO communication.
 ******************************************************************************/
struct max24287_desc {
	struct no_os_gpio_desc *reset_gpio;
	struct no_os_mdio_desc *mdio;
};

/***************************************************************************//**
 * @brief This function initializes a MAX24287 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function sets up the MDIO interface,
 * performs a reset (either hard or soft depending on the parameters),
 * and configures the device for parallel and serial communication. It
 * also performs a sanity check to ensure the device is functioning
 * correctly. If successful, the function allocates and returns a
 * descriptor for the device. The caller is responsible for managing the
 * memory of the descriptor and must call `max24287_remove` to clean up
 * resources when the device is no longer needed.
 *
 * @param dev A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller takes ownership of
 * the allocated descriptor and is responsible for freeing it.
 * @param param A pointer to a `max24287_init_param` structure containing
 * initialization parameters such as MDIO and reset configurations,
 * and communication settings. Must not be null. The structure must
 * be properly initialized before calling this function.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -EINVAL for invalid arguments,
 * -ENOMEM for memory allocation failure, or -EFAULT for device
 * communication errors).
 ******************************************************************************/
int max24287_init(struct max24287_desc **dev,
		  struct max24287_init_param *param);
int max24287_remove(struct max24287_desc *dev);
/***************************************************************************//**
 * @brief Use this function to write a 16-bit value to a specific register of
 * the MAX24287 device. The function requires a valid device descriptor
 * and the register address to be specified. It handles page selection
 * automatically if the register is located on a different page. This
 * function should be called only after the device has been properly
 * initialized. It returns an integer indicating the success or failure
 * of the write operation.
 *
 * @param dev A pointer to a max24287_desc structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param addr An 8-bit unsigned integer representing the register address
 * within the device. The address is used to determine the page and
 * register within that page.
 * @param val A 16-bit unsigned integer representing the value to be written to
 * the specified register.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the write operation.
 ******************************************************************************/
int max24287_write(struct max24287_desc *dev, uint8_t addr, uint16_t val);
/***************************************************************************//**
 * @brief Use this function to read a 16-bit value from a specified register
 * address on the MAX24287 device. The function requires a valid device
 * descriptor and a register address. It handles page selection
 * automatically if the register address is on a different page. This
 * function should be called only after the device has been properly
 * initialized. The read value is stored in the location pointed to by
 * the val parameter. The function returns an error code if the read
 * operation fails.
 *
 * @param dev A pointer to a max24287_desc structure representing the device.
 * Must not be null and should be initialized before calling this
 * function.
 * @param addr A uint8_t representing the register address to read from. The
 * address is used to determine the page and register within the
 * device.
 * @param val A pointer to a uint16_t where the read value will be stored. Must
 * not be null.
 * @return Returns an integer error code. A non-zero value indicates an error
 * occurred during the read operation.
 ******************************************************************************/
int max24287_read(struct max24287_desc *dev, uint8_t addr, uint16_t *val);
/***************************************************************************//**
 * @brief Use this function to update specific bits in a register of the
 * MAX24287 device. It reads the current value from the specified
 * register, applies the bitmask to clear and set the desired bits, and
 * writes the modified value back to the register. This function should
 * be called when you need to change only certain bits in a register
 * without affecting the others. Ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param dev A pointer to a max24287_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param addr The address of the register to modify. Must be a valid register
 * address for the MAX24287 device.
 * @param val The value containing the bits to be set in the register. Only the
 * bits specified by the bitmask will be used.
 * @param bitmask A bitmask indicating which bits in the register should be
 * modified. Bits set to 1 in the bitmask will be affected by the
 * operation.
 * @return Returns 0 on success or a negative error code if the operation fails,
 * such as if reading or writing to the device fails.
 ******************************************************************************/
int max24287_write_bits(struct max24287_desc *dev, uint8_t addr, uint16_t val,
			uint16_t bitmask);
/***************************************************************************//**
 * @brief Use this function to perform a hardware reset on a MAX24287 device,
 * which is necessary to reinitialize the device to its default state.
 * This function should be called when a full reset of the device is
 * required, such as after a configuration change or to recover from an
 * error state. The function requires a valid device descriptor and will
 * return an error code if the reset operation fails. It is important to
 * ensure that the device descriptor is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to a max24287_desc structure representing the device to
 * reset. This must be a valid, non-null pointer to a properly
 * initialized device descriptor. If the pointer is null or the
 * descriptor is not properly initialized, the behavior is undefined.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int max24287_hard_reset(struct max24287_desc *dev);
/***************************************************************************//**
 * @brief Use this function to initiate a software reset on a MAX24287 device,
 * which is typically necessary to reinitialize the device's state
 * without requiring a full power cycle. This function should be called
 * when a reset is needed to recover from an error state or to apply new
 * configurations. It is important to ensure that the device descriptor
 * is properly initialized before calling this function. The function
 * will delay for a short period to allow the reset to take effect.
 *
 * @param dev A pointer to a struct max24287_desc representing the device to
 * reset. This must be a valid, non-null pointer to an initialized
 * device descriptor. If the pointer is invalid or null, the behavior
 * is undefined.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int max24287_soft_reset(struct max24287_desc *dev);
/***************************************************************************//**
 * @brief Use this function to set the parallel operation mode and speed for a
 * MAX24287 device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function
 * supports only the RGMII parallel mode and specific speed settings,
 * rejecting unsupported configurations with an error code. This function
 * is typically used when setting up the device for communication in a
 * network environment where RGMII is required.
 *
 * @param dev A pointer to a max24287_desc structure representing the device to
 * configure. Must not be null, and the device should be initialized
 * before use.
 * @param par An enum value of type max24287_parallel specifying the parallel
 * mode to configure. Only MAX24287_RGMII is supported; other values
 * will result in an error.
 * @param speed An enum value of type max24287_speed specifying the desired
 * speed. Only speeds up to MAX24287_1000_FULLDUPLEX are supported,
 * excluding MAX24287_1000_HALFDUPLEX; unsupported speeds will
 * result in an error.
 * @return Returns 0 on success. On failure, returns a negative error code, such
 * as -ENOTSUP, if the configuration is not supported.
 ******************************************************************************/
int max24287_config_parallel(struct max24287_desc *dev,
			     enum max24287_parallel par, enum max24287_speed speed);
/***************************************************************************//**
 * @brief Use this function to obtain the current parallel interface
 * configuration and speed settings of a MAX24287 device. It is typically
 * called after the device has been initialized and configured. The
 * function reads the relevant register to determine the settings and
 * updates the provided pointers with the current configuration. If the
 * pointers are null, the function will not attempt to update them. The
 * function returns an error code if the read operation fails, otherwise
 * it returns 0.
 *
 * @param dev A pointer to a max24287_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param par A pointer to an enum max24287_parallel variable where the current
 * parallel configuration will be stored. Can be null if the parallel
 * configuration is not needed.
 * @param speed A pointer to an enum max24287_speed variable where the current
 * speed setting will be stored. Can be null if the speed setting
 * is not needed.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max24287_get_config_parallel(struct max24287_desc *dev,
				 enum max24287_parallel *par, enum max24287_speed *speed);
/***************************************************************************//**
 * @brief Use this function to configure the serial interface of a MAX24287
 * device with the specified serial protocol and speed. This function
 * should be called after the device has been initialized. It supports
 * only the SGMII serial protocol and speeds up to 1000 Mbps full duplex.
 * If an unsupported serial protocol or speed is specified, the function
 * will return an error. Ensure that the device descriptor is valid
 * before calling this function.
 *
 * @param dev A pointer to a max24287_desc structure representing the device to
 * configure. Must not be null. The caller retains ownership.
 * @param ser An enum value of type max24287_serial specifying the serial
 * protocol to use. Currently, only MAX24287_SGMII is supported.
 * @param speed An enum value of type max24287_speed specifying the desired
 * speed and duplex mode. Valid values are up to
 * MAX24287_1000_FULLDUPLEX for the SGMII protocol.
 * @return Returns 0 on success, or a negative error code if the configuration
 * is not supported or if an error occurs during the configuration
 * process.
 ******************************************************************************/
int max24287_config_serial(struct max24287_desc *dev, enum max24287_serial ser,
			   enum max24287_speed speed);
/***************************************************************************//**
 * @brief Use this function to obtain the current serial interface configuration
 * and speed settings of a MAX24287 device. It is typically called after
 * the device has been initialized and configured. The function reads the
 * relevant registers to determine the serial mode and speed, storing the
 * results in the provided pointers. If either pointer is null, the
 * corresponding information is not retrieved. The function returns an
 * error code if the read operation fails, otherwise it returns 0.
 *
 * @param dev A pointer to a max24287_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ser A pointer to an enum max24287_serial variable where the serial
 * configuration will be stored. Can be null if the serial
 * configuration is not needed.
 * @param speed A pointer to an enum max24287_speed variable where the speed
 * setting will be stored. Can be null if the speed setting is not
 * needed.
 * @return Returns 0 on success, or a negative error code if a read operation
 * fails.
 ******************************************************************************/
int max24287_get_config_serial(struct max24287_desc *dev,
			       enum max24287_serial *ser, enum max24287_speed *speed);
/***************************************************************************//**
 * @brief Use this function to set the link status of a MAX24287 device to
 * either up or down. This function is typically called when you need to
 * change the operational state of the link, such as during
 * initialization or when responding to network events. Ensure that the
 * device descriptor is properly initialized before calling this
 * function. The function modifies the link status by writing to specific
 * registers, and it returns an error code if the operation fails.
 *
 * @param dev A pointer to a max24287_desc structure representing the device.
 * Must not be null and should be initialized before use. The caller
 * retains ownership.
 * @param up A boolean value indicating the desired link status. Pass true to
 * set the link up, or false to set it down.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a negative value indicates an error occurred during the operation.
 ******************************************************************************/
int max24287_config_link(struct max24287_desc *dev, bool up);
/***************************************************************************//**
 * @brief Use this function to determine the current status of the network link
 * associated with the specified device descriptor. It is typically
 * called to verify connectivity before attempting data transmission or
 * reception. The function must be called with a valid device descriptor
 * that has been properly initialized. If the function encounters an
 * error while reading the link status, it will return false, indicating
 * that the link is not up.
 *
 * @param dev A pointer to a max24287_desc structure representing the device.
 * This must be a valid, initialized descriptor. The function does
 * not modify the descriptor, and the caller retains ownership.
 * @return Returns true if the link is up, otherwise returns false.
 ******************************************************************************/
bool max24287_link_is_up(struct max24287_desc *dev);
/***************************************************************************//**
 * @brief Use this function to display the current register values of a MAX24287
 * device for debugging or informational purposes. It iterates over a
 * predefined set of register addresses, reads their values, and prints
 * them in a human-readable format. This function should be called when
 * you need to verify the state of the device's registers. Ensure that
 * the device descriptor is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to a max24287_desc structure representing the device.
 * Must not be null and should be properly initialized before use.
 * The function will read from the device using this descriptor.
 * @return None
 ******************************************************************************/
void max24287_regmap(struct max24287_desc *dev);

#endif