#ifndef IIO_ADIN1300_H
#define IIO_ADIN1300_H

#include "iio.h"
#include "adin1300.h"

/***************************************************************************//**
 * @brief The `adin1300_iio_desc` structure is a compound data type that
 * encapsulates the relationship between an ADIN1300 device and its
 * corresponding IIO (Industrial Input/Output) device interface. It
 * contains pointers to both the ADIN1300 device descriptor and the IIO
 * device, facilitating the integration and management of the ADIN1300
 * device within an IIO framework.
 *
 * @param dev A pointer to an adin1300_desc structure, representing the ADIN1300
 * device.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 ******************************************************************************/
struct adin1300_iio_desc {
	struct adin1300_desc *dev;
	struct iio_device *iio_dev;
};

/***************************************************************************//**
 * @brief The `adin1300_iio_init_param` structure is used to encapsulate
 * initialization parameters for setting up an ADIN1300 device in an IIO
 * (Industrial I/O) context. It contains a single member, `dev`, which is
 * a pointer to an `adin1300_desc` structure, indicating the specific
 * device instance to be initialized.
 *
 * @param dev A pointer to an adin1300_desc structure, representing the device
 * to be initialized.
 ******************************************************************************/
struct adin1300_iio_init_param {
	struct adin1300_desc *dev;
};

/***************************************************************************//**
 * @brief The `adin1300_iio_attr_id` enumeration defines a set of constants
 * representing different attributes of the ADIN1300 device that can be
 * accessed or modified through the IIO (Industrial I/O) interface. These
 * attributes include link status, speed, and autonegotiation
 * capabilities, which are essential for configuring and monitoring the
 * network interface of the ADIN1300 device.
 *
 * @param ADIN1300_IIO_ATTR_LINK Represents the link status attribute of the
 * ADIN1300 device.
 * @param ADIN1300_IIO_ATTR_SPEED Represents the speed attribute of the ADIN1300
 * device.
 * @param ADIN1300_IIO_ATTR_AUTONEGOTIATE Represents the autonegotiate attribute
 * of the ADIN1300 device.
 ******************************************************************************/
enum adin1300_iio_attr_id {
	ADIN1300_IIO_ATTR_LINK,
	ADIN1300_IIO_ATTR_SPEED,
	ADIN1300_IIO_ATTR_AUTONEGOTIATE,
};

/***************************************************************************//**
 * @brief This function initializes an ADIN1300 IIO device descriptor using the
 * provided initialization parameters. It allocates memory for the
 * descriptor and the associated IIO device structure. The function must
 * be called before any operations are performed on the ADIN1300 IIO
 * device. If memory allocation fails, the function returns an error code
 * and no descriptor is created. The caller is responsible for managing
 * the memory of the descriptor, including freeing it when it is no
 * longer needed.
 *
 * @param iiodev A pointer to a pointer where the initialized ADIN1300 IIO
 * device descriptor will be stored. Must not be null. The caller
 * takes ownership of the allocated descriptor and is responsible
 * for freeing it.
 * @param init_param A pointer to an initialization parameter structure
 * containing the ADIN1300 device descriptor. Must not be
 * null. The structure should be properly initialized before
 * calling this function.
 * @return Returns 0 on success. On failure, returns a negative error code, such
 * as -ENOMEM if memory allocation fails.
 ******************************************************************************/
int32_t adin1300_iio_init(struct adin1300_iio_desc **iio_dev,
			  struct adin1300_iio_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADIN1300 IIO device descriptor when it is no longer needed. This
 * function should be called to prevent memory leaks after the descriptor
 * has been initialized and used. It is safe to call this function with a
 * null pointer, in which case it will have no effect.
 *
 * @param desc A pointer to the adin1300_iio_desc structure to be removed. This
 * pointer must have been previously initialized by
 * adin1300_iio_init. If the pointer is null, the function will
 * safely do nothing.
 * @return Returns 0 after successfully releasing the resources. The function
 * does not return error codes.
 ******************************************************************************/
int32_t adin1300_iio_remove(struct adin1300_iio_desc *desc);

#endif /** IIO_ADIN1300_H */