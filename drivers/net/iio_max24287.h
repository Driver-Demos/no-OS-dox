#ifndef IIO_MAX24287_H
#define IIO_MAX24287_H

#include "iio.h"
#include "max24287.h"

/***************************************************************************//**
 * @brief The `max24287_iio_desc` structure is a compound data type that
 * encapsulates the necessary descriptors for interfacing with a MAX24287
 * device through the Industrial I/O (IIO) subsystem. It contains
 * pointers to both the device-specific descriptor and the IIO device
 * interface, facilitating the integration and management of the MAX24287
 * device within an IIO context.
 *
 * @param dev A pointer to a max24287_desc structure, representing the device
 * descriptor.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 ******************************************************************************/
struct max24287_iio_desc {
	struct max24287_desc *dev;
	struct iio_device *iio_dev;
};

/***************************************************************************//**
 * @brief The 'max24287_iio_init_param' structure is used to encapsulate
 * initialization parameters for setting up a MAX24287 device in an IIO
 * (Industrial Input/Output) context. It contains a single member, 'dev',
 * which is a pointer to a 'max24287_desc' structure, indicating the
 * specific device instance to be initialized.
 *
 * @param dev A pointer to a max24287_desc structure, representing the device to
 * be initialized.
 ******************************************************************************/
struct max24287_iio_init_param {
	struct max24287_desc *dev;
};

/***************************************************************************//**
 * @brief The `max24287_iio_attr_id` enumeration defines a set of constants that
 * represent different attributes related to the MAX24287 device,
 * specifically focusing on its speed and link characteristics. These
 * attributes are used to identify and manage the configuration and
 * operational parameters of the device within the IIO (Industrial I/O)
 * framework.
 *
 * @param MAX24287_IIO_ATTR_PAR_SPEED Represents the parallel speed attribute
 * for the MAX24287 device.
 * @param MAX24287_IIO_ATTR_SER_LINK Represents the serial link attribute for
 * the MAX24287 device.
 * @param MAX24287_IIO_ATTR_SER_SPEED Represents the serial speed attribute for
 * the MAX24287 device.
 ******************************************************************************/
enum max24287_iio_attr_id {
	MAX24287_IIO_ATTR_PAR_SPEED,
	MAX24287_IIO_ATTR_SER_LINK,
	MAX24287_IIO_ATTR_SER_SPEED,
};

/***************************************************************************//**
 * @brief This function initializes a MAX24287 IIO device descriptor using the
 * provided initialization parameters. It allocates memory for the
 * descriptor and the associated IIO device, setting up the necessary
 * structures for further interaction with the MAX24287 device. This
 * function must be called before any operations are performed on the IIO
 * device. If memory allocation fails, the function returns an error code
 * and no resources are allocated.
 *
 * @param iiodev A pointer to a pointer where the initialized MAX24287 IIO
 * descriptor will be stored. Must not be null. The caller takes
 * ownership of the allocated descriptor and is responsible for
 * freeing it using `max24287_iio_remove`.
 * @param init_param A pointer to a `max24287_iio_init_param` structure
 * containing the initialization parameters. Must not be null
 * and must be properly initialized before calling this
 * function.
 * @return Returns 0 on successful initialization. If memory allocation fails,
 * returns -ENOMEM and no resources are allocated.
 ******************************************************************************/
int32_t max24287_iio_init(struct max24287_iio_desc **iio_dev,
			  struct max24287_iio_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to clean up and free resources associated with a
 * MAX24287 IIO descriptor when it is no longer needed. This function
 * should be called to prevent memory leaks after the descriptor has been
 * initialized and used. It is safe to call this function with a null
 * pointer, in which case it will have no effect. Ensure that the
 * descriptor is not used after calling this function, as it will be
 * invalidated.
 *
 * @param desc A pointer to the MAX24287 IIO descriptor to be removed. This
 * pointer must have been previously initialized by
 * max24287_iio_init. If the pointer is null, the function will
 * safely do nothing. The caller retains ownership of the pointer,
 * but it will be invalidated after the function call.
 * @return Always returns 0, indicating successful deallocation.
 ******************************************************************************/
int32_t max24287_iio_remove(struct max24287_iio_desc *desc);

#endif /** IIO_MAX24287_H */