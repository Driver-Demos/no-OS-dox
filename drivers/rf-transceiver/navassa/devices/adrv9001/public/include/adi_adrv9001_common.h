#ifndef _ADI_ADRV9001_COMMON_H_
#define _ADI_ADRV9001_COMMON_H_

#include "adi_adrv9001_common_types.h"
#ifndef __KERNEL__
#include <stdint.h>
#else
#include <linux/types.h>
#endif

#ifndef __maybe_unused
#define __maybe_unused		__attribute__((__unused__))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE

/***************************************************************************//**
 * @brief Use this function to map a given port enumeration to its corresponding
 * numeric index. This is useful when you need to work with numeric
 * representations of ports in your application. The function requires a
 * valid port enumeration and a non-null pointer to store the resulting
 * index. If the provided port is not recognized or the index pointer is
 * null, the function returns an error code.
 *
 * @param port The port to convert, specified as an adi_common_Port_e
 * enumeration. Must be a valid port value such as ADI_RX, ADI_TX,
 * ADI_ORX, ADI_ILB, or ADI_ELB.
 * @param index A pointer to a uint8_t where the resulting numeric index will be
 * stored. Must not be null.
 * @return Returns 0 on success, with the index parameter updated to the
 * corresponding numeric index. Returns -2 if the index pointer is null
 * or if the port is invalid.
 ******************************************************************************/
int32_t adi_common_port_to_index(adi_common_Port_e port, uint8_t *index);

/***************************************************************************//**
 * @brief This function is used to convert a channel number, represented by the
 * adi_common_ChannelNumber_e enumeration, into a corresponding numeric
 * index. It is useful when a numeric representation of a channel is
 * required for further processing or interfacing with other systems. The
 * function must be called with a valid channel enumeration and a non-
 * null pointer for the index output. If the channel is not recognized or
 * the index pointer is null, the function returns an error code.
 *
 * @param channel The channel number to convert, represented by the
 * adi_common_ChannelNumber_e enumeration. Only specific values
 * like ADI_CHANNEL_1 and ADI_CHANNEL_2 are valid.
 * @param index A pointer to a uint8_t where the resulting numeric index will be
 * stored. Must not be null, as a null pointer will result in an
 * error.
 * @return Returns 0 on success, with the index output parameter set to the
 * corresponding numeric index. Returns -2 if the index pointer is null
 * or if the channel is not recognized.
 ******************************************************************************/
int32_t adi_common_channel_to_index(adi_common_ChannelNumber_e channel, uint8_t *index);

/***************************************************************************//**
 * @brief This function is used to map a numeric index to its corresponding
 * adi_common_Port_e enumeration value. It is useful when you have an
 * index and need to determine the associated port type. The function
 * requires a valid index and a non-null pointer to store the result. If
 * the index is not within the valid range of 0 to 4, or if the port
 * pointer is null, the function returns an error code.
 *
 * @param index The numeric index to convert, expected to be in the range 0 to
 * 4. Values outside this range result in an error.
 * @param port A pointer to an adi_common_Port_e where the result will be
 * stored. Must not be null. If null, the function returns an error.
 * @return Returns 0 on success, with the port parameter set to the
 * corresponding adi_common_Port_e. Returns -2 if the index is invalid
 * or the port pointer is null.
 ******************************************************************************/
int32_t adi_common_index_to_port(uint8_t index, adi_common_Port_e *port);

/***************************************************************************//**
 * @brief Use this function to map a numeric index to its corresponding
 * adi_common_ChannelNumber_e value. It is useful when you need to
 * translate an index, typically from a configuration or input, into a
 * channel enumeration used by the system. The function expects a valid
 * index and a non-null pointer to store the result. If the index is not
 * recognized or the channel pointer is null, the function returns an
 * error code.
 *
 * @param index The numeric index to convert, expected to be 0 or 1. Values
 * outside this range will result in an error.
 * @param channel A pointer to an adi_common_ChannelNumber_e where the result
 * will be stored. Must not be null. If null, the function
 * returns an error.
 * @return Returns 0 on success, with the channel parameter set to the
 * corresponding adi_common_ChannelNumber_e. Returns -2 if the index is
 * invalid or the channel pointer is null.
 ******************************************************************************/
int32_t adi_common_index_to_channel(uint8_t index, adi_common_ChannelNumber_e *channel);

#endif

#ifdef __cplusplus
}
#endif

#endif  /* _ADI_ADRV9001_COMMON_H_ */
