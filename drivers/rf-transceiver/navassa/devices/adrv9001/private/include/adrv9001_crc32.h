#ifndef _ADRV9001_CRC32_H_
#define _ADRV9001_CRC32_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function calculates a CRC32 checksum for a specified block of
 * memory, processing the data in chunks. It is useful for verifying data
 * integrity during transmission or storage. The function can be used
 * iteratively on multiple chunks of data by providing a seed CRC from a
 * previous calculation. The `finalCrc` parameter determines whether the
 * function returns an intermediate or final CRC value. This function
 * should be called with a valid buffer and length, and it is the
 * caller's responsibility to ensure that the buffer is correctly sized
 * and initialized.
 *
 * @param buf An array of bytes on which the CRC is calculated. The caller must
 * ensure this buffer is valid and properly initialized.
 * @param bufLen The length of the input array in bytes. It must accurately
 * reflect the size of the buffer to avoid undefined behavior.
 * @param seedCrc A seed value for the CRC calculation, allowing continuation
 * from a previous CRC calculation. Use 0 for the initial
 * calculation.
 * @param finalCrc A flag indicating whether to return the final CRC32 value (1)
 * or an intermediate value for further processing (0).
 * @return Returns a 32-bit checksum representing the CRC32 of the input data.
 ******************************************************************************/
uint32_t adrv9001_Crc32ForChunk(const uint8_t buf[], uint32_t bufLen, uint32_t seedCrc, uint8_t finalCrc);

#ifdef __cplusplus
}
#endif

#endif