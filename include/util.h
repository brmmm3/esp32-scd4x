#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CRC8_POLYNOMIAL             (0x31)
#define CRC8_INIT                   (0xFF)

uint8_t scd4x_calc_cksum(const uint8_t* data, uint16_t count);

uint8_t scd4x_is_data_valid(const uint8_t *data, uint8_t size);

uint8_t scd4x_bytes_to_data(const uint8_t *bytes, uint8_t size, uint8_t *data);

#ifdef __cplusplus
}
#endif
