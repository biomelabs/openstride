#include "sdm_encode.h"

#include <errno.h>
#include <stddef.h>

static uint32_t sdm_float_to_ticks(float value, uint32_t unit_reversal) {
    float scaled;

    if (value <= 0.0f) {
        return 0U;
    }

    scaled = value * (float)unit_reversal;
    if (scaled >= (float)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)(scaled + 0.5f);
}

static uint8_t sdm_update_latency_ticks(float update_latency_s) {
    uint32_t ticks = sdm_float_to_ticks(update_latency_s, SDM_UPDATE_LATENCY_UNIT_REVERSAL);

    if (ticks > UINT8_MAX) {
        return UINT8_MAX;
    }

    return (uint8_t)ticks;
}

static void sdm_encode_speed_bytes(uint8_t *packed_byte, uint8_t *fractional_byte,
                                   float speed_mps) {
    uint32_t speed_raw = sdm_float_to_ticks(speed_mps, SDM_SPEED_UNIT_REVERSAL) & 0x0FFFu;

    *packed_byte = (uint8_t)(speed_raw >> 8) & 0x0Fu;
    *fractional_byte = (uint8_t)(speed_raw & 0xFFu);
}

static void sdm_encode_cadence_bytes(uint8_t *integer_byte, uint8_t *fractional_nibble,
                                     float cadence_spm) {
    uint32_t cadence_raw = sdm_float_to_ticks(cadence_spm, SDM_CADENCE_UNIT_REVERSAL) & 0x0FFFu;

    *integer_byte = (uint8_t)((cadence_raw >> 4) & 0xFFu);
    *fractional_nibble = (uint8_t)(cadence_raw & 0x0Fu);
}

uint8_t sdm_status_pack(enum sdm_use_state use_state, enum sdm_health_state health,
                        enum sdm_battery_state battery, enum sdm_sensor_location location) {
    return (uint8_t)(((uint8_t)use_state & 0x03u) | (((uint8_t)health & 0x03u) << 2) |
                     (((uint8_t)battery & 0x03u) << 4) | (((uint8_t)location & 0x03u) << 6));
}

int sdm_encode_page1(uint8_t payload[SDM_PAYLOAD_SIZE], const sdm_data_t *data,
                     uint16_t time_1_200s) {
    uint8_t speed_integer;
    uint8_t speed_fractional;
    uint32_t distance_raw;
    uint8_t distance_integer;
    uint8_t distance_fractional;

    if ((payload == NULL) || (data == NULL)) {
        return -EINVAL;
    }

    distance_raw = sdm_float_to_ticks(data->distance_m, SDM_DISTANCE_UNIT_REVERSAL) & 0x0FFFu;
    distance_integer = (uint8_t)((distance_raw >> 4) & 0xFFu);
    distance_fractional = (uint8_t)(distance_raw & 0x0Fu);
    sdm_encode_speed_bytes(&speed_integer, &speed_fractional, data->speed_mps);

    payload[0] = SDM_PAGE_1;
    payload[1] = (uint8_t)(time_1_200s % SDM_TIME_UNIT_REVERSAL);
    payload[2] = (uint8_t)((time_1_200s / SDM_TIME_UNIT_REVERSAL) & 0xFFu);
    payload[3] = distance_integer;
    payload[4] = (uint8_t)((distance_fractional << 4) | speed_integer);
    payload[5] = speed_fractional;
    payload[6] = (uint8_t)(data->stride_count & 0xFFu);
    payload[7] = sdm_update_latency_ticks(data->update_latency_s);

    return 0;
}

int sdm_encode_page2(uint8_t payload[SDM_PAYLOAD_SIZE], const sdm_data_t *data,
                     uint8_t status) {
    uint8_t speed_integer;
    uint8_t speed_fractional;
    uint8_t cadence_integer;
    uint8_t cadence_fractional;

    if ((payload == NULL) || (data == NULL)) {
        return -EINVAL;
    }

    sdm_encode_speed_bytes(&speed_integer, &speed_fractional, data->speed_mps);
    sdm_encode_cadence_bytes(&cadence_integer, &cadence_fractional, data->cadence_spm);

    payload[0] = SDM_PAGE_2;
    payload[1] = UINT8_MAX;
    payload[2] = UINT8_MAX;
    payload[3] = cadence_integer;
    payload[4] = (uint8_t)((cadence_fractional << 4) | speed_integer);
    payload[5] = speed_fractional;
    payload[6] = UINT8_MAX;
    payload[7] = status;

    return 0;
}

int sdm_encode_page3(uint8_t payload[SDM_PAYLOAD_SIZE], const sdm_data_t *data, uint8_t status,
                     uint8_t calories) {
    uint8_t speed_integer;
    uint8_t speed_fractional;
    uint8_t cadence_integer;
    uint8_t cadence_fractional;

    if ((payload == NULL) || (data == NULL)) {
        return -EINVAL;
    }

    sdm_encode_speed_bytes(&speed_integer, &speed_fractional, data->speed_mps);
    sdm_encode_cadence_bytes(&cadence_integer, &cadence_fractional, data->cadence_spm);

    payload[0] = SDM_PAGE_3;
    payload[1] = UINT8_MAX;
    payload[2] = UINT8_MAX;
    payload[3] = cadence_integer;
    payload[4] = (uint8_t)((cadence_fractional << 4) | speed_integer);
    payload[5] = speed_fractional;
    payload[6] = calories;
    payload[7] = status;

    return 0;
}
