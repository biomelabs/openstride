#ifndef SDM_ENCODE_H_
#define SDM_ENCODE_H_

#include <stdint.h>

#include "sdm_data.h"

#define SDM_DEVICE_TYPE 0x7Cu
#define SDM_CHANNEL_PERIOD_4HZ 8134u

#define SDM_PAYLOAD_SIZE 8u

#define SDM_PAGE_1 0x01u
#define SDM_PAGE_2 0x02u
#define SDM_PAGE_3 0x03u

#define SDM_TIME_UNIT_REVERSAL 200u
#define SDM_DISTANCE_UNIT_REVERSAL 16u
#define SDM_SPEED_UNIT_REVERSAL 256u
#define SDM_UPDATE_LATENCY_UNIT_REVERSAL 32u
#define SDM_CADENCE_UNIT_REVERSAL 16u

enum sdm_use_state {
    SDM_USE_STATE_INACTIVE = 0x00,
    SDM_USE_STATE_ACTIVE = 0x01,
};

enum sdm_health_state {
    SDM_HEALTH_OK = 0x00,
    SDM_HEALTH_ERROR = 0x01,
    SDM_HEALTH_WARNING = 0x02,
};

enum sdm_battery_state {
    SDM_BATTERY_NEW = 0x00,
    SDM_BATTERY_GOOD = 0x01,
    SDM_BATTERY_OK = 0x02,
    SDM_BATTERY_LOW = 0x03,
};

enum sdm_sensor_location {
    SDM_LOCATION_LACES = 0x00,
    SDM_LOCATION_MIDSOLE = 0x01,
    SDM_LOCATION_OTHER = 0x02,
    SDM_LOCATION_ANKLE = 0x03,
};

uint8_t sdm_status_pack(enum sdm_use_state use_state, enum sdm_health_state health,
                        enum sdm_battery_state battery, enum sdm_sensor_location location);

int sdm_encode_page1(uint8_t payload[SDM_PAYLOAD_SIZE], const sdm_data_t *data,
                     uint16_t time_1_200s);

int sdm_encode_page2(uint8_t payload[SDM_PAYLOAD_SIZE], const sdm_data_t *data,
                     uint8_t status);

int sdm_encode_page3(uint8_t payload[SDM_PAYLOAD_SIZE], const sdm_data_t *data, uint8_t status,
                     uint8_t calories);

#endif /* SDM_ENCODE_H_ */
