#ifndef SDM_DATA_H_
#define SDM_DATA_H_

#include <stdint.h>

typedef struct sdm_data {
    float speed_mps;
    float distance_m;
    uint16_t stride_count;
    float cadence_spm;
    float stride_length_m;
    float update_latency_s;
} sdm_data_t;

static inline sdm_data_t sdm_data_zero(void)
{
    return (sdm_data_t){
        .speed_mps = 0.0f,
        .distance_m = 0.0f,
        .stride_count = 0U,
        .cadence_spm = 0.0f,
        .stride_length_m = 0.0f,
        .update_latency_s = 0.0f,
    };
}

#endif /* SDM_DATA_H_ */
