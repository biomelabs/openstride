#ifndef STRIDE_MAHONY_H_
#define STRIDE_MAHONY_H_

#include <stdbool.h>
#include <stdint.h>

#include "sdm/sdm_data.h"

typedef struct stride_detector {
    sdm_data_t data;

    float orientation_q[4]; /* Body-to-world quaternion [w, x, y, z]. */
    float gyro_bias[3];     /* Estimated gyro bias in body axes, rad/s. */
    float    gravity_norm_est;     /* IIR-filtered accel magnitude during quasi-static periods, m/s².
                                    * Self-calibrates to this unit's actual scale factor. */
    uint16_t gravity_norm_samples; /* quasi-static samples seen; drives convergent-α IIR. */

    float stance_candidate_s;
    bool in_stance;

    float swing_elapsed_s;
    float swing_velocity_world[3];
    float swing_displacement_world[3];

    float last_stride_speed_mps;
    float last_stride_cadence_spm;
    int64_t last_sample_timestamp_us;
    int64_t last_zupt_timestamp_us;
    int64_t last_stride_timestamp_us;

    bool initialized;
} stride_detector_t;

void stride_detector_init(stride_detector_t *detector);

bool stride_detector_update(stride_detector_t *detector, const float accel_mps2[3],
                            const float gyro_rads[3], int64_t timestamp_us, sdm_data_t *out_data);

#endif /* STRIDE_MAHONY_H_ */
