#ifndef STRIDE_ESKF_H_
#define STRIDE_ESKF_H_

#include <stdbool.h>
#include <stdint.h>

#include "sdm/sdm_data.h"

/*
 * Error State Kalman Filter foot stride detector.
 *
 * 15-DOF error state: δθ(3), δv(3), δp(3), δb_g(3), δb_a(3).
 * Nominal state: quaternion orientation, velocity, position, gyro and accel
 * biases — all propagated continuously.  Zero-velocity updates (ZUPT) are
 * applied on every stance sample to bound drift.  Stride length is derived
 * from the horizontal position difference between consecutive stance onsets
 * rather than from accumulated velocity integrals.
 */

#define STRIDE_ESKF_N 15 /* error-state dimension */

typedef struct stride_eskf_detector {
    sdm_data_t data;

    /* Nominal navigation state */
    float q[4];   /* body-to-world quaternion [w, x, y, z] */
    float v[3];   /* velocity in world frame, m/s */
    float p[3];   /* position in world frame, m */
    float b_g[3]; /* gyro bias, rad/s */
    float b_a[3]; /* accel bias, m/s² */

    /* Error-state covariance (15×15, row-major) */
    float P[STRIDE_ESKF_N * STRIDE_ESKF_N];

    /* Stance / stride tracking */
    float stance_candidate_s;
    bool in_stance;
    float swing_elapsed_s;
    float p_swing_start[3]; /* position at last stance→swing transition */
    bool p_swing_start_valid;

    float last_stride_speed_mps;
    float last_stride_cadence_spm;
    int64_t last_sample_timestamp_us;
    int64_t last_zupt_timestamp_us;
    int64_t last_stride_timestamp_us;

    bool initialized;
} stride_eskf_detector_t;

void stride_eskf_detector_init(stride_eskf_detector_t *detector);

bool stride_eskf_detector_update(stride_eskf_detector_t *detector, const float accel_mps2[3],
                                 const float gyro_rads[3], int64_t timestamp_us,
                                 sdm_data_t *out_data);

#endif /* STRIDE_ESKF_H_ */
