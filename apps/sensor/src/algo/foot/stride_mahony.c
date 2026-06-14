#include "algo/foot/stride_mahony.h"

#include <math.h>
#include <stddef.h>

#define STRIDE_GRAVITY_MS2 9.80665f

#define STRIDE_MAX_SAMPLE_INTERVAL_US 1000000LL

/* covers range from slow walk to Usain Bolt, with margin */
#define STRIDE_MIN_INTERVAL_US 400000LL
#define STRIDE_MAX_INTERVAL_US 2000000LL

#define STRIDE_MIN_STANCE_S 0.035f
#define STRIDE_MIN_SWING_S 0.120f

#define STRIDE_STANCE_GYRO_ENTER_RAD_S 1.4f
#define STRIDE_STANCE_GYRO_EXIT_RAD_S 2.1f

#define STRIDE_OUTPUT_ALPHA 0.4f

#define STRIDE_ATTITUDE_CORRECTION_GAIN 1.8f
#define STRIDE_ATTITUDE_CORRECTION_INTEGRAL_GAIN 0.008f
#define STRIDE_ATTITUDE_CORRECTION_ACCEL_BAND_MS2 0.8f
#define STRIDE_ATTITUDE_CORRECTION_GYRO_RAD_S 1.0f
#define STRIDE_GYRO_BIAS_LIMIT_RAD_S 0.25f

/* IIR to track sensor gravity magnitude (accounts for ADXL375 ±14% scale tolerance).
 * Only updates when quasi-static (gyro < 0.5 rad/s) AND the sample is close to the
 * current estimate (outlier gate rejects rotation-reversal moments where gyro briefly
 * dips below threshold while linear accel is still high).
 * α = max(ALPHA_MIN, 1/(n+1)): snaps to the first accepted quiet sample, then
 * asymptotes to ALPHA_MIN for long-term tracking across sessions and temperature. */
#define STRIDE_GRAVITY_NORM_IIR_ALPHA_MIN  0.005f
#define STRIDE_GRAVITY_NORM_IIR_GYRO_MAX   0.5f
/* ±14% ADXL375 spec → valid gravity range [8.4, 11.2] m/s²; ±5 from nominal 9.81
 * covers the full spec range while rejecting dynamic spikes (typically >14 m/s²). */
#define STRIDE_GRAVITY_NORM_IIR_OUTLIER_MS2 5.0f

#define STRIDE_MIN_LENGTH_M 0.20f
#define STRIDE_MAX_LENGTH_M 3.20f

#define STRIDE_DECAY_START_FALLBACK_US 450000LL
#define STRIDE_STALE_DATA_FALLBACK_INTERVAL_US 900000LL
#define STRIDE_STALE_INTERVAL_SCALE 1.5f

static float stride_absf(float value) {
    return (value < 0.0f) ? -value : value;
}

static float stride_clampf(float value, float min, float max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static float stride_dot3(const float a[3], const float b[3]) {
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

static float stride_norm3(const float v[3]) {
    float dot = stride_dot3(v, v);

    if (dot <= 0.0f) {
        return 0.0f;
    }
    return sqrtf(dot);
}

static bool stride_normalize3(float v[3]) {
    float n = stride_norm3(v);

    if (n <= 1.0e-6f) {
        return false;
    }

    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
    return true;
}

static void stride_cross3(const float a[3], const float b[3], float out[3]) {
    out[0] = (a[1] * b[2]) - (a[2] * b[1]);
    out[1] = (a[2] * b[0]) - (a[0] * b[2]);
    out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

static void stride_quat_normalize(float q[4]) {
    float n = sqrtf((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));

    if (n <= 1.0e-9f) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
}

static void stride_quat_rotate_body_to_world(const float q[4], const float body[3],
                                             float world[3]) {
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    world[0] = (1.0f - 2.0f * (qy * qy + qz * qz)) * body[0] +
               (2.0f * (qx * qy - qw * qz)) * body[1] + (2.0f * (qx * qz + qw * qy)) * body[2];
    world[1] = (2.0f * (qx * qy + qw * qz)) * body[0] +
               (1.0f - 2.0f * (qx * qx + qz * qz)) * body[1] +
               (2.0f * (qy * qz - qw * qx)) * body[2];
    world[2] = (2.0f * (qx * qz - qw * qy)) * body[0] + (2.0f * (qy * qz + qw * qx)) * body[1] +
               (1.0f - 2.0f * (qx * qx + qy * qy)) * body[2];
}

static void stride_quat_rotate_world_to_body(const float q[4], const float world[3],
                                             float body[3]) {
    float qc[4] = {q[0], -q[1], -q[2], -q[3]};

    stride_quat_rotate_body_to_world(qc, world, body);
}

static void stride_quat_from_two_vectors(const float from[3], const float to[3], float q[4]) {
    float axis[3];
    float dot = stride_dot3(from, to);

    if (dot < -0.999f) {
        float ortho[3] = {1.0f, 0.0f, 0.0f};

        if (stride_absf(from[0]) > 0.8f) {
            ortho[0] = 0.0f;
            ortho[1] = 1.0f;
            ortho[2] = 0.0f;
        }

        stride_cross3(from, ortho, axis);
        (void)stride_normalize3(axis);
        q[0] = 0.0f;
        q[1] = axis[0];
        q[2] = axis[1];
        q[3] = axis[2];
        return;
    }

    stride_cross3(from, to, axis);
    q[0] = 1.0f + dot;
    q[1] = axis[0];
    q[2] = axis[1];
    q[3] = axis[2];
    stride_quat_normalize(q);
}

static void stride_orientation_init_from_accel(stride_detector_t *detector,
                                               const float accel_mps2[3]) {
    float up_body[3] = {accel_mps2[0], accel_mps2[1], accel_mps2[2]};
    const float up_world[3] = {0.0f, 0.0f, 1.0f};

    if (!stride_normalize3(up_body)) {
        detector->orientation_q[0] = 1.0f;
        detector->orientation_q[1] = 0.0f;
        detector->orientation_q[2] = 0.0f;
        detector->orientation_q[3] = 0.0f;
    } else {
        stride_quat_from_two_vectors(up_body, up_world, detector->orientation_q);
    }
}

static void stride_orientation_update(stride_detector_t *detector, const float accel_mps2[3],
                                      const float gyro_rads[3], float dt_s) {
    float gravity_world[3] = {0.0f, 0.0f, 1.0f};
    float gravity_body[3];
    float accel_unit[3] = {accel_mps2[0], accel_mps2[1], accel_mps2[2]};
    float correction[3] = {0.0f, 0.0f, 0.0f};
    float gyro_corr[3];
    float accel_norm;
    float gyro_norm;
    float *q = detector->orientation_q;
    float qw;
    float qx;
    float qy;
    float qz;
    float qdot[4];
    int axis;

    stride_quat_rotate_world_to_body(q, gravity_world, gravity_body);
    accel_norm = stride_norm3(accel_mps2);
    gyro_norm = stride_norm3(gyro_rads);

    if ((gyro_norm < STRIDE_GRAVITY_NORM_IIR_GYRO_MAX) &&
        (stride_absf(accel_norm - detector->gravity_norm_est) <
         STRIDE_GRAVITY_NORM_IIR_OUTLIER_MS2)) {
        float alpha = 1.0f / (float)(detector->gravity_norm_samples + 1U);

        if (alpha < STRIDE_GRAVITY_NORM_IIR_ALPHA_MIN) {
            alpha = STRIDE_GRAVITY_NORM_IIR_ALPHA_MIN;
        }
        detector->gravity_norm_est = ((1.0f - alpha) * detector->gravity_norm_est) +
                                     (alpha * accel_norm);
        if (detector->gravity_norm_samples < 65535U) {
            detector->gravity_norm_samples++;
        }
    }

    if ((stride_absf(accel_norm - detector->gravity_norm_est) <=
         STRIDE_ATTITUDE_CORRECTION_ACCEL_BAND_MS2) &&
        (gyro_norm <= STRIDE_ATTITUDE_CORRECTION_GYRO_RAD_S) && stride_normalize3(accel_unit)) {
        stride_cross3(gravity_body, accel_unit, correction);
        for (axis = 0; axis < 3; axis++) {
            detector->gyro_bias[axis] = stride_clampf(
                detector->gyro_bias[axis] +
                    (correction[axis] * STRIDE_ATTITUDE_CORRECTION_INTEGRAL_GAIN * dt_s),
                -STRIDE_GYRO_BIAS_LIMIT_RAD_S, STRIDE_GYRO_BIAS_LIMIT_RAD_S);
        }
    }

    for (axis = 0; axis < 3; axis++) {
        gyro_corr[axis] = gyro_rads[axis] - detector->gyro_bias[axis] -
                          (STRIDE_ATTITUDE_CORRECTION_GAIN * correction[axis]);
    }

    qw = q[0];
    qx = q[1];
    qy = q[2];
    qz = q[3];

    qdot[0] = -0.5f * ((qx * gyro_corr[0]) + (qy * gyro_corr[1]) + (qz * gyro_corr[2]));
    qdot[1] = 0.5f * ((qw * gyro_corr[0]) + (qy * gyro_corr[2]) - (qz * gyro_corr[1]));
    qdot[2] = 0.5f * ((qw * gyro_corr[1]) + (qz * gyro_corr[0]) - (qx * gyro_corr[2]));
    qdot[3] = 0.5f * ((qw * gyro_corr[2]) + (qx * gyro_corr[1]) - (qy * gyro_corr[0]));

    q[0] += qdot[0] * dt_s;
    q[1] += qdot[1] * dt_s;
    q[2] += qdot[2] * dt_s;
    q[3] += qdot[3] * dt_s;
    stride_quat_normalize(q);
}

static bool stride_is_stance_sample(const stride_detector_t *detector, const float accel_mps2[3],
                                    const float gyro_rads[3]) {
    float gyro_norm = stride_norm3(gyro_rads);
    float gyro_band =
        detector->in_stance ? STRIDE_STANCE_GYRO_EXIT_RAD_S : STRIDE_STANCE_GYRO_ENTER_RAD_S;

    (void)accel_mps2;
    /* Running stance loading pushes accel well outside a 1g band; gyro alone is
     * the reliable indicator that the foot is flat and briefly stationary. */
    return gyro_norm <= gyro_band;
}

static void stride_reset_swing_integrator(stride_detector_t *detector) {
    detector->swing_elapsed_s = 0.0f;
    detector->swing_velocity_world[0] = 0.0f;
    detector->swing_velocity_world[1] = 0.0f;
    detector->swing_velocity_world[2] = 0.0f;
    detector->swing_displacement_world[0] = 0.0f;
    detector->swing_displacement_world[1] = 0.0f;
    detector->swing_displacement_world[2] = 0.0f;
}

static void stride_integrate_swing(stride_detector_t *detector, const float accel_mps2[3],
                                   float dt_s) {
    float accel_world[3];
    float linear_world[3];
    int axis;

    stride_quat_rotate_body_to_world(detector->orientation_q, accel_mps2, accel_world);

    linear_world[0] = accel_world[0];
    linear_world[1] = accel_world[1];
    linear_world[2] = accel_world[2] - detector->gravity_norm_est;

    for (axis = 0; axis < 3; axis++) {
        detector->swing_velocity_world[axis] += linear_world[axis] * dt_s;
        detector->swing_displacement_world[axis] += detector->swing_velocity_world[axis] * dt_s;
    }

    detector->swing_elapsed_s += dt_s;
}

static bool stride_finalize_zupt(stride_detector_t *detector, int64_t timestamp_us) {
    float interval_s;
    float corrected_displacement[3];
    float stride_length_m;
    int64_t interval_us = timestamp_us - detector->last_zupt_timestamp_us;
    int axis;

    if (detector->last_zupt_timestamp_us < 0) {
        detector->last_zupt_timestamp_us = timestamp_us;
        stride_reset_swing_integrator(detector);
        return false;
    }

    if ((interval_us < STRIDE_MIN_INTERVAL_US) || (interval_us > STRIDE_MAX_INTERVAL_US) ||
        (detector->swing_elapsed_s < STRIDE_MIN_SWING_S)) {
        detector->last_zupt_timestamp_us = timestamp_us;
        stride_reset_swing_integrator(detector);
        return false;
    }

    interval_s = (float)interval_us * 1.0e-6f;
    for (axis = 0; axis < 3; axis++) {
        corrected_displacement[axis] =
            detector->swing_displacement_world[axis] -
            (0.5f * detector->swing_elapsed_s * detector->swing_velocity_world[axis]);
    }

    stride_length_m = sqrtf((corrected_displacement[0] * corrected_displacement[0]) +
                            (corrected_displacement[1] * corrected_displacement[1]));
    if (stride_length_m < STRIDE_MIN_LENGTH_M) {
        detector->last_zupt_timestamp_us = timestamp_us;
        stride_reset_swing_integrator(detector);
        return false;
    }
    if (stride_length_m > STRIDE_MAX_LENGTH_M) {
        stride_length_m = STRIDE_MAX_LENGTH_M;
    }

    float raw_cadence = 60.0f / interval_s;
    float raw_speed = stride_length_m / interval_s;

    if (detector->last_stride_cadence_spm > 0.0f) {
        detector->last_stride_cadence_spm =
            (STRIDE_OUTPUT_ALPHA * raw_cadence) +
            ((1.0f - STRIDE_OUTPUT_ALPHA) * detector->last_stride_cadence_spm);
        detector->last_stride_speed_mps =
            (STRIDE_OUTPUT_ALPHA * raw_speed) +
            ((1.0f - STRIDE_OUTPUT_ALPHA) * detector->last_stride_speed_mps);
    } else {
        detector->last_stride_cadence_spm = raw_cadence;
        detector->last_stride_speed_mps = raw_speed;
    }

    detector->data.stride_count++;
    detector->data.distance_m += stride_length_m;
    detector->data.stride_length_m = stride_length_m;
    detector->data.cadence_spm = detector->last_stride_cadence_spm;
    detector->data.speed_mps = detector->last_stride_speed_mps;
    detector->data.update_latency_s = 0.0f;

    detector->last_stride_timestamp_us = timestamp_us;
    detector->last_zupt_timestamp_us = timestamp_us;

    stride_reset_swing_integrator(detector);
    return true;
}

static void stride_decay_window_us(const stride_detector_t *detector, int64_t *decay_start_us,
                                   int64_t *stale_interval_us) {
    int64_t expected_interval_us = 0;
    int64_t dynamic_stale_interval_us;

    *decay_start_us = STRIDE_DECAY_START_FALLBACK_US;
    *stale_interval_us = STRIDE_STALE_DATA_FALLBACK_INTERVAL_US;

    if (detector->last_stride_cadence_spm > 0.0f) {
        expected_interval_us =
            (int64_t)((60.0f * 1000000.0f / detector->last_stride_cadence_spm) + 0.5f);
        if (expected_interval_us > *decay_start_us) {
            *decay_start_us = expected_interval_us;
        }
        dynamic_stale_interval_us =
            (int64_t)((float)expected_interval_us * STRIDE_STALE_INTERVAL_SCALE);
        if (dynamic_stale_interval_us > *stale_interval_us) {
            *stale_interval_us = dynamic_stale_interval_us;
        }
    }

    if (*stale_interval_us <= *decay_start_us) {
        *stale_interval_us = *decay_start_us + 1;
    }
}

void stride_detector_init(stride_detector_t *detector) {
    if (detector == NULL) {
        return;
    }

    *detector = (stride_detector_t){
        .data = sdm_data_zero(),
        .orientation_q = {1.0f, 0.0f, 0.0f, 0.0f},
        .gyro_bias = {0.0f, 0.0f, 0.0f},
        .gravity_norm_est = STRIDE_GRAVITY_MS2,
        .gravity_norm_samples = 0U,
        .stance_candidate_s = 0.0f,
        .in_stance = false,
        .swing_elapsed_s = 0.0f,
        .swing_velocity_world = {0.0f, 0.0f, 0.0f},
        .swing_displacement_world = {0.0f, 0.0f, 0.0f},
        .last_stride_speed_mps = 0.0f,
        .last_stride_cadence_spm = 0.0f,
        .last_sample_timestamp_us = 0,
        .last_zupt_timestamp_us = -1,
        .last_stride_timestamp_us = -1,
        .initialized = false,
    };
}

bool stride_detector_update(stride_detector_t *detector, const float accel_mps2[3],
                            const float gyro_rads[3], int64_t timestamp_us, sdm_data_t *out_data) {
    int64_t dt_us;
    float dt_s;
    bool stance_sample;
    bool in_stance_now;
    bool stride_detected = false;

    if ((detector == NULL) || (accel_mps2 == NULL) || (gyro_rads == NULL)) {
        return false;
    }

    if (!detector->initialized) {
        stride_orientation_init_from_accel(detector, accel_mps2);
        detector->last_sample_timestamp_us = timestamp_us;
        detector->initialized = true;
        detector->in_stance = stride_is_stance_sample(detector, accel_mps2, gyro_rads);
        detector->stance_candidate_s = detector->in_stance ? STRIDE_MIN_STANCE_S : 0.0f;
        if (detector->in_stance) {
            detector->last_zupt_timestamp_us = timestamp_us;
        }
        if (out_data != NULL) {
            *out_data = detector->data;
        }
        return false;
    }

    dt_us = timestamp_us - detector->last_sample_timestamp_us;
    if (dt_us <= 0) {
        if (out_data != NULL) {
            *out_data = detector->data;
        }
        return false;
    }

    if (dt_us > STRIDE_MAX_SAMPLE_INTERVAL_US) {
        dt_us = STRIDE_MAX_SAMPLE_INTERVAL_US;
    }
    dt_s = (float)dt_us * 1.0e-6f;

    stride_orientation_update(detector, accel_mps2, gyro_rads, dt_s);

    stance_sample = stride_is_stance_sample(detector, accel_mps2, gyro_rads);
    if (stance_sample) {
        detector->stance_candidate_s += dt_s;
    } else {
        detector->stance_candidate_s = 0.0f;
    }

    in_stance_now =
        detector->in_stance ? stance_sample : (detector->stance_candidate_s >= STRIDE_MIN_STANCE_S);

    if (detector->in_stance && !in_stance_now) {
        stride_reset_swing_integrator(detector);
    }

    if (!detector->in_stance && in_stance_now) {
        stride_detected = stride_finalize_zupt(detector, timestamp_us);
    } else if (!in_stance_now) {
        stride_integrate_swing(detector, accel_mps2, dt_s);
    } else {
        detector->swing_velocity_world[0] = 0.0f;
        detector->swing_velocity_world[1] = 0.0f;
        detector->swing_velocity_world[2] = 0.0f;
    }

    if (!stride_detected && (detector->last_stride_timestamp_us >= 0)) {
        int64_t latency_us = timestamp_us - detector->last_stride_timestamp_us;
        int64_t decay_start_us;
        int64_t stale_interval_us;

        if (latency_us < 0) {
            latency_us = 0;
        }

        stride_decay_window_us(detector, &decay_start_us, &stale_interval_us);
        detector->data.update_latency_s = (float)latency_us * 1.0e-6f;
        if (latency_us >= stale_interval_us) {
            detector->data.cadence_spm = 0.0f;
            detector->data.speed_mps = 0.0f;
        } else if (latency_us >= decay_start_us) {
            float fade =
                (float)(latency_us - decay_start_us) / (float)(stale_interval_us - decay_start_us);

            fade = stride_clampf(fade, 0.0f, 1.0f);
            detector->data.cadence_spm = detector->last_stride_cadence_spm * (1.0f - fade);
            detector->data.speed_mps = detector->last_stride_speed_mps * (1.0f - fade);
        }
    }

    detector->in_stance = in_stance_now;
    detector->last_sample_timestamp_us = timestamp_us;

    if (out_data != NULL) {
        *out_data = detector->data;
    }

    return stride_detected;
}
