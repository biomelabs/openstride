#include <errno.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include "algo/foot/stride_mahony.h"
#include "sdm/sdm_encode.h"

static void feed_sample(struct stride_detector *detector, int64_t timestamp_us, float ax, float ay,
                        float az, float gx, float gy, float gz, struct sdm_data *out) {
    float accel[3] = {ax, ay, az};
    float gyro[3] = {gx, gy, gz};

    (void)stride_detector_update(detector, accel, gyro, timestamp_us, out);
}

static int64_t feed_stance(struct stride_detector *detector, int64_t start_us, int32_t duration_ms,
                           struct sdm_data *out) {
    int32_t t_ms;

    for (t_ms = 0; t_ms <= duration_ms; t_ms += 10) {
        feed_sample(detector, start_us + ((int64_t)t_ms * 1000), 0.0f, 0.0f, 9.80665f, 0.0f, 0.0f,
                    0.0f, out);
    }

    return start_us + ((int64_t)duration_ms * 1000);
}

static int64_t feed_swing(struct stride_detector *detector, int64_t start_us, int32_t duration_ms,
                          float forward_peak_accel, struct sdm_data *out) {
    int32_t t_ms;

    for (t_ms = 10; t_ms <= duration_ms; t_ms += 10) {
        float phase = (float)t_ms / (float)duration_ms;
        float ax = (phase < 0.5f) ? forward_peak_accel : -forward_peak_accel;

        feed_sample(detector, start_us + ((int64_t)t_ms * 1000), ax, 0.0f, 9.80665f, 0.0f, 0.0f,
                    0.0f, out);
    }

    return start_us + ((int64_t)duration_ms * 1000);
}

ZTEST(sdm_unit, test_sdm_data_zero_defaults) {
    struct sdm_data data = sdm_data_zero();

    zassert_equal(data.speed_mps, 0.0f, "speed should start at zero");
    zassert_equal(data.distance_m, 0.0f, "distance should start at zero");
    zassert_equal(data.stride_count, 0U, "stride count should start at zero");
    zassert_equal(data.cadence_spm, 0.0f, "cadence should start at zero");
    zassert_equal(data.update_latency_s, 0.0f, "latency should start at zero");
}

ZTEST(sdm_unit, test_status_pack_layout) {
    uint8_t status = sdm_status_pack(SDM_USE_STATE_ACTIVE, SDM_HEALTH_WARNING, SDM_BATTERY_LOW,
                                     SDM_LOCATION_ANKLE);

    zassert_equal(status, 0xF9, "unexpected status packing");
}

ZTEST(sdm_unit, test_page1_encode_known_vector) {
    struct sdm_data data = {
        .speed_mps = 3.25f,
        .distance_m = 12.5f,
        .stride_count = 0x1234u,
        .cadence_spm = 0.0f,
        .update_latency_s = 0.3125f,
    };
    uint8_t payload[SDM_PAYLOAD_SIZE] = {0};
    int err = sdm_encode_page1(payload, &data, 1234u);
    const uint8_t expected[SDM_PAYLOAD_SIZE] = {0x01u, 0x22u, 0x06u, 0x0Cu,
                                                0x83u, 0x40u, 0x34u, 0x0Au};

    zassert_equal(err, 0, "page 1 encoder failed");
    zassert_mem_equal(payload, expected, sizeof(expected), "page 1 payload mismatch");
}

ZTEST(sdm_unit, test_page2_encode_known_vector) {
    struct sdm_data data = {
        .speed_mps = 4.75f,
        .distance_m = 0.0f,
        .stride_count = 0u,
        .cadence_spm = 176.25f,
        .update_latency_s = 0.0f,
    };
    uint8_t payload[SDM_PAYLOAD_SIZE] = {0};
    uint8_t status = sdm_status_pack(SDM_USE_STATE_ACTIVE, SDM_HEALTH_WARNING, SDM_BATTERY_LOW,
                                     SDM_LOCATION_ANKLE);
    int err = sdm_encode_page2(payload, &data, status);
    const uint8_t expected[SDM_PAYLOAD_SIZE] = {0x02u, 0xFFu, 0xFFu, 0xB0u,
                                                0x44u, 0xC0u, 0xFFu, 0xF9u};

    zassert_equal(err, 0, "page 2 encoder failed");
    zassert_mem_equal(payload, expected, sizeof(expected), "page 2 payload mismatch");
}

ZTEST(sdm_unit, test_page3_encode_known_vector) {
    struct sdm_data data = {
        .speed_mps = 2.0f,
        .distance_m = 0.0f,
        .stride_count = 0u,
        .cadence_spm = 80.0f,
        .update_latency_s = 0.0f,
    };
    uint8_t payload[SDM_PAYLOAD_SIZE] = {0};
    uint8_t status =
        sdm_status_pack(SDM_USE_STATE_ACTIVE, SDM_HEALTH_OK, SDM_BATTERY_GOOD, SDM_LOCATION_LACES);
    int err = sdm_encode_page3(payload, &data, status, 77u);
    const uint8_t expected[SDM_PAYLOAD_SIZE] = {0x03u, 0xFFu, 0xFFu, 0x50u,
                                                0x02u, 0x00u, 0x4Du, 0x11u};

    zassert_equal(err, 0, "page 3 encoder failed");
    zassert_mem_equal(payload, expected, sizeof(expected), "page 3 payload mismatch");
}

ZTEST(sdm_unit, test_encoder_null_guards) {
    struct sdm_data data = sdm_data_zero();
    uint8_t payload[SDM_PAYLOAD_SIZE] = {0};

    zassert_equal(sdm_encode_page1(NULL, &data, 0u), -EINVAL, "page1 NULL payload guard failed");
    zassert_equal(sdm_encode_page1(payload, NULL, 0u), -EINVAL, "page1 NULL data guard failed");
    zassert_equal(sdm_encode_page2(NULL, &data, 0u), -EINVAL, "page2 NULL payload guard failed");
    zassert_equal(sdm_encode_page2(payload, NULL, 0u), -EINVAL, "page2 NULL data guard failed");
    zassert_equal(sdm_encode_page3(NULL, &data, 0u, 0u), -EINVAL,
                  "page3 NULL payload guard failed");
    zassert_equal(sdm_encode_page3(payload, NULL, 0u, 0u), -EINVAL, "page3 NULL data guard failed");
}

ZTEST(sdm_unit, test_stride_detector_estimates_stride_with_zupt) {
    struct stride_detector detector;
    struct sdm_data out = sdm_data_zero();
    int64_t t_us = 0;

    stride_detector_init(&detector);

    t_us = feed_stance(&detector, t_us, 100, &out);
    t_us = feed_swing(&detector, t_us, 500, 12.0f, &out);
    t_us = feed_stance(&detector, t_us, 100, &out);

    zassert_equal(out.stride_count, 1u, "expected one completed stride");
    zassert_true(out.stride_length_m > 0.30f, "stride length should be motion-derived");
    zassert_true(out.stride_length_m < 2.00f, "stride length upper bound sanity");
    zassert_true(out.distance_m >= out.stride_length_m, "distance should accumulate stride");
    zassert_true(out.speed_mps > 0.0f, "speed should be positive after detected stride");
}

ZTEST(sdm_unit, test_stride_detector_varies_stride_length_by_motion_profile) {
    struct stride_detector detector;
    struct sdm_data out = sdm_data_zero();
    int64_t t_us = 0;
    float first_stride_m;

    stride_detector_init(&detector);

    t_us = feed_stance(&detector, t_us, 80, &out);
    t_us = feed_swing(&detector, t_us, 480, 8.0f, &out);
    t_us = feed_stance(&detector, t_us, 80, &out);
    first_stride_m = out.stride_length_m;

    t_us = feed_swing(&detector, t_us, 480, 14.0f, &out);
    t_us = feed_stance(&detector, t_us, 80, &out);

    zassert_equal(out.stride_count, 2u, "expected two completed strides");
    zassert_true(out.stride_length_m > first_stride_m + 0.05f,
                 "higher swing acceleration should produce longer stride");
    zassert_true(first_stride_m > 0.15f, "first stride should be non-trivial");
}

ZTEST(sdm_unit, test_stride_detector_static_tilt_correction_converges) {
    struct stride_detector detector;
    struct sdm_data out = sdm_data_zero();
    const float initial_qx = 0.04997917f;
    int i;

    stride_detector_init(&detector);
    feed_sample(&detector, 0, 0.0f, 0.0f, 9.80665f, 0.0f, 0.0f, 0.0f, &out);

    detector.orientation_q[0] = 0.99875026f;
    detector.orientation_q[1] = initial_qx;
    detector.orientation_q[2] = 0.0f;
    detector.orientation_q[3] = 0.0f;

    for (i = 1; i <= 100; i++) {
        feed_sample(&detector, (int64_t)i * 10000, 0.0f, 0.0f, 9.80665f, 0.0f, 0.0f, 0.0f, &out);
    }

    zassert_true(detector.orientation_q[1] < (initial_qx * 0.5f),
                 "static accel correction should reduce roll tilt");
    zassert_true(detector.gyro_bias[0] > 0.0f, "integral term should track correction bias");
}

ZTEST(sdm_unit, test_stride_detector_decay_uses_last_stride_baseline) {
    struct stride_detector detector;
    struct sdm_data out = sdm_data_zero();
    int64_t t_us = 0;
    int64_t expected_interval_us;
    int64_t stale_interval_us;
    float baseline_cadence_spm;
    float baseline_speed_mps;

    stride_detector_init(&detector);

    t_us = feed_stance(&detector, t_us, 80, &out);
    t_us = feed_swing(&detector, t_us, 900, 12.0f, &out);
    t_us = feed_stance(&detector, t_us, 80, &out);

    zassert_true(out.cadence_spm > 50.0f, "baseline cadence should come from stride interval");
    zassert_true(out.speed_mps > 0.20f, "baseline speed should come from stride interval");

    baseline_cadence_spm = out.cadence_spm;
    baseline_speed_mps = out.speed_mps;
    expected_interval_us =
        (int64_t)((60.0f * 1000000.0f / detector.last_stride_cadence_spm) + 0.5f);
    stale_interval_us = (int64_t)((float)expected_interval_us * 1.5f);

    feed_sample(&detector, detector.last_stride_timestamp_us + expected_interval_us, 0.0f, 0.0f,
                9.80665f, 0.0f, 0.0f, 0.0f, &out);
    zassert_true(out.cadence_spm > (baseline_cadence_spm * 0.95f),
                 "cadence should not fade before the next expected stride");
    zassert_true(out.speed_mps > (baseline_speed_mps * 0.95f),
                 "speed should not fade before the next expected stride");

    feed_sample(&detector,
                detector.last_stride_timestamp_us + expected_interval_us +
                    ((stale_interval_us - expected_interval_us) / 2),
                0.0f, 0.0f, 9.80665f, 0.0f, 0.0f, 0.0f, &out);
    zassert_true(out.cadence_spm > 0.0f, "cadence should decay gradually before stale timeout");
    zassert_true(out.speed_mps > 0.0f, "speed should decay gradually before stale timeout");
    zassert_true(out.cadence_spm < baseline_cadence_spm, "cadence should be fading");
    zassert_true(out.speed_mps < baseline_speed_mps, "speed should be fading");

    feed_sample(&detector, detector.last_stride_timestamp_us + stale_interval_us + 10000, 0.0f,
                0.0f, 9.80665f, 0.0f, 0.0f, 0.0f, &out);
    zassert_equal(out.cadence_spm, 0.0f, "cadence should be zero once stale");
    zassert_equal(out.speed_mps, 0.0f, "speed should be zero once stale");
}

ZTEST(sdm_unit, test_stride_detector_ignores_non_monotonic_timestamps) {
    struct stride_detector detector;
    struct sdm_data out = sdm_data_zero();
    uint16_t before_strides;

    stride_detector_init(&detector);

    feed_sample(&detector, 1000000, 0.0f, 0.0f, 9.80665f, 0.0f, 0.0f, 0.0f, &out);
    feed_sample(&detector, 1300000, 10.0f, 0.0f, 9.80665f, 0.0f, 6.0f, 0.0f, &out);
    before_strides = out.stride_count;
    feed_sample(&detector, 1200000, 10.0f, 0.0f, 9.80665f, 0.0f, 6.0f, 0.0f, &out);

    zassert_equal(out.stride_count, before_strides, "out-of-order sample should be ignored");
}

ZTEST_SUITE(sdm_unit, NULL, NULL, NULL, NULL, NULL);
