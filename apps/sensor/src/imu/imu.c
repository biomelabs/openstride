#include "imu.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

static const struct device *imu_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
static const struct sensor_trigger imu_trigger = {
    .type = SENSOR_TRIG_DATA_READY,
    .chan = SENSOR_CHAN_ACCEL_XYZ,
};
static imu_data_cb_t user_cb;
static void *user_ctx;
static uint32_t sample_count;
/* Use a higher ODR on nRF52840 Sense to improve ZUPT integration fidelity. */
#define IMU_TARGET_ODR_HZ 416

static void imu_trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    struct sensor_value ax, ay, az, gx, gy, gz;
    int err;

    ARG_UNUSED(trig);

    err = sensor_sample_fetch(dev);
    if (err < 0) {
        LOG_WRN("IMU sample fetch failed: %d", err);
        return;
    }

    err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &ax);
    err |= sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &ay);
    err |= sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &az);
    err |= sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gx);
    err |= sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gy);
    err |= sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gz);
    if (err < 0) {
        LOG_WRN("IMU channel read failed: %d", err);
        return;
    }

    sample_count++;
    if ((sample_count % 104U) == 1U) {
        int32_t ax_milli = (ax.val1 * 1000) + (ax.val2 / 1000);
        int32_t ay_milli = (ay.val1 * 1000) + (ay.val2 / 1000);
        int32_t az_milli = (az.val1 * 1000) + (az.val2 / 1000);

        LOG_INF("IMU samples=%u accel_milli_ms2=(%d,%d,%d)", sample_count,
                ax_milli, ay_milli, az_milli);
    }

    imu_sample_t s = {
        .accel_x = sensor_value_to_double(&ax),
        .accel_y = sensor_value_to_double(&ay),
        .accel_z = sensor_value_to_double(&az),
        .gyro_x = sensor_value_to_double(&gx),
        .gyro_y = sensor_value_to_double(&gy),
        .gyro_z = sensor_value_to_double(&gz),
        .timestamp_us = k_uptime_get() * 1000,
    };

    if (user_cb) {
        user_cb(&s, user_ctx);
    }
}

int imu_init(void) {
    int err;

    if (!device_is_ready(imu_dev)) {
        return -ENODEV;
    }

    struct sensor_value odr = {.val1 = IMU_TARGET_ODR_HZ};
    err = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (err < 0) {
        return err;
    }

    err = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (err < 0) {
        return err;
    }

    return 0;
}

int imu_start(imu_data_cb_t cb, void *user_data) {
    int err;

    user_cb = cb;
    user_ctx = user_data;
    sample_count = 0U;
    err = sensor_trigger_set(imu_dev, &imu_trigger, imu_trigger_handler);
    if (err < 0) {
        user_cb = NULL;
        user_ctx = NULL;
    }

    return err;
}

void imu_stop(void) {
    sensor_trigger_set(imu_dev, &imu_trigger, NULL);
    user_cb = NULL;
    user_ctx = NULL;
}
