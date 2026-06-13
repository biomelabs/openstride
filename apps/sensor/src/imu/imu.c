#include "imu.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

static const struct device *imu_dev = DEVICE_DT_GET(DT_ALIAS(imu0));

#if DT_NODE_EXISTS(DT_ALIAS(accel0))
static const struct device *ext_accel_dev = DEVICE_DT_GET(DT_ALIAS(accel0));
static bool ext_accel_ready;
static bool ext_accel_fetch_ok;

/*
 * The ADXL345 driver hardcodes SENSOR_G/32 = 31.25 mg/LSB as its scale.
 * The ADXL375 has a fixed 49 mg/LSB sensitivity (ignores the range register),
 * so the driver under-reports by 31.25/49. Multiply by 49/31.25 to correct.
 */
#define ADXL375_SCALE (49.0 / 31.25)
#endif

static imu_data_cb_t user_cb;
static void *user_ctx;
static uint32_t sample_count;

#define IMU_TARGET_ODR_HZ 416

#define IMU_POLL_STACK_SIZE 4096
#define IMU_POLL_PRIORITY   7   /* preemptive, below BLE cooperative threads */
#define IMU_POLL_PERIOD_MS  (1000 / IMU_TARGET_ODR_HZ)

K_THREAD_STACK_DEFINE(imu_poll_stack, IMU_POLL_STACK_SIZE);
static struct k_thread imu_poll_thread;
static atomic_t imu_poll_run;

static void imu_poll_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Poll thread started at %u Hz", IMU_TARGET_ODR_HZ);

    while (atomic_get(&imu_poll_run)) {
        struct sensor_value ax, ay, az, gx, gy, gz;
        int err;
        bool got_ext_accel = false;

        err = sensor_sample_fetch(imu_dev);
        if (err < 0) {
            LOG_WRN("IMU fetch failed: %d", err);
            k_sleep(K_MSEC(IMU_POLL_PERIOD_MS));
            continue;
        }

#if DT_NODE_EXISTS(DT_ALIAS(accel0))
        if (ext_accel_ready) {
            int ext_err = sensor_sample_fetch(ext_accel_dev);

            if (ext_err == 0) {
                ext_err |= sensor_channel_get(ext_accel_dev, SENSOR_CHAN_ACCEL_X, &ax);
                ext_err |= sensor_channel_get(ext_accel_dev, SENSOR_CHAN_ACCEL_Y, &ay);
                ext_err |= sensor_channel_get(ext_accel_dev, SENSOR_CHAN_ACCEL_Z, &az);
            }

            got_ext_accel = (ext_err == 0);

            if (got_ext_accel && !ext_accel_fetch_ok) {
                LOG_INF("External accel recovered");
                ext_accel_fetch_ok = true;
            } else if (!got_ext_accel && ext_accel_fetch_ok) {
                LOG_WRN("External accel fetch failed (%d), falling back to internal", ext_err);
                ext_accel_fetch_ok = false;
            }
        }
#endif

        if (!got_ext_accel) {
            err = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &ax);
            err |= sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &ay);
            err |= sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &az);
            if (err < 0) {
                LOG_WRN("IMU accel read failed: %d", err);
                k_sleep(K_MSEC(IMU_POLL_PERIOD_MS));
                continue;
            }
        }

        err = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gx);
        err |= sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gy);
        err |= sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gz);
        if (err < 0) {
            LOG_WRN("IMU gyro read failed: %d", err);
            k_sleep(K_MSEC(IMU_POLL_PERIOD_MS));
            continue;
        }

        sample_count++;
        if ((sample_count % IMU_TARGET_ODR_HZ) == 1U) {
            int32_t ax_milli = (ax.val1 * 1000) + (ax.val2 / 1000);
            int32_t ay_milli = (ay.val1 * 1000) + (ay.val2 / 1000);
            int32_t az_milli = (az.val1 * 1000) + (az.val2 / 1000);

            LOG_INF("IMU samples=%u src=%s accel_milli_ms2=(%d,%d,%d)",
                    sample_count, got_ext_accel ? "ext" : "int",
                    ax_milli, ay_milli, az_milli);
        }

#if DT_NODE_EXISTS(DT_ALIAS(accel0))
        double accel_scale = got_ext_accel ? ADXL375_SCALE : 1.0;
#else
        double accel_scale = 1.0;
#endif

        imu_sample_t s = {
            .accel_x = sensor_value_to_double(&ax) * accel_scale,
            .accel_y = sensor_value_to_double(&ay) * accel_scale,
            .accel_z = sensor_value_to_double(&az) * accel_scale,
            .gyro_x  = sensor_value_to_double(&gx),
            .gyro_y  = sensor_value_to_double(&gy),
            .gyro_z  = sensor_value_to_double(&gz),
            .timestamp_us = k_uptime_get() * 1000,
        };

        if (user_cb) {
            user_cb(&s, user_ctx);
        }

        k_sleep(K_MSEC(IMU_POLL_PERIOD_MS));
    }
}

int imu_init(void)
{
    int err;

    if (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU device not ready");
        return -ENODEV;
    }

    struct sensor_value odr = {.val1 = IMU_TARGET_ODR_HZ};

    err = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (err < 0) {
        LOG_ERR("Failed to set accel ODR: %d", err);
        return err;
    }

    err = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (err < 0) {
        LOG_ERR("Failed to set gyro ODR: %d", err);
        return err;
    }

#if DT_NODE_EXISTS(DT_ALIAS(accel0))
    ext_accel_ready = device_is_ready(ext_accel_dev);
    ext_accel_fetch_ok = ext_accel_ready;
    if (ext_accel_ready) {
        /*
         * The adxl345 driver initialises DATA_FORMAT to RANGE_8G (0x02) without
         * FULL_RES (0x08).  RANGE_16G | FULL_RES (0x0B) puts the ADXL375 into
         * its full +/-200 g 13-bit mode (49 mg/LSB, ~20 counts per g), which is
         * what ADXL375_SCALE assumes.  RANGE_8G selects a +/-100 g sub-range
         * (~24 mg/LSB, ~41 counts per g) that throws the scale correction off.
         */
        const struct i2c_dt_spec adxl_i2c =
            I2C_DT_SPEC_GET(DT_ALIAS(accel0));
        uint8_t data_format_cmd[] = {0x31, 0x0B}; /* DATA_FORMAT = RANGE_16G | FULL_RES */
        int i2c_err = i2c_write_dt(&adxl_i2c, data_format_cmd, sizeof(data_format_cmd));

        if (i2c_err < 0) {
            LOG_WRN("ADXL375 FULL_RES enable failed: %d", i2c_err);
        } else {
            LOG_INF("ADXL375 FULL_RES enabled (13-bit, 49 mg/LSB)");
        }

        /*
         * adxl345_read_sample() busy-polls DATA_READY (2 I2C reads per loop)
         * when ADXL345_TRIGGER is not set.  Set ODR to 400 Hz so a fresh
         * sample is always ready by the time we poll, collapsing the loop to
         * a single iteration instead of ~30.
         */
        struct sensor_value ext_odr = {.val1 = 400};
        sensor_attr_set(ext_accel_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &ext_odr);
    }
    LOG_INF("Accel source: %s", ext_accel_ready ? "external ADXL375" : "internal LSM6DS3TR-C");
#endif

    return 0;
}

int imu_start(imu_data_cb_t cb, void *user_data)
{
    user_cb = cb;
    user_ctx = user_data;
    sample_count = 0U;

    atomic_set(&imu_poll_run, 1);
    k_thread_create(&imu_poll_thread, imu_poll_stack, IMU_POLL_STACK_SIZE,
                    imu_poll_fn, NULL, NULL, NULL,
                    IMU_POLL_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&imu_poll_thread, "imu_poll");

    return 0;
}

void imu_stop(void)
{
    atomic_set(&imu_poll_run, 0);
    k_thread_join(&imu_poll_thread, K_MSEC(100));
    user_cb = NULL;
    user_ctx = NULL;
}
