#include "transport/ble/imu_stream.h"

#include <string.h>
#include <zephyr/logging/log.h>
#include <bluetooth/services/nus.h>

LOG_MODULE_REGISTER(imu_stream, LOG_LEVEL_INF);

/*
 * Wire format (little-endian, packed):
 *   uint32_t ts_ms      milliseconds since boot
 *   float    accel_x/y/z  m/s²
 *   float    gyro_x/y/z   rad/s
 * = 28 bytes per sample
 *
 * 8 samples batched per NUS notify = 224 bytes, fits inside a 247-byte MTU.
 * At 416 Hz this fires ~52 times/sec.
 */
#define SAMPLE_SIZE  28
#define BATCH_COUNT  2   /* fits in default ATT MTU=65 (payload 62 bytes) */
#define BUF_SIZE     (SAMPLE_SIZE * BATCH_COUNT)

static uint8_t batch_buf[BUF_SIZE];
static uint8_t batch_pos;
static bool    stream_enabled;

static void on_send_enabled(enum bt_nus_send_status status)
{
	stream_enabled = (status == BT_NUS_SEND_STATUS_ENABLED);
	batch_pos = 0;
	LOG_INF("IMU stream %s", stream_enabled ? "on" : "off");
}

static struct bt_nus_cb nus_cb = {
	.send_enabled = on_send_enabled,
};

int imu_stream_init(void)
{
	return bt_nus_init(&nus_cb);
}

void imu_stream_send(const imu_sample_t *s)
{
	if (!stream_enabled) {
		return;
	}

	uint8_t *p = &batch_buf[batch_pos * SAMPLE_SIZE];
	uint32_t ts_ms = (uint32_t)(s->timestamp_us / 1000);

	memcpy(p,      &ts_ms,      4);
	memcpy(p +  4, &s->accel_x, 4);
	memcpy(p +  8, &s->accel_y, 4);
	memcpy(p + 12, &s->accel_z, 4);
	memcpy(p + 16, &s->gyro_x,  4);
	memcpy(p + 20, &s->gyro_y,  4);
	memcpy(p + 24, &s->gyro_z,  4);

	batch_pos++;
	if (batch_pos >= BATCH_COUNT) {
		int err = bt_nus_send(NULL, batch_buf, BUF_SIZE);

		if (err && err != -EAGAIN && err != -ENOTCONN) {
			LOG_WRN("nus_send: %d", err);
		}
		batch_pos = 0;
	}
}
