#include "transport/ble/imu_stream.h"

#include <string.h>
#include <zephyr/kernel.h>
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
 * Up to 8 samples batched per NUS notify = 224 bytes, fits inside a 247-byte
 * ATT MTU (244-byte payload).  batch_count is set dynamically after MTU
 * exchange; defaults to 2 to fit the 65-byte fallback MTU (62-byte payload).
 *
 * imu_stream_send() enqueues into imu_tx_queue (non-blocking, drops on full)
 * so the IMU poll thread never waits on BLE.  The imu_tx thread drains the
 * queue, packs batches, and calls bt_nus_send().
 */
#define SAMPLE_SIZE      28
#define BATCH_COUNT_MAX   8
#define BUF_SIZE         (SAMPLE_SIZE * BATCH_COUNT_MAX)

/* ~150 ms of samples at 416 Hz — enough headroom for a BLE connection event. */
#define TX_QUEUE_DEPTH   64

#define TX_STACK_SIZE   768
/* One below the IMU poll thread (priority 7) so math is never preempted by TX. */
#define TX_PRIORITY       8

K_MSGQ_DEFINE(imu_tx_queue, sizeof(imu_sample_t), TX_QUEUE_DEPTH, 4);

K_THREAD_STACK_DEFINE(imu_tx_stack, TX_STACK_SIZE);
static struct k_thread imu_tx_thread;

static uint8_t batch_count = 2;
static bool    stream_enabled;

static void on_send_enabled(enum bt_nus_send_status status)
{
	stream_enabled = (status == BT_NUS_SEND_STATUS_ENABLED);
	if (!stream_enabled) {
		k_msgq_purge(&imu_tx_queue);
	}
	LOG_INF("IMU stream %s (batch=%u)", stream_enabled ? "on" : "off", batch_count);
}

static struct bt_nus_cb nus_cb = {
	.send_enabled = on_send_enabled,
};

static void imu_tx_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint8_t batch_buf[BUF_SIZE];
	uint8_t pos = 0;

	while (true) {
		imu_sample_t s;

		k_msgq_get(&imu_tx_queue, &s, K_FOREVER);

		if (!stream_enabled) {
			pos = 0;
			continue;
		}

		uint8_t *p = &batch_buf[pos * SAMPLE_SIZE];
		uint32_t ts_ms = (uint32_t)(s.timestamp_us / 1000);

		memcpy(p,      &ts_ms,     4);
		memcpy(p +  4, &s.accel_x, 4);
		memcpy(p +  8, &s.accel_y, 4);
		memcpy(p + 12, &s.accel_z, 4);
		memcpy(p + 16, &s.gyro_x,  4);
		memcpy(p + 20, &s.gyro_y,  4);
		memcpy(p + 24, &s.gyro_z,  4);
		pos++;

		if (pos >= batch_count) {
			int err = bt_nus_send(NULL, batch_buf, (size_t)pos * SAMPLE_SIZE);

			if (err && err != -EAGAIN && err != -ENOTCONN) {
				LOG_WRN("nus_send: %d", err);
			}
			pos = 0;
		}
	}
}

int imu_stream_init(void)
{
	k_thread_create(&imu_tx_thread, imu_tx_stack, TX_STACK_SIZE,
			imu_tx_fn, NULL, NULL, NULL,
			TX_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&imu_tx_thread, "imu_tx");

	return bt_nus_init(&nus_cb);
}

void imu_stream_on_mtu_updated(uint16_t mtu)
{
	uint8_t new_count = (uint8_t)MIN(BATCH_COUNT_MAX, (mtu - 3) / SAMPLE_SIZE);

	if (new_count < 1) {
		new_count = 1;
	}
	batch_count = new_count;
	LOG_INF("IMU stream batch_count=%u (ATT MTU=%u)", batch_count, mtu);
}

void imu_stream_send(const imu_sample_t *s)
{
	if (!stream_enabled) {
		return;
	}
	if (k_msgq_put(&imu_tx_queue, s, K_NO_WAIT) != 0) {
		LOG_DBG("imu_tx_queue full, sample dropped");
	}
}
