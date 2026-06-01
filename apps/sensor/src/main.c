#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "algo/foot/stride.h"
#include "transport/transport.h"

#if DT_HAS_COMPAT_STATUS_OKAY(st_lsm6dsl)
#include "imu/imu.h"
#endif

#if IS_ENABLED(CONFIG_BT)
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Bluetooth RSC Measurement: max 4 notifications per second. */
#define RSC_NOTIFY_MIN_INTERVAL_MS 250
#define RSC_NOTIFY_POLL_MS         50

#if IS_ENABLED(CONFIG_BT)
/* Advertising data: flags + RSC service UUID (0x1814) */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_RSCS_VAL)),
};

/* Scan response: full device name */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int start_advertising(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("adv_start: %d", err);
	} else {
		LOG_INF("Advertising as \"%s\" (RSC 0x1814)", CONFIG_BT_DEVICE_NAME);
	}

	return err;
}

static void adv_restart_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	(void)start_advertising();
}

static K_WORK_DEFINE(adv_restart_work, adv_restart_work_handler);

static void on_conn_recycled(void)
{
	/* Restart connectable advertising after the connection object is freed. */
	k_work_submit(&adv_restart_work);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	LOG_INF("Disconnected (reason 0x%02x)", reason);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.disconnected = on_disconnected,
	.recycled = on_conn_recycled,
};
#endif /* CONFIG_BT */

static stride_detector_t detector;
static sdm_data_t        latest_sdm;
static sdm_data_t        last_notified_sdm;
static int64_t           last_notify_ms;
K_MUTEX_DEFINE(sdm_mtx);

static void notify_sdm_if_due(const sdm_data_t *snap, bool force)
{
	int64_t now_ms = k_uptime_get();

	if (!force && ((now_ms - last_notify_ms) < RSC_NOTIFY_MIN_INTERVAL_MS)) {
		return;
	}

	transport_post(snap);
	last_notified_sdm = *snap;
	last_notify_ms = now_ms;
}

#if DT_HAS_COMPAT_STATUS_OKAY(st_lsm6dsl)
static void on_imu(const imu_sample_t *s, void *ctx)
{
	ARG_UNUSED(ctx);
	float accel[3] = {s->accel_x, s->accel_y, s->accel_z};
	float gyro[3] = {s->gyro_x, s->gyro_y, s->gyro_z};
	sdm_data_t data;

	bool stride_hit = stride_detector_update(&detector, accel, gyro, s->timestamp_us, &data);

	k_mutex_lock(&sdm_mtx, K_FOREVER);
	latest_sdm = data;
	k_mutex_unlock(&sdm_mtx);

	notify_sdm_if_due(&data, stride_hit);
}
#endif /* st_lsm6dsl */

int main(void)
{
	LOG_INF("OpenStride starting");

	stride_detector_init(&detector);
	latest_sdm = sdm_data_zero();
	last_notified_sdm = sdm_data_zero();
	last_notify_ms = 0;

#if IS_ENABLED(CONFIG_BT)
	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable: %d", err);
		return err;
	}
#endif

	transport_init();

#if IS_ENABLED(CONFIG_BT)
	int err2 = start_advertising();

	if (err2) {
		return err2;
	}
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(st_lsm6dsl)
	int imu_err = imu_init();
	if (imu_err == 0) {
		imu_err = imu_start(on_imu, NULL);
	}

	if (imu_err == 0) {
		LOG_INF("IMU started");
	} else {
		LOG_WRN("IMU start failed: %d", imu_err);
	}
#endif

	/* Push BLE updates on metric changes, capped at the RSC 4 Hz limit. */
	while (true) {
		sdm_data_t snap;

		k_sleep(K_MSEC(RSC_NOTIFY_POLL_MS));
		k_mutex_lock(&sdm_mtx, K_FOREVER);
		snap = latest_sdm;
		k_mutex_unlock(&sdm_mtx);
		notify_sdm_if_due(&snap, false);
	}

	return 0;
}
