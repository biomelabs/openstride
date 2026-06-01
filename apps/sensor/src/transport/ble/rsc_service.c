#include "transport/ble/rsc_service.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rsc_svc, LOG_LEVEL_INF);

/* RSC Feature characteristic bits (GATT spec 3.115.1) */
#define RSC_FEAT_STRIDE_LEN    BIT(0)
#define RSC_FEAT_TOTAL_DIST    BIT(1)
#define RSC_FEAT_WALK_RUN_STAT BIT(2)

/* RSC Measurement flags */
#define RSC_FLAG_STRIDE        BIT(0)
#define RSC_FLAG_DIST          BIT(1)
#define RSC_FLAG_RUNNING       BIT(2)

/* Sensor Location: In-Shoe (GATT spec table 3.108) */
#define RSC_SENSOR_LOC_IN_SHOE 0x02u

/* Speed at/above which we report the "running" status bit (vs walking). */
#define RSC_RUNNING_SPEED_THRESHOLD_MPS 2.0f

static bool notify_enabled;

static void rsc_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("RSC notifications %s", notify_enabled ? "enabled" : "disabled");
}

static ssize_t read_rsc_feature(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	uint16_t features = RSC_FEAT_STRIDE_LEN | RSC_FEAT_TOTAL_DIST | RSC_FEAT_WALK_RUN_STAT;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &features, sizeof(features));
}

static ssize_t read_sensor_location(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    void *buf, uint16_t len, uint16_t offset)
{
	uint8_t location = RSC_SENSOR_LOC_IN_SHOE;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &location, sizeof(location));
}

/*
 * GATT service attribute layout:
 *   [0] Primary Service declaration
 *   [1] RSC Measurement characteristic declaration
 *   [2] RSC Measurement characteristic value  <- bt_gatt_notify target
 *   [3] CCC descriptor
 *   [4] RSC Feature characteristic declaration
 *   [5] RSC Feature characteristic value
 *   [6] Sensor Location characteristic declaration
 *   [7] Sensor Location characteristic value
 */
BT_GATT_SERVICE_DEFINE(rsc_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_RSCS),
	BT_GATT_CHARACTERISTIC(BT_UUID_RSC_MEASUREMENT,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(rsc_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_RSC_FEATURE,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_rsc_feature, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LOCATION,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_sensor_location, NULL, NULL),
);

int rsc_service_init(void)
{
	return 0;
}

int rsc_service_notify(const sdm_data_t *data)
{
	if (!notify_enabled) {
		return 0;
	}

	/*
	 * RSC Measurement payload (all little-endian):
	 *   Flags          1 byte   uint8
	 *   Speed          2 bytes  uint16, 1/256 m/s
	 *   Cadence        1 byte   uint8,  strides/min
	 *   Stride Length  2 bytes  uint16, 1/100 m    (optional, present when RSC_FLAG_STRIDE)
	 *   Total Distance 4 bytes  uint32, 1/10 m     (optional, present when RSC_FLAG_DIST)
	 */
	uint8_t buf[10];
	uint8_t idx = 0;

	uint8_t flags = RSC_FLAG_STRIDE | RSC_FLAG_DIST;
	if (data->speed_mps > RSC_RUNNING_SPEED_THRESHOLD_MPS) {
		flags |= RSC_FLAG_RUNNING;
	}

	uint16_t speed_raw = (uint16_t)((data->speed_mps * 256.0f) + 0.5f);
	uint8_t  cadence_raw = (uint8_t)(data->cadence_spm > 255.0f ? 255.0f : data->cadence_spm);

	/*
	 * Report the stride length the algorithm is actually using. Deriving it
	 * from speed/cadence here is a no-op round-trip (speed is itself computed
	 * from cadence x stride length), so use the value directly and round.
	 */
	uint16_t stride_len_raw = 0;
	if (data->cadence_spm > 0.0f) {
		stride_len_raw = (uint16_t)((data->stride_length_m * 100.0f) + 0.5f);
	}

	uint32_t dist_raw = (uint32_t)((data->distance_m * 10.0f) + 0.5f);

	buf[idx++] = flags;
	buf[idx++] = (uint8_t)(speed_raw & 0xFFu);
	buf[idx++] = (uint8_t)(speed_raw >> 8u);
	buf[idx++] = cadence_raw;
	buf[idx++] = (uint8_t)(stride_len_raw & 0xFFu);
	buf[idx++] = (uint8_t)(stride_len_raw >> 8u);
	buf[idx++] = (uint8_t)(dist_raw & 0xFFu);
	buf[idx++] = (uint8_t)((dist_raw >> 8u) & 0xFFu);
	buf[idx++] = (uint8_t)((dist_raw >> 16u) & 0xFFu);
	buf[idx++] = (uint8_t)((dist_raw >> 24u) & 0xFFu);

	return bt_gatt_notify(NULL, &rsc_svc.attrs[2], buf, idx);
}
