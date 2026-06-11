#include "battery/battery.h"

#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

/* Gate ADC path on whether an io-channels entry exists under /zephyr,user. */
#define BAT_NODE   DT_PATH(zephyr_user)
#define HAS_BATADC DT_NODE_HAS_PROP(BAT_NODE, io_channels)

#if HAS_BATADC

#include <zephyr/drivers/adc.h>

static const struct adc_dt_spec bat_chan = ADC_DT_SPEC_GET(BAT_NODE);

/* Voltage divider: VBAT → (R_top + R_bot) / R_bot × ADC_mV
 * Both XIAO nRF52840 Sense and nRF54L15 use equal resistors → ratio 2. */
#define BAT_DIV_NUMERATOR   2
#define BAT_DIV_DENOMINATOR 1

/* Some boards gate the voltage divider behind a regulator (e.g. XIAO nRF54L15
 * uses vbat_pwr / P1.15) to save power while not measuring. */
#define BAT_VBAT_PWR_NODE DT_NODELABEL(vbat_pwr)
#define HAS_VBAT_PWR      DT_NODE_EXISTS(BAT_VBAT_PWR_NODE)

#if HAS_VBAT_PWR
#include <zephyr/drivers/regulator.h>
static const struct device *bat_vbat_pwr = DEVICE_DT_GET(BAT_VBAT_PWR_NODE);
#endif

/* LiPo open-circuit voltage to capacity.  Pairs sorted highest-first. */
static const struct {
	uint16_t mv;
	uint8_t  pct;
} lipo_curve[] = {
	{4200, 100}, {4060, 90}, {3980, 80}, {3900, 70},
	{3830,  60}, {3750, 50}, {3680, 40}, {3600, 30},
	{3500,  20}, {3350, 10}, {3000,  0},
};

static uint8_t mv_to_pct(int32_t mv)
{
	const int n = ARRAY_SIZE(lipo_curve);

	if (mv >= lipo_curve[0].mv)        return 100;
	if (mv <= lipo_curve[n - 1].mv)    return 0;

	for (int i = 0; i < n - 1; i++) {
		if (mv >= lipo_curve[i + 1].mv) {
			int32_t v0 = lipo_curve[i + 1].mv;
			int32_t v1 = lipo_curve[i].mv;
			int32_t p0 = lipo_curve[i + 1].pct;
			int32_t p1 = lipo_curve[i].pct;

			return (uint8_t)(p0 + (p1 - p0) * (mv - v0) / (v1 - v0));
		}
	}
	return 0;
}

static int16_t adc_buf;
static struct adc_sequence adc_seq = {
	.buffer      = &adc_buf,
	.buffer_size = sizeof(adc_buf),
};

#define BAT_MEASURE_INTERVAL_MS (60 * 1000)

/* LiPo valid range – outside this we assume USB/external power → 100 %. */
#define BAT_MV_MIN 2700
#define BAT_MV_MAX 4350

static void measure_battery(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(bat_work, measure_battery);

static void measure_battery(struct k_work *work)
{
#if HAS_VBAT_PWR
	/* Enable voltage divider, wait 1 ms for it to settle, then read. */
	regulator_enable(bat_vbat_pwr);
	k_sleep(K_MSEC(1));
#endif

	int err = adc_read_dt(&bat_chan, &adc_seq);

#if HAS_VBAT_PWR
	regulator_disable(bat_vbat_pwr);
#endif

	if (err) {
		LOG_WRN("ADC read failed: %d", err);
		goto reschedule;
	}

	int32_t mv = adc_buf;

	err = adc_raw_to_millivolts_dt(&bat_chan, &mv);
	if (err) {
		LOG_WRN("ADC conversion failed: %d", err);
		goto reschedule;
	}

	/* Scale for the voltage divider in front of the ADC input. */
	mv = mv * BAT_DIV_NUMERATOR / BAT_DIV_DENOMINATOR;

	uint8_t level;

	if (mv < BAT_MV_MIN || mv > BAT_MV_MAX) {
		/* Out of LiPo range: device is on USB power with no battery. */
		level = 100;
		LOG_DBG("VBAT %d mV – USB power, reporting 100%%", mv);
	} else {
		level = mv_to_pct(mv);
		LOG_INF("VBAT %d mV → %u%%", mv, level);
	}

	bt_bas_set_battery_level(level);

reschedule:
	k_work_reschedule(&bat_work, K_MSEC(BAT_MEASURE_INTERVAL_MS));
}

int battery_init(void)
{
	if (!adc_is_ready_dt(&bat_chan)) {
		LOG_ERR("Battery ADC not ready");
		return -ENODEV;
	}

	int err = adc_channel_setup_dt(&bat_chan);

	if (err) {
		LOG_ERR("ADC channel setup: %d", err);
		return err;
	}

	adc_sequence_init_dt(&bat_chan, &adc_seq);

	/* First sample immediately; subsequent samples every minute. */
	k_work_reschedule(&bat_work, K_NO_WAIT);
	return 0;
}

#else /* HAS_BATADC */

int battery_init(void)
{
	/* No battery ADC in DTS – report 100 % (device runs on USB/external). */
	bt_bas_set_battery_level(100);
	LOG_INF("No battery ADC configured, reporting 100%%");
	return 0;
}

#endif /* HAS_BATADC */
