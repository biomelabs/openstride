#ifndef BATTERY_H_
#define BATTERY_H_

/* Initialize battery monitoring and seed the BLE Battery Service level.
 * Safe to call even when no ADC channel is configured in DTS (becomes a
 * no-op that reports 100 % to indicate USB/external power). */
int battery_init(void);

#endif /* BATTERY_H_ */
