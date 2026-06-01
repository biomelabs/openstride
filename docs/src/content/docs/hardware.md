---
title: Hardware
description: Components that make up an openstride foot pod.
---

All parts are available on Adafruit, DigiKey, Amazon, and AliExpress depending on stock.
If you have trouble sourcing the XIAO nRF54L15 Sense, it's also possible to target the
older nRF52840 Sense with a board overlay — see the README for notes on this.

## Seeed Studio XIAO nRF54L15 Sense

The main MCU. Powered by Nordic Semiconductor's nRF54L15 SoC: 128 MHz Arm Cortex-M33,
ultra-low-power sleep modes, BLE 6.0, Matter, and Zigbee. Available directly from
Seeed Studio or their official AliExpress store, including versions with pre-soldered headers.

The XIAO nRF54L15 Sense includes an onboard SAMD11 CMSIS-DAPv2 co-processor, so
`west flash` and `west debug` work over USB without an external debug probe.

## Adafruit ADXL375 High-G Accelerometer (PID 5374)

±200g, 3-axis MEMS accelerometer. The XIAO has a 16g onboard IMU, but that clips
foot-strike impacts — the ADXL375 captures the full shock signature needed for
accurate stride detection. Communicates over I2C (address 0x53). Includes a 32-level
FIFO buffer and two independent interrupt pins for motion detection.

Daisy-chained to the BMP388 via a 50mm Qwiic / STEMMA QT cable.

## Adafruit BMP388 Barometric Pressure Sensor (PID 3966)

Bosch next-generation barometric pressure sensor with ±0.5m altitude resolution
(8 Pa relative accuracy). Used for grade and elevation tracking. I2C address 0x76/0x77,
3V–5V compatible. Should be the top board in any stacked arrangement to allow a vent
to the outside through the enclosure lid.

## 3.7V LiPo Battery

Matched to the XIAO's onboard BAT+/BAT− pads and charging circuit.
A 502030 form factor (approximately 300–350 mAh) fits within a shoe-mounted enclosure.

## Cables

- **50mm Qwiic/STEMMA QT cable** — daisy-chains the ADXL375 to the BMP388
- **4-pin female jumper cable** — routes the I2C chain back to the XIAO header pins