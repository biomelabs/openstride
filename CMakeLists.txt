cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(openstride)

target_sources(app PRIVATE
    apps/sensor/src/main.c
    apps/sensor/src/algo/foot/stride.c
    apps/sensor/src/sdm/sdm_encode.c
    apps/sensor/src/transport/transport.c
)

# IMU driver: imu.c uses imu0 (LSM6DS3TR-C) for timing/gyro and optionally
# accel0 (ADXL375 via ADXL345 driver) as the preferred accel source.
if(CONFIG_LSM6DSL OR CONFIG_LSM6DSO OR CONFIG_ADXL345)
    target_sources(app PRIVATE apps/sensor/src/imu/imu.c)
endif()

# ANT transport is opt-in (sdk-ant private repo).
if(CONFIG_ANT)
    target_sources(app PRIVATE apps/sensor/src/transport/ant/ant_transport.c)
endif()

# BLE RSC service only when the BT stack is present.
if(CONFIG_BT)
    target_sources(app PRIVATE apps/sensor/src/transport/ble/rsc_service.c)
endif()

# Battery module: ADC sampling + BLE Battery Service reporting.
if(CONFIG_BT_BAS)
    target_sources(app PRIVATE apps/sensor/src/battery/battery.c)
endif()

target_include_directories(app PRIVATE apps/sensor/src)
