cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(openstride_unit_tests)

target_sources(app PRIVATE
    ../../apps/sensor/src/algo/foot/stride_mahony.c
    ../../apps/sensor/src/sdm/sdm_encode.c
    src/test_sdm.c
)

target_include_directories(app PRIVATE ../../apps/sensor/src)
