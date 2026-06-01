#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

typedef struct imu_sample {
    float accel_x, accel_y, accel_z; // m/s^2
    float gyro_x, gyro_y, gyro_z;    // rad/s
    int64_t timestamp_us;
} imu_sample_t;

typedef void (*imu_data_cb_t)(const imu_sample_t *s, void *user_data);

int imu_init(void);
int imu_start(imu_data_cb_t cb, void *user_data);
void imu_stop(void);

#endif // IMU_H_
