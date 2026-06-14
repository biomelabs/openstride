#ifndef IMU_STREAM_H_
#define IMU_STREAM_H_

#include "imu/imu.h"

int  imu_stream_init(void);
void imu_stream_send(const imu_sample_t *s);
void imu_stream_on_mtu_updated(uint16_t mtu);

#endif /* IMU_STREAM_H_ */
