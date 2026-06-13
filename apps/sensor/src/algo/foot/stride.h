#ifndef STRIDE_H_
#define STRIDE_H_

/*
 * Algorithm selector. Include this header instead of stride_mahony.h or
 * stride_eskf.h directly, then control which implementation is active at
 * build time:
 *
 *   Mahony (default):
 *     cmake .. -DSTRIDE_USE_ESKF=OFF
 *
 *   ESKF:
 *     cmake .. -DSTRIDE_USE_ESKF=ON
 *
 * Both implementations expose the same public interface:
 *   stride_detector_t     – detector state struct
 *   stride_detector_init  – initialise a detector
 *   stride_detector_update – feed one IMU sample, receive sdm_data_t
 */

#ifdef STRIDE_USE_ESKF

#include "algo/foot/stride_eskf.h"

typedef stride_eskf_detector_t stride_detector_t;
#define stride_detector_init   stride_eskf_detector_init
#define stride_detector_update stride_eskf_detector_update

#else /* Mahony */

#include "algo/foot/stride_mahony.h"

#endif /* STRIDE_USE_ESKF */

#endif /* STRIDE_H_ */
