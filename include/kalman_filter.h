#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "typedefs.h"

//as matrix, state covariance should be:
// a 0 0 0 d 0 0
// 0 a 0 0 0 d 0
// 0 0 a 0 0 0 d
// 0 0 0 b 0 0 0
// d 0 0 0 c 0 0
// 0 d 0 0 0 c 0
// 0 0 d 0 0 0 c 

//state covariance is stored in the format below
// [ a b c d ]

//as matrix, kalman gain should be:
// a 0 0 0 
// 0 a 0 0 
// 0 0 a 0 
// 0 0 0 b 
// c 0 0 0 
// 0 c 0 0 
// 0 0 c 0 

//kalman gain is stored in the format below
// [ a b c ]

#ifdef __cplusplus
extern "C"{
#endif

    void kf_init(void);
    void kf_initialize_track(const detection_t* measurement, ELEM_T* mean, ELEM_T* covariance);
    void kf_predict(ELEM_T* mean, ELEM_T* covariance);
    void kf_update(const detection_t* measurement, ELEM_T* mean, ELEM_T* covariance);
    void kf_destroy(void);

#ifdef __cplusplus
}
#endif

#endif