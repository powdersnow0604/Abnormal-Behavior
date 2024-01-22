#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "typedefs.h"

//as matrix, state covariance should be:
// a 0 0 0 h 0 0
// 0 b 0 0 0 i 0
// 0 0 c 0 0 0 j
// 0 0 0 d 0 0 0
// k 0 0 0 e 0 0
// 0 l 0 0 0 f 0
// 0 0 m 0 0 0 g 

//state covariance is stored in the format below
// [ a b c d e f g h i j k l m]

//as matrix, kalman gain should be:
// a 0 0 0 
// 0 b 0 0 
// 0 0 c 0 
// 0 0 0 d 
// e 0 0 0 
// 0 f 0 0 
// 0 0 g 0 

//kalman gain is stored in the format below
// [ a b c d e f g]

#ifdef __cplusplus
extern "C"{
#endif

    const int8_t kf_sv_size = 7;
    const int8_t kf_mv_size = 4;
    const int8_t kf_delta_num = kf_sv_size - kf_mv_size;

    void kf_init(void);
    void kf_initialize_track(const ELEM_T* measurement, ELEM_T* mean, ELEM_T* covariance);
    void kf_predict(ELEM_T* mean, ELEM_T* covariance);
    void kf_update(const ELEM_T* measurement, ELEM_T* mean, ELEM_T* covariance);
    void kf_destroy(void);

#ifdef __cplusplus
}
#endif

#endif