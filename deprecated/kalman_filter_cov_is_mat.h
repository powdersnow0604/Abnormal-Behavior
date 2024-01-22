#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "typedefs.h"


#ifdef __cplusplus
extern "C"{
#endif

    const int8_t kf_sv_size = 7;
    const int8_t kf_mv_size = 4;
    const int8_t kf_delta_num = kf_sv_size - kf_mv_size;

    void kf_init(void);
    void kf_initialize_track(const ELEM_T* measurement, ELEM_T* mean, ELEM_T* covariance);
    void kf_predict(ELEM_T* mean, ELEM_T* covariance);
    void kf_project(const ELEM_T* mean, const ELEM_T* covariance);
    void kf_update(const ELEM_T* measurement, ELEM_T* mean, ELEM_T* covariance);
    void kf_destroy(void);

#ifdef __cplusplus
}
#endif

#endif