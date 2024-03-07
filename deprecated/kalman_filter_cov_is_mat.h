#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "typedefs.h"


#ifdef __cplusplus
extern "C"{
#endif

    void kf_init(void);
    void kf_initialize_track(const detection_t* measurement, ELEM_T* mean, ELEM_T* covariance);
    void kf_predict(ELEM_T* mean, ELEM_T* covariance);
    void kf_project(const ELEM_T* mean, const ELEM_T* covariance);
    void kf_update(const detection_t* measurement, ELEM_T* mean, ELEM_T* covariance);
    void kf_destroy(void);

#ifdef __cplusplus
}
#endif

#endif