#ifndef __TYPEDEFS_H__
#define __TYPEDEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

//functions for ELEM_T
#define SQRT_T sqrt
#define GEMV cblas_dgemv
#define GEMM cblas_dgemm
#define POSV LAPACKE_dposv

	//should be floating point
	typedef double ELEM_T;

	typedef uint16_t ID_TYPE;

	typedef struct {
		ELEM_T e1;
		ELEM_T e2;
		ELEM_T e3;
		ELEM_T e4;
	}QELEM_T;

	typedef int8_t index_t;

	typedef uint8_t age_t;

	const uint8_t kf_sv_size = 7;
    const uint8_t kf_mv_size = 4;
    const uint8_t kf_delta_num = kf_sv_size - kf_mv_size;
    const uint8_t kf_cov_size = 4;

	//max tracks is also max detections
	const index_t tk_max_tracks = 64; 

#ifdef __cplusplus
}
#endif

#endif