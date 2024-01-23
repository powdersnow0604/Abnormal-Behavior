#ifndef __TYPEDEFS_H__
#define __TYPEDEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

#ifdef _MSC_VER
#define ALIGN_ATTR(x) __declspec(align((x)))
#else
#define ALIGN_ATTR(x) __attribute__((aligned((x))))
#endif

//functions for ELEM_T
#define SQRT_T sqrtf
#define GEMV cblas_sgemv
#define GEMM cblas_sgemm
#define POSV LAPACKE_sposv

//should be floating point
typedef float ELEM_T;

typedef uint16_t ID_TYPE;

typedef struct {
	ELEM_T e1;
	ELEM_T e2;
	ELEM_T e3;
	ELEM_T e4;
}QELEM_T;

typedef int16_t index_t;

typedef uint8_t age_t;

const uint8_t kf_sv_size = 7;
const uint8_t kf_mv_size = 4;
const uint8_t kf_delta_num = kf_sv_size - kf_mv_size;
const uint8_t kf_cov_size = 4;

//max tracks is also max detections
//index_t 가 허용 가능한 최대의 양수가 tk_max_tracks * 2 이상이어야 함
const index_t tk_max_tracks = 64; 

const ELEM_T iou_threshold = 0.3;

const age_t tk_max_age = 1;

typedef struct
{
    ELEM_T x1;                               /**< top-left corner: x */
    ELEM_T y1;                               /**< top-left corner: y */
    ELEM_T x2;                               /**< bottom-right corner: x */
    ELEM_T y2;                               /**< bottom-right corner: y */
    ELEM_T score;                            /**< probability score */
    int32_t class_num;                      /**< class # (of many) with highest probability */
} ALIGN_ATTR(4) detection_t;

#ifdef __cplusplus
}
#endif

#endif