#include "kalman_filter.h"
#include <stdlib.h>
#include <string.h>
#include "iou.h"

#ifdef KF_DEBUG
#include <stdio.h>
#endif

#ifdef _MSC_VER
#define ALIGNED_ALLOC(align, size) _aligned_malloc((size), (align))
#else
#define ALIGNED_ALLOC(align, size) aligned_alloc((align), (size))
#endif

#ifdef _MSC_VER
#define ALIGNED_FREE(ptr) _aligned_free((ptr))
#else
#define ALIGNED_FREE(ptr) free((ptr))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#include "errctl.h"

    static ELEM_T *kf_kalman_gain;
    static ELEM_T *kf_temp_det;

    const uint8_t kf_sv_size = 7;
    const uint8_t kf_mv_size = 4;
    const uint8_t kf_delta_num = kf_sv_size - kf_mv_size;
    const uint8_t kf_cov_size = 4;

#ifdef KF_DYNAMIC_COV
    static const ELEM_T std_weight_position = 1. / 20;
    static const ELEM_T std_weight_velocity = 1. / 160;
#endif

#ifdef KF_DEBUG
    static void print_mat(const char *des, const ELEM_T *mat, int8_t row, int8_t col)
    {
        int8_t i, j;
        const char *fmt;

        printf("[DEBUG][%s]<%s>\n\n", __func__, des);

        if (sizeof(ELEM_T) == 8)
        {
            fmt = "%.8lf\t";
        }
        else
        {
            fmt = "%.8f\t";
        }

        for (i = 0; i < row; i++)
        {
            for (j = 0; j < col; j++)
                printf(fmt, mat[i * col + j]);
            puts("");
        }

        puts("");
    }

    static void print_cov(const char *des, const ELEM_T *cov, int8_t row, int8_t col)
    {
        int8_t i, j;
        const char *fmt;

        printf("[DEBUG][%s]<%s>\n\n", __func__, des);

        if (sizeof(ELEM_T) == 8)
        {
            fmt = "%.8lf\t";
        }
        else
        {
            fmt = "%.8f\t";
        }

        for (i = 0; i < row; i++)
        {
            for (j = 0; j < col; j++)
            {
                if (i == j && i < 3)
                    printf(fmt, cov[0]);
                else if (i == j && i == 3)
                    printf(fmt, cov[1]);
                else if (i == j && i > 3)
                    printf(fmt, cov[2]);
                else if (i < kf_delta_num && j == kf_mv_size + i)
                    printf(fmt, cov[3]);
                else if (kf_mv_size <= i && j == i - kf_mv_size)
                    printf(fmt, cov[3]);
                else
                    printf(fmt, 0.);
            }
            puts("");
        }

        puts("");
    }
#endif

    void kf_init(void)
    {
        kf_kalman_gain = (ELEM_T *)ALIGNED_ALLOC(ALIGNMENT, 3 * sizeof(ELEM_T));
        if (kf_kalman_gain == NULL)
            err_sys("alloc error kf_kalman_gain");

        kf_temp_det = (ELEM_T *)ALIGNED_ALLOC(ALIGNMENT, kf_mv_size * sizeof(ELEM_T));
        if (kf_temp_det == NULL)
            err_sys("alloc error kf_temo_det");
    }

    void kf_initialize_track(const detection_t *measurement, ELEM_T *mean, ELEM_T *covariance)
    {
        // measurement 는 tlbr 로 입력되는 것을 상정
        index_t i;

        // initialize mean
        for (i = kf_mv_size; i < kf_sv_size; ++i)
        {
            mean[i] = 0;
        }

        to_xysr(measurement, mean);

        // initialize covariance
        covariance[3] = 0;

#ifdef KF_DYNAMIC_COV
        covariance[2] = measurement[3] - measurement[1] + 1;

        covariance[0] = 100 * std_weight_velocity * covariance[2] * std_weight_velocity * covariance[2];
        covariance[1] = 1e-4;
        covariance[2] = 4 * std_weight_position * covariance[2] * std_weight_position * covariance[2];
#else
    covariance[0] = 10;    // 1
    covariance[1] = 10;    // 1
    covariance[2] = 10000; // 1
#endif

#ifdef KF_DEBUG
        print_mat("kf construct track mean", mean, kf_sv_size, 1);
        print_cov("kf construct track covariance", covariance, kf_sv_size, kf_sv_size);
#endif
    }

    void kf_predict(ELEM_T *mean, ELEM_T *covariance)
    {
        // predict mean
        mean[0] += mean[4];
        mean[1] += mean[5];
        mean[2] += mean[6];

        // predict covariance
        covariance[0] += covariance[2] + covariance[3] + covariance[3];
        covariance[3] += covariance[2];

#ifdef KF_DYNAMIC_COV
        ELEM_T h = mean[2] / SQRT_T(mean[2] * mean[3]);
        covariance[0] += std_weight_position * h * std_weight_position * h;
        covariance[1] += 1e-4;
        covariance[2] += std_weight_velocity * h * std_weight_position * h;
#else
    covariance[0] += 1;    // 1e-2
    covariance[1] += 1;    // 1e-2
    covariance[2] += 1e-1; // 1e-2
#endif

#ifdef KF_DEBUG
        print_mat("kf_predict mean", mean, kf_sv_size, 1);
        print_cov("kf_predict covariance", covariance, kf_sv_size, kf_sv_size);
#endif
    }

    void kf_update(const detection_t *measurement, ELEM_T *mean, ELEM_T *covariance)
    {
        // measurement 는 tlbr 로 입력되는 것을 상정

        index_t i;

// calculate kalman gain
#ifdef KF_DYNAMIC_COV
        ELEM_T kf_temp_det[0] = mean[2] / SQRT_T(mean[2] * mean[3]);
        kf_temp_det[0] = std_weight_position * kf_temp_det[0] * std_weight_position * kf_temp_det[0];

        kf_kalman_gain[0] = covariance[0] / (covariance[0] + kf_temp_det[0]);
        kf_kalman_gain[1] = covariance[1] / (covariance[1] + 1e-2);
        kf_kalman_gain[2] = covariance[3] / (covariance[0] + kf_temp_det[0]);
#else
    kf_kalman_gain[0] = covariance[0] / (covariance[0] + 1);  // 1e-1
    kf_kalman_gain[1] = covariance[1] / (covariance[1] + 10); // 1e-1
    kf_kalman_gain[2] = covariance[3] / (covariance[0] + 1);  // 1e-1
#endif

        // measurement: tlbr -> xysr
        to_xysr(measurement, kf_temp_det);

        // calculate innovation
        for (i = 0; i < kf_mv_size; ++i)
        {
            kf_temp_det[i] -= mean[i];
        }

        // update mean
        for (i = 0; i < kf_delta_num; ++i)
        {
            mean[i] += kf_kalman_gain[0] * kf_temp_det[i];
        }

        for (; i < kf_mv_size; ++i)
        {
            mean[i] += kf_kalman_gain[1] * kf_temp_det[i];
        }

        for (; i < kf_sv_size; ++i)
        {
            mean[i] += kf_kalman_gain[2] * kf_temp_det[i - kf_mv_size];
        }

        // update covariance
        covariance[0] -= kf_kalman_gain[0] * covariance[0];
        covariance[1] -= kf_kalman_gain[1] * covariance[1];
        covariance[2] -= kf_kalman_gain[2] * covariance[3];
        covariance[3] -= kf_kalman_gain[0] * covariance[3];

#ifdef KF_DEBUG
        print_mat("kf_update mean", mean, kf_sv_size, 1);
        print_cov("kf_update covariance", covariance, kf_sv_size, kf_sv_size);
#endif
    }
    void kf_destroy(void)
    {
        ALIGNED_FREE(kf_kalman_gain);
        ALIGNED_FREE(kf_temp_det);
    }

#ifdef __cplusplus
}
#endif