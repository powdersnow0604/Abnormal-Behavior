#include "kalman_filter.h"
#include <stdlib.h>
#include <string.h>
#include "iou.h"

#ifdef KF_NOPT_MM
#include "cblas.h"
#include "lapacke.h"
#endif


#ifdef KF_DEBUG
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C"{
#endif
    
    static ELEM_T* kf_motion_mat;
    static ELEM_T* kf_update_mat;
    static ELEM_T* kf_projected_cov; 
    static ELEM_T* kf_kalman_gain;
    static ELEM_T* kf_temp_det;

    #ifdef KF_NOPT_MM
    static ELEM_T* kf_temp_sv_sv;
    static ELEM_T* kf_temp_mv_sv;
    #endif

    #ifdef KF_DYNAMIC_COV
    static const ELEM_T std_weight_position = 1. / 20;
    static const ELEM_T std_weight_velocity = 1. / 160;
    #endif


    #ifdef KF_DEBUG
    static void print_mat(const char* des, const ELEM_T* mat, int8_t row, int8_t col)
    {
        int8_t i, j;
        const char* fmt;

        printf("[DEBUG][%s]<%s>\n\n", __func__, des);

        if(sizeof(ELEM_T) == 8){
            fmt = "%.8lf\t";
        }
        else{
            fmt = "%.8f\t";
        }

        for(i = 0; i < row; i++){
            for(j = 0; j < col; j++) printf(fmt, mat[i * col + j]);
            puts("");
        }

        puts("");
    }
    #endif

    void kf_init(void)
    {
        index_t i;

        kf_motion_mat = (ELEM_T*)calloc(kf_sv_size * kf_sv_size, sizeof(ELEM_T));
        kf_update_mat = (ELEM_T*)calloc(kf_mv_size * kf_sv_size, sizeof(ELEM_T));
        kf_projected_cov = (ELEM_T*)malloc(kf_mv_size * kf_mv_size * sizeof(ELEM_T));
        kf_kalman_gain = (ELEM_T*)calloc(kf_sv_size * kf_mv_size, sizeof(ELEM_T));
        kf_temp_det = (ELEM_T*)malloc(kf_mv_size * sizeof(ELEM_T));
        

        #ifdef KF_NOPT_MM
        kf_temp_sv_sv = (ELEM_T*)malloc(kf_sv_size * kf_sv_size * sizeof(ELEM_T));
        kf_temp_mv_sv = (ELEM_T*)malloc(kf_mv_size * kf_sv_size * sizeof(ELEM_T));
        #endif

        for(i = 0; i < kf_sv_size; i++){
            kf_motion_mat[i * kf_sv_size + i] = 1;
        }

        for(i = 0; i < kf_sv_size - kf_mv_size; i++){
            kf_motion_mat[i * kf_sv_size + i + kf_mv_size] = 1;
        }

        for(i = 0; i < kf_mv_size; i++){
            kf_update_mat[i * kf_sv_size + i] = 1;
        }

        #ifdef KF_DEBUG
        print_mat("kf_motion_mat", kf_motion_mat, kf_sv_size, kf_sv_size);
        print_mat("kf_update_mat", kf_update_mat, kf_mv_size, kf_sv_size);
        #endif
    }

    void kf_initialize_track(const ELEM_T* measurement, ELEM_T* mean, ELEM_T* covariance)
    {
        //measurement 는 tlbr 로 입력되는 것을 상정
        index_t i;

        for(i = kf_sv_size - 1; i >= kf_mv_size; --i){
            mean[i] = 0;
        }

        //to_xysr(measurement, mean);
        memcpy(mean, measurement, kf_mv_size * sizeof(ELEM_T));

        memset(covariance, 0, kf_sv_size * kf_sv_size * sizeof(ELEM_T));

        #ifdef KF_DYNAMIC_COV
        ELEM_T h = measurement[3] - measurement[1] + 1;
        //covariance[0 + kf_sv_size * 0] = 2 * std_weight_position * h;
        covariance[0] = 2 * std_weight_position * h * 2 * std_weight_position * h;
        //covariance[1 + kf_sv_size * 1] = 2 * std_weight_position * h;
        covariance[8] = 2 * std_weight_position * h * 2 * std_weight_position * h;
        //covariance[2 + kf_sv_size * 2] = 2 * std_weight_position * h;
        covariance[16] = 2 * std_weight_position * h * 2 * std_weight_position * h;
        //covariance[3 + kf_sv_size * 3] = 1e-2;
        covariance[24] = 1e-2 * 1e-2;
        //covariance[4 + kf_sv_size * 4] = 10 * std_weight_velocity * h;
        covariance[32] = 10 * std_weight_velocity * h * 10 * std_weight_velocity * h;
        //covariance[5 + kf_sv_size * 5] = 10 * std_weight_velocity * h;
        covariance[40] = 10 * std_weight_velocity * h * 10 * std_weight_velocity * h;
        //covariance[6 + kf_sv_size * 6] = 10 * std_weight_velocity * h;
        covariance[48] = 10 * std_weight_velocity * h * 10 * std_weight_velocity * h;
        #else
        covariance[0] = 1;
        covariance[8] = 1;
        covariance[16] = 1;
        covariance[24] = 1;
        covariance[32] = 1;
        covariance[40] = 1;
        covariance[48] = 1;
        #endif
        
        #ifdef KF_DEBUG
        print_mat("kf construct track mean", mean, kf_sv_size, 1);
        print_mat("kf construct track covariance", covariance, kf_sv_size, kf_sv_size);
        #endif
    }

    void kf_predict(ELEM_T* mean, ELEM_T* covariance)
    {
        //predict mean
        mean[0] += mean[4];
        mean[1] += mean[5];
        mean[2] += mean[6];

        //predict covariance
        #ifndef KF_NOPT_MM
        index_t i;
       
        for(i = kf_mv_size; i < kf_sv_size * kf_delta_num; i += kf_sv_size + 1){
            covariance[i] += covariance[i + kf_mv_size * kf_sv_size];
        }

        for(i = 0; i < kf_delta_num * kf_sv_size; i += kf_sv_size + 1){
            covariance[i] += covariance[i + kf_mv_size * kf_sv_size] + covariance[i + kf_mv_size];
        }

        for(i = kf_mv_size * kf_sv_size; i < kf_sv_size * kf_sv_size; i += kf_sv_size + 1){
            covariance[i] += covariance[i + kf_mv_size];
        }

        #else
        GEMM(CblasRowMajor, CblasNoTrans, CblasNoTrans, kf_sv_size, kf_sv_size, kf_sv_size, 1, kf_motion_mat, kf_sv_size,
        covariance, kf_sv_size, 0, kf_temp_sv_sv, kf_sv_size);
        GEMM(CblasRowMajor, CblasNoTrans, CblasTrans, kf_sv_size, kf_sv_size, kf_sv_size, 1, kf_temp_sv_sv, kf_sv_size,
        kf_motion_mat, kf_sv_size, 0, covariance, kf_sv_size);
        #endif

        
        #ifdef KF_DYNAMIC_COV
        ELEM_T h = mean[2] / SQRT_T(mean[2] * mean[3]);
        covariance[0] += std_weight_position * h * std_weight_position * h;
        covariance[8] += std_weight_position * h * std_weight_position * h;
        covariance[16] += std_weight_position * h * std_weight_position * h;
        covariance[24] += 1e-2 * 1e-2;
        covariance[32] += std_weight_velocity * h * std_weight_velocity * h;
        covariance[40] += std_weight_velocity * h * std_weight_velocity * h;
        covariance[48] += std_weight_velocity * h * std_weight_velocity * h;
        #else
        covariance[0] += 1e-2 * 1e-2;
        covariance[8] += 1e-2 * 1e-2;
        covariance[16] += 1e-2 * 1e-2;
        covariance[24] += 1e-2 * 1e-2;
        covariance[32] += 1e-2 * 1e-2;
        covariance[40] += 1e-2 * 1e-2;
        covariance[48] += 1e-2 * 1e-2;
        #endif
        

        #ifdef KF_DEBUG
        print_mat("kf_predict mean", mean, kf_sv_size, 1);
        print_mat("kf_predict covariance", covariance, kf_sv_size, kf_sv_size);
        #endif

    }

    void kf_project(const ELEM_T* mean, const ELEM_T* covariance)
    {
        #ifndef KF_NOPT_MM
        /*
        kf_projected_cov[0] = covariance[0];
        kf_projected_cov[5] = covariance[8];
        kf_projected_cov[10] = covariance[16];
        kf_projected_cov[15] = covariance[24];
        */
        index_t i, j;
        for(i = 0, j = 0; i < kf_mv_size * kf_mv_size; i += kf_mv_size + 1, j += kf_sv_size + 1){
            kf_projected_cov[i] = covariance[j];
        }
        #else
        GEMM(CblasRowMajor, CblasNoTrans, CblasNoTrans, kf_mv_size, kf_sv_size, kf_sv_size, 1, kf_update_mat, kf_sv_size,
        covariance, kf_sv_size, 0, kf_temp_mv_sv, kf_sv_size);
        GEMM(CblasRowMajor, CblasNoTrans, CblasTrans, kf_mv_size, kf_mv_size, kf_sv_size, 1, kf_temp_mv_sv, kf_sv_size,
        kf_update_mat, kf_sv_size, 0, kf_projected_cov, kf_mv_size);
        #endif

        
        #ifdef KF_DYNAMIC_COV
        ELEM_T h = mean[2] / SQRT_T(mean[2] * mean[3]);
        kf_projected_cov[0] += std_weight_position * h * std_weight_position * h;
        kf_projected_cov[5] +=  std_weight_position * h * std_weight_position * h;
        kf_projected_cov[10] +=  std_weight_position * h * std_weight_position * h;
        kf_projected_cov[15] += 1e-1 * 1e-1;
        #else
        kf_projected_cov[0] += 1e-1 * 1e-1;
        kf_projected_cov[5] += 1e-1 * 1e-1;
        kf_projected_cov[10] += 1e-1 * 1e-1;
        kf_projected_cov[15] += 1e-1 * 1e-1;
        #endif
        

        #ifdef KF_DEBUG
        print_mat("kf_project mean", mean, kf_mv_size, 1);
        print_mat("kf_project covariance", kf_projected_cov, kf_mv_size, kf_mv_size);
        #endif
    }


    void kf_update(const ELEM_T* measurement, ELEM_T* mean, ELEM_T* covariance)
    {
        //measurement 는 tlbr 로 입력되는 것을 상정
        //to_xysr(measurement, kf_temp_det);
        memcpy(kf_temp_det, measurement, sizeof(ELEM_T) * kf_mv_size);

        kf_project(mean, covariance);

        index_t i, j;
        for(i = 0; i < kf_mv_size; ++i){
            kf_temp_det[i] -= mean[i];
        }

        #ifndef KF_NOPT_MM
        /*
        kf_kalman_gain[0] = covariance[0] / kf_projected_cov[0];
        kf_kalman_gain[16] = covariance[28] / kf_projected_cov[0];
        kf_kalman_gain[5] = covariance[8] / kf_projected_cov[5];
        kf_kalman_gain[21] = covariance[36] / kf_projected_cov[5];
        kf_kalman_gain[10] =  covariance[16] / kf_projected_cov[10];
        kf_kalman_gain[26] = covariance[44] / kf_projected_cov[10];
        kf_kalman_gain[15] = covariance[24] / kf_projected_cov[15];
        */

        for(i = 0, j = 0; i < kf_mv_size * kf_mv_size; i += kf_mv_size + 1, j += kf_sv_size + 1){
            kf_kalman_gain[i] = covariance[j] / kf_projected_cov[i];
        }

        for(i = 0, j = kf_sv_size * kf_mv_size; i < kf_mv_size * kf_delta_num; i += kf_mv_size + 1, j += kf_sv_size + 1){
            kf_kalman_gain[i + kf_mv_size * kf_mv_size] = covariance[j] / kf_projected_cov[i];
        }

        /*
        mean[0] += kf_kalman_gain[0] * kf_temp_det[0];
        mean[1] += kf_kalman_gain[5] * kf_temp_det[1];
        mean[2] += kf_kalman_gain[10] * kf_temp_det[2];
        mean[3] += kf_kalman_gain[15] * kf_temp_det[3];
        mean[4] += kf_kalman_gain[16] * kf_temp_det[0];
        mean[5] += kf_kalman_gain[21] * kf_temp_det[1];
        mean[6] += kf_kalman_gain[26] * kf_temp_det[2];
        */

        for(i = 0, j = 0; i < kf_mv_size; ++i, j += kf_mv_size + 1){
            mean[i] += kf_kalman_gain[j] * kf_temp_det[i];
        }

        for(i = 0, j = kf_mv_size * kf_mv_size; i < kf_delta_num; ++i, j += kf_mv_size + 1){
            mean[i + kf_mv_size] += kf_kalman_gain[j] * kf_temp_det[i];
        }

        /*
        covariance[28] -= kf_kalman_gain[16] * covariance[0];
        covariance[32] -= kf_kalman_gain[16] * covariance[4];
        covariance[36] -= kf_kalman_gain[21] * covariance[8];
        covariance[40] -= kf_kalman_gain[21] * covariance[12];
        covariance[44] -= kf_kalman_gain[26] * covariance[16];
        covariance[48] -= kf_kalman_gain[26] * covariance[20];
        covariance[0] -= kf_kalman_gain[0] * covariance[0];
        covariance[4] -= kf_kalman_gain[0] * covariance[4];
        covariance[8] -= kf_kalman_gain[5] * covariance[8];
        covariance[12] -= kf_kalman_gain[5] * covariance[12];
        covariance[16] -= kf_kalman_gain[10] * covariance[16];
        covariance[20] -= kf_kalman_gain[10] * covariance[20];
        covariance[24] -= kf_kalman_gain[15] * covariance[24];
        */

        for(i = kf_mv_size * kf_sv_size, j = kf_mv_size * kf_mv_size; i < kf_sv_size * kf_sv_size; i += kf_sv_size + 1, j += kf_mv_size + 1){
            covariance[i] -= kf_kalman_gain[j] * covariance[i - kf_mv_size * kf_sv_size];
            covariance[i + kf_mv_size] -= kf_kalman_gain[j] * covariance[i - kf_mv_size * kf_sv_size + kf_mv_size];
        }

        for(i = 0, j = 0; i < kf_sv_size * kf_delta_num; i += kf_sv_size + 1, j += kf_mv_size + 1){
            covariance[i] -= kf_kalman_gain[j] * covariance[i];
            covariance[i + kf_mv_size] -= kf_kalman_gain[j] * covariance[i + kf_mv_size];
        }

        for(; i < kf_mv_size * kf_sv_size; i += kf_sv_size + 1, j += kf_mv_size + 1){
            covariance[i] -= kf_kalman_gain[j] * covariance[i]; 
        }


        #else
        GEMM(CblasRowMajor, CblasNoTrans, CblasTrans, kf_mv_size, kf_sv_size, kf_sv_size, 1, kf_update_mat, kf_sv_size,
        covariance, kf_sv_size, 0, kf_kalman_gain, kf_sv_size);
        POSV(LAPACK_ROW_MAJOR, 'L', kf_mv_size, kf_sv_size,kf_projected_cov, kf_mv_size,
        kf_kalman_gain, kf_sv_size);

        GEMV(CblasRowMajor, CblasTrans, kf_mv_size, kf_sv_size, 1, kf_kalman_gain, kf_sv_size, kf_temp_det, 1, 1, mean, 1);
        GEMM(CblasRowMajor, CblasNoTrans, CblasNoTrans, kf_mv_size, kf_sv_size, kf_sv_size, 1, kf_update_mat, kf_sv_size,
        covariance, kf_sv_size, 0, kf_temp_mv_sv, kf_sv_size);
        GEMM(CblasRowMajor, CblasTrans, CblasNoTrans, kf_sv_size, kf_sv_size, kf_mv_size, -1, kf_kalman_gain, kf_sv_size,
        kf_temp_mv_sv, kf_sv_size, 1, covariance, kf_sv_size);
        #endif


        #ifdef KF_DEBUG
        print_mat("kf_update mean", mean, kf_sv_size, 1);
        print_mat("kf_update covariance", covariance, kf_sv_size, kf_sv_size);
        #endif
        
    }
    void kf_destroy(void)
    {
        free(kf_motion_mat);
        free(kf_update_mat);
        free(kf_projected_cov);
        free(kf_kalman_gain);
        free(kf_temp_det);
        

        #ifdef KF_NOPT_MM
        free(kf_temp_sv_sv);
        free(kf_temp_mv_sv);
        #endif
    }


#ifdef __cplusplus
}
#endif