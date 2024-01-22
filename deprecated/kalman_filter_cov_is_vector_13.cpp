#include "kalman_filter.h"
#include <stdlib.h>
#include <string.h>
#include "iou.h"


#ifdef KF_DEBUG
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C"{
#endif
    
    static ELEM_T* kf_kalman_gain;
    static ELEM_T* kf_temp_det;


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

    static void print_cov(const char* des, const ELEM_T* cov, int8_t row, int8_t col)
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
            for(j = 0; j < col; j++){
                if(i == j) printf(fmt, cov[i]);
                else if(i < kf_delta_num && j == kf_mv_size + i) printf(fmt, cov[kf_sv_size + i]);
                else if(kf_mv_size <= i && j == i - kf_mv_size) printf(fmt, cov[kf_sv_size + kf_delta_num + j]);
                else printf(fmt, 0.);        
            }
            puts("");
        }

        puts("");
    }
    #endif

    void kf_init(void)
    {
        kf_kalman_gain = (ELEM_T*)calloc(kf_sv_size * kf_mv_size, sizeof(ELEM_T));
        kf_temp_det = (ELEM_T*)malloc(kf_mv_size * sizeof(ELEM_T));
    }

    void kf_initialize_track(const ELEM_T* measurement, ELEM_T* mean, ELEM_T* covariance)
    {
        //measurement 는 tlbr 로 입력되는 것을 상정
        index_t i;

        for(i = kf_mv_size; i < kf_sv_size; ++i){
            mean[i] = 0;
        }

        //to_xysr(measurement, mean);
        memcpy(mean, measurement, kf_mv_size * sizeof(ELEM_T));

        for(i = kf_sv_size; i < kf_sv_size + (kf_delta_num << 1); ++i){
            covariance[i] = 0;
        }

        #ifdef KF_DYNAMIC_COV
        //required square
        ELEM_T cov_p = measurement[3] - measurement[1] + 1;
        ELEM_T cov_v = 100 * std_weight_velocity * cov_p * std_weight_velocity * cov_p;
        cov_p = 4 * std_weight_position * cov_p * std_weight_position * cov_p;
        covariance[0] = cov_p;
        covariance[1] = cov_p;
        covariance[2] = cov_p;
        covariance[3] = 1e-4;
        covariance[4] = cov_v;
        covariance[5] = cov_v;
        covariance[6] = cov_v;
        #else
        covariance[0] = 1;
        covariance[1] = 1;
        covariance[2] = 1;
        covariance[3] = 1;
        covariance[4] = 1;
        covariance[5] = 1;
        covariance[6] = 1;
        #endif
        
        #ifdef KF_DEBUG
        print_mat("kf construct track mean", mean, kf_sv_size, 1);
        print_cov("kf construct track covariance", covariance, kf_sv_size, kf_sv_size);
        #endif
    }

    void kf_predict(ELEM_T* mean, ELEM_T* covariance)
    {
        //predict mean
        mean[0] += mean[4];
        mean[1] += mean[5];
        mean[2] += mean[6];

        //predict covariance
        index_t i;
       
        for(i = kf_sv_size; i < kf_sv_size + kf_delta_num; ++i){
            covariance[i] += covariance[i - kf_delta_num];
        }

        for(i = 0; i < kf_delta_num; ++i){
            covariance[i] += covariance[i + kf_sv_size] + covariance[i + kf_sv_size + kf_delta_num];
        }

        for(i = kf_sv_size + kf_delta_num; i < kf_sv_size + (kf_delta_num << 1); ++i){
            covariance[i] += covariance[i - (kf_delta_num << 1)];
        }

        
        #ifdef KF_DYNAMIC_COV
        ELEM_T h = mean[2] / SQRT_T(mean[2] * mean[3]);
        covariance[0] += std_weight_position * h * std_weight_position * h;
        covariance[1] += std_weight_position * h * std_weight_position * h;
        covariance[2] += std_weight_position * h * std_weight_position * h;
        covariance[3] += 1e-2 * 1e-2;
        covariance[4] += std_weight_velocity * h * std_weight_velocity * h;
        covariance[5] += std_weight_velocity * h * std_weight_velocity * h;
        covariance[6] += std_weight_velocity * h * std_weight_velocity * h;
        #else
        covariance[0] += 1e-4;
        covariance[1] += 1e-4;
        covariance[2] += 1e-4;
        covariance[3] += 1e-4;
        covariance[4] += 1e-4;
        covariance[5] += 1e-4;
        covariance[6] += 1e-4;
        #endif
        

        #ifdef KF_DEBUG
        print_mat("kf_predict mean", mean, kf_sv_size, 1);
        print_cov("kf_predict covariance", covariance, kf_sv_size, kf_sv_size);
        #endif

    }

    void kf_update(const ELEM_T* measurement, ELEM_T* mean, ELEM_T* covariance)
    {
        //measurement 는 tlbr 로 입력되는 것을 상정
        //to_xysr(measurement, kf_temp_det);
        memcpy(kf_temp_det, measurement, sizeof(ELEM_T) * kf_mv_size);

        index_t i;
        for(i = 0; i < kf_mv_size; ++i){
            kf_temp_det[i] -= mean[i];
        }

        #ifdef KF_DYNAMIC_COV
        ELEM_T d_cov = mean[2] / SQRT_T(mean[2] * mean[3]);
        d_cov = std_weight_position * d_cov * std_weight_position * d_cov;
        #endif

        /*
        kf_kalman_gain[0] = covariance[0] / kf_projected_cov[0];
        kf_kalman_gain[16] = covariance[28] / kf_projected_cov[0];
        kf_kalman_gain[5] = covariance[8] / kf_projected_cov[5];
        kf_kalman_gain[21] = covariance[36] / kf_projected_cov[5];
        kf_kalman_gain[10] =  covariance[16] / kf_projected_cov[10];
        kf_kalman_gain[26] = covariance[44] / kf_projected_cov[10];
        kf_kalman_gain[15] = covariance[24] / kf_projected_cov[15];
        */
        for(i = 0; i < kf_delta_num; ++i){
            #ifdef KF_DYNAMIC_COV
            kf_kalman_gain[i] = covariance[i] / (covariance[i] + d_cov); 
            #else
            kf_kalman_gain[i] = covariance[i] / (covariance[i] + 1e-2); 
            #endif
        }

        for(i = kf_delta_num; i < kf_mv_size; ++i){
            kf_kalman_gain[i] = covariance[i] / (covariance[i] + 1e-2); 
        }

        for(i = kf_mv_size; i < kf_sv_size; ++i){
            #ifdef KF_DYNAMIC_COV
            kf_kalman_gain[i] = covariance[i + (kf_delta_num << 1)] / (covariance[i - kf_mv_size] + d_cov);  
            #else
            kf_kalman_gain[i] = covariance[i + (kf_delta_num << 1)] / (covariance[i - kf_mv_size] + 1e-2); 
            #endif
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
        for(i = 0; i < kf_mv_size; ++i){
            mean[i] += kf_kalman_gain[i] * kf_temp_det[i];
        }

        for(i = kf_mv_size; i < kf_sv_size; ++i){
            mean[i] += kf_kalman_gain[i] * kf_temp_det[i - kf_mv_size];
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

        for(i = kf_sv_size + kf_delta_num; i < kf_sv_size + (kf_delta_num << 1); ++i){
            covariance[i] -= kf_kalman_gain[i - (kf_delta_num << 1)] * covariance[i - kf_sv_size - kf_delta_num];
            covariance[i - (kf_delta_num << 1)] -= kf_kalman_gain[i - (kf_delta_num << 1)] * covariance[i - kf_delta_num];
        }

        for(i = 0; i < kf_delta_num; ++i){
            covariance[i] -= kf_kalman_gain[i] * covariance[i];
            covariance[i + kf_sv_size] -= kf_kalman_gain[i] * covariance[i + kf_sv_size];
        }

        for(i = kf_delta_num; i < kf_mv_size; ++i){
            covariance[i] -= kf_kalman_gain[i] * covariance[i];
        }


        #ifdef KF_DEBUG
        print_mat("kf_update mean", mean, kf_sv_size, 1);
        print_cov("kf_update covariance", covariance, kf_sv_size, kf_sv_size);
        #endif
        
    }
    void kf_destroy(void)
    {
        free(kf_kalman_gain);
        free(kf_temp_det);
    }


#ifdef __cplusplus
}
#endif