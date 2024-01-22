#include <iou.h>
#include "kalman_filter.h"
#include <stdio.h>


ELEM_T mean[7] = {0,};
ELEM_T cov[4] = {0,};
ELEM_T det[4] = {0,1,2,3};


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
                if(i == j && i < 3) printf(fmt, cov[0]);
                else if(i == j && i == 3) printf(fmt, cov[1]);
                else if(i == j && i > 3) printf(fmt, cov[2]);
                else if(i < kf_delta_num && j == kf_mv_size + i) printf(fmt, cov[3]);
                else if(kf_mv_size <= i && j == i - kf_mv_size) printf(fmt, cov[3]);
                else printf(fmt, 0.);        
            }
            puts("");
        }

        puts("");
    }


int main()
{

    kf_init();
    kf_initialize_track(det, mean, cov);

    for(index_t i = 0; i < 100; i++){
        kf_predict(mean, cov);

        det[0] += 1;
        det[1] += 1;
        det[2] += 1;
        det[3] += 1;

        kf_update(det, mean, cov);
    }

    kf_destroy();

    print_mat("mean", mean, 7, 1);
    print_cov("cov", cov, 7, 7);

    return 0;
}