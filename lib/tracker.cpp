#include "tracker.h"
#include "customer.h"
#include "kalman_filter.h"
#include "iou.h"
#include "hungarian.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

customer_t * tk_tracks;
ELEM_T* tk_sv_pointer;
ELEM_T* tk_sc_pointer;
index_t* tk_matching;
ELEM_T* tk_cost_mat;
index_t* tk_assignment;
index_t tk_track_cnt;
void (*get_cost_mat)(ELEM_T* cost_mat, customer_t* tracks, ELEM_T* detections, index_t trk_num, index_t det_num) = get_cost_mat_iou;
HungarianAlgorithm hungalgo;

uint32_t track_id = 0;

static inline index_t max(index_t ls, index_t rs)
{
	return ls > rs ? ls : rs;
}

void tk_init(void)
{
    tk_tracks = (customer_t*)malloc(tk_max_tracks * sizeof(customer_t));
    tk_matching = (index_t*)malloc(tk_max_tracks * sizeof(index_t));
    tk_cost_mat = (ELEM_T*)malloc(tk_max_tracks * tk_max_tracks * sizeof(ELEM_T));
    tk_assignment = (index_t*)malloc(tk_max_tracks * sizeof(index_t));
    tk_track_cnt = 0;

    tk_tracks[0].statemean = tk_sv_pointer = (ELEM_T*)malloc(tk_max_tracks * kf_sv_size * sizeof(ELEM_T));
    tk_tracks[0].statecovariance = tk_sc_pointer = (ELEM_T*) malloc(tk_max_tracks * kf_cov_size * sizeof(ELEM_T));

    index_t i;
    for(i = 1; i < tk_max_tracks; ++i){
        tk_tracks[i].statemean = tk_tracks[i-1].statemean + kf_sv_size;
        tk_tracks[i].statecovariance = tk_tracks[i-1].statecovariance + kf_cov_size;
    }
}

void tk_destroy(void)
{
    free(tk_sv_pointer);
    free(tk_sc_pointer);
    free(tk_tracks);
    free(tk_matching);
    free(tk_cost_mat);
    free(tk_assignment);
}

void tk_create_new_track(ELEM_T* detections, index_t num)
{
    index_t i;

    //track 의 개수가 max 값을 넘기면, track 배열의 index 0 부터 덮어씌움
    for(i = tk_track_cnt; i < tk_track_cnt + num; i = (i + 1) & tk_track_cnt){
        create_customer(tk_tracks + i, detections + i * kf_mv_size, track_id++);
    }

    tk_track_cnt = max(tk_track_cnt + num, tk_max_tracks);
}

void tk_predict(void)
{
    index_t i;
    for(i = tk_track_cnt - 1; i >= 0; --i){
        kf_predict(tk_tracks[i].statemean, tk_tracks[i].statecovariance);
    }
}

void tk_update(ELEM_T* detections, index_t num)
{
    get_cost_mat(tk_cost_mat, tk_tracks, detections, tk_track_cnt, num);
    hungalgo.Solve(tk_cost_mat, tk_assignment, tk_track_cnt, num, tk_max_tracks);
}

void tk_mark_missed(uint8_t* missed);


#ifdef __cplusplus
}
#endif