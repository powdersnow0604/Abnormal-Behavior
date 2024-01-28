#include "tracker.h"
#include "kalman_filter.h"
#include "iou.h"
#include "hungarian.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "errctl.h"

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


static customer_t *tk_tracks;
static ELEM_T *tk_sv_pointer;
static ELEM_T *tk_sc_pointer;
static index_t *tk_matching;
static ELEM_T *tk_cost_mat;
static index_t *tk_assignment;
static uint8_t *tk_unmatched_det_bool;
static index_t tk_track_cnt;

static void (*get_cost_mat)(ELEM_T* cost_mat, const customer_t* tracks, const detection_t* detections, index_t trk_num, index_t det_num) = get_cost_mat_iou;

static uint32_t track_id = 0;

static inline index_t min(index_t lhs, index_t rhs)
{
	return lhs < rhs ? lhs : rhs;
}

void tk_init(void)
{
    kf_init();
    ha_init();

    tk_track_cnt = 0;
    
    tk_tracks = (customer_t*)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(customer_t));
    if(tk_tracks == NULL) err_sys("alloc error tk_track");

    tk_matching = (index_t*)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(index_t));
    if(tk_matching == NULL) err_sys("alloc error tk_matching");

    tk_cost_mat = (ELEM_T*)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * tk_max_dets * sizeof(ELEM_T));
    if(tk_cost_mat == NULL) err_sys("alloc error tk_cost_mat");

    tk_assignment = (index_t*)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(index_t));
    if(tk_assignment == NULL) err_sys("alloc error tk_assignment");

    tk_unmatched_det_bool = (uint8_t*)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(uint8_t));
    if(tk_unmatched_det_bool == NULL) err_sys("alloc error tk_unmatched_det_bool");

    tk_tracks[0].statemean = tk_sv_pointer = (ELEM_T*)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * kf_sv_size * sizeof(ELEM_T));
    if(tk_sv_pointer == NULL) err_sys("alloc error statemean");

    tk_tracks[0].statecovariance = tk_sc_pointer = (ELEM_T*) ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * kf_cov_size * sizeof(ELEM_T));
    if(tk_sc_pointer == NULL) err_sys("alloc error statecovariance");

    index_t i;
    for(i = 1; i < tk_max_tracks; ++i){
        tk_tracks[i].statemean = tk_tracks[i-1].statemean + kf_sv_size;
        tk_tracks[i].statecovariance = tk_tracks[i-1].statecovariance + kf_cov_size;
    }

    for(i = tk_max_tracks-1; i >= 0; --i){
        tk_unmatched_det_bool[i] = 1;
    }
}

void tk_destroy(void)
{
    kf_destroy();
    ha_destroy();

    ALIGNED_FREE(tk_sv_pointer);
    ALIGNED_FREE(tk_sc_pointer);
    ALIGNED_FREE(tk_tracks);
    ALIGNED_FREE(tk_matching);
    ALIGNED_FREE(tk_cost_mat);
    ALIGNED_FREE(tk_assignment);
    ALIGNED_FREE(tk_unmatched_det_bool);
}

void tk_create_new_track(const detection_t* detections, index_t num)
{
    index_t i = 0, ind = tk_track_cnt & (tk_max_tracks - 1), cnt = 0;
    i += ind; i -=ind;
    //track 의 개수가 max 값을 넘기면, track 배열의 index 0 부터 덮어씌움 새로운 정책 필요
    for(i = 0; i < num; ++i){
        if(tk_unmatched_det_bool[i]){
            create_customer(tk_tracks + ind, detections + i, track_id++);
            ind = (ind + 1) & (tk_max_tracks-1);
            ++cnt;
        }
        else{
            tk_unmatched_det_bool[i] = 1;
        }
    }

    tk_track_cnt = min(tk_track_cnt + cnt, tk_max_tracks);
}

void tk_predict(void)
{
    index_t i;
    for(i = tk_track_cnt - 1; i >= 0; --i){
        kf_predict(tk_tracks[i].statemean, tk_tracks[i].statecovariance);
    }
}

void tk_update(const detection_t* detections, index_t num)
{
    get_cost_mat(tk_cost_mat, tk_tracks, detections, tk_track_cnt, num);
    (void)ha_solve(tk_cost_mat, tk_assignment, tk_track_cnt, num);

    index_t i, trk_cnt = tk_track_cnt;
    for(i = tk_track_cnt-1; i >= 0; --i){
        //unmatched tracks
        if(tk_assignment[i] == -1 || 1 - tk_cost_mat[i + trk_cnt * tk_assignment[i]] < iou_threshold){
            tk_mark_missed(i);
        }
        //matched tracks
        else{
            kf_update(detections + tk_assignment[i], tk_tracks[i].statemean, tk_tracks[i].statecovariance);
            tk_tracks[i].age = 0;
            tk_unmatched_det_bool[tk_assignment[i]] = 0;
        }
    }

    //unmatched detections
    tk_create_new_track(detections, num);
}

void tk_mark_missed(index_t missed)
{
    ++(tk_tracks[missed].age);

    if(tk_tracks[missed].age > tk_max_age){
        ELEM_T* temp_mean = tk_tracks[missed].statemean;
        ELEM_T* temp_cov = tk_tracks[missed].statecovariance;

        tk_tracks[missed] = tk_tracks[tk_track_cnt - 1];
        tk_tracks[tk_track_cnt - 1].statemean = temp_mean;
        tk_tracks[tk_track_cnt - 1].statecovariance = temp_cov;

        --tk_track_cnt;
    }
}


customer_t *tk_get_tracks(void)
{
    return tk_tracks;
}

index_t tk_get_track_num(void)
{
    return tk_track_cnt;
}


#ifdef __cplusplus
}
#endif