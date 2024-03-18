#include "tracker.h"
#include "kalman_filter.h"
#include "iou.h"
#include "hungarian.h"
#include <string.h>
#include <bool_vector.h>

#ifdef __cplusplus
extern "C"
{
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
    static index_t *tk_unmatched_tracks;
    static index_t *tk_unmatched_dets;
    static uint32_t tk_img_width;
    static uint32_t tk_img_height;
    static ELEM_T tk_conf_obj_thresh = 0.75;
    static ELEM_T tk_conf_trk_thresh = 0.35;
    static ELEM_T tk_cp_thresh = 0.3;

    static void (*get_cost_mat)(ELEM_T *cost_mat, const customer_t *tracks, const detection_t *detections, index_t trk_num, index_t det_num) = get_cost_mat_iou;

    static uint32_t track_id = 0;

    const index_t tk_max_tracks = 64;
    const index_t tk_max_dets = tk_max_tracks;
    const age_t tk_max_age = 1;
    const age_t tk_max_occ_age = 3;

    static inline index_t min(index_t lhs, index_t rhs)
    {
        return lhs < rhs ? lhs : rhs;
    }

    static inline ELEM_T get_avg_area()
    {
        customer_t* iter = tk_tracks, *end = tk_tracks + tk_track_cnt;
        ELEM_T avg_area = 0;
        for(;iter < end; iter++){
            avg_area += iter->statemean[2];
        }

        return avg_area / tk_track_cnt;
    }

    void tk_init(uint32_t img_width, uint32_t img_height)
    {
        kf_init();
        ha_init();

        tk_track_cnt = 0;
        tk_img_width = img_width;
        tk_img_height = img_height;

        tk_tracks = (customer_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(customer_t));
        if (tk_tracks == NULL)
            err_sys("alloc error tk_track");

        tk_matching = (index_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(index_t));
        if (tk_matching == NULL)
            err_sys("alloc error tk_matching");

        tk_cost_mat = (ELEM_T *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * tk_max_dets * sizeof(ELEM_T));
        if (tk_cost_mat == NULL)
            err_sys("alloc error tk_cost_mat");

        tk_assignment = (index_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(index_t));
        if (tk_assignment == NULL)
            err_sys("alloc error tk_assignment");

        tk_unmatched_det_bool = (uint8_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(uint8_t) >> 3);
        if (tk_unmatched_det_bool == NULL)
            err_sys("alloc error tk_unmatched_det_bool");

        tk_unmatched_tracks = (index_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(index_t) >> 3);
        if (tk_unmatched_tracks == NULL)
            err_sys("alloc error tk_unmatched_tracks");

        tk_unmatched_dets = (index_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_dets * sizeof(index_t) >> 3);
        if (tk_unmatched_dets == NULL)
            err_sys("alloc error tk_unmatched_dets");

        tk_tracks[0].statemean = tk_sv_pointer = (ELEM_T *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * kf_sv_size * sizeof(ELEM_T));
        if (tk_sv_pointer == NULL)
            err_sys("alloc error statemean");

        tk_tracks[0].statecovariance = tk_sc_pointer = (ELEM_T *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * kf_cov_size * sizeof(ELEM_T));
        if (tk_sc_pointer == NULL)
            err_sys("alloc error statecovariance");

        index_t i;
        for (i = 1; i < tk_max_tracks; ++i)
        {
            tk_tracks[i].statemean = tk_tracks[i - 1].statemean + kf_sv_size;
            tk_tracks[i].statecovariance = tk_tracks[i - 1].statecovariance + kf_cov_size;
        }

        memset(tk_unmatched_det_bool, 0xff, tk_max_tracks * sizeof(uint8_t) >> 3);
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
        ALIGNED_FREE(tk_unmatched_tracks);
        ALIGNED_FREE(tk_unmatched_dets);
    }

    void tk_create_new_track(const detection_t *detections, index_t num)
    {
        index_t i = 0, ind = tk_track_cnt & (tk_max_tracks - 1), cnt = 0;

        // track 의 개수가 max 값을 넘기면, track 배열의 index 0 부터 덮어씌움 새로운 정책 필요
        for (i = 0; i < num; ++i)
        {
            if (bv_at(tk_unmatched_det_bool, i))
            {
                create_customer(tk_tracks + ind, detections + i, track_id++);
                ind = (ind + 1) & (tk_max_tracks - 1);
                ++cnt;
            }
            else
            {
                bv_set(tk_unmatched_det_bool, i);
            }
        }

        tk_track_cnt = min(tk_track_cnt + cnt, tk_max_tracks);
    }

    void tk_predict(void)
    {
        customer_t *iter = tk_tracks, *end = tk_tracks + tk_track_cnt;
        for (; iter < end; iter++)
        {
            kf_predict(iter->statemean, iter->statecovariance);
            if (iter->statemean[0] < 0 || iter->statemean[0] > tk_img_width - 1 || iter->statemean[1] < 0 || iter->statemean[1] > tk_img_height - 1)
            {
                tk_delete_track((index_t)(iter - tk_tracks));
            }

            if (iter->tso != UINT8_MAX)
                (iter->tso)++;
        }
    }

    void tk_update(const detection_t *detections, index_t num)
    {
        get_cost_mat(tk_cost_mat, tk_tracks, detections, tk_track_cnt, num);
        (void)ha_solve(tk_cost_mat, tk_assignment, tk_track_cnt, num);

        index_t i, trk_cnt = tk_track_cnt, um_trk_cnt = 0, um_det_cnt = 0;
        for (i = tk_track_cnt - 1; i >= 0; --i)
        {
            // unmatched tracks
            if (tk_assignment[i] == -1 || 1 - tk_cost_mat[i + trk_cnt * tk_assignment[i]] < iou_threshold)
            {
                tk_unmatched_tracks[um_trk_cnt++] = i;
            }
            // matched tracks
            else
            {
                kf_update(detections + tk_assignment[i], tk_tracks[i].statemean, tk_tracks[i].statecovariance);
                tk_tracks[i].age = 0;
                bv_clear(tk_unmatched_det_bool, tk_assignment[i]);
            }
        }

        if (um_trk_cnt > 0)
        {
            for (i = 0; i < num; i++)
            {
                if (bv_at(tk_unmatched_det_bool, i))
                {
                    tk_unmatched_dets[um_det_cnt++] = i;
                }
            }

            if (um_det_cnt > 0)
            {
                get_cost_mat_ext_iou(tk_cost_mat, tk_tracks, detections, tk_unmatched_tracks, tk_unmatched_dets, um_trk_cnt, um_det_cnt);
                (void)ha_solve(tk_cost_mat, tk_assignment, um_trk_cnt, um_det_cnt);

                ELEM_T avg_area = get_avg_area();
                trk_cnt = um_trk_cnt;
                for (i = 0; i < um_trk_cnt; i++)
                {
                    // unmatched tracks
                    if (tk_assignment[i] == -1 || 1 - tk_cost_mat[i + trk_cnt * tk_assignment[i]] < iou_threshold)
                    {
                        ELEM_T conf = get_confidence(tk_tracks + tk_unmatched_tracks[i], avg_area, 10);
                        if (get_cp(tk_tracks + tk_unmatched_tracks[i], tk_tracks, tk_track_cnt, tk_cp_thresh) > tk_cp_thresh && conf > tk_conf_trk_thresh)
                        {
                            tk_mark_occluded(tk_unmatched_tracks[i]);
                        }
                        else if (conf > tk_conf_obj_thresh)
                        {
                            tk_mark_occluded(tk_unmatched_tracks[i]);
                        }
                        else
                        {
                            tk_mark_missed(tk_unmatched_tracks[i]);
                        }
                    }
                    // matched tracks
                    else
                    {
                        kf_update(detections + tk_unmatched_dets[tk_assignment[i]], tk_tracks[tk_unmatched_tracks[i]].statemean, tk_tracks[tk_unmatched_tracks[i]].statecovariance);
                        tk_tracks[tk_unmatched_tracks[i]].age = 0;
                        bv_clear(tk_unmatched_det_bool, tk_unmatched_dets[tk_assignment[i]]);
                    }
                }
            }
            else
            {
                index_t *iter = tk_unmatched_tracks, *end = tk_unmatched_tracks + um_trk_cnt;
                ELEM_T avg_area = get_avg_area();
                for (; iter < end; iter++)
                {
                    // tk_mark_missed(*iter);

                    ELEM_T conf = get_confidence(tk_tracks + *iter, avg_area, 10);
                    if (get_cp(tk_tracks + *iter, tk_tracks, tk_track_cnt, tk_cp_thresh) > tk_cp_thresh && conf > tk_conf_trk_thresh)
                    {
                        tk_mark_occluded(*iter);
                    }
                    else if (conf > tk_conf_obj_thresh)
                    {
                        tk_mark_occluded(*iter);
                    }
                    else
                    {
                        tk_mark_missed(*iter);
                    }
                }
            }
        }

        // unmatched detections
        tk_create_new_track(detections, num);
    }

    void tk_mark_missed(index_t missed)
    {
        ++(tk_tracks[missed].age);

        if (tk_tracks[missed].age > tk_max_age)
        {
            tk_delete_track(missed);
        }
    }

    void tk_mark_occluded(index_t occ)
    {
        if (tk_tracks[occ].age & 0x80)
        {
            ++(tk_tracks[occ].age);

            if ((tk_tracks[occ].age & 0x7f) > tk_max_occ_age)
            {
                tk_delete_track(occ);
            }
            else
            {
                tk_tracks[occ].statemean[4] = 0;
                tk_tracks[occ].statemean[5] = 0;
                tk_tracks[occ].statemean[6] /= 2;
            }
        }
        else
        {
            tk_tracks[occ].age = 0x80;
            tk_tracks[occ].statemean[4] = 0;
            tk_tracks[occ].statemean[5] = 0;
            tk_tracks[occ].statemean[6] /= 2;
        }
    }

    void tk_delete_track(index_t ind)
    {
        ELEM_T *temp_mean = tk_tracks[ind].statemean;
        ELEM_T *temp_cov = tk_tracks[ind].statecovariance;

        tk_tracks[ind] = tk_tracks[tk_track_cnt - 1];
        tk_tracks[tk_track_cnt - 1].statemean = temp_mean;
        tk_tracks[tk_track_cnt - 1].statecovariance = temp_cov;

        --tk_track_cnt;
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