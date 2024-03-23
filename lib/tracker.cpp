#include "tracker.h"
#include "kalman_filter.h"
#include "iou.h"
#include "hungarian.h"
#include "bool_vector.h"
#include "occlusion.h"
#include "errctl.h"
#include <string.h>

#ifdef __cplusplus
extern "C"
{
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
    static index_t tk_normal_track_cnt;
    static index_t tk_occ_track_cnt;

    static void (*get_cost_mat)(ELEM_T *cost_mat, const customer_t *tracks, const detection_t *detections, index_t trk_num, index_t det_num) = get_cost_mat_iou;

    static uint16_t track_id = 0;

    const index_t tk_max_tracks = 64;
    const index_t tk_max_dets = tk_max_tracks;
    const age_t tk_max_age = 1;
    const age_t tk_max_occ_age = 30;
    const ELEM_T tk_img_rng_factor = 0.1;
    const index_t tk_cmf_threshold = 4;

    static inline index_t min(int32_t lhs, int32_t rhs)
    {
        return lhs < rhs ? lhs : rhs;
    }

    static inline void swap2tracks(index_t lhs, index_t rhs)
    {
        customer_t temp = tk_tracks[lhs];
        tk_tracks[lhs] = tk_tracks[rhs];
        tk_tracks[rhs] = temp;
    }


    void tk_init(uint32_t img_width, uint32_t img_height)
    {
        kf_init();
        ha_init();
        occ_init();

        tk_track_cnt = 0;
        tk_normal_track_cnt = 0;
        tk_occ_track_cnt = 0;
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
        occ_destroy();

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

        // 일단 space 가 무한히 있다고 가정하고 code 를 작성함
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

        tk_track_cnt = min((int32_t)(tk_track_cnt) + cnt, tk_max_tracks);
        tk_normal_track_cnt += cnt;
    }

    void tk_predict(void)
    {
        customer_t *start = tk_tracks + tk_occ_track_cnt, *end = tk_tracks + tk_track_cnt - 1;
        for (; start <= end; end--)
        {
            kf_predict(end->statemean, end->statecovariance);
            if (end->statemean[0] < 0 || end->statemean[0] > tk_img_width - 1 || end->statemean[1] < 0 || end->statemean[1] > tk_img_height - 1)
            {
                tk_delete_track_normal((index_t)(end - tk_tracks));
            }

            if (end->tso != UINT8_MAX)
                (end->tso)++;
        }
    }

    void tk_update(const detection_t *detections, index_t num)
    {
        get_cost_mat(tk_cost_mat, tk_tracks + tk_occ_track_cnt, detections, tk_normal_track_cnt, num);
        (void)ha_solve(tk_cost_mat, tk_assignment, tk_normal_track_cnt, num);

        //i for assignment, cost mat, j for tracks
        index_t i, j, trk_cnt = tk_normal_track_cnt, um_trk_cnt = 0, um_det_cnt = 0, new_occ_cnt = 0;
        for (i = tk_normal_track_cnt - 1, j = tk_track_cnt - 1; i >= 0; --i, --j)
        {
            // unmatched tracks
            if (tk_assignment[i] == -1 || 1 - tk_cost_mat[i + trk_cnt * tk_assignment[i]] < iou_threshold)
            {
                if (tk_img_width * tk_img_rng_factor < tk_tracks[j].statemean[0] &&
                    tk_img_height * tk_img_rng_factor < tk_tracks[j].statemean[1] &&
                    tk_img_width * (1 - tk_img_rng_factor) > tk_tracks[j].statemean[0] &&
                    tk_img_height * (1 - tk_img_rng_factor) > tk_tracks[j].statemean[1] && tk_tracks[j].is_stable)
                {
                    tk_unmatched_tracks[um_trk_cnt++] = j;
                }
                else
                {
                    tk_mark_missed(j);
                }
            }
            // matched tracks
            else
            {
                kf_update(detections + tk_assignment[i], tk_tracks[i].statemean, tk_tracks[i].statecovariance);
                tk_tracks[i].age = 0;
                if (tk_tracks[i].cmf != 255 && ++(tk_tracks[i].cmf) == tk_cmf_threshold)
                    tk_tracks[i].is_stable = 1;

                bv_clear(tk_unmatched_det_bool, tk_assignment[i]);
            }
        }

        //occlusion 으로 편입 [혹은 추후에 예정된 detection error 판단]
        if (um_trk_cnt > 0)
        {
            new_occ_cnt = um_trk_cnt;
           
            occ_detect(tk_tracks, tk_unmatched_tracks, um_trk_cnt);
        }

        //occlusion 탈출 여부 판단
        if(tk_occ_track_cnt > 0)
        {
            for (i = 0; i < num; i++)
            {
                if (bv_at(tk_unmatched_det_bool, i))
                {
                    tk_unmatched_dets[um_det_cnt++] = i;
                }
            }
        }

        //tk_occ_track_cnt += new_occ_cnt;
        //tk_normal_track_cnt -= new_occ_cnt;

        // unmatched detections
        tk_create_new_track(detections, num);
    }

    void tk_mark_missed(index_t missed)
    {
        tk_tracks[missed].cmf = 0;
        ++(tk_tracks[missed].age);

        if (tk_tracks[missed].age > tk_max_age)
        {
            tk_delete_track_normal(missed);
        }
    }

    void tk_mark_occluded(index_t occ)
    {
        if (tk_tracks[occ].is_occluded)
        {
            ++(tk_tracks[occ].age);

            if (abs(tk_tracks[occ].statemean[4]) < 0.2 && abs(tk_tracks[occ].statemean[4]) < 0.25)
            {
                if (tk_tracks[occ].age > 40)
                {
                    tk_delete_track_occ(occ);
                }
                else
                {
                    tk_tracks[occ].statemean[6] /= 2;
                }
            }
            else
            {
                if (tk_tracks[occ].age > tk_max_occ_age)
                {
                    tk_delete_track_occ(occ);
                }
                else
                {
                    tk_tracks[occ].statemean[6] /= 2;
                }
            }
        }
        else
        {
            tk_tracks[occ].is_occluded = 1;
            tk_tracks[occ].age = 0;
            tk_tracks[occ].statemean[6] /= 1.5;
            tk_tracks[occ].statemean[5] /= 1.5;
            tk_tracks[occ].statemean[4] /= 1.5;
            tk_tracks[occ].cmf = 0;
            tk_tracks[occ].is_stable = 0;
        }
    }

    void tk_delete_track_normal(index_t ind)
    {
        ELEM_T *temp_mean = tk_tracks[ind].statemean;
        ELEM_T *temp_cov = tk_tracks[ind].statecovariance;

        tk_tracks[ind] = tk_tracks[tk_track_cnt - 1];
        tk_tracks[tk_track_cnt - 1].statemean = temp_mean;
        tk_tracks[tk_track_cnt - 1].statecovariance = temp_cov;

        --tk_track_cnt;
        --tk_normal_track_cnt;
    }

    void tk_delete_track_occ(index_t ind)
    {
        ELEM_T *temp_mean = tk_tracks[ind].statemean;
        ELEM_T *temp_cov = tk_tracks[ind].statecovariance;

        tk_tracks[ind] = tk_tracks[tk_occ_track_cnt - 1];
        tk_tracks[tk_occ_track_cnt - 1] = tk_tracks[tk_track_cnt - 1];
        tk_tracks[tk_track_cnt - 1].statemean = temp_mean;
        tk_tracks[tk_track_cnt - 1].statecovariance = temp_cov;

        --tk_track_cnt;
        --tk_occ_track_cnt;
    }
    
    void tk_track2occ(index_t ind)
    {
        swap2tracks(ind, tk_occ_track_cnt);

        tk_occ_track_cnt++;
        tk_normal_track_cnt--;
    }

    void tk_occ2track(index_t ind)
    {
        swap2tracks(ind, tk_occ_track_cnt - 1);

        tk_occ_track_cnt--;
        tk_normal_track_cnt++;
    }


    customer_t *tk_get_tracks(void)
    {
        return tk_tracks;
    }

    index_t tk_get_track_num(void)
    {
        return tk_track_cnt;
    }

    index_t tk_get_normal_track_num(void)
    {
        return tk_normal_track_cnt;
    }

    index_t tk_get_occ_track_num(void)
    {
        return tk_occ_track_cnt;
    }

#ifdef __cplusplus
}
#endif