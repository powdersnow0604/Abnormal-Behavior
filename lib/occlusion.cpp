#include "occlusion.h"
#include <stdlib.h>
#include "errctl.h"
#include "memory_pool.h"
#include "iou.h"
#include "tracker.h"
#include "bool_vector.h"
#include "link_node.h"

#ifdef __cplusplus
extern "C"
{
#endif

    const index_t occ_max_cnt = 32;
    static const index_t occ_hm_side_len = 8;
    static const ELEM_T occ_detect_iou_threshold = 0.7;
    static const ELEM_T occ_out_iou_threshold = 0.3;

    static const index_t occ_static_cnt = 0;
    static index_t occ_dynamic_cnt;

    static pool occ_pool;
    static pool occ_track_pool;
    static hm_node *occ_hash_map;
    static ELEM_T occ_hm_denom_h;
    static ELEM_T occ_hm_denom_w;
    static customer_node **occ_accessor;

    void occ_init(void)
    {
        pool_init(&occ_pool, sizeof(occlusion_point), occ_max_cnt);
        pool_init(&occ_track_pool, sizeof(customer_node), tk_max_tracks);

        occ_dynamic_cnt = 0;

        occ_hash_map = malloc(sizeof(hm_node) * occ_hm_side_len * occ_hm_side_len);
        if (occ_hash_map == NULL)
            err_sys("hash map allocation error");

        occ_accessor = malloc(sizeof(customer_node**) * tk_max_tracks);
        if (occ_accessor == NULL)
            err_sys("occ accessor allocation error");

        occ_hm_denom_h = tk_img_height / occ_hm_side_len;
        occ_hm_denom_w = tk_img_width / occ_hm_side_len;

        /*initialize static occlusion*/
    }

    void occ_destroy(void)
    {
        pool_destroy(&occ_pool);
        pool_destroy(&occ_track_pool);
        free(occ_hash_map);
        free(occ_accessor);
    }

    void occ_detect(customer_t *tracks, index_t *unmatched, index_t um_num)
    {
        // tracks 배열 기준 인덱스로 앞에서부터 반복해야 함
        index_t *end = unmatched + um_num - 1;
        for (; unmatched <= end; end--)
        {
            int32_t ind_w = (int32_t)(tracks[*unmatched].statemean[0] / occ_hm_denom_w);
            int32_t ind_h = (int32_t)(tracks[*unmatched].statemean[1] / occ_hm_denom_h);

            occlusion_point *iter = occ_hash_map[ind_h * occ_hm_side_len + ind_w].occ;
            ELEM_T max_iou = -1, iou;
            occlusion_point *max_occ_ptr = NULL;
            while (iter)
            {
                iou = get_iou_occ_track(tracks[*unmatched].statemean, iter);
                if (iou > max_iou)
                {
                    max_occ_ptr = iter;
                    max_iou = iou;
                }

                iter = iter->next;
            }

            if (max_iou > occ_detect_iou_threshold)
            {
                // track -> occlusion
                customer_node *nnode = pool_alloc(&occ_track_pool);

                nnode->ind = *unmatched;
                occ_accessor[*unmatched] = nnode;

                tk_track2occ(*unmatched);

                link_node(max_occ_ptr->prev, nnode);
            }
            else
            {
                /*create dynamic occlusion*/
            }
        }
    }

    void occ_out(customer_t *tracks, detection_t *dets, uint8_t *um_dets, index_t num)
    {
        for (index_t i = 0; i < num; ++i)
        {
            if (bv_at(um_dets, i))
            {
                int32_t ind_w = (int32_t)((um_dets[i].x2 - um_dets[i].x1) / occ_hm_denom_w);
                int32_t ind_h = (int32_t)((um_dets[i].y2 - um_dets[i].y1) / occ_hm_denom_h);

                hm_node *hm = occ_hash_map + ind_h * occ_hm_side_len + ind_w;
                occlusion_point *iter = hm->occ;
                ELEM_T max_iou = -1, iou;
                occlusion_point *max_occ_ptr = NULL;
                while (iter)
                {
                    iou = get_iou_occ_det(dets + i, iter);
                    if (iou > max_iou)
                    {
                        max_occ_ptr = iter;
                        max_iou = iou;
                    }

                    iter = iter->next;
                }

                if(max_iou > occ_out_iou_threshold){

                    //occlusion -> track
                    //첫 track 을 out 시킴(stack 구조)
                    //추가적인 metric 을 도입하여 occlusion point 에 속한 track 중 
                    //detection 과 가장 잘 맞는 track 이 out 되도록 설계 가능
                    index_t last_occ = tk_get_occ_track_num() - 1;
                    occ_accessor[last_occ]->ind = max_occ_ptr->tracks->ind;
                    occ_accessor[max_occ_ptr->tracks->ind] =  occ_accessor[last_occ];
                    
                    extract_node(max_occ_ptr->tracks);
                    tk_occ2track(max_occ_ptr->tracks->ind);
                    pool_free(&occ_track_pool, max_occ_ptr->tracks);
                    max_occ_ptr->tracks = max_occ_ptr->tracks->next;

                    if(max_occ_ptr - occ_pool.src < occ_static_cnt && !max_occ_ptr->next){
                        extract_node(hm->occ);

                        pool_free(&occ_pool, hm->occ);
                        hm->occ = hm->occ->next;
                    }
                }
                else{
                    
                }
            }
        }
    }

#ifdef __cplusplus
}
#endif