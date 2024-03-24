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
    static const int32_t occ_grid_max_cap = 8; //**중요**// 한 grid 에 들어갈 수 있는 normal track 의 수, 수정 시 typedef 도 같이 수정해야 함
    const int32_t occ_hm_side_len = 8;  // 현재 grid 는 정사각형으로 생성
    static const int32_t occ_hm_len = occ_hm_side_len * occ_hm_side_len;
    static const ELEM_T occ_detect_iou_threshold = 0.7;
    static const ELEM_T occ_out_iou_threshold = 0.3;
    static const ELEM_T occ_create_docc_threshold = 50;

    static const index_t occ_static_cnt = 0;
    static index_t occ_dynamic_cnt;

    static pool occ_pool;
    static pool occ_track_pool;
    static hm_node *occ_hash_map;
    static ELEM_T occ_hm_denom_h;
    static ELEM_T occ_hm_denom_w;
    static customer_node **occ_accessor;

    static ELEM_T get_dist(ELEM_T *lhs, ELEM_T *rhs)
    {
        // menhatan distance
        return ABS_T(lhs[0] - rhs[0]) + ABS_T(lhs[1] - rhs[1]);
    }

    static occlusion_point *find_last_node(occlusion_point *node)
    {
        while (node->next)
        {
            node = node->next;
        }

        return node;
    }

    static void extract_node(occlusion_point *node, const detection_t *det)
    {
        index_t last_occ = tk_get_occ_track_num() - 1;
        occ_accessor[last_occ]->ind = node->tracks->ind;
        occ_accessor[node->tracks->ind] = occ_accessor[last_occ];

        tk_occ2track(node->tracks->ind, det);

        customer_node *dnode = node->tracks;
        node->tracks = node->tracks->next;
        pool_free(&occ_track_pool, dnode);
        // stack 구조에서는 last track 처리 필요 없음
    }

    static void occ_track2occ(index_t ind, customer_node *nnode)
    {
        index_t first_track = tk_get_occ_track_num();

        nnode->ind = first_track;
        occ_accessor[first_track] = nnode;

        tk_track2occ(ind);
    }

    static void init_occpoint(occlusion_point *occ_node, customer_node *track_node, ELEM_T *mean)
    {
        occ_node->tracks = track_node;
        occ_node->last_track = track_node;

        QELEM_T min_d_track = to_tlbr(mean);
        occ_node->x1 = min_d_track.e1;
        occ_node->y1 = min_d_track.e2;
        occ_node->x2 = min_d_track.e3;
        occ_node->x2 = min_d_track.e4;
    }

    void occ_init(void)
    {
        pool_init(&occ_pool, sizeof(occlusion_point), occ_max_cnt);
        pool_init(&occ_track_pool, sizeof(customer_node), tk_max_tracks);

        occ_dynamic_cnt = 0;

        occ_hash_map = (hm_node *)malloc(sizeof(hm_node) * occ_hm_len);
        if (occ_hash_map == NULL)
            err_sys("hash map allocation error");

        occ_init_hm();

        for (index_t i = 0; i < occ_hm_len; i++)
        {
            occ_hash_map[i].occ = NULL;
        }

        occ_accessor = (customer_node **)malloc(sizeof(customer_node **) * tk_max_tracks);
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

    void occ_init_hm(void)
    {
        hm_node *end = occ_hash_map + occ_hm_len - 1;
        for (; end >= occ_hash_map; end--)
        {
            end->track_num = 0;
        }
    }

    void occ_hm_mark(index_t ind, customer_t *track)
    {
        int32_t hm_ind_w = (int32_t)(track->statemean[0] / occ_hm_denom_w);
        int32_t hm_ind_h = (int32_t)(track->statemean[1] / occ_hm_denom_h);
        int32_t hm_ind = hm_ind_h * occ_hm_side_len + hm_ind_w;

        if (occ_hash_map[hm_ind].track_num < occ_grid_max_cap)
        {
            occ_hash_map[hm_ind].tracks[(occ_hash_map[hm_ind].track_num)++] = ind;
        }
    }

    void occ_detect(customer_t *tracks, index_t *unmatched, index_t um_num)
    {
        // track 이 있는 grid 만 고려, 주변의 grid 또한 고려하도록 만들 수 있음
        // tk_tracks 배열 기준 인덱스로 앞에서부터 반복해야 함
        index_t *end = unmatched + um_num - 1;
        for (; unmatched <= end; end--)
        {
            int32_t ind_w = (int32_t)(tracks[*unmatched].statemean[0] / occ_hm_denom_w);
            int32_t ind_h = (int32_t)(tracks[*unmatched].statemean[1] / occ_hm_denom_h);

            hm_node *hm = occ_hash_map + ind_h * occ_hm_side_len + ind_w;
            occlusion_point *iter = hm->occ;
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
                customer_node *nnode = (customer_node *)pool_alloc(&occ_track_pool);

                occ_track2occ(*unmatched, nnode);

                link_node_oc(max_occ_ptr, nnode);
            }
            else if (hm->track_num)
            {
                /*create dynamic occlusion*/
                index_t *start = hm->tracks, *end = hm->tracks + hm->track_num;
                ELEM_T min_dist = -1, dist;
                index_t min_ind;
                for (; start < end; start++)
                {
                    dist = get_dist(tracks[*unmatched].statemean, tracks[*start].statemean);
                    if (dist < min_dist)
                    {
                        min_ind = *start;
                        min_dist = dist;
                    }
                }

                if (min_dist > occ_create_docc_threshold)
                {
                    customer_node *track_nnode = (customer_node *)pool_alloc(&occ_track_pool);

                    occ_track2occ(*unmatched, track_nnode);

                    occlusion_point *occ_nnode = (occlusion_point *)pool_alloc(&occ_pool);

                    init_occpoint(occ_nnode, track_nnode, tracks[min_ind].statemean);

                    link_node_ho(hm, occ_nnode);
                }
                else
                {
                    // maybe detection error
                    // tk_delete_track_normal(tracks + *unmatched);
                    tk_mark_missed(tracks + *unmatched);
                }
            }
            else
            {
                // maybe detection error
                // tk_delete_track_normal(tracks + *unmatched);
                tk_mark_missed(tracks + *unmatched);
            }
        }
    }

    void occ_out(customer_t *tracks, const detection_t *dets, const index_t *um_dets, index_t num, uint8_t *bvector)
    {
        // track 이 있는 grid 만 고려, 주변의 grid 또한 고려하도록 만들 수 있음
        const index_t *end = um_dets + num;
        for (; um_dets < end; um_dets++)
        {
            const detection_t *det = dets + *um_dets;
            int32_t ind_w = (int32_t)((dets->x2 - dets->x1) / occ_hm_denom_w);
            int32_t ind_h = (int32_t)((dets->y2 - dets->y1) / occ_hm_denom_h);

            hm_node *hm = occ_hash_map + ind_h * occ_hm_side_len + ind_w;
            occlusion_point *iter = hm->occ;
            if (iter == NULL)
                continue;

            ELEM_T max_iou = -1, iou;
            occlusion_point *max_ptr = NULL;
            uint8_t is_first = 0;

            while (iter->next)
            {
                iou = get_iou_occ_det(det, iter->next);
                if (iou > max_iou)
                {
                    max_ptr = iter;
                    max_iou = iou;
                }

                iter = iter->next;
            }

            iou = get_iou_occ_det(det, hm->occ);
            if (iou > max_iou)
            {
                max_iou = iou;
                is_first = 1;
                max_ptr = hm->occ;
            }

            // occlusion -> track
            // 첫 track 을 out 시킴(stack 구조)
            // 추가적인 metric 을 도입하여 occlusion point 에 속한 track 중
            // detection 과 가장 잘 맞는 track 이 out 되도록 설계 가능
            if (max_iou > occ_out_iou_threshold && is_first)
            {
                extract_node(hm->occ, det);

                if (!hm->occ->tracks && hm->occ - (occlusion_point *)occ_pool.src >= occ_static_cnt)
                {
                    occlusion_point *dnode_occ = hm->occ;
                    hm->occ = hm->occ->next;
                    if (!hm->occ && hm->last_occ == dnode_occ)
                    {
                        hm->last_occ = find_last_node(hm->occ);
                    }

                    pool_free(&occ_pool, dnode_occ);
                }

                bv_clear(bvector, *um_dets);
            }
            else if (max_iou > occ_out_iou_threshold)
            {
                extract_node(max_ptr->next, det);

                if (!max_ptr->next->tracks && max_ptr->next - (occlusion_point *)occ_pool.src >= occ_static_cnt)
                {
                    occlusion_point *dnode_occ = max_ptr->next;
                    max_ptr->next = max_ptr->next->next;

                    if (hm->last_occ == dnode_occ)
                    {
                        hm->last_occ = find_last_node(hm->occ);
                    }

                    pool_free(&occ_pool, dnode_occ);
                }

                bv_clear(bvector, *um_dets);
            }
        }
    }

    hm_node *occ_get_hm(void)
    {
        return occ_hash_map;
    }

    index_t occ_get_hm_side_len(void)
    {
        return occ_hm_side_len;
    }

#ifdef __cplusplus
}
#endif