#include "abnormal.h"
#include "tracker.h"
#include "bool_vector.h"
#include "memory_pool.h"
#include <stdlib.h>
#include "errctl.h"
#include <string.h>
#include "occlusion.h"
#include "iou.h"

#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct __Union
    {
        ELEM_T tlbr[4];
        __Union *next;
        __Union *prev;
        __Union *uptr;
    } Union;

    typedef struct
    {
        Union **uptr;
        void *origin;
        uint8_t flag;
    } cluster_node;

    static const ELEM_T stay_thresh_x = 1;
    static const ELEM_T stay_thresh_y = 1;
    static const uint8_t stay_thresh_cont = 30;
    static const ELEM_T fall_thresh = 1;
    static const uint8_t cluster_iteration = 4;
    static const ELEM_T cluster_iou_thresh = 0.8;
    static const uint32_t ra_size = (occ_hm_side_len - 1) * (occ_hm_side_len - 1);

    static Union *unions;
    static cluster_node *nodes;
    static index_t *rand_array;
    static index_t union_cnt;
    static Union *union_head;

    static inline ELEM_T max(ELEM_T lhs, ELEM_T rhs)
    {
        return lhs > rhs ? lhs : rhs;
    }

    static inline ELEM_T min(ELEM_T lhs, ELEM_T rhs)
    {
        return lhs < rhs ? lhs : rhs;
    }

    static void init_node(hm_node *hm, customer_t *tracks, index_t *num)
    {
        index_t *iter = hm->tracks, *end = iter + hm->track_num;
        for (; iter < end; iter++)
        {
            nodes[*num].uptr = NULL;
            nodes[*num].origin = tracks + *iter;
            nodes[*num].flag = 0;
            (*num)++;
        }

        occlusion_point *occ = hm->occ;
        while (occ)
        {
            if (occ->tracks)
            {
                nodes[*num].uptr = NULL;
                nodes[*num].origin = occ;
                nodes[*num].flag = 1;
                (*num)++;
            }
        }
    }

    static void init_nodes(hm_node *hm1, hm_node *hm2, hm_node *hm3, hm_node *hm4, customer_t *tracks, index_t *num)
    {
        init_node(hm1, tracks, num);
        init_node(hm2, tracks, num);
        init_node(hm3, tracks, num);
        init_node(hm4, tracks, num);
    }

    static index_t shuffle(index_t num)
    {
        for (int i = 0; i < num; i++)
        {
            index_t ran = rand() % (ra_size - 1 - i) + i;
            index_t temp = rand_array[i];
            rand_array[i] = rand_array[ran];
            rand_array[ran] = temp;
        }
    }

    static ELEM_T get_bbox_area(cluster_node *node, QELEM_T *tlbr)
    {
        if (node->flag)
        {
            occlusion_point *occ = (occlusion_point *)(node->origin);
            tlbr->e1 = occ->x1;
            tlbr->e2 = occ->y1;
            tlbr->e3 = occ->x2;
            tlbr->e4 = occ->y2;

            return (occ->x2 - occ->x1) * (occ->y2 - occ->y1);
        }
        else
        {
            ELEM_T *track = ((customer_t *)(node->origin))->statemean;
            *tlbr = to_tlbr(track);

            return track[2];
        }
    }

    static ELEM_T get_iou_nodes(cluster_node *lhs, cluster_node *rhs, QELEM_T *tlbr_l, QELEM_T *tlbr_r)
    {
        ELEM_T box_area_l = get_bbox_area(lhs, tlbr_l);
        ELEM_T box_area_r = get_bbox_area(rhs, tlbr_r);

        ELEM_T x1, x2, y1, y2, w, h, inter, iou;
        x1 = max(tlbr_l->e1, tlbr_r->e1);
        y1 = max(tlbr_l->e2, tlbr_r->e2);
        x2 = min(tlbr_l->e3, tlbr_r->e3);
        y2 = min(tlbr_l->e4, tlbr_r->e4);

        w = max(0, x2 - x1);
        h = max(0, y2 - y1);

        inter = w * h;
        iou = inter / (box_area_l + box_area_r - inter);

        return iou;
    }

    static void get_enclose_tlbr(ELEM_T *lhs, ELEM_T *rhs, ELEM_T *res)
    {
        // lhs 에 저장
        res[0] = min(lhs[0], rhs[0]);
        res[1] = min(lhs[1], rhs[1]);
        res[2] = max(lhs[2], rhs[2]);
        res[3] = max(lhs[3], rhs[3]);
    }

    static void extract_union(Union *eunion)
    {
        // 한 번 union 이 생기면, union_head 는 NULL 이 될 수 없음
        eunion->prev->next = eunion->next;
        eunion->next->prev = eunion->prev;

        if (union_head == eunion)
        {
            union_head = eunion->next;
        }
    }

    static void link_union(Union *nunion)
    {
        if (union_head)
        {
            nunion->next = union_head;
            nunion->prev = union_head->prev;
            union_head = nunion;
        }
        else
        {
            union_head = nunion;
            nunion->next = nunion;
            nunion->prev = nunion;
        }
    }

    static void make_union(cluster_node *lhs, cluster_node *rhs, QELEM_T *tlbr_l, QELEM_T *tlbr_r)
    {
        if (lhs->uptr && rhs->uptr)
        {
            // right 가 left 로 편입
            Union *U_l = *(lhs->uptr);
            Union *U_r = *(rhs->uptr);
            get_enclose_tlbr(U_l->tlbr, U_r->tlbr, U_l->tlbr);
            U_r->uptr = U_l->uptr;
            extract_union(U_r);
        }
        else if (!lhs->uptr && rhs->uptr)
        {
            Union *U = *(rhs->uptr);
            lhs->uptr = rhs->uptr;
            get_enclose_tlbr((ELEM_T *)tlbr_l, U->tlbr, U->tlbr);
        }
        else if (lhs->uptr && !rhs->uptr)
        {
            Union *U = *(lhs->uptr);
            rhs->uptr = lhs->uptr;
            get_enclose_tlbr(U->tlbr, (ELEM_T *)tlbr_r, U->tlbr);
        }
        else //! lhs->uptr && !rhs->uptr
        {
            Union *nunion = unions + union_cnt++;
            nunion->uptr = nunion;
            lhs->uptr = &nunion->uptr;
            rhs->uptr = &nunion->uptr;
            get_enclose_tlbr((ELEM_T *)tlbr_l, (ELEM_T *)tlbr_r, nunion->tlbr);
            link_union(nunion);
        }
    }

    static void union2cluster(Union *U, cluster *C)
    {
        C->tlbr[0] = U->tlbr[0];
        C->tlbr[1] = U->tlbr[1];
        C->tlbr[2] = U->tlbr[2];
        C->tlbr[3] = U->tlbr[3];
    }

    static void find_cluster(index_t num, cluster *clusters, index_t *cluster_cnt)
    {
        cluster_node *i = nodes, *end = nodes + num;
        for (; i < end; i++)
        {
            for (cluster_node *j = i + 1; j < end; j++)
            {
                QELEM_T tlbr_l, tlbr_r;
                ELEM_T iou = get_iou_nodes(i, j, &tlbr_l, &tlbr_r);
                if (iou > cluster_iou_thresh)
                {
                    make_union(i, j, &tlbr_l, &tlbr_r);
                }
            }
        }

        if (union_head == NULL)
            return;

        union2cluster(union_head, clusters + (*cluster_cnt)++);

        Union *iter = union_head->next;
        while (iter != union_head)
        {
            union2cluster(iter, clusters + (*cluster_cnt)++);

            iter = iter->next;
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void abnormal_init(void)
    {
        unions = (Union *)malloc(sizeof(Union) * ((tk_max_tracks >> 1) + 1));
        if (unions == NULL)
            err_sys("cluster unions allocation error");

        nodes = (cluster_node *)malloc(sizeof(cluster_node) * tk_max_tracks);
        if (nodes == NULL)
            err_sys("cluster node allocation error");

        rand_array = (index_t *)malloc(sizeof(index_t) * ra_size);
        if (rand_array == NULL)
            err_sys("cluster rand array allocation error");

        for (index_t i = 0; i < ra_size; i++)
        {
            rand_array[i] = i;
        }
    }

    void abnormal_destroy(void)
    {
        free(unions);
        free(nodes);
        free(rand_array);
    }

    void is_stay(uint8_t *bool_vector)
    {
        customer_t *tracks = tk_get_tracks();
        index_t track_cnt = tk_get_normal_track_num();
        index_t track_start = tk_get_occ_track_num();

        for (index_t i = track_start; i < track_start + track_cnt; i++)
        {
            if (tracks[i].statemean[5] < stay_thresh_x && tracks[i].statemean[6] < stay_thresh_y)
            {
                if (tracks[i].stay_cont < stay_thresh_cont)
                {
                    ++(tracks[i].stay_cont);
                }
                else
                {
                    bv_set(bool_vector, i);
                }
            }
            else
            {
                tracks[i].stay_cont = 0;
            }
        }
    }

    void is_fall(uint8_t *bool_vector)
    {
        customer_t *tracks = tk_get_tracks();
        index_t track_cnt = tk_get_normal_track_num();
        index_t track_start = tk_get_occ_track_num();

        for (index_t i = track_start; i < track_start + track_cnt; i++)
        {
            if (tracks[i].statemean[3] >= 1)
            {
                bv_set(bool_vector, i);
            }
        }
    }

    void is_cluster(cluster *clusters)
    {
        index_t cluster_cnt = 0;
        hm_node *hm = occ_get_hm();
        customer_t *tracks = tk_get_tracks();
        shuffle(cluster_iteration);

        for (int i = 0; i < cluster_iteration; i++)
        {
            index_t node_num = 0;
            union_cnt = 0;
            union_head = NULL;
            init_nodes(hm + rand_array[i], hm + rand_array[i] + 1, hm + rand_array[i] + occ_hm_side_len, hm + rand_array[i] + occ_hm_side_len + 1, tracks, &node_num);
            find_cluster(node_num, clusters, &cluster_cnt);
        }
    }

#ifdef __cplusplus
}
#endif