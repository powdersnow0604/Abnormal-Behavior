#ifndef __OCCLUSION_H__
#define __OCCLUSION_H__


#include "typedefs.h"
#include "customer.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct __occlusion_point {

        float tlbr[4];
        __occlusion_point *next;
        //단순히 4 개의 track 만 저장할 수 있도록 구현 + max 값 넘어갔을 때 handling 없음 // 추후 수정 필요
        customer_t tracks[4];

    }occlusion_point;

    extern index_t occ_static_cnt;
    extern index_t occ_dynamic_cnt;
    extern occlusion_point *occ_array;

    void occ_init(void);
    void occ_destroy(void);


#ifdef __cplusplus
}
#endif

#endif