#ifndef __OCCLUSION_H__
#define __OCCLUSION_H__


#include "typedefs.h"
#include "customer.h"

#ifdef __cplusplus
extern "C" {
#endif

    void occ_init(void);
    void occ_destroy(void);

    void occ_detect(customer_t *tracks, index_t *unmatched, index_t um_num);

    void occ_out(customer_t *tracks, const detection_t *dets, const index_t *um_dets, index_t num, uint8_t *bvector);

    void occ_init_hm(void);

    void occ_hm_mark(index_t ind, customer_t *track);

    hm_node *occ_get_hm(void);
    index_t occ_get_hm_side_len(void);


#ifdef __cplusplus
}
#endif

#endif