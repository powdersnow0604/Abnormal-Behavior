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

    void occ_out(customer_t *tracks, detection_t *dets, uint8_t *um_dets, index_t num);


#ifdef __cplusplus
}
#endif

#endif