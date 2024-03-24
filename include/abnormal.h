#ifndef __ABNORMAL_H__
#define __ABNORMAL_H__

#include "typedefs.h"


#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct {
        ELEM_T tlbr[4];
    }cluster;

    void abnormal_init(void);
    void abnormal_destroy(void);

    void is_stay(uint8_t *bool_vector);
    void is_fall(uint8_t *bool_vector);
    void is_cluster(cluster *clusters);
    

#ifdef __cplusplus
}
#endif


#endif