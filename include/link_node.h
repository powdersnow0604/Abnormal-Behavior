#ifndef __LINK_NODE_H__
#define __LINK_NODE_H__

#include "typedefs.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void link_node_oc(occlusion_point *head, customer_node *nnode);
    void link_node_ho(hm_node *head, occlusion_point *nnode);

#ifdef __cplusplus
}
#endif
#endif