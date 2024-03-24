#include "link_node.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void link_node_oc(occlusion_point *head, customer_node *nnode)
    {
        if (head->tracks)
        {
            nnode->next = NULL;
            head->last_track->next = nnode;
        }
        else
        {
            head->tracks = nnode;
        }
        head->last_track = nnode;
    }

     void link_node_ho(hm_node *head, occlusion_point *nnode)
    {
        if (head->occ)
        {
            nnode->next = NULL;
            head->last_occ->next = nnode;
        }
        else
        {
            head->occ = nnode;
        }
        head->last_occ = nnode;
    }

#ifdef __cplusplus
}
#endif