#include "link_node.h"

#ifdef __cplusplus
extern "C" {
#endif

void link_node(node *base, node *nnode)
{
    nnode->next = base->next;
    base->next = nnode;
    nnode->prev = base;
}

void extract_node(node *node)
{
    node->prev->next = node->next;
    node->next->prev = node->prev;
}


#ifdef __cplusplus
}
#endif