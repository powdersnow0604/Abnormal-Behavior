#ifndef __MEMORY_POOL_H__
#define __MEMORY_POOL_H__


#include "typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void *src;
    index_t max_elem;
    index_t elem_size;
    index_t top;
    index_t *stack;
}pool;

void pool_init(pool *_p, index_t elem_size, index_t elem_num);
void *pool_alloc(pool *_p);
void pool_free(pool *_p, void *ptr);
void pool_destroy(pool *_p);


#ifdef __cplusplus
}
#endif

#endif