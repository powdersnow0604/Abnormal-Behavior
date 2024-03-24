#include "memory_pool.h"
#include "errctl.h"
#include "link_node.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void pool_init(pool *_p, index_t elem_size, index_t elem_num)
    {
        _p->max_elem = elem_num;
        _p->elem_size = elem_size;

        _p->src = malloc((long)elem_size * elem_num);
        if (_p->src == NULL)
            err_sys("memory pool src allocation fail");

        _p->stack = (index_t *)malloc(sizeof(index_t) * elem_num);
        if (_p->stack == NULL)
            err_sys("memory pool stack allocation fail");

        for (index_t i = 0; i < elem_num; i++)
        {
            _p->stack[i] = i;
        }

        _p->top = elem_num;
    }

    void *pool_alloc(pool *_p)
    {
        if (_p->top != 0)
        {
            return _p->src + _p->stack[(_p->top)--];
        }
        else
        {
            err_sys("memory pool no element in stack");
        }
    }

    void pool_free(pool *_p, void *ptr)
    {
        if (_p->top != _p->max_elem)
        {
            _p->stack[(_p->top)++] = ((char *)ptr - (char *)_p->src) / _p->elem_size;
        }
        else
        {
            err_sys("memory pool too much elements in stack");
        }
    }

    void pool_destroy(pool *_p)
    {
        free(_p->src);
        free(_p->stack);
    }

    

#ifdef __cplusplus
}
#endif