#include "memory_pool.h"
#include "errctl.h"
#include "link_node.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

void pool_init(pool *_p, index_t elem_size, index_t elem_num)
{
    _p->max_elem = elem_num;

    _p->src = malloc(elem_size * elem_num);
    if(_p->src == NULL)
        err_sys("memory pool src allocation fail");

    _p->stack = malloc(sizeof(index_t) * elem_num);
    if(_p->stack == NULL)
        err_sys("memory pool stack allocation fail");
    
    for(index_t i = 0; i < elem_num; i++){
        _p->stack[i] = i;
    }

    _p->top = elem_num;
}
void *pool_alloc(pool *_p)
{
    if(top != 0){
        return _p->src + _p->stack[(_p->top)--];
    }
    else{
        err_sys("memory pool no element in stack");
    }
}

void pool_free(pool *_p, void *ptr)
{
    if(_p->top != _p->max_elem){
        _p->stack[(_p->top)++] = ptr - _p->src;
    }
    else{
        err_sys("memory pool too much elements in stack");
    }
}

void pool_destroy(pool *_p)
{
    free(_p->src);
    free(_p->stack);
}

void pool_list_init(pool_list *_p, index_t elem_size, index_t elem_num)
{
    _p->src = malloc(elem_num * elem_size);
    if(_p->src == NULL){
        err_sys("pool list src allocation error");
    }

    _p->node->next = _p->src;
    _p->node->prev = _p->src;

    for(index_t i = elem_size; i < elem_num * elem_size; i+=elem_size){
        link_node(_p->node->prev, (char*)_p->src + i);
    }
}

void *pool_list_alloc(pool_list *_p)
{
    if(_p->node->next != _p->node){
        extract_node(_p->node);
        _p->node = _p->node->next;
    }
    else{
        err_sys("pool list alloc error");
    }
}

void pool_list_free(pool_list *_p, node *ptr)
{
    link_node(_p->node->prev, ptr);
}

void pool_list_destroy(pool_list *_p)
{
    free(_p->src);
}


#ifdef __cplusplus
}
#endif