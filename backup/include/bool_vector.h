#ifndef __BOOL_VECTOR_H__
#define __BOOL_VECTOR_H__

#include <stdint.h>


#ifdef __cpluscplus
extern "C"{
#define KEYWORD_RESTRICT
#else
#define KEYWORD_RESTRICT __restrict__
#endif


uint8_t *bv_set(uint8_t * KEYWORD_RESTRICT vec, uint32_t ind);
uint8_t *bv_clear(uint8_t * KEYWORD_RESTRICT vec, uint32_t ind);
uint8_t *bv_assign(uint8_t * KEYWORD_RESTRICT vec, uint32_t ind, uint8_t val);
uint8_t bv_at(uint8_t * KEYWORD_RESTRICT vec, uint32_t ind);


#ifdef __cpluscplus
}
#endif

#endif