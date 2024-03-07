#include "bool_vector.h"

#ifdef __cpluscplus
extern "C"
{
#define KEYWORD_RESTRICT
#else
#define KEYWORD_RESTRICT __restrict__
#endif

    uint8_t *bv_set(uint8_t *KEYWORD_RESTRICT vec, uint32_t ind)
    {
        vec[ind >> 3] |= 1 << (ind & 7);

        return vec;
    }

    uint8_t *bv_clear(uint8_t *KEYWORD_RESTRICT vec, uint32_t ind)
    {
        vec[ind >> 3] &= ~(1 << (ind & 7));

        return vec;
    }

    uint8_t *bv_assign(uint8_t * KEYWORD_RESTRICT vec, uint32_t ind, uint8_t val)
    {
        if(val){
            vec[ind >> 3] |= 1 << (ind & 7);
        }
        else{
            vec[ind >> 3] &= ~(1 << (ind & 7));
        }

        return vec;
    }


    uint8_t bv_at(uint8_t *KEYWORD_RESTRICT vec, uint32_t ind)
    {
        return (vec[ind >> 3] & (0x1 << (ind & 0x7))) != 0x00;
    }

#ifdef __cpluscplus
}
#endif