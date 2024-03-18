#include "occlusion.h"
#include <stdlib.h>
#include "errctl.h"

#ifdef __cplusplus
extern "C"
{
#endif

    const index_t occ_max_cnt = 32;
    index_t occ_static_cnt;
    index_t occ_dynamic_cnt;
    occlusion_point *occ_array;

    void occ_init(void)
    {
        occ_array = (occlusion_point *)malloc(occ_max_cnt * sizeof(occlusion_point));
        if (occ_array == NULL)
            err_sys("alloc error tk_track");

        occ_dynamic_cnt = 0;

        /*initialize static occlusion*/
        occ_static_cnt = 0;
    }

    void occ_destroy(void)
    {
        free(occ_array);
    }

#ifdef __cplusplus
}
#endif