///////////////////////////////////////////////////////////////////////////////
// Hungarian.h: Header file for Class HungarianAlgorithm.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//

#include "typedefs.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void ha_init(void);
    void ha_destroy(void);
    ELEM_T ha_solve(ELEM_T *DistMatrix, index_t *Assignment, index_t trk_num, index_t det_num);

#ifdef __cplusplus
}
#endif
