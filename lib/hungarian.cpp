///////////////////////////////////////////////////////////////////////////////
// Hungarian.cpp: Implementation file for Class HungarianAlgorithm.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//

#include "hungarian.h"
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <bool_vector.h>


#ifdef __cplusplus
extern "C"
{
#endif

#include "errctl.h"

#ifdef _MSC_VER
#define ALIGNED_ALLOC(align, size) _aligned_malloc((size), (align))
#else
#define ALIGNED_ALLOC(align, size) aligned_alloc((align), (size))
#endif

#ifdef _MSC_VER
#define ALIGNED_FREE(ptr) _aligned_free((ptr))
#else
#define ALIGNED_FREE(ptr) free((ptr))
#endif


    static void assignmentoptimal(index_t *assignment, ELEM_T *cost, ELEM_T *distMatrix, index_t nOfRows, index_t nOfColumns);
    static void buildassignmentvector(index_t *assignment, uint8_t *starMatrix, index_t nOfRows, index_t nOfColumns);
    //static void computeassignmentcost(index_t *assignment, ELEM_T *cost, ELEM_T *distMatrix, index_t nOfRows);
    static void step2a(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim);
    static void step2b(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim);
    static void step3(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim);
    static void step4(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim, index_t row, index_t col);
    static void step5(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim);

    static uint8_t *ha_coveredColumns;
    static uint8_t *ha_coveredRows;
    static uint8_t *ha_starMatrix;
    static uint8_t *ha_primeMatrix;
    static uint8_t *ha_newStarMatrix;
    static ELEM_T *ha_distMatrixIn;

    void ha_init(void)
    {
        ha_coveredColumns = (uint8_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_dets * sizeof(uint8_t) >> 3);
        if (ha_coveredColumns == NULL)
            err_sys("alloc error ha_coveredColumns");

        ha_coveredRows = (uint8_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * sizeof(uint8_t) >> 3);
        if (ha_coveredRows == NULL)
            err_sys("alloc error ha_coveredRows");

        ha_starMatrix = (uint8_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * tk_max_dets * sizeof(uint8_t) >> 3);
        if (ha_starMatrix == NULL)
            err_sys("alloc error ha_starMatrix");

        ha_primeMatrix = (uint8_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * tk_max_dets * sizeof(uint8_t) >> 3);
        if (ha_primeMatrix == NULL)
            err_sys("alloc error ha_primeMatrix");

        ha_newStarMatrix = (uint8_t *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * tk_max_dets * sizeof(uint8_t) >> 3);
        if (ha_newStarMatrix == NULL)
            err_sys("alloc error ha_newStarMatrix");

        ha_distMatrixIn = (ELEM_T *)ALIGNED_ALLOC(ALIGNMENT, tk_max_tracks * tk_max_dets * sizeof(ELEM_T));
        if (ha_distMatrixIn == NULL)
            err_sys("alloc error ha_distMatrixIn");
    }

    void ha_destroy(void)
    {
        ALIGNED_FREE(ha_coveredColumns);
        ALIGNED_FREE(ha_coveredRows);
        ALIGNED_FREE(ha_starMatrix);
        ALIGNED_FREE(ha_primeMatrix);
        ALIGNED_FREE(ha_newStarMatrix);
        ALIGNED_FREE(ha_distMatrixIn);
    }

    //********************************************************//
    // A single function wrapper for solving assignment problem.
    //********************************************************//
    ELEM_T ha_solve(ELEM_T *DistMatrix, index_t *Assignment, index_t trk_num, index_t det_num)
    {
        ELEM_T cost = 0.0;

        // Fill in the distMatrixIn. Mind the index is "i + nRows * j".
        // Here the cost matrix of size MxN is defined as a double precision array of N*M elements.
        // In the solving functions matrices are seen to be saved MATLAB-internally in row-order.
        // (i.e. the matrix [1 2; 3 4] will be stored as a vector [1 3 2 4], NOT [1 2 3 4]).
        memcpy(ha_distMatrixIn, DistMatrix, trk_num * det_num * sizeof(ELEM_T));

        // call solving function
        assignmentoptimal(Assignment, &cost, ha_distMatrixIn, trk_num, det_num);

        return cost;
    }

    //********************************************************//
    // Solve optimal solution for assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
    //********************************************************//
    void assignmentoptimal(index_t *assignment, ELEM_T *cost, ELEM_T *distMatrix, index_t nOfRows, index_t nOfColumns)
    {
        ELEM_T *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;
        int32_t nOfElements;
        index_t minDim, row, col;

        /* initialization */
        *cost = 0;
        for (row = 0; row < nOfRows; row++)
            assignment[row] = -1;

        /* generate working copy of distance Matrix */
        /* check if all matrix elements are positive */
        nOfElements = nOfRows * nOfColumns;
        distMatrixEnd = distMatrix + nOfElements;

        #ifdef HA_DEBUG
        for (row = 0; row < nOfElements; row++)
        {
            if (distMatrix[row] < 0)
            {
                puts("All matrix elements have to be non-negative.");
                for (index_t i = 0; i < nOfRows; i++)
                {
                    for (index_t j = 0; j < nOfColumns; j++)
                    {
                        printf("%.2lf ", distMatrix[i + j * nOfRows]);
                    }
                    puts("");
                }
                sleep(10);
                break;
            }
        }
        #endif

        /* memory allocation */
        memset(ha_coveredColumns, 0, (nOfColumns + (-nOfColumns & 0x07)) * sizeof(uint8_t) >> 3);
        memset(ha_coveredRows, 0, (nOfRows + (-nOfRows & 0x07)) * sizeof(uint8_t) >> 3);
        memset(ha_starMatrix, 0, (nOfElements + (-nOfElements & 0x07)) * sizeof(uint8_t) >> 3);
        memset(ha_primeMatrix, 0, (nOfElements + (-nOfElements & 0x07)) * sizeof(uint8_t) >> 3);
        memset(ha_newStarMatrix, 0, (nOfElements + (-nOfElements & 0x07)) * sizeof(uint8_t) >> 3);

        /* preliminary steps */
        if (nOfRows <= nOfColumns)
        {
            minDim = nOfRows;

            for (row = 0; row < nOfRows; row++)
            {
                /* find the smallest element in the row */
                distMatrixTemp = distMatrix + row;
                minValue = *distMatrixTemp;
                distMatrixTemp += nOfRows;
                while (distMatrixTemp < distMatrixEnd)
                {
                    value = *distMatrixTemp;
                    if (value < minValue)
                        minValue = value;
                    distMatrixTemp += nOfRows;
                }

                /* subtract the smallest element from each element of the row */
                distMatrixTemp = distMatrix + row;
                while (distMatrixTemp < distMatrixEnd)
                {
                    *distMatrixTemp -= minValue;
                    distMatrixTemp += nOfRows;
                }
            }

            /* Steps 1 and 2a */
            for (row = 0; row < nOfRows; row++)
                for (col = 0; col < nOfColumns; col++)
                    if (fabs(distMatrix[row + nOfRows * col]) < ELEM_EPSILON)
                        if (!bv_at(ha_coveredColumns, col))
                        {
                            bv_set(ha_starMatrix, row + nOfRows * col);
                            bv_set(ha_coveredColumns, col);
                            break;
                        }
        }
        else /* if(nOfRows > nOfColumns) */
        {
            minDim = nOfColumns;

            for (col = 0; col < nOfColumns; col++)
            {
                /* find the smallest element in the column */
                distMatrixTemp = distMatrix + nOfRows * col;
                columnEnd = distMatrixTemp + nOfRows;

                minValue = *distMatrixTemp++;
                while (distMatrixTemp < columnEnd)
                {
                    value = *distMatrixTemp++;
                    if (value < minValue)
                        minValue = value;
                }

                /* subtract the smallest element from each element of the column */
                distMatrixTemp = distMatrix + nOfRows * col;
                while (distMatrixTemp < columnEnd)
                    *distMatrixTemp++ -= minValue;
            }

            /* Steps 1 and 2a */
            for (col = 0; col < nOfColumns; col++)
                for (row = 0; row < nOfRows; row++)
                    if (fabs(distMatrix[row + nOfRows * col]) < ELEM_EPSILON)
                        if (!bv_at(ha_coveredRows, row)) 
                        {
                            bv_set(ha_starMatrix, row + nOfRows * col);
                            bv_set(ha_coveredColumns, col);
                            bv_set(ha_coveredRows, row);
                            break;
                        }
            for (row = 0; row < nOfRows; row++)
                bv_clear(ha_coveredRows, row);
        }

        /* move to step 2b */
        step2b(assignment, distMatrix, ha_starMatrix, ha_newStarMatrix, ha_primeMatrix, ha_coveredColumns, ha_coveredRows, nOfRows, nOfColumns, minDim);

        /* compute cost and remove invalid assignments */
        // computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);


        return;
    }

    /********************************************************/
    void buildassignmentvector(index_t *assignment, uint8_t *starMatrix, index_t nOfRows, index_t nOfColumns)
    {
        index_t row, col;

        for (row = 0; row < nOfRows; row++)
            for (col = 0; col < nOfColumns; col++)
                if (bv_at(starMatrix, row + nOfRows * col))
                {
#ifdef ONE_INDEXING
                    assignment[row] = col + 1; /* MATLAB-Indexing */
#else
                assignment[row] = col;
#endif
                    break;
                }
    }

    /********************************************************/
    /*
    void computeassignmentcost(index_t *assignment, ELEM_T *cost, ELEM_T *distMatrix, index_t nOfRows)
    {
        index_t row, col;

        for (row = 0; row < nOfRows; row++)
        {
            col = assignment[row];
            if (col >= 0)
                *cost += distMatrix[row + nOfRows * col];
        }
    }
    */

    /********************************************************/
    static void step2a(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim)
    {
        //uint8_t *starMatrixTemp, *columnEnd;
        index_t col, tmp;

        /* cover every column containing a starred zero */
        for (col = 0; col < nOfColumns; col++)
        {
            // starMatrixTemp = starMatrix + nOfRows * col;
            // columnEnd = starMatrixTemp + nOfRows;
            // while (starMatrixTemp < columnEnd)
            // {
            //     if (*starMatrixTemp++)
            //     {
            //         bv_set(coveredColumns, col);
            //         break;
            //     }
            // }
            for(tmp = nOfRows * col; tmp < nOfRows * (col + 1); tmp++){
                if(bv_at(starMatrix, tmp))
                {
                    bv_set(coveredColumns, col);
                    break;
                }
            }
        }

        /* move to step 3 */
        step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    }

    /********************************************************/
    static void step2b(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim)
    {
        index_t col, nOfCoveredColumns;

        /* count covered columns */
        nOfCoveredColumns = 0;
        for (col = 0; col < nOfColumns; col++)
            if (bv_at(coveredColumns, col))
                nOfCoveredColumns++;

        if (nOfCoveredColumns == minDim)
        {
            /* algorithm finished */
            buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
        }
        else
        {
            /* move to step 3 */
            step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
        }
    }

    /********************************************************/
    static void step3(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim)
    {
        uint8_t zerosFound;
        index_t row, col, starCol;

        zerosFound = 1;
        while (zerosFound)
        {
            zerosFound = 0;
            for (col = 0; col < nOfColumns; col++)
                if (!bv_at(coveredColumns, col)) 
                    for (row = 0; row < nOfRows; row++)
                        if ((!bv_at(coveredRows, row)) && (fabs(distMatrix[row + nOfRows * col]) < ELEM_EPSILON)) //if ((!coveredRows[row]) && (fabs(distMatrix[row + nOfRows * col]) < ELEM_EPSILON))
                        {
                            /* prime zero */
                            bv_set(primeMatrix, row + nOfRows * col);

                            /* find starred zero in current row */
                            for (starCol = 0; starCol < nOfColumns; starCol++)
                                if (bv_at(starMatrix, row + nOfRows * starCol))
                                    break;

                            if (starCol == nOfColumns) /* no starred zero found */
                            {
                                /* move to step 4 */
                                step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
                                return;
                            }
                            else
                            {
                                bv_set(coveredRows, row);
                                bv_clear(coveredColumns, starCol);
                                zerosFound = 1;
                                break;
                            }
                        }
        }

        /* move to step 5 */
        step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    }

    /********************************************************/
    static void step4(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim, index_t row, index_t col)
    {
        index_t n, starRow, starCol, primeRow, primeCol;
        int32_t nOfElements = nOfRows * nOfColumns;

        /* generate temporary copy of starMatrix */
        for (n = 0; n < nOfElements; n++)
            bv_assign(newStarMatrix, n, bv_at(starMatrix, n));

        /* star current zero */
        bv_set(newStarMatrix, row + nOfRows * col);

        /* find starred zero in current column */
        starCol = col;
        for (starRow = 0; starRow < nOfRows; starRow++)
            if (bv_at(starMatrix, starRow + nOfRows * starCol))
                break;

        while (starRow < nOfRows)
        {
            /* unstar the starred zero */
            bv_clear(newStarMatrix, starRow + nOfRows * starCol);

            /* find primed zero in current row */
            primeRow = starRow;
            for (primeCol = 0; primeCol < nOfColumns; primeCol++)
                if (bv_at(primeMatrix, primeRow + nOfRows * primeCol))
                    break;

            /* star the primed zero */
            bv_set(newStarMatrix, primeRow + nOfRows * primeCol);

            /* find starred zero in current column */
            starCol = primeCol;
            for (starRow = 0; starRow < nOfRows; starRow++)
                if (bv_at(starMatrix, starRow + nOfRows * starCol))
                    break;
        }

        /* use temporary copy as new starMatrix */
        /* delete all primes, uncover all rows */
        for (n = 0; n < nOfElements; n++)
        {
            bv_clear(primeMatrix, n);
            bv_assign(starMatrix, n, bv_at(newStarMatrix, n));
        }
        for (n = 0; n < nOfRows; n++)
            bv_clear(coveredRows, n);

        /* move to step 2a */
        step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    }

    /********************************************************/
    static void step5(index_t *assignment, ELEM_T *distMatrix, uint8_t *starMatrix, uint8_t *newStarMatrix, uint8_t *primeMatrix, uint8_t *coveredColumns, uint8_t *coveredRows, index_t nOfRows, index_t nOfColumns, index_t minDim)
    {
        ELEM_T h, value;
        index_t row, col;

        /* find smallest uncovered element h */
        h = ELEM_MAX;
        for (row = 0; row < nOfRows; row++)
            if (!bv_at(coveredRows, row)) 
                for (col = 0; col < nOfColumns; col++)
                    if (!bv_at(coveredColumns, col))
                    {
                        value = distMatrix[row + nOfRows * col];
                        if (value < h)
                            h = value;
                    }

        /* add h to each covered row */
        for (row = 0; row < nOfRows; row++)
            if (bv_at(coveredRows, row))
                for (col = 0; col < nOfColumns; col++)
                    distMatrix[row + nOfRows * col] += h;

        /* subtract h from each uncovered column */
        for (col = 0; col < nOfColumns; col++)
            if (!bv_at(coveredColumns, col))
                for (row = 0; row < nOfRows; row++)
                    distMatrix[row + nOfRows * col] -= h;

        /* move to step 3 */
        step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
    }

#ifdef __cplusplus
}
#endif