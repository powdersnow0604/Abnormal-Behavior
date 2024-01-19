#ifndef __CUSTOMER_H__
#define __CUSTOMER_H__

#include "typedefs.h"

extern "C" {
#include <time.h>

	typedef struct {
		//vector with 7 elements
		//center x, center y, area w*h, ratio w/h, velocity of x, velocity of y velocity of x
		ELEM_T* statevector;

		//matrix whose size is (7, 7)
		ELEM_T* statecovariance;

		ID_TYPE id;
		time_t timestamp;
	}customer_t;

	customer_t create_customer(ELEM_T* sv, ELEM_T* sc, ID_TYPE _id);
}

#endif