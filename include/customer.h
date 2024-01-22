#ifndef __CUSTOMER_H__
#define __CUSTOMER_H__

#include "typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>

	typedef struct {
		//vector with 7 elements
		//center x, center y, area w*h, ratio w/h, velocity of x, velocity of y velocity of x
		ELEM_T* statemean;

		//vector with 4 elements
		ELEM_T* statecovariance;

		ID_TYPE id;
		time_t timestamp;
		age_t age;
	}customer_t;

	void create_customer(customer_t* customer, ELEM_T* detection, ID_TYPE _id);

#ifdef __cplusplus
}
#endif

#endif