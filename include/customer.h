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

		time_t timestamp;
		ID_TYPE id;
		age_t age : 6;
		age_t is_occluded : 1;
		age_t is_stable : 1;
		uint8_t tso; //time since observed
		uint8_t cmf; //continuous matched frmaes
	}customer_t;

	void create_customer(customer_t* customer, const detection_t* detection, ID_TYPE _id);

#ifdef __cplusplus
}
#endif

#endif