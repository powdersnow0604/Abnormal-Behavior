#include "customer.h"
#include "kalman_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

	void create_customer(customer_t* customer, ELEM_T* detection, ID_TYPE _id)
	{
		kf_initialize_track(detection, customer->statemean, customer->statecovariance);
		customer->id = _id;
		customer->age = 0;
		time(&customer->timestamp);
	}
#ifdef __cplusplus
}
#endif