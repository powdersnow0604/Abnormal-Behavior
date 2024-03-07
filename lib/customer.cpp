#include "customer.h"
#include "kalman_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

	void create_customer(customer_t* customer, const detection_t* detection, ID_TYPE _id)
	{
		kf_initialize_track(detection, customer->statemean, customer->statecovariance);
		customer->id = _id;
		customer->age = 0;
		customer->is_occluded = 0;
		customer->is_stable = 0;
		customer->cmf = 0;
		time(&customer->timestamp);
		customer->tso = 0;
	}
#ifdef __cplusplus
}
#endif