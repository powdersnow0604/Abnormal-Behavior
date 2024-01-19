#include "customer.h"

extern "C" {

	customer_t create_customer(ELEM_T* sv, ELEM_T* sc, ID_TYPE _id)
	{
		customer_t ret;

		ret.statevector = sv;
		ret.statecovariance = sc;
		ret.id = _id;
		time(&ret.timestamp);

		return ret;
	}
}