#ifndef __CUSTOMER_H__
#define __CUSTOMER_H__

#include "typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>

	void create_customer(customer_t* customer, const detection_t* detection, ID_TYPE _id);

#ifdef __cplusplus
}
#endif

#endif