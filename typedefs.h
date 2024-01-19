#ifndef __TYPEDEFS_H__
#define __TYPEDEFS_H__

extern "C" {
#include <inttypes.h>
#include <math.h>

//sqrt for ELEM_T
#define SQRT_T sqrtf

//abs for ELEM_T
#define ABS_T fabsf

	//maybe floating point
	typedef float ELEM_T;

	//corresponding integer type to ELEM_T for type conversion
	typedef uint32_t CO_ELEM_T;

	typedef uint16_t ID_TYPE;

	typedef struct {
		ELEM_T e1;
		ELEM_T e2;
		ELEM_T e3;
		ELEM_T e4;
	}QELEM_T;
}

#endif