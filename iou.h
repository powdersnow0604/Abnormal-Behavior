#ifndef __IOU_H__
#define __IOU_H__

#include "typedefs.h"

extern "C" {

	//get iou between single track and single detection
	ELEM_T get_iou(ELEM_T* sv_track, ELEM_T* sv_det);

	//xysr -> tlbr
	QELEM_T to_tlbr(ELEM_T* sv);

	//tlbr -> xysr, and store in sv
	void to_xysr(QELEM_T* tlbr, ELEM_T* sv);

}


#endif