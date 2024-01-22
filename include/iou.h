#ifndef __IOU_H__
#define __IOU_H__

#include "typedefs.h"
#include "customer.h"

#ifdef __cplusplus
extern "C" {
#endif

	//get iou between single track and single detection
	ELEM_T get_iou(ELEM_T* sv_track, ELEM_T* sv_det);

	//get cost matrix by iou
	void get_cost_mat_iou(ELEM_T* cost_mat, customer_t* tracks, ELEM_T* detections, index_t trk_num, index_t det_num);

	//xysr -> tlbr
	QELEM_T to_tlbr(const ELEM_T* sv);

	//tlbr -> xysr, and store in sv
	void to_xysr(const ELEM_T* tlbr, ELEM_T* xysr);
#ifdef __cplusplus
}
#endif

#endif