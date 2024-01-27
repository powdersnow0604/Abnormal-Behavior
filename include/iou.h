#ifndef __IOU_H__
#define __IOU_H__

#include "typedefs.h"
#include "customer.h"

#ifdef __cplusplus
extern "C" {
#endif

	//get iou between single track and single detection
	ELEM_T get_iou(ELEM_T* sv_track, const detection_t* sv_det);

	//get cost matrix by iou
	void get_cost_mat_iou(ELEM_T* cost_mat, const customer_t* tracks, const detection_t* detections, index_t trk_num, index_t det_num);

	//xysr -> tlbr
	QELEM_T to_tlbr(const ELEM_T* sv);

	//tlbr -> xysr, and store in sv
	void to_xysr(const detection_t* tlbr, ELEM_T* xysr);
#ifdef __cplusplus
}
#endif

#endif