#ifndef __IOU_H__
#define __IOU_H__

#include "typedefs.h"
#include "customer.h"

#ifdef __cplusplus
extern "C" {
#endif

	//get iou between single track and single detection
	ELEM_T get_iou(ELEM_T* sv_track, const detection_t* sv_det);

	//get iou between single track and single occlusion point
	ELEM_T get_iou_occ_track(ELEM_T* sv_track, const occlusion_point* occ);

	//get iou between single detection and single occlusion point
	ELEM_T get_iou_occ_det(const detection_t* det, const occlusion_point* occ);

	//get cost matrix by iou
	void get_cost_mat_iou(ELEM_T* cost_mat, const customer_t* tracks, const detection_t* detections, index_t trk_num, index_t det_num);

	//get confidence
	ELEM_T get_confidence(const customer_t* track, const ELEM_T avg_area, const ELEM_T alpha);

	//get covered percent
	ELEM_T get_cp(const customer_t* target, const customer_t* tracks, index_t trk_num, ELEM_T thresh);

	//xysr -> tlbr
	QELEM_T to_tlbr(const ELEM_T* sv);

	//tlbr -> xysr, and store in sv
	void to_xysr(const detection_t* tlbr, ELEM_T* xysr);
#ifdef __cplusplus
}
#endif

#endif