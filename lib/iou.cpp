#include "iou.h"

#ifdef __cplusplus
extern "C" {
#endif

	static inline ELEM_T max(ELEM_T lhs, ELEM_T rhs)
	{
		return lhs > rhs ? lhs : rhs;
	}

	static inline ELEM_T min(ELEM_T lhs, ELEM_T rhs)
	{
		return lhs < rhs ? lhs : rhs;
	}

	ELEM_T get_iou(ELEM_T* sv_track, const detection_t* sv_det)
	{
		//sv_track 은 xysr 로, sv_det 는 tlbr 로 들어오는 것을 가정
		QELEM_T tlbr_t = to_tlbr(sv_track);
		ELEM_T box_area_d = (sv_det->x2 - sv_det->x1) * (sv_det->y2 - sv_det->y1);

		ELEM_T x1, x2, y1, y2, w, h, inter, iou;
		x1 = max(tlbr_t.e1, sv_det->x1);
		y1 = max(tlbr_t.e2, sv_det->y1);
		x2 = min(tlbr_t.e3, sv_det->x2);
		y2 = min(tlbr_t.e4, sv_det->y2);

		w = max(0, x2 - x1 + 1);
		h = max(0, y2 - y1 + 1);

		inter = w * h;
		iou = inter / (sv_track[2] + box_area_d + inter);

		return iou;
	}

	void get_cost_mat_iou(ELEM_T* cost_mat, const customer_t* tracks, const detection_t* detections, index_t trk_num, index_t det_num, index_t ldm)
	{
		index_t i, j;
		for(i = 0; i < trk_num; ++i){
			for(j = 0; j < det_num; ++i){
				cost_mat[i * (uint32_t)ldm + j] = 1 - get_iou(tracks[i].statemean, detections + j);
			}
		}
	}

	QELEM_T to_tlbr(const ELEM_T* sv)
	{
		ELEM_T h, w, tmp;
		w = SQRT_T(sv[2] * sv[3]);
		h = sv[2] / w;

		QELEM_T ret;

		tmp = w / 2;
		ret.e1 = sv[0] - tmp;
		ret.e3 = sv[0] + tmp;

		tmp = h / 2;
		ret.e2 = sv[1] - tmp;
		ret.e4 = sv[1] + tmp;

		return ret;
	}

	void to_xysr(const detection_t* tlbr, ELEM_T* xysr)
	{
		ELEM_T h, w;

		w = tlbr->x2 - tlbr->x1 + 1;
		h = tlbr->y2 - tlbr->y1 + 1;

		xysr[0] = tlbr->x1 + w / 2;
		xysr[1] = tlbr->y1 + h / 2;
		xysr[2] = w * h;
		xysr[3] = w / h;
	}
#ifdef __cplusplus
}
#endif