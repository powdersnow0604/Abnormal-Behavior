#include "iou.h"

#ifdef __cplusplus
extern "C" {
#endif

	static inline ELEM_T max(ELEM_T ls, ELEM_T rs)
	{
		return ls > rs ? ls : rs;
	}

	static inline ELEM_T min(ELEM_T ls, ELEM_T rs)
	{
		return ls < rs ? ls : rs;
	}

	ELEM_T get_iou(ELEM_T* sv_track, ELEM_T* sv_det)
	{
		//sv_track 은 xysr 로, sv_det 는 tlbr 로 들어오는 것을 가정
		QELEM_T tlbr_t = to_tlbr(sv_track);
		ELEM_T box_area_d = (sv_det[2] - sv_det[0] + 1) * (sv_det[3] - sv_det[1] + 1);

		ELEM_T x1, x2, y1, y2, w, h, inter, iou;
		x1 = max(tlbr_t.e1, sv_det[0]);
		y1 = max(tlbr_t.e2, sv_det[1]);
		x2 = min(tlbr_t.e3, sv_det[2]);
		y2 = min(tlbr_t.e4, sv_det[3]);

		w = max(0, x2 - x1 + 1);
		h = max(0, y2 - y1 + 1);

		inter = w * h;
		iou = inter / (sv_track[2] + box_area_d + inter);

		return iou;
	}

	void get_cost_mat_iou(ELEM_T* cost_mat, customer_t* tracks, ELEM_T* detections, index_t trk_num, index_t det_num)
	{
		index_t i, j;
		for(i = 0; i < trk_num; ++i){
			for(j = 0; j < det_num; ++i){
				cost_mat[i * (uint16_t)tk_max_tracks + j] = 1 - get_iou(tracks[i].statemean, detections + j * (uint16_t)kf_mv_size);
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

	void to_xysr(const ELEM_T* tlbr, ELEM_T* xysr)
	{
		ELEM_T h, w;

		w = tlbr[2] - tlbr[0] + 1;
		h = tlbr[3] - tlbr[1] + 1;

		xysr[0] = tlbr[0] + w / 2;
		xysr[1] = tlbr[1] + h / 2;
		xysr[2] = w * h;
		xysr[3] = w / h;
	}
#ifdef __cplusplus
}
#endif