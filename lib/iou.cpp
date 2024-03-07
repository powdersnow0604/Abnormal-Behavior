#include "iou.h"


#ifdef __cplusplus
extern "C"
{
#endif

	const ELEM_T iou_threshold = 0.3;

	static inline ELEM_T max(ELEM_T lhs, ELEM_T rhs)
	{
		return lhs > rhs ? lhs : rhs;
	}

	static inline ELEM_T min(ELEM_T lhs, ELEM_T rhs)
	{
		return lhs < rhs ? lhs : rhs;
	}

	ELEM_T get_iou(ELEM_T *sv_track, const detection_t *sv_det)
	{
		// sv_track 은 xysr 로, sv_det 는 tlbr 로 들어오는 것을 가정
		QELEM_T tlbr_t = to_tlbr(sv_track);
		ELEM_T box_area_d = (sv_det->x2 - sv_det->x1) * (sv_det->y2 - sv_det->y1);

		ELEM_T x1, x2, y1, y2, w, h, inter, iou;
		x1 = max(tlbr_t.e1, sv_det->x1);
		y1 = max(tlbr_t.e2, sv_det->y1);
		x2 = min(tlbr_t.e3, sv_det->x2);
		y2 = min(tlbr_t.e4, sv_det->y2);

		w = max(0, x2 - x1);
		h = max(0, y2 - y1);

		inter = w * h;
		iou = inter / (sv_track[2] + box_area_d - inter);

		return iou;
	}

	ELEM_T get_ext_iou(ELEM_T *sv_track, const detection_t *sv_det, ELEM_T ext_w, ELEM_T ext_h)
	{
		// sv_track 은 xysr 로, sv_det 는 tlbr 로 들어오는 것을 가정
		QELEM_T tlbr_t = to_tlbr(sv_track);
		ELEM_T box_area_d = (sv_det->x2 - sv_det->x1) * (sv_det->y2 - sv_det->y1);

		ELEM_T x1, x2, y1, y2, w, h, inter, iou;
		w = SQRT_T(sv_track[2] * sv_track[3]);
		h = sv_track[2] / w;

		// ext_h = 0; /////
		// ext_w = 0; /////
		x1 = max(tlbr_t.e1 - w * ext_w / 2, sv_det->x1);
		y1 = max(tlbr_t.e2 - h * ext_h / 2, sv_det->y1);
		x2 = min(tlbr_t.e3 + w * ext_w / 2, sv_det->x2);
		y2 = min(tlbr_t.e4 + h * ext_h / 2, sv_det->y2);

		w = max(0, x2 - x1);
		h = max(0, y2 - y1);

		inter = w * h;
		iou = inter / (sv_track[2] + box_area_d - inter);

		return iou;
	}

	void get_cost_mat_iou(ELEM_T *cost_mat, const customer_t *tracks, const detection_t *detections, index_t trk_num, index_t det_num)
	{
		// row for detections, col for tracks
		index_t i, j, k = 0;
		for (j = 0; j < det_num; ++j)
		{
			for (i = 0; i < trk_num; ++i)
			{
				cost_mat[k++] = 1 - get_iou(tracks[i].statemean, detections + j);
			}
		}
	}

	void get_cost_mat_ext_iou(ELEM_T *cost_mat, const customer_t *tracks, const detection_t *detections, index_t *um_tracks, index_t *um_dets, index_t trk_num, index_t det_num)
	{
		// row for detections, col for tracks
		index_t i, j, k = 0;
		for (j = 0; j < det_num; ++j)
		{
			for (i = 0; i < trk_num; ++i)
			{
				cost_mat[k++] = 1 - get_ext_iou(tracks[um_tracks[i]].statemean, detections + um_dets[j], (tracks[um_tracks[i]].tso + 1) * 0.2 / UINT8_MAX, (tracks[um_tracks[i]].tso + 1) * 0.2 / UINT8_MAX);
			}
		}
	}

	ELEM_T get_confidence(const customer_t *track, const ELEM_T avg_area, const ELEM_T alpha)
	{
		return min(1, alpha * track->statemean[2] / (((track->age & 0x7f) + 1) * avg_area));
	}

	ELEM_T get_cp(const customer_t *target, const customer_t *tracks, index_t trk_num, ELEM_T thresh)
	{
		const customer_t *iter = tracks, *end = tracks + trk_num;
		ELEM_T maxval = 0;
		for (; iter < end; iter++)
		{
			if (iter != target)
			{
				QELEM_T tlbr_t = to_tlbr(target->statemean);
				QELEM_T tlbr_i = to_tlbr(iter->statemean);

				ELEM_T x1, x2, y1, y2, w, h, inter;

				x1 = max(tlbr_t.e1, tlbr_i.e1);
				y1 = max(tlbr_t.e2, tlbr_i.e2);
				x2 = min(tlbr_t.e3, tlbr_i.e3);
				y2 = min(tlbr_t.e4, tlbr_i.e4);

				w = max(0, x2 - x1);
				h = max(0, y2 - y1);

				inter = w * h / target->statemean[2];

				if(inter > thresh) return inter;

				if(maxval < inter) maxval = inter;
			}
		}

		return maxval;
	}

	QELEM_T to_tlbr(const ELEM_T *sv)
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

	void to_xysr(const detection_t *tlbr, ELEM_T *xysr)
	{
		ELEM_T h, w;

		w = tlbr->x2 - tlbr->x1;
		h = tlbr->y2 - tlbr->y1;

		xysr[0] = tlbr->x1 + w / 2;
		xysr[1] = tlbr->y1 + h / 2;
		xysr[2] = w * h;
		xysr[3] = w / h;
	}
#ifdef __cplusplus
}
#endif