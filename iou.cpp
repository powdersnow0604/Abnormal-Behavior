#include "iou.h"

extern "C" {
	static ELEM_T max(ELEM_T ls, ELEM_T rs)
	{
		return ls > rs ? ls : rs;
	}

	static ELEM_T min(ELEM_T ls, ELEM_T rs)
	{
		return ls < rs ? ls : rs;
	}

	ELEM_T get_iou(ELEM_T* sv_track, ELEM_T* sv_det)
	{
		QELEM_T tlbr_t = to_tlbr(sv_track);
		QELEM_T tlbr_d = to_tlbr(sv_det);

		ELEM_T x1, x2, y1, y2, w, h, inter, iou;
		x1 = max(tlbr_t.e1, tlbr_d.e1);
		y1 = max(tlbr_t.e2, tlbr_d.e2);
		x2 = max(tlbr_t.e3, tlbr_d.e3);
		y2 = max(tlbr_t.e4, tlbr_d.e4);

		w = max(0, x2 - x1);
		h = max(0, y2 - y1);

		inter = w * h;
		iou = inter / (sv_track[3] + sv_det[3] + inter);

		return iou;
	}

	QELEM_T to_tlbr(ELEM_T* sv)
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

	void to_xysr(QELEM_T* tlbr, ELEM_T* sv)
	{
		ELEM_T h, w;

		w = ABS_T(tlbr->e3 - tlbr->e1);
		h = ABS_T(tlbr->e4 - tlbr->e2);

		sv[0] = tlbr->e1 + w / 2;
		sv[1] = tlbr->e2 + h / 2;
		sv[2] = w * h;
		sv[3] = w / h;
	}
}