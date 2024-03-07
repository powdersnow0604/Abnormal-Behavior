#ifndef __TRACKER_H__
#define __TRACKER_H__

#include "typedefs.h"
#include "customer.h"

#ifdef __cplusplus
extern "C" {
#endif

void tk_init(void);
void tk_destroy(void);
void tk_create_new_track(const detection_t* detections, index_t num);
void tk_predict(void);
void tk_update(const detection_t* detection, index_t num);
void tk_mark_missed(index_t missed);

customer_t *tk_get_tracks(void);
index_t tk_get_track_num(void);

#ifdef __cplusplus
}
#endif

#endif