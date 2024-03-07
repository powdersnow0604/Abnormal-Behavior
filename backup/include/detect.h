#ifndef __DETECT_H__
#define __DETECT_H__

#include "typedefs.h"
#include "opencv2/opencv.hpp"

void draw_label(cv::Mat &input_image, std::string label, int left, int top);
std::vector<cv::Mat> pre_process(cv::Mat &input_image, cv::dnn::Net &net);
cv::Mat post_process(cv::Mat &input_image, cv::Mat &backup_image, std::vector<cv::Mat> &outputs, const std::vector<std::string> &class_name);
void det_init();
void det_destroy();


#endif