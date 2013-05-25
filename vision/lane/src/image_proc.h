#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include <opencv2/opencv.hpp>

#define RED     2
#define GREEN   1
#define BLUE    0

void convertWeightedGrayscale(const cv::Mat &frame, cv::Mat &gray, int ksize, float sigma);
void thresholdBrightestPixelRow(const cv::Mat &gray, cv::Mat &thresh, float lower);
float otsuAlgorithm(cv::Mat gray, int maxIntensity);

#endif
