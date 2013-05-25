#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include <opencv2/opencv.hpp>

#define RED     2
#define GREEN   1
#define BLUE    0
using namespace cv;

void convertWeightedGrayscale(const Mat &frame, Mat &gray, int ksize, int sigma);
void thresholdBrightestPixelRow(const Mat &gray, Mat &thresh, float lower);
float otsuAlgorithm(Mat gray, int maxIntensity);

#endif
