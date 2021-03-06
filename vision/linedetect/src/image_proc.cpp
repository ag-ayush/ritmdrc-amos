#include "image_proc.h"

#include <vector>
using namespace std;

#include <cstdio>

//#define DEBUG

/*
 * Convert the provided RGB frame into grayscale
 * using the following algorithm:
 *      gray = 2*frame[blue] - frame[green]
 */
void convertWeightedGrayscale(const Mat &frame, Mat &gray, int ksize, int sigma) 
{
    // split frame into its individual channels
    Mat tmp;
    vector<Mat> frameHSV;
    vector<Mat> frameChannels;
    split(frame, frameChannels);

    cvtColor(frame, tmp, CV_BGR2HSV);
    split(tmp, frameHSV);

    Mat blue, green;

#ifdef DEBUG
    printf("Frame %dx%d, %d channels\n", frame.cols, frame.rows, frame.channels());
    printf("\tFrame[BLUE] %dx%d\n", frameChannels[BLUE].cols, frameChannels[BLUE].rows);
    printf("\tFrame[GREEN] %dx%d\n", frameChannels[GREEN].cols, frameChannels[GREEN].rows);
#endif

    //medianBlur(frameChannels[BLUE], blue, 3);
    //medianBlur(frameChannels[GREEN], green, 3);
    frameChannels[BLUE].copyTo(blue);
    frameChannels[GREEN].copyTo(green);

    float max = -std::numeric_limits<float>::max();
    float min = std::numeric_limits<float>::max();
    for( int i = 0; i < frame.rows; i++ )
    {
        for( int j = 0; j < frame.cols; j++ )
        {
            float value = 2*blue.at<unsigned char>(i,j) - green.at<unsigned char>(i,j);
            value += frameHSV[0].at<unsigned char>(i,j);

            gray.at<float>(i,j) = value;

            if( value > max )
                max = value;
            if( value < min )
                min = value;
        }
    }

    float diff = max-min;
    if( diff == 0 )
        diff = 1;

    for( int i = 0; i < frame.rows; i++ )
    {
        for( int j = 0; j < frame.cols; j++ )
        {
            //uint8_t hue = frameHSV[0].at<uint8_t>(i, j);
            //uint8_t val = hue > 85;
            gray.at<float>(i,j) = (gray.at<float>(i,j)-min)/diff;
            gray.at<float>(i,j) = gray.at<float>(i,j)*gray.at<float>(i,j);
            //gray.at<float>(i,j) *= frameHSV[0].at<unsigned char>(i,j)/179.0;
        }
    }

    //medianBlur(gray, gray, 3);
    GaussianBlur(gray, gray, Size(ksize*2+1, ksize*2+1), sigma/100.0);
}

/*
 * Threshold the image by keeping only the brightest pixel
 * value in each row, and in each column
 */
void thresholdBrightestPixelRow(const Mat &gray, Mat &thresh, float lower)
{
    int i, j;
    for( i = 0; i < gray.rows; i++ )
    {
        float largest  = 0;
        int largestIdx = 0;
        for( j = 0; j < gray.cols; j++ )
        {
            thresh.at<unsigned char>(i,j) = 0;
            if( gray.at<float>(i,j) > largest )
            {
                largest = gray.at<float>(i,j);
                largestIdx = j;
            }
        }
    
        if( largest > lower )
            thresh.at<unsigned char>(i,largestIdx) = 255;
    }

    for( j = 0; j < gray.cols; j++ )
    {
        float largest  = 0;
        int largestIdx = 0;
        for( i = 0; i < gray.rows; i++ )
        {
            if( gray.at<float>(i,j) > largest )
            {
                largest = gray.at<float>(i,j);
                largestIdx = i;
            }
        }
    
        if( largest > lower )
            thresh.at<unsigned char>(largestIdx,j) = 255;
    }
}


float otsuAlgorithm(Mat gray, int maxIntensity)
{
    if( maxIntensity <= 0) return 0;

    const int channel = 0;
    const int histSize = maxIntensity;
    float range[] = {0,1};
    const float* ranges[] = {range};
    Mat hist, prob;
    calcHist( &gray, 1, &channel, Mat(), // do not use mask
              hist, 1, &histSize, ranges);

    prob.create(hist.size(), hist.type());

    for( int i = 0; i < hist.rows; i++ )
    {
        prob.at<float>(i) =  hist.at<float>(i)/(gray.rows*gray.cols);
    }

    float largestSigma = 0;
    int largestIntensity = 0;
    for( int i = 0; i < maxIntensity; i++)
    {
        // compute w1, u1
        float w1 = 0.0, u1 = 0.0;
        for( int j = 0; j <= i; j++ )
        {
            w1 += prob.at<float>(j);
            u1 += prob.at<float>(j)*hist.at<float>(j);
        }

        // compute w2, u2
        float w2 = 0.0, u2 = 0.0;
        for( int j = i+1; j < maxIntensity; j++ )
        {
            w2 += prob.at<float>(j);
            u2 += prob.at<float>(j)*hist.at<float>(j);
        }

        float sigma = w1*w2*(u1-u2)*(u1-u2);
        if( sigma > largestSigma )
        {
            largestSigma = sigma;
            largestIntensity = i;
        }
    }
    return ((float)largestIntensity)/maxIntensity;
}
