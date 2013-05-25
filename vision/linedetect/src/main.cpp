#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cstdio>
#include <vector>

#include "image_proc.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define DEBUG

#define MIN_ANGLE	(-M_PI/2.0)
#define MAX_ANGLE	(M_PI/2.0)
#define N_SAMPLES	(100)

using namespace cv;
using namespace std;

/* camera parameters */
double fov_x, fov_y;
int width, height;

/* camera position */
double camera_height;
double camera_angle;
/*
 * Project the pixel (x,y) into robot space in 
 * polar coordinates
 */
void project( double x, double y, double &angle, double &dist )
{
	// angle of this image ray out of the camera
	double horiz_angle = fov_x*((x-width/2.0)/(width/2.0));
	double vert_angle = fov_y*((y-height/2.0)/(height/2.0));

	// correct for camera position
	vert_angle -= camera_angle;

	// the length of the ray to the ground
	double hypot = camera_height/cos(vert_angle);
	
	// return values
	dist = sin(-vert_angle)*hypot;
	angle = horiz_angle;
}

int main(int argc, char** argv) 
{
    bool killed = false;

	ros::init(argc, argv, "linedetect");
	ros::NodeHandle nh;

	ros::Publisher linePub = nh.advertise<sensor_msgs::LaserScan>("lines", 10);
	sensor_msgs::LaserScan lineMsg;

	/*
	 * Setup laser msg data
	 */
	lineMsg.angle_min = MIN_ANGLE;
	lineMsg.angle_max = MAX_ANGLE;
	lineMsg.angle_increment = (MAX_ANGLE-MIN_ANGLE)/N_SAMPLES;
	
	lineMsg.time_increment = 0.0;

	lineMsg.scan_time = 1.0;
	
	lineMsg.range_min = 0.0;
	lineMsg.range_max = 5.0;

	lineMsg.ranges = vector<float>(N_SAMPLES, lineMsg.range_max);
	lineMsg.intensities = vector<float>(N_SAMPLES, 0.0);

	/*
	 * Camera settings
	 */
	fov_x = 60.0 *M_PI/180.0;
	fov_y = 60.0 *M_PI/180.0;

	width  = 320;
	height = 240;

	camera_height = 1.0;
	camera_angle = 30 *M_PI/180.0;
    

#ifdef DEBUG
    namedWindow("controls", 0);
#endif
    namedWindow("frame", 0);

    int houghThreshold = 38;
    int houghRho = 3;
    int houghTheta = 1;
    int houghMinLen = 50;
    int houghMaxDist = 75;

    int lower = 49;
    int ksize = 10;
    int sigma = 236;
    int delay = 100;

#ifdef DEBUG
    createTrackbar("threshold", "controls", &houghThreshold, 100);
    createTrackbar("rho", "controls", &houghRho, 100);
    createTrackbar("theta", "controls", &houghTheta, 100);
    createTrackbar("minLen", "controls", &houghMinLen, 1000);
    createTrackbar("maxDist", "controls", &houghMaxDist, 1000);
    createTrackbar("lower", "controls", &lower, 100);
    createTrackbar("ksize", "controls", &ksize, 50);
    createTrackbar("sigma", "controls", &sigma, 500);
    createTrackbar("delay", "controls", &delay, 300);
#endif

	cvWaitKey(100);

    Mat frame;

    bool paused = false;
	bool singleStep = false;

    while(!killed && ros::ok() ) {
		ros::spinOnce();

        VideoCapture capture(argc > 1 ? argv[1] : "frontcam.avi");
        // while we have more frames
        while( (!(paused || singleStep) ? capture.read(frame) : true)  && !killed )
        {
            if( frame.rows == 0 || frame.cols == 0 || frame.channels() == 1 )
                continue;


            Mat frameSmall;
            frame.copyTo(frameSmall);

            // create a grayscale image
            Mat gray(frameSmall.rows, frameSmall.cols, CV_32FC1);

            convertWeightedGrayscale(frameSmall, gray, ksize, sigma);

            Mat grayLeft, grayRight;
            grayLeft = gray.colRange(0, gray.cols/2+1);
            grayRight = gray.colRange(gray.cols/2-1, gray.cols);

            // create a theshold image
            Mat thresholdLeft(grayLeft.rows, grayLeft.cols, CV_8UC1);
            Mat thresholdRight(grayRight.rows, grayRight.cols, CV_8UC1);

            thresholdBrightestPixelRow(grayLeft, thresholdLeft, lower/100.0);
            thresholdBrightestPixelRow(grayRight, thresholdRight, lower/100.0);

            thresholdLeft = thresholdLeft.rowRange(thresholdLeft.rows/4, thresholdLeft.rows);
            thresholdLeft = thresholdLeft.colRange(0, thresholdLeft.cols-1);
            thresholdRight = thresholdRight.rowRange(thresholdRight.rows/4, thresholdRight.rows);
            thresholdRight = thresholdRight.colRange(1, thresholdRight.cols);

            // find lines using houghlines
            vector<Vec4i> linesLeft;
            vector<Vec4i> linesRight;

            try{
                HoughLinesP(thresholdLeft, linesLeft, houghRho, houghTheta*CV_PI/200, 
						houghThreshold, houghMinLen/10.0, houghMaxDist/10.0);
                HoughLinesP(thresholdRight, linesRight, houghRho, houghTheta*CV_PI/200, 
						houghThreshold, houghMinLen/10.0, houghMaxDist/10.0);
            }catch(cv::Exception& e) {}

			/* 
			 * Convert lines into robot space 
			 */ 

			// reset msg
			lineMsg.ranges = vector<float>(N_SAMPLES, lineMsg.range_max);
			
			// handle left
			for( unsigned int i = 0; i < linesLeft.size(); i++)
			{
				for( int j = 0; j <= 2; j+=2 )
				{
					double angle, dist;
					project(linesLeft[i][j], linesLeft[i][j+1], angle, dist);

					int idx = round((angle+((MAX_ANGLE-MIN_ANGLE)/2.0))/lineMsg.angle_increment);
					if( idx < 0 || idx >= N_SAMPLES )
					{
						ROS_ERROR("idx = %i\n", idx);
						ROS_ERROR("\tangle = %f\tdist = %f\n", angle, dist);
						idx = idx < 0 ? 0 : N_SAMPLES-1;
					}

					// store the closes thing
					if( lineMsg.ranges[idx] > dist )
					{
						lineMsg.ranges[idx] = dist;
					}
				}
			}

			// handle right
			for( unsigned int i = 0; i < linesRight.size(); i++)
			{
				for( int j = 0; j <= 2; j+=2 )
				{
					double angle, dist;
					project(linesRight[i][j], linesRight[i][j+1], angle, dist);

					int idx = round(angle/lineMsg.angle_increment);

					// store the closes thing
					if( lineMsg.ranges[idx] > dist )
					{
						lineMsg.ranges[idx] = dist;
					}
				}
			}

#ifdef DEBUG
            for( size_t i = 0; i < linesLeft.size(); i++ )
            {
                line( frameSmall, Point(linesLeft[i][0], linesLeft[i][1]+frameSmall.rows/4), 
                        Point(linesLeft[i][2], linesLeft[i][3]+frameSmall.rows/4), 
                        Scalar(0,0,255), 2 );
            }
            for( size_t i = 0; i < linesRight.size(); i++ )
            {
                line( frameSmall, Point(frameSmall.cols/2 + linesRight[i][0], frameSmall.rows/4+linesRight[i][1]), 
                        Point(frameSmall.cols/2 + linesRight[i][2], frameSmall.rows/4+linesRight[i][3]), 
                        Scalar(255,0,0), 2 );
            }

            imshow("frame", frame);
            imshow("frameSmall", frameSmall);
            imshow("gray", gray);
            imshow("Left", thresholdLeft);
            imshow("Right", thresholdRight);
#endif

			/* send the msg */
			linePub.publish(lineMsg);

            char key = cvWaitKey(delay);
            switch(key) {
                case 'q': 
                    killed = true;
                    break;
				case 's': case 'S':
					singleStep = !singleStep;
					break;
                case ' ':
                    paused = !paused;
                    break;
            }
        }
    }
    return 0;
}
