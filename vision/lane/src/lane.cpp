#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <lane_nav_msgs/MarkerArray.h>
#include <lane_nav_msgs/Marker.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <dynparam/boolparam.h>
#include <dynparam/doubleparam.h>
#include <dynparam/intparam.h>
#include <dynparam/stringparam.h>

#include <camera_tools/groundproj.h>

#include <vector>

#include "image_proc.h"

#include <iostream>

sensor_msgs::ImageConstPtr img_raw;
sensor_msgs::LaserScanConstPtr lidar_raw;

ros::Publisher pLidar;

ros::Publisher marker_pub;

ros::Publisher vis_pub;
visualization_msgs::Marker marker;

bool recvCam = false, recvLidar = false;

camera_tools::GroundProjection *gp;

DoubleParam msgWindow;
DoubleParam sigma;
DoubleParam houghRho, houghTheta, houghThreshold, houghMinLen, houghMaxDist;
DoubleParam straightAngle;

IntParam ksize, lower;

BoolParam showDebug;
BoolParam always_straight;
BoolParam disabled;

IntParam lidarFixRange;

StringParam markerFrame;

tf::TransformListener *tl;

double height; //Camera height above lidar

void findLines(const cv::Mat& im, std::vector<cv::Vec4i>& lines) {
	cv::Mat gray(im.rows, im.cols, CV_32FC1);
	cv::Mat thresh;

	convertWeightedGrayscale(im, gray, ksize.get(), sigma.get());

	cv::Mat grayLeft, grayRight;
	grayLeft = gray.colRange(0, gray.cols / 2 + 1);
	grayRight = gray.colRange(gray.cols / 2 - 1, gray.cols);

	// create a theshold image
	cv::Mat thresholdLeft(grayLeft.rows, grayLeft.cols, CV_8UC1);
	cv::Mat thresholdRight(grayRight.rows, grayRight.cols, CV_8UC1);

	thresholdBrightestPixelRow(grayLeft, thresholdLeft, lower.get() / 100.0);
	thresholdBrightestPixelRow(grayRight, thresholdRight, lower.get() / 100.0);

	thresholdLeft = thresholdLeft.rowRange(0, //thresholdLeft.rows / 4,
			thresholdLeft.rows);
	thresholdLeft = thresholdLeft.colRange(0, thresholdLeft.cols - 1);
	thresholdRight = thresholdRight.rowRange(0, //thresholdRight.rows, / 4,
			thresholdRight.rows);
	thresholdRight = thresholdRight.colRange(1, thresholdRight.cols);

	std::vector<cv::Vec4i> left, right;

	try {
		cv::HoughLinesP(thresholdLeft, left, houghRho.get(), houghTheta.get(),
				houghThreshold.get(), houghMinLen.get(), houghMaxDist.get());
		cv::HoughLinesP(thresholdRight, right, houghRho.get(), houghTheta.get(),
				houghThreshold.get(), houghMinLen.get(), houghMaxDist.get());
	} catch (cv::Exception& e) {
	}

	//Adjust points back to the correct location in the original image

	int c_2 = im.cols / 2;
	for (size_t i = 0; i < left.size(); ++i) {
		lines.push_back(cv::Vec4i(
				left[i][0], left[i][1],
				left[i][2], left[i][3]));
	}

	for (size_t i = 0; i < right.size(); ++i) {
		lines.push_back(cv::Vec4i(
				c_2 + right[i][0], right[i][1],
				c_2 + right[i][2], right[i][3]));
	}
}

/**
 * Transforms a list of points in camera image space into lidar space by projecting onto the
 * ground and then transforming into lidar space.
 *
 * NOTE: This function currently assumes a lidar that scans parallel to the ground. In order
 * to make this work for any lidar angle, the point returned from the projection would have
 * to be then projected into the lidar scan along the ground normal.
 *
 * @param lines The list of line segments to project into lidar space. The points are modified in place.
 * @param lidarPts Out param that contains the points projected into the lidar frame. Every two points
 * constitutes a line.
 */
void projectImageToLidar(std::vector<cv::Vec4i>& lines, std::vector<tf::Vector3>& lidarPts) {
	tf::Vector3 ptC, ptL;
	tf::StampedTransform ltf;

	try {
		tl->lookupTransform(markerFrame.get(), "/camera", ros::Time(), ltf);
	} catch (tf::TransformException& e) {
		ROS_ERROR("LANE: %s", e.what());
		return;
	}

	for (size_t i = 0; i < lines.size(); ++i) {
		ptC = gp->project(lines[i][0], lines[i][1], height);

		//std::cout << ptC.x() << ", " << ptC.y() << ", " << ptC.z() << std::endl;
		ptL = ltf * ptC;
		lidarPts.push_back(ptL);

		ptC = gp->project(lines[i][2], lines[i][3], height);
		ptL = ltf * ptC;
		lidarPts.push_back(ptL);
	}
}

void adjustLidarScan(std::vector<tf::Vector3>& lidarPts) {
	double thMin, thMax, tmp;
	double theta, d;

	sensor_msgs::LaserScan newLidar(*lidar_raw);

	size_t start, end;

    size_t minAngleChanged = lidar_raw->ranges.size(), maxAngleChanged = 0;

	for (size_t i = 0; i < lidarPts.size(); i += 2) {
		tf::Vector3 ptA = lidarPts[i];
		tf::Vector3 ptB = lidarPts[i + 1];

        try {
            tf::StampedTransform ltf;
            tl->lookupTransform("/laser", markerFrame.get(), ros::Time(), ltf);

            ptA = ltf*ptA;
            ptB = ltf*ptB;

        } catch (tf::TransformException& e) {
            ROS_ERROR("LANE: %s", e.what());
            return;
        }
		thMin = std::atan2(ptA.y(), ptA.x());
		thMax = std::atan2(ptB.y(), ptB.x());

		if (thMin > thMax) {
			tmp = thMin;
			thMin = thMax;
			thMax = tmp;
		}

		start = (thMin - lidar_raw->angle_min) / lidar_raw->angle_increment;
		end = (thMax - lidar_raw->angle_min) / lidar_raw->angle_increment;

		for (;start <= end; ++start) {
            theta = start*lidar_raw->angle_increment + lidar_raw->angle_min;
			double s = sin(theta);
			double c = cos(theta);
			d = ((ptB.x() - ptA.x()) * ptB.y() - (ptB.y() - ptA.y()) * ptB.x()) / ((ptB.x() - ptA.x()) * s - (ptB.y() - ptA.y()) * c);

			//std::cout << "line/laser d: " << d << ", " << lidar_raw->ranges[start] << std::endl;

			if (d < lidar_raw->ranges[start]) {
				newLidar.ranges[start] = d;

                if( start < minAngleChanged ) minAngleChanged = start;
                if( start > maxAngleChanged ) maxAngleChanged = start;
			}
		}
	}

    ROS_INFO("lane: min_angle = %i\tmax_angle = %i", minAngleChanged, maxAngleChanged);
    // pull the min/max angle back to end of scan
    if( minAngleChanged <= lidarFixRange.get() )
    {
        for( int i = minAngleChanged-1; i >= 0; i-- )
        {
            newLidar.ranges[i] = newLidar.ranges[minAngleChanged];
        }
    }

    if( maxAngleChanged >= lidar_raw->ranges.size() - lidarFixRange.get() )
    {
        for( int i = maxAngleChanged+1; i < newLidar.ranges.size(); i++ )
        {
            newLidar.ranges[i] = newLidar.ranges[maxAngleChanged];
        }
    }

	pLidar.publish(newLidar);
}

double dist( geometry_msgs::PoseStamped& amos, tf::Vector3 &marker )
{
	return hypot( amos.pose.position.x - marker.x(), amos.pose.position.y - marker.y() );
}

geometry_msgs::PoseStamped toPose( tf::Vector3 point )
{
	geometry_msgs::PoseStamped pose;

	pose.pose.position.x = point.x();
	pose.pose.position.y = point.y();
	pose.pose.position.z = point.z();
	pose.pose.orientation.w = 1;

	return pose;
}


void sendMarkers( std::vector<tf::Vector3> &lines ) {
	lane_nav_msgs::MarkerArray markers;

	geometry_msgs::PoseStamped origin;
	origin.pose.position.x = 0;
	origin.pose.position.y = 0;
	origin.pose.position.z = 0;
	origin.pose.orientation.w = 1;
	origin.header.frame_id = "/amos";

	geometry_msgs::PoseStamped closer, farther, amos;
	ros::Time t( ros::Time::now() );
	closer.header.frame_id = markerFrame.get();
	closer.header.stamp = t;
	farther.header.frame_id = markerFrame.get();
	farther.header.stamp = t;

	// get amos' positionin the map
	tl->transformPose( markerFrame.get(), origin, amos );

	double amos_yaw = tf::getYaw( amos.pose.orientation );

	for( size_t i = 0; i < lines.size(); i+=2 )
	{
		lane_nav_msgs::Marker line_marker;

        if (!lines[i]) {
            ROS_WARN("lines[%lu] is null!", i);
            continue;
        }

        if (!lines[i + 1]) {
            ROS_WARN("lines[%lu + 1] is null!", i);
            continue;
        }

		if( dist( amos, lines[i] ) < dist( amos, lines[i+1] ) )
		{
			closer = toPose( lines[i] );
			farther = toPose( lines[i+1] );
		}
		else
		{
			closer = toPose( lines[i+1] );
			farther = toPose( lines[i] );
		}

		double dx, dy;
		dx = farther.pose.position.x - closer.pose.position.x;
		dy = farther.pose.position.y - closer.pose.position.y;
		double line_angle = atan2( dy, dx );

		// check if the line has a small enought delta angle to be left/right
		if( !always_straight.get() && fabs( amos_yaw - line_angle ) < straightAngle.get() )
		{
			// LEFT/RIGHT type

			// check whether one of the points is to the left or right of amos
			if( atan2( farther.pose.position.y - amos.pose.position.y,
						farther.pose.position.x  - amos.pose.position.x ) <= amos_yaw )
			{
				// negative angle == right
				line_marker.type = lane_nav_msgs::Marker::RIGHT;
			}
			else
			{
				// positive angle == left
				line_marker.type = lane_nav_msgs::Marker::LEFT;
			}
		}
		else
		{
			// STRAIGHT type
			line_marker.type = lane_nav_msgs::Marker::STRAIGHT;
		}

		line_marker.closer = closer;
		line_marker.closer.header = closer.header;

		line_marker.farther = farther;
		line_marker.farther.header = farther.header;

		markers.markers.push_back(line_marker);
		markers.header.stamp = t;
		
		if( showDebug.get() )
		{
			marker.id = i;
			marker.pose.position.x = lines[i].x();
			marker.pose.position.y = lines[i].y();
			marker.pose.position.z = lines[i].z();
			marker.color.r = line_marker.type == lane_nav_msgs::Marker::STRAIGHT ? 1.0 : 0.0;
			marker.color.g = line_marker.type == lane_nav_msgs::Marker::LEFT ? 1.0 : 0.0;
			marker.color.b = line_marker.type == lane_nav_msgs::Marker::RIGHT ? 1.0 : 0.0;
			vis_pub.publish(marker);

			marker.id = i+1;
			marker.pose.position.x = lines[i+1].x();
			marker.pose.position.y = lines[i+1].y();
			marker.pose.position.z = lines[i+1].z();
			marker.color.r = line_marker.type == lane_nav_msgs::Marker::STRAIGHT ? 1.0 : 0.0;
			marker.color.g = line_marker.type == lane_nav_msgs::Marker::LEFT ? 1.0 : 0.0;
			marker.color.b = line_marker.type == lane_nav_msgs::Marker::RIGHT ? 1.0 : 0.0;
			vis_pub.publish(marker);
		}
	}

	marker_pub.publish(markers);
}

void doSynthesis() {
	cv_bridge::CvImageConstPtr im = cv_bridge::toCvShare(img_raw,
			sensor_msgs::image_encodings::BGR8);

	std::vector<cv::Vec4i> lines;
	findLines(im->image, lines);

	std::vector<tf::Vector3> lidarPts;
	projectImageToLidar(lines, lidarPts);

	adjustLidarScan(lidarPts);

	sendMarkers( lidarPts );
}

void checkCanUpdate() {
	if (recvCam && recvLidar) {
		ros::Duration timeDiff = img_raw->header.stamp
				- lidar_raw->header.stamp;
		if (fabs(timeDiff.toSec()) < msgWindow.get()) {
			doSynthesis();
		} else {
			ROS_ERROR(
					"Image and lidar scan not received within window. (%f s)", msgWindow.get());
		}
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	if (disabled.get()) {
		return;
	}

	img_raw = msg;
	recvCam = true;
	checkCanUpdate();
}

void lidarCallback(const sensor_msgs::LaserScanConstPtr& msg) {
	lidar_raw = msg;
	recvLidar = true;
	checkCanUpdate();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "lane");
	ros::NodeHandle nh("~");
	//image_transport::ImageTransport it(nh);

	tl = new tf::TransformListener();

	/*cv::namedWindow("raw", 0);
	 cv::namedWindow("gray", 0);
	 cv::namedWindow("left", 0);
	 cv::namedWindow("right", 0);*/

	height = 1.29;   // Camera height

	gp = new camera_tools::GroundProjection(nh, "/front/camera_info", "/amos");

    markerFrame.init(&nh, "markerFrame", "/world");

    lidarFixRange.init(&nh, "lidarFixRange", 10);

	msgWindow.init(&nh, "msgWindow", 0.2);

	ksize.init(&nh, "ksize", 10);
	sigma.init(&nh, "sigma", 2.36);

	lower.init(&nh, "lower", 30);

	houghRho.init(&nh, "houghRho", 1.0);
	houghTheta.init(&nh, "houghTheta", 0.1);
	houghThreshold.init(&nh, "houghThreshold", 38);
	houghMinLen.init(&nh, "houghMinLen", 5.0);
	houghMaxDist.init(&nh, "houghMaxDist", 7.5);

	showDebug.init(&nh, "showDebug", false);
	always_straight.init(&nh, "alwaysStraight", false);

	disabled.init(&nh, "disabled", false);

	straightAngle.init(&nh, "straightAngle", CV_PI*45.0/180.0);

	//Wait for the transform from camera to lidar to become available.
	while (!tl->canTransform("/amos", "/camera", ros::Time(), 0) ||
			!tl->canTransform(markerFrame.get(), "/camera", ros::Time(), 0)) {

		ros::spinOnce();
		ROS_WARN("Lane: Waiting for transforms.");
        usleep(1000);
	}

	ros::Subscriber sLidar = nh.subscribe("/lidar_raw", 1, lidarCallback);

	//image_transport::Subscriber sCam = it.subscribe("/front", 1, imageCallback);
	ros::Subscriber sCam = nh.subscribe("/front/image", 1, imageCallback);

	vis_pub = nh.advertise<visualization_msgs::Marker>( "/visualization_marker", 10, true );
	marker.header.frame_id = markerFrame.get();
	marker.header.stamp = ros::Time();
	marker.ns = "lane";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.lifetime = ros::Duration(0.5);
	marker.scale.x = 0.08;
	marker.scale.y = 0.08;
	marker.scale.z = 0.08;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;

	pLidar = nh.advertise<sensor_msgs::LaserScan>("lidar", 1, false);
	marker_pub = nh.advertise<lane_nav_msgs::MarkerArray>("markers", 1, true);

	ros::spin();

	return 0;
}
