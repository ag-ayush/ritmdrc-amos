/*
 * groundproj.h
 *
 *  Created on: Jun 1, 2012
 *      Author: derrick
 */

#ifndef GROUNDPROJ_H_
#define GROUNDPROJ_H_

#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

namespace camera_tools {

/**
 * Handles projecting pixels from a camera onto the ground plane.
 */
class GroundProjection {
public:
	/**
	 * Initializes the GroundProjection instance
	 * @param nh The NodeHandle instance used to register the camera info callback
	 * @param camNamespace The camera namespace that contains the camera_info topic. IE if the
	 * camera_info topic is /camera/camera_info, specify /camera
	 * @param groundFrame The frame id containing the ground plane to project a point onto.
	 */
	GroundProjection(ros::NodeHandle& nh, std::string camNamespace,
			std::string groundFrame);
	~GroundProjection();

	/**
	 * Performs projection of a camera pixel onto the ground plane.
	 * @param x The x coordinate of the pixel in the image to project
	 * @param y The y coordinate of the pixel in the image to project
	 * @return The point that lies on the ground plane. NOTE: This point is left in camera space
	 * and is stamped with the same frame id as the camera info topic. It is up to the calling
	 * program to transform the point into the appropriate frame.
	 */
	tf::Vector3 project(int x, int y, double height);

private:
	void camInfoCallback(sensor_msgs::CameraInfoConstPtr msg);
	bool updateCamera();

	sensor_msgs::CameraInfoConstPtr camInfo;
	ros::Subscriber sCamInfo;

	tf::TransformListener tl;

	tf::Vector3 norm; //Ground normal in camera space
	tf::Matrix3x3 invK; //Inverse of the intrinsic camera parameters


	std::string targetFrame;

	bool newInfo;
};

}

#endif /* GROUNDPROJ_H_ */
