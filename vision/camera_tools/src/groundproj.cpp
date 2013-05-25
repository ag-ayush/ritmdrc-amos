/*
 * groundproj.cpp
 *
 *  Created on: Jun 1, 2012
 *      Author: derrick
 */

#include "camera_tools/groundproj.h"

namespace camera_tools {

GroundProjection::GroundProjection(ros::NodeHandle& nh,
		std::string camInfoTopic, std::string groundFrame) {

	sCamInfo = nh.subscribe(camInfoTopic, 1, &GroundProjection::camInfoCallback,
			this);

	targetFrame = groundFrame;
	newInfo = false;
}

GroundProjection::~GroundProjection() {
}

void GroundProjection::camInfoCallback(sensor_msgs::CameraInfoConstPtr msg) {
	camInfo = msg;
	newInfo = true;
}

bool GroundProjection::updateCamera() {
	tf::Matrix3x3 K;
	K[0][0] = camInfo->K[0];
	K[0][1] = camInfo->K[1];
	K[0][2] = camInfo->K[2];

	K[1][0] = camInfo->K[3];
	K[1][1] = camInfo->K[4];
	K[1][2] = camInfo->K[5];

	K[2][0] = camInfo->K[6];
	K[2][1] = camInfo->K[7];
	K[2][2] = camInfo->K[8];

	invK = K.inverse();

	tf::StampedTransform camTf;

	tf::Vector3 camPt(0, 0, 0);

	try {
		tl.lookupTransform(camInfo->header.frame_id, targetFrame, ros::Time(), camTf);
	} catch (tf::TransformException& e) {
		ROS_ERROR("GP TF: %s", e.what());
		return false;
	}

	tf::Vector3 camOrigin = camTf.inverse() * camPt;

	tf::Vector3 gndOrigin(0, 0, 0);
	tf::Vector3 gndNorm(0, 0, 1);

	tf::Vector3 gcOrigin = camTf * gndOrigin;
	tf::Vector3 gcNorm = camTf * gndNorm;

	norm = gcNorm - gcOrigin;

	newInfo = false;

	return true;
}

tf::Vector3 GroundProjection::project(int x, int y, double height) {

	tf::Vector3 groundPt(0, 0, 0);

	if (!camInfo) {
		return groundPt;
	}

	if (newInfo) {
		if (!updateCamera()) {
			return groundPt;
		}
	}

	tf::Vector3 px(x, y, 1); //Pixel in the image
	tf::Vector3 p = invK * px; //Map image pixel to world coordinate
	p = p.normalize();

	tf::Vector3 gp = (-height / norm.dot(p)) * p; //Intersect "p" ray with ground plane

	return gp;
}

}
