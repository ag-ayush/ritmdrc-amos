/*
 * common.cpp
 *
 *  Created on: Jun 8, 2012
 *      Author: derrick
 */

#include <gps_convert/LocalWorldConvert.h>
#include "common.h"
#include <fstream>

bool loadWaypoints(tf::TransformListener *tl, std::string filename, std::string frame,
		nav_msgs::Path &path) {

    tl->waitForTransform("/world", frame, ros::Time(), ros::Duration(10));
    
	std::fstream file(filename.c_str());
	if (!file) {
		return false;
	}

	path.poses.clear();

	while (file) {
		double x, y;
		file >> x >> y;
		if (file) {
			geometry_msgs::PoseStamped pt;
			pt.pose.position.x = x;
			pt.pose.position.y = y;
			pt.pose.orientation.w = 1;

            gps_convert::LocalWorldConvert gpsConvert;
            gpsConvert.request.pose = pt;
            if( !ros::service::call("/gps/convert", gpsConvert) )
            {
                ROS_ERROR("IGVC: Can't convert waypoint during loading");
            }

            gpsConvert.response.pose.header.frame_id = "/world";

            tl->transformPose(frame, gpsConvert.response.pose, pt);
            pt.header.stamp = ros::Time();
			path.poses.push_back(pt);
		}
	}

	return true;
}

void speak(ros::Publisher& pub, std::string text) {
    std_msgs::String tts;
    tts.data = text;
    pub.publish(tts);
}
