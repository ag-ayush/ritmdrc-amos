/*
 * qual.cpp
 *
 *  Created on: Jun 8, 2012
 *      Author: derrick
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <dynparam/doubleparam.h>
#include <dynparam/intparam.h>
#include <dynparam/boolparam.h>

#include <igvc/Mode.h>
#include "common.h"

#include <tf/transform_listener.h>

#include <fstream>

#include <cmath>

//The list of waypoints used in the gps course. After reading them in,
nav_msgs::Path waypoints;

int main(int argc, char** argv) {
	ros::init(argc, argv, "qual");
	ros::NodeHandle nh("~");

    ros::Publisher speechPub = nh.advertise<std_msgs::String>("/speech/tts", 1);
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("path", 1);

	tf::TransformListener tl;

	std::string waypt_file, waypt_frame;
    waypt_frame = "/world";

	nh.param<std::string>("waypoint_file", waypt_file,
			"waypoints/waypoints.txt");

	if (!loadWaypoints(&tl, waypt_file, waypt_frame, waypoints)) {
		ROS_ERROR("Unable to read waypoint file %s", waypt_file.c_str());
		return 1;
	}

	if (waypoints.poses.size() < 2) {
		ROS_ERROR(
				"Not enough waypoints read in from file %s", waypt_file.c_str());
		return 1;
	}

	DoubleParam navThresh;
	navThresh.init(&nh, "/navThresh", 1.0);

//	ros::Publisher wayptPub = nh.advertise<geometry_msgs::PoseStamped>("/apf/goal", 1);
	ros::Publisher wayptPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

	wayptPub.publish(waypoints.poses[0]);

    BoolParam gotoDisabled;
    gotoDisabled.init(&nh, "/goto/disabled", false);

	ros::Rate rate(10);

	geometry_msgs::PoseStamped amosLocal, amosWorld, amosMap;

	amosLocal.pose.orientation.w = 1;
	amosLocal.header.frame_id = "/amos";

    tl.waitForTransform("/amos", "/world", ros::Time(), ros::Duration(10));

    gotoDisabled.set(false);

	while (ros::ok()) {

		tl.transformPose("/world", amosLocal, amosWorld);

		if (dist(waypoints.poses[0].pose.position, amosWorld.pose.position)
				< navThresh.get()) {
			ROS_INFO("qual: reached waypoint");
            speak(speechPub, "qualification reached waypoint");

			waypoints.poses.erase(waypoints.poses.begin());
			if (waypoints.poses.size() == 0) {
				ROS_INFO("qual: exhausted waypoints. finished");

                gotoDisabled.set(true);

                speak(speechPub, "qualification finished");
				break;
			}
		}

		wayptPub.publish(waypoints.poses[0]);
		pathPub.publish(waypoints);

		ros::spinOnce();
		rate.sleep();
	}

	ros::spinOnce(); //Make sure all the ros things are cleared out
	sleep(1);
}

