/*
 * igvc2.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: derrick
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <igvc/Mode.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <tf/transform_listener.h>

#include <dynparam/doubleparam.h>
#include <dynparam/intparam.h>
#include <dynparam/boolparam.h>

#include <cmath>
#include <algorithm>

#include "common.h"

/** initialization functions **/
void initParams(ros::NodeHandle& nh);
void initPublishers(ros::NodeHandle& nh);
void initSubscribers(ros::NodeHandle& nh);
void initTimeouts();
void initWaypoints();

/** Mode iteration functions **/
void mainLoop();

void unknownIteration();
void laneIteration();
void gpsIteration();
void fenceIteration();
void gateIteration();
void alignIteration();

/** Common functions for switching to various modes **/
void startGPS(int nextMode);

/** Subscriber callbacks **/
void lidarCallback(const sensor_msgs::LaserScan& msg); //used for monitoring
void cameraCallback(const sensor_msgs::Image& msg);

void laneCallback(const geometry_msgs::PoseStamped& msg);

/** Functions to check status of sensors / nodes **/
int checkStatus(); //calls the other check functions to accumulate status

bool checkLidar();
bool checkCamera();

/** debug / speech functions **/
void announceMode();
void announceStatus();

/** Waypoint lists **/
nav_msgs::Path waypoints; //all points read from file
nav_msgs::Path entryPts; //ends of the lanes
nav_msgs::Path northPts; //pts north of fence
nav_msgs::Path southPts; //pts south of fence
nav_msgs::Path fencePts; //west and east ends of the fence

/** Last stamp time used for timing out sensors **/
ros::Time lastLidar;
ros::Time lastCamera;

/** Tuning parameters **/
DoubleParam entryThresh; //Min dist to switch to start_gps_* mode
DoubleParam navThresh; //Min dist to count waypoint as reached
DoubleParam angleThresh; //Epsilon used when orienting robot
DoubleParam forwardForce; //Constant force used in lane mode to travel forward

DoubleParam lidarTimeout; //Seconds it takes to fault lidar
DoubleParam cameraTimeout; //seconds it takes to fault camera

DoubleParam fenceThresh; //When we've arrived close to an end of the fence
DoubleParam gatePush; //How far past the fence we try and push amos.

BoolParam usePath;

BoolParam gotoDisabled; //Used to stop the robot
BoolParam laneDisabled; //Used to stop lane detection during gps portion
BoolParam stop; //software e-stop

IntParam mode; //stores current state id

/** Various poses used for determining location in various frames **/
geometry_msgs::PoseStamped amosLocal; //origin in amos-local frame
geometry_msgs::PoseStamped amosWorld; //position relative to starting GPS fix

/** Publishers **/
ros::Publisher navPub; //Where we should be headed
ros::Publisher pathPub;
ros::Publisher gotoPub; //Used when aligning to a heading
ros::Publisher speechPub; //Used for debug tts output

/** Subscribers **/
ros::Subscriber lidarSub; //used for monitoring
ros::Subscriber cameraSub; //used for monitoring

ros::Subscriber laneSub; //gets lane_nav goal in lane mode

/** Local vars **/
bool lidarFault;
bool cameraFault;

bool lastLeg; //Set when we start back into the lane and want to watch for end

double entryTheta;

int lastMode;
int status, lastStatus;

geometry_msgs::PoseStamped curGoal;

tf::TransformListener *tl;

void initParams(ros::NodeHandle& nh) {
	entryThresh.init(&nh, "entryThresh", 2.0); //Min dist to switch to start_gps_* mode
	navThresh.init(&nh, "/navThresh", 1.0); //Min dist to count waypoint as reached
	angleThresh.init(&nh, "angleThresh", 0.1); //Epsilon used when orienting robot
	forwardForce.init(&nh, "forwardForce", 0.3); //Constant force used in lane mode to travel forward

	lidarTimeout.init(&nh, "lidarTimeout", 0.5); //Seconds it takes to fault lidar
	cameraTimeout.init(&nh, "cameraTimeout", 0.5); //seconds it takes to fault camera

	gotoDisabled.init(&nh, "/goto/disabled", false); //Used to stop the robot
	laneDisabled.init(&nh, "/lane/disabled", false); //stop lane when in gps
	stop.init(&nh, "/stop", false); //software e-stop

    usePath.init(&nh, "usePath", true);

	fenceThresh.init(&nh, "fenceThresh", 3.0);
	gatePush.init(&nh, "gatePush", 0.35);

	mode.init(&nh, "mode", 0);
}

void initPublishers(ros::NodeHandle& nh) {
	navPub = nh.advertise<geometry_msgs::PoseStamped>("/avoid/goal", 1);
    pathPub = nh.advertise<nav_msgs::Path>("/path/plan", 1);
	gotoPub = nh.advertise<geometry_msgs::PoseStamped>("/goto/goal", 1);
	speechPub = nh.advertise<std_msgs::String>("/speech/tts", 10);
}

void initSubscribers(ros::NodeHandle& nh) {
	lidarSub = nh.subscribe("/lidar_raw", 1, lidarCallback);
	cameraSub = nh.subscribe("/front/image", 1, cameraCallback);

	laneSub = nh.subscribe("/laneGoal", 1, laneCallback);
}

void initTimeouts() {
	lastLidar = ros::Time::now();
	lastCamera = ros::Time::now();
}

void initWaypoints() {
	entryPts.poses.push_back(waypoints.poses[0]);
	entryPts.poses.push_back(waypoints.poses[7]);

	for (int i = 0; i < 3; ++i) {
		southPts.poses.push_back(waypoints.poses[i + 1]);
		northPts.poses.push_back(waypoints.poses[i + 4]);
	}

    fencePts.poses.push_back(waypoints.poses[8]);
    fencePts.poses.push_back(waypoints.poses[9]);
}

int checkStatus() {
	int numFault = 0;

	lidarFault = checkLidar();
	cameraFault = checkCamera();

	if (lidarFault) {
		++numFault;
	}

	if (cameraFault) {
		++numFault;
	}

	return numFault;
}

bool checkLidar() {
	ros::Duration delta = ros::Time::now() - lastLidar;
	if (delta.toSec() > lidarTimeout.get()) {
		return true;
	}

	return false;
}

bool checkCamera() {
	ros::Duration delta = ros::Time::now() - lastCamera;
	if (delta.toSec() > cameraTimeout.get()) {
		return true;
	}

	return false;
}

void announceStatus() {
	static bool lastLidarFault, lastCameraFault;

	if (lidarFault != lastLidarFault && lidarFault) {
		speak(speechPub, "lost lidar");
	} else if (lidarFault != lastLidarFault) {
		speak(speechPub, "lidar resumed");
	}

	if (cameraFault != lastCameraFault && cameraFault) {
		speak(speechPub, "lost camera");
	} else if (cameraFault != lastCameraFault) {
		speak(speechPub, "camera resumed");
	}

	lastLidarFault = lidarFault;
	lastCameraFault = cameraFault;
}

void lidarCallback(const sensor_msgs::LaserScan& msg) {
	lastLidar = msg.header.stamp;
}

void cameraCallback(const sensor_msgs::Image& msg) {
	lastCamera = msg.header.stamp;
}

void laneCallback(const geometry_msgs::PoseStamped& msg) {
	//Make sure we're still in lane mode
	if (mode.get() != igvc::Mode::LANE) {
		return;
	}

	//Put the goal in local space to make it trivial to add the forward force
	geometry_msgs::PoseStamped goalLocal;
    geometry_msgs::PoseStamped goalMsg = msg;
    goalMsg.header.stamp = ros::Time();

    try {
        geometry_msgs::PoseStamped pMsg = msg;
        pMsg.header.stamp = ros::Time();
    	tl->transformPose("/amos", pMsg, goalLocal);
    } catch (tf::TransformException& e) {
        ROS_ERROR("igvc: %s", e.what());
        return;
    }

	goalLocal.pose.position.x += forwardForce.get();

	if (usePath.get()) {
        try{
            nav_msgs::Path p;
            geometry_msgs::PoseStamped goalWorld;
            goalLocal.header.stamp = ros::Time();
            tl->transformPose("/map", goalLocal, goalWorld);
            p.poses.push_back(goalWorld);

            pathPub.publish(p);
        }
        catch( tf::TransformException &e )
        {
            ROS_ERROR("IGVC: %s", e.what());
        }
    } else {
        navPub.publish(goalLocal);
    }
}

void announceMode() {
	switch (mode.get()) {
	case igvc::Mode::UNKNOWN:
		speak(speechPub, "i g v c initializing");
		break;

	case igvc::Mode::LANE:
		speak(speechPub, "i g v c lane mode");
		break;

	case igvc::Mode::START_GPS_CW:
	case igvc::Mode::START_GPS_CCW:
		speak(speechPub, "i g v c leaving lane");
		break;

	case igvc::Mode::GPS_CW_1:
	case igvc::Mode::GPS_CW_2:
	case igvc::Mode::GPS_CCW_1:
	case igvc::Mode::GPS_CCW_2:
		speak(speechPub, "i g v c  g p s mode");
		break;

	case igvc::Mode::FENCE_CW:
	case igvc::Mode::FENCE_CCW:
		speak(speechPub, "i g v c looking for fence");
		break;

	case igvc::Mode::END_GPS_CW:
	case igvc::Mode::END_GPS_CCW:
		speak(speechPub, "i g v c returning to lane");
		break;

	case igvc::Mode::ALIGN_LANE:
		speak(speechPub, "i g v c aligning to lane");
		break;
	}
}

void mainLoop() {
	ros::Rate rate(50);

    ROS_WARN("Waiting for transform");
    tl->waitForTransform("/world", "/amos", ros::Time(), ros::Duration(10));

	amosLocal.header.frame_id="/amos";
	amosLocal.pose.orientation.w = 1;

	status = 0;
	lastStatus = 0;

	lastMode = -1; //set mode to announce when first running

	lastLeg = false;

	//make sure everything is ready to rock and roll!
    ROS_WARN("Killing all disables");
	laneDisabled.set(false);
	gotoDisabled.set(false);
	stop.set(false);

	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();

		//Get amos' position in the waypt frame
		amosLocal.header.stamp = ros::Time();
		tl->transformPose("/world", amosLocal, amosWorld);

        if (mode.get() != lastMode) {
            announceMode();
        }

        int status = checkStatus();

        if (status != lastStatus) {
            stop.set(true);
            announceStatus();
        }

        if (!status) {
        	if (lastStatus) {
        		stop.set(false);
        	}

			switch (mode.get()) {
				case igvc::Mode::UNKNOWN:
					unknownIteration();
					break;

				case igvc::Mode::LANE:
					laneIteration();
					break;

					//The next modes all involve navigating to a gps point
				case igvc::Mode::START_GPS_CW:
				case igvc::Mode::START_GPS_CCW:
				case igvc::Mode::GPS_CW_1:
				case igvc::Mode::GPS_CW_2:
				case igvc::Mode::GPS_CCW_1:
				case igvc::Mode::GPS_CCW_2:
				case igvc::Mode::END_GPS_CW:
				case igvc::Mode::END_GPS_CCW:
					gpsIteration();
					break;

				case igvc::Mode::FENCE_CW:
				case igvc::Mode::FENCE_CCW:
					fenceIteration();
					break;

				case igvc::Mode::ALIGN_LANE:
					alignIteration();
					break;
			}
        }

        lastMode = mode.get();
        lastStatus = status;
	}
}

void startGPS(int nextMode) {
	mode.set(nextMode);
	laneDisabled.set(true);
	entryTheta = tf::getYaw(amosWorld.pose.orientation);

	//if going CCW, reverse middle waypoints order
	if (nextMode == igvc::Mode::START_GPS_CCW) {
		std::reverse(northPts.poses.begin(), northPts.poses.end());
		std::reverse(southPts.poses.begin(), southPts.poses.end());
	}
}

void unknownIteration() {
		mode.set(igvc::Mode::LANE);
}

void laneIteration() {
	//Check if we're on our return trip and have arrived
	if (lastLeg) {
		double dist = hypot(amosWorld.pose.position.x, amosWorld.pose.position.y);
		if (dist < entryThresh.get()) {
			speak(speechPub, "i g v c where is my money");
			gotoDisabled.set(true);
			return;
		}
	}

	//get distance to entryPts
	geometry_msgs::PoseStamped ptCW, ptCCW;
    
    try {
        tl->transformPose("/amos", entryPts.poses[0], ptCW);
        tl->transformPose("/amos", entryPts.poses[1], ptCCW);

        double distCW = hypot(ptCW.pose.position.x, ptCW.pose.position.y);
        double distCCW = hypot(ptCCW.pose.position.x, ptCCW.pose.position.y);

        if (distCW < entryThresh.get()) {
            startGPS(igvc::Mode::START_GPS_CW);
            curGoal = entryPts.poses[0];
            return;
        }

        if (distCCW < entryThresh.get()) {
            startGPS(igvc::Mode::START_GPS_CCW);
            curGoal = entryPts.poses[1];
            return;
        }
    } catch (tf::TransformException& e) {
        ROS_ERROR("igvc: %s", e.what());
    }

	//We haven't switched to  gps mode, but the rest of lane handling happens
	//in the lane callback.
}

void gpsIteration() {
	//Navigating to a waypoint. See if we've arrived.
	geometry_msgs::PoseStamped goalLocal;
	tl->transformPose("/amos", curGoal, goalLocal);

	double dist = hypot(goalLocal.pose.position.x, goalLocal.pose.position.y);
	if (dist < navThresh.get()) {
		//We've arrived. Update to next goal/mode based on current state
		switch (mode.get()) {
		case igvc::Mode::START_GPS_CW:
			mode.set(igvc::Mode::GPS_CW_1);
			curGoal = southPts.poses[0];
			break;

		case igvc::Mode::START_GPS_CCW:
			mode.set(igvc::Mode::GPS_CCW_1);
			curGoal = northPts.poses[0];
			break;

		case igvc::Mode::GPS_CW_1:
			southPts.poses.erase(southPts.poses.begin());
			if (southPts.poses.size() == 0) {
				mode.set(igvc::Mode::FENCE_CW);
				break;
			}

			curGoal = southPts.poses[0];
			break;

		case igvc::Mode::GPS_CCW_1:
			northPts.poses.erase(northPts.poses.begin());
			if (northPts.poses.size() == 0) {
				mode.set(igvc::Mode::FENCE_CCW);

				break;
			}

			curGoal = northPts.poses[0];
			break;

		case igvc::Mode::GPS_CW_2:
			northPts.poses.erase(northPts.poses.begin());
			if (northPts.poses.size() == 0) {
				mode.set(igvc::Mode::END_GPS_CW);
				curGoal = entryPts.poses[1];
				break;
			}

			curGoal = northPts.poses[0];
			break;

		case igvc::Mode::GPS_CCW_2:
			southPts.poses.erase(southPts.poses.begin());
			if (southPts.poses.size() == 0) {
				mode.set(igvc::Mode::END_GPS_CCW);
				curGoal = entryPts.poses[0];
				break;
			}

			curGoal = southPts.poses[0];
			break;

		case igvc::Mode::END_GPS_CW:
		case igvc::Mode::END_GPS_CCW:
			mode.set(igvc::Mode::ALIGN_LANE);
			return;
		}
	}

	navPub.publish(curGoal);
}

void fenceIteration() {
	//Try to get to the west end of the fence (closer to the tent)

	//Navigating to a waypoint. See if we've arrived.
	geometry_msgs::PoseStamped goalLocal;
	tl->transformPose("/amos", fencePts.poses[0], goalLocal);

	double dist = hypot(goalLocal.pose.position.x, goalLocal.pose.position.y);

	if (dist < fenceThresh.get()) {
		if (mode.get() == igvc::Mode::FENCE_CW) {
			mode.set(igvc::Mode::GATE_CW);
		} else {
			mode.set(igvc::Mode::GATE_CCW);
		}

		return;
	}

	navPub.publish(fencePts.poses[0]);
}

void gateIteration() {
	//We want to follow the fence east, looking for the gate

	//see if we've crossed through the fence
	if (mode.get() == igvc::Mode::GATE_CW) {
		if (amosWorld.pose.position.x < fencePts.poses[0].pose.position.x - fenceThresh.get()) {
			mode.set(igvc::Mode::GPS_CW_2);
			curGoal = northPts.poses[0];
			return;
		}
	} else {
		if (amosWorld.pose.position.x > fencePts.poses[0].pose.position.x + fenceThresh.get()) {
			mode.set(igvc::Mode::GPS_CCW_2);
			curGoal = southPts.poses[0];
			return;
		}
	}

	//Navigating to a waypoint. See if we've arrived.
	geometry_msgs::PoseStamped goalLocal;
	tl->transformPose("/amos", fencePts.poses[1], goalLocal);

	double dist = hypot(goalLocal.pose.position.x, goalLocal.pose.position.y);

	if (dist < fenceThresh.get()) {
		if (mode.get() == igvc::Mode::GATE_CW) {
			mode.set(igvc::Mode::FENCE_CW);
		} else {
			mode.set(igvc::Mode::FENCE_CCW);
		}

		return;
	}

	double theta = atan2(goalLocal.pose.position.y, goalLocal.pose.position.x);
	if (mode.get() == igvc::Mode::GATE_CW) {
		theta -= gatePush.get();
	} else {
		theta += gatePush.get();
	}

	goalLocal.pose.position.x = cos(theta) * dist;
	goalLocal.pose.position.y = sin(theta) * dist;

	navPub.publish(fencePts.poses[1]);
}

void alignIteration() {
	double dTheta = entryTheta - tf::getYaw(amosWorld.pose.orientation);
	while (dTheta < -M_PI) dTheta += 2 * M_PI;
	while (dTheta > M_PI) dTheta -= 2 * M_PI;

	if (fabs(dTheta) < angleThresh.get()) {
		laneDisabled.set(false);

		//Mark that we're on the last leg of the course. If we manage to get
		//our starting position... make money bitches!
		lastLeg = true;
		mode.set(igvc::Mode::LANE);
	}

	geometry_msgs::PoseStamped turnGoal;
	turnGoal.header.frame_id = "/amos"; //turning carrot

	turnGoal.pose.position.y = dTheta > 0 ? 1 : -1;
	turnGoal.pose.orientation.w = 1;

	navPub.publish(turnGoal);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "igvc");
	ros::NodeHandle nh("~");

	tl = new tf::TransformListener();

    ROS_WARN("IGVC initializing");

	initParams(nh);
    initPublishers(nh);
    initSubscribers(nh);

	//We will possibly be republishing some sensor data, so we want to run at the typical sensor rate.
	ros::Rate rate(50);

	std::string waypt_file, waypt_frame;
	nh.param < std::string > ("waypoint_file", waypt_file, "waypoints/waypoints.txt");
	nh.param<std::string>("waypoint_frame", waypt_frame, "/world");

	if (!loadWaypoints(tl, waypt_file, waypt_frame, waypoints)) {
		ROS_ERROR("Unable to read waypoint file %s", waypt_file.c_str());
		return 1;
	}

	if (waypoints.poses.size() < 2) {
		ROS_ERROR("Not enough waypoints read in from file %s", waypt_file.c_str());
		return 1;
	}

	initWaypoints();

	mainLoop();

	return 0;
}
