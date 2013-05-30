/*
 * common.h
 *
 *  Created on: Jun 8, 2012
 *      Author: derrick
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

/**
 * Loads waypoints into a path from a file. Points will be stamped with the specified frame and all have
 * an orientation of 0.
 *
 * @param filename The name of the file to load
 * @param frame The transform frame to stamp the waypoints with.
 * @param path The path instance to store the points in. Existing points will be cleared from the path.
 * @return true if reading from the file succeeded, false if there was a problem opening the file.
 */
bool loadWaypoints(tf::TransformListener *tl, std::string filename, std::string frame,
		nav_msgs::Path& path);


inline double dist(geometry_msgs::Point& a, geometry_msgs::Point& b) {
	return hypot(a.x-b.x, a.y-b.y);
}

void speak(ros::Publisher& pub, std::string text);

#endif /* COMMON_H_ */
