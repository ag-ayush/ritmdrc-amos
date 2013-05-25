#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <camera_tools/groundproj.h>
#include <visualization_msgs/Marker.h>

#include <cstdlib>
#include <iostream>

void usage(char *ex) {
	ROS_ERROR("Usage: %s <Global Frame> <Camera>", ex);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "gp_test");
	ros::NodeHandle nh("~");

	tf::TransformListener tl;

	if (argc != 3) {
		usage(argv[0]);
		return 1;
	}

	ros::Publisher pMarker = nh.advertise<visualization_msgs::Marker>( "/visualization_marker", 10);

	std::cout << "Getting arguments" << std::endl;

	std::string globalFrame(argv[1]);
	std::string camera(argv[2]);
	std::string infoTopic = camera + "/camera_info";
	int x, y;

	std::cout << "Instantiating ground projection" << std::endl;

	camera_tools::GroundProjection gp(nh, infoTopic, globalFrame);

	visualization_msgs::Marker marker;
	marker.header.frame_id = globalFrame;
	marker.header.stamp = ros::Time();
	marker.ns = "gp_test";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = .1;
	marker.scale.y = .1;
	marker.scale.z = .1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.lifetime = ros::Duration();

	std::cout << "Giving ROS a second..." << std::endl;

	sleep(1);
	ros::spinOnce();

	while(ros::ok() && std::cin.good()) {
		std::cout << " X Y (ctrl+d to quit) >" << std::flush;

		std::cin >> std::ws >> x;
		if (!std::cin.good()) {
			break;
		}

		std::cin >> std::ws >> y;
		if (!std::cin.good()) {
			break;
		}

		tf::StampedTransform tf;
		tl.lookupTransform(globalFrame, "/camera", ros::Time(), tf);

		tf::Vector3 pt = gp.project(x, y);
		tf::Vector3 ptG = tf * pt;

		printf("\n(%f, %f, %f)\n", pt.x(), pt.y(), pt.z());
		printf("(%f, %f, %f)\n\n", ptG.x(), ptG.y(), ptG.z());

		marker.pose.position.x = ptG.x();
		marker.pose.position.y = ptG.y();
		marker.pose.position.z = ptG.z();
		pMarker.publish( marker );

		ros::spinOnce();
	}

	std::cout << std::endl;
}
