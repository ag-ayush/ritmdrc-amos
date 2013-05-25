#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

int main(int argc, char** argv){
  ros::init(argc, argv, "amos_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    // Lidar
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.44)),
        ros::Time::now(),"amos", "laser"));

    // L wheel
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.3625, 0.0)),
        ros::Time::now(),"amos", "lWheel"));

    // R wheel
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, -0.3625, 0.0)),
        ros::Time::now(),"amos", "rWheel"));

    // camera
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::createQuaternionFromRPY(-M_PI/2.0-atan2( 52, 51 ), 0, -M_PI/2.0), tf::Vector3(-0.09, 0.0, 1.29)),
        ros::Time::now(),"amos", "camera"));


    r.sleep();
  }
}
