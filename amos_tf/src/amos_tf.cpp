#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "amos_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(10);

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener(n);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "/amos";
  pose.header.stamp = ros::Time();

  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>( "odom", 1 );

  // initial world transform
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"map", "world"));

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

    // lookup /map->/amos and publish as Odometry msg
    geometry_msgs::PoseStamped amos_pose;
    try
    {
        listener.transformPose( "/map", pose, amos_pose );

        nav_msgs::Odometry odom;
        odom.header.frame_id = "/map";
        odom.header.stamp = ros::Time::now();
        odom.child_frame_id = "/amos";

        odom.pose.pose = amos_pose.pose;

        odom_pub.publish(odom);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("amos_tf: Can't find /map -> /amos transform");
    }

    r.sleep();
  }
}
