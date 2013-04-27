/*
 * File:    base.h
 * Author:  James Letendre
 *
 * Class to represent the base control of AMOS
 */
#ifndef AMOS_BASE_H
#define AMOS_BASE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Empty.h>

#include <base_msgs/setSpeed.h>
#include <base_msgs/motorOdom.h>
#include <base_msgs/setAccel.h>
#include <base_msgs/setPid.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <nav_msgs/Odometry.h>


class Base {

    public:
        // Constructors/Destructor
        Base();
        ~Base();

        // Main loop
        void loop();

    private:
        // Node handle
        ros::NodeHandle nh;

        // TF instances
        tf::TransformListener tf_listen;
        tf::TransformBroadcaster tf_broadcast;

        // message subscribers
        // FROM BASE
//        ros::Subscriber powerSub;
//        void powerSubCallback( const base_msgs::PowerInfo::ConstPtr &msg );

        ros::Subscriber odomSub;
        void odomSubCallback( const base_msgs::motorOdom::ConstPtr &msg );
        
        // FROM NAV STACK
        ros::Subscriber moveSub;
        void moveSubCallback( const geometry_msgs::Twist::ConstPtr &msg );

        // message publishers               message to publish
        // TO BASE
//        ros::Publisher powerReqPub;         std_msgs::Empty powerReqMsg;
        ros::Publisher odomReqPub;          std_msgs::Empty odomReqMsg;
        ros::Publisher setSpeedPub;         base_msgs::setSpeed setSpeedMsg;
        ros::Publisher setAccelPub;         base_msgs::setAccel setAccelMsg;
        ros::Publisher setPIDPub;           base_msgs::setPid setPIDMsg;

        // Current robot position
        ros::Publisher robotOdomPub;        nav_msgs::Odometry odometry;

        // Robot size
        geometry_msgs::Vector3 size;

        // Frame names
        std::string globalFrame, mapFrame, robotFrame;

        // speed constraints
        double maxFwd, maxRev;

        // update odometry
        void updateOdometry( double leftDist, double rightDist, double l_dt, double r_dt );
};

#endif
