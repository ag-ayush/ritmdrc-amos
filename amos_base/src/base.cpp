/*
 * File:    base.cpp
 * Author:  James Letendre
 *
 * Class to represent the base control of AMOS
 */

#include "base.h"

// Constructors/Destructor
Base::Base()
    : nh("~"), tf_listen(), tf_broadcast()
{
    // read size
    nh.param<double>("length", size.x, 1.0);
    nh.param<double>("width",  size.y, 1.0);
    nh.param<double>("height", size.z, 1.0);

    // read speed constraints
    nh.param<double>("maxSpeedFwd", maxFwd, 1.0);
    nh.param<double>("maxSpeedRev", maxRev, 1.0);

    // frames
    nh.param<std::string>("globalFrame", globalFrame, "/world");
    nh.param<std::string>("mapFrame", mapFrame, "/map");
    nh.param<std::string>("robotFrame", robotFrame, "/amos");

    ROS_INFO("%f, %f, %f", size.x, size.y, size.z );

    // subscribers
//    powerSub = nh.subscribe( "power",   1, &Base::powerSubCallback, this );
    odomSub  = nh.subscribe( "odom",    1, &Base::odomSubCallback, this );
    moveSub  = nh.subscribe( "moveCmd", 1, &Base::moveSubCallback, this );

    // publishers
//    powerReqPub = nh.advertise<std_msgs::Empty>( "powerReq", 1 );
    odomReqPub   = nh.advertise<std_msgs::Empty>( "odomReq",  1 );
    setSpeedPub  = nh.advertise<base_msgs::setSpeed>( "setSpeed", 1 );
    setAccelPub  = nh.advertise<base_msgs::setAccel>( "setAccel", 1 );
    setPIDPub    = nh.advertise<base_msgs::setPid>( "setPID", 1 );
    robotOdomPub = nh.advertise<nav_msgs::Odometry>( "robotOdom", 1 );


    // set initial messages
    setSpeedMsg.left  = 0;
    setSpeedMsg.right = 0;
    setSpeedPub.publish(setSpeedMsg);

    setAccelMsg.left =  0;
    setAccelMsg.right = 0;
    setAccelPub.publish(setAccelMsg);

    double p, i, d;
    nh.param<double>("MotorP", p, 0.091);
    nh.param<double>("MotorI", i, 0.0025);
    nh.param<double>("MotorD", d, 0.018);

    setPIDMsg.p = p;
    setPIDMsg.i = i;
    setPIDMsg.d = d;
    setPIDPub.publish(setPIDMsg);

    // set initial position
    odometry.header.frame_id = mapFrame;
    odometry.child_frame_id = robotFrame;

    odometry.pose.pose.position.x = 0.0;
    odometry.pose.pose.position.y = 0.0;
    odometry.pose.pose.position.z = 0.0;

    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw( 0.0 );
    
    // set initial velocity
    odometry.twist.twist.linear.x = 0.0;
    odometry.twist.twist.linear.y = 0.0;
    odometry.twist.twist.linear.z = 0.0;

    odometry.twist.twist.angular.x = 0.0;
    odometry.twist.twist.angular.y = 0.0;
    odometry.twist.twist.angular.z = 0.0;

    robotOdomPub.publish(odometry);

    // publish with tf
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(odometry.pose.pose.orientation, quaternion);
    tf::Transform transform( 
            quaternion,
            tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z )
            );

    tf_broadcast.sendTransform( tf::StampedTransform( transform, ros::Time::now(), globalFrame, robotFrame ) );
    tf_broadcast.sendTransform( tf::StampedTransform( transform, ros::Time::now(), globalFrame, mapFrame ) );
    tf_broadcast.sendTransform( tf::StampedTransform( transform, ros::Time::now(), mapFrame, robotFrame ) );
}

Base::~Base() 
{
    // stopping would be a good thing to do here
    setSpeedMsg.left  = 0.0;
    setSpeedMsg.right = 0.0;
    setSpeedPub.publish(setSpeedMsg);

    ros::spinOnce();
}

void Base::loop( )
{
    ros::Rate rate( 50 );

    while(ros::ok())
    {
        ros::spinOnce();

        setSpeedPub.publish(setSpeedMsg);
        
        odomReqPub.publish(odomReqMsg);
//        powerReqPub.publish(powerReqMsg);
        
        // send odometry
        robotOdomPub.publish(odometry);

        // publish with tf
        tf::Quaternion quaternion;
        tf::quaternionMsgToTF(odometry.pose.pose.orientation, quaternion);
        tf::Transform transform( 
                quaternion,
                tf::Vector3(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z )
                );

        tf_broadcast.sendTransform( tf::StampedTransform( transform, ros::Time::now(), mapFrame, robotFrame ) );

        rate.sleep();
    }
}

void Base::updateOdometry( double leftDist, double rightDist, double l_dt, double r_dt )
{
    // compute current velocity
    double leftVel = leftDist / l_dt;
    double rightVel = rightDist / r_dt;

    // yaw vel is (vr-vl)/width
    odometry.twist.twist.angular.z = (rightVel-leftVel)/size.y;

    // forward distance traveled is average * sin/cos(yaw)
    odometry.twist.twist.linear.x = (rightVel + leftVel)*cos(tf::getYaw(odometry.pose.pose.orientation))/2.0;
    odometry.twist.twist.linear.y = (rightVel + leftVel)*sin(tf::getYaw(odometry.pose.pose.orientation))/2.0;

    // accumulate orientation
    odometry.pose.pose.position.x += (rightDist + leftDist)*cos(tf::getYaw(odometry.pose.pose.orientation))/2.0;
    odometry.pose.pose.position.y += (rightDist + leftDist)*sin(tf::getYaw(odometry.pose.pose.orientation))/2.0;

    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw( 
            (rightDist-leftDist)/size.y +
            tf::getYaw(odometry.pose.pose.orientation)
            );
}

// message subscribers
// FROM BASE
/*
void Base::powerSubCallback( const base_msgs::PowerInfo::ConstPtr &msg )
{
    // ignore for now
}
*/

void Base::odomSubCallback( const base_msgs::motorOdom::ConstPtr &msg )
{
    // update odometry
    updateOdometry( msg->left, msg->right, msg->leftTime, msg->rightTime );
}

// FROM NAV STACK
void Base::moveSubCallback( const geometry_msgs::Twist::ConstPtr &msg )
{
    // compute wheel velocities
    double left, right;
    left  = msg->linear.x - msg->angular.z*size.y;
    right = msg->linear.x + msg->angular.z*size.y;

    // find scales if needed
    double leftScale = 1.0, rightScale = 1.0;
    if( left > maxFwd ) leftScale = maxFwd/left;
    if( left < maxRev ) leftScale = maxRev/left;

    if( right > maxFwd ) rightScale = maxFwd/right;
    if( right < maxRev ) rightScale = maxRev/right;

    // multiply by smallest scale
    if( rightScale < leftScale ) 
    {
        left *= rightScale;
        right *= rightScale;
    }
    else
    {
        left *= leftScale;
        right *= leftScale;
    }

    ROS_INFO("amos_base: SET SPEED left = %f, right = %f", left, right);

    // set new motor speed
    setSpeedMsg.left  = left;
    setSpeedMsg.right = right;
}

