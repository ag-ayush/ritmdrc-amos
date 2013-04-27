#include <WProgram.h>

#include "motor.h"
#include "comm.h"
#include <ros.h>

ros::NodeHandle_<ArduinoHardware,6,6,128,128> nh;
//ros::NodeHandle nh;
//ros::NodeHandle nh;

ros::Subscriber<base_msgs::setPid> pidSub("motorPid", comm_set_pid_handler);
ros::Subscriber<base_msgs::setSpeed> speedSub("motorSpeed", comm_set_speed_handler);
ros::Subscriber<base_msgs::setAccel> accelSub("motorAccel", comm_set_acceleration_handler);
ros::Subscriber<base_msgs::motorRst> resetSub("motorReset", comm_reset_handler);

ros::Subscriber<std_msgs::Empty> reqOdom("reqOdom", comm_get_odometry_handler);
ros::Subscriber<std_msgs::Empty> reqPower("reqPower", comm_get_power_handler);

base_msgs::motorOdom odom;
base_msgs::powerInfo power;
ros::Publisher odomPub("motorOdom", &odom);
ros::Publisher powerPub("motorPower", &power);

void comm_setup()
{
    nh.initNode();
    nh.subscribe(pidSub);
    nh.subscribe(speedSub);
    nh.subscribe(accelSub);
    nh.subscribe(resetSub);
    nh.subscribe(reqOdom);
    nh.subscribe(reqPower);

    nh.advertise(odomPub);
    nh.advertise(powerPub);

}

void comm_run()
{
    nh.spinOnce();
}

// handles reset command
void comm_reset_handler(const base_msgs::motorRst& msg)
{
    Motor::left.reset();
    Motor::right.reset();
}

// handles set motor speed command
void comm_set_speed_handler(const base_msgs::setSpeed& msg)
{
    Motor::left.setSpeed(msg.left);
    Motor::right.setSpeed(msg.right);
}


// handles set PID parameters
void comm_set_pid_handler(const base_msgs::setPid& msg)
{
    Motor::left.setPID(msg.p, msg.i, msg.d);
    Motor::right.setPID(msg.p, msg.i, msg.d);
}


// handles set acceleration limit command
void comm_set_acceleration_handler(const base_msgs::setAccel& msg)
{
    Motor::left.setAcceleration(msg.left);
    Motor::right.setAcceleration(msg.right);
}


// handles read odometry command
void comm_get_odometry_handler(const std_msgs::Empty& msg)
{
    Motor::left.getOdometry(&odom.left, &odom.leftTime);
    Motor::right.getOdometry(&odom.right, &odom.rightTime);    

    odomPub.publish(&odom);
}


// handles read power information command
void comm_get_power_handler(const std_msgs::Empty &msg)
{
    power.voltageLeft = Motor::left.getVoltage();
    power.currentLeft = Motor::left.getCurrent();
    power.voltageRight = Motor::right.getVoltage();
    power.currentRight = Motor::right.getCurrent();

    powerPub.publish(&power);
}

