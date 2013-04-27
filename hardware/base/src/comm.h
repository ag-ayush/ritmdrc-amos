#include <base_msgs/setPid.h>
#include <base_msgs/setSpeed.h>
#include <base_msgs/setAccel.h>
#include <base_msgs/motorRst.h>
#include <base_msgs/motorOdom.h>
#include <base_msgs/powerInfo.h>
#include <std_msgs/Empty.h>

// ros setup
void comm_setup();

// handle messages
void comm_run();

// ros message handlers
void comm_reset_handler(const base_msgs::motorRst& msg);
void comm_set_speed_handler(const base_msgs::setSpeed& msg);
void comm_set_pid_handler(const base_msgs::setPid& msg);
void comm_set_acceleration_handler(const base_msgs::setAccel& msg);

// ros "services"
void comm_get_odometry_handler(const std_msgs::Empty &msg);
void comm_get_power_handler(const std_msgs::Empty &msg);
