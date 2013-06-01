#include <ros/ros.h>
#include <std_msgs/String.h>

#include <dynparam/boolparam.h>

#include <cstdlib>
#include <sstream>

BoolParam mute;

void speechCallback(const std_msgs::String& msg) {
    if (mute.get()) {
        return;
    }

    std::stringstream cmd;
    cmd << "echo \"" << msg.data << "\" | festival --tts";
    int res = system(cmd.str().c_str());

    if (res) {
        ROS_ERROR("Running festival returned value %d", res);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "speech");
    ros::NodeHandle nh("~");

    mute.init(&nh, "mute", false);

    ros::Subscriber speechSub = nh.subscribe("tts", 1, speechCallback);

    ros::spin();

    return 0;
}
