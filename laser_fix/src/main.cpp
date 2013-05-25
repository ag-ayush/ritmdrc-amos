#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>


ros::NodeHandle *n;
ros::Publisher  *pub;

void laser_callback( const sensor_msgs::LaserScan::ConstPtr &msg )
{
    sensor_msgs::LaserScan new_scan(*msg);

    for( size_t i = 0; i < msg->ranges.size(); i++ )
    {
        double range = msg->ranges[i];

        if( msg->range_min < range && range < msg->range_max )
        {
            // valid
            range = range;
        }
        else if( !std::isfinite(range) && range > 0 )
        {
            // max range
            range = msg->range_max;
        }
        else if ( !std::isfinite(range) && range < 0 )
        {
            // min range
            range = msg->range_min;
        }
        else
        {
            // NaN - min_range
            range = msg->range_min;
        }

        new_scan.ranges[i] = (range);
    }

    new_scan.angle_min = msg->angle_min;
    new_scan.angle_max = msg->angle_max;
    new_scan.angle_increment = msg->angle_increment;

    new_scan.time_increment = msg->time_increment;

    new_scan.scan_time = msg->scan_time;

    new_scan.range_min = msg->range_min;
    new_scan.range_max = msg->range_max + 0.000001; // plus a little so they count

    new_scan.header = msg->header;

    pub->publish(new_scan);
}

int main( int argc, char **argv )
{
    ros::init(argc, argv, "laser_scan_fix");

    n = new ros::NodeHandle("~");

    ros::Subscriber s = n->subscribe( "lidar_in", 1 , &laser_callback );
    ros::Publisher  p = n->advertise<sensor_msgs::LaserScan>( "lidar_out", 1 );

    pub = &p;

    ros::spin();

    return 0;
}

