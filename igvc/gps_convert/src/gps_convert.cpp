#include <ros/ros.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <cstdlib>
#include <imu_msg/Orientation.h>
#include <geodesy/wgs84.h>
#include <dynparam/doubleparam.h>
#include <gps_convert/LocalWorldConvert.h>

ros::Publisher utmPub;

double heading;
geometry_msgs::Point utm;
bool new_gps, new_compass;
double initial_x = 0, initial_y = 0;

#define INITIAL_AVERAGE 5

DoubleParam filterRate;

bool convertUtmService( gps_convert::LocalWorldConvert::Request &req,
        gps_convert::LocalWorldConvert::Response &resp )
{
    resp.pose.pose.position.x = (req.pose.pose.position.x - initial_x);
    resp.pose.pose.position.y = (req.pose.pose.position.y - initial_y);

    resp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI/2.0 - tf::getYaw(req.pose.pose.orientation));

    ROS_ERROR("gps_convert: Converting (%f, %f) to (%f, %f)", req.pose.pose.position.x, req.pose.pose.position.y,
            resp.pose.pose.position.x, resp.pose.pose.position.y );

    return true;
}

void imuCallback(const imu_msg::Orientation &msg)
{
    heading = msg.heading;
    new_compass = true;
}

void gpsCallback(const sensor_msgs::NavSatFix &msg)
{
    geodesy::UTMPoint utmPoint;
    convert(msg, utmPoint);
    utm = geodesy::toGeometry(utmPoint);

    new_gps = true;

    utmPub.publish(utm);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_convert");

    ros::NodeHandle n("~");

    tf::TransformListener tfs;

    filterRate.init(&n, "filterRate", 0.3);

    /* default variable values */
    heading = 0;
    new_gps = false;
    new_compass = false;
    
    utmPub = n.advertise<geometry_msgs::Point>("/gps_utm", 1);
    ros::Subscriber gpsSub = n.subscribe("/gps", 1, gpsCallback);
    ros::Subscriber imuSub = n.subscribe("/imu", 1, imuCallback);

    ros::ServiceServer gpsSrv = n.advertiseService("convert", convertUtmService);

    // publish 0,0 map offset
    static tf::TransformBroadcaster br; 
    tf::Transform amos_to_world( tf::createQuaternionFromYaw(0), tf::Vector3(0, 0, 0) );

    //Old values to perform filtering on the GPS fix
    double prev_x, prev_y;

    // run at 50 Hz
    ros::Rate rate(50);

    // Initial gps point after filtering
    int initializedCount = INITIAL_AVERAGE;

    while(ros::ok())
    {
        ros::spinOnce();
        
        // wait for a new reading on both sensors
        if( new_gps && new_compass )
        {
            new_gps = false;
            new_compass = false;

            if( initializedCount )
            {
                initializedCount--;

                initial_x += utm.x;
                initial_y += utm.y;

                if( !initializedCount )
                {
                    initial_x /= INITIAL_AVERAGE;
                    initial_y /= INITIAL_AVERAGE;

                    prev_x = initial_x;
                    prev_y = initial_y;
                }
            }

            try{
            	prev_x = (1 - filterRate.get()) * prev_x + filterRate.get() * utm.x;
            	prev_y = (1 - filterRate.get()) * prev_y + filterRate.get() * utm.y;

                tf::Transform world_to_amos( tf::createQuaternionFromYaw( heading ), 
                        tf::Vector3( (prev_x - initial_x), (prev_y - initial_y), 0) );

                amos_to_world = world_to_amos.inverse();
            }
            catch( tf::TransformException& e )
            {
                ROS_ERROR("%s", e.what());
            }
        }

        if( !initializedCount )
        {
            // publish our latest transform
            br.sendTransform(tf::StampedTransform(amos_to_world, ros::Time::now(), "/amos", "/world"));
        }

        rate.sleep();
    }
}

