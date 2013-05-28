#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

bool have_goal = false;
geometry_msgs::PoseStamped goalPose;

void goalCallback( const geometry_msgs::PoseStamped::ConstPtr &msg )
{
    // save the new goal
    goalPose = *msg;
    have_goal = true;
}

int main( int argc, char **argv )
{
    ros::init( argc, argv, "goal_republish" );

    ros::NodeHandle n("~");

    ros::Publisher pub  = n.advertise<geometry_msgs::PoseStamped>( "goal_out", 1 );
    ros::Subscriber sub = n.subscribe( "goal_in", 1, goalCallback );

    ros::Rate rate(5);

    while( ros::ok() )
    {
        // handle messages
        ros::spinOnce();

        // re-publish the goal if present
        if( have_goal ) pub.publish(goalPose);

        // wait the rest of our time
        rate.sleep();
    }
}
