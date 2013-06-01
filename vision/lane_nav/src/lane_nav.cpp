/*
 * File:	lane_nav.cpp
 *
 * Navigate through a lane by using lane markers provided by another node
 */
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <dynamicgrid/dynamicgrid.h>
#include <lane_nav_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynparam/doubleparam.h>
#include <dynparam/boolparam.h>
#include <dynparam/stringparam.h>
#include "lane_nav.h"
#include <algorithm>

// global map
DynamicGrid<LaneMarker> *laneMap = NULL;
tf::TransformListener *tfs = NULL;

// parameters
DoubleParam straightSearchDist;
DoubleParam markerStraightForce;
DoubleParam markerPerpForce;
DoubleParam forceK;
DoubleParam distExponent;

BoolParam showDebug;
BoolParam laneDisabled;
BoolParam usePerp;

StringParam markerFrame;

#undef sign
#define sign(x) ((x)==0 ? 0 : (x) < 0 ? -1 : 1)

/*
void adjustStraightMarker(double marker_x, double marker_y)
{
    double sDist = straightSearchDist.get();

    LaneMarker *sMarker = laneMap->get(marker_x, marker_y);

    if( !sMarker ) return;

    // average over our search dist
    for( double dx = -sDist+laneMap->getResolution()/2.0; 
            dx <= sDist-laneMap->getResolution()/2.0; dx+=laneMap->getResolution().get() )
    {
        for( double dy = -sDist+laneMap->getResolution()/2.0; 
                dy <= sDist-laneMap->getResolution()/2.0; dy+=laneMap->getResolution().get() )
        {
            if( dx == 0 && dy == 0 ) continue;

            double x = marker_x + dx;
            double y = marker_y + dy;

            LaneMarker *marker = laneMap->get(x, y);

            if( !marker ) continue;

            sMarker->fx += marker->fx;
            sMarker->fy += marker->fy;

        }
    }
}
*/

void marker_callback( const lane_nav_msgs::MarkerArray &msg )
{
	std::set<LaneMarker*> modified_set;

	for( size_t i = 0; i < msg.markers.size(); i++ )
	{
		double close_x, close_y, far_x, far_y;
		double x,y;
		close_x = msg.markers[i].closer.pose.position.x;
		close_y = msg.markers[i].closer.pose.position.y;
		far_x   = msg.markers[i].farther.pose.position.x;
		far_y   = msg.markers[i].farther.pose.position.y;

		// put marker in the middle of the points
		x = (close_x + far_x) / 2.0;
		y = (close_y + far_y) / 2.0;

		// insert this into the map, allocate if needed
		laneMap->allocateRegion( x,y );
		laneMap->doRegionAllocations();

		// get the marker for this position
		LaneMarker *marker = laneMap->get( x, y );

		geometry_msgs::PoseStamped origin, amosPos;
		origin.pose.position.x = 0;
		origin.pose.position.y = 0;
		origin.pose.position.z = 0;
		origin.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		origin.header.frame_id = "/amos";

		// only update the marker if not present, fx=fy=0, or amos' heading is in the same direction as the marker force
		try
		{
			// present
			tfs->transformPose(markerFrame.get(), origin, amosPos);
			double amos_yaw = tf::getYaw( amosPos.pose.orientation );
			double marker_yaw;

			double toAmos = atan2( amosPos.pose.position.y - far_y, amosPos.pose.position.x - far_x );
			double straightAngle = atan2( far_y - close_y, far_x - close_x );

            double signAngle = toAmos-straightAngle;
            if( signAngle >=  M_PI ) signAngle -= 2*M_PI;
            if( signAngle <= -M_PI ) signAngle += 2*M_PI;

			double perpAngle = straightAngle + M_PI/2.0*sign(signAngle);

			double straightForce;
			double perpForce;
			switch(msg.markers[i].type)
			{
				case lane_nav_msgs::Marker::LEFT:
				case lane_nav_msgs::Marker::RIGHT:
					straightForce = markerStraightForce.get();
					perpForce	  = markerPerpForce.get();
					break;
				case lane_nav_msgs::Marker::STRAIGHT:
                    if (!usePerp.get()) {
                        continue;
                    }

					straightForce = 0;
					perpForce	  = markerPerpForce.get();
					break;
				default:
					ROS_ERROR("lane_nav: Unknown marker type %i", msg.markers[i].type);
					continue;
			}
			// the angle increment
			double fx_inc = straightForce * cos(straightAngle) + perpForce * cos(perpAngle);
			double fy_inc = straightForce * sin(straightAngle) + perpForce * sin(perpAngle);

            marker_yaw = atan2( marker->fy, marker->fx );

            // if in same direction of amos update
            if( (marker->fx == 0.0 && marker->fy == 0.0) || fabs( amos_yaw - marker_yaw ) <= 135.0*M_PI/180.0 )
            {
                marker->fx += fx_inc;
                marker->fy += fy_inc;

                modified_set.insert(marker);
            }

		}
		catch( tf::TransformException &e )
		{
			ROS_ERROR("lane_nav: %s", e.what());
		}

		// re-normalize
		for( std::set<LaneMarker*>::iterator iter = modified_set.begin(); iter != modified_set.end(); iter++ )
		{
			double scale = hypot( (*iter)->fx, (*iter)->fy );

			(*iter)->fx /= scale;
			(*iter)->fy /= scale;
		}
	}
}

double distScale( double dist )
{
	return forceK.get()/pow(dist, distExponent.get());
}

int main( int argc, char **argv )
{
	ros::init( argc, argv, "lane_nav" );

	ros::NodeHandle n("~");

	tfs = new tf::TransformListener();

	/*
	 * Publishers/Subscribers
	 */
	ros::Publisher goalPub = n.advertise<geometry_msgs::PoseStamped>("goal", 1);

	ros::Subscriber markerSub = n.subscribe("markers", 1, marker_callback);

	/*
	 * DynParams
	 */
	BoolParam stop;
	stop.init(&n, "/stop", false);

	laneDisabled.init(&n, "/lane/disabled", false);

    usePerp.init(&n, "usePerp", false);

    markerFrame.init(&n, "/lane/markerFrame", "/world");
    
	DoubleParam scaleForce;
	scaleForce.init(&n, "scaleForce", 5.0);

	DoubleParam markerResolution;
	markerResolution.init(&n, "markerResolution", 0.5);

	DoubleParam gridSize;
	gridSize.init(&n, "gridSize", 10.0);

	DoubleParam searchDist;
	searchDist.init(&n, "searchDist", 3.0);

	markerStraightForce.init(&n, "markerStraightForce", 3.0);
	markerPerpForce.init(&n, "markerPerpForce", 1.0);

    forceK.init(&n, "forceK", 10.0);

    distExponent.init( &n, "distExponent", 1.0 );
    showDebug.init(&n, "showDebug", false);

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
    visualization_msgs::MarkerArray markers;
	/*
	 * The map
	 */
	LaneMarker init;
    init.fx = 0;
    init.fy = 0;
	laneMap = new DynamicGrid<LaneMarker>(markerResolution.get(), 
			gridSize.get(), gridSize.get(), init);

	// points for transforms
	geometry_msgs::PoseStamped origin, amosPos;
	origin.pose.position.x = 0;
	origin.pose.position.y = 0;
	origin.pose.position.z = 0;
	origin.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	origin.header.frame_id = "/amos";

    tfs->waitForTransform(markerFrame.get(), "/amos", ros::Time(), ros::Duration(10));

    ros::Rate rate(5);
	/*
	 * main loop
	 */
	while( ros::ok() )
	{
		ros::spinOnce();

        rate.sleep();

        if (laneDisabled.get()) {
        	continue;
        }

		try {
			// Get amos' position
			tfs->transformPose(markerFrame.get(), origin, amosPos);

			// search for applicable markers in the map
			// calculate sum of marker force
			double fx_total = 0;
			double fy_total = 0;
			double sDist = searchDist.get();
            int i = 0;

            markers.markers.clear();

			for( double dx = -sDist+markerResolution.get()/2.0; dx <= sDist-markerResolution.get()/2.0; dx+=markerResolution.get() )
			{
                for( double dy = -sDist+markerResolution.get()/2.0; dy <= sDist-markerResolution.get()/2.0; dy+=markerResolution.get() )
				{
                    if( dx == 0 && dy == 0 ) continue;

					double x = amosPos.pose.position.x + dx;
					double y = amosPos.pose.position.y + dy;

                    LaneMarker *marker = laneMap->get(x, y);

                    if( !marker ) continue;

                    if( showDebug.get() )
                    {
                        visualization_msgs::Marker vis_marker;
                        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                        vis_marker.header.frame_id = markerFrame.get();
                        vis_marker.header.stamp = ros::Time::now();

                        // Set the namespace and id for this marker.  This serves to create a unique ID
                        // Any marker sent with the same namespace and id will overwrite the old one
                        vis_marker.ns = "line_nav";
                        vis_marker.lifetime = ros::Duration(0.5);
                        vis_marker.id = i++;

                        double scale = 1.0;

                        if( marker->fx == 0.0 && marker->fy == 0.0 ) {
                            vis_marker.type = visualization_msgs::Marker::SPHERE;
                            scale = 0.05;
                        } else {
                            vis_marker.type = visualization_msgs::Marker::ARROW;
                            scale = 0.5;
                        }

                        // Set the marker action.  Options are ADD and DELETE
                        vis_marker.action = visualization_msgs::Marker::ADD;

                        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                        vis_marker.pose.position.x = x;
                        vis_marker.pose.position.y = y;
                        vis_marker.pose.position.z = 0;
                        vis_marker.pose.orientation = tf::createQuaternionMsgFromYaw( atan2( marker->fy, marker->fx ) );;

                        // Set the scale of the marker -- 1x1x1 here means 1m on a side
                        vis_marker.scale.x = scale;
                        vis_marker.scale.y = scale;
                        vis_marker.scale.z = scale;

                        // Set the color -- be sure to set alpha to something non-zero!
                        vis_marker.color.r = 1.0f;
                        vis_marker.color.g = 0.0f;
                        vis_marker.color.b = 0.0f;
                        vis_marker.color.a = 1.0;

                        vis_marker.lifetime = ros::Duration();

                        // Publish the marker
                        markers.markers.push_back(vis_marker);
                    }


                    if( (marker->fx == 0.0 && marker->fy == 0.0) ) continue;

                    fx_total += marker->fx * distScale( hypot( dx, dy ) );
                    fy_total += marker->fy * distScale( hypot( dx, dy ) );
                }
            }

            if( showDebug.get() )
            {
                vis_pub.publish(markers);
            }

            if( scaleForce.get() > 0 )
            {
                double scale = hypot( fx_total, fy_total ) / scaleForce.get();

                if( scale == 0.0 )
                {
                    fx_total = fy_total = 0;
                }
                else
                {
                    fx_total /= scale;
                    fy_total /= scale;
                }
            }

            // output new goal position
            geometry_msgs::PoseStamped goal = amosPos;
            goal.pose.position.x += fx_total;
            goal.pose.position.y += fy_total;

            goalPub.publish(goal);
        } 
        catch( tf::TransformException &e )
        {
            ROS_ERROR("lane_nav: %s", e.what());
        }
    }

    return 0;
}
