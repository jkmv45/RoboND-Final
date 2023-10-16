#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "pick_objects/locations.h"


geometry_msgs::Pose currentPose;
bool dropComplete = false;
bool pickupInProgress = true;

void getPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    ROS_INFO("Current Pose: x: %f y: %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
    currentPose = msg->pose.pose;
}


int main( int argc, char** argv )
{
    std::string mode = "op";
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n("add_markers");
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",10,getPose);

    // Get mode argument if applicable
    n.getParam("mode",mode);
    ROS_INFO("Mode: %s",mode.c_str());

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = PICKUP_X;
    marker.pose.position.y = PICKUP_Y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    uint t = 0;
    while (ros::ok())
    {
        // TEST CODE - for add_marker.sh
        if(mode.compare("test") == 0){
            if(t == 5){
                marker.action = visualization_msgs::Marker::DELETE;
                ROS_INFO("Delete pickup marker");
            } else if(t == 10) {
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = DROPOFF_X;
                marker.pose.position.y = DROPOFF_Y;
                ROS_INFO("Add dropoff marker");
            } else if (t >= 15){
                marker.action = visualization_msgs::Marker::DELETE;
                t = 0;
                ROS_INFO("Delete dropoff marker");
            } else if (t < 5) {
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = PICKUP_X;
                marker.pose.position.y = PICKUP_Y;
                ROS_INFO("Add pickup marker");
            }
            t++;
        } else {
            if(dropComplete){
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = PICKUP_X;
                marker.pose.position.y = PICKUP_Y;
                dropComplete = false;
                pickupInProgress = true;
            } else if(pickupInProgress && fabs(currentPose.position.x - PICKUP_X) < POSTOL && fabs(currentPose.position.y - PICKUP_Y) < POSTOL){
                ROS_INFO("Arrived at pickup location!");
                marker.action = visualization_msgs::Marker::DELETE;
                pickupInProgress = false;
            } else if(!pickupInProgress && fabs(currentPose.position.x - DROPOFF_X) < POSTOL && fabs(currentPose.position.y - DROPOFF_Y) < POSTOL){
                ROS_INFO("Arrived at dropoff location!");
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = DROPOFF_X;
                marker.pose.position.y = DROPOFF_Y;
                dropComplete = true;
            }
            ros::spinOnce();
        }

        // Publish the marker
        marker_pub.publish(marker);

        r.sleep();
    }
}