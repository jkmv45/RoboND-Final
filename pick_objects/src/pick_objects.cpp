#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <pick_objects/locations.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    // ********** Setup **********
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // ********** Define Targets **********
    move_base_msgs::MoveBaseGoal goal, pickup, dropoff, home;
    // set up the frame parameters
    pickup.target_pose.header.frame_id = "map";
    pickup.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    pickup.target_pose.pose.position.x = PICKUP_X;
    pickup.target_pose.pose.position.y = PICKUP_Y;
    pickup.target_pose.pose.orientation.w = 1.0;
    // set up the frame parameters
    dropoff.target_pose.header.frame_id = "map";
    dropoff.target_pose.header.stamp = ros::Time::now();
  
    // Define a position and orientation for the robot to reach
    dropoff.target_pose.pose.position.x = DROPOFF_X;
    dropoff.target_pose.pose.position.y = DROPOFF_Y;
    dropoff.target_pose.pose.orientation.w = 1.0;

    // set up the frame parameters
    home.target_pose.header.frame_id = "map";
    home.target_pose.header.stamp = ros::Time::now();
  
    // Define a position and orientation for the robot to reach
    home.target_pose.pose.position.x = HOME_X;
    home.target_pose.pose.position.y = HOME_Y;
    home.target_pose.pose.orientation.w = 1.0;

    bool goToPickup = true;
    bool goHome = false;
    while(ros::ok()){
        // Goal selection logic; home is first priority; otherwise, alternate between pickup and dropoff target
        if(goHome){
            goal = home;
            ROS_INFO("Sending robot home");
            goHome = false;
        }else if(goToPickup){
            goal = pickup;
            ROS_INFO("Sending pickup target");
        } else {
            goal = dropoff;
            ROS_INFO("Sending dropoff target");
        }
        
        // Set timestamp of goal pose
        goal.target_pose.header.stamp = ros::Time::now();
        
        // Send the goal position and orientation for the robot to reach
        ac.sendGoal(goal);

        // Wait an infinite time for the results
        ac.waitForResult();

        // Check if the robot reached its goal
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Hooray, the robot reached the target");
            goToPickup = !goToPickup; // Toggle goal
        }
        else{
            ROS_INFO("The robot failed to reach target for some reason");
            goHome = true; // Try going home after failure
        }

        // Wait (simulate picking up object)
        ros::Duration(5.0).sleep();
    }


    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending pickup target");
    ac.sendGoal(pickup);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 5 meter forward");
    else
        ROS_INFO("The base failed to move forward 5 meter for some reason");

    ros::Duration(5.0).sleep();

    // ***** Second Move *****
    

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending dropoff target");
    ac.sendGoal(dropoff);
  
    // Wait an infinite time for the results
    ac.waitForResult();
  
    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 5 meters backward");
    else
        ROS_INFO("The base failed to move backward 5 meters for some reason");

    ros::Duration(5.0).sleep();

    return 0;
}