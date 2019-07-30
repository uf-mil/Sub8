#include <ros/ros.h>
#include <mil_msgs/MoveToActionResult.h>
#include <mil_msgs/MoveToAction.h>
#include <mil_msgs/MoveToActionGoal.h>
#include <mil_msgs/MoveToGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include <string>
#include <iostream>

using namespace std;

// declares MoveToAction action client
typedef actionlib::SimpleActionClient<mil_msgs::MoveToAction> MoveToActionClient;

// first arg = forward command, second arg = up command
int main(int argc, char** argv){
  ros::init(argc, argv, "move_test");

  // not sure what the server is called, figure out what the server is actually called
  MoveToActionClient ac("/moveto", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the moveto server to come up!");
    }

    string x = "";
    string y = "";

    if (argc < 2) {
      cout << "Error: No command found.";
      return 0;
    }

    // declares goal for the sub
    mil_msgs::MoveToGoal goal;
    x = string(argv[1]);

    goal.header.frame_id = "base_link";
    goal.header.stamp = ros::Time::now();

    // sets goal to x meter forward
    goal.posetwist.pose.position.x = atoi(argv[1]);
    goal.posetwist.pose.orientation.w = atoi(argv[1]);

    // sets goal to y meters upward if an arg is found
    if(argc == 3) {
      goal.posetwist.pose.position.y = atoi(argv[2]);
      x = string(argv[1]);
      y = string(argv[2]);
    }

    ROS_INFO("Sending goal");

    ac.sendGoal(goal);
    ac.waitForResult();

    // checks current robot state vs. goal state
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    cout << "The sub moved " + x + " meter(s) forward" << endl;
    if(argc == 3) {
      cout << "The sub moved " + y + " meter(s) up";
    }
  }
    else {
    cout << "The sub failed to move forward " + x + " meter(s)" << endl;
    if(argc == 3){
      "The sub failed to move up " + y + " meters(s)";
    }
}
    return 0;
}
