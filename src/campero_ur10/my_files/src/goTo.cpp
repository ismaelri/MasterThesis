#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <time.h>
#include <tf/transform_listener.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


using namespace std;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "goTo");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  moveit_visual_tools::MoveItVisualTools visual_tools("/campero_base_footprint");  
  visual_tools.deleteAllMarkers();

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  string pos = argv[1];
  
  ROS_INFO_STREAM("Go to " << pos);
  move_group.setNamedTarget(pos);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  ROS_INFO_STREAM("Moving...");
  if (success){
    bool success2 = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
    if (success2) {
        move_group.setStartStateToCurrentState();
    }
  }

  sleep(2);
  ROS_INFO_STREAM("Complete");
  ros::shutdown();
  return 0;
}