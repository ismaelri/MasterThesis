/*****************************
 * Autor: Ismael Ruiz, 719478@unizar.es
********************************/

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

//Messages
#include "my_ur10_msgs/plot.h"

using namespace std;


std::vector<double>  QuaternionToEulerAngles(geometry_msgs::Pose our_pose){
    std::vector<double> result(3);
    tf::Quaternion quat(our_pose.orientation.x, our_pose.orientation.y, our_pose.orientation.z, our_pose.orientation.w);
    tf::Matrix3x3 m(quat);
    m.getRPY(result[0], result[1], result[2]);

    return result;
}

bool BadGoalPosition(std::vector<double> errors, float num){
  bool bad = false;
  
  if ((errors[0] <= num &&  errors[0] >= -num) && (errors[1] <= num &&  errors[1] >= -num) && (errors[2] <= num &&  errors[2] >= -num) && 
  (errors[3] <= num &&  errors[3] >= -num) && (errors[4] <= num &&  errors[4] >= -num) && (errors[5] <= num &&  errors[5] >= -num)){
    bad = true;
  }
  return bad;
}

Eigen::MatrixXd PseudoinverseMoorePenrose(Eigen::MatrixXd m){
  Eigen::MatrixXd result;
  Eigen::MatrixXd mt = m.transpose();
  Eigen::MatrixXd p = mt * m;
  if(p.determinant() < 0.001)
    ROS_INFO_STREAM("WARNING!!! The determinant is less than 0.01");
  result = p.inverse() * mt;

  return result;
}

Eigen::VectorXd velocityLimiter(Eigen::VectorXd entry, std::vector<double> max_vel){
    Eigen::VectorXd result(6);
    for(std::size_t i = 0; i < max_vel.size(); ++i){
        if(entry[i] > max_vel[i]){
            result[i] = max_vel[i];
        }
        else if(entry[i] < -max_vel[i]){
            result[i] = -max_vel[i];
        }
        else{
            result[i] = entry[i];
        }

    }
    return result;
}

tf::Transform fromPoseToTransform(geometry_msgs::Pose pos){
  tf::Transform result;

  result.setOrigin(tf::Vector3(pos.position.x, pos.position.y, pos.position.z));
  result.setRotation(tf::Quaternion(pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w));

  return result;
}

geometry_msgs::Pose fromTransformToPose( tf::Transform tran){
  geometry_msgs::Pose result;

  tf::Quaternion q = tran.getRotation();
  result.position.x = tran.getOrigin()[0];
  result.position.y = tran.getOrigin()[1];
  result.position.z = tran.getOrigin()[2];
  result.orientation.x = q[0];
  result.orientation.y = q[1];
  result.orientation.z = q[2];
  result.orientation.w = q[3];

  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_PBC");
  ros::NodeHandle n;
  ros::Publisher velocity_pub = n.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1000);
  ros::Publisher plot_pub = n.advertise<my_ur10_msgs::plot>("/all_plots", 1000);
  ros::Rate loop_rate(20);
  ros::Rate main_loop_rate(100);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  moveit_visual_tools::MoveItVisualTools visual_tools("/base_link");  //campero_base_footprint
  visual_tools.deleteAllMarkers();

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  std::vector<double> joint_values, actual_rpy, desired_rpy;

  tf::TransformListener listener;
  sleep(1);

  geometry_msgs::Pose desired_pose_c, desired_pose;

  // Both translation y rotation
  /*
  desired_pose.position.x = 0.770661;
  desired_pose.position.y = 0.256016;
  desired_pose.position.z = 0.388589;
  desired_pose.orientation.x =  -0.174527;
  desired_pose.orientation.y =  0.68523;
  desired_pose.orientation.z =  0.685157;
  desired_pose.orientation.w =  0.174817;
  */
  
  // Only translation
  
  desired_pose_c.position.x = -0.380959411746;
  desired_pose_c.position.y = 0.055786282653;
  desired_pose_c.position.z = 1.19004854594;
  desired_pose_c.orientation.x =  0.999999069355;
  desired_pose_c.orientation.y =  -0.00127860507124;
  desired_pose_c.orientation.z =  0.000438223290601;
  desired_pose_c.orientation.w =  0.000185521420206;

  // Only rotation
  /*
  desired_pose.position.x = 0.534;
  desired_pose.position.y = -0.222;
  desired_pose.position.z = 0.152;
  desired_pose.orientation.x =  -0.0003101;
  desired_pose.orientation.y =  -0.3894182;
  desired_pose.orientation.z = -0.9210607;
  desired_pose.orientation.w =  0.0007335;
  */

  //Change reference
  geometry_msgs::Pose actual_pose_c = move_group.getCurrentPose().pose; 
  geometry_msgs::Pose actual_pose;
  tf::Transform T_A, T_D;
  tf::Transform T_A_c = fromPoseToTransform(actual_pose_c);
  tf::Transform T_D_c = fromPoseToTransform(desired_pose_c);

  tf::StampedTransform T_B_C;
  listener.lookupTransform("base_link", "campero_base_footprint", ros::Time(0), T_B_C);

  T_A = T_B_C * T_A_c;
  T_D = T_B_C * T_D_c;
  actual_pose = fromTransformToPose(T_A);
  desired_pose = fromTransformToPose(T_D);



  ROS_INFO_STREAM("Desired Translation: \n x: " << desired_pose.position.x << ", y: " << desired_pose.position.y << ", z: " << desired_pose.position.z << "\n");
  ROS_INFO_STREAM("Desired Rotation: \n x: " << desired_pose.orientation.x << ", y: " << desired_pose.orientation.y << ", z: " << desired_pose.orientation.z << ", w: " << desired_pose.orientation.w << "\n");
  desired_rpy = QuaternionToEulerAngles(desired_pose);
  ROS_INFO_STREAM("Desired RPY: \n roll: " << desired_rpy[0] << ", pitch: " << desired_rpy[1] << ", yaw: " << desired_rpy[2] << "\n");

  
  
  ROS_INFO_STREAM("Actual Translation: \n x: " << actual_pose.position.x << ", y: " << actual_pose.position.y << ", z: " << actual_pose.position.z << "\n");
  ROS_INFO_STREAM("Actual Rotation: \n x: " << actual_pose.orientation.x << ", y: " << actual_pose.orientation.y << ", z: " << actual_pose.orientation.z << ", w: " << actual_pose.orientation.w << "\n");
  actual_rpy = QuaternionToEulerAngles(actual_pose);
  ROS_INFO_STREAM("Actual RPY: \n roll: " << actual_rpy[0] << ", pitch: " << actual_rpy[1] << ", yaw: " << actual_rpy[2] << "\n");


  bool keep = false;
  std::vector<double> p_o_errors;
  double x_error, y_error, z_error, roll_error, pitch_error, yaw_error;
  double K = 0.05;
  float m_error = 0.01;
  Eigen::VectorXd velocities(6);
  Eigen::VectorXd joint_velocities(6);
  Eigen::VectorXd limited_joint_velocities(6);
  Eigen::MatrixXd J, J_pi;
  std::vector<double> max_vel = { 2.16, 2.16, 3.15, 3.2, 3.2, 3.2 };
  std::vector<geometry_msgs::Pose> waypoints;
  int cnt = 0;

  geometry_msgs::Pose error_pose = actual_pose;

  while(!keep){
    my_ur10_msgs::plot plots;
    actual_pose_c = move_group.getCurrentPose().pose;
    T_A_c = fromPoseToTransform(actual_pose_c);
    T_A = T_B_C * T_A_c;
    actual_pose = fromTransformToPose(T_A);
    actual_rpy = QuaternionToEulerAngles(actual_pose);
    kinematic_state = move_group.getCurrentState();
    joint_values = move_group.getCurrentJointValues();
    waypoints.push_back(actual_pose);

    p_o_errors.clear();
    x_error = desired_pose.position.x - actual_pose.position.x;
    y_error = desired_pose.position.y - actual_pose.position.y;
    z_error = desired_pose.position.z - actual_pose.position.z;

    tf::Quaternion dq(desired_pose.orientation.x, desired_pose.orientation.y, desired_pose.orientation.z, desired_pose.orientation.w);
    tf::Quaternion aq(actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w);
    tf::Quaternion diff;
    diff = dq * aq.inverse();
      
    error_pose.orientation.x = diff[0];
    error_pose.orientation.y = diff[1];
    error_pose.orientation.z = diff[2];
    error_pose.orientation.w = diff[3];


    ROS_INFO_STREAM("x_error " << x_error << " y_error " << y_error << " z_error " << z_error);
    std::vector<double> error_rpy = QuaternionToEulerAngles(error_pose);

    roll_error = error_rpy[0];
    pitch_error = error_rpy[1];
    yaw_error = error_rpy[2];
    
    ROS_INFO_STREAM("roll_error " << roll_error << " pitch_error " << pitch_error << " yaw_error " << yaw_error);

    plots.j.push_back(joint_values[0]);
    plots.j.push_back(joint_values[1]);
    plots.j.push_back(joint_values[2]);
    plots.j.push_back(joint_values[3]);
    plots.j.push_back(joint_values[4]);
    plots.j.push_back(joint_values[5]);

    plots.e.push_back(x_error);
    plots.e.push_back(y_error);
    plots.e.push_back(z_error);
    plots.e.push_back(roll_error);
    plots.e.push_back(pitch_error);
    plots.e.push_back(yaw_error);

    plots.p.push_back(actual_pose.position.x);
    plots.p.push_back(actual_pose.position.y);
    plots.p.push_back(actual_pose.position.z);
    plots.p.push_back(actual_rpy[0]);
    plots.p.push_back(actual_rpy[1]);
    plots.p.push_back(actual_rpy[2]);
    
    p_o_errors.push_back(x_error);
    p_o_errors.push_back(y_error);
    p_o_errors.push_back(z_error);
    p_o_errors.push_back(roll_error);
    p_o_errors.push_back(pitch_error);
    p_o_errors.push_back(yaw_error);
    keep = BadGoalPosition(p_o_errors, m_error);
    
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    J = kinematic_state->getJacobian(joint_model_group);
    J_pi = PseudoinverseMoorePenrose(J);
    
    if(!keep){
        for(std::size_t i = 0; i < p_o_errors.size(); ++i){
            
            velocities[i] = K * p_o_errors[i];
            if(cnt<=0)
            ROS_INFO_STREAM(i << " p_o_error: " << p_o_errors[i] << " and velocity_error vector: " << velocities[i]);
        }
        plots.finish = false;
    
    }
    else{
        for(std::size_t i = 0; i < p_o_errors.size(); ++i){

            velocities[i] = 0.0;
        }
        plots.finish = true;
    }

    joint_velocities = J_pi * velocities;
    limited_joint_velocities = velocityLimiter(joint_velocities, max_vel);

    plots.v.push_back(limited_joint_velocities[0]);
    plots.v.push_back(limited_joint_velocities[1]);
    plots.v.push_back(limited_joint_velocities[2]);
    plots.v.push_back(limited_joint_velocities[3]);
    plots.v.push_back(limited_joint_velocities[4]);
    plots.v.push_back(limited_joint_velocities[5]);

    plots.t = ros::Time::now().toSec();

    plot_pub.publish(plots);

    cnt++;

    if(cnt<=50){
        ROS_INFO_STREAM("velocities 0: " << velocities[0] << " 1: " << velocities[1] << " 2: " << velocities[2] << " 3: " << velocities[3] << " 4: " << velocities[4] << " 5: " << velocities[5]);
        ROS_INFO_STREAM("Limited joint velocities 0: " << limited_joint_velocities[0] << " 1: " << limited_joint_velocities[1] << " 2: " << limited_joint_velocities[2] << " 3: " << limited_joint_velocities[3] << " 4: " << limited_joint_velocities[4] << " 5: " << limited_joint_velocities[5]);
    }

    

    std_msgs::Float64MultiArray msg;
    bool one_message = false;
    int connections = 0;
    
    msg.data.push_back(limited_joint_velocities[0]);
    msg.data.push_back(limited_joint_velocities[1]);
    msg.data.push_back(limited_joint_velocities[2]);
    msg.data.push_back(limited_joint_velocities[3]);
    msg.data.push_back(limited_joint_velocities[4]);
    msg.data.push_back(limited_joint_velocities[5]);
    


    velocity_pub.publish(msg);

    main_loop_rate.sleep();
  }

  visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
  visual_tools.trigger();

  sleep(2);

  ros::shutdown();
  return 0;
}