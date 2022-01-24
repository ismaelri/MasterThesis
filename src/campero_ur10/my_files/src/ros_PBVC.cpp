/*****************************
 * Autor: Ismael Ruiz, 719478@unizar.es
********************************/

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/CameraInfo.h>
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
#include <my_ur10_msgs/ArucoMarker.h>
#include <my_ur10_msgs/ArucoMarkerArray.h>

using namespace std;

geometry_msgs::Pose desired_pose_aruco;
geometry_msgs::Pose actual_pose_aruco;
bool see_aruco = false;
my_ur10_msgs::ArucoMarkerArray actual_markers_pose;
my_ur10_msgs::ArucoMarkerArray desired_markers_pose;
int H, W;


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

void desiredPoseCallback(my_ur10_msgs::ArucoMarkerArray msg)
{
  desired_pose_aruco = msg.markers[0].pose;
}

void actualPoseCallback(my_ur10_msgs::ArucoMarkerArray msg2)
{
  if(msg2.markers.size() < 1){
    see_aruco = false;
  }
  else{
    actual_pose_aruco = msg2.markers[0].pose;
    actual_markers_pose = msg2;
    see_aruco = true;
  }

}

void cameraInfoCallback(sensor_msgs::CameraInfo data){
  
  H = data.height;
  W = data.width;

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


std::vector<double> sameSingInAngle(std::vector<double> ang, std::vector<double> o_ang){
  std::vector<double> result;
  for(std::size_t i = 0; i < ang.size(); ++i){
    if(o_ang[i] > 3.0 && ang[i] < -3.0){
      result.push_back(abs(ang[i]));
    }else{
      if(o_ang[i] < -3.0 && ang[i] > 3.0)
        result.push_back(-ang[i]);
      else
        result.push_back(ang[i]);
    }
  }
  return result;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_PBVC");
  ros::NodeHandle n;
  ros::Publisher velocity_pub = n.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1000);
  ros::Publisher plot_pub = n.advertise<my_ur10_msgs::plot>("/all_plots", 1000);
  ros::Publisher cyclic_pub = n.advertise<std_msgs::UInt32>("/cyclic_num", 1);
  ros::Subscriber desired_sub = n.subscribe("/desired/markers_pose", 1, desiredPoseCallback);
  ros::Subscriber actual_sub = n.subscribe("/aruco_multiple/markers_pose", 1, actualPoseCallback);
  ros::Subscriber cameraInfo_sub = n.subscribe("/camera/color/camera_info", 1, cameraInfoCallback);
  ros::Rate loop_rate(20);
  ros::Rate main_loop_rate(100);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  string tracking = argv[1];
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  std::vector<double> joint_values, actual_rpy, old_actual_rpy;

  tf::TransformListener listener;
  sleep(1);
  
  ROS_INFO_STREAM("Desired Translation: \n x: " << desired_pose_aruco.position.x << ", y: " << desired_pose_aruco.position.y << ", z: " << desired_pose_aruco.position.z << "\n");
  ROS_INFO_STREAM("Desired Rotation: \n x: " << desired_pose_aruco.orientation.x << ", y: " << desired_pose_aruco.orientation.y << ", z: " << desired_pose_aruco.orientation.z << ", w: " << desired_pose_aruco.orientation.w << "\n");
  std::vector<double> desired_aruco_rpy = QuaternionToEulerAngles(desired_pose_aruco);
  ROS_INFO_STREAM("Desired RPY: \n roll: " << desired_aruco_rpy[0] << ", pitch: " << desired_aruco_rpy[1] << ", yaw: " << desired_aruco_rpy[2] << "\n");

  ROS_INFO_STREAM("Actual Translation: \n x: " << actual_pose_aruco.position.x << ", y: " << actual_pose_aruco.position.y << ", z: " << actual_pose_aruco.position.z << "\n");
  ROS_INFO_STREAM("Actual Rotation: \n x: " << actual_pose_aruco.orientation.x << ", y: " << actual_pose_aruco.orientation.y << ", z: " << actual_pose_aruco.orientation.z << ", w: " << actual_pose_aruco.orientation.w << "\n");
  std::vector<double> actual_aruco_rpy = QuaternionToEulerAngles(actual_pose_aruco);
  ROS_INFO_STREAM("Actual RPY: \n roll: " << actual_aruco_rpy[0] << ", pitch: " << actual_aruco_rpy[1] << ", yaw: " << actual_aruco_rpy[2] << "\n");

  bool keep = false;
  std::vector<double> p_o_errors, old_p_o_errors;
  double x_error, y_error, z_error, roll_error, pitch_error, yaw_error;
  double K = 0.05;
  double Ki = 0.1;
  float m_error = 0.01;
  Eigen::VectorXd velocities(6);
  Eigen::VectorXd joint_velocities(6);
  Eigen::VectorXd limited_joint_velocities(6);
  Eigen::MatrixXd J, J_pi;
  std::vector<double> max_vel = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 }; //changed { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}
  std::vector<geometry_msgs::Pose> waypoints;
  int cnt = 0;
  my_ur10_msgs::ArucoMarkerArray c_actual_markers_pose = actual_markers_pose;

  geometry_msgs::Pose actual_pose_c, actual_pose, desired_base_tool, error_pose;

  tf::Transform T_B_H, T_B_HD, T_CD_A, T_C_A, T_C_CD, T_CD_C, T_C_H, T_A_c, T_A;
  tf::StampedTransform T_H_C, T_B_C;
  listener.lookupTransform("base_link", "campero_base_footprint", ros::Time(0), T_B_C);

  actual_pose_c = move_group.getCurrentPose().pose;
  T_A_c = fromPoseToTransform(actual_pose_c);
  T_A = T_B_C * T_A_c;
  actual_pose = fromTransformToPose(T_A);
  actual_rpy = QuaternionToEulerAngles(actual_pose);
  old_actual_rpy = actual_rpy;

  int num_ite = 0;

  while(!keep){
    if(see_aruco){
      my_ur10_msgs::plot plots;
      actual_pose_c = move_group.getCurrentPose().pose;
      T_A_c = fromPoseToTransform(actual_pose_c);
      T_A = T_B_C * T_A_c;
      actual_pose = fromTransformToPose(T_A);
      actual_rpy = QuaternionToEulerAngles(actual_pose);
      kinematic_state = move_group.getCurrentState();
      joint_values = move_group.getCurrentJointValues();
      waypoints.push_back(actual_pose);

      c_actual_markers_pose = actual_markers_pose;

      
      T_B_H = fromPoseToTransform(actual_pose);
      T_CD_A = fromPoseToTransform(desired_pose_aruco);
      T_C_A = fromPoseToTransform(actual_pose_aruco);
      T_CD_C = T_CD_A * (T_C_A.inverse());
      T_C_CD = T_CD_C.inverse();
      listener.lookupTransform("tool0", "camera_color_optical_frame", ros::Time(0), T_H_C);
      T_C_H = T_H_C.inverse();
      T_B_HD = T_B_H * T_H_C * T_C_CD * T_C_H;
      
      desired_base_tool = fromTransformToPose(T_B_HD);
      
      p_o_errors.clear();
      
      x_error = desired_base_tool.position.x - actual_pose.position.x;
      y_error = desired_base_tool.position.y - actual_pose.position.y;
      z_error = desired_base_tool.position.z - actual_pose.position.z;
      tf::Quaternion dq(desired_base_tool.orientation.x, desired_base_tool.orientation.y, desired_base_tool.orientation.z, desired_base_tool.orientation.w);
      tf::Quaternion aq(actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w);
      tf::Quaternion diff;
      diff = dq * aq.inverse();
      
      error_pose.orientation.x = diff[0];
      error_pose.orientation.y = diff[1];
      error_pose.orientation.z = diff[2];
      error_pose.orientation.w = diff[3];

      std::vector<double> error_rpy = QuaternionToEulerAngles(error_pose);

      roll_error = error_rpy[0];
      pitch_error = error_rpy[1];
      yaw_error = error_rpy[2];
      
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

      actual_rpy = sameSingInAngle(actual_rpy, old_actual_rpy);
      plots.p.push_back(actual_rpy[0]);
      plots.p.push_back(actual_rpy[1]);
      plots.p.push_back(actual_rpy[2]);

      old_actual_rpy = actual_rpy;
      
      p_o_errors.push_back(x_error);
      p_o_errors.push_back(y_error);
      p_o_errors.push_back(z_error);
      p_o_errors.push_back(roll_error);
      p_o_errors.push_back(pitch_error);
      p_o_errors.push_back(yaw_error);
      keep = BadGoalPosition(p_o_errors, m_error);

      
      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, J);
      J_pi = PseudoinverseMoorePenrose(J);
      
      if(!keep){
          for(std::size_t i = 0; i < p_o_errors.size(); ++i){
              
              if(tracking == "true" && cnt >0){ //PI control
				        velocities[i] = K * p_o_errors[i] + Ki * old_p_o_errors[i];
                if(cnt<=0)
                ROS_INFO_STREAM(i << " p_o_error: " << p_o_errors[i] << " and velocity_error vector: " << velocities[i]);
              }
              else{
                velocities[i] = K * p_o_errors[i];
                if(cnt<=0)
                ROS_INFO_STREAM(i << " p_o_error: " << p_o_errors[i] << " and velocity_error vector: " << velocities[i]);
              }
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
      

      if (cnt<1){
        old_p_o_errors = p_o_errors;
      }
      else{ //Integrated
        old_p_o_errors[0] = old_p_o_errors[0] + p_o_errors[0];
        if( old_p_o_errors[0] > 0.05)
          old_p_o_errors[0] = 0.05;
        if( old_p_o_errors[0] < -0.05)
          old_p_o_errors[0] = -0.05;
        old_p_o_errors[1] = old_p_o_errors[1] + p_o_errors[1];
        if( old_p_o_errors[1] > 0.05)
          old_p_o_errors[1] = 0.05;
        if( old_p_o_errors[1] < -0.05)
          old_p_o_errors[1] = -0.05;
        old_p_o_errors[2] = old_p_o_errors[2] + p_o_errors[2];
        if( old_p_o_errors[2] > 0.05)
          old_p_o_errors[2] = 0.05;
        if( old_p_o_errors[2] < -0.05)
          old_p_o_errors[2] = -0.05;
        old_p_o_errors[3] = old_p_o_errors[3] + p_o_errors[3];
        if( old_p_o_errors[3] > 0.05)
          old_p_o_errors[3] = 0.05;
        if( old_p_o_errors[3] < -0.05)
          old_p_o_errors[3] = -0.05;
        old_p_o_errors[4] = old_p_o_errors[4] + p_o_errors[4];
        if( old_p_o_errors[4] > 0.05)
          old_p_o_errors[4] = 0.05;
        if( old_p_o_errors[4] < -0.05)
          old_p_o_errors[4] = -0.05;
        old_p_o_errors[5] = old_p_o_errors[5] + p_o_errors[5];
        if( old_p_o_errors[5] > 0.05)
          old_p_o_errors[5] = 0.05;
        if( old_p_o_errors[5] < -0.05)
          old_p_o_errors[5] = -0.05;
      }
      
      

      plots.v.push_back(limited_joint_velocities[0]);
      plots.v.push_back(limited_joint_velocities[1]);
      plots.v.push_back(limited_joint_velocities[2]);
      plots.v.push_back(limited_joint_velocities[3]);
      plots.v.push_back(limited_joint_velocities[4]);
      plots.v.push_back(limited_joint_velocities[5]);

      plots.v_e.push_back(velocities[0]);
      plots.v_e.push_back(velocities[1]);
      plots.v_e.push_back(velocities[2]);
      plots.v_e.push_back(velocities[3]);
      plots.v_e.push_back(velocities[4]);
      plots.v_e.push_back(velocities[5]);

      plots.t = ros::Time::now().toSec();

      for(std::size_t i = 0; i < c_actual_markers_pose.markers.size(); ++i){
        for(std::size_t j = 0; j < c_actual_markers_pose.markers[i].img_points.size(); ++j){
          plots.m.push_back(c_actual_markers_pose.markers[i].img_points[j].x);
          plots.m.push_back(c_actual_markers_pose.markers[i].img_points[j].y);
        }
      }

      plots.i_dim.push_back(H);
      plots.i_dim.push_back(W);

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
      
      

      

      if(limited_joint_velocities[2] < 0.01 && limited_joint_velocities[2] > -0.01 && limited_joint_velocities[2] != 0.0){
        num_ite++;
        if( num_ite == 2)
          num_ite = 0;
      }
      
      std_msgs::UInt32 n_i;
      n_i.data = num_ite;
      cyclic_pub.publish(n_i);

      main_loop_rate.sleep();


    }
    else{

      std_msgs::Float64MultiArray msg_2;
      msg_2.data.push_back(0.0);
      msg_2.data.push_back(0.0);
      msg_2.data.push_back(0.0);
      msg_2.data.push_back(0.0);
      msg_2.data.push_back(0.0);
      msg_2.data.push_back(0.0);
      velocity_pub.publish(msg_2);
      ROS_INFO_STREAM("The camera cannot see the ArUco marker, robot stopped");

    }

    
    
  }
  
  visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
  visual_tools.trigger();

  sleep(2);

  ros::shutdown();
  return 0;
}
