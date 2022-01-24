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
#include <my_ur10_msgs/plot_IBVC.h>
#include <my_ur10_msgs/e_m_plot.h>
#include <my_ur10_msgs/ArucoMarker.h>
#include <my_ur10_msgs/ArucoMarkerArray.h>

using namespace std;

my_ur10_msgs::ArucoMarkerArray desired_markers_pose;
my_ur10_msgs::ArucoMarkerArray actual_markers_pose;
Eigen::MatrixXd K(3,3);
Eigen::MatrixXd P(3,4);
int H,W;
bool see_aruco = false;




std::vector<double>  QuaternionToEulerAngles(geometry_msgs::Pose our_pose){
    std::vector<double> result(3);
    tf::Quaternion quat(our_pose.orientation.x, our_pose.orientation.y, our_pose.orientation.z, our_pose.orientation.w);
    tf::Matrix3x3 m(quat);
    m.getRPY(result[0], result[1], result[2]);

    return result;
}

bool BadGoalPositionHVC(std::vector<double> errors, float num){
  bool bad = false;
  int cnt_bg = 0;
  int cnt_bg2 = 0;

  for(std::size_t i = 0; i < errors.size()/6; ++i){
    cnt_bg2 = 0;
    for(std::size_t j = 0; j < 24; ++j){

      if(errors[i+j] <= num && errors[i+j] >= -num){
        cnt_bg2++;
        if(cnt_bg2 == 24)
          cnt_bg++;
      }
      else{
        break;
      }
    }
    ROS_INFO_STREAM("Good Arucos: " << cnt_bg);
  }

  if(cnt_bg >= 2)
    bad = true;
  else
    bad = false;
  
  return bad;
}

Eigen::MatrixXd PseudoinverseMoorePenrose(Eigen::MatrixXd m, bool show){
  Eigen::MatrixXd result;
  Eigen::MatrixXd mt = m.transpose();
  Eigen::MatrixXd p = mt * m;
  if(p.determinant() < 0.001 && show)
    ROS_INFO_STREAM("WARNING!!! The determinant is less than 0.001");
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

void desiredMarkersPoseCallback(my_ur10_msgs::ArucoMarkerArray msg)
{
  desired_markers_pose = msg;
}

void actualMarkersPoseCallback(my_ur10_msgs::ArucoMarkerArray msg2)
{
  if(msg2.markers.size() < 2){
    see_aruco = false;
  }
  else{ 
    actual_markers_pose = msg2;
    see_aruco = true;
  }

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

void cameraInfoCallback(sensor_msgs::CameraInfo data){
  //float64[9]  K 3x3
  K(0,0) = data.K[0];
  K(0,1) = data.K[1];
  K(0,2) = data.K[2];
  K(1,0) = data.K[3];
  K(1,1) = data.K[4];
  K(1,2) = data.K[5];
  K(2,0) = data.K[6];
  K(2,1) = data.K[7];
  K(2,2) = data.K[8];
  
  //float64[12] P 3x4 
  P(0,0) = data.P[0];
  P(0,1) = data.P[1];
  P(0,2) = data.P[2];
  P(0,3) = data.P[3];
  P(1,0) = data.P[4];
  P(1,1) = data.P[5];
  P(1,2) = data.P[6];
  P(1,3) = data.P[7];
  P(2,0) = data.P[8];
  P(2,1) = data.P[9];
  P(2,2) = data.P[10];
  P(2,3) = data.P[11];

  H = data.height;
  W = data.width;
}


std::vector<int> obtainIdMarks(my_ur10_msgs::ArucoMarkerArray a_array){
  std::vector<int> result;
  for(std::size_t i = 0; i < a_array.markers.size(); ++i){
    result.push_back(a_array.markers[i].id);
  }

  return result;
}


std::vector<double> obtainAngleAndAxis(my_ur10_msgs::ArucoMarkerArray a_array, my_ur10_msgs::ArucoMarkerArray d_array){ //NEW HVC
  std::vector<double> result;
  double qi, qj, qk, qr, angle, ax, ay, az, s_a;
  tf::Quaternion diff;
  for(std::size_t i = 0; i < a_array.markers.size(); ++i){
    for(std::size_t d = 0; d < d_array.markers.size(); ++d){
      if(a_array.markers[i].id == d_array.markers[d].id){
        tf::Quaternion dq(d_array.markers[d].pose.orientation.x, d_array.markers[d].pose.orientation.y, d_array.markers[d].pose.orientation.z, d_array.markers[d].pose.orientation.w);
        tf::Quaternion aq(a_array.markers[i].pose.orientation.x, a_array.markers[i].pose.orientation.y, a_array.markers[i].pose.orientation.z, a_array.markers[i].pose.orientation.w);
        diff = dq * aq.inverse();
        qi = diff[0];
        qj = diff[1];
        qk = diff[2];
        qr = diff[3];
        s_a = sqrt((qi*qi)+(qj*qj)+(qk*qk));
        angle = 2 * atan2(s_a, qr);
        ax = qi/s_a;
        ay = qj/s_a;
        az = qk/s_a;
        for(std::size_t j = 0; j < 4; ++j){
          result.push_back(ax); //axisX
          result.push_back(ay); //axisY
          result.push_back(az); //axisZ
          result.push_back(angle); //angle
        }
      }
    }
  }
  return result;
}

std::vector<double> obtainFeaturesImage(my_ur10_msgs::ArucoMarkerArray f_array, my_ur10_msgs::ArucoMarkerArray s_array,  Eigen::MatrixXd M, bool actual){ //NEW HVC
  std::vector<double> result;
  double qi, qj, qk, qr, angle, ax, ay, az, s_a;
  tf::Quaternion diff;
  for(std::size_t i = 0; i < f_array.markers.size(); ++i){
    for(std::size_t d = 0; d < s_array.markers.size(); ++d){
      if(f_array.markers[i].id == s_array.markers[d].id){
        for(std::size_t j = 0; j < f_array.markers[i].img_points.size(); ++j){
          Eigen::VectorXd p2d(3);
          p2d(0) = f_array.markers[i].img_points[j].x;
          p2d(1) = f_array.markers[i].img_points[j].y;
          p2d(2) = 1.0;
          Eigen::VectorXd p3d = M.inverse() * p2d;
          result.push_back(p3d[0]);
          result.push_back(p3d[1]);
          result.push_back(log(f_array.markers[i].pose.position.z));
          if(actual){
            tf::Quaternion dq(s_array.markers[d].pose.orientation.x, s_array.markers[d].pose.orientation.y, s_array.markers[d].pose.orientation.z, s_array.markers[d].pose.orientation.w);
            tf::Quaternion aq(f_array.markers[i].pose.orientation.x, f_array.markers[i].pose.orientation.y, f_array.markers[i].pose.orientation.z, f_array.markers[i].pose.orientation.w);
            diff = dq * aq.inverse();
            qi = diff[0];
            qj = diff[1];
            qk = diff[2];
            qr = diff[3];
            s_a = sqrt((qi*qi)+(qj*qj)+(qk*qk));
            angle = 2 * atan2(s_a, qr);
            ax = qi/s_a;
            ay = qj/s_a;
            az = qk/s_a;
            result.push_back(angle*ax); //TitaUx
            result.push_back(angle*ay); //TitaUy
            result.push_back(angle*az); //TitaUz

          }
          else{
            result.push_back(0.0);
            result.push_back(0.0);
            result.push_back(0.0);
          }

        }
      }
    }
  }

  return result;
}

Eigen::MatrixXd obtainHybridIteractionMatrix(std::vector<double> v, std::vector<double> z, std::vector<double> a){
  Eigen::MatrixXd result(v.size(),6); //rows = 6 * (v/6)
  Eigen::MatrixXd Ux(3,3);
  Eigen::MatrixXd Ltitau;
  Eigen::MatrixXd I_(3,3);
  I_ << Eigen::MatrixXd::Identity(3,3);
  int cnt_Z = 0;
  int cnt_A = 0;
  double zp, x, y, sc, sc2, sc_2;
  for(std::size_t i = 0; i < v.size(); i = i + 6){

    zp = z[cnt_Z];
    x = v[i];
    y = v[i+1];

    //Lv
    result(i,0) = -1 / zp;
    result(i,1) = 0;
    result(i,2) = x / zp;
    result(i+1,0) = 0;
    result(i+1,1) = -1 / zp;
    result(i+1,2) = y / zp;
    result(i+2,0) = 0;
    result(i+2,1) = 0;
    result(i+2,2) = -1 / zp;
    
    //0
    result(i+3,0) = 0;
    result(i+3,1) = 0;
    result(i+3,2) = 0;
    result(i+4,0) = 0;
    result(i+4,1) = 0;
    result(i+4,2) = 0;
    result(i+5,0) = 0;
    result(i+5,1) = 0;
    result(i+5,2) = 0;

    //Lw
    result(i,3) = x*y;
    result(i,4) = -(1+(x*x));
    result(i,5) = y;
    result(i+1,3) = 1+(y*y);
    result(i+1,4) = -x*y;
    result(i+1,5) = -x;
    result(i+2,3) = -y;
    result(i+2,4) = x;
    result(i+2,5) = 0;

    //Ltitau
    Ux(0,0) = 0;
    Ux(0,1) = -a[cnt_A+2];
    Ux(0,2) = a[cnt_A+1];
    Ux(1,0) = a[cnt_A+2];
    Ux(1,1) = 0;
    Ux(1,2) = -a[cnt_A];
    Ux(2,0) = -a[cnt_A+1];
    Ux(2,1) = a[cnt_A];
    Ux(2,2) = 0;

    sc = sin(a[cnt_A+3])/a[cnt_A+3];
    sc_2 = sin(a[cnt_A+3]/2)/(a[cnt_A+3]/2);
    sc2 = sc_2 * sc_2;

    Ltitau = I_ - ((a[cnt_A+3]/2)*Ux) + ((1 - (sc/sc2)))*Ux*Ux;

    result(i+3,3) = Ltitau(0,0);
    result(i+3,4) = Ltitau(0,1);
    result(i+3,5) = Ltitau(0,2);
    result(i+4,3) = Ltitau(1,0);
    result(i+4,4) = Ltitau(1,1);
    result(i+4,5) = Ltitau(1,2);
    result(i+5,3) = Ltitau(2,0);
    result(i+5,4) = Ltitau(2,1);
    result(i+5,5) = Ltitau(2,2);

    cnt_Z++;
    cnt_A = cnt_A + 4;
  }

  return result;
}

std::vector<double> obtainCalibratedPoints(std::vector<double> points2d, Eigen::MatrixXd M){
  std::vector<double> result;
  for(std::size_t i = 0; i < points2d.size(); i = i+2){
    Eigen::VectorXd p2d(3);
    p2d(0) = points2d[i];
    p2d(1) = points2d[i+1];
    p2d(2) = 1.0;
    Eigen::VectorXd p3d = M.inverse() * p2d;
    result.push_back(p3d[0]);
    result.push_back(p3d[1]);
  }
  
  return result;
}

std::vector<double> obtainZ(my_ur10_msgs::ArucoMarkerArray a_array, std::vector<int> ids){
  std::vector<double> result;
  for(std::size_t i = 0; i < a_array.markers.size(); ++i){
    for(std::size_t d = 0; d < ids.size(); ++d){
      if(a_array.markers[i].id == ids[d]){
        for(std::size_t j = 0; j < 4; ++j){
          result.push_back(a_array.markers[i].pose.position.z);
        }
      }
    }
    
    
  }
  return result;
}

std::vector<double> diff_Vector(std::vector<double> a, std::vector<double> d){
  std::vector<double> result;
  for(std::size_t i = 0; i < a.size(); i++){
    result.push_back(a[i] - d[i]);
  }
  return result;
}

Eigen::VectorXd toEigenVector(std::vector<double> p){
  Eigen::VectorXd result(p.size());
  for(std::size_t i = 0; i < p.size(); i++){
    result(i) = p[i];
  }
  return result;
}


Eigen::MatrixXd computeV(tf::StampedTransform T){
  Eigen::MatrixXd result(6,6);
  tf::Matrix3x3 R = T.getBasis();
  Eigen::MatrixXd Re(3,3);
  tf::Vector3 x = T.getOrigin();
  Eigen::MatrixXd tx(3,3);
  Eigen::MatrixXd prod;
  
  Re(0,0) = R[0][0];
  Re(0,1) = R[0][1];
  Re(0,2) = R[0][2];
  Re(1,0) = R[1][0];
  Re(1,1) = R[1][1];
  Re(1,2) = R[1][2];
  Re(2,0) = R[2][0];
  Re(2,1) = R[2][1];
  Re(2,2) = R[2][2];

  result(0,0) = Re(0,0);
  result(0,1) = Re(0,1);
  result(0,2) = Re(0,2);
  result(1,0) = Re(1,0);
  result(1,1) = Re(1,1);
  result(1,2) = Re(1,2);
  result(2,0) = Re(2,0);
  result(2,1) = Re(2,1);
  result(2,2) = Re(2,2);

  result(3,3) = Re(0,0);
  result(3,4) = Re(0,1);
  result(3,5) = Re(0,2);
  result(4,3) = Re(1,0);
  result(4,4) = Re(1,1);
  result(4,5) = Re(1,2);
  result(5,3) = Re(2,0);
  result(5,4) = Re(2,1);
  result(5,5) = Re(2,2);

  result(3,0) = 0;
  result(3,1) = 0;
  result(3,2) = 0;
  result(4,0) = 0;
  result(4,1) = 0;
  result(4,2) = 0;
  result(5,0) = 0;
  result(5,1) = 0;
  result(5,2) = 0;

  tx(0,0) = 0;
  tx(0,1) = -x[2];
  tx(0,2) = x[1];
  tx(1,0) = x[2];
  tx(1,1) = 0;
  tx(1,2) = -x[0];
  tx(2,0) = -x[1];
  tx(2,1) = x[0];
  tx(2,2) = 0;

  prod = tx * Re;

  result(0,3) = prod(0,0);
  result(0,4) = prod(0,1);
  result(0,5) = prod(0,2);
  result(1,3) = prod(1,0);
  result(1,4) = prod(1,1);
  result(1,5) = prod(1,2);
  result(2,3) = prod(2,0);
  result(2,4) = prod(2,1);
  result(2,5) = prod(2,2);

  return result;
}

Eigen::VectorXd applyGains(Eigen::VectorXd g, Eigen::VectorXd v){
  Eigen::VectorXd result(6);
  for(std::size_t i = 0; i < 6; i++){
    result[i] = -g[i] * v[i];
  }

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
  ros::init(argc, argv, "ros_HVC");
  ros::NodeHandle n;
  ros::Publisher velocity_pub = n.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1000);
  ros::Publisher plot_pub = n.advertise<my_ur10_msgs::plot_IBVC>("/all_plots_HVC", 1000);
  ros::Publisher cyclic_pub = n.advertise<std_msgs::UInt32>("/cyclic_num", 1);
  ros::Subscriber desired_sub = n.subscribe("/desired/markers_pose", 1, desiredMarkersPoseCallback);
  ros::Subscriber actual_sub = n.subscribe("/aruco_multiple/markers_pose", 1, actualMarkersPoseCallback);
  ros::Subscriber cameraInfo_sub = n.subscribe("/camera/color/camera_info", 1, cameraInfoCallback);
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

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  
  my_ur10_msgs::ArucoMarkerArray c_desired_markers_pose = desired_markers_pose;
  my_ur10_msgs::ArucoMarkerArray c_actual_markers_pose = actual_markers_pose;
  std::vector<double> joint_values, actual_rpy, old_actual_rpy, diff_s, p_o_errors, p_o_errors_bl;
  std::vector<double> s = obtainFeaturesImage(c_actual_markers_pose, c_desired_markers_pose, K, true);
  std::vector<double> d_s = obtainFeaturesImage(c_desired_markers_pose, c_actual_markers_pose, K, false);
  std::vector<double> AngAAxis = obtainAngleAndAxis(c_actual_markers_pose, c_desired_markers_pose);
  std::vector<int> ids_marks = obtainIdMarks(c_actual_markers_pose);
  std::vector<int> ids_marks_desired = obtainIdMarks(c_desired_markers_pose);
  std::vector<double> Z = obtainZ(c_actual_markers_pose, ids_marks_desired);
  std::vector<double> d_Z = obtainZ(c_desired_markers_pose, ids_marks);

  
  tf::TransformListener listener;
  sleep(1);
  tf::StampedTransform T, T_B_C;
  tf::Transform T_A_c, T_A;;

  bool keep = false;
  float m_error = 0.01;
  Eigen::VectorXd vel_camera(6);
  Eigen::VectorXd velocities(6);
  Eigen::VectorXd joint_velocities(6);
  Eigen::VectorXd limited_joint_velocities(6);
  Eigen::VectorXd gains(6);
  gains[0] = 0.05;
  gains[1] = 0.05;
  gains[2] = 0.05;
  gains[3] = 0.05;
  gains[4] = 0.05;
  gains[5] = 0.05;
  Eigen::MatrixXd J, J_pi, L, L_pi, V;
  std::vector<double> max_vel = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
  std::vector<geometry_msgs::Pose> waypoints;
  int cnt = 0;
  geometry_msgs::Pose actual_pose, actual_pose_c;
  Eigen::VectorXd diff_E;
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

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
      my_ur10_msgs::plot_IBVC plots;
      actual_pose_c = move_group.getCurrentPose().pose;
      T_A_c = fromPoseToTransform(actual_pose_c);
      T_A = T_B_C * T_A_c;
      actual_pose = fromTransformToPose(T_A);
      actual_rpy = QuaternionToEulerAngles(actual_pose);
      kinematic_state = move_group.getCurrentState();
      joint_values = move_group.getCurrentJointValues();
      waypoints.push_back(actual_pose);

    

      plots.j.push_back(joint_values[0]);
      plots.j.push_back(joint_values[1]);
      plots.j.push_back(joint_values[2]);
      plots.j.push_back(joint_values[3]);
      plots.j.push_back(joint_values[4]);
      plots.j.push_back(joint_values[5]);
      
      

      plots.p.push_back(actual_pose.position.x);
      plots.p.push_back(actual_pose.position.y);
      plots.p.push_back(actual_pose.position.z);
      
      actual_rpy = sameSingInAngle(actual_rpy, old_actual_rpy);
      plots.p.push_back(actual_rpy[0]);
      plots.p.push_back(actual_rpy[1]);
      plots.p.push_back(actual_rpy[2]);

      old_actual_rpy = actual_rpy;

      c_desired_markers_pose = desired_markers_pose;
      c_actual_markers_pose = actual_markers_pose;
      s = obtainFeaturesImage(c_actual_markers_pose, c_desired_markers_pose, K, true);
      d_s = obtainFeaturesImage(c_desired_markers_pose, c_actual_markers_pose, K, false);
      AngAAxis = obtainAngleAndAxis(c_actual_markers_pose, c_desired_markers_pose);
      ids_marks = obtainIdMarks(c_actual_markers_pose);
      Z = obtainZ(c_actual_markers_pose, ids_marks_desired);
      d_Z = obtainZ(c_desired_markers_pose, ids_marks);
      L = obtainHybridIteractionMatrix(s, Z, AngAAxis);

      L_pi = PseudoinverseMoorePenrose(L, false);
      diff_s = diff_Vector(s, d_s);
      diff_E = toEigenVector(diff_s);
      vel_camera = L_pi * diff_E;
      vel_camera = applyGains(gains, vel_camera);
      keep = BadGoalPositionHVC(diff_s, m_error);
      listener.lookupTransform("base_link", "camera_color_optical_frame", ros::Time(0), T);
      
      kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, J);
      J_pi = PseudoinverseMoorePenrose(J, true);

      V = computeV(T);
      velocities = V * vel_camera;

      
      if(!keep){
        plots.finish = false;
      
      }
      else{
        for(std::size_t i = 0; i < velocities.size(); ++i){

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

      plots.v_c.push_back(vel_camera[0]);
      plots.v_c.push_back(vel_camera[1]);
      plots.v_c.push_back(vel_camera[2]);
      plots.v_c.push_back(vel_camera[3]);
      plots.v_c.push_back(vel_camera[4]);
      plots.v_c.push_back(vel_camera[5]);

      for(std::size_t i = 0; i < ids_marks_desired.size(); ++i){
        plots.ids.push_back(ids_marks_desired[i]);
      }

      plots.t = ros::Time::now().toSec();

      //error and movement of the aruco markers
      for(std::size_t j = 0; j < c_desired_markers_pose.markers.size(); ++j){ 
        int cnt_d = 0;
        for(std::size_t i = 0; i < c_actual_markers_pose.markers.size(); ++i){    

          if(c_actual_markers_pose.markers[i].id == c_desired_markers_pose.markers[j].id){

            my_ur10_msgs::e_m_plot e_m_plot;
            e_m_plot.id = c_actual_markers_pose.markers[i].id;
            // errors
            std::vector<double> a, b, a_c, b_c, c;
            for(std::size_t k = 0; k < c_actual_markers_pose.markers[i].img_points.size(); ++k){
              a.push_back(c_actual_markers_pose.markers[i].img_points[k].x);
              a.push_back(c_actual_markers_pose.markers[i].img_points[k].y);
            }
            for(std::size_t k = 0; k < c_desired_markers_pose.markers[j].img_points.size(); ++k){
              b.push_back(c_desired_markers_pose.markers[j].img_points[k].x);
              b.push_back(c_desired_markers_pose.markers[j].img_points[k].y);
            }
            a_c = obtainCalibratedPoints(a, K);
            b_c = obtainCalibratedPoints(b, K);
            c = diff_Vector(a_c, b_c);
            
            e_m_plot.errors = c;

            e_m_plot.point_movements = a;

            plots.m_info.push_back(e_m_plot);

            cnt_d = 0;
          }
          else{
            cnt_d++;
          }

          if(cnt_d == c_actual_markers_pose.markers.size()){
            my_ur10_msgs::e_m_plot e_m_plot;
            e_m_plot.id = 0;
            plots.m_info.push_back(e_m_plot);
          }
        }

      }

      plots.i_dim.push_back(H);
      plots.i_dim.push_back(W);

      plot_pub.publish(plots);

      cnt++;

      if(cnt<=50){
          ROS_INFO_STREAM("joint velocities 0: " << joint_velocities[0] << " 1: " << joint_velocities[1] << " 2: " << joint_velocities[2] << " 3: " << joint_velocities[3] << " 4: " << joint_velocities[4] << " 5: " << joint_velocities[5]);
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

      if(limited_joint_velocities[2] < 0.001 && limited_joint_velocities[2] > -0.001 && limited_joint_velocities[2] != 0.0){
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
      ROS_INFO_STREAM("The camera cannot see more than 1 ArUco marker, robot stopped");

    }

    
    
  }
  
  
  visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
  visual_tools.trigger();

  sleep(2);
  

  ros::shutdown();
  return 0;
}