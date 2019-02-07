#ifndef OPEN_MANIPULATOR_MOTION_H
#define OPEN_MANIPULATOR_MOTION_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/JointState.h>

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"
#include "robotis_manipulator/robotis_manipulator.h"
#include "open_manipulator_motion/MotionState.h"

#define  NUM_OF_JOINT_AND_TOOL 7

#define  X_OFFSET      -0.015
#define  Y_OFFSET      -0.005
#define  Z_OFFSET       0.03

#define  ROLL_OFFSET   0.03
#define  PITCH_OFFSET  0.045

#define  INIT_POSE        0
#define  HOME_POSE        1
#define  READY_TO_PICKUP  2
#define  READY_TO_PUTDOWN 3
#define  MODE_PICKUP      4
#define  MODE_PUTDOWN     5
#define  MODE_SCAN        100
#define  MODE_END         1000

namespace open_manipulator_motion
{

class OpenManipulatorMotion
{
private:
  // ROS Message
  ros::Publisher  open_manipulator_motion_state_pub_;
  ros::Subscriber open_manipulator_states_sub_;
  ros::Subscriber open_manipulator_kinematics_pose_sub_;
  ros::Subscriber open_manipulator_gui_button_sub_;
  ros::Subscriber open_manipulator_ar_marker_sub_;

  // ROS Service
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_joint_space_path_to_kinematics_pose_client_;
  ros::ServiceClient goal_joint_space_path_to_kinematics_position_client_;
  ros::ServiceClient goal_tool_control_client_;

  // Kinematics variable
  open_manipulator_msgs::KinematicsPose kinematics_pose_;
  Eigen::Quaterniond kinematics_orientation_;
  Eigen::Vector3d    kinematics_orientation_rpy_;
  Eigen::Matrix3d    kinematics_orientation_matrix_;

  // Marker variable
  std::vector<double> object_position_;
  Eigen::Quaterniond object_orientation_;

public:
  OpenManipulatorMotion();
  ~OpenManipulatorMotion();  

  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  void initPublisher();
  void initSubscriber();
  void initClient();
  void timerCallback(const ros::TimerEvent& event);
  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void motionStatesCallback(const std_msgs::Bool::ConstPtr &msg);
  void markerPosCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void sendJointAngle(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6);
  void sendKinematicsPose(double x, double y, double z, double roll, double pitch, double yaw);
  void sendMarkerPose();
  void sendGripperAngle(double gripper);
  void motionWait(double second);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePathToKinematicsPose(std::vector<double> kinematics_pose, double path_time);
  bool setJointSpacePathToKinematicsPosition(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> gripper_pose);
  Eigen::Quaterniond objectOrientationTransformer(Eigen::Quaterniond temp_orientation);
  Eigen::Matrix3d orientationSolver(Eigen::Matrix3d desired_orientation1, Eigen::Matrix3d desired_orientation2, Eigen::Matrix3d present_orientation);

public:  
  bool open_manipulator_is_moving_;
  bool open_manipulator_actuator_enabled_;
  bool motion_flag = false;
  bool send_flag = true;
  int motion_case = HOME_POSE;
};
}

#endif //OPEN_MANIPULATOR_MOTION_H
