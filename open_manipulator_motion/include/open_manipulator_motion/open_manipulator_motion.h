#ifndef OPEN_MANIPULATOR_MOTION_H
#define OPEN_MANIPULATOR_MOTION_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/JointState.h>

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "visualization_msgs/Marker.h"
#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"
#include "robotis_manipulator/robotis_manipulator.h"
#include "open_manipulator_motion/MotionState.h"

#define  NUM_OF_JOINT_AND_TOOL 7

#define  JOINT1                0
#define  JOINT2                1
#define  JOINT3                2
#define  JOINT4                3
#define  JOINT5                4
#define  JOINT6                5

#define  MOTION_NUM            1
#define  LAYER_NUM             1

#define  INIT_POSE             1
#define  MODE_MARKER_DETECT    2
#define  MODE_CAM_INIT         3
#define  READY_TO_PICKUP       4
#define  MODE_PICKUP           5
#define  HOME_POSE             6
#define  READY_TO_PUTDOWN      7
#define  MODE_PUTDOWN          8
#define  HOME_POSE2            9
#define  MODE_SCAN             10
#define  MODE_FAIL_MOTION      100
#define  MODE_END              1000

namespace open_manipulator_motion
{

class OpenManipulatorMotion
{
private:
  // ROS Message
  ros::Publisher  open_manipulator_motion_state_pub_;
  ros::Subscriber open_manipulator_states_sub_;
  ros::Subscriber open_manipulator_joint_states_sub_;
  ros::Subscriber open_manipulator_kinematics_pose_sub_;
  ros::Subscriber open_manipulator_gui_button_sub_;
  ros::Subscriber open_manipulator_ar_marker_sub_;
  ros::Subscriber open_manipulator_visualization_marker_sub_;

  // ROS Service
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_joint_space_path_to_kinematics_pose_client_;
  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_client_;
  ros::ServiceClient goal_tool_control_client_;

  // Kinematics variable  
  open_manipulator_msgs::KinematicsPose kinematics_pose_;
  std::vector<double>                   present_joint_angle_;
  Eigen::Quaterniond                    kinematics_orientation_;
  Eigen::Vector3d                       kinematics_orientation_rpy_;
  Eigen::Matrix3d                       kinematics_orientation_matrix_;

  // Marker variable
  std::vector<double> marker_position_;
  Eigen::Quaterniond  marker_orientation_;

  // Camera variable
  int    marker_id_;
  double camera_x_;
  double camera_y_;
  double pan_position_;
  double tilt_position_;

  // Flag parameter
  bool open_manipulator_is_moving_;
  bool open_manipulator_actuator_enabled_;
  bool marker_exist_;


public:
  OpenManipulatorMotion();
  ~OpenManipulatorMotion();

  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  ros::Timer      motion_timer;

  void initPublisher();
  void initSubscriber();
  void initClient();
  void initValue();
  void motionStatesPublisher(int motion_state);
  void timerCallback(const ros::TimerEvent &);
  void visualMarkerCallback(const visualization_msgs::Marker::ConstPtr &msg);
  void markerPosCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
  void buttonStatesCallback(const std_msgs::Bool::ConstPtr &msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void sendJointAngle(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, double path_time);
  void sendJointFromPresent(int joint_num, double delta, double path_time);
  void sendPoseFromPresent(double delta_x, double delta_y, double delta_z);
  void sendPanTiltFromPresent(double delta_x, double delta_y);
  void sendEndEffectorFromPresent(Eigen::Quaterniond orientation, double delta_z);
  void sendMarkerPose(std::vector<double> position, Eigen::Quaterniond transform_orientation, double delta_x, double delta_y, double delta_z);
  void sendGripperAngle(double gripper);
  void motionWait(double second);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePathToKinematicsPose(std::vector<double> kinematics_pose, double path_time);
  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresent(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> gripper_pose);
  Eigen::Quaterniond markerOrientationTransformer(Eigen::Quaterniond marker_orientation);
  Eigen::Matrix3d orientationSolver(Eigen::Matrix3d desired_orientation1, Eigen::Matrix3d desired_orientation2, Eigen::Matrix3d present_orientation);


public:
  bool send_flag;
  bool motion_flag;
  int  motion_case;
  int  motion_cnt;
  int  layer_cnt;
  int  repeat_motion_cnt;
  int  solution_flag;
  int  get_marker_id;
  int  pan_flag;
  int  tilt_flag;
};
}

#endif //OPEN_MANIPULATOR_MOTION_H
