#ifndef OPEN_MANIPULATOR_MOTION_H
#define OPEN_MANIPULATOR_MOTION_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>

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

#define  JOINT1             1
#define  JOINT2             2
#define  JOINT3             3
#define  JOINT4             4
#define  JOINT5             5
#define  JOINT6             6

#define  X_AXIS             0
#define  Y_AXIS             1
#define  Z_AXIS             2

#define  X_OFFSET           0.0
#define  Y_OFFSET           0.0
#define  Z_OFFSET           0.0
#define  ROLL_OFFSET        0.0
#define  PITCH_OFFSET       0.0

#define  MOTION_NUM         1
#define  LAYER_NUM          1

#define  INIT_POSE          0
#define  HOME_POSE          1
#define  HOME_POSE2         2
#define  READY_TO_PICKUP    3
#define  READY_TO_PUTDOWN   4
#define  MODE_PICKUP        5
#define  MODE_PUTDOWN       6
#define  ORIENTATION_CHECK  7
#define  MODE_MARKER_DETECT 10
#define  MODE_CAM_INIT      50
#define  MODE_SCAN          100
#define  MODE_FAIL_MOTION   500
#define  MODE_END           1000

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
  std::vector<double> present_joint_angle_;
  open_manipulator_msgs::KinematicsPose kinematics_pose_;
  Eigen::Quaterniond kinematics_orientation_;
  Eigen::Vector3d    kinematics_orientation_rpy_;
  Eigen::Matrix3d    kinematics_orientation_matrix_;

  // Marker variable
  std::vector<double> marker_position_;
  std::vector<double> transform_marker_position_;
  Eigen::Quaterniond  marker_orientation_;
  Eigen::Quaterniond  transform_marker_orientation_;

  // Thread Parameter
  pthread_t timer_thread_;

  //
  bool timer_thread_state_;
  bool open_manipulator_is_moving_;
  bool open_manipulator_actuator_enabled_;
  bool marker_exist_ = false;
  double camera_x_;
  double camera_y_;
  double pan_position_;
  double tilt_position_;

public:
  OpenManipulatorMotion();
  ~OpenManipulatorMotion();

  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  void startTimerThread();
  static void *timerThread(void *param);

  void initPublisher();
  void initSubscriber();
  void initClient();
  void initValue();
  void motionStatesPublisher(int motion_state);
  void timerCallback();
  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void buttonStatesCallback(const std_msgs::Bool::ConstPtr &msg);
  void markerPosCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void visualMarkerCallback(const visualization_msgs::Marker::ConstPtr &msg);
  void sendJointAngle(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6);  
  void sendJointFromPresent(int joint_num, double delta, double path_time);
  void sendPoseFromPresent(double delta_x, double delta_y, double delta_z);
  void sendEndEffectorFromPresent(Eigen::Quaterniond orientation, double delta_z);
  void sendMarkerPose(std::vector<double> position, Eigen::Quaterniond orientation, double delta_z);
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
  bool motion_flag = false;
  bool send_flag = true;
  int motion_state = 0;
  int motion_case = INIT_POSE;
  int motion_cnt = 0;
  int repeat_motion_cnt = 0;
  int layer_cnt = 0;
  int pan_flag = 1;
  int tilt_flag = 0;
};
}

#endif //OPEN_MANIPULATOR_MOTION_H
