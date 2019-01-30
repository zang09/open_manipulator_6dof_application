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

namespace open_manipulator_motion
{

class OM_MOTION
{
private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Message
  ros::Publisher open_manipulator_motion_state_pub_;
  ros::Subscriber open_manipulator_kinematics_pose_sub_;
  ros::Subscriber open_manipulator_gui_button_sub_;
  ros::Subscriber open_manipulator_ar_marker_sub_;

  // ROS Service
  ros::ServiceClient goal_joint_space_path_to_kinematics_pose_client_;

  open_manipulator_msgs::KinematicsPose kinematics_pose_;
  Eigen::Vector3d kinematics_orientation_rpy_;

public:
  OM_MOTION();
  ~OM_MOTION();

  void initPublisher();
  void initSubscriber();
  void initClient();
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void motionStatesCallback(const std_msgs::Bool::ConstPtr &msg);
  void markerPosCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  bool setJointSpacePathToKinematicsPose(std::vector<double> kinematics_pose, double path_time);

public:
  bool motion_flag = false;

};
}

#endif //OPEN_MANIPULATOR_MOTION_H
