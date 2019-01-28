#ifndef OPEN_MANIPULATOR_MOTION_H
#define OPEN_MANIPULATOR_MOTION_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/JointState.h>

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
  ros::Subscriber open_manipulator_gui_button_sub_;

  // ROS Service
  ros::ServiceClient goal_joint_space_path_to_kinematics_pose_client_;

public:
  OM_MOTION();
  ~OM_MOTION();

  void initPublisher();
  void initSubscriber();
  void initClient();
  void motionStatesCallback(const std_msgs::Bool::ConstPtr &msg);
  bool setJointSpacePathToKinematicsPose(std::vector<double> kinematics_pose, double path_time);

public:
  bool motion_flag = false;

};
}

#endif //OPEN_MANIPULATOR_MOTION_H
