#ifndef OPEN_MANIPULATOR_MOTION_H
#define OPEN_MANIPULATOR_MOTION_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <sensor_msgs/JointState.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include "open_manipulator_motion/MotionState.h"

namespace open_manipulator_motion
{

class OM_MOTION
{
private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Publisher
  ros::Publisher open_manipulator_motion_state_pub_;

  // ROS Subscribers
  ros::Subscriber open_manipulator_gui_button_sub_;

public:
  OM_MOTION();
  ~OM_MOTION();

  void initPublisher();
  void initSubscriber();
  void motionStatesCallback(const std_msgs::Bool::ConstPtr &msg);
};
}

#endif //OPEN_MANIPULATOR_MOTION_H
