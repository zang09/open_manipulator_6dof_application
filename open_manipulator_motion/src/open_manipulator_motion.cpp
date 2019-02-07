/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
#include "open_manipulator_motion/open_manipulator_motion.h"

using namespace open_manipulator_motion;
using namespace robotis_manipulator;
using namespace std;

OpenManipulatorMotion::OpenManipulatorMotion()
  :node_handle_("")
{
  initPublisher();
  initSubscriber();
  initClient();
}

OpenManipulatorMotion::~OpenManipulatorMotion()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void OpenManipulatorMotion::initPublisher()
{
  open_manipulator_motion_state_pub_ = node_handle_.advertise<open_manipulator_motion::MotionState>("motion_state", 10);
}

void OpenManipulatorMotion::initSubscriber()
{
  open_manipulator_states_sub_          = node_handle_.subscribe("states", 10, &OpenManipulatorMotion::manipulatorStatesCallback, this);
  open_manipulator_kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorMotion::kinematicsPoseCallback, this);
  open_manipulator_gui_button_sub_      = node_handle_.subscribe("button_clicked", 10, &OpenManipulatorMotion::motionStatesCallback, this);
  open_manipulator_ar_marker_sub_       = node_handle_.subscribe("/ar_pose_marker", 10, &OpenManipulatorMotion::markerPosCallback, this);
}

void OpenManipulatorMotion::initClient()
{
  goal_joint_space_path_client_                        = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_to_kinematics_pose_client_     = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_joint_space_path_to_kinematics_pose");
  goal_joint_space_path_to_kinematics_position_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_joint_space_path_to_kinematics_position");
  goal_tool_control_client_                            = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
}

void OpenManipulatorMotion::timerCallback(const ros::TimerEvent& event)
{
  if(motion_flag)
  {
    switch(motion_case)
    {
    case HOME_POSE:
      if(send_flag)
      {
        sendJointAngle(0.0, -0.78, 1.5, 0.0, 0.8, 0.0);

        if(open_manipulator_is_moving_)
          send_flag = false;
      }
      else
      {
        if(!open_manipulator_is_moving_)
        {
          motionWait(1.0);
          send_flag = true;
          motion_case = INIT_POSE;
        }
      }
      break;

    case INIT_POSE:
      if(send_flag)
      {
        sendJointAngle(-1.5, -0.6, 1.4, 0.0, 1.7, 0.0);

        if(open_manipulator_is_moving_)
          send_flag = false;
      }
      else
      {
        if(!open_manipulator_is_moving_)
        {
          motionWait(1.0);
          sendMarkerPose();
          motion_flag = false;
        }
      }
      break;
    }
  }
}

void OpenManipulatorMotion::markerPosCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  if(!(msg->markers.empty()))
  {
    //Get Position
    object_position_.push_back(msg->markers.at(0).pose.pose.position.x);
    object_position_.push_back(msg->markers.at(0).pose.pose.position.y);
    object_position_.push_back(msg->markers.at(0).pose.pose.position.z);

    //Get Orientation
    Eigen::Quaterniond temp_orientation(msg->markers.at(0).pose.pose.orientation.w, msg->markers.at(0).pose.pose.orientation.x, msg->markers.at(0).pose.pose.orientation.y, msg->markers.at(0).pose.pose.orientation.z);
    object_orientation_ = objectOrientationTransformer(temp_orientation);
  }
  else
    ROS_INFO("No markers!");
}

void OpenManipulatorMotion::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  if(msg->open_manipulator_moving_state == msg->IS_MOVING)
    open_manipulator_is_moving_ = true;
  else
    open_manipulator_is_moving_ = false;

  if(msg->open_manipulator_actuator_state == msg->ACTUATOR_ENABLED)
    open_manipulator_actuator_enabled_ = true;
  else
    open_manipulator_actuator_enabled_ = false;
}

void OpenManipulatorMotion::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  Eigen::Quaterniond temp_orientation(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  kinematics_orientation_ = temp_orientation;
  kinematics_orientation_rpy_ = math::convertQuaternionToRPYVector(temp_orientation);
  kinematics_orientation_matrix_ = math::convertQuaternionToRotationMatrix(temp_orientation);

  kinematics_pose_.pose = msg->pose;
}

void OpenManipulatorMotion::motionStatesCallback(const std_msgs::Bool::ConstPtr &msg)
{
  motion_flag = msg->data;

  if(motion_flag)
  {
    motion_case = HOME_POSE;
    send_flag = true;
  }
}

void OpenManipulatorMotion::sendJointAngle(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6)
{
  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  static double path_time = 2.0;

  joint_name.push_back("joint1"); joint_angle.push_back(joint1);
  joint_name.push_back("joint2"); joint_angle.push_back(joint2);
  joint_name.push_back("joint3"); joint_angle.push_back(joint3);
  joint_name.push_back("joint4"); joint_angle.push_back(joint4);
  joint_name.push_back("joint5"); joint_angle.push_back(joint5);
  joint_name.push_back("joint6"); joint_angle.push_back(joint6);

  setJointSpacePath(joint_name, joint_angle, path_time);
}
void OpenManipulatorMotion::sendKinematicsPose(double x, double y, double z, double roll, double pitch, double yaw)
{
  std::vector<double> kinematics_pose;
  static double path_time = 2.0;

  Eigen::Quaterniond temp_orientation = math::convertRPYToQuaternion(roll, pitch, yaw);
  kinematics_pose.push_back(x);
  kinematics_pose.push_back(y);
  kinematics_pose.push_back(z);
  kinematics_pose.push_back(temp_orientation.w());
  kinematics_pose.push_back(temp_orientation.x());
  kinematics_pose.push_back(temp_orientation.y());
  kinematics_pose.push_back(temp_orientation.z());

  setJointSpacePathToKinematicsPose(kinematics_pose, path_time);
}

void OpenManipulatorMotion::sendMarkerPose()
{
  std::vector<double> kinematics_pose;
  static double path_time = 2.5;

  kinematics_pose.push_back(object_position_.at(0)+X_OFFSET);
  kinematics_pose.push_back(object_position_.at(1)+Y_OFFSET);
  kinematics_pose.push_back(object_position_.at(2)+Z_OFFSET);
  kinematics_pose.push_back(object_orientation_.w());
  kinematics_pose.push_back(object_orientation_.x());
  kinematics_pose.push_back(object_orientation_.y());
  kinematics_pose.push_back(object_orientation_.z());

  setJointSpacePathToKinematicsPose(kinematics_pose, path_time);
}

void OpenManipulatorMotion::sendGripperAngle(double gripper)
{
  std::vector<double> gripper_angle;
  gripper_angle.push_back(gripper);

  setToolControl(gripper_angle);
}

bool OpenManipulatorMotion::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorMotion::setJointSpacePathToKinematicsPose(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose.at(3);
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose.at(4);
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose.at(5);
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose.at(6);

  srv.request.path_time = path_time;

  if(goal_joint_space_path_to_kinematics_pose_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorMotion::setJointSpacePathToKinematicsPosition(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.path_time = path_time;

  if(goal_joint_space_path_to_kinematics_position_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorMotion::setToolControl(std::vector<double> gripper_pose)
{
  open_manipulator_msgs::SetJointPosition srv;

  srv.request.joint_position.joint_name.push_back("gripper");
  srv.request.joint_position.position = gripper_pose;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

Eigen::Quaterniond OpenManipulatorMotion::objectOrientationTransformer(Eigen::Quaterniond temp_orientation)
{
  Eigen::Matrix3d temp_orientation_matrix;
  Eigen::Matrix3d forward_transform_matrix;
  Eigen::Matrix3d reverse_transform_matrix;
  Eigen::Matrix3d forward_desired_orientation_matrix;
  Eigen::Matrix3d reverse_desired_orientation_matrix;
  Eigen::Matrix3d object_orientation_matrix;
  Eigen::Vector3d object_orientation_rpy;
  Eigen::Quaterniond object_orientation;

  //Get Orientation Matrix
  temp_orientation_matrix = math::convertQuaternionToRotationMatrix(temp_orientation);

  //Calculate
  forward_transform_matrix = math::convertPitchAngleToRotationMatrix(PI);
  reverse_transform_matrix = math::convertRollAngleToRotationMatrix(PI);

  forward_desired_orientation_matrix = temp_orientation_matrix*forward_transform_matrix;
  reverse_desired_orientation_matrix = temp_orientation_matrix*reverse_transform_matrix;

  //Solve
  object_orientation_matrix = orientationSolver(forward_desired_orientation_matrix, reverse_desired_orientation_matrix, kinematics_orientation_matrix_);

  //Result
  object_orientation_rpy = math::convertRotationMatrixToRPYVector(object_orientation_matrix);
  object_orientation = math::convertRPYToQuaternion(object_orientation_rpy.coeff(0,0)+ROLL_OFFSET, object_orientation_rpy.coeff(1,0)+PITCH_OFFSET, object_orientation_rpy.coeff(2,0));

  return object_orientation;
}

Eigen::Matrix3d OpenManipulatorMotion::orientationSolver(Eigen::Matrix3d desired_orientation1, Eigen::Matrix3d desired_orientation2, Eigen::Matrix3d present_orientation)
{
  Eigen::Vector3d solution1;
  Eigen::Vector3d solution2;
  double solution1_value;
  double solution2_value;

  solution1 = math::orientationDifference(desired_orientation1, present_orientation);
  solution2 = math::orientationDifference(desired_orientation2, present_orientation);

  solution1_value = solution1.transpose() * solution1;
  solution2_value = solution2.transpose() * solution2;

  if(solution1_value < solution2_value)
    return desired_orientation1;
  else
    return desired_orientation2;
}

void OpenManipulatorMotion::motionWait(double second)
{
  second = second*1000000;
  usleep(second);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_motion");
  ros::NodeHandle n;

  OpenManipulatorMotion om_motion;

  ROS_INFO("OpenManipulator Motion Node");

  ros::Timer motion_timer = n.createTimer(ros::Duration(0.01), &OpenManipulatorMotion::timerCallback, &om_motion);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
