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
  open_manipulator_kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorMotion::kinematicsPoseCallback, this);
  open_manipulator_gui_button_sub_ = node_handle_.subscribe("button_clicked", 10, &OpenManipulatorMotion::motionStatesCallback, this);
  open_manipulator_ar_marker_sub_ = node_handle_.subscribe("/ar_pose_marker", 10, &OpenManipulatorMotion::markerPosCallback, this);
}

void OpenManipulatorMotion::initClient()
{
  goal_joint_space_path_to_kinematics_pose_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_joint_space_path_to_kinematics_pose");
}

void OpenManipulatorMotion::markerPosCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  //Eigen::Vector3d temp_orientation_rpy;
  //Eigen::Vector3d object_orientation_rpy;
  Eigen::Matrix3d temp_orientation_matrix;
  Eigen::Matrix3d forward_transform_matrix;
  Eigen::Matrix3d reverse_transform_matrix;
  Eigen::Matrix3d forward_desired_orientation_matrix;
  Eigen::Matrix3d reverse_desired_orientation_matrix;
  Eigen::Matrix3d object_orientation_matrix;
  Eigen::Quaterniond object_orientation;
  std::vector<double> object_position;
  std::vector<double> kinematics_pose;

  if(!(msg->markers.empty()))
  {
    //Get Position
    object_position.push_back(msg->markers.at(0).pose.pose.position.x);
    object_position.push_back(msg->markers.at(0).pose.pose.position.y);
    object_position.push_back(msg->markers.at(0).pose.pose.position.z);

    //Get Orientation
    Eigen::Quaterniond temp_orientation(msg->markers.at(0).pose.pose.orientation.w, msg->markers.at(0).pose.pose.orientation.x, msg->markers.at(0).pose.pose.orientation.y, msg->markers.at(0).pose.pose.orientation.z);
    //temp_orientation_rpy = math::convertQuaternionToRPYVector(temp_orientation);
    temp_orientation_matrix = math::convertQuaternionToRotationMatrix(temp_orientation);

    //Calculate
    forward_transform_matrix = math::convertPitchAngleToRotationMatrix(PI);
    reverse_transform_matrix = math::convertRollAngleToRotationMatrix(PI);

    forward_desired_orientation_matrix = temp_orientation_matrix*forward_transform_matrix;
    reverse_desired_orientation_matrix = temp_orientation_matrix*reverse_transform_matrix;

    //Solve
    object_orientation_matrix = orientationSolver(forward_desired_orientation_matrix, reverse_desired_orientation_matrix, kinematics_orientation_matrix_);

    //Result
//    object_orientation_rpy = math::convertRotationMatrixToRPYVector(object_orientation_matrix);
    object_orientation = math::convertRotationMatrixToQuaternion(object_orientation_matrix);

//    cout << "R: " << object_orientation_rpy.coeff(0,0) << endl;
//    cout << "P: " << object_orientation_rpy.coeff(1,0) << endl;
//    cout << "Y: " << object_orientation_rpy.coeff(2,0) << endl << endl;

    if(motion_flag)
    {
      double path_time = 2.5;
      kinematics_pose.push_back(object_position.at(0));
      kinematics_pose.push_back(object_position.at(1));
      kinematics_pose.push_back(object_position.at(2)+Z_OFFSET);
      kinematics_pose.push_back(object_orientation.w());
      kinematics_pose.push_back(object_orientation.x());
      kinematics_pose.push_back(object_orientation.y());
      kinematics_pose.push_back(object_orientation.z());

      if(!setJointSpacePathToKinematicsPose(kinematics_pose, path_time))
      {
        cout << "Fail Service!" << endl;
        return;
      }
      motion_flag = 0;
    }
  }
  else
  {
    if(motion_flag)
      ROS_INFO("No markers!");

    motion_flag = 0;
  }
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

//  cout << "solution1: " << solution1_value << endl;
//  cout << "solution2: " << solution2_value << endl << endl;

  if(solution1_value <= solution2_value)
    return desired_orientation1;
  else
    return desired_orientation2;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_motion");

  OpenManipulatorMotion openmanipulatormotion;

  ROS_INFO("OpenManipulator Motion Node");

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
