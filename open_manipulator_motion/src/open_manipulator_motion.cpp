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
using namespace std;

OM_MOTION::OM_MOTION()
  :node_handle_("")
{
  initPublisher();
  initSubscriber();
  initClient();
}

OM_MOTION::~OM_MOTION()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void OM_MOTION::initPublisher()
{
  open_manipulator_motion_state_pub_ = node_handle_.advertise<open_manipulator_motion::MotionState>("motion_state", 10);
}

void OM_MOTION::initSubscriber()
{
  open_manipulator_gui_button_sub_ = node_handle_.subscribe("/open_manipulator_6dof/button_clicked", 10, &OM_MOTION::motionStatesCallback, this);
}

void OM_MOTION::initClient()
{
  goal_joint_space_path_to_kinematics_pose_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_joint_space_path_to_kinematics_pose");
}

void OM_MOTION::motionStatesCallback(const std_msgs::Bool::ConstPtr &msg)
{
  std::cout << "Data: " << (int)msg->data << std::endl;

  std::vector<double> kinematics_pose;

  kinematics_pose.push_back(0.0);
  kinematics_pose.push_back(0.0);
  kinematics_pose.push_back(0.467);
  kinematics_pose.push_back(1.0);
  kinematics_pose.push_back(0.0);
  kinematics_pose.push_back(0.0);
  kinematics_pose.push_back(0.0);

  if(!setJointSpacePathToKinematicsPose(kinematics_pose, 2.0))
  {
    cout << "Fail Service!" << endl;
    return;
  }
}

bool OM_MOTION::setJointSpacePathToKinematicsPose(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose.at(3); //kinematics_pose_.pose.orientation.w;
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose.at(4); //kinematics_pose_.pose.orientation.x;
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose.at(5); //kinematics_pose_.pose.orientation.y;
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose.at(6); //kinematics_pose_.pose.orientation.z;

  srv.request.path_time = path_time;

  cout << "call" << endl;

  if(goal_joint_space_path_to_kinematics_pose_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_motion");

  OM_MOTION om_motion;

  ROS_INFO("OpenManipulator Motion Node");

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
