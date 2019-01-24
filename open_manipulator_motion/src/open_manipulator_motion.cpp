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

OM_MOTION::OM_MOTION()
  :node_handle_("")
{}

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

void OM_MOTION::motionStatesCallback(const std_msgs::Bool::ConstPtr& msg)
{
  std::cout << "Data: " << (int)msg->data << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_motion");
  ROS_INFO("OpenManipulator 6DOF Motion node");

  OM_MOTION om_motion;

  om_motion.initPublisher();
  om_motion.initSubscriber();

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
