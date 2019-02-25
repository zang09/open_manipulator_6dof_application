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

OpenManipulatorMotion::OpenManipulatorMotion()
  :node_handle_("")
{
  initPublisher();
  initSubscriber();
  initClient();
  initValue();
}

OpenManipulatorMotion::~OpenManipulatorMotion()
{
  if(ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

void OpenManipulatorMotion::initPublisher()
{
  open_manipulator_motion_state_pub_ = node_handle_.advertise<open_manipulator_motion::MotionState>("motion_state", 10);
  open_manipulator_option_pub_       = node_handle_.advertise<std_msgs::String>("option", 10);
}

void OpenManipulatorMotion::initSubscriber()
{
  open_manipulator_states_sub_               = node_handle_.subscribe("states", 10, &OpenManipulatorMotion::manipulatorStatesCallback, this);
  open_manipulator_joint_states_sub_         = node_handle_.subscribe("joint_states", 10, &OpenManipulatorMotion::jointStatesCallback, this);
  open_manipulator_kinematics_pose_sub_      = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorMotion::kinematicsPoseCallback, this);
  open_manipulator_gui_button_sub_           = node_handle_.subscribe("button_clicked", 10, &OpenManipulatorMotion::buttonStatesCallback, this);
  open_manipulator_ar_marker_sub_            = node_handle_.subscribe("/ar_pose_marker", 10, &OpenManipulatorMotion::markerPosCallback, this);
  open_manipulator_visualization_marker_sub_ = node_handle_.subscribe("/visualization_marker", 10, &OpenManipulatorMotion::visualMarkerCallback, this);
}

void OpenManipulatorMotion::initClient()
{
  goal_joint_space_path_client_                    = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_to_kinematics_pose_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_joint_space_path_to_kinematics_pose");
  goal_joint_space_path_from_present_client_       = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_client_        = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present");
  goal_tool_control_client_                        = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  goal_drawing_trajectory_client_                  = node_handle_.serviceClient<open_manipulator_msgs::SetDrawingTrajectory>("goal_drawing_trajectory");
}

void OpenManipulatorMotion::initValue()
{
  marker_exist_ = false;
  get_marker_id = 1000;
  send_flag     = true;
  pan_flag      = 1;
  tilt_flag     = 0;
  solution_flag = 0;
  motion_case   = HOME_POSE2;  //INIT_POSE
  motion_cnt    = 10;   //MOTION_NUM
  layer_cnt     = 3; //LAYER_NUM
  repeat_motion_cnt = 0;
}

void OpenManipulatorMotion::motionStatesPublisher(int motion_state)
{
  open_manipulator_motion::MotionState msg;
  msg.motion_state = motion_state;
  open_manipulator_motion_state_pub_.publish(msg);
}

void OpenManipulatorMotion::optionPublisher(std::string opt)
{
  std_msgs::String msg;
  msg.data = opt;
  open_manipulator_option_pub_.publish(msg);
}

void OpenManipulatorMotion::timerCallback(const ros::TimerEvent&)
{
  static std::vector<double> temp_position;
  static Eigen::Quaterniond temp_orientation;

  static int marker_cnt = 0;
  static int missing_cnt = 0;
  static int IK_fail_cnt = 0;

  static double temp_camera_x = 0.0;
  static double temp_camera_y = 0.0;
  static double goal_camera_x = 0.0;
  static double goal_camera_y = 0.0;

  if(motion_flag)
  {
    switch(motion_case)
    {
    case INIT_POSE:
      if(send_flag)
      {
        sendJointAngle(-PI/2, -0.6, 1.52, 0.0, 2.0, 0.0, 2.0);
        sendGripperAngle(0.003);  //0.005
        if(open_manipulator_is_moving_)
          send_flag = false;
      }
      else if(!open_manipulator_is_moving_)
      {
        motionWait(0.5);
        send_flag = true;
        ROS_INFO("INIT");
        motion_case = MODE_MARKER_DETECT;
      }
      break;


    case MODE_MARKER_DETECT:
      if(marker_exist_)
      {
        marker_cnt++;
        missing_cnt = 0;
      }
      else
      {
        missing_cnt++;
        marker_cnt = 0;
      }

      if(marker_cnt > 15)
      {
        ROS_INFO("Detect Marker");
        marker_cnt=0;
        motion_case = MODE_CAM_INIT;
      }
      else if(missing_cnt > 20)
      {
        pan_flag = 1;
        tilt_flag = 0;
        ROS_INFO("Missing");
        missing_cnt = 0;
        motion_case = MODE_SCAN;
      }
      break;


    case MODE_CAM_INIT:
      if(!marker_exist_)
        missing_cnt++;
      else
        missing_cnt = 0;

      if(missing_cnt > 30)
      {
        missing_cnt = 0;
        motion_case = MODE_MARKER_DETECT;
      }

      if(get_marker_id == marker_id_)
      {
        goal_camera_x = 0.0;
        goal_camera_y = 0.0;
        temp_camera_x = camera_x_;
        temp_camera_y = camera_y_;

        sendPanTiltFromPresent(-(temp_camera_x-goal_camera_x)*D2R/5.0, (temp_camera_y-goal_camera_y)*D2R/8.0);

        if(abs(temp_camera_x - goal_camera_x) < 1.0 && abs(temp_camera_y - goal_camera_y) < 1.0) //1.0
        {
          motionWait(1.5);
          motion_case = READY_TO_PICKUP;
        }
      }
      else
        get_marker_id = marker_id_;
      break;

    case READY_TO_PICKUP:
      if(send_flag)
      {        
        temp_position = marker_position_;
        temp_orientation = markerOrientationTransformer(marker_orientation_);
        sendMarkerPose(temp_position, temp_orientation, 0, 0, -0.035);
        if(open_manipulator_is_moving_)
          send_flag = false;
        else
        {
          IK_fail_cnt++;
          if(IK_fail_cnt >= 3)
          {
            ROS_ERROR("Fail to solve IK!");
            IK_fail_cnt = 0;
            motion_case = MODE_FAIL_MOTION;
          }
        }
      }
      else if(!open_manipulator_is_moving_)
      {
        IK_fail_cnt = 0;
        send_flag = true;
        motion_case = MODE_PICKUP;
      }
      break;


    case MODE_PICKUP:
      if(send_flag)
      {
        sendEndEffectorFromPresent(temp_orientation, 0.038);
        if(open_manipulator_is_moving_)
          send_flag = false;
      }
      else if(!open_manipulator_is_moving_)
      {
        //sendJointFromPresent(JOINT5, -0.05, 0.5);
        motionWait(0.5);
        sendGripperAngle(-0.01);
        send_flag = true;
        motion_case = HOME_POSE;
        motionWait(1.0);
      }
      break;


    case HOME_POSE:
      if(send_flag)
      {
        sendEndEffectorFromPresent(temp_orientation, -0.038);
        if(open_manipulator_is_moving_)
          send_flag = false;
      }
      else if(!open_manipulator_is_moving_)
      {
        sendJointAngle(0.0, -0.58, 1.8, 0.0, 0.4, 0.0, 2.0);
        send_flag = true;
        motion_case = READY_TO_PUTDOWN;
        motionWait(1.5);
      }
      break;


    case READY_TO_PUTDOWN:
      if(send_flag)
      {
        if(motion_cnt == 1+4*(layer_cnt-1))
          sendJointAngle(1.733, 0.486, 0.566, -0.015, 2.01, 0.126, 2.0);
        else if(motion_cnt == 2+4*(layer_cnt-1))
          sendJointAngle(1.405, 0.428, 0.637, -0.025, 1.967, -0.2, 2.0);
        else if(motion_cnt == 3+4*(layer_cnt-1))
          sendJointAngle(1.543, 0.606, 0.667, 0.008,   1.8, 1.525, 2.0);
        else if(motion_cnt == 4+4*(layer_cnt-1))
          sendJointAngle(1.513, 0.156, 1.385, 0.014, 1.572, 1.477, 2.0);

        if(open_manipulator_is_moving_)
          send_flag = false;
      }
      else if(!open_manipulator_is_moving_)
      {
        send_flag = true;
        motion_case = MODE_PUTDOWN;
        motionWait(0.5);
      }
      break;


    case MODE_PUTDOWN:
      if(send_flag)
      {
        //DOWN
        if(motion_cnt == 1+4*(layer_cnt-1))
          sendPoseFromPresent(0, 0, -0.085+0.030*(layer_cnt-1));
        else if(motion_cnt == 2+4*(layer_cnt-1))
          sendPoseFromPresent(0, 0, -0.090+0.031*(layer_cnt-1));
        else if(motion_cnt == 3+4*(layer_cnt-1))
          sendPoseFromPresent(0, 0, -0.038+0.028*(layer_cnt-1));
        else if(motion_cnt == 4+4*(layer_cnt-1))
          sendPoseFromPresent(0, 0, -0.042+0.027*(layer_cnt-1));

        if(open_manipulator_is_moving_)
          send_flag = false;
      }
      else if(!open_manipulator_is_moving_)
      {
        motionWait(0.5);
        sendGripperAngle(0.0);
        send_flag = true;
        motion_case = HOME_POSE2;
        motionWait(1.0);
      }
      break;


    case HOME_POSE2:
      if(send_flag)
      {
        //UP
        if(motion_cnt == 1+4*(layer_cnt-1))
          sendPoseFromPresent(0, 0, 0.085-0.030*(layer_cnt-1));
        else if(motion_cnt == 2+4*(layer_cnt-1))
          sendPoseFromPresent(0, 0, 0.090-0.031*(layer_cnt-1));
        else if(motion_cnt == 3+4*(layer_cnt-1))
          sendPoseFromPresent(0, 0, 0.060-0.030*(layer_cnt-1));
        else if(motion_cnt == 4+4*(layer_cnt-1))
        {
          sendPoseFromPresent(0, 0, 0.060-0.030*(layer_cnt-1));
          layer_cnt++;
        }

        if(open_manipulator_is_moving_)
          send_flag = false;
      }
      else if(!open_manipulator_is_moving_)
      {
        //Exceptional
        if(motion_cnt == 9 || motion_cnt == 10)
        {
          sendJointFromPresent(JOINT5, -PI/8, 0.8);
          motionWait(0.8);
        }
        send_flag = true;
        motion_cnt++;

        if(motion_cnt > 10)
        {
          sendJointAngle(0.0, -0.78, 1.5, 0.0, 0.8, 0.0, 2.0);
          motionWait(2.2);
          motion_case = MODE_SUCCESS_MOTION;
        }
        else
        {
          sendJointAngle(0.0, -0.58, 1.8, 0.0, 0.4, 0.0, 2.0);
          motionWait(1.5);
          motion_case = INIT_POSE;
        }
      }
      break;


    case MODE_SCAN:
      if(marker_exist_)
        marker_cnt++;
      else
        marker_cnt = 0;

      if(marker_cnt > 3)
      {
        marker_cnt = 0;
        motion_case = MODE_MARKER_DETECT;
      }

      if(pan_flag == 1)
      {
        sendJointAngle(-PI/2, -0.6, 1.52, 0.0, 2.0, 0.0, 1.5);
        motionWait(1.0);
        pan_flag = 2;
      }
      else if(pan_flag == 2)  // Go To Right
      {
        sendJointFromPresent(JOINT1, -2.5*D2R, 0.1);
        if(pan_position_ < -135.0)
        {
          pan_flag = 0;
          tilt_flag = 1;
        }
      }
      else if(pan_flag == 3)  // Go To Left
      {
        sendJointFromPresent(JOINT1, 2.5*D2R, 0.1);
        if(pan_position_ > -45.0)
        {
          pan_flag = 0;
          tilt_flag = 2;
        }
      }

      if(tilt_flag == 1)      //UP
      {
        sendJointFromPresent(JOINT5, -1.5*D2R, 0.1);
        if(tilt_position_ < 100)
        {
          tilt_flag = 0;
          pan_flag = 3;
        }
      }
      else if(tilt_flag == 2) //DOWN
      {
        sendJointFromPresent(JOINT5, 1.5*D2R, 0.1);
        if(tilt_position_ >= 110)
        {
          tilt_flag = 0;
          pan_flag = 2;
        }
      }
      break;


    case MODE_FAIL_MOTION:
      if(repeat_motion_cnt == 2)
      {
        repeat_motion_cnt = 0;
        send_flag = true;
        pan_flag = 1;
        tilt_flag = 0;
        motion_case = MODE_SCAN;
      }
      else if(send_flag)
      {
        if(repeat_motion_cnt == 0)
        {
          sendJointFromPresent(JOINT1, 10*D2R, 0.15);
          motionWait(0.15);
        }
        else
        {
          sendJointFromPresent(JOINT1, 20*D2R, 0.3);
          motionWait(0.3);
        }
        send_flag = false;
      }
      else
      {
        sendJointFromPresent(JOINT1, -20*D2R, 0.3);
        motionWait(0.3);
        send_flag = true;
        repeat_motion_cnt++;
      }
      break;


    case MODE_SUCCESS_MOTION:
      if(send_flag)
      {        
        optionPublisher("switching_kinematics");
        sendDrawingTrajectory(0.08, 2.0, 0.0, 3.0);        
        send_flag = false;
      }
      else if(!open_manipulator_is_moving_)
      {
        optionPublisher("switching_kinematics");
        sendJointAngle(0.0, -0.78, 1.5, 0.0, 0.8, 0.0, 2.0);
        motion_case = MODE_END;
      }
      break;


    case MODE_END:
      motion_flag = false;
      motionStatesPublisher(MODE_END);
      break;
    }
  }
}

void OpenManipulatorMotion::visualMarkerCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
  camera_x_  = msg->pose.position.x*100.0;
  camera_y_  = msg->pose.position.y*100.0;
  marker_id_ = msg->id;
}

void OpenManipulatorMotion::markerPosCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  std::vector<double> temp_position;
  int get_marker_size;

  if(!(msg->markers.empty()))
  {
    marker_exist_ = true;
    get_marker_size = msg->markers.size();

    //Scan ID
    for(int i=0; i<get_marker_size; i++)
    {
      if(get_marker_id == msg->markers.at(i).id)
      {
        //Get Position
        temp_position.push_back(msg->markers.at(i).pose.pose.position.x);
        temp_position.push_back(msg->markers.at(i).pose.pose.position.y);
        temp_position.push_back(msg->markers.at(i).pose.pose.position.z);
        marker_position_ = temp_position;

        //Get Orientation
        Eigen::Quaterniond temp_orientation(msg->markers.at(i).pose.pose.orientation.w, msg->markers.at(i).pose.pose.orientation.x, msg->markers.at(i).pose.pose.orientation.y, msg->markers.at(i).pose.pose.orientation.z);
        marker_orientation_ = temp_orientation;
        break;
      }
    }
  }
  else
  {
    marker_exist_ = false;
    get_marker_id = 1000;
  }
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

void OpenManipulatorMotion::buttonStatesCallback(const std_msgs::Bool::ConstPtr &msg)
{
  motion_flag = msg->data;

  if(motion_flag)
  {
    motion_timer.start();
    initValue();
  }
  else
    motion_timer.stop();
}

void OpenManipulatorMotion::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL);
  for(int i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint5"))  temp_angle.at(4) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint6"))  temp_angle.at(5) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("gripper"))  temp_angle.at(6) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;

  pan_position_ = temp_angle.at(JOINT1)*R2D;
  tilt_position_ = temp_angle.at(JOINT5)*R2D;
}

void OpenManipulatorMotion::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  Eigen::Quaterniond temp_orientation(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  kinematics_orientation_ = temp_orientation;
  kinematics_orientation_rpy_ = math::convertQuaternionToRPYVector(temp_orientation);
  kinematics_orientation_matrix_ = math::convertQuaternionToRotationMatrix(temp_orientation);

  kinematics_pose_.pose = msg->pose;
}

void OpenManipulatorMotion::sendJointAngle(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, double path_time)
{
  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;

  joint_name.push_back("joint1"); joint_angle.push_back(joint1);
  joint_name.push_back("joint2"); joint_angle.push_back(joint2);
  joint_name.push_back("joint3"); joint_angle.push_back(joint3);
  joint_name.push_back("joint4"); joint_angle.push_back(joint4);
  joint_name.push_back("joint5"); joint_angle.push_back(joint5);
  joint_name.push_back("joint6"); joint_angle.push_back(joint6);

  if(!setJointSpacePath(joint_name, joint_angle, path_time))
  {
    ROS_ERROR("Fail to send service!");
    return;
  }
}

void OpenManipulatorMotion::sendJointFromPresent(int joint_num, double delta, double path_time)
{
  std::vector<double> goalJoint;
  std::vector<std::string> joint_name;

  goalJoint.resize(6, 0.0);
  goalJoint.at(joint_num) = delta;

  joint_name.push_back("joint1");
  joint_name.push_back("joint2");
  joint_name.push_back("joint3");
  joint_name.push_back("joint4");
  joint_name.push_back("joint5");
  joint_name.push_back("joint6");

  if(!setJointSpacePathFromPresent(joint_name, goalJoint, path_time))
  {
    ROS_ERROR("Fail to send service!");
    return;
  }
}

void OpenManipulatorMotion::sendPoseFromPresent(double delta_x, double delta_y, double delta_z)
{
  std::vector<double> goal_pose;
  static double path_time = 1.5;

  goal_pose.resize(7, 0.0);
  goal_pose.at(0) = delta_x;
  goal_pose.at(1) = delta_y;
  goal_pose.at(2) = delta_z;

  if(!setTaskSpacePathFromPresent(goal_pose, path_time))
  {
    ROS_ERROR("Fail to send service!");
    return;
  }
}

void OpenManipulatorMotion::sendPanTiltFromPresent(double delta_x, double delta_y)
{
  std::vector<double> goalJoint;
  std::vector<std::string> joint_name;
  double path_time = 0.1;

  goalJoint.resize(6, 0.0);
  goalJoint.at(0) = delta_x;
  goalJoint.at(4) = delta_y;

  joint_name.push_back("joint1");
  joint_name.push_back("joint2");
  joint_name.push_back("joint3");
  joint_name.push_back("joint4");
  joint_name.push_back("joint5");
  joint_name.push_back("joint6");

  if(!setJointSpacePathFromPresent(joint_name, goalJoint, path_time))
  {
    ROS_ERROR("Fail to send service!");
    return;
  }
}

void OpenManipulatorMotion::sendEndEffectorFromPresent(Eigen::Quaterniond orientation, double delta_z)
{
  Eigen::Vector3d goal_pose_vector;
  std::vector<double> goal_pose;
  static double path_time = 1.5;

  goal_pose_vector = math::convertQuaternionToRotationMatrix(orientation)*math::convertXYZToVector(0, 0, delta_z);

  goal_pose.resize(7, 0.0);
  goal_pose.at(0) = goal_pose_vector.coeff(0,0);
  goal_pose.at(1) = goal_pose_vector.coeff(1,0);
  goal_pose.at(2) = goal_pose_vector.coeff(2,0);

  if(!setTaskSpacePathFromPresent(goal_pose, path_time))
  {
    ROS_ERROR("Fail to send service!");
    return;
  }
}

void OpenManipulatorMotion::sendMarkerPose(std::vector<double> position, Eigen::Quaterniond transform_orientation, double delta_x, double delta_y, double delta_z)
{
  Eigen::Vector3d delta_position_vector;
  std::vector<double> delta_position;
  std::vector<double> transform_position;
  std::vector<double> kinematics_pose;
  static double path_time = 2.0;

  delta_position_vector = math::convertQuaternionToRotationMatrix(transform_orientation)*math::convertXYZToVector(delta_x, delta_y, delta_z);
  delta_position.push_back(delta_position_vector.coeff(0,0));
  delta_position.push_back(delta_position_vector.coeff(1,0));
  delta_position.push_back(delta_position_vector.coeff(2,0));

  transform_position.push_back(position.at(0) + delta_position.at(0));
  transform_position.push_back(position.at(1) + delta_position.at(1));
  transform_position.push_back(position.at(2) + delta_position.at(2));

  if(solution_flag == 1)
  {
    kinematics_pose.push_back(transform_position.at(0) + 0.002);
    kinematics_pose.push_back(transform_position.at(1) + 0.002);
    kinematics_pose.push_back(transform_position.at(2) + 0.005);
    kinematics_pose.push_back(transform_orientation.w());
    kinematics_pose.push_back(transform_orientation.x());
    kinematics_pose.push_back(transform_orientation.y());
    kinematics_pose.push_back(transform_orientation.z());

  }
  else if(solution_flag == 2)
  {
    kinematics_pose.push_back(transform_position.at(0) + 0.002);
    kinematics_pose.push_back(transform_position.at(1));
    kinematics_pose.push_back(transform_position.at(2) + 0.01);
    kinematics_pose.push_back(transform_orientation.w());
    kinematics_pose.push_back(transform_orientation.x());
    kinematics_pose.push_back(transform_orientation.y());
    kinematics_pose.push_back(transform_orientation.z());
  }

  if(!setJointSpacePathToKinematicsPose(kinematics_pose, path_time))
  {
    ROS_ERROR("Fail to send service!");
    return;
  }
}

void OpenManipulatorMotion::sendGripperAngle(double gripper)
{
  std::vector<double> gripper_angle;
  gripper_angle.push_back(gripper);

  if(!setToolControl(gripper_angle))
  {
    ROS_ERROR("Fail to send service!");
    return;
  }
}

void OpenManipulatorMotion::sendDrawingTrajectory(double radius, double revolution, double start_angle, double path_time)
{
  std::string name;
  std::vector<double> arg;

  name = "circle";
  arg.push_back(radius);
  arg.push_back(revolution);
  arg.push_back(start_angle);

  if(!setDrawingTrajectory(name, arg, path_time))
  {
    ROS_ERROR("Fail to send service!");
    return;
  }
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

bool OpenManipulatorMotion::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorMotion::setTaskSpacePathFromPresent(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.planning_group = "gripper";
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose.at(3);
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose.at(4);
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose.at(5);
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose.at(6);

  srv.request.path_time = path_time;

  if(goal_task_space_path_from_present_client_.call(srv))
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

bool OpenManipulatorMotion::setDrawingTrajectory(std::string name, std::vector<double> arg, double path_time)
{
  open_manipulator_msgs::SetDrawingTrajectory srv;

  srv.request.end_effector_name = "gripper";
  srv.request.drawing_trajectory_name = name;
  srv.request.path_time = path_time;

  for(int i = 0; i < arg.size(); i ++)
    srv.request.param.push_back(arg.at(i));

  if(goal_drawing_trajectory_client_.call(srv))
  {    
    return srv.response.is_planned;
  }
  return false;
}

Eigen::Quaterniond OpenManipulatorMotion::markerOrientationTransformer(Eigen::Quaterniond marker_orientation)
{//Transform marker orientation to end effector

  Eigen::Matrix3d marker_orientation_matrix;
  Eigen::Matrix3d forward_transform_matrix;
  Eigen::Matrix3d reverse_transform_matrix;
  Eigen::Matrix3d forward_desired_orientation_matrix;
  Eigen::Matrix3d reverse_desired_orientation_matrix;
  Eigen::Matrix3d transform_marker_orientation_matrix;
  Eigen::Vector3d transform_marker_orientation_rpy;
  Eigen::Quaterniond transform_marker_orientation;

  //Get Orientation Matrix
  marker_orientation_matrix = math::convertQuaternionToRotationMatrix(marker_orientation);

  //Calculate
  forward_transform_matrix = math::convertPitchAngleToRotationMatrix(PI);
  reverse_transform_matrix = math::convertRollAngleToRotationMatrix(PI);

  forward_desired_orientation_matrix = marker_orientation_matrix*forward_transform_matrix; // End effector orientation
  reverse_desired_orientation_matrix = marker_orientation_matrix*reverse_transform_matrix; // Rotate End effector orientation by YawAngle 3.14

  //Solve
  transform_marker_orientation_matrix = orientationSolver(forward_desired_orientation_matrix, reverse_desired_orientation_matrix, kinematics_orientation_matrix_);

  //Result
  transform_marker_orientation_rpy = math::convertRotationMatrixToRPYVector(transform_marker_orientation_matrix);
  transform_marker_orientation = math::convertRPYToQuaternion(transform_marker_orientation_rpy.coeff(0,0), transform_marker_orientation_rpy.coeff(1,0)+0.03, transform_marker_orientation_rpy.coeff(2,0));

  return transform_marker_orientation;
}

Eigen::Matrix3d OpenManipulatorMotion::orientationSolver(Eigen::Matrix3d desired_orientation1, Eigen::Matrix3d desired_orientation2, Eigen::Matrix3d present_orientation)
{//Calculation orientation difference

  Eigen::Vector3d solution1;
  Eigen::Vector3d solution2;
  double solution1_value;
  double solution2_value;

  solution1 = math::orientationDifference(desired_orientation1, present_orientation);
  solution2 = math::orientationDifference(desired_orientation2, present_orientation);

  solution1_value = solution1.transpose() * solution1;
  solution2_value = solution2.transpose() * solution2;

  if(solution1_value < solution2_value)
  {
    solution_flag = 1;
    //std::cout << "solution1" << std::endl;
    return desired_orientation1;
  }
  else
  {
    solution_flag = 2;
    //std::cout << "solution2" << std::endl;
    return desired_orientation2;
  }
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

  om_motion.motion_timer = n.createTimer(ros::Duration(0.05), &OpenManipulatorMotion::timerCallback, &om_motion);
  om_motion.motion_timer.stop();

  ros::spin();
  return 0;
}
