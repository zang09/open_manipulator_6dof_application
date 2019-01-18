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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

/***********************************************************
** Modified by Hae-Bum Jung
************************************************************/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/open_manipulator_6dof_control_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;

namespace open_manipulator_control_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    connect(ui.tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabSelected()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    qnode.init();
}

MainWindow::~MainWindow() {}

void MainWindow::timerCallback()
{
    std::vector<double> joint_angle = qnode.getPresentJointAngle();
    if(joint_angle.size() != 7) //5
        return;

    ui.txt_j1->setText(QString::number(joint_angle.at(0),'f', 3));
    ui.txt_j2->setText(QString::number(joint_angle.at(1),'f', 3));
    ui.txt_j3->setText(QString::number(joint_angle.at(2),'f', 3));
    ui.txt_j4->setText(QString::number(joint_angle.at(3),'f', 3));
    ui.txt_j5->setText(QString::number(joint_angle.at(4),'f', 3));
    ui.txt_j6->setText(QString::number(joint_angle.at(5),'f', 3));
    ui.txt_grip->setText(QString::number(joint_angle.at(6),'f', 3));

    std::vector<double> position = qnode.getPresentKinematicsPos();
    Eigen::Vector3d orientation_rpy = qnode.getPresentKinematicsOriRPY();

    if(position.size() != 3)
        return;
    if(orientation_rpy.size() != 3)
        return;

    ui.txt_x->setText(QString::number(position.at(0),'f', 3));
    ui.txt_y->setText(QString::number(position.at(1),'f', 3));
    ui.txt_z->setText(QString::number(position.at(2),'f', 3));

    ui.txt_roll->setText(QString::number(orientation_rpy.coeffRef(0,0),'f', 3));
    ui.txt_pitch->setText(QString::number(orientation_rpy.coeffRef(1,0),'f', 3));
    ui.txt_yaw->setText(QString::number(orientation_rpy.coeffRef(2,0),'f', 3));

    if(qnode.getOpenManipulatorActuatorState() == true)
        ui.txt_actuactor_state->setText("Actuator enabled");
    else
        ui.txt_actuactor_state->setText("Actuator disabled");
    if(qnode.getOpenManipulatorMovingState() == true)
        ui.txt_moving_state->setText("Robot is moving");
    else
        ui.txt_moving_state->setText("Robot is stopped");
}

void MainWindow::motion_wait(unsigned int time)
{
    sleep(time);
}
void MainWindow::tabSelected()
{
    if(ui.tabWidget->currentIndex()==0)
        on_btn_read_joint_angle_clicked();
    if(ui.tabWidget->currentIndex()==1)
        on_btn_read_kinematic_pose_clicked();
}

void MainWindow::writeLog(QString str)
{
    ui.plainTextEdit_log->moveCursor (QTextCursor::End);
    ui.plainTextEdit_log->appendPlainText(str);
}

void MainWindow::on_btn_timer_start_clicked(void)
{        
    start_flag = !start_flag;

    if(start_flag)
    {
        timer = new QTimer(this);
        m_timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(timerCallback()));
        connect(m_timer, SIGNAL(timeout()), this, SLOT(on_btn_pick_clicked(void)));
        timer->start(100);

        writeLog("QTimer start : 100ms");
        ui.btn_timer_start->setText("Timer Stop");
        ui.btn_actuator_disable->setEnabled(true);
        ui.btn_actuator_enable->setEnabled(true);
        ui.btn_gripper_close->setEnabled(true);
        ui.btn_gripper_open->setEnabled(true);
        ui.btn_home_pose->setEnabled(true);
        ui.btn_init_pose->setEnabled(true);
        ui.btn_read_joint_angle->setEnabled(true);
        ui.btn_read_kinematic_pose->setEnabled(true);
        ui.btn_send_joint_angle->setEnabled(true);
        ui.btn_send_kinematic_pose->setEnabled(true);
        ui.btn_send_drawing_trajectory->setEnabled(true);
        ui.btn_set_gripper->setEnabled(true);
        ui.btn_pick->setEnabled(true);
        ui.btn_multi->setEnabled(true);
    }
    else
    {
        timer->stop();

        writeLog("QTimer stop!");
        ui.btn_timer_start->setText("Timer Start");
        ui.btn_actuator_disable->setEnabled(false);
        ui.btn_actuator_enable->setEnabled(false);
        ui.btn_gripper_close->setEnabled(false);
        ui.btn_gripper_open->setEnabled(false);
        ui.btn_home_pose->setEnabled(false);
        ui.btn_init_pose->setEnabled(false);
        ui.btn_read_joint_angle->setEnabled(false);
        ui.btn_read_kinematic_pose->setEnabled(false);
        ui.btn_send_joint_angle->setEnabled(false);
        ui.btn_send_kinematic_pose->setEnabled(false);
        ui.btn_send_drawing_trajectory->setEnabled(false);
        ui.btn_set_gripper->setEnabled(false);
        ui.btn_pick->setEnabled(false);
        ui.btn_multi->setEnabled(false);
    }
}

void MainWindow::on_btn_clear_log_clicked(void)
{
    ui.plainTextEdit_log->clear();
}

void MainWindow::on_btn_actuator_enable_clicked(void)
{
    if(!qnode.setActuatorState(true))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }

    writeLog("Send actuator state to enable");
}

void MainWindow::on_btn_actuator_disable_clicked(void)
{
    if(!qnode.setActuatorState(false))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }

    writeLog("Send actuator state to disable");
}

void MainWindow::on_btn_init_pose_clicked(void)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.0);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);

    if(!qnode.setJointSpacePath(joint_name, joint_angle, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }

    writeLog("Send joint angle to initial pose");
}

void MainWindow::on_btn_home_pose_clicked(void)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.78);
    joint_name.push_back("joint3"); joint_angle.push_back(1.5);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.8);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);

    if(!qnode.setJointSpacePath(joint_name, joint_angle, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }
    writeLog("Send joint angle to home pose");
}

void MainWindow::on_btn_gripper_open_clicked(void)
{
    std::vector<double> joint_angle;
    joint_angle.push_back(0.01);

    if(!qnode.setToolControl(joint_angle))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }

    writeLog("Send gripper open");
}

void MainWindow::on_btn_gripper_close_clicked(void)
{
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    if(!qnode.setToolControl(joint_angle))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }

    writeLog("Send gripper close");
}

void MainWindow::on_btn_read_joint_angle_clicked(void)
{
    std::vector<double> joint_angle = qnode.getPresentJointAngle();
    ui.doubleSpinBox_j1->setValue(joint_angle.at(0));
    ui.doubleSpinBox_j2->setValue(joint_angle.at(1));
    ui.doubleSpinBox_j3->setValue(joint_angle.at(2));
    ui.doubleSpinBox_j4->setValue(joint_angle.at(3));
    ui.doubleSpinBox_j5->setValue(joint_angle.at(4));
    ui.doubleSpinBox_j6->setValue(joint_angle.at(5));
    ui.doubleSpinBox_gripper->setValue(joint_angle.at(6));

    writeLog("Read joint angle");
}
void MainWindow::on_btn_send_joint_angle_clicked(void)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = ui.doubleSpinBox_time_js->value();

    joint_name.push_back("joint1"); joint_angle.push_back(ui.doubleSpinBox_j1->value());
    joint_name.push_back("joint2"); joint_angle.push_back(ui.doubleSpinBox_j2->value());
    joint_name.push_back("joint3"); joint_angle.push_back(ui.doubleSpinBox_j3->value());
    joint_name.push_back("joint4"); joint_angle.push_back(ui.doubleSpinBox_j4->value());
    joint_name.push_back("joint5"); joint_angle.push_back(ui.doubleSpinBox_j5->value());
    joint_name.push_back("joint6"); joint_angle.push_back(ui.doubleSpinBox_j6->value());


    if(!qnode.setJointSpacePath(joint_name, joint_angle, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }

    writeLog("Send joint angle");
}
void MainWindow::on_btn_read_kinematic_pose_clicked(void)
{
    std::vector<double> position = qnode.getPresentKinematicsPos();
    Eigen::Vector3d orientation_rpy = qnode.getPresentKinematicsOriRPY();

    ui.doubleSpinBox_x->setValue(position.at(0));
    ui.doubleSpinBox_y->setValue(position.at(1));
    ui.doubleSpinBox_z->setValue(position.at(2));
    ui.doubleSpinBox_roll->setValue(orientation_rpy.coeffRef(0,0));
    ui.doubleSpinBox_pitch->setValue(orientation_rpy.coeffRef(1,0));
    ui.doubleSpinBox_yaw->setValue(orientation_rpy.coeffRef(2,0));

    writeLog("Read kinematic_pose");
}
void MainWindow::on_btn_send_kinematic_pose_clicked(void)
{
    std::vector<double> kinematics_pose;
    Eigen::Quaterniond temp_orientation;
    temp_orientation = RM_MATH::convertRPYToQuaternion(ui.doubleSpinBox_roll->value(), ui.doubleSpinBox_pitch->value(), ui.doubleSpinBox_yaw->value());

    double path_time = ui.doubleSpinBox_time_cs->value();
    kinematics_pose.push_back(ui.doubleSpinBox_x->value());
    kinematics_pose.push_back(ui.doubleSpinBox_y->value());
    kinematics_pose.push_back(ui.doubleSpinBox_z->value());
    kinematics_pose.push_back(temp_orientation.w());
    kinematics_pose.push_back(temp_orientation.x());
    kinematics_pose.push_back(temp_orientation.y());
    kinematics_pose.push_back(temp_orientation.z());

    if(!qnode.setJointSpacePathToKinematicsPose(kinematics_pose, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }

    writeLog("Send kinematic_pose");
}
void MainWindow::on_btn_set_gripper_clicked(void)
{
    std::vector<double> joint_angle;
    joint_angle.push_back(ui.doubleSpinBox_gripper->value());
    if(!qnode.setToolControl(joint_angle))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }
    writeLog("Send gripper value");
}

void MainWindow::on_btn_get_manipulator_setting_clicked(void)
{
    qnode.setOption("print_open_manipulator_setting");
    writeLog("Check the terminal of open_manipulator_controller package");
}

void MainWindow::on_radio_drawing_line_clicked(void)
{
    ui.txt_drawing_arg_1->setText("Transpose X");
    ui.txt_drawing_arg_2->setText("Transpose Y");
    ui.txt_drawing_arg_3->setText("Transpose Z");
    ui.txt_drawing_arg_unit_1->setText("m");
    ui.txt_drawing_arg_unit_2->setText("m");
    ui.txt_drawing_arg_unit_3->setText("m");
    ui.doubleSpinBox_drawing_arg_1->setValue(0.0);
    ui.doubleSpinBox_drawing_arg_2->setValue(0.0);
    ui.doubleSpinBox_drawing_arg_3->setValue(0.0);
}
void MainWindow::on_radio_drawing_circle_clicked(void)
{
    ui.txt_drawing_arg_1->setText("Radius");
    ui.txt_drawing_arg_2->setText("Revolution");
    ui.txt_drawing_arg_3->setText("Start angle");
    ui.txt_drawing_arg_unit_1->setText("m");
    ui.txt_drawing_arg_unit_2->setText("rev");
    ui.txt_drawing_arg_unit_3->setText("rad");
    ui.doubleSpinBox_drawing_arg_1->setValue(0.0);
    ui.doubleSpinBox_drawing_arg_2->setValue(0.0);
    ui.doubleSpinBox_drawing_arg_3->setValue(0.0);
}
void MainWindow::on_radio_drawing_rhombus_clicked(void)
{
    ui.txt_drawing_arg_1->setText("Radius");
    ui.txt_drawing_arg_2->setText("Revolution");
    ui.txt_drawing_arg_3->setText("Start angle");
    ui.txt_drawing_arg_unit_1->setText("m");
    ui.txt_drawing_arg_unit_2->setText("rev");
    ui.txt_drawing_arg_unit_3->setText("rad");
    ui.doubleSpinBox_drawing_arg_1->setValue(0.0);
    ui.doubleSpinBox_drawing_arg_2->setValue(0.0);
    ui.doubleSpinBox_drawing_arg_3->setValue(0.0);
}
void MainWindow::on_radio_drawing_heart_clicked(void)
{
    ui.txt_drawing_arg_1->setText("Radius");
    ui.txt_drawing_arg_2->setText("Revolution");
    ui.txt_drawing_arg_3->setText("Start angle");
    ui.txt_drawing_arg_unit_1->setText("m");
    ui.txt_drawing_arg_unit_2->setText("rev");
    ui.txt_drawing_arg_unit_3->setText("rad");
    ui.doubleSpinBox_drawing_arg_1->setValue(0.0);
    ui.doubleSpinBox_drawing_arg_2->setValue(0.0);
    ui.doubleSpinBox_drawing_arg_3->setValue(0.0);
}
void MainWindow::on_btn_send_drawing_trajectory_clicked(void)
{
    std::string name;
    if(ui.radio_drawing_line->isChecked()) name = "line";
    else if(ui.radio_drawing_circle->isChecked()) name = "circle";
    else if(ui.radio_drawing_rhombus->isChecked()) name = "rhombus";
    else if(ui.radio_drawing_heart->isChecked()) name = "heart";

    std::vector<double> arg;
    arg.push_back(ui.doubleSpinBox_drawing_arg_1->value());
    arg.push_back(ui.doubleSpinBox_drawing_arg_2->value());
    arg.push_back(ui.doubleSpinBox_drawing_arg_3->value());

    double path_time = ui.doubleSpinBox_time_drawing->value();

    if(!qnode.setDrawingTrajectory(name, arg, path_time))
    {
        writeLog("[ERR!!] Failed to send service");
        return;
    }
    writeLog("Send drawing trajectory");
}

void MainWindow::on_btn_control_up_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    goal_pose.at(2) = DELTA;
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Up");
}
void MainWindow::on_btn_control_down_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    goal_pose.at(2) = -DELTA;
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Down");
}
void MainWindow::on_btn_control_left_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    goal_pose.at(1) = -DELTA;
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Left");
}
void MainWindow::on_btn_control_right_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    goal_pose.at(1) = DELTA;
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Right");
}
void MainWindow::on_btn_control_fwd_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    goal_pose.at(0) = DELTA;
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Forward");
}
void MainWindow::on_btn_control_back_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    goal_pose.at(0) = -DELTA;
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Back");
}
void MainWindow::on_btn_control_init_pos_clicked(void)
{
    std::vector<double> kinematics_pose;
    kinematics_pose.push_back(0.16);   //x
    kinematics_pose.push_back(0.0);    //y
    kinematics_pose.push_back(0.25);   //z
    qnode.setTaskSpacePathPositionOnly(kinematics_pose, 2.0);
    writeLog("Send Init");
}

void MainWindow::on_btn_control_p_pitch_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    Eigen::Quaterniond temp_orientation = RM_MATH::convertRPYToQuaternion(0.0,DELTA_ORI,0.0);
    goal_pose.at(3) = temp_orientation.w();
    goal_pose.at(4) = temp_orientation.x();
    goal_pose.at(5) = temp_orientation.y();
    goal_pose.at(6) = temp_orientation.z();
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Plus Pitch");
}
void MainWindow::on_btn_control_m_pitch_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    Eigen::Quaterniond temp_orientation = RM_MATH::convertRPYToQuaternion(0.0,-DELTA_ORI,0.0);
    goal_pose.at(3) = temp_orientation.w();
    goal_pose.at(4) = temp_orientation.x();
    goal_pose.at(5) = temp_orientation.y();
    goal_pose.at(6) = temp_orientation.z();
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Minus Pitch");
}
void MainWindow::on_btn_control_p_roll_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    Eigen::Quaterniond temp_orientation = RM_MATH::convertRPYToQuaternion(DELTA_ORI,0.0,0.0);
    goal_pose.at(3) = temp_orientation.w();
    goal_pose.at(4) = temp_orientation.x();
    goal_pose.at(5) = temp_orientation.y();
    goal_pose.at(6) = temp_orientation.z();
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Plus Roll");
}
void MainWindow::on_btn_control_m_roll_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    Eigen::Quaterniond temp_orientation = RM_MATH::convertRPYToQuaternion(-DELTA_ORI,0.0,0.0);
    goal_pose.at(3) = temp_orientation.w();
    goal_pose.at(4) = temp_orientation.x();
    goal_pose.at(5) = temp_orientation.y();
    goal_pose.at(6) = temp_orientation.z();
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Minus Roll");
}
void MainWindow::on_btn_control_p_yaw_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    Eigen::Quaterniond temp_orientation = RM_MATH::convertRPYToQuaternion(0.0,0.0,-0.03);
    goal_pose.at(3) = temp_orientation.w();
    goal_pose.at(4) = temp_orientation.x();
    goal_pose.at(5) = temp_orientation.y();
    goal_pose.at(6) = temp_orientation.z();
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Plus Yaw");
}
void MainWindow::on_btn_control_m_yaw_clicked(void)
{
    std::vector<double> goal_pose;    goal_pose.resize(7, 0.0);
    Eigen::Quaterniond temp_orientation = RM_MATH::convertRPYToQuaternion(0.0,0.0,0.03);
    goal_pose.at(3) = temp_orientation.w();
    goal_pose.at(4) = temp_orientation.x();
    goal_pose.at(5) = temp_orientation.y();
    goal_pose.at(6) = temp_orientation.z();
    qnode.setTaskSpacePathFromPresent(goal_pose, PATH_TIME);
    writeLog("Send Minus Yaw");
}
void MainWindow::on_btn_control_init_ori_clicked(void)
{
    std::vector<double> orientation_pose;
    Eigen::Quaterniond temp_orientation = RM_MATH::convertRPYToQuaternion(-0.695,1.55,-0.7);
    orientation_pose.push_back(temp_orientation.w());
    orientation_pose.push_back(temp_orientation.x());
    orientation_pose.push_back(temp_orientation.y());
    orientation_pose.push_back(temp_orientation.z());
    qnode.setTaskSpacePathOrientationOnly(orientation_pose, PATH_TIME);
    writeLog("Send Init");
}

void MainWindow::on_btn_pick_clicked(void)  //callback
{
    m_timer->start(100);

    std::vector<double> kinematics_pose1;
    std::vector<double> joint_angle1;
    static double path_time = 2.0;      //DEFAULT
    kinematics_pose1.push_back(0.15);   //x
    kinematics_pose1.push_back(-0.25);  //y
    kinematics_pose1.push_back(0.15);   //z
    joint_angle1.push_back(-0.01);

    std::vector<double> kinematics_pose2;
    std::vector<double> joint_angle2;
    kinematics_pose2.push_back(0.15);   //x
    kinematics_pose2.push_back(0.25);   //y
    kinematics_pose2.push_back(0.15);   //z
    joint_angle2.push_back(0.01);

    std::vector<double> kinematics_pose3;
    std::vector<double> joint_angle3;
    kinematics_pose3.push_back(0.293);  //x
    kinematics_pose3.push_back(0.0);    //y
    kinematics_pose3.push_back(0.203);  //z
    joint_angle3.push_back(0.01);

    timer_cnt++;

    switch(timer_cnt)
    {
    case 1:
        writeLog("Send joint angle to DO motion");
        //qnode.setTaskSpacePath(kinematics_pose1, path_time);
        break;
    case 21:
        //qnode.setToolControl(joint_angle1);
        break;
    case 31:
        //qnode.setTaskSpacePath(kinematics_pose2, 5.0);
        break;
    case 61:
        qnode.setToolControl(joint_angle2);
        break;
    case 71:
        //qnode.setTaskSpacePath(kinematics_pose3, path_time);
        break;
    case 91:
        qnode.setToolControl(joint_angle3);
    case 92:
        m_timer->stop();
        timer_cnt = 0;
        break;
    }
}

void MainWindow::on_btn_multi_clicked(void)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle1;
    std::vector<double> joint_angle2;
    std::vector<double> joint_angle3;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle1.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle1.push_back(-45.0*D2R);
    joint_name.push_back("joint3"); joint_angle1.push_back(90.0*D2R);
    joint_name.push_back("joint4"); joint_angle1.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle1.push_back(45.0*D2R);
    joint_name.push_back("joint6"); joint_angle1.push_back(0.0);

    joint_angle2.push_back(0.01);
    joint_angle2.push_back(-0.01);

    switch(1)
    {
    case 1:
        qnode.setJointSpacePath(joint_name, joint_angle1, path_time);
        //ROBOTIS_MANIPULATOR::RobotisManipulator::TrajectoryWait(path_time, ROBOTIS_MANIPULATOR::RobotisManipulator::getAllActiveJointValue());
        qnode.setToolControl(joint_angle2);
        //ROBOTIS_MANIPULATOR::RobotisManipulator::TrajectoryWait(1.0, ROBOTIS_MANIPULATOR::RobotisManipulator::getAllActiveJointValue());
        qnode.setToolControl(joint_angle3);
    }

    /*
     *
     * std::vector<double> kinematics_pose1;
    std::vector<double> joint_angle1;
    static double path_time = 4.0;      //DEFAULT
    kinematics_pose1.push_back(0.15);   //x
    kinematics_pose1.push_back(-0.25);  //y
    kinematics_pose1.push_back(0.15);   //z
    joint_angle1.push_back(-0.01);

    std::vector<double> kinematics_pose2;
    std::vector<double> joint_angle2;
    kinematics_pose2.push_back(0.15);   //x
    kinematics_pose2.push_back(0.25);   //y
    kinematics_pose2.push_back(0.15);   //z
    joint_angle2.push_back(0.01);

    std::vector<double> kinematics_pose3;
    std::vector<double> joint_angle3;
    kinematics_pose3.push_back(0.293);  //x
    kinematics_pose3.push_back(0.0);    //y
    kinematics_pose3.push_back(0.203);  //z

    joint_angle3.push_back(0.01);

    switch(1)
    {
    case 1:
        qnode.setMotionLocation(kinematics_pose1, path_time, joint_angle1);
    case 2:
        motion_wait(1);
        qnode.setMotionLocation(kinematics_pose2, 6.0, joint_angle2);
    case 3:
        motion_wait(1);
        qnode.setMotionLocation(kinematics_pose3, path_time, joint_angle3);
    default:
        break;
    }
    */

    writeLog("Send joint angle to Motion");
}

}  // namespace open_manipulator_control_gui
