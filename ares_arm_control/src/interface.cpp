//
// Created by seden on 23.02.2024.
//

#include "interface.h"

MyRobot::MyRobot(ros::NodeHandle& nh) :
  nh_(nh),
  motor1(0x0B),
  motor2(0X0C),
  motor3(0X0D),
  motor4(0X0E)
{
  init();
  motor1.connect("vcan0");
  motor2.connect("vcan0");
  motor3.connect("vcan0");
  motor4.connect("vcan0");
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
  loop_hz_=10;
  ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
  my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

MyRobot::~MyRobot() {
}

void MyRobot::init() {
    ROS_INFO("Initializing hardware interface");
    //State handles are initialized and registered to the state interface.
    hardware_interface::JointStateHandle joint1StateHandle("joint1", &joint_position_[0],&joint_velocity_[0],&joint_effort_[0]);
    joint_state_interface_.registerHandle(joint1StateHandle);
    hardware_interface::JointStateHandle joint2StateHandle("joint2", &joint_position_[1],&joint_velocity_[1],&joint_effort_[1]);
    joint_state_interface_.registerHandle(joint2StateHandle);
    hardware_interface::JointStateHandle joint3StateHandle("joint3", &joint_position_[2],&joint_velocity_[2],&joint_effort_[2]);
    joint_state_interface_.registerHandle(joint3StateHandle);
    hardware_interface::JointStateHandle joint4StateHandle("joint4", &joint_position_[3],&joint_velocity_[3],&joint_effort_[3]);
    joint_state_interface_.registerHandle(joint4StateHandle);
    
    // Position handles are initialized and registered to the position interface.
        
    hardware_interface::JointHandle joint1PositionHandle(joint1StateHandle, &joint_position_command_[0]);
    position_joint_interface_.registerHandle(joint1PositionHandle);
    hardware_interface::JointHandle joint2PositionHandle(joint2StateHandle, &joint_position_command_[1]);
    position_joint_interface_.registerHandle(joint2PositionHandle);
    hardware_interface::JointHandle joint3PositionHandle(joint3StateHandle, &joint_position_command_[2]);
    position_joint_interface_.registerHandle(joint3PositionHandle);
    hardware_interface::JointHandle joint4PositionHandle(joint4StateHandle, &joint_position_command_[3]);
    position_joint_interface_.registerHandle(joint4PositionHandle);

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void MyRobot::read() {

  joint_position_[0]=static_cast<double>(motor1.getPosition()*(M_PI/180.0));
  joint_position_[1]=static_cast<double>(motor2.getPosition()*(M_PI/180.0));
  joint_position_[2]=static_cast<double>(motor3.getPosition()*(M_PI/180.0));
  joint_position_[3]=static_cast<double>(motor4.getPosition()*(M_PI/180.0));

}

void MyRobot::write(ros::Duration elapsed_time) {
  
  motor1.sendPosition(joint_position_command_[0]*(180.0/M_PI)*29.0);
  motor2.sendPosition(joint_position_command_[1]*(180.0/M_PI));
  motor3.sendPosition(joint_position_command_[2]*(180.0/M_PI)*20.0);
  motor4.sendPosition(joint_position_command_[3]*(180.0/M_PI)*10.0);

}

int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "MyRobot_hardware_interface_node");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2); 
    
    
    // Create the object of the robot hae_interface class and spin the thread. 
    MyRobot ROBOT(nh);
    spinner.spin();
    
    return 0;
}
