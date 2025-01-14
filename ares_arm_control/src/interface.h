//
// Created by seden on 23.02.2024.
//

#ifndef ARES_ARM_HARDWARE_INTERFACE_HARDWARE_INTERFACE_H
#define ARES_ARM_HARDWARE_INTERFACE_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <tmotor.hpp>

class MyRobot : public hardware_interface::RobotHW 
{
    public:
        MyRobot(ros::NodeHandle& nh);
        ~MyRobot();
        void init();
        void update(const ros::TimerEvent& e);
        void read();  
        void write(ros::Duration elapsed_time);
        
    protected:
        
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;

        double joint_position_[4];
        double joint_velocity_[4];
        double joint_effort_[4];
        double joint_position_command_[4];
        TMotor::AKManager motor1;
        TMotor::AKManager motor2;
        TMotor::AKManager motor3;
        TMotor::AKManager motor4;
        
        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

#endif //ARES_ARM_HARDWARE_INTERFACE_HARDWARE_INTERFACE_H
