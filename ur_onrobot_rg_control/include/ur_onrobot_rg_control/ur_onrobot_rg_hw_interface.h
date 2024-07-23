#ifndef ONROBOT_HW_INTERFACE_H
#define ONROBOT_HW_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <ur_onrobot_rg_control/OnRobotRGOutputCopy.h>
#include <ur_onrobot_rg_control/OnRobotRGInputCopy.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class OnRobotRGHWInterface : public hardware_interface::RobotHW
{
public:
    OnRobotRGHWInterface(ros::NodeHandle& nh);
    void read();
    void write();

private:
    void jointPositionCommandCB(const std_msgs::Float64::ConstPtr& msg);
    void inputCB(const ur_onrobot_rg_control::OnRobotRGInputCopy::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Publisher output_pub_;
    ros::Subscriber input_sub_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    ur_onrobot_rg_control::OnRobotRGOutputCopy output_command_;
    ur_onrobot_rg_control::OnRobotRGInputCopy input_data_;

    double joint_position_;
    double joint_velocity_;
    double joint_effort_;
    double joint_command_;
};

#endif // ONROBOT_HW_INTERFACE_H
