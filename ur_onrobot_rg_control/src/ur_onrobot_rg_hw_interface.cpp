#include "ur_onrobot_rg_control/ur_onrobot_rg_hw_interface.h"
#include <cmath>

OnRobotRGHWInterface::OnRobotRGHWInterface(ros::NodeHandle& nh) : nh_(nh), position_received_(false)
{
    joint_position_ = std::numeric_limits<double>::quiet_NaN(); // NaN means no position received yet
    joint_velocity_ = 0.0;
    joint_effort_ = 0.0;
    joint_command_ = std::numeric_limits<double>::quiet_NaN(); // NaN means no command received yet

    output_pub_ = nh_.advertise<ur_onrobot_rg_control::OnRobotRGOutputCopy>("OnRobotRGOutputCopy", 10);
    input_sub_ = nh_.subscribe("OnRobotRGInputCopy", 10, &OnRobotRGHWInterface::inputCB, this);

    hardware_interface::JointStateHandle state_handle("finger_joint", &joint_position_, &joint_velocity_, &joint_effort_);
    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle("finger_joint"), &joint_command_);
    position_joint_interface_.registerHandle(pos_handle);

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

void OnRobotRGHWInterface::inputCB(const ur_onrobot_rg_control::OnRobotRGInputCopy::ConstPtr& msg)
{
    input_data_ = *msg;
    position_received_ = true;
}

void OnRobotRGHWInterface::read()
{
    if (!position_received_) return; // Do nothing if no position received yet
    
    double finger_D = (input_data_.gWDF / 10.0) / 1000.0;
    try
    {
        joint_position_ = 0.803591 - asin(((finger_D - 0.034356) / 2.0 + 0.018389) / 0.055000);
    }
    catch (const std::exception&)
    {
        joint_position_ = 0;
    }
}

void OnRobotRGHWInterface::write()
{
    if (std::isnan(joint_command_)) return; // Do nothing if no command received yet

    double finger_theta = joint_command_;
    if (finger_theta > 0.785398)
        finger_theta = 0.785398;
    if (finger_theta < -0.558505)
        finger_theta = -0.558505;

    double finger_D = (0.034356 + 2 * (0.05500 * sin(-finger_theta + 0.803591) - 0.018389)) * 10000;
    output_command_.rGWD = static_cast<int>(finger_D);
    output_command_.rGFR = 400; // Set appropriate force
    output_command_.rCTR = 16;
    output_pub_.publish(output_command_);
}
