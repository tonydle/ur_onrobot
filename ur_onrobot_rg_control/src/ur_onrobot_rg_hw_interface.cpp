#include "ur_onrobot_rg_control/ur_onrobot_rg_hw_interface.h"

OnRobotRGHWInterface::OnRobotRGHWInterface(ros::NodeHandle& nh) : nh_(nh), position_received_(false)
{
    joint_position_ = 0.0;
    joint_velocity_ = 0.0;
    joint_effort_ = 0.0;
    joint_command_ = 0.0;
    last_joint_command_ = joint_command_;

    std::string gripper;
    nh_.param<std::string>("gripper", gripper, "rg2");
    
    if (gripper == "rg6") {
        output_command_.rGFR = 1200;
        open_position_ = 0.160;
    } else { // default to "rg2"
        output_command_.rGFR = 400;
        open_position_ = 0.110;
    }
    output_command_.rCTR = 16;

    output_pub_ = nh_.advertise<ur_onrobot_rg_control::OnRobotRGOutputCopy>("OnRobotRGOutputCopy", 10);
    input_sub_ = nh_.subscribe("OnRobotRGInputCopy", 10, &OnRobotRGHWInterface::inputCB, this);
    open_srv_ = nh_.advertiseService("open", &OnRobotRGHWInterface::openCB, this);
    close_srv_ = nh_.advertiseService("close", &OnRobotRGHWInterface::closeCB, this);

    hardware_interface::JointStateHandle state_handle("finger_width", &joint_position_, &joint_velocity_, &joint_effort_);
    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle("finger_width"), &joint_command_);
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
    
    joint_position_ = (input_data_.gWDF / 10.0) / 1000.0;
    if(joint_position_ < 0) joint_position_ = 0;
    if(joint_position_ - open_position_ > 0.5 ) joint_position_ = 0; // To avoid false readings (when offset is not set correctly)
}

void OnRobotRGHWInterface::write()
{
    std::lock_guard<std::mutex> lock(command_mutex_);

    if (abs(joint_command_ - last_joint_command_) < 0.0001) return;

    calculateAndPublishOutput();

    last_joint_command_ = joint_command_;
}

void OnRobotRGHWInterface::calculateAndPublishOutput()
{
    double finger_D = joint_command_ * 10000;
    if (finger_D < 0) finger_D = 0;
    output_command_.rGWD = static_cast<int>(finger_D);
    output_pub_.publish(output_command_);
}

bool OnRobotRGHWInterface::openCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        joint_command_ = open_position_;
    }
    calculateAndPublishOutput();
    res.success = true;
    res.message = "Opening";
    return true;
}

bool OnRobotRGHWInterface::closeCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        joint_command_ = 0.0;
    }
    calculateAndPublishOutput();
    res.success = true;
    res.message = "Closing";
    return true;
}