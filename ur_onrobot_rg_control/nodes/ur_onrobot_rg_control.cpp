#include "ur_onrobot_rg_control/ur_onrobot_rg_hw_interface.h"
#include "controller_manager/controller_manager.h"

#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur_onrobot_rg_hw_interface_node");
    ros::NodeHandle nh;
    ros::CallbackQueue queue;
    nh.setCallbackQueue(&queue);

    OnRobotRGHWInterface robot(nh);
    controller_manager::ControllerManager cm(&robot, nh);

    ros::AsyncSpinner spinner(4, &queue);
    spinner.start();

    ros::Time ts = ros::Time::now();

    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::Duration d = ros::Time::now() - ts;
        ts = ros::Time::now();
        robot.read();
        cm.update(ts, d);
        robot.write();
        rate.sleep();
    }

    spinner.stop();

    return 0;
}
