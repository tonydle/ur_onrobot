#!/usr/bin/env python3

import numpy as np
import math
import threading
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from onrobot_rg_control.msg import OnRobotRGOutput
from onrobot_rg_control.msg import OnRobotRGInput


class OnRobotJointPositionControllerServer:
    def __init__(self):
        self._output_pub = rospy.Publisher(
            'OnRobotRGOutput', OnRobotRGOutput, queue_size=10)
        self._input_sub = rospy.Subscriber(
            "OnRobotRGInput", OnRobotRGInput, self._input_cb)
        self._input_data = OnRobotRGInput()

        self._joint_state_pub = rospy.Publisher(
            "joint_states", JointState, queue_size=10)

        self._output_command = OnRobotRGOutput()
        self._joint_position_sub = rospy.Subscriber(
            "joint_position_controller/command", Float64, self._joint_position_controller_cb)
        
        self._joint_state_msg = JointState()
        self._joint_state_msg.name = ["finger_joint"]

    def _joint_position_controller_cb(self, msg):
        finger_theta = (msg.data)
        if finger_theta > 0.785398:
            finger_theta = 0.785398
        if finger_theta < -0.558505:
            finger_theta = -0.558505
        finger_D = (0.034356 + 2 * \
            (0.05500 * math.sin(-finger_theta + 0.803591) - 0.018389)) * 10000
        self._output_command = self._gen_command(
            int(finger_D) , self._output_command)
        self._output_pub.publish(self._output_command)

    def _input_cb(self, msg):
        # Current width between the gripper
        self._input_data = msg

    def run(self):
        rate = rospy.Rate(50)  # 1 Hz
        while not rospy.is_shutdown():
            self._joint_state_msg.header.stamp = rospy.Time.now()
            finger_D = (self._input_data.gWDF / 10) / 1000
            try:
                finger_theta = 0.803591 - \
                    math.asin(((finger_D - 0.034356)/2 + 0.018389)/0.055000)
            except ValueError:
                finger_theta = 0
            self._joint_state_msg.position = [finger_theta]
            self._joint_state_pub.publish(self._joint_state_msg)
            rate.sleep()

    def _gen_command(self, char, command):
        """ Updates the command according to the input character.

            Args:
                char (str): set command service request message
                command (OnRobotRGOutput): command to be sent

            Returns:
                command: command message with parameters set
        """

        if gtype == 'rg2':
            max_force = 400
            max_width = 1100
        elif gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            rospy.signal_shutdown(
                rospy.get_name() +
                ": Select the gripper type from rg2 or rg6.")

        if char == 'c':
            command.rGFR = max_force
            command.rGWD = 0
            command.rCTR = 16
        elif char == 'o':
            command.rGFR = max_force
            command.rGWD = max_width
            command.rCTR = 16
        elif char == 'i':
            command.rGFR += 25
            command.rGFR = min(max_force, command.rGFR)
            command.rCTR = 16
        elif char == 'd':
            command.rGFR -= 25
            command.rGFR = max(0, command.rGFR)
            command.rCTR = 16
        else:
            # If the command entered is a int, assign this value to rGWD
            try:
                command.rGFR = max_force
                command.rGWD = min(max_width, int(char))
                command.rCTR = 16
            except ValueError:
                pass

        return command


if __name__ == '__main__':
    gtype = rospy.get_param('/onrobot/gripper', 'rg2')
    rospy.init_node(
        'OnRobotRGJointPositionControllerServer',
        anonymous=True,
        log_level=rospy.INFO)
    node = OnRobotJointPositionControllerServer()
    node.run()
