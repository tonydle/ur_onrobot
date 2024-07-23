#!/usr/bin/env python3

import rospy
from onrobot_rg_control.msg import OnRobotRGOutput
from onrobot_rg_control.msg import OnRobotRGInput
from ur_onrobot_rg_control.msg import OnRobotRGOutputCopy
from ur_onrobot_rg_control.msg import OnRobotRGInputCopy

class MessageBridge:
    def __init__(self):
        self.pub_original_output = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=10)
        self.sub_copy_output = rospy.Subscriber('OnRobotRGOutputCopy', OnRobotRGOutputCopy, self.copy_output_callback)

        self.pub_copy_input = rospy.Publisher('OnRobotRGInputCopy', OnRobotRGInputCopy, queue_size=10)
        self.sub_original_input = rospy.Subscriber('OnRobotRGInput', OnRobotRGInput, self.original_input_callback)

    def copy_output_callback(self, msg):
        copy_msg = OnRobotRGOutputCopy()
        copy_msg.rGFR = msg.rGFR
        copy_msg.rGWD = msg.rGWD
        copy_msg.rCTR = msg.rCTR
        self.pub_original_output.publish(copy_msg)

    def original_input_callback(self, msg):
        copy_msg = OnRobotRGInputCopy()
        copy_msg.gFOF = msg.gFOF
        copy_msg.gGWD = msg.gGWD
        copy_msg.gSTA = msg.gSTA
        copy_msg.gWDF = msg.gWDF
        self.pub_copy_input.publish(copy_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('onrobot_message_bridge', anonymous=True)
    bridge = MessageBridge()
    bridge.run()
