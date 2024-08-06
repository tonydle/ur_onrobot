#!/usr/bin/env python3

import rospy
from onrobot_rg_control import baseOnRobotRG
import ur_onrobot_rg_modbus_serial.comModbusSerial
from std_srvs.srv import Trigger, TriggerResponse
from onrobot_rg_control.msg import OnRobotRGInput
from onrobot_rg_control.msg import OnRobotRGOutput
from ur_dashboard_msgs.msg import RobotMode

class OnRobotRGSerial:
    """ OnRobotRGSerial connects to the gripper with Modbus Serial.

        Attributes:
            gripper (onrobot_rg_control.baseOnRobotRG.onrobotbaseRG):
                instance of onrobotbaseRG used for the connection establishment
            pub (rospy.Publisher): the publisher for OnRobotRGInput
            mode (int): current mode of the robot
            restartPowerCycle:
                Restarts the power cycle of the gripper.
            mainLoop:
                Loops the sending status and command, and receiving message.
    """

    def __init__(self):
        self.gripper = baseOnRobotRG.onrobotbaseRG(gtype)
        self.gripper.client = ur_onrobot_rg_modbus_serial.comModbusSerial.communication(dummy)

        self.gripper.client.connectToDevice(device, changer_addr)

        self.pub = rospy.Publisher('OnRobotRGInput', OnRobotRGInput, queue_size=1)
        rospy.Subscriber('OnRobotRGOutput', OnRobotRGOutput, self.gripper.refreshCommand)
        rospy.Subscriber('/ur/ur_hardware_interface/robot_mode', RobotMode, self.robotModeCallback)

        rospy.Service('restart_power', Trigger, self.restartPowerCycle)

        self.mode = None
        self.prev_msg = []
        self.rate = rospy.Rate(20)  # 20 Hz
        self.mainLoop()

    def restartPowerCycle(self, request):
        rospy.loginfo("Restarting the power cycle of all grippers connected.")
        self.gripper.restartPowerCycle()
        rospy.sleep(1)
        return TriggerResponse(success=True, message="Power cycle restarted.")

    def robotModeCallback(self, msg):
        self.mode = msg.mode

    def mainLoop(self):
        while not rospy.is_shutdown():
            if self.mode == 7:
                status = self.gripper.getStatus()
                self.pub.publish(status)

                rospy.sleep(0.05)
                if not int(format(status.gSTA, '016b')[-1]):  # not busy
                    if self.prev_msg != self.gripper.message:  # find new message
                        rospy.loginfo(rospy.get_name() + ": Sending message.")
                        self.gripper.sendCommand()
                self.prev_msg = self.gripper.message
                rospy.sleep(0.05)
            else:
                self.rate.sleep()

if __name__ == '__main__':
    try:
        device = rospy.get_param('/onrobot/device', '/tmp/ttyUR')
        gtype = rospy.get_param('/onrobot/gripper', 'rg2')
        changer_addr = rospy.get_param('/onrobot/changer_addr', '65')
        dummy = rospy.get_param('/onrobot/dummy', False)
        rospy.init_node('OnRobotRGSerialNode', anonymous=True, log_level=rospy.DEBUG)
        OnRobotRGSerial()
    except rospy.ROSInterruptException:
        pass
