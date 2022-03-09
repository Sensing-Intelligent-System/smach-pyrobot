#!/usr/bin/env python

import rospy
import smach
import smach_ros

from smach_tutorials.msg import TestAction, TestGoal
from actionlib import *
from actionlib_msgs.msg import *

import time
from pyrobot import Robot

base_config_dict={'base_controller': 'ilqr'} 
robot = Robot('locobot', base_config=base_config_dict)

# Create a trivial action server
class TestServer:
    def __init__(self,name):
        self._sas = SimpleActionServer(name, TestAction, execute_cb=self.execute_cb)
        self.forward_position = [0.5,0.0,0.0]
        self.backward_position = [0.0,0.0,0.0]
        self.joint_position_1 = [0.408, 0.721, -0.471, -1.4, 0.920]
        self.joint_position_2 = [-0.675, 0, 0.23, 1, -0.70]
        self.counter = 0
        self.cyc = 1

    def execute_cb(self, msg):
        if msg.goal == 0:
            robot.base.go_to_absolute(self.forward_position)
            self._sas.set_succeeded()
            time.sleep(2.5)
        elif msg.goal == 1:
            robot.base.go_to_absolute(self.backward_position)
            self._sas.set_succeeded()
            time.sleep(2.5)
        elif msg.goal == 2:
            robot.arm.set_joint_positions(self.joint_position_1, plan=False)
            robot.gripper.close()
            robot.arm.go_home()
            self._sas.set_succeeded()
            time.sleep(2.5)
        elif msg.goal == 3:
            robot.arm.set_joint_positions(self.joint_position_2, plan=False)
            robot.gripper.open()
            robot.arm.go_home()
            self._sas.set_succeeded()
            time.sleep(2.5)
        elif msg.goal == 4:
            if self.counter == self.cyc:
                self._sas.set_succeeded()
                time.sleep(2.5)
            else:
                self.counter += 1
                self._sas.set_aborted()
                time.sleep(2.5)

if __name__ == '__main__':
    # rospy.init_node('pick_and_place_server')
    server = TestServer('test_action')
    rospy.spin()
