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

    def execute_cb(self, msg):
        if msg.goal == 0:
            self._sas.set_succeeded()
            time.sleep(3)
            robot.base.go_to_absolute(self.forward_position)
            time.sleep(3)
        elif msg.goal == 1:
            self._sas.set_aborted()
            robot.base.go_to_absolute(self.backward_position)
            time.sleep(3)
        elif msg.goal == 2:
            self._sas.set_preempted()
            time.sleep(3)

def main():

    # Start an action server
    server = TestServer('test_action')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    # Open the container
    with sm0:
        # Add states to the container

        # Add a simple action state. This will use an empty, default goal
        # As seen in TestServer above, an empty goal will always return with
        # GoalStatus.SUCCEEDED, causing this simple action state to return
        # the outcome 'succeeded'
        smach.StateMachine.add('GOAL_DEFAULT',
                               smach_ros.SimpleActionState('test_action', TestAction),
                               {'succeeded':'GOAL_Forward'})

        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.
        smach.StateMachine.add('GOAL_Forward',
                               smach_ros.SimpleActionState('test_action', TestAction,
                               goal = TestGoal(goal=1)),
                               {'aborted':'GOAL_Backward'})

        
        # Add another simple action state. This will give a goal
        # that should abort the action state when it is received, so we
        # map 'aborted' for this state onto 'succeeded' for the state machine.
        def goal_callback(userdata, default_goal):
            goal = TestGoal()
            goal.goal = 2
            return goal

        smach.StateMachine.add('GOAL_Backward',
                               smach_ros.SimpleActionState('test_action', TestAction,
                               goal_cb = goal_callback),
                               {'preempted':'End'})

        # For more examples on how to set goals and process results, see 
        # executive_smach/smach_ros/tests/smach_actionlib.py

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm0, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm0.execute()

    #rospy.signal_shutdown('All done.')

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
