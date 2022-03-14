#!/usr/bin/env python

import rospy
import smach
import smach_ros

from smach_tutorials.msg import TestAction, TestGoal
from actionlib import *
from actionlib_msgs.msg import *

import time
from pyrobot import Robot

def main():
    rospy.init_node('pick_and_place_client')

    # Start an action server
    # server = TestServer('test_action')

    # Create a SMACH state machine
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted', 'End'])

    # Open the container
    with sm0:
        # Add states to the container

        # Add a simple action state. This will use an empty, default goal
        # As seen in TestServer above, an empty goal will always return with
        # GoalStatus.SUCCEEDED, causing this simple action state to return
        # the outcome 'succeeded'
        smach.StateMachine.add('Pick',
                               smach_ros.SimpleActionState('test_action', TestAction,
                               goal = TestGoal(goal=2)),
                               {'succeeded':'Move_to','aborted':'Pick'})

        smach.StateMachine.add('Move_to',
                               smach_ros.SimpleActionState('test_action', TestAction,
                               goal = TestGoal(goal=0)),
                               {'succeeded':'Place','aborted':'Move_to'})

        smach.StateMachine.add('Place',
                               smach_ros.SimpleActionState('test_action', TestAction,
                               goal = TestGoal(goal=3)),
                               {'succeeded':'Move_back','aborted':'Place'})

        smach.StateMachine.add('Move_back',
                               smach_ros.SimpleActionState('test_action', TestAction,
                               goal = TestGoal(goal=1)),
                               {'succeeded':'If_done','aborted':'Move_back'})

        smach.StateMachine.add('If_done',
                               smach_ros.SimpleActionState('test_action', TestAction,
                               goal = TestGoal(goal=4)),
                               {'succeeded':'End','aborted':'Pick'})

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
