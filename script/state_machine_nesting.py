#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pyrobot import Robot

base_config_dict={'base_controller': 'ilqr'} 
robot = Robot('locobot', base_config=base_config_dict)

# define state Base_move
class Base_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['forward','backward'])
        self.counter = 0
        self.forward_position = [0.5,0.0,0.0]
        self.backward_position = [0.0,0.0,0.0]

    def execute(self, userdata):
        rospy.loginfo('Executing state Base_move')
        if self.counter < 1:
            self.counter += 1
            robot.base.go_to_absolute(self.forward_position)
            time.sleep(1)
            return 'forward'
        else:
            robot.base.go_to_absolute(self.backward_position)
            time.sleep(1)
            return 'backward'


# define state Arm_move
class Arm_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arm_move'])
        self.joint_position = [0, 0.3, 0.23, 1, 0]

    def execute(self, userdata):
        rospy.loginfo('Executing state Arm_move')
        robot.arm.set_joint_positions(self.joint_position, plan=False)
        time.sleep(3)
        return 'arm_move'


# define state Camera_move
class Camera_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['camera_move'])
        self.reset_pan = 0.0
        self.reset_tilt = 0.8

    def execute(self, userdata):
        rospy.loginfo('Executing state Camera_move')
        time.sleep(3)
        robot.camera.set_pan(self.reset_pan)
        robot.camera.set_tilt(self.reset_tilt)
        time.sleep(3)
        return 'camera_move'


def main():

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['End'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('Camera_move', Camera_move(), transitions={'camera_move':'SUB'})

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['outcome4'])

        # Open the container
        with sm_sub:

            # Add states to the container
            smach.StateMachine.add('Base_move', Base_move(), transitions={'forward':'Arm_move', 'backward':'outcome'})
            smach.StateMachine.add('Arm_move', Arm_move(), transitions={'arm_move':'Base_move'})

        smach.StateMachine.add('SUB', sm_sub, transitions={'outcome4':'End'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
