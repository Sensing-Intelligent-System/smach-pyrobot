#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from pyrobot import Robot

base_config_dict={'base_controller': 'ilqr'} 
robot = Robot('locobot', base_config=base_config_dict)

# define state Init
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init'], input_keys=['Init_in'], output_keys=['Init_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Init')
        rospy.loginfo('Counter = %f'%userdata.Init_in)
        userdata.Init_out = userdata.Init_in + 1
        time.sleep(2)
        return 'init'


# define state Base_move
class Base_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['forward','backward'], input_keys=['Base_move_in'], output_keys=['Base_move_out'])
        self.counter = 0
        self.forward_position = [0.5,0.0,0.0]
        self.backward_position = [0.0,0.0,0.0]

    def execute(self, userdata):
        rospy.loginfo('Executing state Base_move')
        rospy.loginfo('Counter = %f'%userdata.Base_move_in)
        userdata.Base_move_out = userdata.Base_move_in + 1
        if self.counter < 1:
            self.counter += 1
            robot.base.go_to_absolute(self.forward_position)
            time.sleep(2)
            return 'forward'
        else:
            robot.base.go_to_absolute(self.backward_position)
            time.sleep(2)
            return 'backward'


# define state Arm_move
class Arm_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arm_home','arm_move'], input_keys=['Arm_move_in'], output_keys=['Arm_move_out'])
        self.counter = 0
        self.joint_position = [0, 0.3, 0.23, 1, 0]

    def execute(self, userdata):
        rospy.loginfo('Executing state Arm_move')
        rospy.loginfo('Counter = %f'%userdata.Arm_move_in)
        userdata.Arm_move_out = userdata.Arm_move_in + 1
        if self.counter < 1:
            self.counter += 1
            robot.arm.set_joint_positions(self.joint_position, plan=False)
            time.sleep(1)
            return 'arm_move'
        else:
            robot.arm.go_home()
            time.sleep(1)
            return 'arm_home'

def main():

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['End'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Init', Init(), transitions={'init':'Arm_move'}, 
                               remapping={'Init_in':'sm_counter', 'Init_out':'sm_counter'})
        smach.StateMachine.add('Base_move', Base_move(), transitions={'forward':'Arm_move', 'backward':'End'},
                               remapping={'Base_move_in':'sm_counter', 'Base_move_out':'sm_counter'})
        smach.StateMachine.add('Arm_move', Arm_move(), transitions={'arm_home':'Base_move','arm_move':'Base_move'},
                               remapping={'Arm_move_in':'sm_counter', 'Arm_move_out':'sm_counter'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
