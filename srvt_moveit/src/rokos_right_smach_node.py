#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rokos Smach Lib for right rokos"""
#import roslib
import rospy
import smach
import smach_ros

#from smach import State
#from smach import StateMachine

import rokos_smach_lib as rsl
from move_plan_node import MoveitPlanClass

#from actionlib import *
#from actionlib_msgs.msg import *


def right_rokos_smach_main_func():
    """Smach main function"""
    try:
        rospy.init_node('rokos_right_smach_moveit_node')

        rokos_type = "right_rokos"
        rokos_right_class = MoveitPlanClass("rokos_right_arm", rokos_type)

        # Create the top level SMACH state machine
        sm_top = smach.StateMachine(outcomes=['done'])

        # Open the container
        with sm_top:

            smach.StateMachine.add('Start_State', rsl.SRVTSmach(),
                                transitions={'succeeded':'Move_CON', 'aborted': 'Start_State'})

            # Create the sub SMACH state machine
            sm_move_con = smach.StateMachine(outcomes=['rokos_right_done'])
            sm_move_con.userdata.task = []

            # Open the container
            with sm_move_con:
                smach.StateMachine.add('Rokos_Right_Get_Tasks',
                 rsl.RokosGetTasksState("right_rokos_task_service"),
                                    transitions={'succeeded':'General_Selection',
                                                 'aborted':'Rokos_Right_Get_Tasks',
                                                 'other': 'rokos_right_done'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task'})

                smach.StateMachine.add('General_Selection',
                 rsl.GeneralSelectionState(),
                                    transitions={'Rokos_Move': 'Rokos_Right_Move',
                                                 'Rokos_Camera': 'Rokos_Right_Camera',
                                                 'Rokos_Take_Photo': 'Rokos_Right_Take_Photo',
                                                 'Get_Task': 'Rokos_Right_Get_Tasks'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_output':'current_task',
                                                'task_id_output':'task_id'})

                smach.StateMachine.add('Rokos_Right_Move',
                 rsl.RokosMoveState(rokos_right_class),
                                    transitions={'succeeded':'General_Selection',
                                                 'aborted':'Rokos_Right_Move'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})

                smach.StateMachine.add('Rokos_Right_Camera',
                 rsl.RokosCameraState(rokos_right_class),
                                    transitions={'succeeded':'General_Selection',
                                                 'aborted':'Rokos_Right_Camera'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})

                smach.StateMachine.add('Rokos_Right_Take_Photo',
                 rsl.RokosTakePhotoState("right_rokos_image_service"),
                                    transitions={'succeeded':'General_Selection',
                                                 'aborted':'Rokos_Right_Take_Photo'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})


            smach.StateMachine.add('Move_CON', sm_move_con,
                                transitions={'rokos_right_done':'done'})


        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROKOS_RIGHT_TASK_SMACH')
        sis.start()
        sm_top.execute()
        rospy.spin()
        sis.stop()

    except Exception as err:
        print(err)

if __name__ == '__main__':
    right_rokos_smach_main_func()
