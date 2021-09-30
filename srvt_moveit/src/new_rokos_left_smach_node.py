#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import smach
import smach_ros

from smach import State
from smach import StateMachine

#import new_rokos_smach_lib as rsl
#from move_plan_node import MoveitRokosPlanClass

#import new_rokos_smach_lib_v2 as rsl
#from move_plan_node_v2 import MoveitRokosPlanClass

import new_rokos_smach_lib_v3 as rsl
from move_plan_node_v3 import MoveitRokosPlanClass

from actionlib import *
from actionlib_msgs.msg import *


def smach_main_func():
    try:
        rospy.init_node('rokos_left_smach_moveit_node')

        rokos_type = "left_rokos"
        rokos_left_class = MoveitRokosPlanClass("rokos_left_arm", rokos_type)

        # Create the top level SMACH state machine
        sm_top = smach.StateMachine(outcomes=['done'])

        # Open the container
        with sm_top:

            smach.StateMachine.add('Start_State', rsl.start_states(),
                                transitions={'succeeded':'Move_CON', 'aborted': 'Start_State'})

            # Create the sub SMACH state machine
            sm_move_con = smach.StateMachine(outcomes=['rokos_left_done'])

            sm_move_con.userdata.task = list()

            # Open the container
            with sm_move_con:

                smach.StateMachine.add('Rokos_Left_Get_Tasks', rsl.rokos_get_tasks_state("left_rokos_task_service"),
                                    transitions={'succeeded':'General_Selection','aborted':'Rokos_Left_Get_Tasks', 'other': 'rokos_left_done'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task'})

                smach.StateMachine.add('General_Selection', rsl.general_selection_state(),
                                    transitions={'Rokos_Move': 'Rokos_Left_Move', 'Rokos_Camera': 'Rokos_Left_Camera', 'Rokos_Take_Photo': 'Rokos_Left_Take_Photo', 'Get_Task': 'Rokos_Left_Get_Tasks'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_output':'current_task',
                                                'task_id_output':'task_id'})

                smach.StateMachine.add('Rokos_Left_Move', rsl.rokos_move_state(rokos_left_class),
                                    transitions={'succeeded':'General_Selection','aborted':'Rokos_Left_Move'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})

                smach.StateMachine.add('Rokos_Left_Camera', rsl.rokos_camera_state(rokos_left_class),
                                    transitions={'succeeded':'General_Selection','aborted':'Rokos_Left_Camera'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})
                
                smach.StateMachine.add('Rokos_Left_Take_Photo', rsl.rokos_take_photo_state("left_rokos_image_service"),
                                    transitions={'succeeded':'General_Selection','aborted':'Rokos_Left_Take_Photo'},
                                    remapping={ 'task_input':'task',
                                                'task_output':'task',
                                                'current_task_input':'current_task',
                                                'task_id_input':'task_id'})


            smach.StateMachine.add('Move_CON', sm_move_con,
                                transitions={'rokos_left_done':'done'})


        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROKOS_LEFT_TASK_SMACH')
        sis.start()
        sm_top.execute()
        rospy.spin()
        sis.stop()

    except Exception as err:
        print(err)

if __name__ == '__main__':
    smach_main_func()
