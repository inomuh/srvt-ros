#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import smach
import smach_ros
import math
import time
import copy
from datetime import datetime
from dateutil.relativedelta import relativedelta

from smach import State
from smach import StateMachine

from task_log_node import TaskLogClass
from move_plan_node_v2 import MoveitRokosPlanClass
from class_rokos import RokosClass
from class_image_saver import ImageSaver
from std_msgs.msg import String
from srvt_moveit.srv import *
from actionlib import *
from actionlib_msgs.msg import *


class start_states(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.counter = 0

    def execute(self, userdata):
        try:
            if self.counter < 3:
                self.counter += 1
                print(self.counter)
                time.sleep(1)

                return 'aborted'

            else:
                print("Start Rokos Use Case")
                self.counter = 0

                return 'succeeded'

        except Exception as err:
            print(err)
            return 'aborted'


class rokos_get_tasks_state(smach.State):
    def __init__(self, service_name):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted', 'other'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output'])

        self.count = 0
        self.service_name = service_name
        self.rokos_type = self.select_rokos_type(self.service_name)
        general_selection_state.rokos_type = self.rokos_type


    def execute(self, userdata):
        try:
            rospy.loginfo(str(self.rokos_type) + " Get Task")
            if self.count < -1:
                return 'other'

            if not list(userdata.task_input):
                self.count += 1
                set_request = str(self.rokos_type + " Tasks " + str(self.count))
                get_tasks = self.get_task_client_func(set_request)

                if get_tasks:
                    home_position = {'Right_Rokos': [[-7.8, -1.146, 0.686, 0.0, 0.0], [5, 3, 4, 1, 2], [00000, 'GENERAL', 0, 'HOME']], 'Left_Rokos': [[-7.8, 1.146, 0.686, 0.0, 0.0], [5, 3, 4, 1, 2], [00000, 'GENERAL', 0, 'HOME']]}
                    get_tasks.append(home_position[self.rokos_type])
                    task_copy = copy.deepcopy(get_tasks)
                    userdata.task_output = get_tasks
                    rospy.loginfo("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n-> Task Accepted\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n")
                    print("\n\nGet Tasks = " + str(get_tasks) + "\n\n")

                    general_selection_state.mission_list = task_copy
                    general_selection_state.time_start = datetime.now()

                    return 'succeeded'

                else:
                    print("Wait_Task")
                    time.sleep(3)
                    return 'aborted'

            else:
                return 'succeeded'

        except Exception as err:
            print(err)
            return 'aborted'


    def get_task_client_func(self, request):
        rospy.wait_for_service(str(self.service_name))

        try:
            rokos_task_service = rospy.ServiceProxy(str(self.service_name), TaskService)
            response = rokos_task_service.call(TaskServiceRequest(request))

            get_task = list(eval(response.response))

            return get_task

        except rospy.ServiceException as err:
            print("Service call failed: " + str(err))

    @classmethod
    def select_rokos_type(cls, service_name):
        try:
            if 'left' in service_name:
                return 'Left_Rokos'

            elif 'right' in service_name:
                return 'Right_Rokos'

            else:
                return 'No-Name'

        except Exception as err:
            print(err)


class general_selection_state(smach.State):
    rokos_type = None
    time_start = None
    mission_list = None

    def __init__(self):
        smach.State.__init__(self,  outcomes=['Rokos_Move', 'Rokos_Camera', 'Rokos_Take_Photo', 'Get_Task'],
                                    input_keys=['task_input'],
                                    output_keys=['task_output', 'current_task_output', 'task_id_output'])

        self.get_index = 1
        self.state_control = True
        self.time_finish = None
        self.rokos_task_log = TaskLogClass()

    def execute(self, userdata):
        try:
            get_task_list = userdata.task_input

            if len(get_task_list) == 0:
                self.time_finish = datetime.now()

                if self.time_start != None and self.time_finish != None:
                    time_result = self.get_time_diff(self.time_start, self.time_finish)
                    print("\n\n--> Start_Time = " + str(self.time_start) + "\n--> Finish_Time = " + str(self.time_finish))
                    print("\n\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n\n")
                    print("Time Diff = " + str(time_result))
                    print("\n\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n\n")
                    self.rokos_task_log.log_file_func(self.rokos_type, self.time_start, self.time_finish, time_result, self.mission_list)

                    time.sleep(3)

                print("\n\nGet Task\n\n")

                return 'Get_Task'

            else:
                current_task = get_task_list[0]
                queue_control = self.queue_control_func(current_task[1])
                userdata.task_id_output = current_task[2][0]

                if queue_control:
                    get_task_list.pop(0)
                    self.state_control = True
                    userdata.task_output = get_task_list
                    userdata.current_task_output = current_task[2]

                    state = 'Rokos_Take_Photo'

                else:
                    get_index = current_task[1].index(self.get_index)
                    userdata.task_output = get_task_list

                    if get_index < 3:
                        state = 'Rokos_Move'
                        self.get_index += 3

                    else:
                        state = 'Rokos_Camera'
                        self.state_control = False
                        self.get_index += 2

                    if self.get_index > 5:
                        self.get_index = 1

                    userdata.current_task_output = self.state_control

                return state

        except Exception as err:
            print(err)

    @classmethod
    def queue_control_func(cls, queue_list):
        try:
            counter = 0

            for item in queue_list:
                if item == True:
                    counter += 1

            if counter == len(queue_list):
                return True

            else:
                return False

        except Exception as err:
            print(err)

    @classmethod
    def get_time_diff(cls, time_start, time_finish):
        t_diff = relativedelta(time_finish, time_start)

        return '{h}:{m}:{s}.{ms}'.format(h=t_diff.hours, m=t_diff.minutes, s=t_diff.seconds, ms=t_diff.microseconds)


class rokos_move_state(smach.State):
    def __init__(self, rokos_class):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input', 'current_task_input', 'task_id_input'],
                                    output_keys=['task_output'])

        self.rokos_class = rokos_class
        self.last_position = list([-7.8, 0.0, 0.0])
        self.r_class = RokosClass()


    def execute(self, userdata):
        try:
            get_task_list = list(userdata.task_input)
            current_task_control = userdata.current_task_input
            task_id = userdata.task_id_input

            # Bu bölümde X,Z eş zamanlı, Y ekseni farklı şekilde hareket işlemi gerçekleştirilir.
            current_task = copy.deepcopy(get_task_list[0][0][:3])
            current_tasks, current_task_type = self.r_class.set_custom_current_task_new_func(current_task, current_task_control, self.last_position)

            # [[None, None, None], [None, None, None]] durumunda hareket işlemi olmayacağı için state'i hareket işlemi gerçekleştirmeden sonlandırır. 
            if current_tasks[0] == False and current_task_type == False:
                success = True

            else:
                # Mevcut görevi planlayıcıya gönderir ve execute eder.
                success = self.rokos_class.cartesian_path_execution_func(current_tasks, current_task_type)

            if success:
                # Mevcut görevin Queue listesine True atar.
                get_task_list[0][1][0] = True
                get_task_list[0][1][1] = True
                get_task_list[0][1][2] = True
                userdata.task_output = get_task_list
                # Görevi tamamladığı için son pozisyonu günceller.
                self.last_position = current_task

                return 'succeeded'

            elif success == False:
                print("\n\nError. ID = " + str(task_id) + "\n\n")
                return 'aborted'

        except Exception as err:
            print(err)
            return 'aborted'

    @classmethod
    def get_position_func(cls, task_list, task_indis):
        try:
            position_list = list()

            for item in range(3):
                if item == task_indis:
                    value = task_list[task_indis]

                else:
                    value = None

                position_list.append(value)

            return position_list

        except Exception as err:
            print(err)


class rokos_camera_state(smach.State):
    def __init__(self, rokos_class):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input', 'current_task_input', 'task_id_input'],
                                    output_keys=['task_output'])

        self.rokos_class = rokos_class
        self.tolerance = 0.1


    def execute(self, userdata):
        try:
            get_task_list = list(userdata.task_input)
            current_task_index = userdata.current_task_input
            task_id = userdata.task_id_input

            degrees_list = [get_task_list[0][0][3], get_task_list[0][0][4]]

            radians_list = self.convert_degrees_to_radians(degrees_list)
            success = self.rokos_class.camera_move_joint_state_func(radians_list)

            robot_pose = self.rokos_class.robot.get_current_state()
            tolerance_control = RokosClass.camera_tolerance_control_func(robot_pose, radians_list, self.tolerance)

            if tolerance_control:
                get_task_list[0][1][3] = True
                get_task_list[0][1][4] = True

                userdata.task_output = get_task_list

                return 'succeeded'

            elif tolerance_control == False:
                print("\n\nError. ID = {}\n\n".format(userdata.task_id_input))
                return 'aborted'

        except Exception as err:
            print(err)
            return 'aborted'
          
    @classmethod
    def convert_degrees_to_radians(cls, degrees_list):
        try:
            new_list = list()
            for item in degrees_list:
                value = math.radians(item)
                new_list.append(value)

            return new_list

        except Exception as err:
            print(err)


class rokos_take_photo_state(smach.State):
    def __init__(self, service_name):
        smach.State.__init__(self,  outcomes=['succeeded', 'aborted'],
                                    input_keys=['task_input', 'current_task_input', 'task_id_input'],
                                    output_keys=['task_output'])

        self.service_name = service_name

    def execute(self, userdata):
        try:
            get_task_list = list(userdata.task_input)
            userdata.task_output = get_task_list
            get_current_info = list(userdata.current_task_input)
            task_id = userdata.task_id_input

            if get_current_info[2] == 0:
                print("Mode 0")
                return 'succeeded'

            else:
                print("\n\nTake Photo State\n")
                image_name = str(str(get_current_info[0]) + "_id_" + str(get_current_info[1]) + "_vehicle_" + str(get_current_info[3]))
                print("Image Name = {}\n".format(image_name))

                get_response = self.get_image_client_func(image_name)

                print("\nTask ID = {}".format(userdata.task_id_input))
                print("get response = {}\n".format(get_response))

                return get_response

        except Exception as err:
            print(err)


    def get_image_client_func(self, request):
        rospy.wait_for_service(str(self.service_name))

        try:
            rokos_image_service = rospy.ServiceProxy(str(self.service_name), ImageService)
            response = rokos_image_service.call(ImageServiceRequest(request))

            get_response = str(response.response)

            return get_response

        except rospy.ServiceException as err:
            print("Service call failed: " + str(err))

