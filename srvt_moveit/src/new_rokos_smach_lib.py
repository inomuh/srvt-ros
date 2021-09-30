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
from move_plan_node import MoveitRokosPlanClass
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
                # Task alindi
                get_tasks = self.get_task_client_func(set_request)

                if get_tasks:
                    home_position = {'Right_Rokos': [[-7.8, -1.146, 0.686, 0.0, 0.0], [3, 1, 2, 4, 5], [00000, 'GENERAL', 0, 'HOME']], 'Left_Rokos': [[-7.8, 1.146, 0.686, 0.0, 0.0], [3, 1, 2, 4, 5], [00000, 'GENERAL', 0, 'HOME']]}
                    # Görevler bittikten sonra home position a gitsin diye home position degerleri tanitildi
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

        self.get_index = 1          # yapilacak hareket sirasi
        #self.time_start = None
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
                # queue_control = mevcut görevdeki siralari inceler, eğer hepsi True ise direk 'Rokos_Take_Photo' state'ine geçmektedir
                queue_control = self.queue_control_func(current_task[1])
                userdata.task_id_output = current_task[2][0]

                if queue_control:
                    # Mevcut indexteki degeri listeden cikarir
                    get_task_list.pop(0)
                    userdata.task_output = get_task_list

                    userdata.current_task_output = current_task[2]  # info

                    state = 'Rokos_Take_Photo'

                else:
                    # mevcut hareket sirasinin dizideki indexini bul
                    # Eğer self.get_index = 1 ise [3, 2, 1, 4, 5] dizideki indexini bulur yani
                    # get_index = 2 degeri olur, yani Z degerininde hareket oldugunu ifade eder
                    get_index = current_task[1].index(self.get_index)
                    userdata.task_output = get_task_list
                    userdata.current_task_output = get_index

                    # State atama
                    # ilk 3 deger X,Y,Z
                    if get_index < 3:
                        state = 'Rokos_Move'
                        self.get_index += 1

                    # Son 2 deger kamera degerleri
                    else:
                        state = 'Rokos_Camera'
                        self.get_index += 2

                    if self.get_index > 5:
                        # yapilacak hareket sirasini sifirla
                        self.get_index = 1

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
        # Son pozisyonu tut
        self.last_position = [-7.8, 0.0, 0.0]


    def execute(self, userdata):
        try:
            # gelen görevler
            get_task_list = list(userdata.task_input)
            # general_selection_state den gelen get_index degeri -> [0, 1, 2] değerlerinden birini içerir
            current_task_index = userdata.current_task_input
            task_id = userdata.task_id_input

            if current_task_index == -1:
                return 'aborted'

            # [[-7.8, -2.84, 1.903, -10.0, -0.0], [1, 3, 2, 4, 5], [35269, '13M38_1', 1, 'ARKA']]
            # get_task_list[0][0] = [-7.8, -2.84, 1.903, -10.0, -0.0]   -> pozisyonlar
            # get_task_list[0][1] = [1, 3, 2, 4, 5]   -> siralar
            # get_task_list[0][2] = [35269, '13M38_1', 1, 'ARKA']   -> bilgiler
            

            # self.last_position ile mevcut görevdeki posizyonu karsilastiriyor
            if self.last_position[current_task_index] == get_task_list[0][0][current_task_index]:
                get_task_list[0][1][current_task_index] = True
                userdata.task_output = get_task_list

                return 'succeeded'

            # Son pozisyondan farkli ise hareket gerceklestir
            else:
                # indis 1 için;
                # Current tasktaki pozisyondaki, [-7.8, -2.84, 1.903, -10.0, -0.0] indis degerini alir, yani -2.84
                # Current task = [None, -2.84, None]
                # Tek eksende hareket etmesi için
                # None eksende hareket yok, hareket gerçekleşmiyor current position uygulanıyor
                current_task = self.get_position_func(get_task_list[0][0], current_task_index)
                success = self.rokos_class.cartesian_path_execution_func(current_task, False)

                # Success, hareketin tolerans degere uygun şekilde tamamlandiysa True dönmekte, False ise state görevi tekrar uygulamaktadır.
                if success:
                    # Belirtilen indiste hareket başarılı gerçekleştirildiğinde Queue daki değeri True olur.
                    get_task_list[0][1][current_task_index] = True
                    userdata.task_output = get_task_list
                    # daha sonra başarılı bir şekilde uyguladığı hareketi self.last_positiona atar
                    self.last_position[current_task_index] = get_task_list[0][0][current_task_index]

                    return 'succeeded'

                elif success == False:
                    print("\n\nError. ID = " + str(userdata.task_id_input) + "\n\n")
                    return 'aborted'

        except Exception as err:
            print(err)
            return 'aborted'

    @classmethod
    def get_position_func(cls, task_list, task_indis):
        # task_indis = 1 için;
        # task_list = [0.5, 2.5, 5.5]
        # sonuc = [None, 2.5, None]
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
        #
        # mantığı rokos_move_state ile aynı
        #

        try:
            get_task_list = list(userdata.task_input)
            current_task_index = userdata.current_task_input
            task_id = userdata.task_id_input

            if current_task_index == -1:
                return 'aborted'

            list_control = True

            # 4, indis olarak 4 te ise 5 indis olarak 3 tedir
            if current_task_index == int(len(get_task_list[0][0]) - 1):
                degrees_list = [get_task_list[0][0][current_task_index - 1], get_task_list[0][0][current_task_index]]
                list_control = False

            # 4, indis olarak 3 te ise 5 indis olarak 4 tedir
            else:
                degrees_list = [get_task_list[0][0][current_task_index], get_task_list[0][0][current_task_index + 1]]
                list_control = True

            radians_list = self.convert_degrees_to_radians(degrees_list)
            success = self.rokos_class.camera_move_joint_state_func(radians_list)

            robot_pose = self.rokos_class.robot.get_current_state()
            # Controller için control
            tolerance_control = RokosClass.camera_tolerance_control_func(robot_pose, radians_list, self.tolerance)

            if tolerance_control:
                # Görev tamamlanarak queue lara True atama işlemi yapilir.
                if list_control:
                    get_task_list[0][1][current_task_index + 1] = True

                else:
                    get_task_list[0][1][current_task_index - 1] = True

                get_task_list[0][1][current_task_index] = True
                userdata.task_output = get_task_list

                return 'succeeded'

            elif tolerance_control == False:
                print("\n\nError. ID = " + str(userdata.task_id_input) + "\n\n")
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

            # infolardaki mode kismini okur, eğer 0 ise görüntü kaydetmez
            if get_current_info[2] == 0:
                print("Mode 0")
                return 'succeeded'

            else:
                print("\n\nTake Photo State\n")
                # infolardaki bilgilere göre görüntü ismi oluşturur.
                image_name = str(str(get_current_info[0]) + "_id_" + str(get_current_info[1]) + "_vehicle_" + str(get_current_info[3]))
                print("Image Name = " + image_name + "\n")

                # görüntü ismi ile service'e istekte bulunur, görüntü kaydetme işlemi başarılı ise get_response değeri succeeded olur.
                get_response = self.get_image_client_func(image_name)

                print("\nTask ID = " + str(userdata.task_id_input))
                print("get response = " + str(get_response) + "\n")

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

