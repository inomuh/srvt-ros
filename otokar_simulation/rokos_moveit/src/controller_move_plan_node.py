#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from datetime import datetime
from math import pi
from moveit_commander.conversions import pose_to_list
from log_node import LogClass
from rokos_moveit.srv import *

import os
import yaml
from moveit_msgs.msg import RobotTrajectory
import pickle

class MoveitRokosPlanClass(object):
    """MoveitRokosPlanClass"""
    def __init__(self, g_name, ns=""):
        super(MoveitRokosPlanClass, self).__init__()

        self.ns_name, self.rd_name = self.get_namespace_func(ns)
        group_name = g_name
        self.robot_desc_name = str(self.rd_name + "robot_description")
        self.service_name = str(self.ns_name + "_image_service")
        print(self.service_name)

        #moveit_commander.roscpp_initialize(sys.argv)

        #joint_state_topic = [str('joint_states:=/' + str(self.ns_name) + '/joint_states')]
        #moveit_commander.roscpp_initialize(joint_state_topic)
        #moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander(self.robot_desc_name, self.ns_name)
        scene = moveit_commander.PlanningSceneInterface(self.ns_name)

        group = moveit_commander.MoveGroupCommander(group_name, self.robot_desc_name, self.ns_name)

        display_trajectory_publisher = rospy.Publisher((self.rd_name + 'move_group/display_planned_path'),
                                                      moveit_msgs.msg.DisplayTrajectory,
                                                      queue_size=20)


        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()

        group_names = robot.get_group_names()
        print("Robot Groups:", robot.get_group_names())

        planning_frame = group.get_planning_frame()
        #print("============ Reference frame: %s" % planning_frame)

        eef_link = group.get_end_effector_link()
        #print("============ End effector: %s" % eef_link)

        #print("============ Printing robot state")
        #print robot.get_current_state()

        current_state = robot.get_current_state()
        self.log_class = LogClass(group_name, current_state)

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.box_counter = 0

        self.plan_file_counter = 0
        #self.group.set_planning_time(5)         # set planning time
        self.set_planning_time_value = 5


    @classmethod
    def get_namespace_func(cls, ns):
        ns_name = ""
        rd_name = ""

        if ns != "":
            ns_name = str(ns)
            rd_name = str(str(ns) + "/")

        return ns_name, rd_name


    def main_menu_func(self):
        try:
            control = True

            while control:
                self.menu_yazi()
                menu_value = input("+    Menuden islem seciniz = ")
                print("\n\n")

                if menu_value == "1":
                    #self.rokos_move_position_plan_cartesian_path_func()
                    self.dynamic_go_to_pose_goal()

                elif menu_value == "2":
                    result = self.camera_move_joint_state_func()
                    print(result)

                elif menu_value == "3":
                    result = self.take_photo_service_func()

                    if result:
                        print("\n Fotograf Cekildi \n")

                    else:
                        print("\n Fotograf Cekilirken Hata Olustu \n")

                elif menu_value == "4":
                    #self.new_box()
                    print("here")
                    self.add_mesh_func()

                elif menu_value == "5":
                    plan_value = input("Plan Numarasi Girin = ")
                    read_plan = self.read_plan_format_func(int(plan_value))

                    print("\n\nRead plan = {0}\n\nPlan Type = {1}\n\n".format(read_plan, type(read_plan)))

                elif menu_value == "6":
                    plan_value = input("Plan Numarasi Girin = ")
                    read_plan = self.read_plan_format_func(int(plan_value))
                    radian_list = read_plan.joint_trajectory.points[0].positions[-2:]
                    current_joint = self.group.get_current_joint_values()
                    camera_control = self.camera_tolerance_control_func(current_joint, radian_list)

                    if not camera_control:
                        self.dynamic_camera_move_joint_state_func(radian_list)

                    print("\n\nRead Plan = {0}\n\nPlan Type = {1}\n\n".format(read_plan, type(read_plan)))
                    success = self.plan_execution_func(read_plan)
                    print("Success Mess = {}".format(success))


                elif menu_value == "0":
                    control = False

                else:
                    print("here")
                    continue

                

        except Exception as err:
            print(err)


    def dynamic_camera_move_joint_state_func(self, camera_position_list, tolerance=0.01, wait_value=True):
        try:
            group = self.group

            joint_goal = copy.deepcopy(group.get_current_joint_values())
            joint_goal[4] = camera_position_list[0]
            joint_goal[5] = camera_position_list[1]

            group.go(joint_goal, wait=wait_value)
            group.stop()
            current_joints = group.get_current_joint_values()

            return all_close(joint_goal, current_joints, tolerance)

        except Exception as err:
            print(err)


    @classmethod
    def camera_tolerance_control_func(cls, current_robot_positions, position_list, tolerance=0.01):
        try:
            cam_1_diff = abs(float(current_robot_positions[4] - position_list[0]))
            cam_2_diff = abs(float(current_robot_positions[5] - position_list[1]))
            #print("_\n|\n+-->cam_1_diff = " + str(cam_1_diff))
            #print("current_robot_positions = " + str(current_robot_positions[4]) + " -^.^- position_list = " + str(position_list[0]))
            #print("_\n|\n+-->cam_2_diff = " + str(cam_2_diff))
            #print("current_robot_positions = " + str(current_robot_positions[5]) + " -^.^- position_list = " + str(position_list[1]))

            if (cam_1_diff < tolerance) and (cam_2_diff < tolerance):
                return True

            else:
                return False

        except Exception as err:
            print(err)



    def menu_yazi(self):
        print(chr(27) + "[2J")
        print("+-------------------------------------------+")
        print("|  1-) Rokos Move                           |")
        print("|  2-) Camera Move                          |")
        print("|  3-) Take Photo                           |")
        print("|  4-) Add Object                           |")
        print("|  5-) Read Plan                            |")
        print("|  6-) Execute Read Plan                    |")
        print("|                                           |")
        print("|  0-) Exit                                 |")
        print("+-------------------------------------------+")
        print("|\n|")


    def camera_move_joint_state_func(self):
        try:
            camera_1_value = input("Camera 1 Orientation Degree Value = ")
            camera_2_value = input("Camera 2 Orientation Degree Value = ")

            query_list =list(["", None, "None", "n", " "])

            joint_goal = self.group.get_current_joint_values()
            #print(joint_goal)

            if camera_1_value not in query_list:
                camera_1_radian_value = math.radians(float(camera_1_value))
                #print(camera_1_radian_value)
                joint_goal[4] = camera_1_radian_value
            """
            joint_goal[4] = float(camera_1_value)
            """

            if camera_2_value not in query_list:
                camera_2_radian_value = math.radians(float(camera_2_value))
                #print(camera_2_radian_value)
                joint_goal[5] = camera_2_radian_value

            """
            joint_goal[5] = float(camera_2_value)
            """

            self.group.set_planning_time(0.5)
            print("\n\nPlanning Time 1 = {}\n\n".format(self.group.get_planning_time()))
            self.group.go(joint_goal, wait=True)
            print("\n\nPlanning Time 2 = {}\n\n".format(self.group.get_planning_time()))
            self.group.stop()
            current_joints = self.group.get_current_joint_values()

            return all_close(joint_goal, current_joints, 0.01)

        except Exception as err:
            print(err)


    def rokos_move_position_plan_cartesian_path_func(self):
        try:
            x_value = input("X Coordinate Value = ")
            y_value = input("Y Coordinate Value = ")
            z_value = input("Z Coordinate Value = ")

            query_list =list(["", None, "None", "n", " "])
            waypoints = list()
            wpose = self.group.get_current_pose().pose

            if x_value not in query_list:
                wpose.position.x = float(x_value)

            if y_value not in query_list:
                wpose.position.y = float(y_value)

            if z_value not in query_list:
                wpose.position.z = float(z_value)

            waypoints.append(copy.deepcopy(wpose))

            (plan, fraction) = self.group.compute_cartesian_path(
                                            waypoints,      # waypoints to follow
                                            0.01,           # eef_step
                                            0.0)            # jump_threshold

            #print(plan)

            #self.log_class.main_func(plan)
            self.group.execute(plan, wait=True)

        except Exception as err:
            print(err)


    def take_photo_service_func(self):
        try:
            current_time = self.datenow_func()
            #print(current_time)
            get_response = self.get_image_client_func(str(current_time))

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


    def plan_execution_func(self, plan):
        try:
            group = self.group

            #self.log_class.main_func(plan)
            success = group.execute(plan, wait=True)

            return success

        except Exception as err:
            print(err)



    def go_to_pose_goal_func(self, pose_goal):
        try:
            group = self.group
            group.set_planning_time(self.set_planning_time_value)
            #print("\n\nPlanning Time = {}\n\n".format(group.get_planning_time()))

            #result_0 = group.get_path_constraints()
            #print("\n\nResult 0 = {result}".format(result=result_0))

            #(_, get_current_plan_00, _, _) = group.plan()
            group.set_pose_target(pose_goal)
            #(_, get_current_plan_0, _, _) = group.plan()

            (get_error_code_val, get_current_plan, get_planning_time, get_error_code) = group.plan()
            print("\n\n\nIn Function 1\n-> Get Error Code Value = {0}\n-> Get Planning Time = {1}\n-> Get Error Code = {2}\n\n".format(get_error_code_val, get_planning_time, get_error_code))



            #print("\n\nPlanning Time 1 = {}\n\n".format(group.get_planning_time()))
            plan_exec = group.go(wait=True)

            #(get_plan_control, get_current_plan, get_planning_time, get_error_code) = group.plan()
            #print("\n\nPlan = {plan}\nPlanning Time = {p_time:15f}\n\n".format(plan=get_current_plan, p_time=get_planning_time))
            #print("\nPlanning Time = {p_time:15f}\n".format(p_time=get_planning_time))

            #print("\n\nPlan Type = {}".format(type(get_current_plan)))

            #temp_dict = {"Plan": str(get_current_plan)}
            #temp_dict = get_current_plan
            #len_points_0 = len(get_current_plan_0.joint_trajectory.points)
            #print("\n\nPlan Points = {0}\n\n\nPlan Counts = {1}\n".format(get_current_plan_0.joint_trajectory.points, len_points_0))

            #len_points = len(get_current_plan.joint_trajectory.points)
            #print("\n\nPlan Points = {0}\n\n\nPlan Counts = {1}\n".format(get_current_plan.joint_trajectory.points, len_points))
            #self.save_plan_func(get_current_plan.joint_trajectory.points)
            #print("\n\nPlanning Time 2 = {}\n\n".format(group.get_planning_time()))

            #(get_error_code_val_1, get_current_plan_1, get_planning_time_1, get_error_code_1) = group.plan()
            #print("\n\n\nIn Function 2\n-> Get Error Code Value = {0}\n-> Get Planning Time = {1}\n-> Get Error Code = {2}\n\n".format(get_error_code_val_1, get_planning_time_1, get_error_code_1))
            group.stop()

            (get_error_code_val_2, get_current_plan_2, get_planning_time_2, get_error_code_2) = group.plan()
            print("\n\n\nIn Function 3\n-> Get Error Code Value = {0}\n-> Get Planning Time = {1}\n-> Get Error Code = {2}\n\n".format(get_error_code_val_2, get_planning_time_2, get_error_code_2))
            group.clear_pose_targets()
            
            #len_points_00 = len(get_current_plan_0.joint_trajectory.points)
            #print("\n\n0 -->Plan Points = {0}\n\n\nPlan Counts = {1}\n".format(get_current_plan_00.joint_trajectory.points, len_points_00))

            #len_points_0 = len(get_current_plan_0.joint_trajectory.points)
            #print("\n\n1 -->Plan Points = {0}\n\n\nPlan Counts = {1}\n".format(get_current_plan_0.joint_trajectory.points, len_points_0))
            #temp_plan = get_current_plan.joint_trajectory.points[0].positions
            #len_points = len(get_current_plan.joint_trajectory.points)
            #print("\n\n-->Plan Points = {0}\nPlan Type = {1}\nPlan Counts = {2}".format(get_current_plan.joint_trajectory.points, type(get_current_plan), len_points))
            ##self.save_plan_func(get_current_plan.joint_trajectory.points)
            #self.save_plan_func(get_current_plan)
            #print("\n\nPlanning Time 3 = {}\n\n".format(group.get_planning_time()))




            #(get_error_code_val_1, get_current_plan_1, get_planning_time_1, get_error_code_1) = group.plan()
            #print("\n\n\nIn Function 2\n-> Get Error Code Value = {0}\n-> Get Planning Time = {1}\n-> Get Error Code = {2}\n\n".format(get_error_code_val_1, get_planning_time_1, get_error_code_1))

            print("\n\nPlanning Time = {}\n\n".format(group.get_planning_time()))

            current_pose = self.group.get_current_pose().pose
            #current_pose = self.group.get_current_pose()

            return all_close(pose_goal, current_pose, 0.01), get_current_plan     #0.005     # 0.01

        except Exception as err:
            print(err)


    def dynamic_go_to_pose_goal(self):
        try:
            print("\n\n\n")
            x_value = input("X Coordinate Value = ")
            y_value = input("Y Coordinate Value = ")
            z_value = input("Z Coordinate Value = ")

            set_planning_time = input("Get Planning Time = ")
            query_list =list(["", None, "None", "n", " "])
            
            pose_goal = self.group.get_current_pose().pose

            """
            #pose_goal = geometry_msgs.msg.PoseStamped()
            current_pose = self.group.get_current_pose()
            pose_goal.header = current_pose.header
            pose_goal.pose.orientation.x = current_pose.pose.orientation.x
            pose_goal.pose.orientation.y = current_pose.pose.orientation.y
            pose_goal.pose.orientation.z = current_pose.pose.orientation.z
            pose_goal.pose.orientation.w = current_pose.pose.orientation.w
            pose_goal.pose
            """

            #get_effector_link = self.group.get_end_effector_link()
            #print("\n\nEffector_link = {}".format(get_effector_link))

            if x_value not in query_list:
                pose_goal.position.x = float(x_value)

            if y_value not in query_list:
                pose_goal.position.y = float(y_value)

            if z_value not in query_list:
                pose_goal.position.z = float(z_value)
            
            try:
                if set_planning_time not in query_list:
                    self.set_planning_time_value = float(set_planning_time)

                else:
                    self.set_planning_time_value = 5.0

            except:
                self.set_planning_time_value = 5.0

            """
            if x_value not in query_list:
                pose_goal.pose.position.x = float(x_value)
            else:
                pose_goal.pose.position.x = current_pose.pose.position.x

            if y_value not in query_list:
                pose_goal.pose.position.y = float(y_value)
            else:
                pose_goal.pose.position.y = current_pose.pose.position.y

            if z_value not in query_list:
                pose_goal.pose.position.z = float(z_value)
            else:
                pose_goal.pose.position.z = current_pose.pose.position.z
            """

            result, get_current_plan = self.go_to_pose_goal_func(pose_goal)

            #len_points = len(get_current_plan.joint_trajectory.points)
            #print("\n\n-->Plan Points = {0}\nPlan Type = {1}\nPlan Counts = {2}".format(get_current_plan.joint_trajectory.points, type(get_current_plan), len_points))
            
            print("\n\n\n")
            plan_saving = input("Plan Saving = ")

            if str(plan_saving) == "x" or str(plan_saving) == "X":
                self.save_plan_func(get_current_plan) 


            print("\n\nDynamic Go to Pose Result, {r}".format(r=result))

            #result_constraints = self.group.get_path_constraints()
            #print("\n\nResult Constraints = {result}".format(result=result_constraints))

        except Exception as err:
            print(err)


    def dynamic_position_plan_cartesian_path(self):
        try:
            group = self.group
            control = True

            while control:
                x_value = input("X Coordinate Value = ")
                y_value = input("Y Coordinate Value = ")
                z_value = input("Z Coordinate Value = ")

                query_list =list(["", None, "None", "n", " "])
                waypoints = list()
                wpose = group.get_current_pose().pose

                if x_value not in query_list:
                    wpose.position.x = float(x_value)

                if y_value not in query_list:
                    wpose.position.y = float(y_value)

                if z_value not in query_list:
                    wpose.position.z = float(z_value)

                waypoints.append(copy.deepcopy(wpose))

                (plan, fraction) = group.compute_cartesian_path(
                                                waypoints,      # waypoints to follow
                                                0.01,           # eef_step
                                                0.0)            # jump_threshold

                #print(plan)

                #self.log_class.main_func(plan)
                group.execute(plan, wait=True)

        except Exception as err:
            print(err)

    @classmethod
    def convert_degrees_to_radians(cls, degrees_list):
        radians_list = list()

        for item in degrees_list:
            value = math.radians(item)
            radians_list.append(value)

        return radians_list


    def save_plan_func(self, current_plan):
        try:
            #self.write_plan_yaml_format_func(current_plan)
            self.write_plan_format_func(current_plan, self.plan_file_counter)
            self.plan_file_counter += 1

        except Exception as err:
            print(err)


    def write_plan_yaml_format_func(self, write_data):
        new_file_name = "Deneme_plan_log"
        current_workspace = self.get_current_workspace()

        full_file_name = (str(current_workspace) + 'rokos_moveit/plan_file/' + str(new_file_name) + '.yaml')

        with open(str(full_file_name), 'wb') as outfile:
            yaml.dump(write_data, outfile, default_flow_style=False)


    def read_plan_yaml_format_func(self):
        new_file_name = "Deneme_plan_log"
        current_workspace = self.get_current_workspace()

        full_file_name = (str(current_workspace) + 'rokos_moveit/plan_file/' + str(new_file_name) + '.yaml')

        with open(str(full_file_name), 'rb') as load_file:
            read_yaml = yaml.safe_load(load_file)


        return read_yaml


    def write_plan_format_func(self, write_data, file_counter):
        new_file_name = str("Deneme_plan_log_v_" + str(file_counter))
        current_workspace = self.get_current_workspace()

        full_file_name = (str(current_workspace) + 'rokos_moveit/plan_file/' + new_file_name + '.dat')

        with open(str(full_file_name), 'wb') as outfile:
            pickle.dump(write_data, outfile, pickle.HIGHEST_PROTOCOL)

        outfile.close()


    def read_plan_format_func(self, file_counter):
        new_file_name = str("Deneme_plan_log_v_" + str(file_counter))
        current_workspace = self.get_current_workspace()

        full_file_name = (str(current_workspace) + 'rokos_moveit/plan_file/' + str(new_file_name) + '.dat')

        with open(str(full_file_name), 'rb') as load_file:
            read_yaml = pickle.load(load_file)

        return read_yaml

    """
    def write_plan_format_func(self, write_data):
        new_file_name = "Deneme_plan_log"
        current_workspace = self.get_current_workspace()

        full_file_name = (str(current_workspace) + 'rokos_moveit/plan_file/' + str(new_file_name) + '.txt')

        with open(str(full_file_name), 'wb') as outfile:
            outfile.write(write_data)

        outfile.close()


    def read_plan_format_func(self):
        new_file_name = "Deneme_plan_log"
        current_workspace = self.get_current_workspace()

        full_file_name = (str(current_workspace) + 'rokos_moveit/plan_file/' + str(new_file_name) + '.yaml')

        with open(str(full_file_name), 'rb') as load_file:
            read_yaml = load_file.read()

        #load_file = file(str(full_file_name), 'r')
        #temp_dict = yaml.load(load_file)

        return read_yaml
    """

    @classmethod
    def get_current_workspace(cls):
        file_full_path = os.path.dirname(os.path.realpath(__file__))
        directory_name = sys.argv[0].split('/')[-3]
        workspace_name = file_full_path.split(str(directory_name))[0]

        return workspace_name

    @classmethod
    def datenow_func(cls):
        now = datetime.now()
        dt_string = now.strftime("%Y_%m_%d_-_%H_%M_%S")

        return str(dt_string)


    def new_box(self):
        name = "Test_Box"
        box_pose_x = float(-5.0 * float(3.0 * self.box_counter))
        pose = [box_pose_x, -2.0, 1.0, 1.0]
        dimensions = [2.5, 2.5, 2.5]
        

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]

        p.pose.orientation.w = pose[3]

        self.scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))
        self.box_counter += 1


    def add_mesh_func(self):

        file_name = "/home/ros/Desktop/VALU3S/Otokar/deneme3.stl"
        name = "Otokar_Sase"

        # Right
        pose = [-1.86, -3.165, 2.3]
        dimensions = [0.5, 0.5, 0.5]

        # Left
        #pose = [-1.86, 3.165, 2.3]
        #dimensions = [0.5, 0.5, 0.5]

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]

        p.pose.orientation.w = 1.0

        self.scene.add_mesh(name, p, file_name, (dimensions[0], dimensions[1], dimensions[2]))


    def add_box(self, timeout=4):
        box_name = "Box_Name"
        scene = self.scene
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = -3.0
        box_pose.pose.position.z = 1.0
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(2.5, 2.5, 2.5))

        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            print("\n\nattached_objects \n")
            print(attached_objects)
            print(scene.get_attached_objects())
            print("\n\n")

            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()


            print("\n\nscene.get_known_object_names()")
            print(scene.get_known_object_names())


            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


#----------------------------------------------------------------------------

def get_current_workspace():
    file_full_path = os.path.dirname(os.path.realpath(__file__))
    directory_name = sys.argv[0].split('/')[-3]
    workspace_name = file_full_path.split(str(directory_name))[0]

    return workspace_name


def read_plan_format_func():
    new_file_name = "Deneme_plan_log"
    current_workspace = get_current_workspace()

    full_file_name = (str(current_workspace) + 'rokos_moveit/plan_file/' + str(new_file_name) + '.dat')

    with open(str(full_file_name), 'rb') as load_file:
        read_yaml = pickle.load(load_file)

    return read_yaml

def test_func():
    temp_list = [[0,1,2,3,4,5], [3,4,5,6,7,8]]

    radian_list = temp_list[0][-2:]

    print("Radian list = {}".format(radian_list))

    #read_plan = read_plan_format_func()
    #len_points = len(read_plan.joint_trajectory.points)
#
    #print("\n\nRead plan = {0}\n\nPlan type = {1}\nPlan Count = {2}".format(read_plan, type(read_plan), len_points))


#----------------------------------------------------------------------------

def all_close(goal, actual, tolerance):
    try:
        all_equal = True

        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

    except Exception as err:
        print(err)


def main_func(param=""):
    try:
        if param == "":
            #param = "left"
            param = "right"

        g_name = str("rokos_" + str(param) + "_arm")
        ns_name = str(str(param) + "_rokos")

        moveit_class = MoveitRokosPlanClass(g_name, ns_name)

        print("Start Cartesian Plan Use Case")
        print("\n\n")

        moveit_class.main_menu_func()
        

        #moveit_class.camera_move_joint_state_func()


        """
        # Sadece XYZ icin
        moveit_class.dynamic_position_plan_cartesian_path()
        """

        print("\n\n")
        

        print("Finish cartesian plan")

    except rospy.ROSInterruptException as err:
        print("Error: " + str(err))
        return
    except KeyboardInterrupt as err:
        print("Error: " + str(err))
        return
    except Exception as err:
        print("Error: " + str(err))
        return


if __name__ == '__main__':
    rospy.init_node('move_plan_node', anonymous=True)
    param = ""

    # Default Right, Left icin consoleda left yazmak gerekiyor.
    if len(sys.argv) > 1:
        param = sys.argv[1]

    #param = "right"


    main_func(param)


    #test_func()
