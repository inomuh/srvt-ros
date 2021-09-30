#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from moveit_commander.conversions import pose_to_list
from log_node import LogClass

class MoveitRokosPlanClass(object):
    """MoveitRokosPlanClass"""
    def __init__(self, g_name, ns=""):
        super(MoveitRokosPlanClass, self).__init__()

        self.ns_name, self.rd_name = self.get_namespace_func(ns)
        group_name = g_name
        self.robot_desc_name = str(self.rd_name + "robot_description")

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
        #print "============ Reference frame: %s" % planning_frame

        eef_link = group.get_end_effector_link()
        #print "============ End effector: %s" % eef_link

        #print "============ Printing robot state"
        #print robot.get_current_state()
        #print ""

        current_state = robot.get_current_state()
        self.log_class = LogClass(group_name, current_state)

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def get_test_func(self):
        return self.ns_name

    @classmethod
    def get_namespace_func(cls, ns):
        ns_name = ""
        rd_name = ""

        if ns != "":
            ns_name = str(ns)
            rd_name = str(str(ns) + "/")

        return ns_name, rd_name


    def go_to_joint_state(self):
        group = self.group

        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        group.go(joint_goal, wait=True)
        group.stop()
        current_joints = self.group.get_current_joint_values()

        return all_close(joint_goal, current_joints, 0.01)


    def camera_init_state(self):
        group = self.group

        joint_goal = group.get_current_joint_values()
        joint_goal[4] = 0
        joint_goal[5] = 0

        group.go(joint_goal, wait=True)
        group.stop()
        current_joints = self.group.get_current_joint_values()

        return all_close(joint_goal, current_joints, 0.01)


    def camera_move_joint_state_func(self, camera_position_list):
        try:
            group = self.group

            joint_goal = group.get_current_joint_values()
            joint_goal[4] = camera_position_list[0]
            joint_goal[5] = camera_position_list[1]

            group.go(joint_goal, wait=True)
            group.stop()
            current_joints = self.group.get_current_joint_values()

            return all_close(joint_goal, current_joints, 0.01)

        except Exception as err:
            print(err)


    def cartesian_path_execution_func(self, position_list, list_type):
        """
            list_type = bool
                True = [ [x0, y0, z0], [x1, y1, z1], ...]
                False = [x, y, z]
        """
        try:
            group = self.group
            wpose = group.get_current_pose().pose

            waypoints = self.create_path_plan_func(position_list, wpose, list_type)

            (plan, fraction) = group.compute_cartesian_path(
                                            waypoints,   # waypoints to follow
                                            0.01,        # eef_step
                                            0.0)         # jump_threshold

            self.log_class.main_func(plan)
            success = group.execute(plan, wait=True)

            return success
        
        except Exception as err:
            print(err)


    def create_path_plan_func(self, position_list, wpose, list_type):
        try:
            waypoints = list()

            if position_list:
                if list_type:
                    for waypoint in position_list:
                        schema_waypoints = self.schema_path_plan_func(waypoint, wpose)
                        waypoints.append(schema_waypoints)

                else:
                    schema_waypoints = self.schema_path_plan_func(position_list, wpose)
                    waypoints.append(schema_waypoints)


            return waypoints

        except Exception as err:
            print(err)

    @classmethod
    def schema_path_plan_func(cls, position_list, wpose):
        try:
            for index, point in enumerate(position_list):
                if point != None:
                    if index == 0:
                        wpose.position.x = point
                    elif index == 1:
                        wpose.position.y = point
                    elif index == 2:
                        wpose.position.z = point

            schema_waypoints = copy.deepcopy(wpose)

            return schema_waypoints

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

                query_list =list(["", None, "None", "n"])
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

                print(plan)

                self.log_class.main_func(plan)
                group.execute(plan, wait=True)

        except Exception as err:
            print(err)


    def dynamic_plan_cartesian_path(self):
        try:
            group = self.group
            control = True

            while control:
                x_value = input("X Coordinate Value = ")

                if str(x_value) == "x":
                    control = False
                    break

                else:
                    waypoints = []

                    wpose = group.get_current_pose().pose
                    wpose.position.y = float(x_value)
                    waypoints.append(copy.deepcopy(wpose))

                    (plan, fraction) = group.compute_cartesian_path(
                                                    waypoints,      # waypoints to follow
                                                    0.01,           # eef_step
                                                    0.0)            # jump_threshold

                    print(plan)

                    self.log_class.main_func(plan)
                    group.execute(plan, wait=True)

        except Exception as err:
            print(err)


    def execute_plan(self, plan):
        group = self.group
        group.execute(plan, wait=True)



def all_close(goal, actual, tolerance):
    try:
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
