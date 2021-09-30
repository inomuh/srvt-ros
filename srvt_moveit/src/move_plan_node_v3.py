#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from moveit_commander.conversions import pose_to_list
from log_node import LogClass
from tf.transformations import quaternion_from_euler


class MoveitRokosPlanClass(object):
    """
        MoveitRokosPlanClass

        Rokos robot kollarindaki planner ve controllerlari kullanmak için moveit kütüphanelerinin bulunduğu classtır.
    """
    def __init__(self, g_name, ns=""):
        super(MoveitRokosPlanClass, self).__init__()

        self.ns_name, self.rd_name = self.get_namespace_func(ns)
        group_name = g_name
        self.robot_desc_name = str(self.rd_name + "robot_description")

        self.robot = moveit_commander.RobotCommander(self.robot_desc_name, self.ns_name)
        self.scene = moveit_commander.PlanningSceneInterface(self.ns_name)

        self.group = moveit_commander.MoveGroupCommander(group_name, self.robot_desc_name, self.ns_name)

        self.display_trajectory_publisher = rospy.Publisher((self.rd_name + 'move_group/display_planned_path'),
                                                      moveit_msgs.msg.DisplayTrajectory,
                                                      queue_size=20)

        #planning_frame = self.group.get_planning_frame()
        #eef_link = self.group.get_end_effector_link()

        current_state = self.robot.get_current_state()
        # başlangıç değerini loga yazmak için
        self.log_class = LogClass(group_name, current_state)

    @classmethod
    def get_namespace_func(cls, ns):
        ns_name = ""
        rd_name = ""

        if ns != "":
            ns_name = str(ns)
            rd_name = str(str(ns) + "/")

        return ns_name, rd_name


    def plan_execution_func(self, plan):
        """
            oluşturulan plan dosyalarini execute etmek için kullanilir. (.dat file, okur ve execute eder)
        """
        try:
            group = self.group

            self.log_class.main_func(plan)
            success = group.execute(plan, wait=True)

            return success

        except Exception as err:
            print(err)


    def go_to_pose_goal_func(self, pose_goal, use_only_pose=True, tolerance=0.005):
        """
            Ana hareket fonksiyonu
            Collision Free Trajectory Planning için kullanılmakta, Scene' e bir nesne aktarıldıysa 
            çarpışmasız hareket etmesi için kullanılır.
        """
        try:
            group = self.group

            # hedef positionu, pose_goal değerini planlayiciya pose olarak gönderir.
            group.set_pose_target(pose_goal)
            # Burada mevcut oluşturulan planin get_current_plan değerini almak için kullanilir.
            (get_plan_control, get_current_plan, get_planning_time, get_error_code) = group.plan()

            # belirtilen positionu execute eder
            plan_exec = group.go(wait=True)

            group.stop()
            group.clear_pose_targets()

            # Eger use_only_pose değeri True ise pose_goal sadece X,Y,Z position bilgilerini içerir.
            if use_only_pose:
                current_pose = self.group.get_current_pose().pose

            # Eger False ise pose_goal değeri X,Y,Z position bilgilerinin yanında, quaternion olarak X,Y,Z,W bilgilerinide içermektedir.
            else:
                current_pose = self.group.get_current_pose()

            return all_close(pose_goal, current_pose, tolerance), get_current_plan     # 0.01

        except Exception as err:
            print(err)


    def dynamic_go_to_pose_goal(self, position_list, use_only_pose=True):
        """
            go_to_pose_goal_func fonksiyonuna gönderilecek verileri düzenler.
            Fonksiyonun inputları, go_to_pose_goal_func fonksiyonuna düzenlenerek input olarak girer,
            return' u ise go_to_pose_goal_func fonksiyonun return'udur.
        """
        try:
            if use_only_pose:
                pose_goal = self.group.get_current_pose().pose

                # None değerleri current olarak bırakarak, gereksiz hareket işlemini engeller.
                # Ek planlama yapmamaktadır. 
                for index, point in enumerate(position_list):
                    if point != None:
                        if index == 0:
                            pose_goal.position.x = point
                        elif index == 1:
                            pose_goal.position.y = point
                        elif index == 2:
                            pose_goal.position.z = point

            else:
                pose_goal = self.group.get_current_pose()
                temp_position_list = position_list[:3]

                if position_list[3] != None and position_list[4] != None:
                    degrees_list = [position_list[3], position_list[4]]
                    q_x, q_y, q_z, q_w = self.degree_to_quaternion(degrees_list)
                    pose_goal.pose.orientation.x = q_x
                    pose_goal.pose.orientation.y = q_y
                    pose_goal.pose.orientation.z = q_z
                    pose_goal.pose.orientation.w = q_w
                    #print("\n\nQuaternion Result => X = {x}, Y = {y}, Z = {z}, W = {w}\n\n".format(x=q_x, y=q_y, z=q_z, w=q_w))

                for index, point in enumerate(temp_position_list):
                    if point != None:
                        if index == 0:
                            pose_goal.pose.position.x = point
                        elif index == 1:
                            pose_goal.pose.position.y = point
                        elif index == 2:
                            pose_goal.pose.position.z = point

            go_to_result, get_current_plan = self.go_to_pose_goal_func(pose_goal, use_only_pose)       # , use_only_pose=use_only_pose
            print("\nDynamic Go to Pose Result, {result}\n".format(result=go_to_result))

            return go_to_result, get_current_plan

        except Exception as err:
            print(err)


    def degree_to_quaternion(self, degrees_list):
        # Dereceyi radiana çevirerek bunları Roll, Pitch, Yaw formatına çevirir.
        # Daha sonra return olarak quaternion formatına çevirerek quaternion X,Y,Z,W ya çevirir.
        radians_list = self.convert_degrees_to_radians(degrees_list)
        roll = 0.0
        pitch = radians_list[1]
        yaw = radians_list[0]
        quaternion_result = quaternion_from_euler(roll, pitch, yaw)

        print("\n\nQuaternion Result = {}\n\n".format(quaternion_result))

        return quaternion_result


    @classmethod
    def convert_degrees_to_radians(cls, degrees_list):
        radians_list = list()

        for item in degrees_list:
            value = math.radians(item)
            radians_list.append(value)

        return radians_list


    def camera_move_joint_state_func(self, camera_position_list, tolerance=0.01, wait_value=True):
        # Kameraların hareketi için kullanılan fonksiyondur.
        # Jointlere değer vererek hareket işlemini gerçekleştirir.
        try:
            group = self.group

            joint_goal = copy.deepcopy(group.get_current_joint_values())
            # joint_goal' deki 4. ve 5. indisler kameranın jointlerini belirtir.
            joint_goal[4] = camera_position_list[0]
            joint_goal[5] = camera_position_list[1]

            group.go(joint_goal, wait=wait_value)
            group.stop()
            current_joints = group.get_current_joint_values()

            return all_close(joint_goal, current_joints, tolerance)

        except Exception as err:
            print(err)


    def cartesian_path_execution_func(self, position_list, list_type):
        """
            list_type = bool
                True = [ [x0, y0, z0], [x1, y1, z1], ...]
                False = [x, y, z]

            Orn
            X, Y = 0.2
            Z = Current

            True
            [0.2, None, None], [None, 0.2, None]

            False
            [0.2, 0.2, None]
        """
        try:
            group = self.group
            # Position şablonunu belli eder, içinde current değerler yer alır. X,Y,Z position içerir.
            wpose = group.get_current_pose().pose

            # Waypointleri yani görev uzayındaki koordinatları oluşturmaktadır.
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

            # Pozisyon listesi doluysa koordinatları uygun şekilde waypointlere dönüştürür.
            if position_list:
                # list type tekrardan kontrol edilir. Çoklu görev var ise
                if list_type:
                    # [ [x0, y0, z0], [x1, y1, z1], ...] listesinde waypoint [x0, y0, z0] değerini ifade eder.
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


    def add_box_func(self):
        # Scene' e kutu ekliyor.
        name = "Test_Box"

        pose = [-2.0, -2.0, 1.0]
        dimensions = [2.5, 2.5, 2.5]

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]

        p.pose.orientation.w = 1.0

        self.scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))


    def add_mesh_func(self):
        # Scene' e mesh ekliyor.
        file_name = "/home/hakan/otokar_ws/src/otokar_simulation/model/bus_skeleton_rviz/model/sase-rviz.stl"
        name = "Otokar_Sase"

        pose = [-1.86, -3.165, 2.3]
        dimensions = [0.5, 0.5, 0.5]        # scale

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]

        p.pose.orientation.w = 1.0

        self.scene.add_mesh(name, p, file_name, (dimensions[0], dimensions[1], dimensions[2]))


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