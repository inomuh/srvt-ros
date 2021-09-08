#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy
import cv2
import os
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import time


class ImageSaver(object):
    """
    Camera rgb image saver
    """
    def __init__(self, g_name=""):
        self.group_name = str(g_name)
        self.current_color_image = None
        self.current_tof_image = None

        self.current_workspace = self.get_current_workspace()
        self.dir_name = str(self.current_workspace) + 'rokos_moveit/image_file/' + str(self.group_name)

        self.bridge = CvBridge()
        self.color_cam_sub = rospy.Subscriber((self.group_name + '/color_camera/image_raw'), Image, self.__color_cam_callback)
        self.tof_cam_sub = rospy.Subscriber((self.group_name + '/tof_camera/depth/image_raw'), Image, self.__tof_cam_callback)


    def __color_cam_callback(self, msg):
        self.current_color_image = msg


    def __tof_cam_callback(self, msg):
        self.current_tof_image = msg


    def color_image_saver_func(self, image_name):
        try:
            if self.current_color_image != None:
                temp_image = self.current_color_image
                self.__display(temp_image, "color_image", str(image_name))

                return True

            else:
                return False

        except Exception as err:
            print(err)


    def tof_image_saver_func(self, image_name):
        try:
            if self.current_tof_image != None:
                temp_image = self.current_tof_image
                self.__display(temp_image, "tof_image", str(image_name))

                return True

            else:
                return False

        except Exception as err:
            print(err)


    def __display(self, img_msg, img_type, img_name):
        try:
            current_time = self.datenow_func()

            if img_type == "tof_image":
                # TOF için type değiştirilebilir.
                img = self.bridge.imgmsg_to_cv2(img_msg, "32FC1") # passthrough
                new_dir = self.dir_name + "/tof_image"
                img_file_format = ".pgm"

            else:
                img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                new_dir = self.dir_name + "/color_image"
                img_file_format = ".jpg"

            image_name = str(img_name + "_" + self.group_name + "_" + img_type + "_" + current_time + str(img_file_format))
            cv2.imwrite(os.path.join(new_dir, image_name), img) # saving image

        except Exception as err:
            print(err)

    @classmethod
    def datenow_func(cls):
        now = datetime.now()
        dt_string = now.strftime("%Y_%m_%d_-_%H_%M_%S")

        return str(dt_string)

    @classmethod
    def get_current_workspace(cls):
        """
            Get Current Workspace Function
        """
        file_full_path = os.path.dirname(os.path.realpath(__file__))
        directory_name = sys.argv[0].split('/')[-3]
        workspace_name = file_full_path.split(str(directory_name))[0]

        return workspace_name
