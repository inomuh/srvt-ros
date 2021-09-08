#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import time


def main_func():
    rospy.init_node('start_system_node', anonymous=True)
    os.system("gnome-terminal -x roslaunch otokar_gazebo start_otokar.launch")

    time.sleep(10)
    os.system("gnome-terminal -x roslaunch rokos_moveit start_otokar.launch")


if __name__ == '__main__':
    main_func()