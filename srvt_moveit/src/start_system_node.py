#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""start_system_node"""
import os
import time
import rospy

def main_func():
    """main func"""
    rospy.init_node('start_system_node', anonymous=True)
    os.system("gnome-terminal -x roslaunch srvt_gazebo start_otokar.launch")

    time.sleep(10)
    os.system("gnome-terminal -x roslaunch srvt_moveit start_rokos.launch")


if __name__ == '__main__':
    main_func()
