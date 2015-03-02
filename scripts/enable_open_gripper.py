#!/usr/bin/env python

from baxter_interface import RobotEnable, Gripper
import rospy
import os

if __name__ == '__main__':
    rospy.init_node('Prepper')

    enableObj = RobotEnable()
    enableObj.enable()

    lGrip = Gripper('left')
    rGrip = Gripper('right')

    lGrip.calibrate()
    rGrip.calibrate()
    lGrip.open()
    rGrip.open()

    os.system("~/catkin_ws/baxter.sh; rosrun baxter_examples gripper_cuff_control.py -g both")
