#!/usr/bin/env python

from robot_interface import RobotEnable, Gripper
import os

if __name__ == '__main__':
    enableObj = RobotEnable()
    enableObj.enable()

    lGrip = Gripper('left')
    rGrip = Gripper('right')

    lGrip.open()
    rGrip.open()

    os.system("~/ros_ws/baxter.sh; rosrun baxter_examples gripper_cuff_control.py -g both")
