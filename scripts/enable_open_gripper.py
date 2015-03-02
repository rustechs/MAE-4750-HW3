#!/usr/bin/env python

import robot_interface import RobotEnable, Gripper
import os

if __name__ == '__main__':
    enableObj = RobotEnable()
    enableObj.

    lGrip = Gripper('left')
    rGrip = Gripper('right')

    lGrip.open()
    rGrip.open()
    os.system("rosrun baxter_examples gripper_cuff_control.py -g both")
