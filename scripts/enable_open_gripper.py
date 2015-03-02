#!/usr/bin/env python

from baxter_interface import RobotEnable, Gripper
import rospy

if __name__ == '__main__':
    rospy.init_node('Prepper')

    enableObj = RobotEnable()
    enableObj.enable()

    lGrip = Gripper('left')
    rGrip = Gripper('right')

    lGrip.open()
    rGrip.open()