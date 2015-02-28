#!/usr/bin/env python

'''
    Robot Interface node for HW3.

     Query the position of the robot's arm and return it
    *Returns the configuration, and the cartesian pose of end-effector
    *Pick-and-place routine that accepts source and destination coordinates
'''

import rospy
import baxter_interface
import tf2_ros
# import roslib
from bax_hw3.msg import *
# from bax_hw3.srv import *
# from waiter import Waiter


# The baxter class.
class Baxter():

    # Method for on-lining baxter
    # All required calls from baxter_interface
    def __init__(self):
        self.start #???

        baxter_interface.RobotEnable() #??? This is the API responsible for enabling/disabling the robot, as well as running version verification

        right_arm = baxter_interface.Limb('right')
        left_arm = baxter_interface.Limb('left')

        right_gripper = baxter_interface.Gripper('right')
        left_gripper = baxter_interface.Gripper('left')

        right_gripper_pose = Pose( Point(0,0,3), Quaternion(0,1,0, pi/2) )    # our "Pose" message 
        left_gripper_pose = Pose( Point(0,0,3), Quaternion(0,1,0, pi/2) )


    # Method for getting joint configuration
    # Direct call to baxter_interface
    def getJoints(self,limbSide):
        # Returns: dict({str:float})
        # unordered dict of joint name Keys to angle (rad) Values

        if limbSide == 'left':
            self.leftAngles = left_arm.joint_angles()
            return self.leftAngles       #??? return the joints dictionary right?
        elif limbSide == 'right':
            self.rightAngles = right_arm.joint_angles()
            return self.rightAngles
        else:
            rospy.logwarn('Invalid limb side name #: ' + str(limbSide))


    # Method for getting end-effector position
    # Uses forward kinematics ROS library, TF
    # Angular pose will always be top-down, so wrist-gripper displacement doesn't have to be factored in
    def getEndPose(self,limbSide):
        # left_arm.endpoint_pose(): pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}

        if limbSide == 'left':
            right_gripper_pose = left_arm.endpoint_pose()    #??? return to pose message Right?
        elif limbSide == 'right':
            left_gripper_pose = right_arm.endpoint_pose()
        else:
            rospy.logwarn('Invalid limb side name #: ' + str(limbSide))



    # Method for setting joint positions
    # Direct call to baxter_interface
    # set_left and set_right are dict({str:float}), same size as joint angles
    def setJoints(self,limbSide,angles):
        # set_joint_positions(self, positions, raw=False)
        # positions (dict({str:float})) - joint_name:angle command
        # raw (bool) - advanced, direct position control mode

        if limbSide == 'left':
            left_arm.move_to_joint_positions(angles,raw=False)
        elif limbSide == 'right':
            right_arm.move_to_joint_positions(angles,raw=False)
        else:
            rospy.logwarn('Invalid limb side name #: ' + str(limbSide))



    # Method for setting cartesian position of hand
    # Uses some sort of external IK engine - dunno where
    # Would be great if it could be made non-blocking for bimanual operation
    def setEndPose(self,poseL,poseR):

        self.setEndPose('left') = left_arm.
        self.setEndPose('right') = right_arm.
