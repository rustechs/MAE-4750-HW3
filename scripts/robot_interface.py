#!/usr/bin/env python

'''
    Robot Interface node for HW3.

     Query the position of the robot's arm and return it
    *Returns the configuration, and the cartesian pose of end-effector
    *Pick-and-place routine that accepts source and destination coordinates
'''

import argparse  #IK service?
import sys       #IK serivce?

import rospy
import baxter_interface

from bax_hw3.msg import *

from geometry_msgs.msg import PoseStamped

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


# from bax_hw3.srv import *
# from waiter import Waiter


# The baxter class.
class Baxter():

    # Method for on-lining baxter
    # All required calls from baxter_interface
    def __init__(self, baxter_name):

        self.name = baxter_name
        self.enable()
        
        #Arm Objects
        right_arm = baxter_interface.Limb('right')
        left_arm = baxter_interface.Limb('left')

        #Gripper Objects
        right_gripper = baxter_interface.Gripper('right')
        left_gripper = baxter_interface.Gripper('left')

        # Creat Gripper Pose message
        right_gripper_pose = Pose( Point(1,1,3), Quaternion(0,1,0, pi/2) )   #!!! is Point(1,1,3) a good starting point?
        left_gripper_pose = Pose( Point(1,1,3), Quaternion(0,1,0, pi/2) )

        #Initialize Gripper Pose
        self.setEndPose(self, "right", right_gripper_pose)     #??? How do we avoid grippers hit each other?
        self.setEndPose(self, "left", left_gripper_pose)



    def enable(self):
        baxter_interface.RobotEnable() #??? This is the API responsible for enabling/disabling the robot, as well as running version verification


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


    def ik_gripper(self, limbSide, setPose):
        # self.ik_gripper(string, "Pose" msg type)

        rospy.init_node("gripper_ik_service_client")

        ns = "ExternalTools/" + limbSide + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        PoseStamped(header=hdr,pose=setPose,)  #??? "," after "pose=gripperPose"?
        ikreq.pose_stamp.append(PoseStamped)

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)     #??? What format/type ?

        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("IK Service call failed: %s" % (e,))


        if (resp.isValid[0]):
                print("IK service: SUCCESS - Valid Joint Solution Found:")
                # Format solution into Limb API-compatible dictionary
                limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position)) #???
                print limb_joints
                return limb_joints
            else:
                print("IK service: INVALID POSE - No Valid Joint Solution Found.")


    # Method for setting cartesian position of hand
    # Uses some sort of external IK engine - dunno where
    # Would be great if it could be made non-blocking for bimanual operation
    def setEndPose(self, limbSide, setPose):
        #setPose = {'position': (x, y, z), 'orientation': (x, y, z, w)}

        ik_joints = self.ik_gripper(self, limbSide, setPose)
        self.setJoints(self,limbSide,ik_joints)
