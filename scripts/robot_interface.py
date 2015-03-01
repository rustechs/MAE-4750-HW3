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



# The baxter class.
class Baxter():

    # Method for on-lining baxter
    # All required calls from baxter_interface
    def __init__(self, baxter_name):

        self.name = baxter_name
        self.enable()
        
        #Arm Objects
        self.right_arm = baxter_interface.Limb('right')
        self.left_arm = baxter_interface.Limb('left')

        #Gripper Objects
        self.right_gripper = baxter_interface.Gripper('right')
        self.left_gripper = baxter_interface.Gripper('left')


        rospy.init_node("gripper_ik_service_client")

        self.nsL = "ExternalTools/left/PositionKinematicsNode/IKService"
        self.nsR = "ExternalTools/right/PositionKinematicsNode/IKService"
        self.iksvcL = rospy.ServiceProxy(self.nsL, SolvePositionIK)
        self.iksvcR = rospy.ServiceProxy(self.nsR, SolvePositionIK)

        rospy.wait_for_service(self.nsL)  #Wait for services to be exist
        rospy.wait_for_service(self.nsR)

        self.ikreq = SolvePositionIKRequest()


    def enable(self):
        baxter_interface.RobotEnable() #??? This is the API responsible for enabling/disabling the robot, as well as running version verification


    # #Set up the All parameters for Right Gripper Pose to 0
    # def zero(self):
    #     initialFrame = self.getEndPose("right")

    #     # #Initialize Gripper Pose
    #     # self.setEndPose(self, "right", Pose( Point(0,0,0), Quaternion(0,0,0,0)) #???


    # Method for getting joint configuration
    # Direct call to baxter_interface
    def getJoints(self,limbSide):
        # Returns: dict({str:float})
        # unordered dict of joint name Keys to angle (rad) Values
        try
            if limbSide == 'left':
                return self.left_arm.joint_angles()
            elif limbSide == 'right':
                return self.right_arm.joint_angles()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side name #: ' + str(limbSide))
            raise


    # Method for getting end-effector position
    # Uses forward kinematics ROS library, TF
    # Angular pose will always be top-down, so wrist-gripper displacement doesn't have to be factored in
    def getEndPose(self,limbSide):
        # left_arm.endpoint_pose(): pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}
        try:
            if limbSide == 'left':
                return self.left_arm.endpoint_pose() 
            elif limbSide == 'right':
                return self.right_arm.endpoint_pose()
            else: 
                raise
        except:
            rospy.logwarn('Invalid limb side name #: ' + str(limbSide))
            raise


    # Method for setting joint positions
    # Direct call to baxter_interface
    # set_left and set_right are dict({str:float}), same size as joint angles
    def setJoints(self,limbSide,angles):
        # set_joint_positions(self, positions, raw=False)
        # positions (dict({str:float})) - joint_name:angle command
        # raw (bool) - advanced, direct position control mode

        try:
            if limbSide == 'left':
                self.left_arm.move_to_joint_positions(angles,raw=False)
            elif limbSide == 'right':
                self.right_arm.move_to_joint_positions(angles,raw=False)
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side name #: ' + str(limbSide))
            raise


    # Method for calculating joint angles given a desired end-effector pose
    def getIKGripper(self, limbSide, setPose):
        # self.ik_gripper(string, "Pose" msg type)

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')    #??? 'what's their base frame???
        PoseStamped(header=hdr,pose=setPose,)
        self.ikreq.pose_stamp.append(PoseStamped)

        try:

            if limbSide == 'left':
                resp = self.iksvcL(self.ikreq)     #??? What format/type is the service response?
            elif limbSide == 'right':
                resp = self.iksvcR(self.ikreq)
            else: 
                rospy.logwarn('Invalid limb side name #: ' + str(limbSide))
        except:
            rospy.logerr("IK Service call failed: %s" % (e,))

        # try:
        #     rospy.wait_for_service(ns, 5.0)
        #     resp = iksvc(ikreq)     #??? What format/type ?

        # except (rospy.ServiceException, rospy.ROSException), e:
        #     rospy.logerr("IK Service call failed: %s" % (e,))

        if (resp.isValid[0]):
            print("IK service: SUCCESS - Valid Joint Solution Found for limb-"+str(limbSide)+" :")
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position)) #??? joints dictionary
            print limb_joints
            return limb_joints
        else:
            print("IK service: INVALID POSE - No Valid Joint Solution Found.")


    # Method for setting cartesian position of hand
    # Uses some sort of external IK engine - dunno where
    # Would be great if it could be made non-blocking for bimanual operation
    def setEndPose(self, limbSide, setPose):
        #setPose = {'position': (x, y, z), 'orientation': (x, y, z, w)}

        ik_joints = self.getIKGripper(self, limbSide, setPose)
        self.setJoints(self,limbSide,ik_joints)
