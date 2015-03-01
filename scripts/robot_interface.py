#!/usr/bin/env python

'''
    Robot Interface node for HW3.

     Query the position of the robot's arm and return it
    *Returns the configuration, and the cartesian pose of end-effector
    *Pick-and-place routine that accepts source and destination coordinates
'''

import argparse 
import sys       

import rospy
import baxter_interface

from bax_hw3.msg import *

from geometry_msgs.msg import PoseStamped

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)



# The baxter class definition
# Acts as a wrapper for many useful baxter_interface methods
# Also spawns a node to interface with IK Service
class Baxter():

    # Baxter class constructor
    def __init__(self, baxter_name):

        # Give him a creative name
        self.name = baxter_name

        # self.enable() # Probably should be called manually
        
        # Create baxter arm instances
        self.right_arm = baxter_interface.Limb('right')
        self.left_arm = baxter_interface.Limb('left')

        # Create baxter gripper instances
        self.right_gripper = baxter_interface.Gripper('right')
        self.left_gripper = baxter_interface.Gripper('left')

        ############ DOES THIS ACTUALLY WORK ??? ############
        rospy.init_node("ik_service_client")

        self.nsL = "ExternalTools/left/PositionKinematicsNode/IKService"
        self.nsR = "ExternalTools/right/PositionKinematicsNode/IKService"
        self.iksvcL = rospy.ServiceProxy(self.nsL, SolvePositionIK)
        self.iksvcR = rospy.ServiceProxy(self.nsR, SolvePositionIK)

         # Wait for services to exist
        rospy.wait_for_service(self.nsL)         
        rospy.wait_for_service(self.nsR) 

        self.ikreq = SolvePositionIKRequest()
        #####################################################

    # Enable the robot
    # Must be manually called after instantiation 
    def enable(self):
        baxter_interface.RobotEnable.enable()

    # Disable the robot
    def disable(self):
        baxter_interface.RobotEnable.disable()

    # Check if robot is enabled
    def isEnabled(self)
        return baxter_interface.RobotEnable._state.enabled
        
    # Stop the robot
    # Equivalent to hitting e-stop
    def stop(self):
        baxter_interface.RobotEnable.stop()

    # Close specified gripper
    # Defaults to blocking
    def closeGrip(self, limbSide, block=True):
        try:
            if limbSide == 'left':
                self.left_gripper.close(block)
            elif limbSide == 'right':
                self.right_gripper.close(block)
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Open specified gripper
    # Defaults to blocking
    def openGrip(self, limbSide, block=True):
        try:
            if limbSide == 'left':
                self.left_gripper.open(block)
            elif limbSide == 'right':
                self.right_gripper.open(block)
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Set specified gripper's applied force
    # Specify force in % (0-100), mapping to 0-30 N
    def setGripForce(self, limbSide, force):
        try:
            if limbSide == 'left':
                self.left_gripper.set_moving_force(force)
                self.left_gripper.set_holding_force(force)
            elif limbSide == 'right':
                self.right_gripper.set_moving_force(force)
                self.right_gripper.set_holding_force(force)
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Check if specified gripper is ready
    # Returns true iff gripper is calibrated, not in error state, and not moving
    def getGripReady(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.ready()
            elif limbSide == 'right':
                return self.right_gripper.ready()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Check if specified gripper is gripping (i.e. force threshold reached)
    def getGripGripping(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.gripping()
            elif limbSide == 'right':
                return self.right_gripper.gripping()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Check if specified gripper missed object
    # (i.e. gripper closed without reaching force threshold)
    def getGripMissed(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.missed()
            elif limbSide == 'right':
                return self.right_gripper.missed()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise
    
    # Get specified gripper's current position
    # Returns as percent (0-100) of full travel range
    def getGripPos(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.position()
            elif limbSide == 'right':
                return self.right_gripper.position()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise

    # Get specified gripper's current applied force
    # Returns as percent (0-100) of max applicable force
    def getGripForce(self, limbSide):
        try:
            if limbSide == 'left':
                return self.left_gripper.force()
            elif limbSide == 'right':
                return self.right_gripper.force()
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side argument: ' + limbSide)
            raise
    
    # Method for getting joint configuration
    # Direct call to baxter_interface
    def getJoints(self, limbSide):
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
            rospy.logwarn('Invalid limb side name ' + limbSide)
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
            rospy.logwarn('Invalid limb side name #: ' + limbSide)
            raise


    # Method for setting joint positions
    # Direct call to baxter_interface
    # set_left and set_right are dict({str:float}), same size as joint angles
    def setJoints(self,limbSide,angles):
        # set_joint_positions(self, positions, False)
        # positions (dict({str:float})) - joint_name:angle command
        # raw (bool) - advanced, direct position control mode

        try:
            if limbSide == 'left':
                self.left_arm.move_to_joint_positions(angles,False)
            elif limbSide == 'right':
                self.right_arm.move_to_joint_positions(angles,False)
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side name #: ' + limbSide)
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
                rospy.logwarn('Invalid limb side name #: ' + limbSide)
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
