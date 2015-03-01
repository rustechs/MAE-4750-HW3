#!/usr/bin/env python

'''
    Robot Interface node for HW3.

     Query the position of the robot's arm and return it
    *Returns the configuration, and the cartesian pose of end-effector
    *Pick-and-place routine that accepts source and destination coordinates
'''

import argparse, sys, rospy, cv2, cv_bridge
import baxter_interface

import roslib
import tf2_ros

from bax_hw3.msg import *

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Image

# The baxter class definition
# Acts as a wrapper for many useful baxter_interface methods
# Also spawns a node to interface with IK Service
class Baxter():

    # Baxter class constructor
    def __init__(self, baxter_name="Baxter"):

        rospy.init_node("Baxter_Node")
        
        # Give him a creative name
        self.name = baxter_name

        # self.enable() # Probably should be called manually
        
        # Create baxter arm instances
        self.right_arm = baxter_interface.Limb('right')
        self.left_arm = baxter_interface.Limb('left')

        self.BaxEnable = baxter_interface.RobotEnable()

        # Create baxter gripper instances
        self.right_gripper = baxter_interface.Gripper('right')
        self.left_gripper = baxter_interface.Gripper('left')

        ############ DOES THIS ACTUALLY WORK ??? ############

        self.nsL = "ExternalTools/left/PositionKinematicsNode/IKService"
        self.nsR = "ExternalTools/right/PositionKinematicsNode/IKService"
        self.iksvcL = rospy.ServiceProxy(self.nsL, SolvePositionIK)
        self.iksvcR = rospy.ServiceProxy(self.nsR, SolvePositionIK)

         # Wait for services to exist
        rospy.wait_for_service(self.nsL)         
        rospy.wait_for_service(self.nsR) 

        # Set up publishing to the face
        self.facepub = rospy.Publisher('/robot/xdisplay', Image, latch=True)

        ######## tf trnaform ########
        #rospy.init_node('baxter_tf_broadcaster')
        self.br = tf2_ros.TransformBroadcaster()   #creat tf broadcaster object

        #rospy.init_node('tf_baxter')
        self.buf = tf2_ros.Buffer();
        tf2_ros.TransformListener(self.buf)    #creat tf listener object


        #####################################################

    # Tansformation from a local frame Pose to global frame
    def tfBaxter(self,localPose):
    # World frame is "base" for baxter
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.br.sendTransform(self.zeroPose.position,
                                self.zeroPose.orientation, #Zero point relative to "base"
                                rospy.Time.now(),
                                "basePose",   # Transfer to base frame
                                localPose)   # Transfer from

        try:
                (trans,rot) = self.buf.lookupTransform('/'+localPose, '/basePose')
                return trans

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    #The pose at calibration 0 point of our local working frame
    def zero(self):
        self.zeroPose = self.getEndPose('right')

    def face(self, fname):
        img = cv2.imread(fname + '.png')
        msg = cb_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.facepub.publish(msg)

    # Enable the robot
    # Must be manually called after instantiation 
    def enable(self):
        self.BaxEnable.enable()

    # Disable the robot
    def disable(self):
        self.BaxEnable.RobotEnable.disable()

    # Check if robot is enabled
    def isEnabled(self):
        return self.BaxEnable._state.enabled
        
    # Stop the robot
    # Equivalent to hitting e-stop
    def stop(self):
        self.BaxEnable.stop()

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
        try:
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
        # move_to_joint_positions(self, positions, False)
        # positions (dict({str:float})) - joint_name:angle command

        try:
            if limbSide == 'left':
                self.left_arm.move_to_joint_positions(angles)
            elif limbSide == 'right':
                self.right_arm.move_to_joint_positions(angles)
            else:
                raise
        except:
            rospy.logwarn('Invalid limb side name #: ' + limbSide)
            raise


    # Method for calculating joint angles given a desired end-effector pose
    def getIKGripper(self, limbSide, setPose):

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')    #??? 'what's their base frame???
        ps = PoseStamped(header=hdr, pose=setPose,)
        ikreq = SolvePositionIKRequest(ps, [], 0)

        try:

            if limbSide == 'left':
                resp = self.iksvcL(ikreq)     #??? What format/type is the service response?
            elif limbSide == 'right':
                resp = self.iksvcR(ikreq)
            else: 
                rospy.logwarn('Invalid limb side name #: ' + limbSide)
                raise
        except:
            rospy.logerr("IK Service call failed: %s" % sys.exc_info()[0])
            import pdb; pdb.set_trace()
            raise

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

        ik_joints = self.getIKGripper(limbSide, setPose)
        self.setJoints(self,limbSide,ik_joints)
