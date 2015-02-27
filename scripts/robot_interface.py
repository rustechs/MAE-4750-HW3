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
        self.start #What does start do???

        # How do we distinguish left hand and right hand ???
        gripper = Pose( Point(0,0,3), Quaternion(0,1,0, pi/2) )

        right_arm = baxter_interface.Limb('right')
        left_arm = baxter_interface.Limb('left')


    # Method for getting joint configuration
    # Direct call to baxter_interface
    def getJoints(self,limbSide):
        # Returns: dict({str:float})
        # unordered dict of joint name Keys to angle (rad) Values

        if limbSide == 'left':
            self.leftCongif = left_arm.joint_angles()
            return self.leftCongif
        elif limbSide == 'right':
            self.rightCongif = right_arm.joint_angles()
            return self.rightCongif
        else:
            rospy.logwarn('Invalid limb side name #: ' + str(limbSide))


    # Method for getting end-effector position
    # Uses forward kinematics ROS library, TF
    # Angular pose will always be top-down, so wrist-gripper displacement doesn't have to be factored in
    def getEndPose(self,limbSide):
        # pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}

        if limbSide == 'left':
            self.leftEndPose = left_arm.endpoint_pose()
            return self.leftEndPose
        elif limbSide == 'right':
            self.rightEndPose = right_arm.endpoint_pose()
            return self.rightEndPose
        else:
            rospy.logwarn('Invalid limb side name #: ' + str(limbSide))



    # Method for setting joint positions
    # Direct call to baxter_interface
    # set_left and set_right are dict({str:float}), same size as joint angles
    def setJoints(self,limbSide,set_positions):
        # set_joint_positions(self, positions, raw=False)
        # positions (dict({str:float})) - joint_name:angle command
        # raw (bool) - advanced, direct position control mode

        if limbSide == 'left':
            left_arm.move_to_joint_positions(set_positions,raw=False)
        elif limbSide == 'right':
            right_arm.move_to_joint_positions(set_positions,raw=False)
        else:
            rospy.logwarn('Invalid limb side name #: ' + str(limbSide))


    # Method for setting cartesian position of hand
    # Uses some sort of external IK engine - dunno where
    # Would be great if it could be made non-blocking for bimanual operation
    def setEndPose(self,poseL,poseR):

        self.setEndPose('left') = left_arm.
        self.setEndPose('right') = right_arm.








# Or Class robot_interface ???
def robot_interface():

    

    # Declare that we'll be publishing to the state topic
    self.statePub = rospy.Publisher('state', State, queue_size = 10)
    self.statePub = rospy.Publisher('state', State, queue_size = 10)


    # Declare that we'll be handling the MoveRobot service
    rospy.Service('pick_place', PickPlace, self.handlePickPlace)


    # Initializes the node (connects to roscore)
    # Setting anonymous to True guaruntees unique node name
    rospy.init_node('robot_interface_node', anonymous = True)




    # This callback function is an implementation of Pick and Place
    def handlePickPlace(source, destination):
        

# This is what runs when the script is executed externally.
# It runs the main node function, and catches exceptions.
if __name__ == '__main__':
    try:
        robot_interface()
    except rospy.ROSInterruptException:
        pass
