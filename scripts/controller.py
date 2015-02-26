#!/usr/bin/env python

'''
    Controller node for HW1.

    Listens to a ROS topic /command of type String to command the controller,
    where commands are 'scatter', 'stack_ascending', 'stack_descending'

    It in turn sends commands to the MoveRobot service to get things done.
'''

import rospy
from hw1.msg import *
from hw1.srv import *
from waiter import Waiter

# The controller class.
class Controller():

    # This is the main controller function. After initializing its subscriber
    # status and setting up a service proxy, it continually works until the
    # specified objective is achieved.
    def __init__(self):

        # Load the number of blocks and default configuration
        self.nBlocks = rospy.get_param('num_blocks')
        initstr = rospy.get_param('configuration')

        # Initialize the objective by sending a message to self
        self.handleCommand( Command(initstr) )

        # This node subscribes to the "command" and "state" topics.
        # Use a MsgBuffer to queue up the state messages.
        rospy.Subscriber("command", Command, self.handleCommand)
        self.stateWaiter = Waiter()
        rospy.Subscriber("state", State, self.stateWaiter.cb)

        # Set up the service proxy for sending commands to robot
        self.moveRobot = rospy.ServiceProxy('MoveRobot', MoveRobot)

        # Initializes the node (connects to roscore)
        # Setting anonymous to True guaruntees unique node name
        rospy.init_node('controller_node', anonymous = True)

    # This is the callback for processing incoming command messages.
    def handleCommand(self, cmd):

        # Get the command and create the objective
        # This has code similar to sim_master initialization code
        initstr = cmd.command
        if initstr == 'scattered':
            self.objective = [0 for i in range(self.nBlocks)]
        elif initstr == 'stacked_ascending':
            self.objective = range(self.nBlocks)
        elif initstr == 'stacked_descending':
            self.objective = range(2, self.nBlocks + 1) + [0]
        rospy.loginfo('Objective has changed to ' + initstr)

    # Function that takes action depending on the state of the workspace
    def planMove(self):

        # Work with a fresh state.
        # State vector: state[n] returns the block that block n+1 is on.
        state = self.stateWaiter.fresh()

        # If the state matches the objective, express victory
        if list(state.is_on) == self.objective:
            rospy.loginfo('Objective currently met.')
            return
        stateArr = state.is_on

        # Otherwise, start working.

        # Start by climbing up the stack until either a bad block or no block
        # is found. If there's a bad block, take off the block on top of it.
        bad = False
        thisBlock = self.isBelow(0, self.objective)
        while not bad:
            thisOK = self.isInPosition(thisBlock, stateArr)
            if thisOK:
                thisBlock = self.isBelow(thisBlock, self.objective)
            else:
                targBlock = thisBlock
                targOn = self.isAbove(thisBlock, self.objective)
                bad = True

        self.moveRobot('open',0)
        self.moveRobot('move_to', targBlock)
        self.moveRobot('close',0)
        self.moveRobot('move_over', targOn)
        self.moveRobot('open',0)

    def chainUp(self, block, stateArr):
        above = self.isBelow(block, stateArr)
        if above is None:
            return block
        else:
            return self.chainUp(above, stateArr)

    # Returns the block above block
    def isBelow(self, block, stateArr):
        try:
            out = stateArr.index(block) + 1
        except ValueError:
            out = None
        return out

    # Returns the block below block
    def isAbove(self, block, stateArr):
        return stateArr[block - 1]

    def isInPosition(self, block, stateArr):
        return stateArr[block - 1] is self.objective[block - 1]

# This is what runs when the script is executed externally.
# It runs the main node function, and catches exceptions.
if __name__ == '__main__':
    try:
        cObject = Controller()
        while True:
            cObject.planMove();
    except:
        import pdb, traceback, sys
        type, value, tb = sys.exc_info()
        traceback.print_exc()
        pdb.post_mortem(tb)
