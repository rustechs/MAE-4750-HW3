#!/usr/bin/env python

'''
    Controller node for HW3.

    Listens to a ROS topic /command of type String to command the controller,
    where commands are 'scatter', 'stack_ascending', 'stack_descending'

    Implements an internal model of the system (block set) in order to plan
    moves and coordinate limb motion.
'''

import rospy, copy
from bax_hw3.msg import *
from bax_hw3.srv import *
from geometry_msgs.msg import Point, Quaternion, Pose
from waiter import Waiter

# A simple Block class for legibility.
class Block():
    def __init__(self, number, x = 0, y = 0, z = 0):
        self.pos = Point(x, y, z)
        self.n = number

# A class representing an allowed "slot" for a block in the workspace.
class Slot():
    def __init__(self, x, y, z):
        self.pos = Point(x, y, z)
        self.contains = None

    # Change a Block's coordinates to match this slot's
    def eat(self, block):
        block.pos = self.pos
        self.contains = block

    # "Lets go" of a block
    def vomit(self):
        self.contains = None

# The controller class.
class Controller():

    # Initializes the controller as a ROS node which recieves commands.
    def __init__(self):

        # Load number of blocks, configuration, bimanual setting
        h = .044
        d = .08
        self.nBlocks = rospy.get_param('num_blocks')
        initstr = rospy.get_param('configuration')
        self.bimanual = (1 == rospy.get_param('bimanual'))

        # Initialize the block set and stack list together.
        # The stack list is a list of (occupied) slots.
        y = [.044*i for i in range(self.nBlocks)]
        if initstr == 'stacked_descending':
            y.reverse()
        self.blocks = [Block(i) for i in range(self.nBlocks)]
        self.stack = [Slot(0, 0, y[i]) for i in range(self.nBlocks)]
        for i, slot in enumerate(self.stack):
            slot.eat(self.blocks[i])

        # Initialize valid table slots
        #   If unimanual, add slots extending left of the stack.
        #   If bimanual, add slots to left for even indices, right for odd
        if self.bimanual:
            self.slots = []
            for i in range(self.nBlocks):
                s = -1 + 2*(i%2)
                m = 1 + i/2
                self.slots.append(Slot(s*d*m, 0, 0))
        else:
            self.slots = [Slot(-(i+1)*d, 0, 0) for i in range(self.nBlocks)]

        # Initialize a collision prevention variable called "hasStack", which
        # at all times should be maintained with either 'left', 'right', or
        # None values to indicate who has control of the stack.
        self.hasStack = 'left'

        # TODO initialize the Baxter object and home it.

        # Initialize the objective by sending a message to self
        self.handleCommand( Command(initstr) )

        # This node subscribes to the "command" topic.
        rospy.Subscriber("command", Command, self.handleCommand)

        # Initializes the node (connects to roscore)
        # Setting anonymous to True guaruntees unique node name
        rospy.init_node('controller_hw3', anonymous = True)

    # This is the callback for processing incoming command messages.
    def handleCommand(self, cmd):

        # Get the command and create the objective.
        # The objective is encoded as a numerical array of block numbers in
        # the stack, from bottom to top.
        initstr = cmd.command
        self.objective = range(self.nBlocks)
        if initstr == 'stacked_descending':
            self.objective.reverse()
        rospy.loginfo('Objective has changed to ' + initstr)

    # Function that takes action depending on the state of the workspace.
    # All the secret sauce is in here.
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
