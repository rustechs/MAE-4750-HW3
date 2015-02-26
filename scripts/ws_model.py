#!/usr/bin/env python

'''
    sim_master.py
    Anthony McNicoll 2015

    Maintains current state of the robot and blocks
    Accepts commands to the robot by implementing MoveRobot service
    Reports the state of the system at 1 Hz in State message
'''

import rospy, time
from hw1.msg import *
from hw1.srv import *

# Virtual object representing a block.
# It has the "isOn" and "isBelow" attributes as well as a "placeOn" method.
class Block():

    # Instantiate with no argument for the table (over nothing, under many)
    def __init__(self, isOn = None):
        self.isBelow = None
        if isOn is None:
            self.isOn = None
        else:
            self.placeOn(isOn)

    # If the target block is unoccupied or is the table, put block on it.
    # Returns boolean of success.
    def placeOn(self, block):
        if (block.isBelow is None) or (block.isOn is None):
            block.isBelow = self
            self.isOn = block
            return True
        else:
            rospy.logwarn('Cannot place block where one already is.')
        return False

    # Eye-friendly method for checking if this "block" is truly a table.
    def isTable(self):
        return (self.isOn is None)

# Virtual object representing a gripper.
# It has attributes onBlock, overBlock, isGripped, and methods moveTo, moveOver
class Gripper():

    # Initialize open and in midair
    def __init__(self):
        self.onBlock = None
        self.overBlock = None
        self.isGripped = False

    # Tries to move the gripper to a block targBlock. Returns bool of success.
    def moveTo(self, targBlock):
        if self.isGripped:
            rospy.logwarn('Cannot move to block while gripped.')
        elif targBlock.isTable():
            rospy.logwarn('Cannot move to table.')
        else:
            self.onBlock = targBlock
            self.overBlock = targBlock.isBelow
            return True
        return False

    # Tries to move the gripper over block targBlock.
    # If there is a block over the target block, this becomes a moveTo call.
    # If the gripper previously was closed on a block, this moves the block.
    def moveOver(self, targBlock):

        # Proceed if we're moving to an empty space.
        if (targBlock.isBelow is None) or (targBlock.isTable):
            self.overBlock = targBlock

            # If we were holding something, consider it moved.
            if (not (self.onBlock is None)) and self.isGripped:
                self.onBlock.isOn.isBelow = None
                return self.onBlock.placeOn(targBlock)

            # Otherwise, make sure we left whatever it was behind.
            else:
                self.onBlock = None
                return True

        # Otherwise, this function becomes a call to moveTo.
        else:
            return self.moveTo(targBlock.isBelow)
        return False

    def open(self):
        self.isGripped = False
        return True

    # Disallow gripping of blocks that are underneath other
    def close(self):
        if (self.onBlock is None) or (self.onBlock.isBelow is None):
            self.isGripped = True
            return True
        else:
            rospy.logwarn('Cannot grip locked blocks.')
        return False

# This is the overall "wrapper class" for the simulation node as a whole.
class Simulation():

    # Sets up the nodes for simulation upon construction.
    def __init__(self):

        # Define list of accepted configurations and robot commands
        self.configs = ['scattered', 'stacked_ascending', 'stacked_descending']
        self.actions= ['open', 'close', 'move_to', 'move_over']

        # Load number of blocks and initialize state from config parameters
        # This information is stored in a list of block objects
        self.nBlocks = rospy.get_param('num_blocks')
        initstr = rospy.get_param('configuration')
        self.initializeBlocks(initstr)

        # Instantiate the gripper object
        self.gripper = Gripper()

        # Declare that we'll be publishing to the state topic
        self.statePub = rospy.Publisher('state', State, queue_size = 10)

        # Declare that we'll be handling the MoveRobot service
        rospy.Service('MoveRobot', MoveRobot, self.handleMoveRobot)

        # Initializes the node (connects to roscore)
        # Setting anonymous to True guaruntees unique node name
        rospy.init_node('sim_node', anonymous = True)

    # This callback function is an implementation of MoveRobot.
    def handleMoveRobot(self, cmd):
        if not (cmd.action in self.actions):
            rospy.logwarn('Invalid command issued: ' + cmd.action)
        else:
            # Commands take time to perform
            time.sleep(.5)
            if cmd.target <= self.nBlocks:
                targetBlock = self.blockList[cmd.target]
                if cmd.action == 'open':
                    return MoveRobotResponse( self.gripper.open() )
                elif cmd.action == 'close':
                    return MoveRobotResponse( self.gripper.close() )
                elif cmd.action == 'move_to':
                    return MoveRobotResponse(self.gripper.moveTo(targetBlock))
                elif cmd.action == 'move_over':
                    return MoveRobotResponse(
                                            self.gripper.moveOver(targetBlock))
            else:
                rospy.logwarn('Invalid target block #: ' + str(cmd.target))
        return MoveRobotResponse(False)

    # This function is in charge of instantiating blocks depending on
    # configuration and organizing them into a list to be returned.
    def initializeBlocks(self, initstr):

        # Check input
        if initstr not in self.configs:
            rospy.logwarn('Invalid: %s. Defaulting to scattered.' % initstr)
            initstr = 'scattered'

        # Instantiate the "table block," as well on the rest on table
        table = Block(None)
        blocks = [ table ]
        blocks.extend( [Block(table) for i in range(self.nBlocks)] )

        # Make a list of what blocks come first, bottom to top
        if initstr == 'scattered':
            onTop = [0 for i in range(self.nBlocks)]
        elif initstr == 'stacked_ascending':
            onTop = range(self.nBlocks)
        elif initstr == 'stacked_descending':
            onTop = range(2, self.nBlocks + 1) + [0]

        # Stack them in the right order and return
        for b in range(1, self.nBlocks + 1):
            blocks[b].placeOn(blocks[onTop[b - 1]])
        self.blockList = blocks

    # Publish the current state to the 'state' topic at 1 Hz
    def publishLoop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            is_on = [ self.block2num(self.blockList[n].isOn)
                      for n in range (1, self.nBlocks + 1)  ]
            self.statePub.publish(is_on, self.gripper.isGripped)
            rate.sleep()

    # Given a block object, find its "ID number"
    def block2num(self, block):
        return self.blockList.index(block)

# This is what runs when the script is executed externally.
# It runs the main node function, and catches exceptions.
if __name__ == '__main__':
    try:
        simObject = Simulation()
        simObject.publishLoop()
    except:
        import pdb, traceback, sys
        type, value, tb = sys.exc_info()
        traceback.print_exc()
        pdb.post_mortem(tb)
