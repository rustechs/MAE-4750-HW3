#!/usr/bin/env python

'''
    Waiter class for keeping messages

    A useful class which keeps track of incoming messages, and can either
    immediately return the last message, or wait until a fresh one comes in.
'''

import time

# The Waiter class.
class Waiter():

    # Just initialize the buffer.
    def __init__(self):
        self.buffer = None

    # Returns whether something is available.
    def pending(self):
        return not (self.buffer is None)

    # Flush (delete contents of) the buffer.
    def flush(self):
        self.buffer = None

    # Returns the latest message, even if it is none
    def read(self, timeout = None):
        out = self.buffer
        self.flush()
        return out

    # Empties buffer, blocks for new message, returns it
    def fresh(self, timeout = None):
        self.flush()
        while self.pending() is False:
            pass
        return self.read()

    # Gets new messages and loads them up into the buffer
    def cb(self, msg):
        self.buffer = msg
