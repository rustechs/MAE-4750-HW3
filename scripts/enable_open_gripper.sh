#!/bin/bash

rosrun baxter_tools enable_robot.py -e
rosrun baxter_examples gripper_cuff_control.py
