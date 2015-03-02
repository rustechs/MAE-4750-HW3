# MAE-4750-HW3
Homework 3 for MAE 4750: Baxter Plays with Blocks.

This package supports both unimanual and bimanual stacking of blocks.

It is tested functional with stacks up to 5 blocks tall.

To run, first make your stack of blocks in the middle of the table. The current launchfile assumes stacked_descending as the initial configuration, but you are free to alter this as desired.

Then, open a terminal window, connect to Baxter by cd'ing to ros_ws and running 

. baxter.sh

and enable Baxter/open grippers by running

rosrun bax_hw3 enable_open_gripper.py

Finally, you can launch the stacking controller by running

roslanch bax_hw3 hw.launch

and sending a stack command (either 'stacked_ascending' or 'stacked_descending') to the node by running a terminal command of the form

rostopic pub /commandme bax_hw3/Command 'stacked_ascending' -1