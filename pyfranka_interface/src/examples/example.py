#!/usr/bin/env python3
import pyfranka_interface as franka
import math
import numpy as np

r = franka.Robot_('192.168.2.10',False)
#r.translate([0, 0, -0.1], 2)
#r.rotate([math.pi / 4, 0, 0])

des_pos_mat = np.array([[ 0.54279235,  0.52770503, -0.65337036,  0.33259518],
 						 [ 0.35479985, -0.84919205, -0.39111844,  0.08303922],
                         [-0.76123208, -0.01951961, -0.64817709,  0.29786793],
                         [ 0.,          0.,          0.,          1.        ]])


# move the robot to initial position
#r.init()
cur_state = r.getState()
print("init_state")
print(cur_state.q)
print(cur_state.T)

#r.move("relative",delta_transform)
#r.move("absolute",des_pos_mat)
#get the intermediate motion and execute them
#cur_state = r.getState()
#print("final state")
#print(cur_state.T)


#gripper test
grip_state = r.getgripState()
print("grip_state before closing/opening=",grip_state.isgrasped," ",grip_state.width," ",grip_state.max_width)
grasping_width=0.07   # 0.07 open 0.03 close
velocity      =0.1
force         = 0.5
r.grasp(grasping_width,velocity,force)
grip_state = r.getgripState()
print("grip_state after closing/opening=",grip_state.isgrasped," ",grip_state.width," ",grip_state.max_width)
