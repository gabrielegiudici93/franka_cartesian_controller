#!/usr/bin/env python3
import pyfranka_interface as franka
import math
import numpy as np
import time
from trajectory import SinusoidalTraj, Trajectory


dim_joints = 7
init_pos_joint = [0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, np.pi/4]
y_center = init_pos_joint
tzero_y = np.zeros((dim_joints,)) 
frequency_vec = 2*np.ones((dim_joints,))
amplitude_vec = np.array([0.15,0.,0.0,0.0,0.0,0.0,0.0])
des_traj = SinusoidalTraj.InitTrajectoryVector(dim_joints,y_center, frequency_vec, tzero_y, amplitude_vec)

r = franka.Robot_('192.168.2.10',False)

cur_perfomance = franka.CtrlPerformance()

# move the robot to initial position
r.init()
cur_state = r.getState()
print("init_state")
print(cur_state.T)
time.sleep(1)


p_diag = np.array([800, 500, 500, 700, 800, 900, 600])
d_diag = np.array([50, 20, 20, 20, 20, 50, 50])

r.setFLController(p_diag, d_diag)
r.runDetach()


timer = 0
total_duration = 20
iteration_start = 0
desired_frequency = 1000
while timer < total_duration:
 # Capture the time at the beginning of this iteration
    iteration_start = time.time()
    
    # Your code goes here
    # init position 
    # test 7 joint regulation
    #q_des = np.array([0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, 0.0])
    #qd_des = np.array([0,0,0,0,0,0,0])
    # test tracking sinusoidal trajectory
    q_des,qd_des,qdd_des = SinusoidalTraj.GetDesiredTrajectory(des_traj,timer)
    # debug
    #print(q_des)
    # Simulate some work by sleeping for a random amount of time
    r.updateDesiredValues(q_des, qd_des)
    # get last perfomances from the controller 
    cur_perfomance = r.getCtrlPerformance()

    # DEBUG
    #print("q_des=",cur_perfomance.q_des)
    #print("q_err=",cur_perfomance.q_err)
    #print("qd_err=",cur_perfomance.qd_err)
    #print("last_tau =",cur_perfomance.last_tau)

    # Calculate the actual time elapsed during this iteration
    elapsed_time = (time.time() - iteration_start)
    
    # Example: just print the current timer value
    #print(f"timer: {timer} seconds")

    # Update the timer with the elapsed time
    if elapsed_time < 0.001:
        timer += 0.001
    else:
        timer += elapsed_time

    sleep_time = 0.001 - elapsed_time
    if sleep_time > 0:
        time.sleep(sleep_time)

# call join on the control thread
r.stopFLController()
r.closeControlThread()
print("end of test_FL_franka_interface.py")



