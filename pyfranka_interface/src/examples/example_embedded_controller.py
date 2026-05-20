import pyfranka_interface as franka
import math
import numpy as np
import threading
import time 
from trajectory import SinusoidalTraj, Trajectory

# ATTENTION THIS WAY OF ACCESS THE CONTROL INTERFACE IS EASILY AFFECTED BY TIME DELAY HENCE IT CAN BE USED BUT WITH EXTREME CAUTION

dim_joints = 7
init_pos_joint = [0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, np.pi/4]
y_center = init_pos_joint
tzero_y = np.zeros((dim_joints,)) 
frequency_vec = 0.1*np.ones((dim_joints,))
amplitude_vec = np.array([0.1,0.0,0.0,0.0,0.0,0.0,0.0])
des_traj = SinusoidalTraj.InitTrajectoryVector(dim_joints,y_center, frequency_vec, tzero_y, amplitude_vec)
# proportional and derivative gains for PID
P = np.diag([100, 100, 100, 100, 100, 100, 100])
D = np.diag([10, 10, 10, 10, 10, 10, 10])

class PrintData:
    def __init__(self):
        self.lock = threading.Lock()
        self.gravity = np.zeros(7)
        # Assuming these are also numpy arrays
        self.mass_matrix = np.zeros((7, 7))
        self.coriolis = np.zeros(7)

    def update_data(self, gravity, mass_matrix, coriolis):
        with self.lock:
            self.has_data = True
            self.gravity = np.array(gravity)
            self.mass_matrix = np.array(mass_matrix)
            self.coriolis = np.array(coriolis)

    def print_data(self):
        with self.lock:
            print("Gravity Vector:", self.gravity)
            print("Mass Matrix:\n", self.mass_matrix)
            print("Coriolis Forces:", self.coriolis)
            print("\n")

running = True
print_data_flag = False
def print_data_thread(print_data_object):
    while running:
        start_time = time.time()  
        print_data_object.print_data()
        elapsed_time = time.time() - start_time
        sleep_time = max(0.001 - elapsed_time, 0)  # Ensure sleep_time is not negative
        time.sleep(sleep_time)



# Create a robot object
r = franka.Robot_('172.16.0.2',False)

### Set the control interface to switch between different control modes
controlInterface = "joint_pos"
print_data_obj = PrintData()

def ctrl_func_joint_pos(state, init_state, cur_time):
    
    ret = franka.ReturnControlCommand()
    #print("cur_time = ", cur_time)
    delta_angle = np.pi / 8.0 * (1 - np.cos(np.pi / 2.5 * cur_time))
    delta_vec = [0, 0, 0, delta_angle, delta_angle, 0, delta_angle]
    delta_vec = np.array(delta_vec)
    output = init_state.q + delta_vec
    if cur_time >= 5.0:
        ret.running_controller = False
    else:
        ret.running_controller = True
    # testing the feedback linearization control without commanding any torques
    # update data object
    #print(r.getGravityVector())
    #grav_vec = r.getGravityVector()
    #M= r.getMassMatrix()
    #coriolis_vec = r.getCoriolisVector()
    #q_des,qd_des,qdd_des = SinusoidalTraj.GetDesiredTrajectory(des_traj,cur_time)
    #u = P @ (q_des - state.q) + D @ (qd_des - state.q_vel)
    #n = coriolis_vec + grav_vec
    
    #tau_FL = M @ u + n
    #print("cur_q_des", q_des)
    #print("time = ", cur_time)
    #print(tau_FL)

    if print_data_flag:
        print_data_obj.update_data(r.getGravityVector(), r.getMassMatrix(), r.getCoriolisVector())
    ret.joint_space_command = output
    return ret


def ctrl_func_joint_vel(state, init_state, cur_time):
    ret = franka.ReturnControlCommand()
    time_max = 1.0
    omega_max = 1.0
    cycle = math.floor(pow(-1.0, (cur_time - math.fmod(cur_time, time_max)) / time_max))
    omega = cycle * omega_max / 2.0 * (1.0 - np.cos(2.0 * np.pi / time_max * cur_time))
    velocities = [0.0, 0.0, 0.0, omega, omega, omega, omega]
    velocities = np.array(velocities)
    if cur_time >= 2 * time_max:
        ret.running_controller = False
    else:
        ret.running_controller = True
    ret.joint_space_command = velocities
    return ret


def ctrl_func_cartesian_pose(state, init_state, cur_time):
    ret = franka.ReturnControlCommand()
    transform = init_state.T
    kRadius = 0.3
    angle = np.pi / 4 * (1 - np.cos(np.pi / 5.0 * cur_time))
    delta_x = kRadius * np.sin(angle)
    delta_z = kRadius * (np.cos(angle) - 1)

    transform[0, 3] = transform[0, 3] + delta_x
    transform[2, 3] = transform[2, 3] + delta_z

    if cur_time >= 10.0:
        ret.running_controller = False
    else:
        ret.running_controller = True
    ret.cartesian_command = transform
    return ret

def ctrl_func_torque(state, init_state, cur_time):
    ret = franka.ReturnControlCommand()

    q_des,qd_des,qdd_des = SinusoidalTraj.GetDesiredTrajectory(des_traj,cur_time)
    grav_vec = r.getGravityVector()
    M= r.getMassMatrix()
    coriolis_vec = r.getCoriolisVector()

    u = P @ (q_des - state.q) + D @ (qd_des - state.q_vel)
    n = coriolis_vec + grav_vec
    
    tau_FL = M @ u + n

    if cur_time >= 10.0:
        ret.running_controller = False
    else:
        ret.running_controller = True
    
    ret.joint_space_command = tau_FL

    return ret


if __name__ == "__main__":
    # You can use these values directly, for example, to set update flags
    update_coriolis = franka.UpdateFlags.UpdateCoriolis
    update_gravity = franka.UpdateFlags.UpdateGravity
    update_mass = franka.UpdateFlags.UpdateMass
    # with this function i ask the controller to compute the gravity mass and coriolis
    r.setUpdateFlags(update_coriolis.value | update_gravity.value | update_mass.value)
    # bring the robot to the initial position
    r.init()
    # sleep for 2 seconds
    time.sleep(1)
    if print_data_flag:
        print_data_thread = threading.Thread(target=print_data_thread, args=(print_data_obj,))
        print_data_thread.start()
    if controlInterface == "joint_pos":
        r.setController(ctrl_func_joint_pos, "joint_pos")
        r.run()
    elif controlInterface == "joint_vel":
        r.setController(ctrl_func_joint_vel, "joint_vel")
        r.run()
    elif controlInterface == "cart_pos":
        r.setController(ctrl_func_cartesian_pose, "cartesian_pose")
        r.run()
    elif controlInterface == "torque":
         r.setController(ctrl_func_torque, "torque")

    if print_data_flag:
        running = False
        print_data_thread.join()
    print("Done")