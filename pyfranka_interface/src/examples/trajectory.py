
import casadi as ca
import numpy as np
from abc import ABC, abstractmethod
import pickle
import matplotlib.pyplot as plt


# here i can create different trajectory classes
# create here a simple interface class for the trajectory (monodimensional trajectory)
class Trajectory(ABC):
  def __init__(self):
    # empty variables to store the current values
    self.cur_pos = None
    self.cur_vel = None
    self.cur_acc = None
    # empty variables to store the previous values
    self.prev_pos = None
    self.prev_vel = None
    self.prev_acc = None

  @abstractmethod
  def Evaluate(self, x):
    pass
  @abstractmethod
  def getCur(self):
    pass
  @abstractmethod
  def getPrev(self):
    pass

  @abstractmethod
  def plot(self,t_range, name):
    pass

  # 
  # traj is a list of trajectories objects
  @classmethod
  def PlotTrajectory(self,traj:list,step:float,T:float,name:str):
    t_range = np.arange(0,T,step)
    for i in range(len(traj)):
      cur_name = name + str(i)
      traj[i].plot(t_range,cur_name)
  
  

class SinusoidalTraj(Trajectory):
  def __init__(self, y_center, frequency, tzero_y, amplitude = np.pi, joint_ll = None, joint_ul = None):
    '''
    y_center: center of the sinusoidal function on the y axis
    frequency: frequency of the sinusoidal function
    tzero_y: desired y_value for t = 0 (related to shift) (we need to ensure that the t_zero value is contained in the y_center +/- amplitude  )
    amplitude: amplitude of the sinusoidal function
    '''
    super().__init__()
    # Define a symbolic variable
    x = ca.MX.sym('x')
    if(amplitude == 0):
      shift = 0
    else:
      shift = np.arcsin(tzero_y/amplitude)
    # Define the sinusoidal function
    frequency_vel = 0
    frequency_func = frequency*ca.fabs(ca.cos(frequency_vel*x))
    sinusoidal = y_center + amplitude*ca.sin(frequency_func*x + shift)
    # in casadi if sinusoidal is bigger than joint_ul retunr joint_ul
    # if sinusoidal is smaller than joint_ll return joint_ll
    # else return sinusoidal
    if(joint_ll is not None):
      sinusoidal = ca.if_else(sinusoidal < joint_ll, joint_ll, sinusoidal)
    if(joint_ul is not None):
      sinusoidal = ca.if_else(sinusoidal > joint_ul, joint_ul, sinusoidal)
    

    # Calculate the first derivative
    first_derivative = ca.jacobian(sinusoidal, x)

    # Calculate the second derivative
    second_derivative = ca.jacobian(first_derivative, x)

    # Create a function to evaluate the expressions
    self.f = ca.Function('sinusoidal_function', [x], [sinusoidal, first_derivative, second_derivative])
    
  def Evaluate(self, x):
    self.prev_pos = self.cur_pos
    self_prev_vel = self.cur_vel
    self_prev_acc = self.cur_acc
    self.cur_pos,self.cur_vel,self.cur_acc = self.f(x)

  def getCur(self):
    return self.cur_pos,self.cur_vel,self.cur_acc
  def getPrev(self):
    return self.prev_pos,self.prev_vel,self.prev_acc

  def plot(self,t_range, name = "sinusoidal trajectory"):
    pos_value = np.zeros((len(t_range),))
    vel_value = np.zeros((len(t_range),))
    acc_value = np.zeros((len(t_range),))
    for i,t in enumerate(t_range):
      self.Evaluate(t)
      pos_value[i]= self.cur_pos
      vel_value[i] = self.cur_vel
      acc_value[i] = self.cur_acc
    plt.figure()
    plt.plot(t_range,pos_value)
    #plt.plot(t_range,vel_value)
    #plt.plot(t_range,acc_value)
    plt.title(name)
    plt.xlabel("time")
    plt.ylabel("joint pos (rad)")
    # adding legend 
    #plt.legend(["pos","vel","acc"])
    plt.show()
    #print("t:", t, "pos:", self.cur_pos, "vel:", self.cur_vel, "acc:", self.cur_acc)
    
  # here I create a static function to initiliaze the trajectory vector
  # here with dim i control the number of trajectory i want to create
  @classmethod
        
  def InitTrajectoryVector(self,dim,y_center, frequency, tzero_y, amplitude,joint_ll=[],joint_ul=[]):
    if(len(frequency)==1):
      frequency = [frequency]*dim
    if(len(frequency) != dim):
      raise ValueError("frequency vector does not match motor dim.")
    if(len(amplitude)==1):
      amplitude = [amplitude]*dim
    if(len(amplitude) != dim):
      raise ValueError("amplitude vector does not match motor dim.")
    if(len(y_center)==1):
      y_center = [y_center]*dim
    if(len(y_center) != dim):
      raise ValueError("y_center vector does not match motor dim.")
    if(len(tzero_y)!=dim):
      raise ValueError("tzero_y vector does not match motor dim.")
    trajs = []
    if(joint_ll==[] and joint_ul==[]):
      #  here  i initialize the trajectory vector without joint limits
      for i in range(dim):
        trajs.append(SinusoidalTraj(y_center[i],frequency[i],tzero_y[i],amplitude[i]))
    else:
      # if joint_limits re not empty i have to check if the dimension is correct
      if(len(joint_ll)is not dim):
        raise ValueError("joint_ll vector does not match motor dim.")
      if(len(joint_ul)is not dim):
        raise ValueError("joint_ul vector does not match motor dim.")
      for i in range(dim):
        trajs.append(SinusoidalTraj(y_center[i],frequency[i],tzero_y[i],amplitude[i],joint_ll[i],joint_ul[i]))
    return trajs
  
  @classmethod
  def GetDesiredTrajectory(self,traj,t):
    des_q= np.zeros((len(traj),))
    des_qd = np.zeros((len(traj),))  
    des_qdd = np.zeros((len(traj),))
    for i in range(len(traj)):
      traj[i].Evaluate(t)
      des_q[i],des_qd[i],des_qdd[i] = traj[i].getCur()
    return des_q,des_qd,des_qdd
  
  @classmethod
  def GetDesiredTrajectoryPrev(self,traj):
    des_q= np.zeros((len(traj),))
    des_qd = np.zeros((len(traj),))  
    des_qdd = np.zeros((len(traj),))
    for i in range(len(traj)):
      des_q[i],des_qd[i],des_qdd[i] = traj[i].getPrev()
    return des_q,des_qd,des_qdd

  

# TODO manage the normalization flag because as it is now i have to use the same normalization for the input and the target
# for both network the fix and the float   
class HorizonMotion(Trajectory):
  def __init__(self,file_path):
    # open the pickle file and store the results in a variable 
    with open(file_path, "rb") as f:
      self.horizon_motion = pickle.load(f)
      self.q_des  = self.horizon_motion["q_ref"]
      self.qd_des = self.horizon_motion["q_dot_ref"]
      self.qdd_des= self.horizon_motion["q_ddot_ref"]
      self.tau_des= self.horizon_motion["tau_ref"]
      self.f_des= self.horizon_motion["contact_map_ref"]
      # DEBUG -----------------------
      self.JtF_sum_des = self.horizon_motion["JtF_sum_ref"] 
      self.frame_res_force_mapping_des = self.horizon_motion["frame_res_force_mapping_store_ref"]
      self.ID_des = self.horizon_motion["ID_ref"]
      # -----------------------------
      self.cur_pos = None
      self.cur_vel = None
      self.cur_acc = None
      self.cur_tau = None
      self.cur_f_FL = None
      self.cur_f_FR = None
      self.cur_f_RL = None
      self.cur_f_RR = None
      # DEBUG -----------------------
      self.cur_JtF_sum = None
      self.cur_ID = None
      self.cur_frame_res_force_mapping = None
      # -----------------------------

    
  def Evaluate(self, x):
    self.prev_pos = self.cur_pos
    self.prev_vel = self.cur_vel
    self.prev_tau = self.cur_tau
    self.prev_f_FL = self.cur_f_FL
    self.prev_f_FR = self.cur_f_FR
    self.prev_f_RL = self.cur_f_RL
    self.prev_f_RR = self.cur_f_RR
    self.cur_pos = self.q_des[:,x]
    self.cur_vel = self.qd_des[:,x]
    self.cur_acc= self.qdd_des[:,x]
    self.cur_tau = self.tau_des[:,x]
    # DEBUG -----------------------
    self.cur_JtF_sum = self.JtF_sum_des[:,x]
    self.cur_frame_res_force_mapping = self.frame_res_force_mapping_des[x]
    self.cur_ID = self.ID_des[:,x]
    # -----------------------------
    self.cur_f_FL = self.f_des['FL_foot'][:,x]
    self.cur_f_FR = self.f_des['FR_foot'][:,x]
    self.cur_f_RL = self.f_des['FR_foot'][:,x]
    self.cur_f_RR = self.f_des['FR_foot'][:,x]

  def getCur(self):
    return self.cur_pos,self.cur_vel,self.cur_acc,self.cur_tau,self.cur_f_FL,self.cur_f_FR,self.cur_f_RL,self.cur_f_RR,self.cur_JtF_sum,self.cur_frame_res_force_mapping,self.cur_ID
    
  def getPrev(self):
    return self.prev_pos,self.prev_vel,self.prev_acc,self.prev_tau,self.prev_f_FL,self.prev_f_FR,self.prev_f_RL,self.prev_f_RR,self.prev_JtF_sum,self.prev_frame_res_force_mapping,self.prev_ID

  def plot(self,t_range, name):
    pass