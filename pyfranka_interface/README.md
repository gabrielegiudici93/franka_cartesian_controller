# franka_interface

A simple library for moving the Franka robot from python or c++

## Install

- libfranka:

Install libfranka 0.9.2 from source [https://frankaemika.github.io/docs/installation_linux.html](https://frankaemika.github.io/docs/installation_linux.html).
in date 4/2/2024 0.9.2 corresponds to the current main branch

```
cd <Chosen directory>
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
git checkout <version>
git submodule update
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

Copy the `libfranka.so` files (incl. the numbered ones) from the build folder and paste them into franka_interface's `thirdparty/libfranka/lib` folder

- franka_interface

```
cd franka_interface
./waf configure --python
./waf
```

The interface requires python3. If the default alias on your system for 'python' is python2, you will need to run the ```waf``` script explicitly in python3, for example ```python3 ./waf configure --python``` and ```python3 ./waf```.

Another important note is that ALL of your python files should start with a python3 shebang, eg ```#!/usr/bin/env python3```.


## to install only the Python interface in a Mamba Environment

This guide assumes you have [Mamba](https://github.com/mamba-org/mamba) installed on your system. If not, you can install Mamba through [Miniconda](https://docs.conda.io/en/latest/miniconda.html) or [Miniforge](https://github.com/conda-forge/miniforge).

### Step 1: Create the Mamba Environment

1. **Prepare the Environment File**: Ensure you have an `environment.yml` file at the root of your project. This file should specify all the dependencies required by your project, including Python and any libraries your project depends on.


2. **Create the Environment**: Open your terminal, navigate to the directory containing the `environment.yml` file, and run the following command:

    ```bash
    mamba env create -f environment.yml
    ```

    Mamba will read the `environment.yml` file and create an environment named `myenv` (or whatever name you have specified in the file) with all the specified dependencies installed.

### Step 2: Activate the Environment

Once the environment is created, you need to activate it:

```bash
mamba activate myenv
```
### Step 3: Install the Python Library

With the environment activated, you can now install your Python library. Navigate to the root directory of your project where the `setup.py` file is located, and run:

```bash
python setup.py install
```

### Veryfing installation

```bash
python example.py  # Use the actual name of your library
```


## Panda connection

Connect Ethernet directly to the Panda's control unit. Set IPv4 connection to Manual / Static with Mask `255.255.255.0` and address `172.16.0.10` (or any free IP on the same subnet).
Open a browser and navigate to `http://172.16.0.2` to access the Desk interface. Desk username and password are set by your lab admin at first robot setup — ask whoever configured the robot if you don't have them.

## Library link

Do not forget to have <PATH TO LIBFRANKA FOLDER>/lib in your LD_LIBRARY_PATH:
```
export LD_LIBRARY_PATH=<PATH TO LIBFRANKA FOLDER>/lib:$LD_LIBRARY_PATH
```

###  Library Philosophy

the library is built so that the user can build a callback containing their control logic. For the Unitree robots, the control loop frequency is 1000 Hz (1 ms). this frequency could be changed in the code but it is better not to do that. You can command the robot in joint positions, velocities and torques, and cartesian pose. Moreover for the the franka_interface it is possible to send a single desired cartesian or joint position which does not require the definition of any callback or you can move the gripper with  

## Using The Library in C++

first of all, you need to include the robot.hpp that contains all the library (it is a template library so even the function body is defined inside a special include file called .tpp)
for using the library in cpp it is necessary to build the function callback with a specific function signature.

there are two different use cases that can be addressed with the library:
- sending single commands for both the robot and the gripper
- only with internal measures (only the robot state) and no data persistence
- using both internal and external measures and data persistence

### Sending Single Desired Position (joint/cartesian space) and for controlling the gripper
if you want to perform only regulation tasks in which you do not care about the trajectory but want only to reach a desired final position you can use several functions that allows you to do that:

- `void translate(const Eigen::Vector3d& delta, double duration = -1);`

- `void rotate(const Eigen::Vector3d& rpy, double duration = -1);`

- `void move(const Eigen::Vector3d& delta, const Eigen::Vector3d& rpy, double duration = -1);`

- `void move(const Eigen::Affine3d& m, double duration = -1);`

which allows controlling the end effector by giving cartesian space displacements, i.e. the robot will move w.r.t. its current pose

- `void extMove(const std::string& relative_or_absolute, const Eigen::Matrix4d& m, double duration = -1);`

which allows us to do both relative and absolute movement in the cartesian space. m represents a roto-translation matrix

- `void move_joints(const std::array<double, 7>& joint_positions, double duration = 1);`

that allows to move the robot to a desired joint positions


### Internal Measure and no Data Persistence
if you need only internal measures and no data persistence in your controller (a memory to save information during the controller execution
for a purely reactive controller) you can use this object declaration of the interface (for the sake of the example we will use the high-level interface but the same applies for the low-level)

```
franka_interface::RobotInterface<void> bot;
```
in the template, if you specify "void" you are signalling the library that there are no external measures and your controller will not have any data persistency during its execution.
in this case, the control_callback function has to have this signature:

```
ReturnControlCommand   control_callback(franka_interface::SimpleState& state, franka_interface::SimpleState& init_state,double cur_time){
         ReturnControlCommand cmd;
         ...
         function body
         ...
         return cmd;
}
```
the function has to return a structure called ReturnControlCommand and needs to take as input 3 variables state, init_state and time.
state and init_state are SimpleState and contain a lot of information about the current state of the robot and the initial state. cur_time is the current time from the start of the controller in milliseconds

at the end of the control_callback, you must always return a structure containing the current commands to send to the robot. the return function is defined in this way.
```
struct ReturnControlCommand{
   Eigen::VectorXd joint_space_command;
   Eigen::Matrix4d cartesian_command;
   bool running_controller;
   ReturnControlCommand() : joint_space_command(7), running_controller(true){};
};
```
it contains every possible returning command that we could send to the robot. in your control callback if you want to control:
- joint position, velocities or torques you need to use joint_space_command in the ReturnControlCommand structure.
- Cartesian pose you need to use cartesian_command from the ReturnControlCommand structure

To stop the execution of the controller when certain conditions are met (like a certain time has passed or the task is completed) if you do: 

```
cmd.runningController = false;
```
 this will trigger a sequence of actions that will safely stop the execution of the controller.
 in order to execute the callback you can do:
 ```
 franka_interface::RobotInterface<void> bot;
 std::control_modality;
 bot.setController(&controlLogic,control_modality);
 bot.run()  
 // or 
 bot.runDetach();
 ```
 with control_modality you select the kind of control you want to use and it has to be coherent with the control_callback that has been defined previously. the control_modality are:
 - joint_pos
 - joint_vel
 - torque
 - cartesian_pose
 
 if you use run() the main() execution will be blocked and it will only resume after the task. If you do runDetach() a thread containing the control loop will be executed and in your main() you will be able to do other stuff. In the case of runDetach() at the end of the code you should call 
 
 ```
 bot.closeControlThread()
 ```
 to properly close the running thread hosting the controller
 
 ### Internal Measure 
 if you want to use external measures or you just want to save data during the controller execution you first have to define a struct that contains all the information that you need:
 ```
 struct bar{
 
 data a;
 class object b;
 ...
 }
 ```
 
 once you have defined that you will have to declare the interface object as:
  ```
 franka_interface::RobotInterface<bar> bot;
  ```
 and the signature of the control_callback function becomes:
 
 ```
 ReturnControlCommand   control_callback(franka_interface::SimpleState& state, franka_interface::SimpleState& init_state,bar& data,double cur_time){
         ReturnControlCommand cmd; 
         ...
         function body
         ...
         return cmd;
}
```
differently from the other modality in this example you have a new input "bar& data" which is exactly the data structure that you have defined and that data structure can be used at each control step to query external information or for updating or recalling persistent data.

Once the control_calback is defined you can pass the function to the interface with:
- with  ```setCtrl(control_callback,control_modality);```


when using ```setCtrl(control_callback);``` you need to define the behaviour of the empty constructor of the struct bar


for any control interface you can look at the example in the folder /franka_interface/src

## Using The Library in Python

if you want to use the library in Python the use is very close to the C++ version.
you need to define a control_callback with a precise signature for the high-level and low-level cases. 
the control callback in both cases will be
```
  def control_callback(state, init_state, time): 
```
since in Python, the variable types are implicitly induced by the Python interpreter to signal to Python that we are defining a control callback compatible with the library we need to specify the return variable in this way:

```
  def control_callback(state, init_state, time): 
          cmd = Robot_.ReturnControlCommand()
          ...
          return cmd
```

**be careful!** It is really important to define and return the cmd object otherwise you will get weird errors that are not easy to read because the library is obtained from the pybind
Once the control_callback is defined you can pass the function to the cpp code by doing:
```
robot = bot.Robot_()  
control_modality = #- joint_pos -joint_vel - torque - cartesian_pose
robot.set_controller(control_callback,control_modality)
bot.run()
```
in the Python version, there is no need to have a version with and one without external measure/data persistency because in Python, it is easy to pass external objects in the callback by using global variables

an example of use for the interface can be found in the folder `/franka_interface/src`

