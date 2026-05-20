#ifndef _FRANKA_ROBOT_HPP_
#define _FRANKA_ROBOT_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/control_types.h> 
#include <franka/exception.h>
#include <franka/robot.h>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <bitset> // For bit manipulation
#include <franka/model.h>
#include <franka/gripper.h>
#include "com_def.hpp"
#include "cartesian_franka/cartesian_motion_generator.hpp"
#include "cartesian_franka/joint_motion_generator.hpp"
#include "cartesian_franka/torque_FL_motion_generator.hpp"
#include <chrono>
#include <atomic>


namespace franka_interface { // cf = franka_interface
   
   enum UpdateFlags {
        UpdateNone = 0,
        UpdateCoriolis = 1 << 0, // 1 in bit 100
        UpdateGravity = 1 << 1,  // 2 in bit 010
        UpdateMass = 1 << 2      // 4 in bit 001    

    };
    // therefore if i do UpdateCoriolis | UpdateMass i get 101 (5) or if i do UpdateCoriolis | UpdateGravity i get 011 (6) and so on
   
   //void structure 
   struct Ref{};
   // specialization for class memebr
   template < typename T > struct TorVoidStr{ typedef T Value; };
   // instead of void i go with an empty struct Ref
   template < > struct TorVoidStr<void> { typedef Ref Value; };
   // specialization through the structure RefOVoidStr for function signature
   template < typename T > struct RefOVoidStr{ typedef T& Value; };
   // instead of void i go with an empty struct Ref
   template < > struct RefOVoidStr<void> { typedef Ref Value; };

    template <class T>
    class RobotInterface {
    public:

        /// take the IP of the robot as input 
	   RobotInterface(const std::string& ip,bool realtime, bool hand_franka=true, bool auto_init=true, double speed_factor = 0.5):loop_stop(false)
       {
            this->active_franka_hand = hand_franka;
            if(realtime){
            _robot.reset(new franka::Robot(ip.c_str()));
            }
            else{
            _robot.reset(new franka::Robot(ip.c_str(), franka::RealtimeConfig::kIgnore));
            }
            
            if(this->active_franka_hand){
            _gripper.reset(new franka::Gripper(ip.c_str()));
            }
            _set_default_behavior();
            
            // _model needs to be initialized before getSimpleState 
            _model = std::make_unique<franka::Model>(_robot->loadModel());

            this->init_state = this->getSimpleState();
            //this->_gripper.homing();
            //this->_model = this->_robot.loadModel();

            //debugging
            //std::cout<< "printing current cartesian pose" << std::endl;
            //std::cout << this->init_state.T.matrix() << std::endl;

            // by default update flag is set to 0
            this->updateFlags = 0;

            if (auto_init){
                //init(speed_factor);
            }
        }


        /// go to the starting position using joint positions
        void init(double speed_factor = 0.5);

        // all these function are relative to the current position/orientation
        void translate(const Eigen::Vector3d& delta, double duration = -1);
        void rotate(const Eigen::Vector3d& rpy, double duration = -1);
        void extMove(const std::string& relative_or_absolute, const Eigen::Matrix4d& m, double duration = -1);
        void move(const Eigen::Vector3d& delta, const Eigen::Vector3d& rpy, double duration = -1);
        void move(const Eigen::Affine3d& m, double duration = -1);

        // move to a joint position
        void move_joints(const std::array<double, 7>& joint_positions, double duration = 1);
        void FL_controller(const Vector7d & p_diagonal,const Vector7d & d_diagonal);
        /// end-effector transform (position & rotation)
        Eigen::Affine3d affine3d();
        Eigen::Vector3d position();      // end_effector position for tracking.
        Eigen::Vector3d orientation();   // current end_effector orientation for tracking.

        /// the libfranka robot if needed for other functions
        //const franka::Robot& franka() const { return _robot; };
        void automaticErrorRecovery();

        // new stuff ------------------------------------------------------------------------------------------------------------------------------------------------

        // gripper function
        GripSimpleState getGripperSimpleState();
        void moveGripper(double width,double speed);
        void grasp(double width,double speed,double force,
                    double epsilon_inner=0.005, double epsilon_outer=0.005);

        // added by luke
        bool isError();
        franka::Errors getErrorStruct();
        std::string getErrorString();
        // end added by luke


        // get function (todo extend this function and the corresponding structure to get all we need inside the controller)
        franka::RobotState getState();
        SimpleState getSimpleState();
        void state2SimpleState(franka::RobotState state,SimpleState & sstate);

        Eigen::Matrix4d getFrame(std::string& frame);

        // set commands


        // run command
        void run();
        void runDetach();


        // close thread
        void closeControlThread(){
            this->stopFLController();
        	sendControl.join();
        }

        // methods for Feedback Linearization Controller
        void setFLController(const Vector7d & p_diagonal, const Vector7d & d_diagonal);
        void setFLControllerTest(const Vector7d & p_diagonal, const Vector7d & d_diagonal);
        void updateDesiredValues(Vector7d & q_d,Vector7d & qd_des); 
        void stopFLController(){
            this->loop_stop = true;
        };
        // get and set current performance of the controller
        void setCtrlPerformance(const Vector7d & q_des, const Vector7d & q_err, const Vector7d & qd_err, const Vector7d & last_tau);
        CtrlPerformance getCtrlPerformance();
        // get and set current state of the robot
        void setCurActionState(const Vector7d & q, const Vector7d & qd, const Vector7d & tau);
        SimpleState getCurActionState();

        void setController(std::function<ReturnControlCommand(SimpleState&, SimpleState&, double)> func, std::string control_level);
        // with this function you can set the internal control logic and any exteroceptive measure or persistent data
        void setCtrl(std::function<ReturnControlCommand(SimpleState&, SimpleState&, typename RefOVoidStr<T>::Value, double)> ctrl, std::string control_level);
        // with this function you can set the internal control logic and any exteroceptive measure or persistent data
        //void setCtrlMeasr(std::function<ReturnControlCommand(SimpleState&, SimpleState&, typename RefOVoidStr<T>::Value, double)> ctrl, typename RefOVoidStr<T>::Value ext_data,std::string control_level);
        
        franka::Model frankaModel() {return _robot->loadModel();};

        // Method to update the mass matrix
        void updateMassMatrix(const franka::RobotState& robot_state);
        void updateCoriolisVector(const franka::RobotState& robot_state);
        void updateGravityVector(const franka::RobotState& robot_state);

        // Performs updates of dyanmical quantity of the robot based on the current flags and given robot state
        void updateVectors(const franka::RobotState& robot_state); 

         // Sets the update flags
         // example of usages:
         // cpp code: robotInterface.setUpdateFlags(UpdateFlags::UpdateCoriolis | UpdateFlags::UpdateMass); // in this example im computing both coriolis and mass matrix (is the bitwsie or to build the vector 101)
        void setUpdateFlags(int flags) {
            this->updateFlags = flags;
        }

        // Getter for the mass matrix
        Eigen::Matrix<double, 7, 7> getMassMatrix() const {
            return mass_matrix;
        }
        // Getter for the coriolis vrctor
        Eigen::VectorXd getCoriolisVector() const {
            return coriolis_vector;
        }
        //getter for the gravity vector
        Eigen::VectorXd getGravityVector() const {
            return gravity_vector;
        }

        // Getter for the Jacobian vector in base frame, for EE frame
        Eigen::VectorXd getEndEffectorZeroJacobian() const;
        // Getter for the Jacobian vector in base frame, for EE frame
        Eigen::VectorXd getEndEffectorZeroJacobianFromState(const franka::RobotState& robot_state) const;

    protected:
        void _set_default_behavior();

        std::thread sendControl;



        //# for accessing robot stuff
        std::unique_ptr<franka::Robot> _robot;
        std::unique_ptr<franka::Gripper> _gripper;
        // selecting control type
        std::string control_level;
        // control function to interfaces with the robot.control() franka library
        std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)> ctrl_func_joint_pos;
        std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)> ctrl_func_joint_vel;
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)> ctrl_func_tau;
        std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)> ctrl_func_cart_pose;
        std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)> ctrl_func_cart_vel;

        // init_state (to change with a struct)
        SimpleState init_state;
        // used to store stuff from outside
        typename TorVoidStr<T>::Value persistent_data={};
        // current time
        double time;
        std::unique_ptr<franka::Model> _model;
        // current matrices
        Eigen::Matrix<double, 7, 7> mass_matrix = Eigen::Matrix<double, 7, 7>::Zero(); // To store the mass matrix;
        Eigen::VectorXd coriolis_vector = Eigen::VectorXd::Zero(7); // To store the Coriolis force vector;
        Eigen::VectorXd gravity_vector = Eigen::VectorXd::Zero(7); // To store the gravity vector;
        
        // variables for FL Controller
        Vector7d cur_q_des;
        Vector7d cur_qd_des;
        Matrix7d P;
        Matrix7d D;
        std::mutex mutex_; // Mutex for protecting q_des and qd_des
        std::mutex mutex_data_read; // Mutex for protecting the data reading perfomance
        std::mutex mutex_state; // Mutex for protecting the state reading
        CtrlPerformance ctrl_performance;
        SimpleState cur_state;
        // flag to stop the controller
        std::atomic<bool> loop_stop;
        

        int updateFlags; 
       

	// with this flag I check if the franka robot is using the default franka hand
	bool active_franka_hand;
    };
#include "robot.tpp"
} // namespace cartesian_franka

#endif
