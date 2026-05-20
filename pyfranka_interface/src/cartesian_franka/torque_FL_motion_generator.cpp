#include "torque_FL_motion_generator.hpp"
#include <chrono>

namespace franka_interface {



    franka::JointPositions FLMotionGenerator::operator()(const franka::RobotState& robot_state,
        franka::Duration period)
    {   
         // Start timing
        auto start = std::chrono::high_resolution_clock::now();
        time_ += period.toSec();

        // the robot will not move just for testing
        franka::JointPositions output(this->cur_q_des);
        
        // Convert std::array from libfranka to Eigen
        Eigen::Map<const Vector7d> grav_vec(this->_model.gravity(robot_state).data());
        Eigen::Map<const Matrix7d> M(this->_model.mass(robot_state).data());
        Eigen::Map<const Vector7d> coriolis_vec(this->_model.coriolis(robot_state).data());

        // Assuming state.q and state.q_vel are std::array<double, 7>
        Eigen::Map<const Vector7d> q(state.q.data());
        Eigen::Map<const Vector7d> q_vel(state.q_vel.data());

        std::unique_lock<std::mutex> lock(mutex_);
            Eigen::VectorXd u = P * (this->cur_q_des - q) + D * (this->cur_qd_des - q_vel);
        lock.unlock();

        Eigen::VectorXd n = coriolis_vec + grav_vec;

        Eigen::VectorXd tau_FL = M * u + n;

        // Stop timing
        auto stop = std::chrono::high_resolution_clock::now();

        // Calculate duration
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        // For example, print the duration
        std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl;



        output.motion_finished = false;
        if(time_ > 5.0){
            output.motion_finished = true;
        }   
        return output;
    }






}