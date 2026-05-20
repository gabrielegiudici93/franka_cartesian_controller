#ifndef _FRANKA_FL_GENERATOR_HPP_
#define _FRANKA_FL_GENERATOR_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <mutex>
#include <atomic>
#include "com_def.hpp" 

namespace franka_interface {

    class FLMotionGenerator {
    public:
        
        FLMotionGenerator(franka::RobotState & cur_robot_state,
                         const Vector7d & p_diagonal, const Vector7d & d_diagonal){ //loop_stop(false){
             Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(cur_robot_state.q.data());
             Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_vel(cur_robot_state.dq.data()); // Assuming dq for velocity
             cur_q_des = q;
             cur_qd_des = q_vel; 
             P = p_diagonal.asDiagonal();
             D = d_diagonal.asDiagonal();          
        };


        void updateDesiredValues(Vector7d & q_d,Vector7d & qd_des) {
        std::lock_guard<std::mutex> lock(mutex_); // Lock the mutex for the scope of this function
        // Update q_des and qd_des here
        // Example:
            cur_q_des = q_d;
            cur_qd_des = qd_des;
        }
        
        franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

    private:   
        //std::unique_ptr<franka::Model> _model;
        //franka::Model _model;

        Vector7d cur_q_des;
        Vector7d cur_qd_des;
        Matrix7d P;
        Matrix7d D;
        std::mutex mutex_; // Mutex for protecting q_des and qd_des
        //std::atomic<bool> loop_stop;

        double time_ = 0.0;
       
    };
} // namespace franka_interface
#endif
