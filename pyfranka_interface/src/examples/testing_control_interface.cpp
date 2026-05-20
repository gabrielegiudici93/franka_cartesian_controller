// Copyright (c) 2021 Inria, 2022 ucl

#include <cassert>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "cartesian_franka/robot.hpp"

// torque example (this does not work)
Eigen::VectorXd cartesian_impedance_control_test(franka::RobotState& robot_state, franka::Model& model, Eigen::Vector3d& position_d,
		                                    Eigen::Quaterniond& orientation_d, Eigen::MatrixXd& stiffness, Eigen::MatrixXd& damping){

	std::array<double, 7> coriolis_array = model.coriolis(robot_state);
	std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
	// convert to Eigen
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
	Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
	Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
	Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	Eigen::Vector3d position(transform.translation());
	Eigen::Quaterniond orientation(transform.linear());

	// compute error to desired equilibrium pose
	// position error
	Eigen::Matrix<double, 6, 1> error;
	error.head(3) << position - position_d;

	// orientation error
	// "difference" quaternion
	if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
	orientation.coeffs() << -orientation.coeffs();
	}
	// "difference" quaternion
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
	error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	// Transform to base frame
	error.tail(3) << -transform.linear() * error.tail(3);

	// compute control
	Eigen::VectorXd tau_task(7), tau_d(7);

	// Spring damper system with damping ratio=1
	tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
	tau_d << tau_task + coriolis;

	return tau_d;
}

// testing function for new version
franka_interface::ReturnControlCommand ctrl_func_joint_pos_internal(franka_interface::SimpleState& state, franka_interface::SimpleState& init_state,double cur_time){

	//std::cout << "cur_time = "<< cur_time<< std::endl;
	franka_interface::ReturnControlCommand ret;
	double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * cur_time));

	Eigen::VectorXd output = init_state.q;
    Eigen::VectorXd delta(7);
    delta<< 0,0,0,delta_angle,delta_angle,0,delta_angle;

	output= output + delta;

	if (cur_time >= 5.0) {
		ret.running_controller = false;
	}else{
		ret.running_controller = true;
	}
	ret.joint_space_command = output;
	//ret.joint_space_command = init_pos;
	return ret;
}


franka_interface::ReturnControlCommand ctrl_func_joint_vel_internal(franka_interface::SimpleState& state, franka_interface::SimpleState& init_state,double cur_time){
	double time_max = 1.0;
	double omega_max = 1.0;
	franka_interface::ReturnControlCommand ret;
	double cycle = std::floor(std::pow(-1.0, (cur_time - std::fmod(cur_time, time_max)) / time_max));
	double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * cur_time));

	Eigen::VectorXd velocities(7);
	velocities << 0.0, 0.0, 0.0, omega, omega, omega, omega;
	//velocities << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	if (cur_time >= 2 * time_max) {
		ret.running_controller = false;
	}else{
		ret.running_controller = true;
	}
	ret.joint_space_command = velocities;
	return ret;
}


franka_interface::ReturnControlCommand ctrl_func_cartesian_pose_internal(franka_interface::SimpleState& state, franka_interface::SimpleState& init_state,double cur_time){
	franka_interface::ReturnControlCommand ret;
	Eigen::Matrix4d transform(init_state.T);
	constexpr double kRadius = 0.3;
	double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * cur_time));
	double delta_x = kRadius * std::sin(angle);
	double delta_z = kRadius * (std::cos(angle) - 1);


	transform.data()[12] = transform.data()[12] + delta_x;
	transform.data()[14] = transform.data()[14] + delta_z;

	//std::cout<< "printing desired cartesian pose" << std::endl;
	//std::cout << transform.matrix() << std::endl;

	if (cur_time >= 10.0) {
		ret.running_controller = false;
	}else{
		ret.running_controller = true;
	}
	ret.cartesian_pose_command=transform;
	return ret;
}


franka_interface::ReturnControlCommand ctrl_func_cartesian_vel_internal(franka_interface::SimpleState& state, franka_interface::SimpleState& init_state,double cur_time){
	franka_interface::ReturnControlCommand ret;
    // Not implemented yet

	if (cur_time >= 10.0) {
		ret.running_controller = false;
	}else{
		ret.running_controller = true;
	}
	// ret.cartesian_pose_command=transform;
	return ret;
}

int main(int argc, char** argv)
{
	std::cout << "1" << std::endl;
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    std::string control_mode = "cartesian_pose";
    std::cout << "2" << std::endl;
    try {
    	franka_interface::RobotInterface<void> robot(argv[1],false,false);

        if(control_mode=="torque"){

        }else if(control_mode=="joint_velocity"){
        	robot.setController(&ctrl_func_joint_vel_internal,"joint_vel");
        	robot.run();

        }else if(control_mode=="joint_position"){

        	robot.setController(&ctrl_func_joint_pos_internal,"joint_pos");
        	robot.run();

        }else if(control_mode=="cartesian_pose"){

        	robot.setController(&ctrl_func_cartesian_pose_internal,"cartesian_pose");
        	robot.run();

        }else if(control_mode=="cartesian_vel"){

        	robot.setController(&ctrl_func_cartesian_vel_internal,"cartesian_vel");
        	robot.run();

        }else{
        	std::cout<<"wrong control mode fix it!"<<std::endl;
        }
        // only when run detach
        //robot.closeControlThread();
    }
    catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
    // closing control thread

    return 0;
}
