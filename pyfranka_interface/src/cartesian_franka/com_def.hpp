// com_def.hpp

#ifndef _COM_DEF_HPP_
#define _COM_DEF_HPP_


#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/robot.h>

namespace franka_interface {

	  typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;
      typedef Eigen::Matrix<int, 7, 1, Eigen::ColMajor> Vector7i;
      typedef Eigen::Matrix<double, 7, 7, Eigen::ColMajor> Matrix7d;
      typedef Eigen::Matrix<double, 42, 1> Vector42d;
	
	  struct GripSimpleState{
	   	   bool isgrasped;
	   	   double width;
	   	   double max_width;
	   
	   };
	   	
	   struct SimpleState{
		   Eigen::VectorXd q;
		   Eigen::VectorXd q_vel;
		   Eigen::VectorXd tau;
		   Eigen::Matrix4d T;
		   SimpleState() : q(7),q_vel(7),tau(7){};
	   };

	   struct ReturnControlCommand{
		   Eigen::VectorXd joint_space_command;
		   Eigen::Matrix4d cartesian_pose_command;
		   Eigen::VectorXd cartesian_vel_command;
		   bool running_controller;
		   ReturnControlCommand() : joint_space_command(7), running_controller(true){};
	   };
	   
	   struct CtrlPerformance{
		   Eigen::VectorXd q_des;
		   Eigen::VectorXd q_err;
		   Eigen::VectorXd qd_err;
		   Eigen::VectorXd last_tau;
	   };
   
}

#endif
