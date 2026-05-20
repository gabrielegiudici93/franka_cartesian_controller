//robot.tpp



template <class T>
void RobotInterface<T>::_set_default_behavior()
    {
        _robot->setCollisionBehavior(
            {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}},
            {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}},
            {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}},
            {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}},
            {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}},
            {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}},
            {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}},
            {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}});
        _robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        _robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    }
    template <class T>
    void RobotInterface<T>::init(double speed_factor)
    {
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        move_joints(q_goal, speed_factor);
    }
	
    // REGULATION TASK 
    
    
    template <class T>
    void RobotInterface<T>::extMove(const std::string& relative_or_absolute, const Eigen::Matrix4d& m, double duration)
    {	
    
    	if(relative_or_absolute == "relative"){
	    	Eigen::Affine3d delta(m);
		CartesianMotionGenerator generator(delta, duration);
		_robot->control(generator);
        }else if(relative_or_absolute == "absolute"){
        
            SimpleState cur_state = getSimpleState();
            Eigen::Affine3d cur_pose(cur_state.T);
            Eigen::Affine3d des_pose(m);
            
            Eigen::Vector3d delta_translation   = des_pose.translation() - cur_pose.translation();
            Eigen::Quaterniond cur_rot          = Eigen::Quaterniond(cur_pose.linear());
            Eigen::Quaterniond delta_rotation   = Eigen::Quaterniond(des_pose.linear())*cur_rot.inverse();
            
            // relative motion
            Eigen::Affine3d delta(Eigen::Affine3d::Identity());
            delta.linear() = delta_rotation.normalized().toRotationMatrix();
            delta.translation() = delta_translation; 	
                CartesianMotionGenerator generator(delta, duration);
            _robot->control(generator);
        }else{
        	std::cout<<"the field relative_or_absolute is wrong, only admisible string are relative, absolute, check the spelling!"<<std::endl;  
        }
        
    }
     template <class T>
     void RobotInterface<T>::move(const  Eigen::Affine3d& delta, double duration)
    {	
        CartesianMotionGenerator generator(delta, duration);
        this->_robot->control(generator);
    }
    template <class T>
    void RobotInterface<T>::move(const Eigen::Vector3d& delta, const Eigen::Vector3d& rpy, double duration)
    {
        Eigen::Affine3d t = Eigen::Translation3d(delta)
            * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
        assert(t.translation() == delta);
        move(t, duration);
    }
    template <class T>
    void RobotInterface<T>::rotate(const Eigen::Vector3d& rpy, double duration)
    {
        Eigen::Quaterniond q = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
        Eigen::Affine3d t(q);
        move(t, duration);

    }
     template <class T>
     void RobotInterface<T>::translate(const Eigen::Vector3d& delta, double duration)
    {
        // std::cout<<"translate to:"<<delta.transpose()<<" duration="<<duration<<std::endl;    
        Eigen::Affine3d t = Eigen::Affine3d::Identity();
        t.translation() = delta;
        move(t, duration);
    }
    template <class T>
    void RobotInterface<T>::move_joints(const std::array<double, 7>& joint_positions, double speed_factor)
    {	
        JointMotionGenerator joint_motion_generator(speed_factor, joint_positions);
        _robot->control(joint_motion_generator);
    }

    template <class T>
    Eigen::Vector3d RobotInterface<T>::position()
    {
        return affine3d().translation();
    }
    template <class T>
    Eigen::Vector3d RobotInterface<T>::orientation()
    {
        Eigen::Quaterniond q(affine3d().linear());
        return q.toRotationMatrix().eulerAngles(0, 1, 2);
    }
    template <class T>
    Eigen::Affine3d RobotInterface<T>::affine3d()
    {
        franka::RobotState robot_state = _robot->readOnce();
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity(); // = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        // the map should work...
        for (int i = 0; i < 16; ++i)
            m.data()[i] = robot_state.O_T_EE_c[i];
        Eigen::Affine3d transform(m);
        assert(transform.translation()[0] == robot_state.O_T_EE_c[12]);
        assert(transform.translation()[1] == robot_state.O_T_EE_c[13]);
        assert(transform.translation()[2] == robot_state.O_T_EE_c[14]);
        return transform;
    }
    template <class T>
    void RobotInterface<T>::automaticErrorRecovery()
    {
        _robot->automaticErrorRecovery();
    }
    //// new stuff ---------------------------------------------------------------------------------------
    // PUBLIC FUNCTION

    template <class T>
    GripSimpleState RobotInterface<T>::getGripperSimpleState(){
    	if(this->active_franka_hand){
	    	franka::GripperState gripper_state = this->_gripper->readOnce();
	    	GripSimpleState ret;
	    	ret.isgrasped = gripper_state.is_grasped;
	    	ret.width     = gripper_state.width; 
	    	ret.max_width = gripper_state.max_width;
	    	return ret;
    	}else{
    		GripSimpleState ret;
    		std::cout << "the franka robot is not currently using the default franka hand. change the configuration in the franka web server" << std::endl;
    		return ret;
    	
    	}
    }
    template <class T>
    void RobotInterface<T>::moveGripper(double width,double speed){
    	if(this->active_franka_hand){
    		this->_gripper->move(width, speed);
    	}else{
    		std::cout << "the franka robot is not currently using the default franka hand. change the configuration in the franka web server" << std::endl;    	
    	}
    }
    template <class T>
    void RobotInterface<T>::grasp(double width,double speed,double force, double epsilon_inner, double epsilon_outer){
	if(this->active_franka_hand){																
    		this->_gripper->grasp(width, speed, force, epsilon_inner, epsilon_outer);
    	}else{	
    		std::cout << "the franka robot is not currently using the default franka hand. change the configuration in the franka web server" << std::endl; 	
    	}
    }
    	
    // ----- added by Luke -----
    template <class T>
    bool RobotInterface<T>::isError()
    {
      /* return a boolean indicating if there is currently an error reported
      from the franka */

      franka::RobotState robot_state = _robot->readOnce();

      // Errors struct supports bool operator
      return bool(robot_state.current_errors);
    }

    template <class T>
    franka::Errors RobotInterface<T>::getErrorStruct()
    {
      /* return the error struct, see: https://frankaemika.github.io/libfranka/structfranka_1_1Errors.html#details
      Access the members of this struct to examine the errors */

      franka::RobotState robot_state = _robot->readOnce();

      // Errors struct supports bool operator
      return robot_state.current_errors;
    }

    template <class T>
    std::string RobotInterface<T>::getErrorString()
    {
      /* return a string with names of active errors: "[active_error_name1, 
      active_error_name_2, ... active_error_name_n]" If no errors are active, 
      the string contains empty brackets: "[]" */

      franka::RobotState robot_state = _robot->readOnce();

      // Errors struct supports bool operator
      return std::string(robot_state.current_errors);
    }
    // ----- end added by luke -----

    // get function
    template <class T>
    franka::RobotState RobotInterface<T>::getState(){
    	franka::RobotState whole_state=this->_robot->readOnce();
    	return whole_state;
    }

    template <class T>
    SimpleState RobotInterface<T>::getSimpleState(){
        SimpleState cur_state;
    	franka::RobotState whole_state=this->_robot->readOnce();
    	// joint positions
		for (int i = 0; i < 7; ++i)
			cur_state.q[i]=whole_state.q[i];
		// joint velocities
		for (int i = 0; i < 7; ++i)
			cur_state.q_vel[i]=whole_state.dq[i];
		// torques
		for (int i = 0; i < 7; ++i)
			cur_state.tau[i]=whole_state.tau_J[i];
    	// reading e-e cartesian position
		Eigen::Matrix4d m; // = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
		// the map should work...
		for (int i = 0; i < 16; ++i)
			m.data()[i] = whole_state.O_T_EE_c[i];
		cur_state.T = m;
		return cur_state;
    }
    template <class T>
    void RobotInterface<T>::state2SimpleState(franka::RobotState rstate,SimpleState & sstate){
    	// joint positions
    	for (int i = 0; i < 7; ++i)
    		sstate.q[i]=rstate.q[i];
    	// joint velocities
    	for (int i = 0; i < 7; ++i)
    	    sstate.q_vel[i]=rstate.dq[i];
    	// torques
    	for (int i = 0; i < 7; ++i)
    	    sstate.tau[i]=rstate.tau_J[i];
    	// cartesian pose
    	Eigen::Matrix4d m; // = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
    	for (int i = 0; i < 16; ++i)
			m.data()[i] = rstate.O_T_EE_c[i];
    	sstate.T = m;
    }

    template<class T>
    Eigen::Matrix4d RobotInterface<T>::getFrame(std::string& frame){
        franka::RobotState whole_state=this->_robot->readOnce();
        std::array<double,16> pose; 
        Eigen::Matrix4d m;

        if (frame == "joint1"){
            pose = this->_robot->loadModel().pose( franka::Frame::kJoint1, whole_state);
        }
        else if (frame == "joint2"){
            pose = this->_robot->loadModel().pose( franka::Frame::kJoint2, whole_state);
        }
        else if (frame == "joint3"){
            pose = this->_robot->loadModel().pose( franka::Frame::kJoint3, whole_state);
        }
        else if (frame == "joint4"){
            pose = this->_robot->loadModel().pose( franka::Frame::kJoint4, whole_state);
        }
        else if (frame == "joint5"){
            pose = this->_robot->loadModel().pose( franka::Frame::kJoint5, whole_state);
        }
        else if (frame == "joint6"){
            pose = this->_robot->loadModel().pose( franka::Frame::kJoint6, whole_state);
        }
        else if (frame == "joint7"){
            pose = this->_robot->loadModel().pose( franka::Frame::kJoint7, whole_state);
        }
        else if (frame == "flange"){
            pose = this->_robot->loadModel().pose( franka::Frame::kFlange, whole_state);
        }
        else if (frame == "end_effector"){
            pose = this->_robot->loadModel().pose( franka::Frame::kEndEffector, whole_state);
        }
        else if (frame == "stiffness"){
            pose = this->_robot->loadModel().pose( franka::Frame::kStiffness, whole_state );
        }
        else{
            std::cerr<<"Wrong name, use jointX, flange, stiffness, or end_effector";
        };

    for (int i = 0; i < 16; ++i){
            m.data()[i] = pose[i];
        }
        return m;
    }

	template <class T>
	void RobotInterface<T>::setFLController(const Vector7d & p_diagonal, const Vector7d & d_diagonal){
		this->control_level = "torque";
		this->P = p_diagonal.asDiagonal();
    	this->D = d_diagonal.asDiagonal(); 
		franka::RobotState cur_robot_state = this->_robot->readOnce();
		Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(cur_robot_state.q.data());
		Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_vel(cur_robot_state.dq.data()); // Assuming dq for velocity
		// i need this to set up the initial state for the wholw body module that takes the input from an external topic
		this->cur_state.q = q;
		this->cur_state.q_vel = q_vel; 
		// i need this to set up the initial desired state for the low level controller where if im not commanding anythign the robot will not move
		this->cur_q_des = q;
		this->cur_qd_des = q_vel;
		this->ctrl_func_tau = [this](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques{

			auto start = std::chrono::high_resolution_clock::now();
			this->time = this->time + period.toSec();
			// current state of the robot
			// Assuming state.q and state.q_vel are std::array<double, 7>
			Eigen::Map<const Vector7d> q(robot_state.q.data());
			Eigen::Map<const Vector7d> q_vel(robot_state.dq.data());

			// the robot will not move just for testing
			//franka::JointPositions output={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
			//std::array<double, 7> q_goal = {{0, -M_PI_4,    0, -3 * M_PI_4,   0, M_PI_2, M_PI_4}};
			franka::Torques output={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
			//Eigen::VectorXd::Map(&output.q[0], 7) = q;
			
			// Convert std::array from libfranka to Eigen
			//Eigen::Map<const Vector7d> grav_vec(this->_model->gravity(robot_state).data()); // important gravity already compensated inside!!!!!!
			Eigen::Map<const Matrix7d> M(this->_model->mass(robot_state).data());
			Eigen::Map<const Vector7d> coriolis_vec(this->_model->coriolis(robot_state).data());

			std::unique_lock<std::mutex> lock(this->mutex_);
				Eigen::VectorXd local_q_des = this->cur_q_des;
				Eigen::VectorXd local_qd_des = this->cur_qd_des;
				//Eigen::VectorXd u = P * (this->cur_q_des - q) + D * (this->cur_qd_des - q_vel);
			lock.unlock();
				Eigen::VectorXd u = P * (local_q_des - q) + D * (local_qd_des - q_vel);

			// add measures to evaluate the controller perfomances

			// DEBUG
			//std::cout << "currrent q_vel: " << q_vel.transpose() << std::endl;
			//std::cout << "current q_vel_des: " << local_qd_des.transpose() << std::endl;
			//std::cout << "position error: " << (local_q_des - q).transpose() << std::endl;
			//std::cout << "velocity error: " << (local_qd_des - q_vel).transpose() << std::endl;
			//std::cout << "coriolis_vec: " << coriolis_vec.transpose() << std::endl;
			//std::cout << "u: " << u.transpose() << std::endl;
			//std::cout << "cur_q_des: " << this->cur_q_des.transpose() << std::endl;
			//std::cout << "position error: " << (this->cur_q_des - q).transpose() << std::endl;
			// feedback linearization
			Eigen::VectorXd tau_FL = M * u + coriolis_vec;

			// render available the current joint state outside
			this->setCurActionState(q,q_vel,tau_FL);

			Eigen::VectorXd::Map(&output.tau_J[0], 7) = tau_FL;
			std::cout << "tau_FL: " << tau_FL.transpose() << std::endl;
			//this->setCtrlPerformance(local_q_des, (local_q_des - q), (local_qd_des - q_vel), tau_FL);

			// Stop timing
			auto stop = std::chrono::high_resolution_clock::now();

			// Calculate duration
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
			// Convert microseconds to milliseconds
			// double duration_milliseconds = duration.count() / 1000.0;
			// For example, print the duration
			//std::cout << "Time taken by function: " << duration_milliseconds << " milliseconds" << std::endl;

			output.motion_finished = false;
			if(this->loop_stop){
				output.motion_finished = true;
			}   
			return output;
		};	
			
	}
	// this is a fake function that control the robot in joint position not torques do anything but it is useful for testing the control interface
	template <class T>
	void RobotInterface<T>::setFLControllerTest(const Vector7d & p_diagonal, const Vector7d & d_diagonal){
		this->control_level = "joint_pos";
		this->P = p_diagonal.asDiagonal();
    	this->D = d_diagonal.asDiagonal(); 
		franka::RobotState cur_robot_state = this->_robot->readOnce();
		
		Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(cur_robot_state.q.data());
		Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_vel(cur_robot_state.dq.data()); // Assuming dq for velocity
		std::cout << "cur_robot_state.q: " << q<< std::endl;
		std::cout << "cur_robot_state.dq: " << q_vel << std::endl;
		this->cur_state.q = q;
		this->cur_state.q_vel = q_vel; 
		this->cur_q_des = q;
		this->cur_qd_des = q_vel;
		this->ctrl_func_joint_pos = [this](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions{

			auto start = std::chrono::high_resolution_clock::now();
			this->time = this->time + period.toSec();
			// current state of the robot
			// Assuming state.q and state.q_vel are std::array<double, 7>
			Eigen::Map<const Vector7d> q(robot_state.q.data());
			Eigen::Map<const Vector7d> q_vel(robot_state.dq.data());
			
			// the robot will not move just for testing
			//franka::JointPositions output={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
			//std::array<double, 7> q_goal = {{0, -M_PI_4,    0, -3 * M_PI_4,   0, M_PI_2, M_PI_4}};
			franka::JointPositions output={{0.0, -M_PI_4, 0.0, -3 * M_PI_4, 0.0, M_PI_2,  M_PI_4}};
			//Eigen::VectorXd::Map(&output.q[0], 7) = q;
			
			// Convert std::array from libfranka to Eigen
			//Eigen::Map<const Vector7d> grav_vec(this->_model->gravity(robot_state).data()); // important gravity already compensated inside!!!!!!
			Eigen::Map<const Matrix7d> M(this->_model->mass(robot_state).data());
			Eigen::Map<const Vector7d> coriolis_vec(this->_model->coriolis(robot_state).data());

			std::unique_lock<std::mutex> lock(this->mutex_);
				Eigen::VectorXd u = P * (this->cur_q_des - q) + D * (this->cur_qd_des - q_vel);
			lock.unlock();

			Eigen::VectorXd tau_FL = M * u + coriolis_vec;
			//std::cout << "tau_FL: " << tau_FL.transpose() << std::endl;
			// render available the current joint state outside
			this->setCurActionState(q,q_vel,tau_FL);
			// Stop timing
			auto stop = std::chrono::high_resolution_clock::now();

			// Calculate duration
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
			// Convert microseconds to milliseconds
			// double duration_milliseconds = duration.count() / 1000.0;
			// For example, print the duration
			//std::cout << "Time taken by function: " << duration_milliseconds << " milliseconds" << std::endl;

			output.motion_finished = false;
			if(this->time > 5.0){
				output.motion_finished = true;
			}   
			return output;
		};	
			
	}
	template <class T>
	void RobotInterface<T>::updateDesiredValues(Vector7d & q_d,Vector7d & qd_des) {
        std::lock_guard<std::mutex> lock(this->mutex_); // Lock the mutex for the scope of this function
        // Update q_des and qd_des here
        // Example:
            this->cur_q_des = q_d;
            this->cur_qd_des = qd_des;
    }

	template <class T>
	void RobotInterface<T>::setCtrlPerformance(const Vector7d & q_des, const Vector7d & q_err, const Vector7d & qd_err, const Vector7d & last_tau){
		std::lock_guard<std::mutex> lock(this->mutex_data_read);
			this->ctrl_performance.q_des = q_des;
			this->ctrl_performance.q_err = q_err;
			this->ctrl_performance.qd_err = qd_err;
			this->ctrl_performance.last_tau = last_tau;
	}
	template <class T>
	CtrlPerformance RobotInterface<T>::getCtrlPerformance(){
		std::lock_guard<std::mutex> lock(this->mutex_data_read);
		return this->ctrl_performance;
	}
	template <class T>
	void RobotInterface<T>::setCurActionState(const Vector7d & q, const Vector7d & qd,const Vector7d & tau){
		std::lock_guard<std::mutex> lock(this->mutex_state);
		this->cur_state.q = q;
		this->cur_state.q_vel = qd;
		this->cur_state.tau = tau;
	}
	template <class T>
    SimpleState RobotInterface<T>::getCurActionState(){
		std::lock_guard<std::mutex> lock(this->mutex_state);
		return this->cur_state;
	}



    template <class T>
    void RobotInterface<T>::setController(std::function<ReturnControlCommand(SimpleState&, SimpleState&, double)> func, std::string control_level){
		// setting_the control interface
    	if(control_level == "joint_pos"){
			this->control_level = control_level;
			this->ctrl_func_joint_pos = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions{
				//
				ReturnControlCommand ret;
				SimpleState st;
				franka::JointPositions ret_franka={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state, this->time);
				// convert from VectorXd to franka::JointPositions
				Eigen::VectorXd::Map(&ret_franka.q[0], 7) = ret.joint_space_command;
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
			};
    	}else if(control_level == "joint_vel"){
    		this->control_level = control_level;
    		this->ctrl_func_joint_vel = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities{
				//
				ReturnControlCommand ret;
				SimpleState st;
				franka::JointVelocities ret_franka={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state, this->time);
				// convert from VectorXd to franka::JointVelocities
				Eigen::VectorXd::Map(&ret_franka.dq[0], 7) = ret.joint_space_command;
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
			};

    	}else if(control_level == "torque"){
    		this->control_level = control_level;
			this->ctrl_func_tau = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques{
				//
				ReturnControlCommand ret;
				SimpleState st;
				franka::Torques ret_franka={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state, this->time);
				// convert from VectorXd to franka::JointVelocities
				Eigen::VectorXd::Map(&ret_franka.tau_J[0], 7) = ret.joint_space_command;
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
			};

    	}else if(control_level == "cartesian_pose"){
    		this->control_level = control_level;
			this->ctrl_func_cart_pose = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose{
				//
				ReturnControlCommand ret;
				SimpleState st;
				std::array<double, 16> ret_franka;
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state, this->time);
				// convert from Eigen::Affine3 to franka::JointVelocities
				std::copy_n(ret.cartesian_pose_command.data(), ret_franka.size(), ret_franka.begin());
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
			};

    	}else if(control_level == "cartesian_vel"){
    		this->control_level = control_level;
			this->ctrl_func_cart_vel = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities{
				//
				ReturnControlCommand ret;
				SimpleState st;
				std::array<double, 6> ret_franka;
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state, this->time);
				// convert from Eigen::Affine3 to franka::JointVelocities
				std::copy_n(ret.cartesian_vel_command.data(), ret_franka.size(), ret_franka.begin());
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
            };
    	}else{
    		
			std::cout << "RobotInterface<T>::setController:: control level non existent, control levels allowed are:"
            << " joint_pos, joint_vel, torque, cartesian_pose, cartesian_vel. " 
            << "Check for typos: "<< control_level  << ";" << std::endl;
        }

    }
    
    template <class T>
    void RobotInterface<T>::setCtrl(std::function<ReturnControlCommand(SimpleState&, SimpleState&,typename RefOVoidStr<T>::Value, double)> func, std::string control_level){
        // setting_the control interface
    	if(control_level == "joint_pos"){
			this->control_level = control_level;
			this->ctrl_func_joint_pos = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions{
				//
				ReturnControlCommand ret;
				SimpleState st;
				franka::JointPositions ret_franka={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state,this->persistent_data,this->time);
				// convert from VectorXd to franka::JointPositions
				Eigen::VectorXd::Map(&ret_franka.q[0], 7) = ret.joint_space_command;
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
			};
    	}else if(control_level == "joint_vel"){
    		this->control_level = control_level;
    		this->ctrl_func_joint_vel = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities{
				//
				ReturnControlCommand ret;
				SimpleState st;
				franka::JointVelocities ret_franka={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state,this->persistent_data,this->time);
				// convert from VectorXd to franka::JointVelocities
				Eigen::VectorXd::Map(&ret_franka.dq[0], 7) = ret.joint_space_command;
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
			};

    	}else if(control_level == "torque"){
    		this->control_level = control_level;
			this->ctrl_func_tau = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques{
				//
				ReturnControlCommand ret;
				SimpleState st;
				franka::Torques ret_franka={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state,this->persistent_data,this->time);
				// convert from VectorXd to franka::JointVelocities
				Eigen::VectorXd::Map(&ret_franka.tau_J[0], 7) = ret.joint_space_command;
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
			};

    	}else if(control_level == "cartesian_pose"){
    		this->control_level = control_level;
			this->ctrl_func_cart_pose = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose{
				//
				ReturnControlCommand ret;
				SimpleState st;
				std::array<double, 16> ret_franka;
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state,this->persistent_data,this->time);
				// convert from Eigen::Affine3 to franka::JointVelocities
				std::copy_n(ret.cartesian_pose_command.data(), ret_franka.size(), ret_franka.begin());
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
			};

    	}else if(control_level == "cartesian_vel"){
    		this->control_level = control_level;
			this->ctrl_func_cart_vel = [this, func](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities{
				//
				ReturnControlCommand ret;
				SimpleState st;
				std::array<double, 6> ret_franka;
				// updating the dynamical information of the robot
				if(updateFlags>0)
					this->updateVectors(robot_state);
				// updating the time
				this->time = this->time + period.toSec();
				// Adapt the simple state
				this->state2SimpleState(robot_state,st);
				// control logic from outside
				ret=func(st, this->init_state,this->persistent_data,this->time);
				// convert from Eigen::Affine3 to franka::JointVelocities
				std::copy_n(ret.cartesian_vel_command.data(), ret_franka.size(), ret_franka.begin());
				if (!ret.running_controller) {
					std::cout << "Finished motion" << std::endl;
					// resetting internal control time to zero
					this->time = 0.0;
					return franka::MotionFinished(ret_franka);
				}else{
					return ret_franka;
				}
            };
    	}else{
    		std::cout << "RobotInterface<T>::setCtrl:: control level non existent, control levels allowed are: "
            << "joint_pos, joint_vel, torque, cartesian_pose, cartesian_vel. " 
            << "Check for typos: " << control_level << ";" << std::endl;
    	}

    }
    
    
    template <class T>
    void RobotInterface<T>::run(){
		this->time = 0.0;
        //todo adding while for dealing with error recover
    	if(this->control_level=="torque"){
            std::cout << "Running with torque control" << std::endl;
    		this->_robot->control(this->ctrl_func_tau);
    	}else if(this->control_level=="joint_vel"){
            std::cout << "Running with joint_vel control" << std::endl;
    		this->_robot->control(this->ctrl_func_joint_vel);
    	}else if(this->control_level=="joint_pos"){
            std::cout << "Running with joint_pos control" << std::endl;
    		this->_robot->control(this->ctrl_func_joint_pos);
    	}else if(this->control_level=="cartesian_pose"){
            std::cout << "Running with cartesian_pose control" << std::endl;
    		// kept for reference as an example of binding std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)> ctrl_func = std::bind(&franka_interface::RobotInterface::sendCartesianPoses, this, std::placeholders::_1, std::placeholders::_2);
    	    this->_robot->control(this->ctrl_func_cart_pose);
    	}else if(this->control_level=="cartesian_vel"){
            std::cout << "Running with cartesian_vel control" << std::endl;
            this->_robot->control(this->ctrl_func_cart_vel);
    	}else{
			std::cout << "RobotInterface<T>::run:: control level non existent, control levels allowed are:"
            << " joint_pos, joint_vel, torque, cartesian_pose, cartesian_vel. " 
            << "Check for typos: "<< this->control_level  << ";" << std::endl;
		}
    }
    template <class T>
    void RobotInterface<T>::runDetach(){
    	this->sendControl = std::thread{&RobotInterface::run, this};
    }


	// Method to update the mass matrix
	template <class T>
	void RobotInterface<T>::updateMassMatrix(const franka::RobotState& robot_state) {
		auto mass_array = this->_model->mass(robot_state);
		for (int i = 0; i < 7; ++i) {
			for (int j = 0; j < 7; ++j) {
				this->mass_matrix(i, j) = mass_array[i * 7 + j];
			}
		}
	}
	template <class T>
	void RobotInterface<T>::updateCoriolisVector(const franka::RobotState& robot_state) {
        auto coriolis_array = this->_model->coriolis(robot_state); // Assuming model is a franka::Model instance
        // Convert std::array to Eigen::VectorXd
        for (size_t i = 0; i < coriolis_array.size(); ++i) {
            this->coriolis_vector(i) = coriolis_array[i];
        }
    }
	template <class T>
	// Updates the internal gravity vector based on the given robot state
    void RobotInterface<T>::updateGravityVector(const franka::RobotState& robot_state) {
        auto gravity_array = this->_model->gravity(robot_state); // Assuming model is a franka::Model instance
        // Convert std::array to Eigen::VectorXd
        for (size_t i = 0; i < gravity_array.size(); ++i) {
            this->gravity_vector(i) = gravity_array[i];
        }
    }

	// Computes the end-effector zero Jacobian
	template <class T>
    Eigen::VectorXd RobotInterface<T>::getEndEffectorZeroJacobian() const {
        auto jacobian = this->_model->zeroJacobian(franka::Frame::kEndEffector, this->_robot->readOnce()); // Assuming model is a franka::Model instance
        return Eigen::VectorXd::Map(jacobian.data(), jacobian.size());
    }
	template <class T>
    Eigen::VectorXd RobotInterface<T>::getEndEffectorZeroJacobianFromState(const franka::RobotState& robot_state) const {
        // TODO: Remove this->_model and replace with something belonging to robot_state
        auto jacobian = this->_model->zeroJacobian(franka::Frame::kEndEffector, robot_state); // Assuming model is a franka::Model instance
        return Eigen::VectorXd::Map(jacobian.data(), jacobian.size());
    }
    
	// Performs updates of the dynamical information of the robot based on the current flags and given robot state
	template <class T>
    void RobotInterface<T>::updateVectors(const franka::RobotState& robot_state) {
        if (updateFlags & UpdateCoriolis) {
			//std::cout<<"updating coriolis"<<std::endl;
            updateCoriolisVector(robot_state);
        }
        if (updateFlags & UpdateGravity) {
			//std::cout<<"updating gravity"<<std::endl;
            updateGravityVector(robot_state);
        }
        if (updateFlags & UpdateMass) {
			//std::cout<<"updating mass"<<std::endl;
            updateMassMatrix(robot_state);
        }
    }

    // // Returns the state of the robot arm
    // template <class T>
    // franka::RobotState RobotInterface<franka::Model>::readOnce(){
    // 	return this->_robot->readOnce();
    // }






