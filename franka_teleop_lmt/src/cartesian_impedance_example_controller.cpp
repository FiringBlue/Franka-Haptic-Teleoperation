//============================================================================
// Name        : cartesian_impedance_example_controller.cpp
// Author      : Basak Gülecyüz (basak.guelecyuez@tum.de)
// Version     : May 2023
// Description : Cartesian Impedance Controller for teleoperation 
//============================================================================
#include <franka_teleop_lmt/cartesian_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_teleop_lmt/pseudo_inversion.h>

namespace franka_teleop_lmt {

bool CartesianImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
		ros::NodeHandle& node_handle) {

	//////////////////////////////////////////////
	//////////// FrankaArmInit //////////////////
	//////////////////////////////////////////////
	std::string arm_id;
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
		return false;
	}
	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR(
				"CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
				"aborting controller init!");
		return false;
	}


	//////////////////////////////////////////////
	//////////// FrankaModelInterface ////////////
	//////////////////////////////////////////////
	auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
		ROS_ERROR_STREAM(
				"CartesianImpedanceExampleController: Error getting model interface from hardware");
		return false;
	}
	try {
		model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
				model_interface->getHandle(arm_id + "_model"));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"CartesianImpedanceExampleController: Exception getting model handle from interface: "
				<< ex.what());
		return false;
	}


	//////////////////////////////////////////////
	//////////// FrankaStateInterface ////////////
	//////////////////////////////////////////////
	auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
		ROS_ERROR_STREAM(
				"CartesianImpedanceExampleController: Error getting state interface from hardware");
		return false;
	}
	try {
		state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
				state_interface->getHandle(arm_id + "_robot"));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
				"CartesianImpedanceExampleController: Exception getting state handle from interface: "
				<< ex.what());
		return false;
	}


	//////////////////////////////////////////////
	//////////// EffortJointInterface ////////////
	//////////////////////////////////////////////
	auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		ROS_ERROR_STREAM(
				"CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
		return false;
	}
	for (size_t i = 0; i < 7; ++i) {
		try {
			joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			ROS_ERROR_STREAM(
					"CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
			return false;
		}
	}


	///////////////////////////////////////////////////////
	// Subscribe to leader commands 
	///////////////////////////////////////////////////////
	F_Sub = new ros::Subscriber;
	*F_Sub = node_handle.subscribe<std_msgs::Float64MultiArray>("LFcommand", 1, &CartesianImpedanceExampleController::setCommandCallback, this,
			ros::TransportHints().reliable().tcpNoDelay());


	///////////////////////////////////////////////////////
	// Publish follower force feedback 
	///////////////////////////////////////////////////////
	F_Pub = new ros::Publisher;
	*F_Pub = node_handle.advertise<std_msgs::Float64MultiArray>("FLfeedback", 1,true);


	///////////////////////////////////////////////////////
	// Initialize Record Thread 
    ///////////////////////////////////////////////////////
	std::string package_path = ros::package::getPath("franka_teleop_lmt");
	int demo_num;
	ros::param::get("demo_num", demo_num);
	std::stringstream ss_demo_num;
	ss_demo_num << demo_num;
	std::string record_path =  package_path + "/TeleopData/follower_" + ss_demo_num.str() + ".txt";
	fidRecord = fopen(record_path.c_str(), "w");
	if (fidRecord== NULL) {
		ROS_INFO("Failed: ");
		return 1;
	}
	fprintf(fidRecord, "time\t");                	// timestamp
	fprintf(fidRecord, "Vf_x\tVf_y\tVf_z\t");    	// Leader Velocity
	fprintf(fidRecord, "Xf_x\tXf_y\tXf_z\t");    	// Leader Position
	fprintf(fidRecord, "Fc_x\tFc_y\tFc_z\t");  		// Leader Force
		fprintf(fidRecord, "first_packet\n"); 		// first packet received
	fclose(fidRecord);
	TimerRecorder = node_handle.createTimer(ros::Duration(TimerPeriodHaptic),
									&CartesianImpedanceExampleController::RecordSignals, this);  // create a timer callback for data record

	///////////////////////////////////////////////////////
	// Initializations
	///////////////////////////////////////////////////////
	position_d_.setZero();
	orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

	cartesian_stiffness_.setIdentity();
	cartesian_stiffness_.topLeftCorner(3, 3)<< translational_stiffness_ * Eigen::Matrix3d::Identity();
	cartesian_stiffness_.bottomRightCorner(3, 3)<< rotational_stiffness_ * Eigen::Matrix3d::Identity();

	cartesian_damping_.setIdentity();
	cartesian_damping_.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness_) * Eigen::Matrix3d::Identity();
	cartesian_damping_.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness_) * Eigen::Matrix3d::Identity();

	return true;
}

void CartesianImpedanceExampleController::starting(const ros::Time& /*time*/) {


	// compute initial velocity with jacobian and set x_attractor and q_d_nullspace
	// to initial configuration
	franka::RobotState initial_state = state_handle_->getRobotState();

	// get jacobian
	std::array<double, 42> jacobian_array =
			model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

	// convert to eigen
	Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
	Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));


	// set equilibrium point to current state
	position_d_ = initial_transform.translation();
	orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

	// set nullspace equilibrium configuration to initial q
	q_d_nullspace_ = q_initial;

	// Set initial feedback to leader
	for (int i=1; i<3; i++){

		Xf[i] = position_d_[i];
		Fc[i] = 0;
	}

}

void CartesianImpedanceExampleController::update(const ros::Time& time,
		const ros::Duration& period) {

	/////////////////////////////////////////////////////////////////////////////
	// TO-DO: Read the desired cartesian velocity and set target position
	/////////////////////////////////////////////////////////////////////////////
	   
	if (commandMutex.try_lock()) {
		if (commandQ.size()) {
			
			// Get the front value from commandQ into variable command_.
        	// Pop from queue commandQ
			/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Your code here!
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
			command_ = commandQ.front();
			commandQ.pop();

		}
		commandMutex.unlock(); 
    }

	if(command_.size()==3){
		for (int i=0; i<3; i++){


			// Compute the position_d_ using the velocity commands
			/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Your code here!
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
			position_d_ = position_d_ + period.toSec() * Eigen::Vector3d(command_[0], command_[1], command_[2]);
			
		}
		// After reading command, clear the command
		command_.clear();
	}    


	// get state variables
	franka::RobotState robot_state = state_handle_->getRobotState();
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
	std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
	std::array<double, 49> Inertia_array = model_handle_->getMass();

	// convert to Eigen
	Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
	Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
	Eigen::Map<Eigen::Matrix<double, 7, 7>> Inertia(Inertia_array.data());
	Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

	// get end effector position and orientation
	Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	Eigen::Vector3d position(transform.translation());
	Eigen::Quaterniond orientation(transform.linear());

	// compute error to desired pose
	// position error
	Eigen::Matrix<double, 6, 1> error;
	error.head(3) << position - position_d_;

	// orientation error
	if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
		orientation.coeffs() << -orientation.coeffs();
	}
	// "difference" quaternion
	Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
	error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
	// Transform to base frame
	error.tail(3) << -transform.linear() * error.tail(3);

	// compute control
	// allocate variables
	Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_task_cartesian(6);

	// pseudoinverse for nullspace handling
	// kinematic pseuoinverse
	Eigen::MatrixXd jacobian_transpose_pinv;
	pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

	/////////////////////////////////////////////////////////////////////////////
	// TO-DO: Cartesian impedance controller
	/////////////////////////////////////////////////////////////////////////////

	// First compute tau_task_cartesian: Cartesian control force with impedance controller
	// Second compute tau_task in joint space using jacobian.transpose() and tau_task_cartesian

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Your code here!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
	tau_task_cartesian = (-cartesian_stiffness_ * error) - (cartesian_damping_ * (jacobian * dq));
	tau_task = jacobian.transpose() * tau_task_cartesian + coriolis;


	// nullspace PD control with damping ratio = 1
	tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
			jacobian.transpose() * jacobian_transpose_pinv) *
					(nullspace_stiffness_ * (q_d_nullspace_ - q) -
							(2.0 * sqrt(nullspace_stiffness_)) * dq);
	// Desired torque
	tau_d <<  tau_task + tau_nullspace + coriolis;

	// Saturate torque rate to avoid discontinuities
	tau_d << saturateTorqueRate(tau_d, tau_J_d);
	for (size_t i = 0; i < 7; ++i) {
		joint_handles_[i].setCommand(tau_d(i));
	}


	// Update feedback to teleoperator node
	robot_state = state_handle_->getRobotState();
	transform = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
	position = transform.translation();
	Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext_hat(robot_state.O_F_ext_hat_K.data());
	Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
	Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext_hat(robot_state.tau_ext_hat_filtered.data());

	// End effector Cartesian space feedback to teleoperator node
	for (int i=0; i<3; i++){
		Vf[i] = (position[i] - Xf[i]) / period.toSec();
		Xf[i] = position[i];
		Fc[i] = tau_task_cartesian[i];
		Fe[i] = F_ext_hat[i];
	}

    /////////////////////////////////////////////////////////
    // Record Signals for analysis
    /////////////////////////////////////////////////////////
    RecordData* tmp = new RecordData;  // temporary variable storing the signals

    // time
    tmp->time = CurrentTime;


    // Leader states
    for (int k = 0; k < 3; k++) {
      tmp->Vf[k] = Vf[k];
      tmp->Xf[k] = Xf[k];
      tmp->Fc[k] = Fc[k];
  
    }

	// First packet received
	tmp->first_packet = first_packet;
    
    // Push to record queue
    if (recordMutex.try_lock()) {
      RecordQueue.push(*tmp);
      delete tmp;
      //ROS_INFO("Controller Mutex Lock Record Queue 1! ");
      recordMutex.unlock(); 
    }

    /////////////////////////////////////////////////////////
    // TO-DO: Publish force feedback to leader
    /////////////////////////////////////////////////////////

	// Feedback data
	feedback_.clear();
	copy(Fc, Fc+3, back_inserter(feedback_));
	// printf("Feedback Force: %f, %f, %f \n", feedback_[0], feedback_[1], feedback_[2]);

	std_msgs::Float64MultiArray feedback_msg;
	
	// Send feedback to leader
	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Your code here!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
	feedback_msg.data = feedback_;
	F_Pub->publish(feedback_msg);

	// Increment the current time
	CurrentTime = CurrentTime + TimerPeriodHaptic;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
		const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
		const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
	Eigen::Matrix<double, 7, 1> tau_d_saturated{};
	for (size_t i = 0; i < 7; i++) {
		double difference = tau_d_calculated[i] - tau_J_d[i];
		tau_d_saturated[i] =
				tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
	}
	return tau_d_saturated;
}



void CartesianImpedanceExampleController::setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg) {
	
	const std::lock_guard<std::mutex> lock(commandMutex);
	commandQ.push(msg->data);

	if(commandQ.size()>0){
		first_packet = 1;
	}
}

void CartesianImpedanceExampleController::RecordSignals(const ros::TimerEvent&) {
  std::string package_path = ros::package::getPath("franka_teleop_lmt");
  int demo_num;
  ros::param::get("demo_num", demo_num);
  std::stringstream ss_demo_num;
  ss_demo_num << demo_num;
  std::string record_path =  package_path + "/TeleopData/follower_" + ss_demo_num.str() + ".txt";
  fidRecord = fopen(record_path.c_str(), "a");
  if (fidRecord == NULL) {
    ROS_INFO("File error");
  }
  bool DataArrived = false;
  RecordData* tmp = new RecordData;

  const std::lock_guard<std::mutex> lock(recordMutex);
  //ROS_INFO("Controller Mutex Lock Record Queue 2! ");
  if (RecordQueue.size() > 0) {

    *tmp = RecordQueue.front();
    RecordQueue.pop();

    DataArrived = true;
  } else {
    DataArrived = false;
  }
  // pthread_mutex_unlock (&RecordSynch);
  if (DataArrived == true) {
    fprintf(fidRecord, "%f\t", tmp->time);
    fprintf(fidRecord, "%f\t%f\t%f\t", tmp->Vf[0], tmp->Vf[1], tmp->Vf[2]);
    fprintf(fidRecord, "%f\t%f\t%f\t", tmp->Xf[0], tmp->Xf[1], tmp->Xf[2]);
    fprintf(fidRecord, "%f\t%f\t%f\t", tmp->Fc[0], tmp->Fc[1], tmp->Fc[2]);
		fprintf(fidRecord, "%d\n", tmp->first_packet);
  }
  fclose(fidRecord);
}


}  // namespace franka_teleop_lmt

PLUGINLIB_EXPORT_CLASS(franka_teleop_lmt::CartesianImpedanceExampleController,
		controller_interface::ControllerBase)
