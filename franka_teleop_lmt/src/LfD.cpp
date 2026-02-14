//============================================================================
// Name        : LfD.cpp
// Author      : Basak Gülecyüz (basak.guelecyuez@tum.de)
//               Wenhao Zhao (wenhao.zhao@tum.de)
// Version     : Feb 2026
// Description : LfD Reproduction node
//============================================================================

// General
#include <assert.h>
#include <gtest/gtest.h>
#include <pthread.h>
#include <time.h>
#include <queue>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

// ROS
//#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <ros/package.h>


////////////////////////////////////////////////////////////////
// Declared Variables for ROS
////////////////////////////////////////////////////////////////
ros::NodeHandle* n;                 // LfD ROS Node
ros::Publisher* LfD_Pub;            // LfD publisher

////////////////////////////////////////////////////////////////
// Declared Variables for LfD Reproduction
////////////////////////////////////////////////////////////////

// Path to package
std::string package_path = ros::package::getPath("franka_teleop_lmt");


// Variables for learned trajectory
const int repro_duration = 40000; // ms
double TimerPeriodHaptic = 0.001;
double learned_trajectory[3][int(repro_duration)] = {0};
double Vl[3] = {0, 0, 0}; // learned velocity
double number_columns[3] = {0, 0, 0};

// Variables to be communicated to follower robot 
std::vector<double> command(3, 0.0);        



int main(int argc, char** argv) {

	////////////////////////////////////////////////////////////////
	// Initialize ROS
	////////////////////////////////////////////////////////////////

  	ros::init(argc, argv, "LfD");
  	n = new ros::NodeHandle;
	ros::NodeHandle pnh("~");
	
	// Leader publisher for commands
	LfD_Pub = new ros::Publisher;
	*LfD_Pub = n->advertise<std_msgs::Float64MultiArray>("/cartesian_impedance_example_controller/LFcommand", 1, false);

 
	// Load the learned trajectory to be reproduced
	std::string csv_name;
	pnh.param<std::string>("csv_name", csv_name, "learned_part1.csv");

	std::string LfD_path =  package_path + "/TeleopData/SegmentDemo_learned/" + csv_name; 
	std::ifstream iFile(LfD_path.c_str());
	std::string line;
	int dim_count = 0;
	int N = 0;
	while (getline(iFile, line))
	{
		std::stringstream ss(line);
		int time_count = 0;
		while (ss) {
			std::string line2;
			if (!getline(ss, line2, ','))
				break;
			std::stringstream sss(line2);
			double tm;
			sss >> tm;
			learned_trajectory[dim_count][time_count] = tm;
			++time_count;
		}
		number_columns[dim_count] = time_count;
		++dim_count;
		if(N < time_count){
			N = time_count;
		}
	}


	// auto z = learned_trajectory[2][time_counter];
	// auto z_prev = learned_trajectory[2][time_counter - 1];
	// auto vz = (z - z_prev) / TimerPeriodHaptic;

	// if (time_counter == 2 || time_counter == 50 || time_counter == 150 ||
	// 	time_counter == 200 || time_counter == 400 || time_counter == 800) {
	// ROS_WARN("IDX=%d  Zprev=%.6f  Z=%.6f  dZ=%.6f  Vz=%.6f",
	// 		time_counter, z_prev, z, (z - z_prev), vz);
	// }

	
	std::cout << "new0: " << Vl[0]
				<< "new1: " << Vl[1]
				<< "new2: " << Vl[2] << std::endl;


	// // after loading and computing N
	// std::string dump_path = package_path + "/TeleopData/z_dump_from_cpp.txt";
	// std::ofstream dump(dump_path);
	// dump << "k\tz\n";
	// for (int k = 0; k < N; ++k) {
	// dump << k << "\t" << learned_trajectory[2][k] << "\n";
	// }
	// dump.close();
	// ROS_WARN("Dumped Z read by C++ to: %s", dump_path.c_str());
	// const double V_MAX = 0.25; // or rosparam

	// Sleep for 5 seconds 
	ros::Duration(5.0).sleep();
	ROS_INFO("Publisher is set! ");

	//
	ros::Rate loop_rate(1000);
    int time_counter = 1;



	std_msgs::Float64MultiArray msg;
	while (ros::ok()) {

		// Vl is drawn from the learned model
		if(time_counter <= N -10){
			for (int i=0; i<3; i++){
				// compute velocity from position
				Vl[i] = (learned_trajectory[i][time_counter] - learned_trajectory[i][time_counter - 1]) / TimerPeriodHaptic;
				// Z dead-zone to avoid drift on plateau
				// if (std::abs(Vl[2]) < 0.02) {
				// 	Vl[2] = 0.0;
				// }
				// if(Vl[i]> V_MAX) Vl[i] = V_MAX;
				// if(Vl[i]< -V_MAX) Vl[i] = -V_MAX;
				// std::cout << "Vl[" << i << "]: " << Vl[i] << std::endl;
				// std::cout << "learned_trajectory[" << i << "][" << time_counter << "]: " << learned_trajectory[i][time_counter]
			}
		}
		else{
			for (int i=0; i<3; i++){
				Vl[i] = 0;
			}
		}

		Vl[2] *= 0.35; // optional scaling for Z velocity

		std::cout << " 0: " << Vl[0]
		<< " 1: " << Vl[1]
		<< " 2: " << Vl[2]
		<< " path_length: " << N
		<< std::endl;
		// Publish the command to follower robot 
		command.clear();
        command.insert(command.end(), Vl, Vl+3);

		msg.data = command;
		LfD_Pub->publish(msg);
		
		++time_counter;
		ros::spinOnce();
		loop_rate.sleep();

	}
  return 0;
}

