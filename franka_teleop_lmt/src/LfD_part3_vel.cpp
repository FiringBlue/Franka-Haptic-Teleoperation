//============================================================================
// Name        : LfD_part3_vel.cpp
// Description : Replay learned throw velocity primitive (3xN) at 1kHz
//============================================================================

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/package.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

static bool loadCsv3xN(const std::string& path,
                       std::vector<std::vector<double>>& data,
                       int& N_out) {
  std::ifstream in(path.c_str());
  if (!in.is_open()) return false;

  data.assign(3, std::vector<double>());
  std::string line;
  int row = 0;

  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (row >= 3) break;  // only first 3 rows

    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      if (cell.empty()) continue;
      try {
        data[row].push_back(std::stod(cell));
      } catch (...) {
        // ignore parse errors
      }
    }
    row++;
  }

  if (row < 3) return false;

  N_out = (int)std::min({data[0].size(), data[1].size(), data[2].size()});
  if (N_out < 2) return false;

  // truncate to same length
  for (int i = 0; i < 3; ++i) data[i].resize(N_out);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "LfD_part3_throw_vel");
  ros::NodeHandle nh;

  ros::Publisher pub =
      nh.advertise<std_msgs::Float64MultiArray>(
          "/cartesian_impedance_example_controller/LFcommand", 1, false);

  const std::string package_path = ros::package::getPath("franka_teleop_lmt");
  std::string csv_path = package_path + "/TeleopData/SegmentDemo_learned/learned_part3_vel.csv";

  // optional: allow override via rosparam or argv
  if (argc > 1) csv_path = argv[1];
  nh.param<std::string>("csv_path", csv_path, csv_path);

  std::vector<std::vector<double>> V;
  int N = 0;
  if (!loadCsv3xN(csv_path, V, N)) {
    ROS_ERROR_STREAM("Failed to load velocity CSV (3xN): " << csv_path);
    return 1;
  }

  // optional safety clamp (you can tune)
  double v_max = 0.8;  // m/s
  nh.param("v_max", v_max, v_max);

  ROS_INFO_STREAM("Loaded throw velocity: N=" << N << " from " << csv_path);
  ros::Duration(1.0).sleep();

  ros::Rate rate(1000);
  int k = 0;

  std_msgs::Float64MultiArray msg;
  msg.data.resize(3);

  while (ros::ok()) {
    if (k < N) {
      msg.data[0] = std::max(-v_max, std::min(v_max, V[0][k]));
      msg.data[1] = std::max(-v_max, std::min(v_max, V[1][k]));
      msg.data[2] = std::max(-v_max, std::min(v_max, V[2][k]));
    } else {
      msg.data[0] = msg.data[1] = msg.data[2] = 0.0;
    }

    pub.publish(msg);

    k++;
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
