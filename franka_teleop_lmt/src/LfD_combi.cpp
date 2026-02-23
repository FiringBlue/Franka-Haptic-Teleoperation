//============================================================================
// Name        : LfD_combi.cpp
// Author      : Wenhao Zhao (wenhao.zhao@tum.de)
// Version     : Feb 2026
// Description : LfD_combi Reproduction node
//============================================================================

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <ros/package.h>

static std::vector<double> parseLine(std::string line) {
  std::replace(line.begin(), line.end(), ',', ' ');
  std::replace(line.begin(), line.end(), ';', ' ');
  std::stringstream ss(line);
  std::vector<double> v;
  double x;
  while (ss >> x) v.push_back(x);
  return v;
}

// Load 3xN CSV
static bool load3xN(const std::string& path,
                    std::vector<double>& x,
                    std::vector<double>& y,
                    std::vector<double>& z) {
  std::ifstream f(path.c_str());
  if (!f.is_open()) {
    ROS_ERROR("Cannot open: %s", path.c_str());
    return false;
  }

  std::string l1,l2,l3;
  if (!std::getline(f,l1) || !std::getline(f,l2) || !std::getline(f,l3)) {
    ROS_ERROR("Need 3 lines (x,y,z): %s", path.c_str());
    return false;
  }

  x = parseLine(l1);
  y = parseLine(l2);
  z = parseLine(l3);

  size_t N = std::min(x.size(), std::min(y.size(), z.size()));
  x.resize(N); y.resize(N); z.resize(N);

  if (N < 2) {
    ROS_ERROR("Too few samples in %s", path.c_str());
    return false;
  }

  ROS_WARN("Loaded %s | N=%zu", path.c_str(), N);
  return true;
}

static void runPart(const std::vector<double>& x,
                    const std::vector<double>& y,
                    const std::vector<double>& z,
                    double dt,
                    double z_scale,
                    ros::Publisher& vel_pub,
                    ros::Rate& rate,
                    ros::Publisher* grip_pub = nullptr,
                    int release_sample = -1,
                    ros::Publisher* pitch_pub = nullptr,
                    double pitch_rate = 0.0) {

  std_msgs::Float64MultiArray msg;
  msg.data.resize(3);

  bool release_sent = false;

  for (size_t k = 1; ros::ok() && k < x.size(); ++k) {

    double vx = (x[k]-x[k-1]) / dt;
    double vy = (y[k]-y[k-1]) / dt;
    double vz = (z[k]-z[k-1]) / dt;

    vz *= z_scale;

    msg.data[0] = vx;
    msg.data[1] = vy;
    msg.data[2] = vz;

    vel_pub.publish(msg);

    // ---- SIMPLE RELEASE ----
    if (grip_pub && !release_sent && release_sample > 0) {
      if ((int)k == release_sample) {
        std_msgs::Bool g;
        g.data = true;
        grip_pub->publish(g);
        release_sent = true;

        ROS_WARN("Release triggered at sample %d", release_sample);
      }
    }

    // ---- pitch rotation (only for part3) ----
    if (pitch_pub) {
      std_msgs::Float64 ymsg;
      ymsg.data = pitch_rate;
      pitch_pub->publish(ymsg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  msg.data = {0.0,0.0,0.0};
  vel_pub.publish(msg);

  // stop pitch
  if (pitch_pub) {
    std_msgs::Float64 ymsg;
    ymsg.data = 0.0;
    pitch_pub->publish(ymsg);
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "LfD_combi");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string package_path = ros::package::getPath("franka_teleop_lmt");

  // Topics
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float64MultiArray>(
      "/cartesian_impedance_example_controller/LFcommand",1,false);

  ros::Publisher grip_pub = nh.advertise<std_msgs::Bool>(
      "/teleop/gripper_toggle",1,false);
  
  ros::Publisher pitch_pub = nh.advertise<std_msgs::Float64>(
      "/cartesian_impedance_example_controller/pitch_rate_cmd", 1, false);

  // Paths
  std::string base_dir;
  pnh.param<std::string>("base_dir", base_dir,
      package_path + "/TeleopData/SegmentDemo_learned/");

  std::string part1_csv, part2_csv, part3_csv;
  pnh.param<std::string>("part1_csv", part1_csv, "");  
  pnh.param<std::string>("part2_csv", part2_csv, "");  // default empty
  pnh.param<std::string>("part3_csv", part3_csv, "");  // default empty

  // Timing
  double dt; pnh.param<double>("dt", dt, 0.001);
  int loop_hz; pnh.param<int>("loop_hz", loop_hz, 1000);
  double grasp_pause_s; pnh.param<double>("grasp_pause_s", grasp_pause_s, 1.0);

  // Z scales
  double z1,z2,z3;
  pnh.param<double>("z_scale_part1", z1, 0.35);
  pnh.param<double>("z_scale_part2", z2, 0.60);
  pnh.param<double>("z_scale_part3", z3, 1.20);

  ros::Rate rate(loop_hz);
  ros::Duration(2.0).sleep();

  // Load part1
  std::vector<double> x1,y1,zv1;
  if(!load3xN(base_dir+part1_csv,x1,y1,zv1)) return 1;

  ROS_WARN("=== Run Part1 ===");
  runPart(x1,y1,zv1,dt,z1,vel_pub,rate,&grip_pub);

  // If no part2 -> single CSV test mode
  if(part2_csv.empty()) {
    ROS_WARN("Single CSV mode finished.");
    return 0;
  }

  // Send gripper once
  std_msgs::Bool g;
  g.data=true;
  grip_pub.publish(g);
  ROS_WARN("Gripper toggle sent ONCE. Pause %.1f s...",grasp_pause_s);
  ros::Duration(grasp_pause_s).sleep();

  // Run part2
  std::vector<double> x2,y2,zv2;
  if(!load3xN(base_dir+part2_csv,x2,y2,zv2)) return 1;

  ROS_WARN("=== Run Part2 ===");
  runPart(x2,y2,zv2,dt,z2,vel_pub,rate);

  // If no part3 -> stop here
  if(part3_csv.empty()) {
    ROS_WARN("Two-part mode finished.");
    return 0;
  }

  // Run part3 (throw)
  std::vector<double> x3,y3,zv3;
  if(!load3xN(base_dir+part3_csv,x3,y3,zv3)) return 1;
  int release_sample;
  pnh.param<int>("release_sample", release_sample, 100);
  double pitch_rate;
  pnh.param<double>("pitch_rate", pitch_rate, -1.0);

  ROS_WARN("=== Run Part3 (Throw) ===");
  runPart(x3,y3,zv3,
          dt,z3,
          vel_pub,rate,
          &grip_pub,release_sample,
          &pitch_pub, pitch_rate);

  ROS_WARN("All parts finished.");
  return 0;
}
