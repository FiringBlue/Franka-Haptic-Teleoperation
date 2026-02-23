//============================================================================
// Name        : dummy_follower.cpp
// Author      : Wenhao Zhao (wenhao.zhao@tum.de)
// Version     : Feb 2026
// Description : dummy follower node for teleoperation between force dimensiion haptic devices <-> franka arm 
//============================================================================

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <algorithm>
#include <cmath>
#include <mutex>

class DummyFollower {
public:
  DummyFollower(ros::NodeHandle& nh)
  : nh_(nh) {
    // Params (can be set via rosparam)
    nh_.param("k", k_, 3.0);                 // gain: N per (m/s) if you treat it that way
    nh_.param("max_force", max_force_, 2.0); // clamp each axis (absolute)
    nh_.param("rate_hz", rate_hz_, 200);     // publish rate

    sub_ = nh_.subscribe("/cartesian_impedance_example_controller/LFcommand",
                         1, &DummyFollower::cmdCallback, this,
                         ros::TransportHints().tcpNoDelay(true));

    pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
        "/cartesian_impedance_example_controller/FLfeedback", 1, false);

    last_cmd_.assign(3, 0.0);

    ROS_INFO("DummyFollower started. Sub: LFcommand, Pub: FLfeedback");
    ROS_INFO("Params: k=%.3f, max_force=%.3f, rate_hz=%d", k_, max_force_, rate_hz_);
  }

  void spin() {
    ros::Rate r(rate_hz_);
    while (ros::ok()) {
      std::vector<double> cmd;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        cmd = last_cmd_;
      }

      // Compute fake feedback force: F = -k * v
      std_msgs::Float64MultiArray msg;
      msg.data.resize(3, 0.0);

      for (int i = 0; i < 3; ++i) {
        double f = -k_ * cmd[i];

        // Clamp per-axis to keep it safe
        f = std::max(-max_force_, std::min(max_force_, f));
        msg.data[i] = f;
      }

      pub_.publish(msg);

      ros::spinOnce();
      r.sleep();
    }
  }

private:
  void cmdCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 3) {
      ROS_WARN_THROTTLE(1.0, "LFcommand size < 3, ignoring");
      return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    last_cmd_[0] = msg->data[0];
    last_cmd_[1] = msg->data[1];
    last_cmd_[2] = msg->data[2];
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::mutex mtx_;

  std::vector<double> last_cmd_;

  double k_;
  double max_force_;
  int rate_hz_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dummy_follower");
  ros::NodeHandle nh("~");

  DummyFollower node(nh);
  node.spin();
  return 0;
}
