#pragma once

#include <memory>
#include <string>
#include <vector>
#include <pthread.h>
#include <time.h>
#include <queue>
#include <std_msgs/Float64MultiArray.h>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>



namespace franka_teleop_lmt {

class CartesianImpedanceExampleController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
  void RecordSignals(const ros::TimerEvent&);  // Record function

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;


  double translational_stiffness_{150};
  double rotational_stiffness_{60};
  double nullspace_stiffness_{2.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;

  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;                  // desired position
  Eigen::Quaterniond orientation_d_;            // desired orientation

  std::vector<double> command_;     // received command from teleoeprator
  std::vector<double> feedback_;    // slave feedback to send to teleoperator
  std::queue<std::vector<double>>commandQ;   // Force feedback queue
  std::mutex commandMutex;

// To be sent to follower (feedback)
  double Xf[3];                 // follower position end-effector
  double Vf[3];                 // follower velocity end-effector
  double Fc[3];                 // follower control force end-effector
  double Fc_filtered[3];        // follower control force end-effector filtered
  double Fe[3];                 // estimated external force

// Robot publisher and subscriber
  ros::Subscriber *F_Sub;   // command subscriber from teleoeperator
  ros::Publisher *F_Pub;    // feedback publisher to teleoperator
  bool first_packet = 0;
  
// Data recording
  typedef struct {
    double time;
    bool first_packet;
    double Xf[3];
    double Vf[3];
    double Fc[3];
  } RecordData;
  FILE* fidRecord;
  std::queue<RecordData> RecordQueue;
  std::mutex recordMutex;
  ros::Timer TimerRecorder;           // Timer for recorder thread
  double TimerPeriodHaptic = 0.001;   // Comm. rate (1/T)
  double CurrentTime = 0;
};

}  // namespace franka_teleop_lmt
