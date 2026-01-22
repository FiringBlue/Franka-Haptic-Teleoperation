#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/HomingAction.h>

class GripperToggleNode {
public:
  GripperToggleNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh),
    grasp_ac_(getActionName(pnh_, "grasp_action", "franka_gripper/grasp"), true),
    move_ac_( getActionName(pnh_, "move_action",  "franka_gripper/move"),  true),
    homing_ac_(getActionName(pnh_, "homing_action","franka_gripper/homing"),true)
  {
    pnh_.param<std::string>("toggle_topic", toggle_topic_, std::string("/teleop/gripper_toggle"));

    // grasp params
    pnh_.param<double>("grasp_width", grasp_width_, 0.0);
    pnh_.param<double>("grasp_speed", grasp_speed_, 0.1);
    pnh_.param<double>("grasp_force", grasp_force_, 20.0);
    pnh_.param<double>("eps_inner", eps_inner_, 0.005);
    pnh_.param<double>("eps_outer", eps_outer_, 0.005);

    // open params (Move)
    pnh_.param<double>("open_width", open_width_, 0.08);
    pnh_.param<double>("open_speed", open_speed_, 0.1);

    // homing
    pnh_.param<bool>("do_homing_on_start", do_homing_on_start_, false);

    sub_ = nh_.subscribe(toggle_topic_, 1, &GripperToggleNode::toggleCb, this);

    ROS_INFO("Waiting for gripper action servers...");
    grasp_ac_.waitForServer();
    move_ac_.waitForServer();
    homing_ac_.waitForServer();
    ROS_INFO("Gripper action servers are ready.");

    if (do_homing_on_start_) {
      franka_gripper::HomingGoal hg;
      homing_ac_.sendGoal(hg);
      homing_ac_.waitForResult(ros::Duration(10.0));
      ROS_INFO("Homing done (or timeout).");
    }
  }

private:
  static std::string getActionName(ros::NodeHandle& pnh,
                                  const std::string& param_key,
                                  const std::string& default_name) {
    std::string name;
    pnh.param<std::string>(param_key, name, default_name);
    return name;
  }

  void toggleCb(const std_msgs::BoolConstPtr& msg) {
    (void)msg; // event only

    if (!closed_) {
      // close (grasp)
      franka_gripper::GraspGoal goal;
      goal.width = grasp_width_;
      goal.epsilon.inner = eps_inner_;
      goal.epsilon.outer = eps_outer_;
      goal.speed = grasp_speed_;
      goal.force = grasp_force_;

      ROS_INFO("Sending GRASP: width=%.3f speed=%.3f force=%.1f", goal.width, goal.speed, goal.force);
      grasp_ac_.sendGoal(goal);
      // do NOT block; just toggle state
      closed_ = true;
    } else {
      // open (move)
      franka_gripper::MoveGoal goal;
      goal.width = open_width_;
      goal.speed = open_speed_;

      ROS_INFO("Sending MOVE(open): width=%.3f speed=%.3f", goal.width, goal.speed);
      move_ac_.sendGoal(goal);
      closed_ = false;
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;

  std::string toggle_topic_;

  // action clients (MUST be constructed in initializer list)
  actionlib::SimpleActionClient<franka_gripper::GraspAction>  grasp_ac_;
  actionlib::SimpleActionClient<franka_gripper::MoveAction>   move_ac_;
  actionlib::SimpleActionClient<franka_gripper::HomingAction> homing_ac_;

  // params
  double grasp_width_, grasp_speed_, grasp_force_, eps_inner_, eps_outer_;
  double open_width_, open_speed_;
  bool do_homing_on_start_;

  bool closed_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_toggle_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  GripperToggleNode node(nh, pnh);
  ros::spin();
  return 0;
}
