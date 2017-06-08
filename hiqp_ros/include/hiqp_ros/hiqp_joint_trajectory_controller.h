#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <hiqp_ros/hiqp_client.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include <mutex>

namespace hiqp_ros {

class HiQPJointTrajectoryController {
 public:
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
      joint_trajectory_action_server_;
  HiQPJointTrajectoryController(ros::NodeHandle nh, const std::string& robot_namespace, const std::string& arm_namespace, hiqp_ros::HiQPClient& hiqp_client);

 protected:
  ros::NodeHandle nh_;
  std::string arm_namespace_;

  hiqp_ros::HiQPClient& hiqp_client_;
  static ros::Subscriber joint_state_sub_;
  static void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg);
  static std::vector<double> joint_state_;
  static std::vector<double> getJointState();
  static std::mutex joint_state_mutex_;

  void processGoal(
      const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
};
}
