#include <hiqp_ros/hiqp_joint_trajectory_controller.h>

namespace hiqp_ros {
std::vector<double> HiQPJointTrajectoryController::joint_state_;
ros::Subscriber HiQPJointTrajectoryController::joint_state_sub_;
std::mutex HiQPJointTrajectoryController::joint_state_mutex_;

HiQPJointTrajectoryController::HiQPJointTrajectoryController(ros::NodeHandle nh, const std::string& robot_namespace, const std::string& arm_namespace)
    : joint_trajectory_action_server_(
      nh, "hiqp_joint_trajectory_controller/follow_joint_trajectory",
          boost::bind(&HiQPJointTrajectoryController::processGoal, this, _1),
          false),
      hiqp_client_(robot_namespace, "hiqp_joint_velocity_controller"),
      arm_namespace_(arm_namespace)
{
  joint_state_sub_ = nh_.subscribe("/joint_states", 1, &HiQPJointTrajectoryController::jointStateCallback);
  joint_trajectory_action_server_.start();
}

void HiQPJointTrajectoryController::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg) {
  joint_state_mutex_.lock();
  joint_state_.clear();
  
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_1_l") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_2_l") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_7_l") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_3_l") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_4_l") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_5_l") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_6_l") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_1_r") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_2_r") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_7_r") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_3_r") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_4_r") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_5_r") - joint_state_msg->name.begin()]);
  joint_state_.push_back(joint_state_msg->position[std::find(joint_state_msg->name.begin(), joint_state_msg->name.end(), "yumi_joint_6_r") - joint_state_msg->name.begin()]);

  joint_state_mutex_.unlock();
}

std::vector<double> HiQPJointTrajectoryController::getJointState() {
  joint_state_mutex_.lock();
  return joint_state_;
  joint_state_mutex_.unlock();
}

void HiQPJointTrajectoryController::processGoal(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
  // TODO: this would only work for yumi.
  ROS_INFO("Received joint trajectory.");
  ROS_INFO("Processing using HiQP as tracking controller");
  std::cout << *goal;
  auto& goal_joint_names = goal->trajectory.joint_names;
  if(goal_joint_names.size() != 7) {
    ROS_INFO("HiQPJointTrajectoryController is only supported for yumi now. Joint angles must be exactly 7.");
    this->joint_trajectory_action_server_.setAborted();
    return;
  }

  if(goal->trajectory.points[0].positions.size() < 1) {
    ROS_INFO("Joint positions are only supported.");
    this->joint_trajectory_action_server_.setAborted();
    return;
  }

  std::vector<double> jointState = getJointState();
  for(auto trajectory_point : goal->trajectory.points) {
    if(!ros::ok()) {
      break;
    }
    std::vector<double> all_joints;
    if(arm_namespace_ == "left_arm") {
      //all_joints = trajectory_point.positions;

      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_1_l") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_2_l") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_7_l") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_3_l") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_4_l") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_5_l") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_6_l") - goal_joint_names.begin()]);
      
      for(size_t i = 7; i < 14; i++) {
        all_joints.push_back(jointState[i]);
      }
    }
    else /* (arm_namespace_ == "right_arm") */ {
      for(size_t i = 0; i < 7; i++) {
        all_joints.push_back(jointState[i]);
      }
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_1_r") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_2_r") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_7_r") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_3_r") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_4_r") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_5_r") - goal_joint_names.begin()]);
      all_joints.push_back(trajectory_point.positions[std::find(goal_joint_names.begin(), goal_joint_names.end(), "yumi_joint_6_r") - goal_joint_names.begin()]);
    }

    hiqp_client_.setJointAngles(all_joints, false);
  }
  joint_trajectory_action_server_.setSucceeded();
}

}

int main(int argn, char* args[]) {
  
  ros::init(argn, args, "hiqp_joint_trajectory_controller_node");
  ros::NodeHandle right_arm_nh("/right_arm");
  ros::NodeHandle left_arm_nh("/left_arm");
  hiqp_ros::HiQPJointTrajectoryController jt_left_controller(left_arm_nh, "yumi", "left_arm");
  hiqp_ros::HiQPJointTrajectoryController jt_right_controller(right_arm_nh, "yumi", "right_arm");

  ros::MultiThreadedSpinner m_t_spinner(4);
  m_t_spinner.spin();
  return 0;
}
