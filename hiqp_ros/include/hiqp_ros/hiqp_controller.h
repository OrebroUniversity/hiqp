// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.h>

//#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
//#include <hardware_interface/force_torque_sensor_interface.h>
//#include <hardware_interface/robot_hw.h>

#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <hiqp/robot_state.h>
#include <hiqp_msgs/msg/joint_controller_state.hpp>
#include <hiqp_ros/utilities.h>
#include <hiqp_ros/ros_topic_subscriber.h>
#include <hiqp_ros/ros_visualizer.h>
#include <hiqp_ros/hiqp_service_handler.h>

//auto-gernerated by generate_parameters library
#include "hiqp_parameters.hpp"

namespace hiqp_ros {

  using hiqp::HiQPTimePoint;
  using hiqp::RobotState;
  using hiqp::RobotStatePtr;

  /*! \brief A major re-write of the old controller for compatibility with ros2 control.
   *  Thanks to the changes in ros2 control, we no longer need to specialize into separate 
   *  classes for different interface types. This should be handled by the interface type 
   *  specified in the config files and urdf. Previous code is thus merged into one controller
   *  and appropriate changes are made to talk to resource manager.   
   *  \author Todor Stoyanov */
  class HiqpController
    : public controller_interface::ControllerInterface {
      public:
        HiqpController()
          : controller_interface::ControllerInterface(),
          is_active_(true),
          monitoring_active_(false),
          period_(0,0),
          visualizer_(new ROSVisualizer()),
          task_manager_ptr_(new hiqp::TaskManager(visualizer_)) {};
        ~HiqpController() noexcept = default;

        //ROS2 control requires that we implement these functions:
        //called during configuration of command interfaces
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        //called during configuration of state interfaces
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        //calls init on base class
        controller_interface::CallbackReturn on_init() override;
        //configure parameters
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        //clears state and will start control after this
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;
        //clean-up back to a state from which we can start the controller
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        //called once every cycle to update --> use realtime tools within this
        controller_interface::return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        /*! \brief Implement this to compute the desired joint accelerations ddq and
         * corresponding joint controls u (e.g., via integration of ddq). Do not
         * resize ddq! */
        virtual void updateControls(Eigen::VectorXd &ddq, Eigen::VectorXd &u);

      protected:

        // Parameters from ROS for joint_trajectory_controller
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        //inline rclcpp::Node::SharedPtr getControllerNodeHandle() { return controller_nh_; }
        inline unsigned int getNJoints() { return n_joints_; }
        inline RobotStatePtr getRobotState() { return robot_state_ptr_; }

        rclcpp::Duration period_;

        //sets up the urdf model into a kdl tree
        int loadUrdfToKdlTree();

        //These should not be neded anymore, we use resource manager
        //int loadJointsAndSetJointHandlesMap();
        //int loadSensorsAndSetSensorHandlesMap();
        
        void readState();
        void setControls();
        void publishControllerState();

        RobotStatePtr robot_state_ptr_;
        HiQPTimePoint last_sampling_time_point_;
        double desired_sampling_time_;
        Eigen::VectorXd ddq_, u_, u_vel_;

        //use this->get_node() instead!
        //rclcpp::Node::SharedPtr controller_nh_;

        //I don't think this is needed either
        //std::mutex handles_mutex_;
        unsigned int n_joints_;
        unsigned int n_sensors_;  

        //New way of dealing with hardware interfaces
        template<typename T>
          using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
        InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
        InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

         // Storing command joint names for interfaces
        std::vector<std::string> command_joint_names_;


        realtime_tools::RealtimePublisher<hiqp_msgs::msg::JointControllerState>
          c_state_pub_;
        rclcpp::Time last_c_state_update_;
        double c_state_publish_rate_;

        //controller can work with the following hardware interfaces
        const std::vector<std::string> allowed_interface_types_ = {
          hardware_interface::HW_IF_VELOCITY
         // hardware_interface::HW_IF_ACCELERATION,
         // hardware_interface::HW_IF_EFFORT,
        };

        HiqpController(const HiqpController &other) = delete;
        HiqpController(HiqpController &&other) = delete;
        HiqpController &operator=(const HiqpController &other) = delete;
        HiqpController &operator=(HiqpController &&other) noexcept = delete;
      private:

        void monitorTasks(double vel_ctl_comp_time);
        void renderPrimitives();

        void loadRenderingParameters();
        int loadAndSetupTaskMonitoring();
        void addTfTopicSubscriptions();
        void loadJointLimitsFromParamServer();
        void loadGeometricPrimitivesFromParamServer();
        void loadTasksFromParamServer();

        bool is_active_;
        bool monitoring_active_;
        double monitoring_publish_rate_;
        rclcpp::Time last_monitoring_update_;

        double rendering_publish_rate_;
        rclcpp::Time last_rendering_update_;

        //ros::Publisher monitoring_pub_;

        ROSTopicSubscriber topic_subscriber_;

        HiQPServiceHandler service_handler_;  // takes care of all ros service calls

        std::shared_ptr<Visualizer> visualizer_;
        std::shared_ptr<hiqp::TaskManager> task_manager_ptr_;
    };


} // namespace force_controllers

