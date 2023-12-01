#ifndef HIQP_CLIENT_HH
#define HIQP_CLIENT_HH

#include <hiqp_msgs/srv/activate_task.hpp>
#include <hiqp_msgs/srv/deactivate_task.hpp>
#include <hiqp_msgs/srv/monitor_task.hpp>
#include <hiqp_msgs/srv/remove_all_primitives.hpp>
#include <hiqp_msgs/srv/remove_all_tasks.hpp>
#include <hiqp_msgs/srv/remove_primitives.hpp>
#include <hiqp_msgs/srv/remove_tasks.hpp>
#include <hiqp_msgs/srv/set_primitives.hpp>
#include <hiqp_msgs/srv/set_tasks.hpp>
#include <hiqp_msgs/srv/get_all_primitives.hpp>
#include <hiqp_msgs/srv/get_all_tasks.hpp>
#include <hiqp_msgs/srv/is_task_set.hpp>

#include <hiqp_msgs/msg/task_measures.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>

#include <numeric>
#include <rclcpp/rclcpp.hpp>

namespace hiqp_ros {

enum TaskDoneReaction { NONE = 0, PRINT_INFO = 1, DEACTIVATE = 2, REMOVE = 3 };

class HiQPClient {
  std::string robot_namespace_;
  std::string controller_namespace_;
  std::string controller_node_name_;
  /**
   * A nodehandle for the controller namespace.
   *
   */
  rclcpp::Node::SharedPtr nh_;

	/**
   * A client to the get_all_primitives service.
   */
  rclcpp::Client<hiqp_msgs::srv::GetAllPrimitives>::SharedPtr get_all_primitives_client_;

  /**
   * A client to the get_all_tasks service.
   */
  rclcpp::Client<hiqp_msgs::srv::GetAllTasks>::SharedPtr get_all_tasks_client_;
  
  /**
   * A client to the set_primitives service.
   */
  rclcpp::Client<hiqp_msgs::srv::SetPrimitives>::SharedPtr set_primitives_client_;

  /**
   * A client to the set_tasks service.
   */
  rclcpp::Client<hiqp_msgs::srv::SetTasks>::SharedPtr set_tasks_client_;

  /**
   * A client to the activate_task service.
   */
  rclcpp::Client<hiqp_msgs::srv::ActivateTask>::SharedPtr activate_task_client_;

  /**
   * A client to the deactivate_task service.
   */
  rclcpp::Client<hiqp_msgs::srv::DeactivateTask>::SharedPtr deactivate_task_client_;

  /**
   * A client to the remove_tasks service.
   */
  rclcpp::Client<hiqp_msgs::srv::RemoveTasks>::SharedPtr remove_tasks_client_;

  /**
   * A client to the remove_primitive service.
   */
  rclcpp::Client<hiqp_msgs::srv::RemovePrimitives>::SharedPtr remove_primitives_client_;

  rclcpp::Client<hiqp_msgs::srv::RemoveAllTasks>::SharedPtr remove_all_tasks_client_;

  rclcpp::Client<hiqp_msgs::srv::RemoveAllPrimitives>::SharedPtr remove_all_primitives_client_;
  
  rclcpp::Client<hiqp_msgs::srv::IsTaskSet>::SharedPtr is_task_set_client_;

  rclcpp::Subscription<hiqp_msgs::msg::TaskMeasures>::SharedPtr task_measures_sub_;

  std::mutex resource_mutex_;

  std::map<std::string, double> task_name_sq_error_map_;
  std::map<std::string, int> task_name_task_sign_map_;

  /**
   * A callback function for monitoring the task.
   *
   * @param task_measures A ConstPtr to message containing a vector of task
   * measures.
   */
  void taskMeasuresCallback(
      const hiqp_msgs::msg::TaskMeasures::SharedPtr task_measures);

  /** execution stuff */
  bool running_;
  std::shared_ptr<std::thread> async_spinner_;
  std::promise<void> stop_async_spinner_;
  
 public:
  HiQPClient(const std::string& robot_namespace,
             const std::string& controller_namespace =
                 "hiqp_joint_velocity_controller",
             bool auto_connect = true);

  void connectToServer();
  
  /**
   * Call the set_primitive service on the hiqp controller.
   *
   * @param name The name of the primitive instance.
   * @param type The type of primitive.
   * @param frame_id The frame_id in which to add the primitive.
   * @param visible Should this be visible in rviz?
   * @param color If visible is set to true, color and alpha values.
   * @param parameters Parameters for the primitive.
   * @return success or failure
   */
  bool setPrimitive(const std::string& name, const std::string& type,
                    const std::string& frame_id, bool visible,
                    const std::vector<double>& color,
                    const std::vector<double>& parameters);

  bool setPrimitives(const std::vector<hiqp_msgs::msg::Primitive>& primitives);

  /**
   * Call the set_task service on the hiqp controller.
   *
   * @param name The name of the task instance.
   * @param priority Priority level.
   * @param visible Visibility.
   * @param active Initial state of the task.
   * @param monitored Is it monitored.
   * @param def_params Task name and list of strings that are passed to the init
   * fn.
   * @param dyn_params Name of the dynamics and the parameters.
   * @return success or failure
   */
  bool setTask(const std::string& name, int16_t priority, bool visible,
               bool active, bool monitored,
               const std::vector<std::string>& def_params,
               const std::vector<std::string>& dyn_params);

  bool setTasks(const std::vector<hiqp_msgs::msg::Task>& tasks);

  bool deactivateTask(const std::string& task_name);

  void activateTasks(const std::vector<std::string>& task_names);

  bool removeTask(const std::string& task_name);
  
  bool removeTasks(const std::vector <std::string>& task_names);

  bool removePrimitive(const std::string& primitive_name);
  
  bool removePrimitives(const std::vector <std::string>& primitive_names);

  bool removeAllTasks();

  bool removeAllPrimitives();
  
  std::vector<hiqp_msgs::msg::Primitive> getAllPrimitives();

  std::vector<hiqp_msgs::msg::Task> getAllTasks();

  bool resetHiQPController();

  bool setJointAngles(const std::vector<double>& joint_angles, bool remove = true, double tol=1e-2);

  /**
   * Wait for a group of tasks to be completed, and then unload them all.
   *
   * @param task_names
   */
  void waitForCompletion(const std::vector<std::string>& task_names,
                         const std::vector<TaskDoneReaction>& reactions,
                         const std::vector<double>& error_tol,
                         double max_exec_time=0);
                         
  bool isTaskSet(const std::string& task_name);


  /** spins the nodes in a new thread, non-blocking call */
  void run();
  /** signals the spin thread to stop */
  void quit();

  //special group to actually do multithreading
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  template<class T> 
  bool blocking_call(std::shared_ptr<rclcpp::Client<T> > &client, 
      std::shared_ptr<typename T::Request> &request, std::shared_ptr<typename T::Response> &response);
  
  //returns node handle, can be used to add more to the client callback groups
  rclcpp::Node::SharedPtr getHandle() { return nh_; }
};

hiqp_msgs::msg::Task createTaskMsg(const std::string& name, int16_t priority,
                              bool visible, bool active, bool monitored,
                              const std::vector<std::string>& def_params,
                              const std::vector<std::string>& dyn_params);

hiqp_msgs::msg::Primitive createPrimitiveMsg(const std::string& name,
                                        const std::string& type,
                                        const std::string& frame_id,
                                        bool visible,
                                        const std::vector<double>& color,
                                        const std::vector<double>& parameters);

/** as of yet untested blocking call to a service.
  * requires pointer to client and a formed request,
  * result goes in response
  * requires that someone is spinning the node on which the client is created from outside
  * typically, we want an async spinner and to register nh_ to that
  */
template<class T> 
bool HiQPClient::blocking_call(std::shared_ptr<rclcpp::Client<T> > &client, 
      std::shared_ptr<typename T::Request> &request, 
      std::shared_ptr<typename T::Response> &response)
{
  using namespace std::chrono_literals;
  //wait for service to exist
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(nh_->get_logger(), "service not available, waiting...");
  }
  
  std::mutex res_mutex_;              //mutex variable to wait for response on service
  std::condition_variable res_cv_;    //condition variable associated with service rsponse

  //lambda magic to unblock once response is available 
  auto callback =  [&res_cv_,&response](typename rclcpp::Client<T>::SharedFutureWithRequest future) 
  {
    //std::cerr<<"Result callback\n";
    response = future.get().second;
    res_cv_.notify_all();
  };

  {
    // Wait for the result.
    std::unique_lock<std::mutex> response_locker(res_mutex_);
    //std::cerr<<"Request sent\n";
    auto result = client->async_send_request(request, std::move(callback));
    //std::cerr<<"Waiting for result\n";
    auto endTime = std::chrono::system_clock::now() + std::chrono::seconds(5);
    auto res = res_cv_.wait_until(response_locker, endTime);
    if (res == std::cv_status::timeout)
    {
      RCLCPP_ERROR(nh_->get_logger(),"Service %s timed out waiting for response!",client->get_service_name());
      return false;
    }
  }
  
  return true;
}


}

#endif
