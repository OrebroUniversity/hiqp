#include <hiqp_ros/hiqp_client.h>
#include <memory>
#include <cassert>

#include <rclcpp/executor.hpp>

namespace hiqp_ros {

HiQPClient::HiQPClient(const std::string& robot_namespace,
                       const std::string& controller_namespace,
                       bool auto_connect)
    : robot_namespace_(robot_namespace),
      controller_namespace_(controller_namespace),
      running_(false) {
  controller_node_name_ = (robot_namespace == "") ? controller_namespace : (robot_namespace + "/" + controller_namespace);
  nh_ = std::make_shared<rclcpp::Node> ("hiqp_client");
  if (auto_connect) this->connectToServer();
}

void HiQPClient::connectToServer() {

  //callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto callback_group2 = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  //clients for talking to controller
  set_tasks_client_ = nh_->create_client<hiqp_msgs::srv::SetTasks>(controller_node_name_+"/set_tasks",rmw_qos_profile_services_default, callback_group_);
  set_tasks_client_->wait_for_service();

  set_primitives_client_ =
      nh_->create_client<hiqp_msgs::srv::SetPrimitives>(controller_node_name_+"/set_primitives",rmw_qos_profile_services_default, callback_group_);
  set_primitives_client_->wait_for_service();

  activate_task_client_ =
      nh_->create_client<hiqp_msgs::srv::ActivateTask>(controller_node_name_+"/activate_task",rmw_qos_profile_services_default, callback_group_);
  activate_task_client_->wait_for_service();

  deactivate_task_client_ =
      nh_->create_client<hiqp_msgs::srv::DeactivateTask>(controller_node_name_+"/deactivate_task",rmw_qos_profile_services_default, callback_group_);
  deactivate_task_client_->wait_for_service();

	get_all_primitives_client_ =
      nh_->create_client<hiqp_msgs::srv::GetAllPrimitives>(controller_node_name_+"/get_all_primitives",rmw_qos_profile_services_default, callback_group_);
  get_all_primitives_client_->wait_for_service();

  get_all_tasks_client_ =
      nh_->create_client<hiqp_msgs::srv::GetAllTasks>(controller_node_name_+"/get_all_tasks",rmw_qos_profile_services_default, callback_group_);
  get_all_tasks_client_->wait_for_service();
  
  remove_tasks_client_ =
      nh_->create_client<hiqp_msgs::srv::RemoveTasks>(controller_node_name_+"/remove_tasks",rmw_qos_profile_services_default, callback_group_);
  remove_tasks_client_->wait_for_service();

  remove_primitives_client_ =
      nh_->create_client<hiqp_msgs::srv::RemovePrimitives>(controller_node_name_+"/remove_primitives",rmw_qos_profile_services_default, callback_group_);
  remove_primitives_client_->wait_for_service();

  remove_all_tasks_client_ =
      nh_->create_client<hiqp_msgs::srv::RemoveAllTasks>(controller_node_name_+"/remove_all_tasks",rmw_qos_profile_services_default, callback_group_);
  remove_all_tasks_client_->wait_for_service();

  remove_all_primitives_client_ =
      nh_->create_client<hiqp_msgs::srv::RemoveAllPrimitives>(
          controller_node_name_+"/remove_all_primitives",rmw_qos_profile_services_default, callback_group_);
  remove_all_primitives_client_->wait_for_service();
  
  is_task_set_client_ =
      nh_->create_client<hiqp_msgs::srv::IsTaskSet>(
          controller_node_name_+"/is_task_set",rmw_qos_profile_services_default, callback_group_);
  is_task_set_client_->wait_for_service();

  //subscribers
  std::string topic_global= controller_node_name_ + "/task_measures";
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group2;
  //std::cerr<<"Subscribing to "<<topic_global<<std::endl;
  task_measures_sub_ = nh_->create_subscription<hiqp_msgs::msg::TaskMeasures>(topic_global, 10,
      std::bind(&HiQPClient::taskMeasuresCallback, this, std::placeholders::_1), options);

  RCLCPP_INFO(nh_->get_logger(),"Connected to HiQP Servers.");
}

/** spins the nodes in a new thread, non-blocking call */
void HiQPClient::run() {
  if(running_) return;

  running_=true;
  async_spinner_ = std::shared_ptr<std::thread> (new std::thread(
           [stop_token = stop_async_spinner_.get_future(), this]() {
           auto args = rclcpp::ExecutorOptions();
           //4 threads
           rclcpp::executors::SingleThreadedExecutor executor; //
           //rclcpp::executors::MultiThreadedExecutor executor(args,4);
           executor.add_node(this->nh_);
           executor.spin_until_future_complete(stop_token);
           //at this point we are done and should signal the node
           this->running_ = false;
           }
        ));

}
/** signals the spin thread to stop */
void HiQPClient::quit() {
  stop_async_spinner_.set_value();
  async_spinner_->join();
}

bool HiQPClient::setPrimitives(
    const std::vector<hiqp_msgs::msg::Primitive>& primitives) {
 
  auto request = std::make_shared<hiqp_msgs::srv::SetPrimitives::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::SetPrimitives::Response>();
 
  request->primitives = primitives; 

  if (this->blocking_call<hiqp_msgs::srv::SetPrimitives>
      (set_primitives_client_, request, response)) {
    int returnValue =
        std::accumulate(response->success.begin(),
                        response->success.end(), 0);

    if (returnValue == response->success.size()) {
      RCLCPP_INFO(nh_->get_logger(),"Set primitive(s) succeeded.");
      return true;
    } else {
      RCLCPP_WARN(nh_->get_logger(),"Either all or some of the primitives were not added.");
      return false;
    }
  } else {
    RCLCPP_WARN(nh_->get_logger(),"set_primitive service call failed.");
  }
  return false;
}

bool HiQPClient::setPrimitive(const std::string& name, const std::string& type,
                              const std::string& frame_id, bool visible,
                              const std::vector<double>& color,
                              const std::vector<double>& parameters) {
  hiqp_msgs::msg::Primitive primitive;
  primitive.name = name;
  primitive.type = type;
  primitive.frame_id = frame_id;
  primitive.visible = visible;
  primitive.color = color;
  primitive.parameters = parameters;

  std::vector<hiqp_msgs::msg::Primitive> primitives{primitive};

  return setPrimitives(primitives);
}

bool HiQPClient::setTask(const std::string& name, int16_t priority,
                         bool visible, bool active, bool monitored,
                         const std::vector<std::string>& def_params,
                         const std::vector<std::string>& dyn_params) {
  hiqp_msgs::msg::Task task;

  task.name = name;
  task.priority = priority;
  task.visible = visible;
  task.active = active;
  task.monitored = monitored;
  task.def_params = def_params;
  task.dyn_params = dyn_params;

  std::vector<hiqp_msgs::msg::Task> tasks{task};

  return setTasks(tasks);
}

bool HiQPClient::setTasks(const std::vector<hiqp_msgs::msg::Task>& tasks) {
  
  auto request = std::make_shared<hiqp_msgs::srv::SetTasks::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::SetTasks::Response>();
 
  request->tasks = tasks; 

  if (this->blocking_call<hiqp_msgs::srv::SetTasks>
      (set_tasks_client_, request, response)) {
    
    int returnValue =
        std::accumulate(response->success.begin(),
                        response->success.end(), 0);

    if (returnValue == response->success.size()) {
      RCLCPP_INFO(nh_->get_logger(),"Set task(s) succeeded.");
    } else {
      RCLCPP_WARN(nh_->get_logger(),"Either all or some of the tasks were not added.");
    }
    
    return (returnValue == response->success.size());
  } else {
    RCLCPP_WARN(nh_->get_logger(),"set_tasks service call failed.");
  }
  return false;
}

bool HiQPClient::removeTask(const std::string& task_name) {
  RCLCPP_INFO(nh_->get_logger(),"Removing Task: %s...", task_name.c_str());
  return removeTasks({task_name});
}

bool HiQPClient::removeTasks(const std::vector<std::string>& task_names) {

  auto request = std::make_shared<hiqp_msgs::srv::RemoveTasks::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::RemoveTasks::Response>();
 
  request->names = task_names; 

  if (this->blocking_call<hiqp_msgs::srv::RemoveTasks>
      (remove_tasks_client_, request, response)) {
    int returnValue =
        std::accumulate(response->success.begin(),
                        response->success.end(), 0);

    if (returnValue == response->success.size()) {
      RCLCPP_INFO(nh_->get_logger(),"Remove task(s) succeeded.");
      
      //resource_mutex_.lock();
      for(auto task_name : task_names) {
         task_name_sq_error_map_.erase(task_name);
      }
      //resource_mutex_.unlock();

      return true;
    } else {
      RCLCPP_WARN(nh_->get_logger(),"Either all or some of the tasks were not removed.");
      return false;
    }

  } else {
    RCLCPP_WARN(nh_->get_logger(),"remove_tasks service call failed.");
    return false;
  }
}

bool HiQPClient::removePrimitive(const std::string& primitive_name) {
  RCLCPP_INFO(nh_->get_logger(),"Removing primitive: %s...", primitive_name.c_str());
  return removePrimitives({primitive_name});
}

bool HiQPClient::removePrimitives(
    const std::vector<std::string>& primitive_names) {
  
  auto request = std::make_shared<hiqp_msgs::srv::RemovePrimitives::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::RemovePrimitives::Response>();
 
  request->names = primitive_names; 

  if (this->blocking_call<hiqp_msgs::srv::RemovePrimitives>
      (remove_primitives_client_, request, response)) {
    int returnValue =
        std::accumulate(response->success.begin(),
                        response->success.end(), 0);

    if (returnValue == response->success.size()) {
      RCLCPP_INFO(nh_->get_logger(),"Remove primitive(s) succeeded.");
      return true;
    } else {
      RCLCPP_WARN(nh_->get_logger(),"Either all or some of the primitives were not removed.");
      return false;
    }
  } else {
    RCLCPP_WARN(nh_->get_logger(),"remove_primitives service call failed.");
    return false;
  }
}

bool HiQPClient::deactivateTask(const std::string& task_name) {
  RCLCPP_INFO(nh_->get_logger(),"Deactivating Task: %s...", task_name.c_str());
  
  auto request = std::make_shared<hiqp_msgs::srv::DeactivateTask::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::DeactivateTask::Response>();
 
  request->name = task_name; 

  if (!this->blocking_call<hiqp_msgs::srv::DeactivateTask>
      (deactivate_task_client_, request, response)) {
    RCLCPP_WARN(nh_->get_logger(),
        "Deactivating task \'%s\' failed. See server output/log for details.",
        task_name.c_str());
    return false;
  }
  return true;
}

void HiQPClient::activateTasks(const std::vector<std::string>& task_names) {
  for (auto& task_name : task_names){
    RCLCPP_INFO(nh_->get_logger(),"Activating Tasks: %s...", task_name.c_str());
    auto request = std::make_shared<hiqp_msgs::srv::ActivateTask::Request>();
    auto response = std::make_shared<hiqp_msgs::srv::ActivateTask::Response>();

    request->name = task_name; 

    if (!this->blocking_call<hiqp_msgs::srv::ActivateTask>
        (activate_task_client_, request, response)) {

      RCLCPP_WARN(nh_->get_logger(),
          "Activating task \'%s\' failed. See server output/log for details.",
          task_name.c_str());
    }
  }
}

std::vector<hiqp_msgs::msg::Primitive> HiQPClient::getAllPrimitives() {
  
  auto request = std::make_shared<hiqp_msgs::srv::GetAllPrimitives::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::GetAllPrimitives::Response>();

	std::vector<hiqp_msgs::msg::Primitive> primitives;
  if (!this->blocking_call<hiqp_msgs::srv::GetAllPrimitives>
      (get_all_primitives_client_, request, response)) {
    primitives = response->primitives;
  } else {
    RCLCPP_WARN(nh_->get_logger(),"get_all_primitives service call failed.");
    primitives = {};
  }
  return primitives;
}

std::vector<hiqp_msgs::msg::Task> HiQPClient::getAllTasks() {
  auto request = std::make_shared<hiqp_msgs::srv::GetAllTasks::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::GetAllTasks::Response>();

	std::vector<hiqp_msgs::msg::Task> tasks;
  if (!this->blocking_call<hiqp_msgs::srv::GetAllTasks>
      (get_all_tasks_client_, request, response)) {
    tasks = response->tasks;
  } else {
    RCLCPP_WARN(nh_->get_logger(),"get_all_tasks service call failed.");
    tasks = {};
  }
  return tasks;
}

std::string taskMeasuresAsString(
    const hiqp_msgs::msg::TaskMeasures::ConstPtr& task_measures) {
  std::string s;
  for (auto task_measure : task_measures->task_measures) {
    double sq_error;
    if (task_measure.task_sign == 0) {
      sq_error =
          std::inner_product(task_measure.e.begin(), task_measure.e.end(),
                             task_measure.e.begin(), 0.0);

      s += "\n[" + task_measure.task_name + "] Progress: " +
           std::to_string(exp(-sq_error) * 100.0) + ", Error: " +
           std::to_string(sq_error);
    } else {
      sq_error = task_measure.e[0];
      s += "\n[" + task_measure.task_name + "] Progress: " +
           (sq_error * task_measure.task_sign > 0 ? "In limits" : "Off limits");
    }
  }
  //s += "\n";
  return s;
}


void HiQPClient::taskMeasuresCallback(
    const hiqp_msgs::msg::TaskMeasures::SharedPtr task_measures) {
  RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "%s",
                            taskMeasuresAsString(task_measures).c_str());
  //resource_mutex_.lock();
  //task_name_task_sign_map_.clear();
  //task_name_sq_error_map_.clear();
  for (auto task_measure : task_measures->task_measures) {
    double sq_error;
    if (task_measure.task_sign == hiqp_msgs::msg::TaskMeasure::EQ)
      sq_error =
          std::inner_product(task_measure.e.begin(), task_measure.e.end(),
                             task_measure.e.begin(), 0.0);
    else
      sq_error = task_measure.e[0];

    task_name_task_sign_map_[task_measure.task_name] = task_measure.task_sign;
    task_name_sq_error_map_[task_measure.task_name] = sq_error;
  }
  //resource_mutex_.unlock();
}

std::string taskNamesVectorAsString(
    const std::vector<std::string>& task_names) {
  std::string taskNamesAsString;
  for (auto task_name : task_names) {
    taskNamesAsString += task_name + ",";
  }
  taskNamesAsString += '\b';
  return taskNamesAsString;
}

void HiQPClient::waitForCompletion(
    const std::vector<std::string>& task_names,
    const std::vector<TaskDoneReaction>& reactions,
    const std::vector<double>& error_tol, double max_exec_time) {
  
  assert(task_names.size() == reactions.size() &&
             reactions.size() == error_tol.size());
  int status = 0;
  auto clock = nh_->get_clock();

  auto start = clock->now();
  
  auto duration = std::chrono::duration<double>(max_exec_time);
  rclcpp::Duration max_exec_dur(duration);

  bool time_exceeded = false;
  
  auto isTaskSet = std::make_shared<hiqp_msgs::srv::IsTaskSet::Request>();
  auto isTaskSetResponse = std::make_shared<hiqp_msgs::srv::IsTaskSet::Response>();
 
  std::cerr<<"waiting for all tasks to be monitored\n";
  while (status < task_names.size() && rclcpp::ok()) {
    status=0;
    for (auto i = 0; i < task_names.size(); i++) {
      auto& task_name = task_names[i];
      auto it_sq_error = task_name_sq_error_map_.find(task_name);
      auto it_sign = task_name_task_sign_map_.find(task_name);
      if (it_sq_error != task_name_sq_error_map_.end() && it_sign != task_name_task_sign_map_.end()) {
        RCLCPP_INFO(nh_->get_logger(),"%s task is now being monitored.", task_name.c_str());
        status += 1;
        continue;
      }
    }
  }
  status=0;

  std::cerr<<"task "<<task_names[0]<<" with tolerance "<<error_tol[0]<<std::endl; 
  while (status < task_names.size() && rclcpp::ok()) {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *clock, 1000, 
        "[waitForCompletion]: %d out of %ld tasks complete.",
                      status, task_names.size());
    status = 0;
    for (auto i = 0; i < task_names.size(); i++) {
      auto& task_name = task_names[i];
      double tol = error_tol[i];

      //resource_mutex_.lock();
      auto it_sq_error = task_name_sq_error_map_.find(task_name);
      auto it_sign = task_name_task_sign_map_.find(task_name);

      if (max_exec_time != 0 && ((clock->now() - start) > max_exec_dur)) {
        RCLCPP_INFO(nh_->get_logger(),"Max exection time exceeded");
        status += 1;
        //resource_mutex_.unlock();
        time_exceeded = true;
        break;
      }
     
      /* 
      isTaskSet->name = task_name;

      if (this->blocking_call<hiqp_msgs::srv::IsTaskSet>
          (is_task_set_client_, isTaskSet, isTaskSetResponse)) {
      	if (!isTaskSetResponse->is_set) {
      		RCLCPP_INFO(nh_->get_logger(),"%s task has been removed.", task_name.c_str());
      		status += 1;
        	//resource_mutex_.unlock();
        	continue;
      	}
      } else {
      	RCLCPP_WARN(nh_->get_logger(),"is_task_set_ service call failed.");
        continue;
      }
      */
      
      if (it_sq_error == task_name_sq_error_map_.end() || it_sign == task_name_task_sign_map_.end()) {
        RCLCPP_INFO(nh_->get_logger(),"%s task is not monitored.", task_name.c_str());
        status += 1;
        //resource_mutex_.unlock();
        continue;
      }

      //std::cerr<<"task "<<it_sq_error->first<<" error "<<it_sq_error->second<<" tol "
      //  <<tol<<" sign "<<it_sign->second<<std::endl;
      if (it_sign->second == 0) {
        if (it_sq_error->second < tol) {
          status += 1;
      		//RCLCPP_INFO(nh_->get_logger(),"%s task has been completed with error %lf.", task_name.c_str(), it_sq_error->second);
          //std::cerr<<"equality task done\n";
        }
      } else {
        if (it_sq_error->second * it_sign->second  >
            -1 * tol) {
          status += 1;
      		//RCLCPP_INFO(nh_->get_logger(),"%s task (inequality) has been completed with error %lf.", task_name.c_str(), it_sq_error->second);
          //std::cerr<<"inequality task done\n";
        }
      }
      //resource_mutex_.unlock();
    }
    if (time_exceeded) {
      break;
    }
  }

  RCLCPP_INFO(nh_->get_logger(),"All tasks completed.");

  std::vector<std::string> tasks_to_remove;

  for (auto i = 0; i < task_names.size(); i++) {
    auto& task_name = task_names[i];
    auto& reaction = reactions[i];

    switch (reaction) {
      case TaskDoneReaction::NONE:
        break;
      case TaskDoneReaction::REMOVE:
        tasks_to_remove.push_back(task_name);
        break;
      case TaskDoneReaction::PRINT_INFO:
        RCLCPP_INFO(nh_->get_logger(),"Task %s completed with error %lf", task_name.c_str(),
                 task_name_sq_error_map_[task_name]);
        break;
      case TaskDoneReaction::DEACTIVATE:
        deactivateTask(task_name);
        break;
    }
    //resource_mutex_.lock();
    task_name_sq_error_map_.erase(task_name);
    //resource_mutex_.unlock();
  }
  removeTasks(tasks_to_remove);
}

bool HiQPClient::setJointAngles(const std::vector<double>& joint_angles,
    bool remove, double tol) {
  std::vector<std::string> def_params{"TDefFullPose"};

  for (auto jointValue : joint_angles) {
    def_params.push_back(std::to_string(jointValue));
  }

  bool ret = this->setTask("joint_configuration", 3, true, true, true, def_params,
      {"TDynLinear", "0.75"});
  if (ret) {
    if (remove) {
      std::cerr<<"setting tolerance of "<<tol<<std::endl;
      waitForCompletion({"joint_configuration"}, {TaskDoneReaction::REMOVE},
          {tol});
    }
    else {
      waitForCompletion({"joint_configuration"}, {TaskDoneReaction::NONE},
          {tol});
    }
  } else {
    RCLCPP_ERROR(nh_->get_logger(),"could not set joint configuration task");
  }
  return ret;
}

bool HiQPClient::removeAllTasks() {
  
  auto request = std::make_shared<hiqp_msgs::srv::RemoveAllTasks::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::RemoveAllTasks::Response>();

  if (this->blocking_call<hiqp_msgs::srv::RemoveAllTasks>
      (remove_all_tasks_client_, request, response)) {
    if (response->success) {
      RCLCPP_INFO(nh_->get_logger(),"All tasks removed.");
      return true;
    } else {
      RCLCPP_ERROR(nh_->get_logger(),"Failed to remove all tasks.");
    }
  } else {
    RCLCPP_FATAL(nh_->get_logger(),"remove_all_tasks service call failed.");
  }
  return false;
}

bool HiQPClient::removeAllPrimitives() {
  auto request = std::make_shared<hiqp_msgs::srv::RemoveAllPrimitives::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::RemoveAllPrimitives::Response>();

  if (this->blocking_call<hiqp_msgs::srv::RemoveAllPrimitives>
      (remove_all_primitives_client_, request, response)) {
    if (response->success) {
      RCLCPP_INFO(nh_->get_logger(),"All primitives removed.");
      return true;
    } else {
      RCLCPP_ERROR(nh_->get_logger(),"Failed to remove all primitives.");
    }
  } else {
    RCLCPP_FATAL(nh_->get_logger(),"remove_all_primitives service call failed.");
  }
  return false;
}

bool HiQPClient::resetHiQPController() {
  bool ret = removeAllTasks();
  ret = ret && removeAllPrimitives();
  return ret;
}

bool HiQPClient::isTaskSet(const std::string& task_name) {

  auto request = std::make_shared<hiqp_msgs::srv::IsTaskSet::Request>();
  auto response = std::make_shared<hiqp_msgs::srv::IsTaskSet::Response>();

  if (this->blocking_call<hiqp_msgs::srv::IsTaskSet>
      (is_task_set_client_, request, response)) {
		if (response->is_set) {
      RCLCPP_INFO(nh_->get_logger(),"All tasks removed.");
      return true;
  	} else {
    	RCLCPP_WARN(nh_->get_logger(),"is_task_set service call failed.");
  	}
  }
  return false;
}

hiqp_msgs::msg::Task createTaskMsg(const std::string& name, int16_t priority,
                              bool visible, bool active, bool monitored,
                              const std::vector<std::string>& def_params,
                              const std::vector<std::string>& dyn_params) {
  hiqp_msgs::msg::Task task;
  task.name = name;
  task.priority = priority;
  task.visible = visible;
  task.active = active;
  task.monitored = monitored;
  task.def_params = def_params;
  task.dyn_params = dyn_params;

  return task;
}

hiqp_msgs::msg::Primitive createPrimitiveMsg(const std::string& name,
                                        const std::string& type,
                                        const std::string& frame_id,
                                        bool visible,
                                        const std::vector<double>& color,
                                        const std::vector<double>& parameters) {
  hiqp_msgs::msg::Primitive primitive;
  primitive.name = name;
  primitive.type = type;
  primitive.frame_id = frame_id;
  primitive.visible = visible;
  primitive.color = color;
  primitive.parameters = parameters;

  return primitive;
}

}

