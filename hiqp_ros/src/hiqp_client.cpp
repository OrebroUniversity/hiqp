#include <hiqp_ros/hiqp_client.h>
#include <memory>
#include <cassert>

namespace hiqp_ros {

HiQPClient::HiQPClient(const std::string& robot_namespace,
                       const std::string& controller_namespace,
                       bool auto_connect)
    : robot_namespace_(robot_namespace),
      controller_namespace_(controller_namespace),
      running_(false) {
  if (auto_connect) this->connectToServer();
  nh_ = std::make_shared<rclcpp::Node> (robot_namespace + "/" + controller_namespace);
}

void HiQPClient::connectToServer() {

  //clients for talking to controller
  set_tasks_client_ = nh_->create_client<hiqp_msgs::srv::SetTasks>("set_tasks");
  set_tasks_client_->wait_for_service();

  set_primitives_client_ =
      nh_->create_client<hiqp_msgs::srv::SetPrimitives>("set_primitives");
  set_primitives_client_->wait_for_service();

  activate_task_client_ =
      nh_->create_client<hiqp_msgs::srv::ActivateTask>("activate_task");
  activate_task_client_->wait_for_service();

  deactivate_task_client_ =
      nh_->create_client<hiqp_msgs::srv::DeactivateTask>("deactivate_task");
  deactivate_task_client_->wait_for_service();

	get_all_primitives_client_ =
      nh_->create_client<hiqp_msgs::srv::GetAllPrimitives>("get_all_primitives");
  get_all_primitives_client_->wait_for_service();

  get_all_tasks_client_ =
      nh_->create_client<hiqp_msgs::srv::GetAllTasks>("get_all_tasks");
  get_all_tasks_client_->wait_for_service();
  
  remove_tasks_client_ =
      nh_->create_client<hiqp_msgs::srv::RemoveTasks>("remove_tasks");
  remove_tasks_client_->wait_for_service();

  remove_primitives_client_ =
      nh_->create_client<hiqp_msgs::srv::RemovePrimitives>("remove_primitives");
  remove_primitives_client_->wait_for_service();

  remove_all_tasks_client_ =
      nh_->create_client<hiqp_msgs::srv::RemoveAllTasks>("remove_all_tasks");
  remove_all_tasks_client_->wait_for_service();

  remove_all_primitives_client_ =
      nh_->create_client<hiqp_msgs::srv::RemoveAllPrimitives>(
          "remove_all_primitives");
  remove_all_primitives_client_->wait_for_service();
  
  is_task_set_client_ =
      nh_->create_client<hiqp_msgs::srv::IsTaskSet>(
          "is_task_set");
  is_task_set_client_->wait_for_service();

  //subscribers
  std::string topic_global= "/" + robot_namespace_ + "/" + controller_namespace_ + "/task_measures";
  //std::cerr<<"Subscribing to "<<topic_global<<std::endl;
  task_measures_sub_ = nh_->create_subscription<hiqp_msgs::msg::TaskMeasures>(topic_global, 10,
      std::bind(&HiQPClient::taskMeasuresCallback, this, std::placeholders::_1));

  RCLCPP_INFO(nh_->get_logger(),"Connected to HiQP Servers.");
}

/** spins the nodes in a new thread, non-blocking call */
void HiQPClient::run() {
  if(running_) return;

  running_=true;
  async_spinner_ = std::shared_ptr<std::thread> (new std::thread(
           [stop_token = stop_async_spinner_.get_future(), this]() {
           rclcpp::executors::MultiThreadedExecutor executor;
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
 
  hiqp_msgs::srv::SetPrimitives setPrimitivesMsg;
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

#if 0
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
  hiqp_msgs::msg::SetTasks setTasksMsg;
  setTasksMsg.request.tasks = tasks;

  if (set_tasks_client_.call(setTasksMsg)) {
    int returnValue = std::accumulate(setTasksMsg.response.success.begin(),
                                      setTasksMsg.response.success.end(), 0);

    if (returnValue == setTasksMsg.response.success.size()) {
      RCLCPP_INFO(nh_->get_logger(),"Set task(s) succeeded.");
    } else {
      RCLCPP_WARN(nh_->get_logger(),"Either all or some of the tasks were not added.");
    }
    
    return (returnValue == setTasksMsg.response.success.size());
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
  hiqp_msgs::msg::RemoveTasks removeTasksMsg;
  removeTasksMsg.request.names = task_names;

  if (remove_tasks_client_.call(removeTasksMsg)) {
    int returnValue = std::accumulate(removeTasksMsg.response.success.begin(),
                                      removeTasksMsg.response.success.end(), 0);

    if (returnValue == removeTasksMsg.response.success.size()) {
      RCLCPP_INFO(nh_->get_logger(),"Remove task(s) succeeded.");
      
      resource_mutex_.lock();
      for(auto task_name : task_names) {
         task_name_sq_error_map_.erase(task_name);
      }
      resource_mutex_.unlock();

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
  hiqp_msgs::msg::RemovePrimitives removePrimitivesMsg;
  removePrimitivesMsg.request.names = primitive_names;

  if (remove_primitives_client_.call(removePrimitivesMsg)) {
    int returnValue =
        std::accumulate(removePrimitivesMsg.response.success.begin(),
                        removePrimitivesMsg.response.success.end(), 0);

    if (returnValue == removePrimitivesMsg.response.success.size()) {
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
  hiqp_msgs::msg::DeactivateTask deactivateTaskMsg;
  deactivateTaskMsg.request.name = task_name;

  if (!deactivate_task_client_.call(deactivateTaskMsg)) {
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
        hiqp_msgs::msg::ActivateTask activateTaskMsg;
        activateTaskMsg.request.name = task_name;

        if (!activate_task_client_.call(activateTaskMsg)) {
            RCLCPP_WARN(nh_->get_logger(),
                    "Activating task \'%s\' failed. See server output/log for details.",
                    task_name.c_str());
        }
    }
}

std::vector<hiqp_msgs::msg::Primitive> HiQPClient::getAllPrimitives() {
	hiqp_msgs::msg::GetAllPrimitives getAllPrimitivesMsg;
	std::vector<hiqp_msgs::msg::Primitive> primitives;

	if (get_all_primitives_client_.call(getAllPrimitivesMsg)) {
    primitives = getAllPrimitivesMsg.response.primitives;
  } else {
    RCLCPP_WARN(nh_->get_logger(),"get_all_primitives service call failed.");
    primitives = {};
  }
  return primitives;
}

std::vector<hiqp_msgs::msg::Task> HiQPClient::getAllTasks() {
	hiqp_msgs::msg::GetAllTasks getAllTasksMsg;
	std::vector<hiqp_msgs::msg::Task> tasks;

	if (get_all_tasks_client_.call(getAllTasksMsg)) {
    tasks = getAllTasksMsg.response.tasks;
  } else {
    RCLCPP_WARN(nh_->get_logger(),"get_all_tasks service call failed.");
    tasks = {};
  }
  return tasks;
}

std::string taskMeasuresAsString(
    const hiqp_msgs::msg::TaskMeasuresConstPtr& task_measures) {
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
  s += "\n";
  return s;
}

void HiQPClient::taskMeasuresCallback(
    const hiqp_msgs::msg::TaskMeasuresSharedPtr task_measures) {
  RCLCPP_INFO_THROTTLE(nh_->get_logger(), nh_->get_clock(), 5000, "%s",
                            taskMeasuresAsString(task_measures).c_str());
  resource_mutex_.lock();
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
  resource_mutex_.unlock();
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
  ros::Time start = ros::Time::now();
  
  ros::Duration max_exec_dur(max_exec_time);
  bool time_exceeded = false;
  
  hiqp_msgs::msg::IsTaskSet isTaskSetMsg;
  
  while (status < task_names.size() && ros::ok()) {
  RCLCPP_INFO_THROTTLE(nh_->get_logger(), nh_->get_clock(), 5000, 
        "[waitForCompletion]: %d out of %ld tasks complete.",
                      status, task_names.size());
    status = 0;
    for (auto i = 0; i < task_names.size(); i++) {
      auto& task_name = task_names[i];
      auto& tol = error_tol[i];

      resource_mutex_.lock();
      auto it_sq_error = task_name_sq_error_map_.find(task_name);

      if (max_exec_time != 0 && ((ros::Time::now() - start) > max_exec_dur)) {
        RCLCPP_INFO(nh_->get_logger(),"Max exection time exceeded");
        status += 1;
        resource_mutex_.unlock();
        time_exceeded = true;
        break;
      }
      
      isTaskSetMsg.request.name = task_name;
      if (is_task_set_client_.call(isTaskSetMsg)) {
      	if (!isTaskSetMsg.response.is_set) {
      		RCLCPP_INFO(nh_->get_logger(),"%s task has been removed.", task_name.c_str());
      		status += 1;
        	resource_mutex_.unlock();
        	continue;
      	}
      } else {
      	RCLCPP_WARN(nh_->get_logger(),"is_task_set_ service call failed.");
      }
      
      if (it_sq_error == task_name_sq_error_map_.end()) {
        resource_mutex_.unlock();
        continue;
      }

      //std::cerr<<"error "<<it_sq_error->second<<std::endl;
      if (task_name_task_sign_map_[task_name] == 0) {
        if (it_sq_error->second < tol) {
          status += 1;
        }
      } else {
        if (it_sq_error->second * task_name_task_sign_map_[task_name] >
            -1 * tol) {
          status += 1;
        }
      }
      resource_mutex_.unlock();
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
    resource_mutex_.lock();
    task_name_sq_error_map_.erase(task_name);
    resource_mutex_.unlock();
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
                {"TDynPD", "0.75"});
  if (ret) {
      if (remove) {
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
  hiqp_msgs::msg::RemoveAllTasks removeAllTasksMsg;
  if (remove_all_tasks_client_.call(removeAllTasksMsg)) {
    if (removeAllTasksMsg.response.success) {
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
  hiqp_msgs::msg::RemoveAllPrimitives removeAllPrimitivesMsg;
  if (remove_all_primitives_client_.call(removeAllPrimitivesMsg)) {
    if (removeAllPrimitivesMsg.response.success) {
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
	hiqp_msgs::msg::IsTaskSet isTaskSetMsg;
  isTaskSetMsg.request.name = task_name;

  if (is_task_set_client_.call(isTaskSetMsg)) {
		if (isTaskSetMsg.response.is_set) {
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
#endif

}

