#include <hiqp_ros/hiqp_client.h>

namespace hiqp_ros {

HiQPClient::HiQPClient(const std::string& robot_namespace,
                       const std::string& controller_namespace,
                       bool auto_connect)
    : robot_namespace_(robot_namespace),
      controller_namespace_(controller_namespace),
      nh_(robot_namespace + "/" + controller_namespace),
      robot_nh_(robot_namespace) {
  // TODO: read error_tolerance from param.
  if (auto_connect) this->connectToServer();
}

void HiQPClient::connectToServer() {
  set_tasks_client_ = nh_.serviceClient<hiqp_msgs::SetTasks>("set_tasks");
  set_tasks_client_.waitForExistence();

  set_primitives_client_ =
      nh_.serviceClient<hiqp_msgs::SetPrimitives>("set_primitives");
  set_primitives_client_.waitForExistence();

  activate_task_client_ =
      nh_.serviceClient<hiqp_msgs::ActivateTask>("activate_task");
  activate_task_client_.waitForExistence();

  deactivate_task_client_ =
      nh_.serviceClient<hiqp_msgs::DeactivateTask>("deactivate_task");
  deactivate_task_client_.waitForExistence();

  remove_tasks_client_ =
      nh_.serviceClient<hiqp_msgs::RemoveTasks>("remove_tasks");
  remove_tasks_client_.waitForExistence();

  remove_primitives_client_ =
      nh_.serviceClient<hiqp_msgs::RemovePrimitives>("remove_primitives");
  remove_primitives_client_.waitForExistence();

  remove_all_tasks_client_ =
      nh_.serviceClient<hiqp_msgs::RemoveAllTasks>("remove_all_tasks");
  remove_all_tasks_client_.waitForExistence();

  remove_all_primitives_client_ =
      nh_.serviceClient<hiqp_msgs::RemoveAllPrimitives>(
          "remove_all_primitives");
  remove_all_primitives_client_.waitForExistence();

  task_measures_sub_ = nh_.subscribe("task_measures", 1,
                                     &HiQPClient::taskMeasuresCallback, this);

  ROS_INFO("Connected to HiQP Servers.");
}

void HiQPClient::setPrimitives(
    const std::vector<hiqp_msgs::Primitive>& primitives) {
  hiqp_msgs::SetPrimitives setPrimitivesMsg;
  setPrimitivesMsg.request.primitives = primitives;

  if (set_primitives_client_.call(setPrimitivesMsg)) {
    int returnValue =
        std::accumulate(setPrimitivesMsg.response.success.begin(),
                        setPrimitivesMsg.response.success.end(), 0);

    if (returnValue == setPrimitivesMsg.response.success.size())
      ROS_INFO("Set primitive(s) succeeded.");
    else
      ROS_WARN("Either all or some of the primitives were not added.");
  } else
    ROS_WARN("set_primitive service call failed.");
}

void HiQPClient::setPrimitive(const std::string& name, const std::string& type,
                              const std::string& frame_id, bool visible,
                              const std::vector<double>& color,
                              const std::vector<double>& parameters) {
  hiqp_msgs::Primitive primitive;
  primitive.name = name;
  primitive.type = type;
  primitive.frame_id = frame_id;
  primitive.visible = visible;
  primitive.color = color;
  primitive.parameters = parameters;

  std::vector<hiqp_msgs::Primitive> primitives{primitive};

  setPrimitives(primitives);
}

void HiQPClient::setTask(const std::string& name, int16_t priority,
                         bool visible, bool active, bool monitored,
                         const std::vector<std::string>& def_params,
                         const std::vector<std::string>& dyn_params) {
  hiqp_msgs::Task task;

  task.name = name;
  task.priority = priority;
  task.visible = visible;
  task.active = active;
  task.monitored = monitored;
  task.def_params = def_params;
  task.dyn_params = dyn_params;

  std::vector<hiqp_msgs::Task> tasks{task};

  setTasks(tasks);
}

void HiQPClient::setTasks(const std::vector<hiqp_msgs::Task>& tasks) {
  hiqp_msgs::SetTasks setTasksMsg;
  setTasksMsg.request.tasks = tasks;

  if (set_tasks_client_.call(setTasksMsg)) {
    int returnValue = std::accumulate(setTasksMsg.response.success.begin(),
                                      setTasksMsg.response.success.end(), 0);

    if (returnValue == setTasksMsg.response.success.size())
      ROS_INFO("Set task(s) succeeded.");
    else
      ROS_WARN("Either all or some of the tasks were not added.");

  } else
    ROS_WARN("set_tasks service call failed.");
}

void HiQPClient::removeTask(const std::string& task_name) {
  ROS_INFO("Removing Task: %s...", task_name.c_str());
  removeTasks({task_name});
}

void HiQPClient::removeTasks(const std::vector<std::string>& task_names) {
  hiqp_msgs::RemoveTasks removeTasksMsg;
  removeTasksMsg.request.names = task_names;

  if (remove_tasks_client_.call(removeTasksMsg)) {
    int returnValue = std::accumulate(removeTasksMsg.response.success.begin(),
                                      removeTasksMsg.response.success.end(), 0);

    if (returnValue == removeTasksMsg.response.success.size())
      ROS_INFO("Remove task(s) succeeded.");
    else
      ROS_WARN("Either all or some of the tasks were not removed.");

  } else
    ROS_WARN("remove_tasks service call failed.");
}

void HiQPClient::removePrimitive(const std::string& primitive_name) {
  ROS_INFO("Removing primitive: %s...", primitive_name.c_str());
  removePrimitives({primitive_name});
}

void HiQPClient::removePrimitives(
    const std::vector<std::string>& primitive_names) {
  hiqp_msgs::RemovePrimitives removePrimitivesMsg;
  removePrimitivesMsg.request.names = primitive_names;

  if (remove_primitives_client_.call(removePrimitivesMsg)) {
    int returnValue =
        std::accumulate(removePrimitivesMsg.response.success.begin(),
                        removePrimitivesMsg.response.success.end(), 0);

    if (returnValue == removePrimitivesMsg.response.success.size())
      ROS_INFO("Remove primitive(s) succeeded.");
    else
      ROS_WARN("Either all or some of the primitives were not removed.");

  } else
    ROS_WARN("remove_primitives service call failed.");
}

void HiQPClient::deactivateTask(const std::string& task_name) {
  ROS_INFO("Deactivating Task: %s...", task_name.c_str());
  hiqp_msgs::DeactivateTask deactivateTaskMsg;
  deactivateTaskMsg.request.name = task_name;

  if (!deactivate_task_client_.call(deactivateTaskMsg)) {
    ROS_WARN(
        "Deactivating task \'%s\' failed. See server output/log for details.",
        task_name.c_str());
  }
}

std::string taskMeasuresAsString(
    const hiqp_msgs::TaskMeasuresConstPtr& task_measures) {
  std::string s;
  for (auto task_measure : task_measures->task_measures) {
    double sq_error =
        std::inner_product(task_measure.e.begin(), task_measure.e.end(),
                           task_measure.e.begin(), 0.0);

    s += "\n[" + task_measure.task_name + "] Progress: " +
         std::to_string(exp(-sq_error) * 100.0);
  }
  s += "\n";
  return s;
}

void HiQPClient::taskMeasuresCallback(
    const hiqp_msgs::TaskMeasuresConstPtr& task_measures) {
  resource_mutex_.lock();
  ROS_INFO_DELAYED_THROTTLE(5.0, "%s",
                            taskMeasuresAsString(task_measures).c_str());
  for (auto task_measure : task_measures->task_measures) {
    double sq_error =
        std::inner_product(task_measure.e.begin(), task_measure.e.end(),
                           task_measure.e.begin(), 0.0);

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
    const std::vector<double>& error_tol) {
  ROS_ASSERT(task_names.size() == reactions.size() &&
             reactions.size() == error_tol.size());
  int status = 0;
  while (status < task_names.size()) {
    ROS_INFO_THROTTLE(5, "[waitForCompletion]: %d out of %ld tasks complete.",
                      status, task_names.size());
    status = 0;
    for (auto i = 0; i < task_names.size(); i++) {
      auto& task_name = task_names[i];
      auto& tol = error_tol[i];

      resource_mutex_.lock();
      auto it_sq_error = task_name_sq_error_map_.find(task_name);
      if (it_sq_error == task_name_sq_error_map_.end()) {
        resource_mutex_.unlock();
        continue;
      }

      if (it_sq_error->second < tol) {
        status += 1;
      }
      resource_mutex_.unlock();
    }
  }

  ROS_INFO("All tasks completed.");

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
        ROS_INFO("Task %s completed with error %lf", task_name.c_str(),
                 task_name_sq_error_map_[task_name]);
        break;
      case TaskDoneReaction::DEACTIVATE:
        deactivateTask(task_name);
        break;
    }
    task_name_sq_error_map_.erase(task_name);
  }
  removeTasks(tasks_to_remove);
}

void HiQPClient::setJointAngles(const std::vector<double>& joint_angles) {
  // TODO: Read joint angles task error tolerance from param server.
  std::vector<std::string> def_params{"TDefFullPose"};

  for (auto jointValue : joint_angles) {
    def_params.push_back(std::to_string(jointValue));
  }

  this->setTask("joint_angles_task", 3, true, true, true, def_params,
                {"TDynLinear", "0.5"});
  waitForCompletion({"joint_angles_task"}, {TaskDoneReaction::REMOVE}, {1e-3});
}

void HiQPClient::removeAllTasks() {
  hiqp_msgs::RemoveAllTasks removeAllTasksMsg;
  if (remove_all_primitives_client_.call(removeAllTasksMsg)) {
    if (removeAllTasksMsg.response.success) {
      ROS_INFO("All tasks removed.");
    } else {
      ROS_ERROR("Failed to remove all tasks.");
    }
  } else
    ROS_FATAL("remove_all_tasks service call failed.");
}

void HiQPClient::removeAllPrimitives() {
  hiqp_msgs::RemoveAllPrimitives removeAllPrimitivesMsg;
  if (remove_all_primitives_client_.call(removeAllPrimitivesMsg)) {
    if (removeAllPrimitivesMsg.response.success) {
      ROS_INFO("All primitives removed.");
    } else {
      ROS_ERROR("Failed to remove all primitives.");
    }
  } else
    ROS_FATAL("remove_all_primitives service call failed.");
}

void HiQPClient::resetHiQPController() {
  removeAllTasks();
  removeAllPrimitives();
}

hiqp_msgs::Task createTaskMsg(const std::string& name, int16_t priority,
                              bool visible, bool active, bool monitored,
                              const std::vector<std::string>& def_params,
                              const std::vector<std::string>& dyn_params) {
  hiqp_msgs::Task task;
  task.name = name;
  task.priority = priority;
  task.visible = visible;
  task.active = active;
  task.monitored = monitored;
  task.def_params = def_params;
  task.dyn_params = dyn_params;

  return task;
}

hiqp_msgs::Primitive createPrimitiveMsg(const std::string& name,
                                        const std::string& type,
                                        const std::string& frame_id,
                                        bool visible,
                                        const std::vector<double>& color,
                                        const std::vector<double>& parameters) {
  hiqp_msgs::Primitive primitive;
  primitive.name = name;
  primitive.type = type;
  primitive.frame_id = frame_id;
  primitive.visible = visible;
  primitive.color = color;
  primitive.parameters = parameters;

  return primitive;
}
}
