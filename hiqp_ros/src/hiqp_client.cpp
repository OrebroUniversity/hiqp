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

  remove_task_client_ = nh_.serviceClient<hiqp_msgs::RemoveTask>("remove_task");
  remove_task_client_.waitForExistence();

  remove_primitive_client_ =
      nh_.serviceClient<hiqp_msgs::RemovePrimitive>("remove_primitive");
  remove_primitive_client_.waitForExistence();

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
                         const std::vector<std::string>& dyn_params,
                         TaskDoneReaction tdr, double error_tolerance) {
  hiqp_msgs::Task task;

  task.name = name;
  task.priority = priority;
  task.visible = visible;
  task.active = active;
  task.monitored = monitored;
  task.def_params = def_params;
  task.dyn_params = dyn_params;

  std::vector<hiqp_msgs::Task> tasks{task};
  std::vector<TaskDoneReaction> tdr_vector{tdr};
  std::vector<double> etol_vector{error_tolerance};

  setTasks(tasks, tdr_vector, etol_vector);
}

void HiQPClient::setTasks(const std::vector<hiqp_msgs::Task>& tasks,
                          const std::vector<TaskDoneReaction>& tdr_vector,
                          const std::vector<double>& etol_vector) {
  if (tasks.size() != tdr_vector.size() ||
      tdr_vector.size() != etol_vector.size() ||
      etol_vector.size() != tasks.size()) {
    ROS_FATAL(
        "The size of all three vectors should be equal in call to "
        "HiQPClient::setTasks(). FAILED! ");
    return;
  }
  hiqp_msgs::SetTasks setTasksMsg;
  setTasksMsg.request.tasks = tasks;

  if (set_tasks_client_.call(setTasksMsg)) {
    int returnValue = std::accumulate(setTasksMsg.response.success.begin(),
                                      setTasksMsg.response.success.end(), 0);

    if (returnValue == setTasksMsg.response.success.size())
      ROS_INFO("Set task(s) succeeded.");
    else
      ROS_WARN("Either all or some of the tasks were not added.");

    // If the service call succeeded and this task was added. We can set up a
    // reaction.
    for (int i = 0; i < tasks.size(); i++) {
      auto& task = tasks[i];
      auto& returnVal = setTasksMsg.response.success[i];

      if (task.monitored && returnVal) {
        task_name_reaction_map_[task.name] = tdr_vector[i];
        task_name_etol_map_[task.name] = etol_vector[i];
      }
    }
  } else
    ROS_WARN("set_tasks service call failed.");
}

void HiQPClient::removeTask(const std::string& task_name) {
  ROS_INFO("Removing Task: %s...", task_name.c_str());
  hiqp_msgs::RemoveTask removeTaskMsg;
  removeTaskMsg.request.name = task_name;

  if (!remove_task_client_.call(removeTaskMsg)) {
    ROS_WARN("Removing task \'%s\' failed. See server output/log for details.",
             task_name.c_str());
  }
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

void HiQPClient::taskMeasuresCallback(
    const hiqp_msgs::TaskMeasuresConstPtr& task_measures) {
  for (auto task_measure : task_measures->task_measures) {
    double sq_error =
        std::inner_product(task_measure.e.begin(), task_measure.e.end(),
                           task_measure.e.begin(), 0.0);

    ROS_INFO_DELAYED_THROTTLE(5.0, "[%s] Progress: %lf",
                              task_measure.task_name.c_str(),
                              exp(-sq_error) * 100.0);
    auto it_reaction = task_name_reaction_map_.find(task_measure.task_name);
    auto it_error_tolerance = task_name_etol_map_.find(task_measure.task_name);
    if (sq_error < it_error_tolerance->second) {
      ROS_INFO("Error is %lf for task: %s.\n", sq_error,
               task_measure.task_name.c_str());
      switch (it_reaction->second) {
        case TaskDoneReaction::PRINT_INFO:
          std::cout << task_measure;
          break;
        case TaskDoneReaction::REMOVE:
          removeTask(task_measure.task_name);
          task_name_reaction_map_.erase(it_reaction);
          task_name_etol_map_.erase(it_error_tolerance);
          break;
        case TaskDoneReaction::DEACTIVATE:
          deactivateTask(task_measure.task_name);
          it_reaction->second = TaskDoneReaction::NONE;
          break;
        default:
          continue;
      }
    }
  }
}

void HiQPClient::setJointAngles(const std::vector<double>& joint_angles) {
  // TODO: Read joint angles task error tolerance from param server.
  std::vector<std::string> def_params{"TDefFullPose"};

  for (auto jointValue : joint_angles) {
    def_params.push_back(std::to_string(jointValue));
  }

  this->setTask("joint_angles_task", 3, true, true, true, def_params,
                {"TDynLinear", "0.1"}, TaskDoneReaction::REMOVE, 1e-3);
}

void HiQPClient::waitForCompletion(const std::string& task_name) {
  bool task_completed = false;
  ros::Rate rate(5);
  while (ros::ok() && !task_completed) {
    auto it = task_name_reaction_map_.find(task_name);
    if (it == task_name_reaction_map_.end()) {
      task_completed = true;
    }
    rate.sleep();
  }
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

hiqp_msgs::Primitive createPrimitiveMsg(const std::string& name, const std::string& type,
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

  return primitive;
}
