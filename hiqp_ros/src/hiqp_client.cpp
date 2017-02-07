#include <hiqp_ros/hiqp_client.h>

namespace hiqp_ros {

HiQPClient::HiQPClient(const std::string& controller_namespace,
                       bool auto_connect)
  : nh_(controller_namespace),
    error_tolerance_(0.001) {
  // TODO: read error_tolerance from file.
  if (auto_connect) this->connectToServer();
}

void HiQPClient::connectToServer() {
  set_tasks_client_ = nh_.serviceClient<hiqp_msgs::SetTasks>("set_tasks");
  set_primitives_client_ =
      nh_.serviceClient<hiqp_msgs::SetPrimitives>("set_primitives");
  activate_task_client_ = nh_.serviceClient<hiqp_msgs::ActivateTask>("activate_task");
  deactivate_task_client_ = nh_.serviceClient<hiqp_msgs::DeactivateTask>("deactivate_task");
  remove_task_client_ = nh_.serviceClient<hiqp_msgs::RemoveTask>("remove_task");
  remove_primitive_client_ = nh_.serviceClient<hiqp_msgs::RemovePrimitive> ("remove_primitive");

  task_measures_sub_ = nh_.subscribe("task_measures", 1, &HiQPClient::taskMeasuresCallback, this);
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
                         TaskDoneReaction tdr) {
  hiqp_msgs::Task task;

  task.name = name;
  task.priority = priority;
  task.visible = visible;
  task.active = active;
  task.monitored = monitored;
  task.def_params = def_params;
  task.dyn_params = dyn_params;

  std::vector<hiqp_msgs::Task> tasks {task};
  std::vector<TaskDoneReaction> tdr_vector {tdr};

  setTasks(tasks, tdr_vector);
}

void HiQPClient::setTasks(const std::vector<hiqp_msgs::Task>& tasks,
                          const std::vector<TaskDoneReaction>& tdr_vector) {
  hiqp_msgs::SetTasks setTasksMsg;
  setTasksMsg.request.tasks = tasks;

  if (set_tasks_client_.call(setTasksMsg)) {
    int returnValue = std::accumulate(setTasksMsg.response.success.begin(),
                                      setTasksMsg.response.success.end(), 0);

    if (returnValue == setTasksMsg.response.success.size())
      ROS_INFO("Set task(s) succeeded.");
    else
      ROS_WARN("Either all or some of the tasks were not added.");

    // If the service call succeeded and this task was added. We can set up a reaction.
    for(int i = 0; i < tasks.size(); i++) {
      auto& task = tasks[i];
      auto& returnVal = setTasksMsg.response.success[i];
      
      if(task.monitored && returnVal) {
        ROS_INFO("Ok here. %d", tdr_vector[i]);
        task_name_reaction_map_[task.name] = tdr_vector[i];
      }
    }
  } else
    ROS_WARN("set_tasks service call failed.");
}

void HiQPClient::removeTask(const std::string& task_name) {
  ROS_INFO("Removing Task: %s...", task_name.c_str());
  hiqp_msgs::RemoveTask removeTaskMsg;
  removeTaskMsg.request.name = task_name;

  if(!remove_task_client_.call(removeTaskMsg)) {
    ROS_WARN("Removing task \'%s\' failed. See server output/log for details.", task_name.c_str());
  }
}

void HiQPClient::deactivateTask(const std::string& task_name) {
  ROS_INFO("Deactivating Task: %s...", task_name.c_str());
  hiqp_msgs::DeactivateTask deactivateTaskMsg;
  deactivateTaskMsg.request.name = task_name;

  if(!deactivate_task_client_.call(deactivateTaskMsg)) {
    ROS_WARN("Deactivating task \'%s\' failed. See server output/log for details.", task_name.c_str());
  }
}

void HiQPClient::taskMeasuresCallback(const hiqp_msgs::TaskMeasuresConstPtr& task_measures) {
  for(auto task_measure : task_measures->task_measures) {
    double sq_error = std::inner_product(task_measure.e.begin(), task_measure.e.end(), task_measure.e.begin(), 0);
    auto it = task_name_reaction_map_.find(task_measure.task_name);
    if(sq_error < error_tolerance_) {
      ROS_INFO("Error tolerance reached for task: %s.\n", task_measure.task_name.c_str());
      switch(it->second) {
      case TaskDoneReaction::PRINT_INFO:
        std::cout << task_measure;
        break;
      case TaskDoneReaction::REMOVE:
        removeTask(task_measure.task_name);
        task_name_reaction_map_.erase(it);
        break;
      case TaskDoneReaction::DEACTIVATE:
        deactivateTask(task_measure.task_name);
        it->second = TaskDoneReaction::NONE;
        break;
      default:
        continue;
      }
    }
  }
}

}
