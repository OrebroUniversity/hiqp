#include <hiqp/hiqp_client.h>

namespace hiqp {

HiQPClient::HiQPClient(const std::string& controller_namespace, bool auto_connect) :
  nh_(controller_namespace) {
  
  if(auto_connect)
    this->connectToServer();
}

void HiQPClient::connectToServer() {
  set_tasks_client_ = nh_.serviceClient<hiqp_msgs::SetTasks>("set_tasks");
  set_primitives_client_ = nh_.serviceClient<hiqp_msgs::SetPrimitives>("set_primitives");
  ROS_INFO("Connected to HiQP Servers.");
}

void HiQPClient::setPrimitives(const std::vector <hiqp_msgs::Primitive>& primitives) {

  hiqp_msgs::SetPrimitives setPrimitivesMsg;
  setPrimitivesMsg.request.primitives = primitives;
  
  if(set_primitives_client_.call(setPrimitivesMsg)) {

    int returnValue = std::accumulate(setPrimitivesMsg.response.success.begin(), setPrimitivesMsg.response.success.end(), 0);

    if(returnValue == setPrimitivesMsg.response.success.size()) 
      ROS_INFO("Set primitive(s) succeeded.");
    else
      ROS_WARN("Either all or some of the primitives were not added.");
  } 
  else
    ROS_WARN("set_primitive service call failed.");  
}

void HiQPClient::setPrimitive (const std::string& name,
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

  std::vector <hiqp_msgs::Primitive> primitives { primitive };
  
  setPrimitives(primitives);
}
  
  
void HiQPClient::setTask (const std::string& name,
                          int16_t priority,
                          bool visible,
                          bool active,
                          bool monitored,
                          const std::vector <std::string>& def_params,
                          const std::vector <std::string>& dyn_params) {


  hiqp_msgs::Task task;

  task.name = name;
  task.priority = priority;
  task.visible = visible;
  task.active = active;
  task.monitored = monitored;
  task.def_params = def_params;
  task.dyn_params = dyn_params;

  std::vector <hiqp_msgs::Task> tasks { task };
  
  setTasks(tasks);
}

void HiQPClient::setTasks (const std::vector <hiqp_msgs::Task>& tasks) {
  hiqp_msgs::SetTasks setTasksMsg;
  setTasksMsg.request.tasks = tasks;

  if(set_tasks_client_.call(setTasksMsg)) {
    
    int returnValue = std::accumulate(setTasksMsg.response.success.begin(), setTasksMsg.response.success.end(), 0);

    if(returnValue == setTasksMsg.response.success.size()) 
      ROS_INFO("Set task(s) succeeded.");
    else
      ROS_WARN("Either all or some of the tasks were not added.");
  } 
  else
    ROS_WARN("set_tasks service call failed.");  
}
  
}
