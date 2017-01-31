#include <hiqp/hiqp_client.h>

namespace hiqp {

HiQPClient::HiQPClient(const std::string& controller_namespace, bool auto_connect) :
  nh_(controller_namespace) {
  
  if(auto_connect)
    this->connectToServer();
}

void HiQPClient::connectToServer() {
  set_task_client_ = nh_.serviceClient<hiqp_msgs::SetTask>("set_task");
  set_primitive_client_ = nh_.serviceClient<hiqp_msgs::SetPrimitive>("set_primitive");
  ROS_INFO("Connected to HiQP Servers.");
}

void HiQPClient::setPrimitives(const std::vector <hiqp_msgs::Primitive>& primitives) {

  hiqp_msgs::SetPrimitive setPrimitiveMsg;
  setPrimitiveMsg.request.primitives = primitives;
  
  if(set_primitive_client_.call(setPrimitiveMsg)) {

    int returnValue = std::accumulate(setPrimitiveMsg.response.success.begin(), setPrimitiveMsg.response.success.end(), 0);
    if(returnValue == 0) 
      ROS_INFO("Operation succeeded.");
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

  std::vector <hiqp_msgs::Primitive> primitives;
  primitives.resize(1);
  hiqp_msgs::Primitive& primitive = primitives[0];
  
  primitive.name = name;
  primitive.type = type;
  primitive.frame_id = frame_id;
  primitive.visible = visible;
  primitive.color = color;
  primitive.parameters = parameters;
  
  setPrimitives(primitives);
}
  
  
void HiQPClient::setTask (const std::string& name,
                          int16_t priority,
                          bool visible,
                          bool active,
                          bool monitored,
                          const std::vector <std::string>& def_params,
                          const std::vector <std::string>& dyn_params) {

  // TODO: Must implement checks to validate message.
  hiqp_msgs::SetTask setTaskMsg;
  hiqp_msgs::SetTask::Request& req = setTaskMsg.request;
  req.name = name;
  req.priority = priority;
  req.visible = visible;
  req.active = active;
  req.monitored = monitored;
  req.def_params = def_params;
  req.dyn_params = dyn_params;
  
  ROS_INFO("Adding a new task.");
  if(set_task_client_.call(setTaskMsg)) {
    ROS_INFO("Operation %s", setTaskMsg.response.success ? "succeeded." : "failed.");
  }
  else
    ROS_WARN("set_task service call failed.");
}
  
}
