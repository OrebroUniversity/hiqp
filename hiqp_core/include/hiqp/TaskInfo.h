#pragma once

namespace hiqp {

struct TaskInfo {
  TaskInfo(const std::string& _name,
           bool _priority,
           bool _active,
           bool _monitored,
           const std::vector <std::string>& _def_params,
           const std::vector <std::string>& _dyn_params) :
    name(_name),
    priority(_priority),
    active(_active),
    monitored(_monitored),
    def_params(_def_params),
    dyn_params(_dyn_params) {}

  std::string name;
  bool priority;
  bool active;
  bool monitored;
  std::vector <std::string> def_params;
  std::vector <std::string> dyn_params;
  
}; // struct TaskInfo

} // namespace hiqp
