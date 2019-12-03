#pragma once

#include<vector>
#include<string>

namespace hiqp {

struct PrimitiveInfo {
  PrimitiveInfo(const std::string& _name, const std::string& _type,
                const std::string& _frame_id, bool _visible,
                const std::vector<double>& _color,
                const std::vector<double>& _parameters)
      : name(_name),
        type(_type),
        frame_id(_frame_id),
        visible(_visible),
        color(_color),
        parameters(_parameters) {}

  std::string name;
  std::string type;
  std::string frame_id;
  bool visible;
  std::vector<double> color;
  std::vector<double> parameters;

};  // struct PrimitiveInfo

}  // namespace hiqp
