
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <thread>
#include <atomic>

#include "ros/ros.h"

#include <hiqp_msgs_srvs/MonitorDataMsg.h>
#include <hiqp_msgs_srvs/StringArray.h>

void splitString(const std::string& s, std::vector<std::string>& v)
{
  std::istringstream iss(s);
  v.clear();
  std::copy(std::istream_iterator<std::string>(iss),
            std::istream_iterator<std::string>(),
            std::back_inserter(v));
}

struct MonitoredAtomics {
  std::atomic<double> e_atstartpos_;
  std::atomic<double> e_underplane_;
  std::atomic<double> e_abovefloor_;
  std::atomic<double> e_atcylinder_;

  MonitoredAtomics() : e_atstartpos_(0), e_underplane_(0), e_abovefloor_(0), e_atcylinder_(0) {}

  inline void setStartPos(double val) { e_atstartpos_.store(val, std::memory_order_relaxed); }

  inline double getStartPos() { return e_atstartpos_.load(std::memory_order_relaxed); }

  inline void setUnderPlane(double val) { e_underplane_.store(val, std::memory_order_relaxed); }

  inline double getUnderPlane() { return e_underplane_.load(std::memory_order_relaxed); }

  inline void setAboveFloor(double val) { e_abovefloor_.store(val, std::memory_order_relaxed); }

  inline double getAboveFloor() { return e_abovefloor_.load(std::memory_order_relaxed); }

  inline void setAtCylinder(double val) { e_atcylinder_.store(val, std::memory_order_relaxed); }

  inline double getAtCylinder() { return e_atcylinder_.load(std::memory_order_relaxed); }
};

MonitoredAtomics VALUES;










void monitoringDataCallback(const hiqp_msgs_srvs::MonitorDataMsg& msg)
{
  for (std::vector<hiqp_msgs_srvs::PerfMeasMsg>::const_iterator it = msg.data.cbegin();
       it != msg.data.cend();
       ++it)
  {
    if (it->measure_tag.compare("e") == 0) {
      if (it->task_name.compare("bring_back_to_start") == 0) {
        VALUES.setStartPos(it->data.at(0));
      } else if (it->task_name.compare("bring_gripper_point_under_plane") == 0) {
        VALUES.setUnderPlane(it->data.at(0));
      } else if (it->task_name.compare("bring_gripper_point_above_floor") == 0) {
        VALUES.setAboveFloor(it->data.at(0));
      } else if (it->task_name.compare("bring_gripper_point_to_cylinder") == 0) {
        VALUES.setAtCylinder(it->data.at(0));
      }
    }
  }
}





/*
      The starting position grid:
         x = 0.2
         y = [-0.45, -0.225, 0, 0.225, 0.45]
         z = [0.25, 0.425, 0.6, 0.775, 0.95]
*/
void fsm(ros::NodeHandle *n)
{
  ros::Publisher publisher = n->advertise<hiqp_msgs_srvs::StringArray>
    ("/yumi/hiqp_kinematics_controller/experiment_commands", 1000);

  std::string input;
  std::cout << "input: ";
  std::getline(std::cin, input);

  std::vector<std::string> commands;

  while (input.compare("quit") != 0)
  {
    if (input.compare("m") == 0)
    {
      std::cout << "e_atstartpos_ = " << VALUES.getStartPos() << "\n";
      std::cout << "e_underplane_ = " << VALUES.getUnderPlane() << "\n";
      std::cout << "e_abovefloor_ = " << VALUES.getAboveFloor() << "\n";
      std::cout << "e_atcylinder_ = " << VALUES.getAtCylinder() << "\n";
    }

    splitString(input, commands);
    hiqp_msgs_srvs::StringArray msg;
    for (auto&& command : commands)
    {
      msg.params.push_back(command);
    }
    publisher.publish(msg);

    std::cout << "input: ";
    std::getline(std::cin, input);
  }
}






void monitor(ros::NodeHandle *n)
{
  ros::Subscriber subscriber = n->subscribe(
    "/yumi/hiqp_kinematics_controller/monitoring_data", 1000, monitoringDataCallback);
  ros::spin();
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "hiqp_experiment_rosnode");
  ros::NodeHandle n;
  
  std::thread first(fsm, &n);
  std::thread second(monitor, &n);

  first.join();

  return 0;
}








