
#include <iostream>
#include <string>
#include <thread>

#include "ros/ros.h"

#include <hiqp_msgs_srvs/MonitorDataMsg.h>
#include <hiqp_msgs_srvs/StringArray.h>




struct current_shit {
  double e_atstartpos = 0;

  double e_underplane = 0;
  double e_abovefloor = 0;
  double e_atcylinder = 0;
};



void monitoringDataCallback(const hiqp_msgs_srvs::MonitorDataMsg& msg)
{
  std::cout << "monitoring";
}

void fsm(ros::NodeHandle *n)
{
  ros::Publisher publisher = n->advertise<hiqp_msgs_srvs::StringArray>
    ("/yumi/hiqp_kinematics_controller/experiment_commands", 1000);

  std::string input;
  std::cout << "input: ";
  std::cin >> input;

  while (input.compare("quit") != 0)
  {
    hiqp_msgs_srvs::StringArray msg;
    msg.params.push_back("set_cyl_pos");
    msg.params.push_back("0.6");
    msg.params.push_back("0.6");
    publisher.publish(msg);

    std::cout << "input: ";
    std::cin >> input;
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








