
#include <iostream>
#include <fstream>
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





std::vector<std::string> explode(const std::string& str, const char& ch) {
    std::string next;
    std::vector<std::string> result;

    // For each character in the string
    for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
        // If we've hit the terminal character
        if (*it == ch) {
            // If we have some characters accumulated
            if (!next.empty()) {
                // Add them to the result vector
                result.push_back(next);
                next.clear();
            }
        } else {
            // Accumulate the next character into the sequence
            next += *it;
        }
    }
    if (!next.empty())
         result.push_back(next);
    return result;
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



void manualInput(ros::NodeHandle *n)
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


struct TestCase {
  TestCase(double sx, double sy, double sz, double cx, double cy, double cz)
  : spx(sx), spy(sy), spz(sz), cpx(cx), cpy(cy), cpz(cz)
  {}

  double spx;
  double spy;
  double spz;

  double cpx;
  double cpy;
  double cpz;
};

std::vector<TestCase>::iterator findTestCase(std::vector<TestCase>& test_cases, double sy, double sz, double cx, double cy)
{
  std::vector<TestCase>::iterator it = test_cases.begin();
  for (; it != test_cases.end(); ++it) {
    if (it->spy == sy && it->spz == sz && it->cpx == cx && it->cpy == cy)
      break;
  }
  return it;
}

/*
      The starting position grid:
         x = 0.2
         y = [-0.45, -0.225, 0, 0.225, 0.45]
         z = [0.25, 0.425, 0.6, 0.775, 0.95]

      The cylinder position grid:
         radius: 0.033
         height: 0.1
         x = [0.077, 0.191, 0.305, 0.419, 0.533]
         y = [-0.533, -0.3415, -0.15, 0.0415, 0.233]
         z = 0.115
*/
void fsm(ros::NodeHandle *n)
{
  ros::Publisher publisher = n->advertise<hiqp_msgs_srvs::StringArray>
    ("/yumi/hiqp_kinematics_controller/experiment_commands", 1000);






  // generate all test cases
  double startpos_x    = 0.2;
  double startpos_y[5] = {-0.45, -0.225, 0, 0.225, 0.45};
  double startpos_z[5] = {0.25, 0.425, 0.6, 0.775, 0.95};

  double cylpos_x[5] = {0.077, 0.191, 0.305, 0.419, 0.533};
  double cylpos_y[5] = {-0.533, -0.3415, -0.15, 0.0415, 0.233};
  double cylpos_z    = 0.115;

  std::vector<TestCase> test_cases;

  for (int spyidx = 0; spyidx < 5; ++spyidx) {
    for (int spzidx = 0; spzidx < 5; ++spzidx) {
      for (int cpxidx = 0; cpxidx < 5; ++cpxidx) {
        for (int cpyidx = 0; cpyidx < 5; ++cpyidx) {
          test_cases.push_back( TestCase(startpos_x,
                                         startpos_y[spyidx],
                                         startpos_z[spzidx],
                                         cylpos_x[cpxidx],
                                         cylpos_y[cpyidx],
                                         cylpos_z) );
        }
      }
    }
  }

  std::cout << "Generated " << test_cases.size() << " test cases.\n";





  // Read old test cases and remove them from the generated ones
  std::ifstream ifs;
  ifs.open("results_old.csv");
  std::string line;
  int n_removals = 0;
  while (std::getline(ifs, line))
  {
    std::vector<std::string> split_line = explode(line, ',');
    std::vector<TestCase>::iterator pos = findTestCase(test_cases,
                                   std::stod(split_line.at(1)),
                                   std::stod(split_line.at(2)),
                                   std::stod(split_line.at(3)),
                                   std::stod(split_line.at(4)));
    test_cases.erase(pos);
    n_removals--;
  }
  ifs.close();
  std::cout << "Removed " << n_removals << " test cases.\n";





  // ready to go
  std::ofstream ofs;
  ofs.open("results_new.csv");

  std::string input;
  std::cout << "input: ";
  std::getline(std::cin, input);
  if (input.compare("start") == 0) {

    double task_value_limit = 0.01;

    for (int i = 0; i<5*5*5*5; ++i)
    {
      double spx = test_cases.at(i).spx;
      double spy = test_cases.at(i).spy;
      double spz = test_cases.at(i).spz;
      double cpx = test_cases.at(i).cpx;
      double cpy = test_cases.at(i).cpy;
      double cpz = test_cases.at(i).cpz;

      // goto start position
      {
        hiqp_msgs_srvs::StringArray msg;
        msg.params.push_back("goto_start_pos");
        msg.params.push_back(std::to_string(spx));
        msg.params.push_back(std::to_string(spy));
        msg.params.push_back(std::to_string(spz));
        publisher.publish(msg);
      }

      std::cout << "sent: goto_start_position\n";
      while (VALUES.getStartPos() > task_value_limit) {}
      ros::Duration(1.5).sleep();
      std::cout << "start position reached\n";

      // set cylinder position
      {
        hiqp_msgs_srvs::StringArray msg;
        msg.params.push_back("set_cyl_pos");
        msg.params.push_back(std::to_string(cpx));
        msg.params.push_back(std::to_string(cpy));
        msg.params.push_back(std::to_string(cpz));
        publisher.publish(msg);
      }

      std::cout << "sent: set_cyl_pos\n";
      ros::Duration(1.5).sleep();
      std::cout << "waited for 0.5 seconds\n";

      // grab cylinder
      {
        hiqp_msgs_srvs::StringArray msg;
        msg.params.push_back("grab_cylinder");
        publisher.publish(msg);
      }

      std::cout << "sent: grab_cylinder\n";

      // await test results
      double e_cyl = 0;
      double t_cyl = 0;

      bool stagnation_reached = false;
      bool success = false;
      ros::Time start_time = ros::Time::now();
      ros::Time end_time;

      while (!stagnation_reached)
      {
        end_time = ros::Time::now();

        if (VALUES.getAtCylinder() < task_value_limit &&
            VALUES.getAboveFloor() > task_value_limit &&
            VALUES.getUnderPlane() < task_value_limit)
        {
          success = true;
          stagnation_reached = true;
        }

        ros::Duration elap_time = end_time - start_time;
        if (elap_time.toSec() >= 10)
        {
          std::cout << "- - - task timeout!\n";
          std::cout << "e_cyl = " << VALUES.getAtCylinder() << "\n";
          std::cout << "e_flo = " << VALUES.getAboveFloor() << "\n";
          std::cout << "e_pla = " << VALUES.getUnderPlane() << "\n";
          stagnation_reached = true;
        }
      }

      ros::Duration(0.5).sleep();
      std::cout << "stagnation reached\n\n";

      ofs << spx << "," << spy << "," << spz << ","
          << cpx << "," << cpy << "," << cpz << ","
          << (success ? "success" : "failure") << std::endl << std::flush;

      ros::Duration(0.25).sleep();
    }

  }

  ofs.close();



  
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








