// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <hiqp/geometric_primitives/geometric_primitive_couter.h>
#include <hiqp/geometric_primitives/geometric_primitive_visualizer.h>
#include <hiqp/task_manager.h>
#include <hiqp/utilities.h>
#include <ros/console.h>
#include <iomanip>  // std::setw
#include <iostream>
#include <fstream>
#include <queue>

#ifdef HIQP_CASADI
#include <hiqp/solvers/casadi_solver.h>
#endif
#ifdef HIQP_GUROBI
#include <hiqp/solvers/gurobi_solver.h>
#include <hiqp/solvers/gurobi_solver_cascade.h>
#include <hiqp/solvers/rp_solver.h>
#endif

#include <Eigen/Dense>

using hiqp::geometric_primitives::GeometricPrimitiveVisualizer;
using hiqp::geometric_primitives::GeometricPrimitiveCouter;

namespace hiqp {

TaskManager::TaskManager(std::shared_ptr<Visualizer> visualizer)
    : visualizer_(visualizer) {
  geometric_primitive_map_ = std::make_shared<GeometricPrimitiveMap>();
#ifdef HIQP_CASADI
  solver_ = std::make_shared<CasadiSolver>();
#endif
#ifdef HIQP_GUROBI
  solver_ = std::make_shared<RPSolver>();
  //solver_ = std::make_shared<GurobiSolver>();
  //solver_ = std::make_shared<GurobiSolverCascade>();
#endif
}

TaskManager::~TaskManager() noexcept {}

void TaskManager::init(unsigned int n_controls, bool beVelControl) { 
  n_controls_ = n_controls; 
  for (int i = 0; i < n_controls_; ++i) controls_.push_back(0);

  solver_->setVelocityControlMode(beVelControl);
}

bool TaskManager::getVelocityControls(RobotStatePtr robot_state,
                                        std::vector<double> &controls) {
    if (task_map_.size() < 1) {
      for (int i=0; i<controls.size(); ++i)
        controls.at(i) = 0;
      
      return false;
    }

    resource_mutex_.lock();
    solver_->clearStages();
    
    //clear the task map from prior iteration
    robot_state->task_status_map_.clear();
   
    auto cmp = [](std::shared_ptr<Task> left, std::shared_ptr<Task> right) { return left->getPriority() > right->getPriority(); };
    std::priority_queue<std::shared_ptr<Task>, std::vector<std::shared_ptr<Task> >, decltype(cmp) > task_queue(cmp);
 
    bool dump=false;
    for (auto&& kv : task_map_) {
      if (kv.second->getActive()) {
          task_queue.push(kv.second);
          dump = dump || (kv.first == "ee_rl");
      }
    }
   
    while(!task_queue.empty()) {
        std::shared_ptr<Task> task = task_queue.top();
        if (task->update(robot_state) == 0) {
          solver_->appendVelocityStage(task->getPriority(),
			     task->getDynamics(),
                             task->getJacobian(),
                             task->getTaskTypes());
	  ///TSV: added below to enable anyone with a robot state to check what tasks are running
	  ///     Logic being: some tasks (RL in particular) can use information about the null-space
	  ///     of higher priority tasks to decide on actions
	  //fill in task state
	  TaskStatus state;
	  state.priority_ = task->getPriority();
	  state.J_ = task->getJacobian();
	  state.e_ = task->getValue();
	  state.de_ = task->getValueDerivative();
	  state.dde_star_ = task->getDynamics();
	  state.task_signs_ = task->getTaskTypes();
	  //push it into the buffer
	  robot_state->task_status_map_.push_back(state);
          
	  //DEBUG ==================================================
	  //std::cerr<<"Task: "<<task->getTaskName()<<" at prio "<<task->getPriority()<<std::endl;
	  //std::cerr<<"task map is now "<<robot_state->task_status_map_.size()<<" tasks long\n";
	  // std::cerr<<"J_: "<<std::endl<<task->getJacobian()<<std::endl;
	  // std::cerr<<"J_dot_: "<<std::endl<< task->getJacobianDerivative()<<std::endl;
	  //std::cerr<<"e_: "<<task->getValue().transpose()<<std::endl;
	  // std::cerr<<"e_dot_: "<<task->getValueDerivative().transpose()<<std::endl;
	  // std::cerr<<"dde_star: "<<task->getDynamics().transpose()<<std::endl;
	  // std::cerr<<"dq: "<<robot_state->kdl_jnt_array_vel_.qdot.data.transpose()<<std::endl;
          // std::cerr<<"q: "<<robot_state->kdl_jnt_array_vel_.q.data.transpose()<<std::endl;
	  // std::cerr<<"__________________________________________________________"<<std::endl<<std::endl;
	  //DEBUG END ===============================================
        }
      task_queue.pop();  
    }
    
    if (!solver_->solve(controls_)) {
      ROS_WARN_THROTTLE(10, "Unable to solve the hierarchical QP, setting accelerations to zero ");
 
      double dt=robot_state->sampling_time_;
      for (int i = 0; i < controls.size(); ++i){
        controls.at(i) = 0.0;
      }
      controls_ = controls;
   
      resource_mutex_.unlock();
      return false;
    }
    controls = controls_;
 
    robot_state->ddq_star = controls;
    resource_mutex_.unlock();
    return true;
  }


bool TaskManager::getAccelerationControls(RobotStatePtr robot_state,
                                      std::vector<double>& controls) {
  if (task_map_.size() < 1) {
    for (int i = 0; i < controls.size(); ++i) controls.at(i) = 0;
    controls_ = controls;
    return false;
  }

  resource_mutex_.lock();

  //TSV: moved solver access within mutex
  solver_->clearStages();
  
  //clear the task map from prior iteration
  robot_state->task_status_map_.clear();

  auto cmp = [](std::shared_ptr<Task> left, std::shared_ptr<Task> right) { return left->getPriority() > right->getPriority(); };
  std::priority_queue<std::shared_ptr<Task>, std::vector<std::shared_ptr<Task> >, decltype(cmp) > task_queue(cmp);

  bool dump=false;
  for (auto&& kv : task_map_) {
    if (kv.second->getActive()) {
	task_queue.push(kv.second);
	dump = dump || (kv.first == "ee_rl");
    }
  }

  while(!task_queue.empty()) {
      std::shared_ptr<Task> task = task_queue.top();
      if (task->update(robot_state) == 0) {
        solver_->appendStage(task->getPriority(),
			     task->getDynamics(),
                             task->getJacobian(),
			     task->getJacobianDerivative(),
			     robot_state->kdl_jnt_array_vel_.qdot,
                             task->getTaskTypes());

	///TSV: added below to enable anyone with a robot state to check what tasks are running
	///     Logic being: some tasks (RL in particular) can use information about the null-space
	///     of higher priority tasks to decide on actions
	//fill in task state
	TaskStatus state;
	state.priority_ = task->getPriority();
	state.J_ = task->getJacobian();
	state.dJ_ = task->getJacobianDerivative();
	state.e_ = task->getValue();
	state.de_ = task->getValueDerivative();
	state.dde_star_ = task->getDynamics();
	state.task_signs_ = task->getTaskTypes();
	//push it into the buffer
	robot_state->task_status_map_.push_back(state);

	//DEBUG ==================================================
	//std::cerr<<"Task: "<<task->getTaskName()<<" at prio "<<task->getPriority()<<std::endl;
	//std::cerr<<"task map is now "<<robot_state->task_status_map_.size()<<" tasks long\n";
	// std::cerr<<"J_: "<<std::endl<<task->getJacobian()<<std::endl;
	// std::cerr<<"J_dot_: "<<std::endl<< task->getJacobianDerivative()<<std::endl;
	//std::cerr<<"e_: "<<task->getValue().transpose()<<std::endl;
	// std::cerr<<"e_dot_: "<<task->getValueDerivative().transpose()<<std::endl;
	// std::cerr<<"dde_star: "<<task->getDynamics().transpose()<<std::endl;
	// std::cerr<<"dq: "<<robot_state->kdl_jnt_array_vel_.qdot.data.transpose()<<std::endl;
        // std::cerr<<"q: "<<robot_state->kdl_jnt_array_vel_.q.data.transpose()<<std::endl;
	// std::cerr<<"__________________________________________________________"<<std::endl<<std::endl;
#if 0
	KDL::JntArray qdot__ = robot_state->kdl_jnt_array_vel_.qdot;
	KDL::JntArray q__ = robot_state->kdl_jnt_array_vel_.q;
	unsigned int q_nr=qdot__.rows();
	std::ofstream dq;
	std::ofstream q;
	std::ofstream de;
	std::ofstream e;
	std::ofstream dde_star;
	std::ofstream J;
	std::ofstream dJ;
	dq.open ("/home/tsv/hiqp_logs/"+task->getTaskName()+"_dq.dat", std::ios::out | std::ios::app );
	q.open ("/home/tsv/hiqp_logs/"+task->getTaskName()+"_q.dat", std::ios::out | std::ios::app );
	de.open ("/home/tsv/hiqp_logs/"+task->getTaskName()+"_de.dat", std::ios::out | std::ios::app );
	e.open ("/home/tsv/hiqp_logs/"+task->getTaskName()+"_e.dat", std::ios::out | std::ios::app );
	dde_star.open ("/home/tsv/hiqp_logs/"+task->getTaskName()+"_dde_star.dat", std::ios::out | std::ios::app );
	J.open ("/home/tsv/hiqp_logs/"+task->getTaskName()+"_J.dat", std::ios::out | std::ios::app );
	dJ.open ("/home/tsv/hiqp_logs/"+task->getTaskName()+"_dJ.dat", std::ios::out | std::ios::app );	  
	for(unsigned int i=0; i<q_nr; i++){
	  dq<<qdot__(i)<<" ";
          q<<q__(i)<<" ";	    
	}
	e<<task->getValue().transpose()<<"\n";
        de<<task->getValueDerivative().transpose()<<"\n";
        dde_star<<task->getDynamics().transpose()<<"\n";
	J<<task->getJacobian()<<"\n"<<"\n";
	dJ<<task->getJacobianDerivative()<<"\n"<<"\n";	  
	dq<<"\n";
	q<<"\n";	  
	dq.close();
	q.close();
	e.close();
	de.close();
	dde_star.close();
	J.close();
	dJ.close();
#endif

	//DEBUG END ===============================================
      }
      task_queue.pop();  
  }
  
  if (!solver_->solve(controls_)) {
    ROS_WARN_THROTTLE(10, "Unable to solve the hierarchical QP, setting accelerations to zero ");

    double dt=robot_state->sampling_time_;
    for (int i = 0; i < controls.size(); ++i){
      controls.at(i) = 0.0;
    }
    controls_ = controls;

    resource_mutex_.unlock();
    return false;
  }
  controls = controls_;

  robot_state->ddq_star = controls;
  resource_mutex_.unlock();

 
   #if 0
   if(dump) { 
   //  DEBUG ==================================================
   std::ofstream u;
   u.open ("/home/tsv/hiqp_logs/ddq_star.dat", std::ios::out | std::ios::app );
   for (int i = 0; i < controls.size(); ++i) u<<controls.at(i)<<" ";
   u<<"\n";	  
   u.close();
   // DEBUG END ===============================================
   }
   #endif

  return true;
}

void TaskManager::getTaskMeasures(std::vector<TaskMeasure>& data) {
  data.clear();
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getMonitored()) {
      kv.second->monitor();
      data.push_back(TaskMeasure(
          kv.second->getTaskName(),
	  kv.second->getTaskTypes()[0],
          kv.second->getValue(),
          kv.second->getValueDerivative(),
	  kv.second->getDynamics(),
          kv.second->getPerformanceMeasures()));
    }
  }
  resource_mutex_.unlock();
}

void TaskManager::renderPrimitives() {
  resource_mutex_.lock();
  GeometricPrimitiveVisualizer geom_prim_vis(visualizer_, 0);
  geometric_primitive_map_->acceptVisitor(geom_prim_vis);
  resource_mutex_.unlock();
}

int TaskManager::setTask(const std::string& task_name, unsigned int priority,
                         bool visible, bool active, bool monitored,
                         const std::vector<std::string>& def_params,
                         const std::vector<std::string>& dyn_params,
                         RobotStatePtr robot_state) {
  resource_mutex_.lock();
  std::shared_ptr<Task> task;
  std::string action = "Added";

  TaskMap::iterator it = task_map_.find(task_name);
  if (it == task_map_.end()) {
    task = std::make_shared<Task>(geometric_primitive_map_, visualizer_,
                                  n_controls_);
  } else {
    task = it->second;
    action = "Updated";
  }

  task->setTaskName(task_name);
  task->setPriority(priority);
  task->setVisible(visible);
  task->setActive(active);
  task->setMonitored(monitored);

  if (task->init(def_params, dyn_params, robot_state) != 0) {
    printHiqpWarning("The task '" + task_name + "' was not added!");

    resource_mutex_.unlock();
    return -1;
  } else {
    task_map_.emplace(task_name, task);
    printHiqpInfo(action + " task '" + task_name + "'");
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::removeTask(std::string task_name) {
  resource_mutex_.lock();
  if (task_map_.erase(task_name) == 1) {
    geometric_primitive_map_->removeDependency(task_name);
    resource_mutex_.unlock();
    return 0;
  }
  resource_mutex_.unlock();
  return -1;
}

int TaskManager::removeAllTasks() {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.begin();
  while (it != task_map_.end()) {
    geometric_primitive_map_->removeDependency(it->first);
    ++it;
  }
  task_map_.clear();
  resource_mutex_.unlock();
  return 0;
}

std::vector<TaskInfo> TaskManager::getAllTaskInfo() {
  resource_mutex_.lock();
  std::vector<TaskInfo> all_task_info;
  for (auto it : task_map_) {
    all_task_info.push_back(
        TaskInfo(it.second->getTaskName(), it.second->getPriority(),
                 it.second->getActive(), it.second->getMonitored(),
                 it.second->getDefParams(), it.second->getDynParams()));
  }

  resource_mutex_.unlock();
  return all_task_info;
}

int TaskManager::listAllTasks() {
  resource_mutex_.lock();
  int longest_name_length = 0;
  TaskMap::iterator it = task_map_.begin();
  while (it != task_map_.end()) {
    if (it->first.size() > longest_name_length)
      longest_name_length = it->first.size();
    ++it;
  }

  std::multimap<unsigned int, std::string> task_info_map;
  it = task_map_.begin();
  while (it != task_map_.end()) {
    std::stringstream ss;
    ss << std::setw(8) << it->second->getPriority() << " | "
       << std::setw(longest_name_length) << it->first << " | " << std::setw(6)
       << it->second->getActive() << " | " << std::setw(9)
       << it->second->getMonitored();
    task_info_map.emplace(it->second->getPriority(), ss.str());
    ++it;
  }
  resource_mutex_.unlock();

  std::cout << " - - - LISTING ALL REGISTERED TASKS - - -\n";
  std::cout << "Priority | Unique name";
  for (int i = 0; i < longest_name_length - 11; ++i) std::cout << " ";
  std::cout << " | Active | Monitored\n";
  std::cout << "----------------------";
  for (int i = 0; i < longest_name_length - 11; ++i) std::cout << "-";
  std::cout << "---------------------\n";

  for (auto&& info : task_info_map) {
    std::cout << info.second << "\n";
  }

  return 0;
}

int TaskManager::activateTask(const std::string& task_name) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.find(task_name);
  if (it != task_map_.end()) {
    it->second->setActive(true);
  } else {
    printHiqpWarning("When trying to activate task '" + task_name +
                     "': No task with that name found.");
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::deactivateTask(const std::string& task_name) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.find(task_name);
  if (it != task_map_.end()) {
    it->second->setActive(false);
  } else {
    printHiqpWarning("When trying to deactivate task '" + task_name +
                     "': No task with that name found.");
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::monitorTask(const std::string& task_name) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.find(task_name);
  if (it != task_map_.end()) {
    it->second->setMonitored(true);
  } else {
    printHiqpWarning("When trying to activate monitoring of task '" +
                     task_name + "': No task with that name found.");
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::demonitorTask(const std::string& task_name) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.find(task_name);
  if (it != task_map_.end()) {
    it->second->setMonitored(false);
  } else {
    printHiqpWarning("When trying to deactivate monitoring of task '" +
                     task_name + "': No task with that name found.");
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::setPrimitive(const std::string& name, const std::string& type,
                              const std::string& frame_id, bool visible,
                              const std::vector<double>& color,
                              const std::vector<double>& parameters) {
  resource_mutex_.lock();
  geometric_primitive_map_->setGeometricPrimitive(name, type, frame_id, visible,
                                                  color, parameters);
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::removePrimitive(std::string name) {
  resource_mutex_.lock();
  GeometricPrimitiveVisualizer geom_prim_vis(visualizer_, 1);
  geometric_primitive_map_->acceptVisitor(geom_prim_vis, name);
  geom_prim_vis.removeAllVisitedPrimitives();
  geometric_primitive_map_->removeGeometricPrimitive(name);
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::removeAllPrimitives() {
  resource_mutex_.lock();
  GeometricPrimitiveVisualizer geom_prim_vis(visualizer_, 1);
  geometric_primitive_map_->acceptVisitor(geom_prim_vis);
  geom_prim_vis.removeAllVisitedPrimitives();
  geometric_primitive_map_->clear();
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::listAllPrimitives() {
  resource_mutex_.lock();
  std::cout << "LISTING ALL REGISTERED GEOMETRIC PRIMITIVES:\n";
  std::cout << "Name | Frame ID | Visible | Visual ID | Type\n";
  GeometricPrimitiveCouter geom_prim_cout;
  geometric_primitive_map_->acceptVisitor(geom_prim_cout);
  resource_mutex_.unlock();
  return 0;
}

std::vector<PrimitiveInfo> TaskManager::getAllPrimitiveInfo() {
  resource_mutex_.lock();
  std::vector<PrimitiveInfo> all_primitive_info;
  all_primitive_info = this->geometric_primitive_map_->getAllPrimitiveInfo();
  resource_mutex_.unlock();
  return all_primitive_info;
}

int TaskManager::removePriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  TaskMap::iterator it = task_map_.begin();
  while (it != task_map_.end()) {
    if (it->second->getPriority() == priority) {
      geometric_primitive_map_->removeDependency(it->second->getTaskName());
      it = task_map_.erase(it);
    } else {
      ++it;
    }
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::activatePriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getPriority() == priority) kv.second->setActive(true);
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::deactivatePriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getPriority() == priority) kv.second->setActive(false);
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::monitorPriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getPriority() == priority) kv.second->setMonitored(true);
  }
  resource_mutex_.unlock();
  return 0;
}

int TaskManager::demonitorPriorityLevel(unsigned int priority) {
  resource_mutex_.lock();
  for (auto&& kv : task_map_) {
    if (kv.second->getPriority() == priority) kv.second->setMonitored(false);
  }
  resource_mutex_.unlock();
  return 0;
}

}  // namespace hiqp
