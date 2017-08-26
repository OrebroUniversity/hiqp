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

#include <hiqp/solvers/gurobi_solver.h>
#include <hiqp/utilities.h>

#include <gurobi_c++.h>
#include <ros/assert.h>
#include <Eigen/Dense>
#include <cassert>
#include <iomanip>
#include <iostream>

#define OUTPUT_FLAG 0
#define PRESOLVE -1
#define OPTIMALITY_TOL 1e-6
#define SCALE_FLAG 1
#define TIME_LIMIT 1.0  // 0.005
#define DUAL_REDUCTIONS 1
#define TIKHONOV_FACTOR 5 * 1e-5

namespace hiqp {

GurobiSolver::GurobiSolver() {
  env_.set(GRB_IntParam_OutputFlag, OUTPUT_FLAG);
  env_.set(GRB_IntParam_Presolve, PRESOLVE);
  env_.set(GRB_DoubleParam_OptimalityTol, OPTIMALITY_TOL);
  env_.set(GRB_IntParam_ScaleFlag, SCALE_FLAG);
  env_.set(GRB_DoubleParam_TimeLimit, TIME_LIMIT);
  env_.set(GRB_IntParam_DualReductions, DUAL_REDUCTIONS);
}

bool GurobiSolver::solve(std::vector<double>& solution) {
  if (stages_map_.empty()) return false;

  n_solution_dims_ = solution.size();
  hqp_constraints_.reset(n_solution_dims_);
  unsigned int current_priority = 0;

  for (auto&& kv : stages_map_) {
    current_priority = kv.first;

    const HiQPStage& current_stage = kv.second;

    //DEBUG =========================================
    // std::cerr<<"Current stage: "<<std::endl;
    // std::cerr<<"stage priority: "<<current_priority<<std::endl;
    // std::cerr<<"B_: "<<std::endl<<current_stage.B_<<std::endl;
    // std::cerr<<"constraint_signs_: ";
    // for (int i=0; i<current_stage.constraint_signs_.size();i++)
    //   std::cerr<<current_stage.constraint_signs_[i]<<" ";

    // std::cerr<<std::endl;
    // std::cerr<<"b_: "<<std::endl<<current_stage.b_<<std::endl;
    //END DEBUG =========================================
	
    hqp_constraints_.appendConstraints(current_stage);
    //DEBUG =========================================
    // std::cerr<<"HQP Constraints: "<<std::endl;
    // std::cerr<<"B_: "<<std::endl<<hqp_constraints_.B_<<std::endl;
    // std::cerr<<"constraint_signs_: ";
    // for (int i=0; i<hqp_constraints_.constraint_signs_.size();i++)
    //   std::cerr<<hqp_constraints_.constraint_signs_[i]<<" ";

    // std::cerr<<std::endl;
    // std::cerr<<"b_: "<<std::endl<<hqp_constraints_.b_<<std::endl;
    // std::cerr<<"w_: "<<std::endl<<hqp_constraints_.w_<<std::endl;
    //END DEBUG =========================================

    QPProblem qp_problem(env_, hqp_constraints_, n_solution_dims_);

    try {
      qp_problem.setup();
    } catch (GRBException e) {
      std::cerr << "In GurobiSolver::QPProblem::setup : Gurobi exception with "
                   "error code "
                << e.getErrorCode() << ", and error message "
                << e.getMessage().c_str() << ".\n";
      return false;
    }

    try {
      if(!qp_problem.solve()) return false;
    } catch (GRBException e) {
      std::cerr << "In GurobiSolver::QPProblem::solve : Gurobi exception with "
                   "error code "
                << e.getErrorCode() << ", and error message "
                << e.getMessage().c_str() << ".\n";
      return false;
    }

    try {
      qp_problem.getSolution(solution);
    } catch (GRBException e) {
      ROS_ERROR_THROTTLE(10,
                                 "In GurobiSolver::QPProblem::getSolution : "
                                 "Gurobi exception with error code: %d, and "
                                 "error_message: %s.",
                                 e.getErrorCode(), e.getMessage().c_str());
      return false;
    }
  }

  return true;
}

GurobiSolver::QPProblem::QPProblem(const GRBEnv& env,
                                   HQPConstraints& hqp_constraints,
                                   unsigned int solution_dims)
    : model_(env),
      hqp_constraints_(hqp_constraints),
      solution_dims_(solution_dims),
      lb_ddq_(nullptr),
      ub_ddq_(nullptr),
      ddq_(nullptr),
      lb_w_(nullptr),
      ub_w_(nullptr),
      w_(nullptr),
      rhsides_(nullptr),
      lhsides_(nullptr),
      coeff_ddq_(nullptr),
      coeff_w_(nullptr),
      constraints_(nullptr) {}

GurobiSolver::QPProblem::~QPProblem() {
  delete[] lb_ddq_;
  delete[] ub_ddq_;
  delete[] ddq_;
  delete[] lb_w_;
  delete[] ub_w_;
  delete[] w_;
  delete[] rhsides_;
  delete[] coeff_ddq_;
  delete[] coeff_w_;
  delete[] lhsides_;
  delete[] constraints_;
}

void GurobiSolver::QPProblem::setup() {
  unsigned int stage_dims = hqp_constraints_.n_stage_dims_;
  unsigned int acc_stage_dims = hqp_constraints_.n_acc_stage_dims_;
  unsigned int total_stage_dims = stage_dims + acc_stage_dims;

  // Allocate and set lower and upper bounds for joint accelerations and slack
  // variables
  lb_ddq_ = new double[solution_dims_];
  ub_ddq_ = new double[solution_dims_];
  std::fill_n(lb_ddq_, solution_dims_, -GRB_INFINITY);
  std::fill_n(ub_ddq_, solution_dims_, GRB_INFINITY);

  lb_w_ = new double[stage_dims];
  ub_w_ = new double[stage_dims];
  std::fill_n(lb_w_, stage_dims, -GRB_INFINITY);
  std::fill_n(ub_w_, stage_dims, GRB_INFINITY);

  ddq_ = model_.addVars(lb_ddq_, ub_ddq_, NULL, NULL, NULL, solution_dims_);
  w_ = model_.addVars(lb_w_, ub_w_, NULL, NULL, NULL, stage_dims);
  model_.update();

  // Allocate and set right-hand-side constants
  rhsides_ = new double[total_stage_dims];
  Eigen::Map<Eigen::VectorXd>(rhsides_, total_stage_dims) =
      hqp_constraints_.b_ + hqp_constraints_.w_;

  // Allocate and set left-hand-side expressions
  lhsides_ = new GRBLinExpr[total_stage_dims];
  coeff_ddq_ = new double[solution_dims_];
  coeff_w_ = new double[stage_dims];

  // lhs corresponding to the previously solved stages (w/o slacks in the decision variables)
  for (unsigned int i = 0; i < acc_stage_dims; ++i) {
    Eigen::Map<Eigen::VectorXd>(coeff_ddq_, solution_dims_) =
        hqp_constraints_.B_.row(i);
    lhsides_[i].addTerms(coeff_ddq_, ddq_, solution_dims_);
  }

  // lhs for the current stage (with slacks in the decision variables)
  for (unsigned int i = 0; i < stage_dims; ++i) {
    Eigen::Map<Eigen::VectorXd>(coeff_ddq_, solution_dims_) =
        hqp_constraints_.B_.row(acc_stage_dims + i);
    lhsides_[acc_stage_dims + i].addTerms(coeff_ddq_, ddq_, solution_dims_);
    if (acc_stage_dims == 0)
      // Force the slack variables to be zero in the highest stage
      lhsides_[acc_stage_dims + i] -= w_[i] * 0.0;
    else
      lhsides_[acc_stage_dims + i] -= w_[i];
  }

  // Add constraints to the QP model
  constraints_ =
      model_.addConstrs(lhsides_, &hqp_constraints_.constraint_signs_[0],
                        rhsides_, NULL, total_stage_dims);

  // Add objective function
  GRBQuadExpr obj;
  std::fill_n(coeff_ddq_, solution_dims_, TIKHONOV_FACTOR);
  std::fill_n(coeff_w_, stage_dims, 1.0);
  obj.addTerms(coeff_ddq_, ddq_, ddq_, solution_dims_);
  obj.addTerms(coeff_w_, w_, w_, stage_dims);
  model_.setObjective(obj, GRB_MINIMIZE);
  model_.update();

  // DEBUG =============================================
  // std::cerr << std::setprecision(2) << "Gurobi solver stage " << s_count << "
  // matrices:" << std::endl;
  // std::cerr << "A" << std::endl << A_ << std::endl;
  // std::cerr << "signs: ";
  // for (unsigned k=0; k<constraint_signs_.size(); k++)
  //   std::cerr << constraint_signs_[k] << " ";
  // std::cerr << std::endl << "b" << b_.transpose() << std::endl;
  // std::cerr << "w" << w_.transpose() << std::endl;
  // DEBUG END ==========================================
}

  bool GurobiSolver::QPProblem::solve() {
    model_.optimize();
    int status = model_.get(GRB_IntAttr_Status);
    double runtime = model_.get(GRB_DoubleAttr_Runtime);

    if (status != GRB_OPTIMAL) {
      if (status == GRB_TIME_LIMIT){
	ROS_WARN("In GurobiSolver::QPProblem::solve(...): Stage solving runtime %f sec exceeds the set time limit of %f sec.",
		 runtime, TIME_LIMIT);
      }
      else if (status == GRB_SUBOPTIMAL){
	ROS_WARN("In GurobiSolver::QPProblem::solve(...): Only suboptimal QP solution found.");
      }
      else if (status == GRB_NUMERIC){
	ROS_WARN("In GurobiSolver::QPProblem::solve(...): QP optimization was terminated due to unrecoverable numerical difficulties.");
      }
      else if (status == GRB_ITERATION_LIMIT){
	ROS_WARN("In GurobiSolver::QPProblem::solve(...): QP optimization was terminated because iteration limit was reached.");
      }       
      else{
	ROS_ERROR("In GurobiSolver::QPProblem::solve(...): No optimal solution found for stage with "
		  "priority %d. Status is %d.",
		  0, status);
     
	//DEBUG =======================================
	model_.write("/home/rkg/Desktop/model.lp");
	exit(0);
	//DEBUG END ====================================
	return false;
      }
    }
    return true;
  }

void GurobiSolver::QPProblem::getSolution(std::vector<double>& solution) {
  unsigned int stage_dims = hqp_constraints_.n_stage_dims_;
  unsigned int acc_stage_dims = hqp_constraints_.n_acc_stage_dims_;

  for (unsigned int i = 0; i < solution_dims_; ++i)
    solution.at(i) = ddq_[i].get(GRB_DoubleAttr_X);

  for (unsigned int i = 0; i < stage_dims; ++i)
    hqp_constraints_.w_(acc_stage_dims + i) = w_[i].get(GRB_DoubleAttr_X);
}

void GurobiSolver::HQPConstraints::reset(unsigned int n_solution_dims) {
  n_acc_stage_dims_ = 0;
  n_stage_dims_ = 0;
  w_.resize(0);
  b_.resize(0);
  B_.resize(0, n_solution_dims);
  constraint_signs_.clear();
}

void GurobiSolver::HQPConstraints::appendConstraints(
    const HiQPStage& current_stage) {
  // append stage dimensions from the previously solved stage
  n_acc_stage_dims_ += n_stage_dims_;
  n_stage_dims_ = current_stage.nRows;

  //append the constraint signs 
  for (unsigned int i = 0; i < n_stage_dims_; ++i) {
    if (current_stage.constraint_signs_.at(i) < 0)
      constraint_signs_.push_back(GRB_LESS_EQUAL);
    else if (current_stage.constraint_signs_.at(i) > 0)
      constraint_signs_.push_back(GRB_GREATER_EQUAL);
    else {
      constraint_signs_.push_back(GRB_EQUAL);
    }
  }

  b_.conservativeResize(n_acc_stage_dims_ + n_stage_dims_);
  b_.tail(n_stage_dims_) = current_stage.b_;
  B_.conservativeResize(n_acc_stage_dims_ + n_stage_dims_, Eigen::NoChange);
  B_.bottomRows(n_stage_dims_) = current_stage.B_;
  w_.conservativeResize(n_acc_stage_dims_ + n_stage_dims_);
  w_.tail(n_stage_dims_).setZero();
}

}  // namespace hiqp
