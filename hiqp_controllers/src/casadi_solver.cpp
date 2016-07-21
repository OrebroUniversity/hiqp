/*!
 * \file   task_pop.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



// HiQP Includes
#include <hiqp/casadi_solver.h>
#include <hiqp/hiqp_utils.h>

// CasADi Includes
#include <casadi/casadi.hpp>

// Eigen Includes
#include <Eigen/Dense>







namespace hiqp
{








int CasADiSolver::solve
(
	std::vector<double>& solution
)
{
	StageMapIterator it = stages_map_.find(1);

	HiQPStage& stage = it->second;

	Eigen::MatrixXd u = double(stage.e_dot_star_(0, 0)) * dls(stage.J_, 0.01);

    for (int i= 0; i<solution.size(); ++i)
    {
        solution[i] = u(i);
    }

/*
	double 					e_dot_star 	= stages_map_.find(0)->second.e_dot_star_;
	const Eigen::MatrixXd&  J 			= stages_map_.find(0)->second.J_;
	double& 				w 			= stages_map_.find(0)->second.w_;


	casadi::SX w_p;

	casadi::SX q_dot = casadi::SX::zeros(14);

	casadi::SX f = 0.5 * casadi::sq( casadi::fabs( w_p ) );

	casadi::SX g = J * q_dot - e_dot_star;


	casadi::SXDict qp = {{"x", vertcat(w_p, q_dot)}, {"f", f}, {"g", g}};

	casadi::qpsol solver("solver", "gurobi", qp);
*/
	


	return 0;
}









} // namespace hiqp




