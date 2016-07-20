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

// CasADi Includes
#include <casadi/casadi.hpp>







namespace hiqp
{








int CasADiSolver::solveHiQPProblem
(
	std::vector<double>& solution
)
{

	double 					e_dot_star 	= stages_map_.find(0)->second.e_dot_star_;
	const Eigen::MatrixXd&  J 			= stages_map_.find(0)->second.J_;
	double& 				w 			= stages_map_.find(0)->second.w_;


	casadi::SX w_p;

	casadi::SX q_dot = casadi::SX::zeros(14);

	casadi::SX f = 0.5 * casadi::sq( casadi::fabs( w_p ) );

	casadi::SX g = J * q_dot - e_dot_star;


	casadi::SXDict qp = {{"x", vertcat(w_p, q_dot)}, {"f", f}, {"g", g}};

	casadi::qpsol solver("solver", "gurobi", qp);

	


	return 0;
}









} // namespace hiqp




