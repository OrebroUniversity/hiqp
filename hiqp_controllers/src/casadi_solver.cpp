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








const double kDampingFactor = 1e-5; // dont set to zero!




int CasADiSolver::solve
(
	std::vector<double>& solution
)
{
	unsigned int solutionSize = solution.size();

	// Setup the parts of the QP problem that wont change between interations
	casadi::SX					qdot = casadi::SX::sym("q_dot", solutionSize, 1);
	casadi::SX 					f_qdot = kDampingFactor * casadi::SX::dot(qdot, qdot);
	casadi::SX 					g_i;
	std::vector<double> 		lbx(solutionSize, -std::numeric_limits<double>::infinity() );
	std::vector<double> 		ubx(solutionSize,  std::numeric_limits<double>::infinity() );
	std::vector<double> 		lbg, ubg;

	// Iterate over all stages and solve one QP per each stage
	StageMapIterator it = stages_map_.begin();
	while (it != stages_map_.end())
	{
		HiQPStage& stage = it->second;
		++it;
		int rows = stage.nRows;

		casadi::SX 					w_p = casadi::SX::sym("w_p", rows, 1);
		casadi::SX 					f_w = 0.5 * casadi::SX::dot(w_p, w_p);
		casadi::SX 					g_p = -w_p;

		for (int k=0; k<rows; ++k)
		{
			g_p(k, 0) += - stage.e_dot_star_(k);
			for (int kk=0; kk<solutionSize; ++kk)
				g_p(k, 0) += qdot(kk, 0) * stage.J_(k, kk);
		}

		lbx.insert( lbx.end(), rows, 0 );
		ubx.insert( ubx.end(), rows, std::numeric_limits<double>::infinity() );

		for (int sign : stage.constraint_signs_)
		{
			switch (sign)
			{
				case -1: // less-than-or-equal-to
				lbg.insert( lbg.end(), 1, -std::numeric_limits<double>::infinity() );
				ubg.insert( ubg.end(), 1, 0.0 );
				break;

				case 0: // equal-to
				lbg.insert( lbg.end(), 1, 0.0 );
				ubg.insert( ubg.end(), 1, 0.0 );
				break;

				case 1: // greater-than-or-equal-to
				lbg.insert( lbg.end(), 1, 0.0 );
				ubg.insert( ubg.end(), 1, std::numeric_limits<double>::infinity() );
				break;

				default: // equal-to
				lbg.insert( lbg.end(), 1, 0.0 );
				ubg.insert( ubg.end(), 1, 0.0 );
				break;
			}
		}
		

		casadi::DMDict arg = {{"lbx", lbx},
                              {"ubx", ubx},
                			  {"lbg", lbg},
                			  {"ubg", ubg}};  
      	
      	// For debugging purposes
        std::cout << "edotstar = " << stage.e_dot_star_ << "\n\n";
        std::cout << "J = " << stage.J_ << "\n\n";
        
        // std::cout << "x = " << vertcat(qdot, w_p) << "\n\n";
        // std::cout << "f_qdot = " << f_qdot << "\n\n";
        // std::cout << "f_w = " << f_w << "\n\n";
        // std::cout << "f = " << f_qdot + f_w << "\n\n";
        // std::cout << "g_i = " << g_i << "\n\n";
        // std::cout << "g_p = " << g_p << "\n\n";
        // std::cout << "g = " << vertcat(g_i, g_p) << "\n\n";
        // std::cout << "lbx = " << lbx << "\n\n";
        // std::cout << "ubx = " << ubx << "\n\n";
        // std::cout << "lbg = " << lbg << "\n\n";
        // std::cout << "ubg = " << ubg << "\n\n";
		
		casadi::SXDict qp = {{ "x", vertcat(qdot, w_p)   }, 
		                     { "f", f_qdot + f_w },
		                     { "g", vertcat(g_i, g_p)    }};
        casadi::Function solver = qpsol("solver", "gurobi", qp);
        casadi::DMDict res = solver(arg);
        casadi::DM x_opt = res["x"];
		std::vector<double> x_opt_d(x_opt);

		for (int k=0; k<rows; ++k)
		{
			casadi::SX g_i__ = - stage.e_dot_star_(k);
			for (int kk=0; kk<solutionSize; ++kk)
				g_i__ += qdot[kk] * stage.J_(k, kk);
			g_i__ += - x_opt_d[k + solutionSize];
			g_i = vertcat(g_i, g_i__);
		}

        //std::cout << "w_p = " << x_opt_d[solutionSize] << "\n\n";

		lbx.erase( lbx.end()-rows, lbx.end() );
		ubx.erase( ubx.end()-rows, ubx.end() );

		if (it == stages_map_.end())
		{
			std::cout << "solution = [";
			for (int k=0; k<solutionSize; ++k)
			{
				std::cout << x_opt_d[k] << ", ";
				solution.at(k) = x_opt_d[k];
			}
			std::cout << "]\n\n";
		}

	}

}










} // namespace hiqp




