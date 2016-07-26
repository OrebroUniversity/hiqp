






#include <cstdlib>
#include <iostream>
#include <ctime>

#include <casadi/casadi.hpp>

#include <Eigen/Dense>






const int kNumStages = 5;
const int kNumControls = 14;
const double kDampingFactor = 1e-2; // dont set to zero!







struct HiQPStage
{
	int nRows;

	Eigen::MatrixXd e_dot_star_;

	Eigen::MatrixXd J_;
};





double getRandVal(int precision = 1e6)
{
	return double(std::rand()%precision)/double(precision);
}






void getStages(std::vector<HiQPStage>& stages)
{
	for (int i=0; i<kNumStages; ++i)
	{
		int rows = i%3 + 1; // 1 2 3 1 2
		HiQPStage stage;
		stage.nRows = rows;
		stage.e_dot_star_.resize(rows, 1);
		stage.J_.resize(rows, kNumControls);

		for (int j=0; j<rows; ++j)
		{
			stage.e_dot_star_(j,0) = getRandVal()*5;
			for (int k=0; k<kNumControls; ++k)
			{
				stage.J_(j,k) = getRandVal();
			}
		}

		stages.push_back(stage);
	}
}









casadi::SX vertcat2
(
	const std::vector<casadi::SX>& x1, 
	const std::vector<casadi::SX>& x2
)
{
	std::vector<casadi::SX> x;
	x.push_back( vertcat(x1) );
	x.push_back( vertcat(x2) );
	return vertcat(x);
}


casadi::SX vertcat2
(
	const casadi::SX& x1, 
	const casadi::SX& x2
)
{
	std::vector<casadi::SX> x;
	x.push_back( x1 );
	x.push_back( x2 );
	return vertcat(x);
}

casadi::SX vertcat2
(
	const std::vector<casadi::SX>& x1, 
	const casadi::SX& x2
)
{
	std::vector<casadi::SX> x;
	x.push_back( vertcat(x1) );
	x.push_back( x2 );
	return vertcat(x);
}









int main(int argc, char** argv)
{
	std::srand(std::time(0));


	std::vector<HiQPStage> stages;
	getStages(stages);

	int i=0;
	for (auto&& stage : stages)
	{
		++i;
		std::cout << "Stage #" << i << ":\n";
		std::cout << "edotstar = " << stage.e_dot_star_ << "\n";
		std::cout << "J = " << stage.J_ << "\n";
	}

	std::cout << "\n\n";




	std::vector<double> u;



	// Setup the parts of the QP problem that wont change between interations
	casadi::SX					qdot = casadi::SX::sym("q_dot", kNumControls, 1);
	casadi::SX 					f_qdot = kDampingFactor * casadi::SX::dot(qdot, qdot);
	casadi::SX 					g_i;
	std::vector<double> 		lbx(kNumControls, -std::numeric_limits<double>::infinity() );
	std::vector<double> 		ubx(kNumControls,  std::numeric_limits<double>::infinity() );
	std::vector<double> 		lbg, ubg;



	// Iterate over all stages and solve one QP per each stage
	for (int p=0; p<kNumStages; ++p)
	{
		HiQPStage& stage = stages.at(p);
		int rows = stage.nRows;

		casadi::SX 					w_p = casadi::SX::sym("w_p", rows, 1);
		casadi::SX 					f_w = 0.5 * casadi::SX::dot(w_p, w_p);
		casadi::SX 					g_p = -w_p;

		for (int k=0; k<rows; ++k)
		{
			g_p(k, 0) += - stage.e_dot_star_(k, 0);
			for (int kk=0; kk<kNumControls; ++kk)
				g_p(k, 0) += qdot(kk, 0) * stage.J_(k, kk);
		}

		lbx.insert( lbx.end(), rows, 0 );
		ubx.insert( ubx.end(), rows, std::numeric_limits<double>::infinity() );
		lbg.insert( lbg.end(), rows, 0 );
		ubg.insert( ubg.end(), rows, std::numeric_limits<double>::infinity() );

		casadi::DMDict arg = {{"lbx", lbx},
                              {"ubx", ubx},
                			  {"lbg", lbg},
                			  {"ubg", ubg}};  
      
      	
      	// For debugging purposes
        casadi::SX x = vertcat2(qdot, w_p);
        casadi::SX f = f_qdot + f_w;
        //casadi::SX g = vertcat2(g_i, g_p);
        std::cout << "x = " << x << "\n\n";
        std::cout << "f_qdot = " << f_qdot << "\n\n";
        std::cout << "f_w = " << f_w << "\n\n";
        std::cout << "f = " << f << "\n\n";
        std::cout << "g_i = " << g_i << "\n\n";
        std::cout << "g_p = " << g_p << "\n\n";
        //std::cout << "g = " << g << "\n\n";
	

		casadi::SXDict qp = {{ "x", vertcat2(qdot, w_p)   }, 
		                     { "f", f_qdot + f_w },
		                     { "g", vertcat2(g_i, g_p)    }};
        casadi::Function solver = qpsol("solver", "gurobi", qp);
        casadi::DMDict res = solver(arg);
        casadi::DM x_opt = res["x"];
		std::vector<double> x_opt_d(x_opt);

		for (int k=0; k<rows; ++k)
		{
			casadi::SX g_i__ = - stage.e_dot_star_(k, 0);
			for (int kk=0; kk<kNumControls; ++kk)
				g_i__ += qdot[kk] * stage.J_(k, kk);
			g_i__ += - x_opt_d[k + kNumControls];
			g_i = vertcat2(g_i, g_i__);
		}

		lbx.erase( lbx.end()-rows, lbx.end() );
		ubx.erase( ubx.end()-rows, ubx.end() );

		if (p == kNumStages - 1)
		{
			for (int k=0; k<kNumControls; ++k)
				u.push_back( x_opt_d[k] );
		}

	}


	std::cout << "u = ";
	for (int k=0; k<kNumControls; ++k)
		std::cout << u[k] << ", ";
	std::cout << "\n";

	return 0;

}