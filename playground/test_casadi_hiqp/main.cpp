






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

	Eigen::MatrixXd w_;
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

	for (int p=0; p<kNumStages; ++p)
	{


		HiQPStage& stage = stages.at(p);

		int rows = stage.nRows;

		stage.w_.resize(rows, 1);



		std::vector<casadi::SX> x;

		for (int k=0; k<rows; ++k)
		{
			x.push_back( casadi::SX::sym("w_p") );
		}

		for (int k=0; k<kNumControls; ++k) 
		{
			x.push_back( casadi::SX::sym("qdot") );
		}



		casadi::SX f = casadi::SX::sym("f");

		for (int k=0; k<rows; ++k)
		{
			f += casadi::sq( x[k] );
		}

		f *= 0.5 / kDampingFactor;

		for (int k=0; k<kNumControls; ++k)
		{
			f += casadi::sq( x[k+rows] );
		}

		f *= kDampingFactor;



		std::vector<casadi::SX> g;

		for (int i=0; i<=p; ++i)
		{
			int rr = stages.at(i).nRows;

			casadi::SX g_i = casadi::SX::sym("g_i", rr, 1);

			for (int k=0; k<rr; ++k)
			{
				g_i(k,0) = - stages.at(i).e_dot_star_(k, 0);

				for (int kk=0; kk<kNumControls; ++kk)
				{
					g_i(k,0) += x[kk+rows] * stages.at(i).J_(k, kk);
				}

				if (i < p)    g_i(k,0) += - stages.at(i).w_(k, 0);
				else          g_i(k,0) += - x[k];
			}

			g.push_back( g_i );
		}



		casadi::SXDict qp = {{"x", vertcat(x)}, 
		                     {"f", f},
		                     {"g", vertcat(g)}};

        casadi::Function solver = qpsol("solver", "qpoases", qp);



        std::vector<double> lbx, ubx, lbg, ubg;

        for (int k=0; k<rows; ++k)
        {
        	lbx.push_back( 0 );
			ubx.push_back( std::numeric_limits<double>::infinity() );
        }
		
		for (int k=0; k<kNumControls; ++k) 
		{
			lbx.push_back( -std::numeric_limits<double>::infinity() );
			ubx.push_back(  std::numeric_limits<double>::infinity() );
		}

		for (int k=0; k<=p; ++k)
		{
			int rr = stages.at(k).nRows;
			for (int kk=0; kk<rr; ++kk)
			{
				lbg.push_back( 0 );
				ubg.push_back( std::numeric_limits<double>::infinity() );
			}
		}

		casadi::DMDict arg = {{"lbx", lbx},
                              {"ubx", ubx},
                			  {"lbg", lbg},
                			  {"ubg", ubg}};  


		std::cout << "p=" << p << "\n";
        casadi::DMDict res = solver(arg);



        casadi::DM x_opt = res["x"];

		std::vector<double> x_opt_d(x_opt);

		for (int k=0; k<rows; ++k)
		{
			stages.at(p).w_(k,0) = x_opt_d[k];
		}
		


		if (p == kNumStages - 1)
		{
			for (int k=0; k<kNumControls; ++k)
				u.push_back( x_opt_d[k+rows] );
		}

	}



	std::cout << "u = ";
	for (int k=0; k<kNumControls; ++k)
		std::cout << u[k] << ", ";
	std::cout << "\n";



	return 0;




	// Below is the old code!!!
/*
	double edotstar[3] = {0.5, 0.6, 0.7};
	Eigen::MatrixXd J[3];
	J[0].resize(1, 14);
	J[1].resize(1, 14);
	J[2].resize(1, 14);
	J[0] << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0.1, 0.1, 0.1;
	J[1] << 0.1, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	J[2] << 0, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0;


	double w[3] = {0.0, 0.0, 0.0};

	std::vector<double> u;

	for (int p=0; p<3; ++p)
	{

		std::vector<casadi::SX> x;
		x.push_back( casadi::SX::sym("w_p") );

		for (int j=0; j<14; ++j) 
		{
			x.push_back( casadi::SX::sym("qdot",1,1) );
		}
		
		casadi::SX f = 0.5 * casadi::sq( x[0] );

		std::vector<casadi::SX> g;
		for (int i=0; i<=p; ++i)
		{
			casadi::SX g_i = - edotstar[i] + (i==p ? 0 : -w[i]);
			for (int j=0; j<14; ++j)
			{
				g_i += x[j+1] * J[i](j);
			}
			g.push_back( g_i );
		}
		


		casadi::SXDict qp = {{"x", vertcat(x)}, 
		                     {"f", f},
		                     {"g", vertcat(g)}};

		casadi::Function solver = qpsol("solver", "qpoases", qp);


		std::vector<double> lbx, ubx, lbg, ubg;
		lbx.push_back(0);
		ubx.push_back(1000000);
		for (int k=0; k<14; ++k) 
		{
			lbx.push_back(-10);
			ubx.push_back(10);
		}
		for (int k=0; k<=p; ++k)
		{
			lbg.push_back(0);
			ubg.push_back(1000000000);
		}

		casadi::DMDict arg = {{"lbx", lbx},
                              {"ubx", ubx},
                			  {"lbg", lbg},
                			  {"ubg", ubg}};  

  		casadi::DMDict res = solver(arg);

		casadi::DM x_opt = res["x"];
		std::vector<double> x_opt_d(x_opt);
		w[p] = x_opt_d[0];




		if (p==2)
		{
			for (int k=0; k<14; ++k)
				u.push_back(x_opt_d[k+1]);
		}



	}

	std::cout << "u = ";
	for (int k=0; k<14; ++k)
		std::cout << u[k] << ", ";
	std::cout << "\n";

	return 0;
*/

}