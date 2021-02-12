/** 
 * Re-implementation of the Reverse priority solver from 
 * ``Unilateral constraints in the Reverse Priority redundancy resolution method''
 * IROS2015.
 *
 * Note: some changes to bring this up to acceleration level necessary
 */

#ifndef RP_SOLVER_H
#define RP_SOLVER_H

#include <Eigen/Core>
#include <hiqp/hiqp_solver.h>

namespace hiqp {
/*! \brief Linear solver implementation in Eigen
 *  \author Todor Stoyanov */
class RPSolver : public HiQPSolver {
 public:
  RPSolver();
  ~RPSolver() noexcept {}

  bool solve(std::vector<double>& solution);

 private:
  RPSolver(const RPSolver& other) = delete;
  RPSolver(RPSolver&& other) = delete;
  RPSolver& operator=(const RPSolver& other) = delete;
  RPSolver& operator=(RPSolver&& other) noexcept = delete;
  
  unsigned int n_solution_dims_;  // number of solution dimensions
  double lambda_smooth_ = 0.9;

  template<typename _Matrix_Type_>
  _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
  {
        // For a non-square matrix
        //Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
  }
  template<typename _Matrix_Type_>
  _Matrix_Type_ dampedPseudoInverse(const _Matrix_Type_ &a, double epsilon = 0.1, double mumaxsq = 0.1)
  {
        // For a non-square matrix
        //std::cerr<<"A = ["<<a<<"];\n";
        //Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Array<double,-1,-1> sigma_ = svd.singularValues().array();
	Eigen::Array<double,-1,-1> sigma_inv_;
	Eigen::Array<double,-1,-1> mu_ = sigma_/epsilon;
	mu_ = (1 - (mu_*mu_))*mumaxsq;
	sigma_inv_ = (sigma_.abs() > epsilon).select(sigma_.inverse(), sigma_/(sigma_*sigma_ + mu_*mu_));
        //std::cerr<<"U = ["<<svd.matrixU()<<"];\n";
        //std::cerr<<"V = ["<<svd.matrixV()<<"];\n";
	//std::cerr<<"sigma = ["<<sigma_.transpose()<<"]\n";
	//std::cerr<<"sigma_inv = ["<<sigma_inv_.transpose()<<"]\n";
	//std::cerr<<"pinvA = ["<<svd.matrixV() * sigma_inv_.matrix().asDiagonal() * svd.matrixU().adjoint()<<"];\n";
	return svd.matrixV() * sigma_inv_.matrix().asDiagonal() * svd.matrixU().adjoint();
  }

  /** \brief computes matrix T_k by a rank update of J_Aug_pinv_ with J_ */
  inline void rank_update(const Eigen::MatrixXd &J_, 
		  const Eigen::MatrixXd &J_Aug_pinv_,Eigen::MatrixXd &T)
  {

    double dim_J = J_.rows();
    double dim_Ja_pinv_ = J_Aug_pinv_.cols();

    T.resize(J_Aug_pinv_.rows(),1);
    T << J_Aug_pinv_.col(0);

    int i=1, j=1;
    while(i<=dim_J-1)
    {
      while(j<=dim_Ja_pinv_-1)
      {
        T.conservativeResize(T.rows(), T.cols()+1);
        T.col(T.cols()-1) = J_Aug_pinv_.col(i);
        j++;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(T, Eigen::ComputeFullU | Eigen::ComputeFullV);
        if(svd.rank()==i+1)
        {
          i++;
          break;
        } else {
          T.conservativeResize(T.rows(), T.cols()-1);
        }
      }
    }
  }

};

}  // namespace hiqp

#endif  // include guard
