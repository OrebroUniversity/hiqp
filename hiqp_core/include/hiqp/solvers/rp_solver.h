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

  template<typename _Matrix_Type_>
  _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
  {
        // For a non-square matrix
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
  }
  template<typename _Matrix_Type_>
  _Matrix_Type_ dampedPseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
  {
        // For a non-square matrix
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
  }
};

}  // namespace hiqp

#endif  // include guard
