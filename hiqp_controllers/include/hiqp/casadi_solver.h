/*!
 * \file   casadi_solver.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_CASADI_SOLVER_H
#define HIQP_CASADI_SOLVER_H




// HiQP Includes
#include <hiqp/hiqp_solver.h>








namespace hiqp
{





class CasADiSolver : public HiQPSolver
{

public:

	CasADiSolver() {}
	~CasADiSolver() noexcept {}

	int solve(std::vector<double>& solution);


private:

	// No copying of this class is allowed !
    CasADiSolver(const CasADiSolver& other) = delete;
    CasADiSolver(CasADiSolver&& other) = delete;
    CasADiSolver& operator=(const CasADiSolver& other) = delete;
    CasADiSolver& operator=(CasADiSolver&& other) noexcept = delete;


};










} // namespace hiqp












#endif // include guard




