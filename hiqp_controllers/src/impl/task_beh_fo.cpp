/*!
 * \file   task_beh_fo.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/impl/task_beh_fo.h>
#include <hiqp/hiqp_utils.h>







namespace hiqp
{







int TaskBehFO::init
(
    const std::vector<std::string>& parameters
)
{
    if (parameters.size() != 1)
        return -1;

    lambda_ = std::stod( parameters.at(0) );
    
    return 0;
}







int TaskBehFO::apply
(
	const Eigen::VectorXd& e,
	Eigen::VectorXd& e_dot_star
)
{
	e_dot_star = -lambda_ * e;

	//std::cout << "e_dot_star = " << e_dot_star << "\n\n";

	return 0;
}








} // namespace hiqp