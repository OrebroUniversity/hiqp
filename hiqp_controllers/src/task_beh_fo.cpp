/*!
 * \file   task_beh_fo.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/task_beh_fo.h>
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
	double e, 
	const Eigen::MatrixXd& J, 
	std::vector<double>& controls
)
{
	Eigen::MatrixXd u = -lambda_ * dls(J, 0.1) * e;
     
    for (int i=0; i<controls.size(); ++i)
         controls.at(i) = u(i,0);

	return 0;
}








} // namespace hiqp