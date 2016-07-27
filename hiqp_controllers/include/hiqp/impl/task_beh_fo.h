/*!
 * \file   task_beh_fo.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_BEHAVIOUR_FIRSTORDER_H
#define HIQP_TASK_BEHAVIOUR_FIRSTORDER_H

// HiQP Includes
#include <hiqp/task_behaviour.h>



namespace hiqp
{






/*!
 * \class TaskBehFO
 * \brief This is my awesome controller
 *
 *  It's awesome!
 */	
class TaskBehFO : public TaskBehaviour
{
public:

	TaskBehFO() {}

	~TaskBehFO() noexcept {}

	int init(const std::vector<std::string>& parameters);

	int apply
	(
		const Eigen::VectorXd& e,
		Eigen::VectorXd& e_dot_star
	);

private:

	// No copying of this class is allowed !
	TaskBehFO(const TaskBehFO& other) = delete;
	TaskBehFO(TaskBehFO&& other) = delete;
	TaskBehFO& operator=(const TaskBehFO& other) = delete;
	TaskBehFO& operator=(TaskBehFO&& other) noexcept = delete;

	double lambda_;

};







} // namespace hiqp






#endif // include guard