/*!
 * \file   task_behaviour.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_BEHAVIOUR_H
#define HIQP_TASK_BEHAVIOUR_H



// STL Includes
#include <vector>

// Eigen Includes
#include <Eigen/Dense>





namespace hiqp
{
	






class TaskBehaviour
{
public:

	TaskBehaviour() {}
	~TaskBehaviour() noexcept {}

	virtual int init(const std::vector<std::string>& parameters) = 0;

	virtual double apply
	(
		double e
	) = 0;



private:

	// No copying of this class is allowed !
	TaskBehaviour(const TaskBehaviour& other) = delete;
	TaskBehaviour(TaskBehaviour&& other) = delete;
	TaskBehaviour& operator=(const TaskBehaviour& other) = delete;
	TaskBehaviour& operator=(TaskBehaviour&& other) noexcept = delete;

};








} // namespace hiqp






#endif // include guard