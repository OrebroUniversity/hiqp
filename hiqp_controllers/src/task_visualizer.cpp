/*!
 * \file   task_visualizer.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/task_visualizer.h>






namespace hiqp
{


TaskVisualizer::TaskVisualPrimitive::TaskVisualPrimitive
(
	double r, double g, double b, double a
)
: r_(r), g_(g), b_(b), a_(a) 
{}

////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								P L A N E                                     //
// 																			  //
////////////////////////////////////////////////////////////////////////////////

TaskVisualizer::TaskVisualPlane::TaskVisualPlane
(
	double nx, double ny, double nz, double d,
	double r, double g, double b, double a
)
: TaskVisualPrimitive(r, g, b, a), nx_(nx), ny_(ny), nz_(nz), d_(d)
{}





std::size_t TaskVisualizer::createPlane
(
	double nx, double ny, double nz, double d, 
	double r, double g, double b, double a
)
{
	TaskVisualPrimitive* visual = new TaskVisualPlane(nx, ny, nz, d,
													  r, g, b, a);
	std::size_t hash_key = 
		std::hash<TaskVisualPlane*>{}( (TaskVisualPlane*)visual );
	
	primitives_map_.insert( MapElement(hash_key, visual) );

	return hash_key;
}






int TaskVisualizer::setPlaneGeometry
(
	std::size_t id, 
	double nx, double ny, double nz, double d
)
{
	MapIterator it = primitives_map_.find(id);
	if (it == primitives_map_.end())
		return -1;

	TaskVisualPlane* plane = (TaskVisualPlane*)(it->second);
	plane->setGeometry(nx, ny, nz, d);

	return 0;
}






int TaskVisualizer::setPlaneEsthetics
(
	std::size_t id, 
	double r, double g, double b, double a
)
{
	MapIterator it = primitives_map_.find(id);
	if (it == primitives_map_.end())
		return -1;

	TaskVisualPrimitive* plane = (TaskVisualPrimitive*)(it->second);
	plane->setEsthetics(r,g,b,a);

	return 0;
}




////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								S P H E R E                                   //
// 																			  //
////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////
// 																			  //
//							       B O X                                      //
// 																			  //
////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								  C O N E                                     //
// 																			  //
////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								  L I N E                                     //
// 																			  //
////////////////////////////////////////////////////////////////////////////////











} // namespace hiqp



