/*!
 * \file   task_visualizer.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */


#include <iostream>
 
#include <hiqp/task_visualizer.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>






namespace hiqp
{


const std::string kNamespace  = "yumi";

const float kAlphaLevel = 0.5f;







////////////////////////////////////////////////////////////////////////////////
// 																			  //
//                        T A S K   V I S U A L I Z E R                       //
// 																			  //
////////////////////////////////////////////////////////////////////////////////

class TaskVisualPrimitive
{
public:
	TaskVisualPrimitive( double r, double g, double b, double a )
	: id_(-1), r_(r), g_(g), b_(b), a_(a), visibility_(true)
	{}

	inline void setEsthetics(double r, double g, double b, double a)
	{ r_ = r; g_ = g; b_ = b; a_ = a; }

	virtual void draw(const ros::Publisher& marker_pub, int action) = 0;

	std::size_t		id_;
	bool 			visibility_;

	double 			r_; 
	double 			g_; 
	double 			b_; 
	double 			a_;
};





TaskVisualizer::TaskVisualizer() 
: next_id_(0)
{}





int TaskVisualizer::init(ros::NodeHandle* controller_nh)
{
	controller_nh_ = controller_nh;

	marker_pub_ = controller_nh_->advertise<visualization_msgs::Marker>
		("visualization_marker", 1);
}





int TaskVisualizer::redraw()
{
	for (MapElement&& element : primitives_map_)
	{
		TaskVisualPrimitive* primitive = element.second;
		primitive->draw(marker_pub_, visualization_msgs::Marker::ADD);
	}
}






int TaskVisualizer::setVisibility(std::size_t id, bool visibility)
{
	MapIterator it = primitives_map_.find(id);
	if (it == primitives_map_.end())
		return -1;
	it->second->visibility_ = visibility;
	return 0;
}





std::size_t TaskVisualizer::insertPrimitive(TaskVisualPrimitive* primitive)
{ 
	primitives_map_.insert( MapElement(next_id_, primitive) );
	primitive->id_ = next_id_;
	next_id_++;
	return next_id_-1;
}




int TaskVisualizer::setPrimitiveEsthetics
(
	std::size_t id, 
	double r, double g, double b, double a
)
{
	MapIterator it = primitives_map_.find(id);
	if (it == primitives_map_.end())
		return -1;

	TaskVisualPrimitive* primitive = (TaskVisualPrimitive*)(it->second);
	primitive->setEsthetics(r,g,b,a);

	primitive->draw(marker_pub_, visualization_msgs::Marker::MODIFY);

	return 0;
}










////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								P L A N E                                     //
// 																			  //
////////////////////////////////////////////////////////////////////////////////

class TaskVisualPlane : public TaskVisualPrimitive
{
public:
	TaskVisualPlane(const std::string& base_link_name,
					double nx, double ny, double nz, double d,
			        double r, double g, double b, double a);

	void setGeometry(double nx, double ny, double nz, double d);

	void draw(const ros::Publisher& marker_pub, int action);

	std::string		base_link_name_;
	double 			nx_; 
	double 			ny_; 
	double 			nz_; 
	double 			d_;
};





TaskVisualPlane::TaskVisualPlane
(
	const std::string& base_link_name,
	double nx, double ny, double nz, double d,
	double r, double g, double b, double a
)
: TaskVisualPrimitive(r, g, b, a), base_link_name_(base_link_name)
{
	setGeometry(nx, ny, nz, d);
}






void TaskVisualPlane::setGeometry
(
	double nx, 
	double ny, 
	double nz, 
	double d
)
{
	// Normalize the normal vector!
	double n = sqrt(nx*nx + ny*ny + nz*nz);
	nx_ = nx/n;
	ny_ = ny/n;
	nz_ = nz/n;
	d_ = d;
}






void TaskVisualPlane::draw
(
	const ros::Publisher& marker_pub,
	int action
)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/" + base_link_name_;
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    marker.id = id_;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = action; 

    marker.pose.position.x = d_;
    marker.pose.position.y = d_;
    marker.pose.position.z = d_;

    marker.pose.orientation.x = nx_;
    marker.pose.orientation.y = ny_;
    marker.pose.orientation.z = nz_;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 15.0;
    marker.scale.y = 15.0;
    marker.scale.z = 0.001;

    marker.color.r = r_;
    marker.color.g = g_;
    marker.color.b = b_;
    marker.color.a = a_;

    marker.lifetime = ros::Duration(0); // forever

	//visualization_msgs::MarkerArray marker_array; // std::vector<...>

    marker_pub.publish(marker);

    std::cout << "TaskVisualizer::TaskVisualPlane::draw" << "\n";

    return;
}





std::size_t TaskVisualizer::createPlane
(
	const std::string& base_link_name,
	double nx, double ny, double nz, double d, 
	double r, double g, double b, double a
)
{
	TaskVisualPrimitive* plane = new TaskVisualPlane(base_link_name,
													 nx, ny, nz, d,
													 r, g, b, a);
	std::size_t id = insertPrimitive(plane);

	plane->draw(marker_pub_, visualization_msgs::Marker::ADD);

	std::cout << "TaskVisualizer::createPlane\n";

	return id;
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

	plane->draw(marker_pub_, visualization_msgs::Marker::MODIFY);

	return 0;
}





int TaskVisualizer::setPlaneEsthetics
(
	std::size_t id, 
	double r, double g, double b, double a
)
{
	setPrimitiveEsthetics(id, r, g, b, a);

	return 0;
}










////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								S P H E R E                                   //
// 																			  //
////////////////////////////////////////////////////////////////////////////////

class TaskVisualSphere : public TaskVisualPrimitive
{
public:
	TaskVisualSphere(const std::string& base_link_name,
					 double x, double y, double z, double radius,
			         double r, double g, double b, double a);

	void setGeometry(double x, double y, double z, double radius);

	void draw(const ros::Publisher& marker_pub, int action);

	std::string		base_link_name_;
	double 			x_; 
	double 			y_; 
	double 			z_;
	double 			radius_;
};





TaskVisualSphere::TaskVisualSphere
(
	const std::string& base_link_name,
	double x, double y, double z, double radius,
	double r, double g, double b, double a
)
: TaskVisualPrimitive(r, g, b, a), base_link_name_(base_link_name), x_(x), y_(y), z_(z), radius_(radius)
{}






void TaskVisualSphere::setGeometry
(
	double x, double y, double z, double radius
)
{
	x_ = x;
	y_ = y;
	z_ = z;
	radius_ = radius;
}






void TaskVisualSphere::draw
(
	const ros::Publisher& marker_pub,
	int action
)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/" + base_link_name_;
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    marker.id = id_;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = action; 

    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = z_;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2*radius_;
    marker.scale.y = 2*radius_;
    marker.scale.z = 2*radius_;

    marker.color.r = r_;
    marker.color.g = g_;
    marker.color.b = b_;
    marker.color.a = a_;

    marker.lifetime = ros::Duration(0); // forever

	//visualization_msgs::MarkerArray marker_array; // std::vector<...>

    marker_pub.publish(marker);

    std::cout << "TaskVisualizer::TaskVisualSphere::draw" << "\n";

    return;
}





std::size_t TaskVisualizer::createSphere
(
	const std::string& base_link_name,
	double x, double y, double z, double radius, 
	double r, double g, double b, double a
)
{
	TaskVisualPrimitive* plane = new TaskVisualSphere(base_link_name,
													  x, y, z, radius,
													  r, g, b, a);
	std::size_t id = insertPrimitive(plane);

	plane->draw(marker_pub_, visualization_msgs::Marker::ADD);

	std::cout << "TaskVisualizer::createSphere\n";

	return id;
}





int TaskVisualizer::setSphereGeometry
(
	std::size_t id, 
	double x, double y, double z, double radius
)
{
	MapIterator it = primitives_map_.find(id);
	if (it == primitives_map_.end())
		return -1;

	TaskVisualSphere* sphere = (TaskVisualSphere*)(it->second);
	sphere->setGeometry(x, y, z, radius);

	sphere->draw(marker_pub_, visualization_msgs::Marker::MODIFY);

	return 0;
}





int TaskVisualizer::setSphereEsthetics
(
	std::size_t id, 
	double r, double g, double b, double a
)
{
	setPrimitiveEsthetics(id, r, g, b, a);

	return 0;
}








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





////////////////////////////////////////////////////////////////////////////////
// 																			  //
//						      C Y L I N D E R                                 //
// 																			  //
////////////////////////////////////////////////////////////////////////////////










} // namespace hiqp



