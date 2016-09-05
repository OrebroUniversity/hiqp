// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.




/*!
 * \file   ros_topic_subscriber.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */


// HiQP Includes
#include <hiqp/geometric_primitive_map.h>

// STL Includes
#include <string>
#include <vector>

// ROS Includes
#include "ros/ros.h"


#ifndef HIQP_ROS_TOPIC_SUBSCRIBER_H
#define HIQP_ROS_TOPIC_SUBSCRIBER_H

namespace hiqp
{


class ROSTopicSubscriber
{
public:

	ROSTopicSubscriber() {}

	~ROSTopicSubscriber() {}

	int init(GeometricPrimitiveMap* primitive_map)
	{
		primitive_map_ = primitive_map;
	}

	template<typename ROSMessageType>
	int addSubscription
	(
		ros::NodeHandle &controller_nh,
		const std::string& topic_name,
		unsigned int buffer_size
	)
	{
		sub = controller_nh.subscribe(
			topic_name, 
			buffer_size, 
			&ROSTopicSubscriber::topicCallback<ROSMessageType>,
			this
		);
		ROS_INFO_STREAM("Subsribed to topic '" << topic_name << "'");
	}

	/*! \brief Implement this function for your own message!
	*/
	template<typename ROSMessageType>
	void topicCallback(const ROSMessageType& msg);


private:

	// No copying of this class is allowed !
	ROSTopicSubscriber(const ROSTopicSubscriber& other) = delete;
	ROSTopicSubscriber(ROSTopicSubscriber&& other) = delete;
	ROSTopicSubscriber& operator=(const ROSTopicSubscriber& other) = delete;
	ROSTopicSubscriber& operator=(ROSTopicSubscriber&& other) noexcept = delete;

	ros::Subscriber 			sub;

	GeometricPrimitiveMap* 		primitive_map_;

};



}

#endif