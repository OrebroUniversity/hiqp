#include <iostream>
#include <string>
#include "ros/ros.h"

#include <hiqp_msgs_srvs/JustADouble.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hiqp_rostopicsender");
	ros::NodeHandle n;

	ros::Publisher publisher = n.advertise<hiqp_msgs_srvs::JustADouble>
		("/yumi/hiqp_controllers/justadouble", 1000);
	
	std::string input;

	while (ros::ok())
	{

		while (std::getline(std::cin, input))
		{
			if (input.compare("f") == 0)
			{
				std::cout << "move formward\n";

				hiqp_msgs_srvs::JustADouble msg;
				msg.value = -0.05;
				publisher.publish(msg);
			}
			else if (input.compare("b") == 0)
			{
				std::cout << "move backward\n";

				hiqp_msgs_srvs::JustADouble msg;
				msg.value = 0.05;
				publisher.publish(msg);
			}
			else if (input.compare("quit") == 0)
			{
				break;
			}
			else
			{
				std::cout << "mind your language!\n";
			}
		}
		break;

	}

	std::cout << "exiting...\n";

	return 0;
}