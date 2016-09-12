


#include <iostream>
#include <string>
#include "ros/ros.h"

#include <hiqp_msgs_srvs/Vector3d.h>






std::vector<double> split_stod(const std::string &s, char delim) {
    std::vector<double> elems;

	std::stringstream ss;
    ss.str(s);
    std::string item;
    while (getline(ss, item, delim)) {
        elems.push_back( std::stod(item) );
    }

    return elems;
}






int main(int argc, char** argv)
{
	ros::init(argc, argv, "hiqp_rostopicsender");
	ros::NodeHandle n;

	ros::Publisher publisher = n.advertise<hiqp_msgs_srvs::Vector3d>
		("/yumi/hiqp_controllers/vector3d", 1000);
	
	std::string input;

	while (ros::ok())
	{
		std::cout << "Cylinder control ros node.\n";
		std::cout << "syntax: dx,dy,dr\n";

		while (std::getline(std::cin, input))
		{

			std::vector<double> elems = split_stod(input, ',');

			if (elems.size() != 3)
			{
				std::cout << "mind your language!\n";
			}
			else
			{
				std::cout << "got it!\n";

				hiqp_msgs_srvs::Vector3d msg;
				msg.val1 = elems.at(0);
				msg.val2 = elems.at(1);
				msg.val3 = elems.at(2);
				publisher.publish(msg);
			}

		}
		break;

	}

	std::cout << "exiting...\n";

	return 0;
}