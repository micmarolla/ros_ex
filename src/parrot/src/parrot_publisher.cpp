/**
 * Publisher node accepts as input string or characters using the keyboard,
 * and publish such data on a ROS topic named '/strings'.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

int main(int argc, char **argv){
	// Initialize ROS node
	ros::init(argc, argv, "parrot_publisher");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::String>("/strings", 10);

	ros::Rate rate(10);
	
	// Main cycle
	while(ros::ok()){
		// Retrieve input
		std::cout << "Input: ";
		std_msgs::String msg;
		std::getline(std::cin, msg.data);
		
		// Publish data
		ROS_INFO("%s", msg.data.c_str());
		pub.publish(msg);

		rate.sleep();
	}

	return 0;
}
