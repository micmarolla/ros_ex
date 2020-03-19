/**
 * Subscriber node accepts read data from ROS topic named '/strings'
 * and prints them out.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

// Subscriber class
class Parrot_Sub{

public:
	Parrot_Sub();
	void callback(std_msgs::StringConstPtr data);

private:
	ros::NodeHandle _nh;
	ros::Subscriber _topic_sub;

};

// Subscribe to the '/string' topic
Parrot_Sub::Parrot_Sub(){
	_topic_sub = _nh.subscribe("/strings", 0, &Parrot_Sub::callback, this);
}

// Print received data
void Parrot_Sub::callback(const std_msgs::String::ConstPtr data){
	ROS_INFO("Listener: %s", data->data.c_str());
}


int main(int argc, char **argv){
	ros::init(argc, argv, "parrot_subscriber");
	Parrot_Sub sub;
	ros::spin();
	return 0;
}
