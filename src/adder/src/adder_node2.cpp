/**
 * This node subscribe to '/randTopic', get two floats, and publish
 * a new message with such data and their sum onto '/sumTopic'.
 */

#include "ros/ros.h"
#include "adder/randMsg.h"
#include "adder/respMsg.h"

// Node2 class
class Node2{

public:
	Node2();
	void callback(adder::randMsgConstPtr data);

private:
	ros::NodeHandle _nh;
	ros::Subscriber _sub;
	ros::Publisher _pub;

};

// Subscribe to the '/string' topic
Node2::Node2(){
	_sub = _nh.subscribe("/randTopic", 0, &Node2::callback, this);
	_pub = _nh.advertise<adder::respMsg>("/sumTopic", 10);
}

// Print received data
void Node2::callback(const adder::randMsgConstPtr data){
	ROS_INFO("Listener: %f, %f", data->data1, data->data2);

	// Generate and publish new message
	adder::respMsg msg;
	msg.data1 = data->data1;
	msg.data2 = data->data2;
	msg.sum = msg.data1 + msg.data2;
	
	ROS_INFO("Publishing: %f, %f, %f", msg.data1, msg.data2, msg.sum);
	_pub.publish(msg);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "adder_node2");
	Node2 node;
	ros::spin();
	return 0;
}
