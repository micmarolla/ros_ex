/**
 * This ROS service server generates a Fibonacci sequence.
 * It takes two unsigned int as input: the index of the
 * first element to return, and the number of elements
 * to return.
 */

#include "ros/ros.h"
#include "fibonacci/service.h"
#include <iostream>
#include <sstream>

using namespace std;

/* 
 * Generate Fibonacci sequence in form of a string.
 * Parameters:
 *  - index: the index of the first element of the sequence to return
 *  - length: the number of elements to return
 */
string generateFibonacci(uint32_t index, uint32_t length){
	stringstream ss;
	uint32_t a = 0, b = 1; // seeds
	uint32_t c = 0;

	/* Generate the sequence
	 * This implementation uses a single for cycle.
	 * Until i < index, the sequence is calculated but
	 * nothing is printed. When i >= index, the elements
	 * are calculated AND printed into the stringstream.
	 */
	for (int i = 0; i < index + length; ++i){
		if (i >= index)
			ss << c << " ";

		c = a + b;
		
		// This makes the double "1" correctely calculated
		if (i == 0) continue;

		a = b;
		b = c;		
	}

	return ss.str();
}


// Callback for when the server receives a request from the client.
bool service_callback(fibonacci::service::Request &req, fibonacci::service::Response &res){
	string str = generateFibonacci(req.index, req.length);
	res.out = str;
	ROS_INFO("From Client [%u, %u], Server says [%s]", req.index, req.length, res.out.c_str());
	return true;
}

// Initialize ROS and the server.
int main(int argc, char **argv){
	ros::init(argc, argv, "fibonacci_server");
	ros::NodeHandle nh;
	ros::ServiceServer server = nh.advertiseService("service", service_callback);
	ROS_INFO("Ready to receive from client.");
	ros::spin();
	return 0;
}
