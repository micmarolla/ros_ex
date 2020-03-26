/**
 * This ROS service client asks for a Fibonacci sequence.
 * It takes two unsigned int as input from the user, namely
 * index of the first element and length of the sequence,
 * and send a request to the server.
 */

#include "ros/ros.h"
#include "fibonacci/service.h"
#include <iostream>
#include <sstream>

using namespace std;

int main(int argc, char **argv){
	// Initialize ROS and client
	ros::init(argc, argv, "fibonacci_client");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<fibonacci::service>("service");


	uint32_t index, length;
	cout << "Insert input separated by a space." << endl;

	while(ros::ok()){
		// Get input
		cout << "Index and length: ";
		cin >> index >> length;

		// Create service request
		fibonacci::service srv;
		srv.request.index = index;
		srv.request.length = length;

		// Send request to server
		if (client.call(srv)){
			cout << "From client: [" << srv.request.index << ", " << srv.request.length;
			cout << "], Server says [" << srv.response.out << "]" << endl;
		}
		else{
			ROS_ERROR("Failed to call service");
			return 1;
		}
		
		ros::spinOnce();
	}
	
	return 0;
}
