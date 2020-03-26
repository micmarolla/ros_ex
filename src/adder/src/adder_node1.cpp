/**
 * Publisher node generates two random float and publish them
 * onto a ROS topic named '/randTopic'.
 */

#include "ros/ros.h"
#include "adder/randMsg.h"
#include <cstdlib>
#include <ctime>

using namespace std;

int main(int argc, char **argv){
	// Initialize ROS node
	ros::init(argc, argv, "adder_node1");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<adder::randMsg>("/randTopic", 0);
	
	adder::randMsg msg;
	srand(time(NULL));	// Initialize random seed
	
	// Main cycle
	while(ros::ok()){
		// Generate random floats between 0 and 1
		msg.data1 = (float)rand() / RAND_MAX;;
		msg.data2 = (float)rand() / RAND_MAX;;
		
		// Publish data
		ROS_INFO("%f, %f", msg.data1, msg.data2);
		pub.publish(msg);

	}

	return 0;
}
