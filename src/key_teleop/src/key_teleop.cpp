#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread.hpp"

#include <iostream>

using namespace std;

class KEY_CTRL {
	public:
		KEY_CTRL();
		void key_input();
		void run();

	private:
		ros::NodeHandle _nh;
		ros::Publisher  _vel_pub;
   		ros::Publisher  _keys_pub;

};


KEY_CTRL::KEY_CTRL() {
	_keys_pub = _nh.advertise<std_msgs::Char>("/cmd_vel/key", 0);
}


void KEY_CTRL::key_input() {
    ros::Rate r(10);
	char input;

	cout << "Keyboard input: " << endl;
	cout << "[w]: Forward direction velocity" << endl;
	cout << "[x]: Backward direction velocity" << endl;
	cout << "[a]: Left angular velocity" << endl;
	cout << "[d]: Right angular velocity" << endl;
	cout << "[s]: Stop the robot" << endl;
    cout << "[r]: Return to automatic drive" << endl;

	while (ros::ok()) {

		cin >> input;

        std_msgs::Char key_msg;
        key_msg.data = input;
        _keys_pub.publish(key_msg);

		

        r.sleep();
	}
}

void KEY_CTRL::run() {
	boost::thread key_input_t( &KEY_CTRL::key_input, this );
	ros::spin();
}


int main(int argc, char** argv ) {

	ros::init(argc, argv, "key_ctrl");

	KEY_CTRL kc;
	kc.run();

	return 0;
}
