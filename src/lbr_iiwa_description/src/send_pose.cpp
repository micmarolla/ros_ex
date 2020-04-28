#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "send_pose");
    
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<geometry_msgs::Pose> ("/des_pos", 0);

    ros::Rate rate(10);
    geometry_msgs::Pose pose;
    tf::Quaternion quat;
    double r,p,y;
    while(ros::ok()){
        cout << "Insert position: " << endl;
        cin >> pose.position.x >> pose.position.y >> pose.position.z;

        cout << "Insert orientation (RPY): " << endl;
        cin >> r >> p >> y;

        quat.setRPY(r,p,y);
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        pub.publish(pose);

        cout << "Continue [S/n]? ";
        string input;
        cin.ignore();
        getline(cin, input);
        if(!input.empty() && (input == "n" || input == "N"))
            break;

        rate.sleep();
    }
}
