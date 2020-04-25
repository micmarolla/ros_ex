#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <actionlib/client/simple_action_client.h>
#include "surveillance_task/navAction.h"

#define SAFE_DISTANCE       1.0
#define DISTANCE_TRESHOLD   0.02
#define LIN_VEL             0.2
#define ANG_VEL             0.4
#define CONIC_ANGLE         20.0


using namespace std;

class SURV_CLIENT {
    public:
        SURV_CLIENT();
        void human_cb(std_msgs::CharConstPtr );
        void laser_cb(sensor_msgs::LaserScanConstPtr );
        void odom_cb(nav_msgs::OdometryConstPtr );
        void run();
        void main_loop();
                
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _laser_sub;
        ros::Subscriber _human_sub;
        ros::Publisher _cmd_pub;

        std::list<geometry_msgs::Point> _point_list;
        std::list<bool> _check_obst_list;
        bool _obstacle;
        bool _curr_check_obst;
        bool _recording;
        geometry_msgs::Point _curr_pos;

        std::list<geometry_msgs::Point>::iterator _learn_traj(std::list<geometry_msgs::Point>::iterator,  std::list<bool>::iterator);
        bool _is_in_angle(geometry_msgs::Point , geometry_msgs::Point ) const;
            
};

SURV_CLIENT::SURV_CLIENT() {
    _human_sub = _nh.subscribe("/cmd_vel/key", 0, &SURV_CLIENT::human_cb,this); 
    _laser_sub = _nh.subscribe("/laser/scan",0,&SURV_CLIENT::laser_cb,this);
    _odom_sub = _nh.subscribe("/odom", 0, &SURV_CLIENT::odom_cb, this);
    _cmd_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 0);
    _obstacle = false;
    _curr_check_obst = true;
    _recording = false;

    geometry_msgs::Point p;
    p.x = 0.0;  p.y = 0.0;
    _point_list.push_back(p);
    _check_obst_list.push_back(1);
    p.x = 2.0;  p.y = 0.0;
    _point_list.push_back(p);
    _check_obst_list.push_back(1);
    p.x = 2.0;  p.y = 2.0;
    _point_list.push_back(p);
    _check_obst_list.push_back(1);
    p.x = 0.0;  p.y = 2.0;
    _point_list.push_back(p);
    _check_obst_list.push_back(1);
}


void SURV_CLIENT::human_cb(std_msgs::CharConstPtr human) {
    float lin = 0; //Forward velocity	
    float rot = 0; //Rotational velocity

    if( human->data == 'w' ) 
	    lin = (lin < 0.0 ) ? 0.0 : LIN_VEL;
    else if( human->data == 'x' ) 
        lin = (lin > 0.0 ) ? 0.0 : -LIN_VEL;
    else if( human->data == 'a' ) 
        rot = (rot > 0.0 ) ? 0.0 : -ANG_VEL;
    else if( human->data == 'd' )
        rot = (rot < 0.0 ) ? 0.0 : ANG_VEL;
    else if( human->data == 's' ) 
        lin = rot = 0.0;
    else if (human->data == 'r')
        _obstacle = false;

    if(lin == 0)
        _recording = false;
    else
        _recording = true;

    geometry_msgs::Twist cmd;
    cmd.angular.z = rot;
    cmd.linear.x = lin;
    _cmd_pub.publish(cmd);
}


void SURV_CLIENT::laser_cb( sensor_msgs::LaserScanConstPtr laser_data ) {
    if(!_curr_check_obst)
        return;

	int first_index = int( (( 90-20 )/180.0*M_PI) / laser_data->angle_increment);
    int last_index  = int( (( 90+20 )/180.0*M_PI) / laser_data->angle_increment);

    for (int i=first_index; i<=last_index; ++i){
        if(laser_data->ranges[i] < SAFE_DISTANCE){
            _obstacle = true;
            break;
        }
    }
}
 
void SURV_CLIENT::odom_cb( nav_msgs::OdometryConstPtr odom) {
    _curr_pos.x = odom->pose.pose.position.x;
    _curr_pos.y = odom->pose.pose.position.y;
}


std::list<geometry_msgs::Point>::iterator
            SURV_CLIENT::_learn_traj(std::list<geometry_msgs::Point>::iterator next_point,
            std::list<bool>::iterator check_it){

    cout << "Learning trajectory from human" << endl;
    ros::Rate r(0.5);
    *check_it = 0;

    auto point_it = next_point;
    auto prev_point = point_it;
    prev_point--;

    
    geometry_msgs::Point last_point = _curr_pos;

    // Exit when human go back to autonomous drive
    while(_obstacle){
        // Calc distance from lastPoint to currPoint
        float distance = sqrt(pow(last_point.x - _curr_pos.x ,2)
                + pow(last_point.y - _curr_pos.y,2));

        if (distance >= DISTANCE_TRESHOLD && _recording){
            cout << "Saving new pos" << endl;
            _point_list.insert(point_it, _curr_pos);
            _check_obst_list.insert(check_it, 0);
        }

        last_point = _curr_pos;

        r.sleep();
    }

    // Check if need to ignore one point
    if (!_is_in_angle(*next_point, *prev_point)){
    
        point_it = _point_list.erase(point_it);
    }
    

    return point_it;
}

bool SURV_CLIENT::_is_in_angle(geometry_msgs::Point point, geometry_msgs::Point prev) const{
    // Determine normalized cone axis
    geometry_msgs::Point cone_axis;
    cone_axis.x = prev.x - point.x;
    cone_axis.y = prev.y - point.y;
    float cone_norm = sqrt(pow(cone_axis.x,2)+pow(cone_axis.y,2));
    cone_axis.x /= cone_norm;
    cone_axis.y /= cone_norm;

    // Determine axis from point to curr_point
    geometry_msgs::Point curr_axis;
    curr_axis.x = _curr_pos.x - point.x;
    curr_axis.y = _curr_pos.y - point.y;
    float curr_norm = sqrt(pow(curr_axis.x,2)+pow(curr_axis.y,2));
    curr_axis.x /= curr_norm;
    curr_axis.y /= curr_norm;

    // Check for angle
    float dot_prod = cone_axis.x*curr_axis.x + cone_axis.y*curr_axis.y;
    float angle = acos(dot_prod);
    
    return (angle < CONIC_ANGLE*M_PI/180.0);
}

 
//main loop! 
void SURV_CLIENT::main_loop() {

    actionlib::SimpleActionClient<surveillance_task::navAction> ac("auto_nav_server", true);
    ac.waitForServer(); //will wait for infinite time

    surveillance_task::navGoal g;
    
    while(ros::ok()) {
        ros::Rate r(10);
        bool done = false;
          
        //motion logic
        auto ch_ob = _check_obst_list.begin();
        for(auto point = _point_list.begin(); point != _point_list.end(); ){
            done = false;
            g.x_dest = (*point).x;
            g.y_dest = (*point).y;
            _curr_check_obst = *ch_ob;
          
            ac.sendGoal(g);

            while ( !done && !_obstacle) {
                if ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
                        ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED ) {
                    done = true;
                }
                r.sleep();
            }
            
            if( _obstacle) {
                ac.cancelGoal();
                point = _learn_traj(point, ch_ob);

            }else{
                ++point;
                ++ch_ob;
            }

            cout << "Point list:" << endl;
            for(auto& pt : _point_list)
                cout << pt.x << " " << pt.y << endl;
        }

        if( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
            cout << "Target position reached!" << endl;

        r.sleep();
    }

}

 
 
void SURV_CLIENT::run() {
    boost::thread ctrl_loop_t( &SURV_CLIENT::main_loop, this );
    ros::spin();
}



int main( int argc, char** argv) {

    ros::init(argc, argv, "surv_client" );
    SURV_CLIENT s;
    s.run();
    
    return 0;

}   
