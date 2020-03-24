/**
 * This ROS action client asks for control data for a stepper motor.
 * It accepts in input from commands line the following data:
 *  - desired final positon
 *  - initial position
 *  - maximum velocity
 *  - maximum time
 */

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "controller/controlAction.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "motor");
    
    // Check for input data
    if(argc != 5){
        ROS_WARN("Usage: motor <desired_pos> <initial_pos> <max_velocity> <max_time>");
        return 1;
    }
    if(atof(argv[4]) <= 0){
        ROS_ERROR("Timeout can't be less or equal to zero.");
        return 1;
    }

    // Init client and wait for server
    actionlib::SimpleActionClient<controller::controlAction> ac("controller_server", true);
    ROS_INFO("Waiting for controller server to start.");
    ac.waitForServer();

    // Sending goal
    ROS_INFO("Controller server started, sending goal.");
    controller::controlGoal goal;
    goal.des_pos   = atof(argv[1]);
    goal.init_pos  = atof(argv[2]);
    goal.max_vel   = atof(argv[3]);
    goal.time      = atof(argv[4]);
    ROS_INFO("Sending goal [%.3f,%.3f,%.3f] and preempt time [%.1f]",
        goal.des_pos, goal.init_pos, goal.max_vel, goal.time);
    ac.sendGoal(goal);

    // Check for result
    bool timeout = ac.waitForResult(ros::Duration(goal.time));
    ac.cancelGoal();

    if(timeout){
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Control action finished: %s", state.toString().c_str());
    }else
        ROS_INFO("Control action did not finish before the timeout.");
    
    return 0;

}
