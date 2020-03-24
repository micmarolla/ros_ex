/**
 * This ROS action server generates control data for a stepper motor.
 * It receives in input initial position of the motor, the desired final
 * position, maximum rotational velocity and time to complete motion.
 * Every input data is assumed expressed in IS units.
 * The control loop is executed at 10Hz.
 *
 * Actually, since this is a ROS exercise, every motor dynamics is neglected.
 * It is assumed that the motor has a linear characteristics of the type :
 * new_position = last_position + velocity * elapsed_time,
 * where velocity has to be less or equal to max_vel specified by the client.
 * When the control succeed, the final position is achieved with a 1mm
 * tolerance.
 * 
 * Actually, given the simple control logic, it's possible to check if the goal
 * is feasible or not before actuating it; for the sake of the exercise,
 * the control is carried out anyway, and the server is preempted if it cannot
 * make the motor reach the desired position.
 * The code that make this preliminar check has been writtend anyway and
 * commented out.
 *
 * In executeCallback function, the 'work in progress' and the 'succeeded'
 * blocks of code are located in two if-s, and not in a if-else. This is because
 * otherwise the server used one control cycle to reach the final position, and
 * another one to notice it. This leads the service to be preempted even if it
 * reaches the final goal. Placing the code in two if blocks solves the problem.
 */

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h>
#include "controller/controlAction.h"
#include <cstdlib>

#define RATE        10      // Control loop at 10 Hz
#define TOLERANCE   0.001   // Tolerance: 1 mm


class Controller{

private:
	ros::NodeHandle _nh;
	actionlib::SimpleActionServer<controller::controlAction> as;

	controller::controlFeedback feedback;
	controller::controlResult result;
	
	std::string action_name;
    float pos;          // current position
    float distance;     // distance to cover
    float velocity;

public:
    // Init
	Controller(std::string name) :
			as(_nh, name, boost::bind(&Controller::executeCallback, this, _1), false),
			action_name(name){
        pos = 0;
		as.registerPreemptCallback(boost::bind(&Controller::preemptCallback, this));
		as.start();
	}

	void preemptCallback(){
		ROS_WARN("%s got preempted", action_name.c_str());
		result.final_pos = pos;
		as.setPreempted(result, "Preempted");
	}

	void executeCallback(const controller::controlGoalConstPtr &goal){
        // Check if the action server is active or preempted
		if(!as.isActive() || as.isPreemptRequested())
			return;

		ros::Rate rate(RATE);     // Control loop at 10 Hz
		ROS_INFO("%s is processing the goal [%.3f,%.3f,%.3f,%.3f]", action_name.c_str(),
            goal->des_pos, goal->init_pos, goal->max_vel, goal->time);

        // Calculate distance and velocity
        pos = goal->init_pos;
        distance = goal->des_pos - goal->init_pos;
        velocity = distance / goal->time;
        if (abs(velocity) >= abs(goal->max_vel)){
            velocity = goal->max_vel;                       // Saturate velocity
            velocity = velocity * ((distance>0) ? 1 : -1);  // Get the correct sign
        }
        ROS_INFO("Distance: %.3f, velocity: %.3f", distance, velocity);

        // Check goal feasibility
        /*float min_time = distance / goal->max_vel;

        // Reject goal if not feasible
        if (min_time < goal->time || velocity >= goal->max_vel){
            ROS_INFO("Goal is not achievable");
            //as.setRejected()
            result.final_pos = goal->init_pos;
            as.setAborted(result, "Goal not achievable");
            ROS_INFO("%s shutting down", action_name.c_str());
            return;
        }*/

        
        // Main cycle
        while(true){

            // Manage abortion
            if(!ros::ok()){
                result.final_pos = pos;
                as.setAborted(result, "Failed");
                ROS_INFO("%s shutting down", action_name.c_str());
                break;
            }

            // Manage deactivation and preemption
            if(!as.isActive() || as.isPreemptRequested())
                return;

            // Work in progress
            if (pos < goal->des_pos - TOLERANCE || pos > goal->des_pos + TOLERANCE){
                pos = pos + velocity / RATE;    // Update position
                feedback.current_pos = pos;
                ROS_INFO("Setting to goal %.3f / %.3f", feedback.current_pos, goal->des_pos);
                as.publishFeedback(feedback);
            }

            // Success
            if(pos >= goal->des_pos - TOLERANCE && pos <= goal->des_pos + TOLERANCE){
                ROS_INFO("%s succeded at getting to goal %.3f",
                    action_name.c_str(), goal->des_pos);
                result.final_pos = pos;
                as.setSucceeded(result);
            }

            rate.sleep();

        }
	}

};


int main(int argc, char** argv){
    ros::init(argc, argv, "controller_server");
    ROS_INFO("Starting Control Action Server");
    Controller cont(ros::this_node::getName());
    ros::spin();
    return 0;
}
