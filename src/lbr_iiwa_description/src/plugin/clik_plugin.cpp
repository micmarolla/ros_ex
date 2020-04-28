#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

using namespace std;
	
namespace gazebo
{
  class CLIKPlugin : public ModelPlugin {

	private:
        ros::NodeHandle* _nh;
        physics::ModelPtr model;

        ros::Subscriber _pos_sub;
        ros::Subscriber _js_sub;
        ros::Publisher *_cmd_pub;

        event::ConnectionPtr updateConnection;

        int _joint_num;
        KDL::Tree iiwa_tree;
        KDL::Chain _k_chain;

        KDL::ChainFkSolverPos_recursive *_fksolver;
        KDL::ChainIkSolverVel_pinv  *_ik_solver_vel;
        KDL::ChainIkSolverPos_NR    *_ik_solver_pos;

        KDL::JntArray *_q;
        KDL::Frame _des_frame;

        // Init KDL solver
        bool _initIK();
        

	public:
        // Free memory
        ~CLIKPlugin();
        
        // Load the plugin
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        // Calculate and apply inverse kinematics
        void OnUpdate();

        // Update curr joint state
        void joint_states_cb( sensor_msgs::JointState );

        // Retrieve desired position
        void pos_cb( geometry_msgs::PoseConstPtr );
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CLIKPlugin)



CLIKPlugin::~CLIKPlugin(){
    delete[] _cmd_pub;
    delete _q;
    delete _ik_solver_pos;
    delete _ik_solver_vel;
    delete _nh;
}


void CLIKPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Setup plugin
    _nh = new ros::NodeHandle();	
    model = _parent;
    _pos_sub = _nh->subscribe("/des_pos", 0, &CLIKPlugin::pos_cb, this);
    _js_sub = _nh->subscribe("/lbr_iiwa/joint_states", 0, &CLIKPlugin::joint_states_cb, this);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CLIKPlugin::OnUpdate, this));

    // Setup inverse kinematics solver
    if(!_initIK())
        exit(1);

    ROS_INFO("Robot tree correctly loaded");

    _cmd_pub = new ros::Publisher[_joint_num];
    _cmd_pub[0] = _nh->advertise<std_msgs::Float64>("/lbr_iiwa/joint1_position_controller/command", 0);
    _cmd_pub[1] = _nh->advertise<std_msgs::Float64>("/lbr_iiwa/joint2_position_controller/command", 0);
    _cmd_pub[2] = _nh->advertise<std_msgs::Float64>("/lbr_iiwa/joint3_position_controller/command", 0);
    _cmd_pub[3] = _nh->advertise<std_msgs::Float64>("/lbr_iiwa/joint4_position_controller/command", 0);
    _cmd_pub[4] = _nh->advertise<std_msgs::Float64>("/lbr_iiwa/joint5_position_controller/command", 0);
    _cmd_pub[5] = _nh->advertise<std_msgs::Float64>("/lbr_iiwa/joint6_position_controller/command", 0);
    _cmd_pub[6] = _nh->advertise<std_msgs::Float64>("/lbr_iiwa/joint7_position_controller/command", 0);
}


bool CLIKPlugin::_initIK(){
    // Setup robot model
    std::string robot_desc_string;
    _nh->param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    // Setup inverse kinematics solver
    if (!iiwa_tree.getChain("lbr_iiwa_link_0", "lbr_iiwa_link_7", _k_chain)){
        ROS_ERROR("Failed to construct kdl chain");
        return false;
    }

    _fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );
    _ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 );

    _joint_num = _k_chain.getNrOfJoints();
    _q = new KDL::JntArray(_joint_num);

    _des_frame.M = KDL::Rotation::Identity();
    _des_frame.p = KDL::Vector(0.5, 0.5, 0.5);
    
    return true;
}


void CLIKPlugin::pos_cb( geometry_msgs::PoseConstPtr pose){
    _des_frame.M = KDL::Rotation::Quaternion(pose->orientation.x, pose->orientation.y, pose->orientation.z, pose->orientation.w);
    //_des_frame.M = KDL::Rotation::Identity();
    _des_frame.p = KDL::Vector(pose->position.x, pose->position.y, pose->position.z);

    cout << "Desired position: " << _des_frame.p.x() << " " << _des_frame.p.y()
            << " " << _des_frame.p.z() << endl;
    cout << "Desired orientation: " << pose->orientation.x <<  " " << pose->orientation.y
            << " " << pose->orientation.z << " " << pose->orientation.w << endl;
}


void CLIKPlugin::joint_states_cb(sensor_msgs::JointState js) {
	for(int i=0; i<_k_chain.getNrOfJoints(); i++ ) 
		_q->data[i] = js.position[i];
}


void CLIKPlugin::OnUpdate(){
    // Calc inverse kinematics
    KDL::JntArray q_out(_joint_num);
    int result = _ik_solver_pos->CartToJnt(*_q, _des_frame, *_q);
    if(result != KDL::SolverI::E_NOERROR ){
        cout << _ik_solver_pos->strError(result) << endl;
        return;
    }

    // Retrieve current position
    KDL::Frame f;
    _fksolver->JntToCart(*_q, f);
    //cout << "Current position: " << f.p.x() << " " << f.p.y() << " " << f.p.z() << endl;

    // Apply q to the robot
    std_msgs::Float64 cmd;
    for(int i=0; i<_joint_num; ++i) {
        cmd.data = _q->data[i];
        _cmd_pub[i].publish (cmd);
    }
}

}
