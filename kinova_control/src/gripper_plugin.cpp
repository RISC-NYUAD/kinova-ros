#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iostream>
#include <string>
#include <time.h>
#include <sstream>
#include <Eigen/Dense>


namespace gazebo
{
class KinovaGripperPlugin : public ModelPlugin
{

public: ros::NodeHandle nh;
public: ros::Subscriber finger_trajectory_sub, finger_tip_1_sub, finger_tip_2_sub, finger_tip_3_sub;
public: Eigen::Vector3d f_cmd, ft_cmd ;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
	  	this->model = _parent;			
		f_cmd = Eigen::Vector3d::Zero();
		ft_cmd = Eigen::Vector3d::Zero();
					
		std::string fingerTopic = "/j2s7s300/effort_finger_trajectory_controller/command" ;
		std::string tip1_Topic = "/j2s7s300/finger_tip_1_position_controller/command" ;
		std::string tip2_Topic = "/j2s7s300/finger_tip_2_position_controller/command" ;		
		std::string tip3_Topic = "/j2s7s300/finger_tip_3_position_controller/command" ;

		finger_trajectory_sub = nh.subscribe(fingerTopic, 1, &KinovaGripperPlugin::finger_traj_callback, this); 
		finger_tip_1_sub = nh.subscribe(tip1_Topic, 1, &KinovaGripperPlugin::tip_1_callback, this); 
		finger_tip_2_sub = nh.subscribe(tip2_Topic, 1, &KinovaGripperPlugin::tip_2_callback, this); 		
		finger_tip_3_sub = nh.subscribe(tip3_Topic, 1, &KinovaGripperPlugin::tip_3_callback, this); 
		
	  	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&KinovaGripperPlugin::onUpdate, this));
		
	}  

public: void onUpdate()
	{
		std::string preface = "j2s7s300::j2s7s300_joint_finger_" ;
		std::string preface_tip = "tip_" ;
		physics::JointPtr motor_joint;
		double torque = 0.0;
		for(int i = 0 ; i < 3 ; i++){
			motor_joint = this->model->GetJoint(preface + std::to_string(i+1));
			double angle = motor_joint->Position(0);
			double rate = motor_joint->GetVelocity(0);
			torque = -0.01*(angle - f_cmd(i)) - 0.0006*rate;			
			motor_joint->SetForce(0,torque);
		}
		for(int i = 0 ; i < 3 ; i++){
			motor_joint = this->model->GetJoint(preface + preface_tip + std::to_string(i+1));
			double angle = motor_joint->Position(0);
			double rate = motor_joint->GetVelocity(0);
			torque = -0.01*(angle - ft_cmd(i)) - 0.0006*rate;			
			motor_joint->SetForce(0,torque);
		}		

	}

public: void finger_traj_callback(const trajectory_msgs::JointTrajectory& msg)
	{
		trajectory_msgs::JointTrajectoryPoint temp = msg.points[0];
		f_cmd << temp.positions[0], temp.positions[1], temp.positions[2] ;
		return;		
	}

public: void tip_1_callback(const std_msgs::Float64& msg)
	{
		ft_cmd(0) = msg.data;
		return;		
	}

public: void tip_2_callback(const std_msgs::Float64& msg)
	{
		ft_cmd(1) = msg.data;
		return;		
	}

public: void tip_3_callback(const std_msgs::Float64& msg)
	{
		ft_cmd(2) = msg.data;
		return;		
	}


};
GZ_REGISTER_MODEL_PLUGIN(KinovaGripperPlugin)
}























