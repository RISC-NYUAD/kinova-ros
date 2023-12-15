#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <string>
#include <time.h>
#include <sstream>
#include <Eigen/Dense>

//#define WITH_CONTROL 

namespace gazebo
{
class MobileBasePlugin : public ModelPlugin
{

public: ros::NodeHandle nh;
public: ros::Subscriber base_vel_sub;
public: ros::Publisher base_pose_pub;
public: Eigen::Vector3d J_f;
private: double Kp, Kd, Ki, vx_cmd, vy_cmd, wz_cmd, x_err_prev, x_err_dot, y_err_prev, y_err_dot, w_err_prev, w_err_dot;
public: double LIM_;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
	  	this->model = _parent;			
		J_f = Eigen::Vector3d::Zero();
		this->Kp = 1.0;
		this->Kd = 0.5;
		this->Ki = 0.1;
		this->vx_cmd = 0.0;
		this->vy_cmd = 0.0;
		this->wz_cmd = 0.0;
		LIM_ = 1.0;
		this->x_err_dot = 0.0;
		this->x_err_prev = 0.0;
		this->y_err_dot = 0.0;
		this->y_err_prev = 0.0;
		this->w_err_dot = 0.0;
		this->w_err_prev = 0.0;
		
					
		if(_sdf->HasElement("Kp"))
			this->Kp = _sdf->GetElement("Kp")->Get<double>();				
		if(_sdf->HasElement("Kd"))
			this->Kd = _sdf->GetElement("Kd")->Get<double>();				
		if(_sdf->HasElement("Ki"))
			this->Ki = _sdf->GetElement("Ki")->Get<double>();				

		std::string cmdTopic = "/j2s7s300/mobile_base/cmd_vel" ;
		std::string poseTopic = "/j2s7s300/mobile_base/pose" ;

		base_vel_sub = nh.subscribe(cmdTopic, 1, &MobileBasePlugin::vel_callback, this); 
		base_pose_pub = nh.advertise<geometry_msgs::Pose>(poseTopic, 1);
				
	  	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MobileBasePlugin::onUpdate, this));
		
	}  

public: void onUpdate()
	{
		std::string link_name = "j2s7s300::base_link" ;
		physics::LinkPtr base_link = this->model->GetLink(link_name) ;
		ignition::math::Vector3d lin_vel = base_link->RelativeLinearVel();
		ignition::math::Vector3d ang_vel = base_link->RelativeAngularVel();		

	    ignition::math::Pose3d pose = base_link->WorldCoGPose();
	    ignition::math::Quaternion orientation = pose.Rot();
	    ignition::math::Vector3d position = pose.Pos();
	    geometry_msgs::Pose out_msg;
	    out_msg.position.x = position.X();	
	    out_msg.position.y = position.Y();
	    out_msg.position.z = position.Z();
	    out_msg.orientation.x = orientation.X();
	    out_msg.orientation.y = orientation.Y();
	    out_msg.orientation.z = orientation.Z();
	    out_msg.orientation.w = orientation.W();	    	    	
		base_pose_pub.publish(out_msg);

		#ifdef WITH_CONTROL

		double x_error = lin_vel.X() - this->vx_cmd;
		double y_error = lin_vel.Y() - this->vy_cmd;		
		double w_error = ang_vel.Z() - this->wz_cmd;
		
		double x_error_dot = (x_error - this->x_err_prev)/0.002;
		this->x_err_dot = 0.3*x_error_dot + 0.7*this->x_err_dot;
		J_f(0) += 0.002 * x_error;
		if(J_f(0)>LIM_){
			J_f(0) = LIM_;
		}if(J_f(0)<-LIM_){
			J_f(0) = -LIM_;
		}
		double f_x = -(this->Kp)*x_error - (this->Kd)*(this->x_err_dot) - (this->Ki)*J_f(0);
		f_x *= 100.0;
		this->x_err_prev = x_error;

		double y_error_dot = (y_error - this->y_err_prev)/0.002;
		this->y_err_dot = 0.3*y_error_dot + 0.7*this->y_err_dot;
		J_f(1) += 0.002 * y_error;
		if(J_f(1)>LIM_){
			J_f(1) = LIM_;
		}if(J_f(1)<-LIM_){
			J_f(1) = -LIM_;
		}
		double f_y = -(this->Kp)*y_error - (this->Kd)*(this->y_err_dot) - (this->Ki)*J_f(1);
		f_y *= 100.0;
		this->y_err_prev = y_error;

		double w_error_dot = (w_error - this->w_err_prev)/0.002;
		this->w_err_dot = 0.3*w_error_dot + 0.7*this->w_err_dot;
		J_f(2) += 0.002 * w_error;
		if(J_f(2)>LIM_){
			J_f(2) = LIM_;
		}if(J_f(2)<-LIM_){
			J_f(2) = -LIM_;
		}
		double m_z = -(this->Kp)*w_error - (this->Kd)*(this->w_err_dot) - (this->Ki)*J_f(2);
		m_z *= 60.0;
		this->w_err_prev = w_error;

		base_link->AddForce(ignition::math::Vector3d(f_x,f_y,0.0));
		base_link->AddTorque(ignition::math::Vector3d(0.0,0.0,m_z));
		
		#else
		ignition::math::Matrix3d R_bw(orientation);
		ignition::math::Vector3d vel_local = R_bw * ignition::math::Vector3d(this->vx_cmd,this->vy_cmd,0.0);
		
		base_link->SetLinearVel(vel_local);
		base_link->SetAngularVel(ignition::math::Vector3d(0.0,0.0,this->wz_cmd));		
		
		#endif
		
	}

public: void vel_callback(const geometry_msgs::Twist& msg)
	{
		this->vx_cmd = msg.linear.x;
		this->vy_cmd = msg.linear.y; 
		this->wz_cmd = msg.angular.z;
		return;		
	}


};
GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}























