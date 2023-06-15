#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <string>

namespace gazebo {
class BallHoldSensor : public ModelPlugin {
public:
  ros::NodeHandle nh;
  
public:
  ros::Subscriber gain_sub, reset_sub, finger_sub;
  
public:
  physics::LinkPtr ball;
  
public:
  double ft1, close_time;

private:
  physics::ModelPtr model;

private:
  physics::WorldPtr world;
  
private:
  double applied_gain;
  
private:
  bool uninitialized;

private:
  double seq;

private:
  event::ConnectionPtr updateConnection;

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->applied_gain = 0;
    this->seq = 0;
    this->uninitialized = true ;
    ft1 = 0;
    close_time = 0;
    

    std::string gainTopicName = "/j2s7s300/ball/gain";
    std::string resetTopicName = "/j2s7s300/ball/reset";    
    std::string fingerTopicName = "/j2s7s300/finger_tip_1_position_controller/command";    

    gain_sub = nh.subscribe(gainTopicName, 1, &BallHoldSensor::onGain, this);    
    finger_sub = nh.subscribe(fingerTopicName, 1, &BallHoldSensor::onGripperAction, this);        
    reset_sub = nh.subscribe(resetTopicName, 1, &BallHoldSensor::onReset, this);    
    
    std::string ballName = "ball::ball/base_link";
    ball = this->model->GetLink(ballName);    
    
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BallHoldSensor::onUpdate, this));
  }

public: 
  void setBallForces() {
    ignition::math::Vector3d des_ball_position = ignition::math::Vector3d(1.0,0.0,1.0) ;
    
    //ignition::math::Pose3d ball_pose(ball_position, ignition::math::Quaternion(1.0,0.0,0.0,0.0));
  	//ball->SetWorldPose(ball_pose);
  	ignition::math::Vector3d ball_position = ball->WorldCoGPose().Pos();
	ignition::math::Quaternion ball_quat = ball->WorldCoGPose().Rot();
	ignition::math::Vector3d ball_velocity = ball_quat.RotateVector(ball->RelativeLinearVel()) ; 
	
  	double fx = -1.5 * (ball_position.X() - des_ball_position.X()) - 0.007*ball_velocity.X();
  	double fy = -1.5 * (ball_position.Y() - des_ball_position.Y()) - 0.007*ball_velocity.Y();
  	double fz = -1.5 * (ball_position.Z() - des_ball_position.Z()) - 0.03*ball_velocity.Z() + 0.05*9.81;
  	fx *= this->applied_gain ;
  	fy *= this->applied_gain ;
  	fz *= this->applied_gain ;
  	ball->AddForceAtRelativePosition(ignition::math::Vector3d(fx,fy,fz), ignition::math::Vector3d(0.0,0.0,0.0));
  }

public: 
  void resetBallPosition() {
    ignition::math::Vector3d des_ball_position = ignition::math::Vector3d(1.0,0.0,1.0) ;
    
    ignition::math::Pose3d ball_pose(des_ball_position, ignition::math::Quaternion(1.0,0.0,0.0,0.0));
	ball->ResetPhysicsStates();	
  	ball->SetWorldPose(ball_pose);	
  }


public:
  void onUpdate() {
    if(this->applied_gain > 0){
		setBallForces();
	}
  }

public:
  void onGripperAction(const std_msgs::Float64& msg){
  	if(msg.data > 0.1){
  		if(ft1 < 0.1){
  			close_time = ros::Time::now().toSec();
  		}
  		ft1 = msg.data;
  		double dt = ros::Time::now().toSec()-close_time; 
//  		ROS_WARN("--%.2f\n",dt);
  		if(dt>3.0){
	  		this->applied_gain = 0;  			
  		}
  	}else{
  		ft1 = msg.data;
  		close_time = 0.0;
  	}
  }

public: 
  void onGain(const std_msgs::Int16& msg) {
	if(msg.data >0){
		this->applied_gain = 1;
	}else{
		this->applied_gain = 0;
	}
  }

public:
  void onReset(const std_msgs::Bool& msg) {
	if(this->uninitialized){
		this->uninitialized = false;
    }
	resetBallPosition();
	this->applied_gain = 1 ;	
  }


};
GZ_REGISTER_MODEL_PLUGIN(BallHoldSensor)
} // namespace gazebo





