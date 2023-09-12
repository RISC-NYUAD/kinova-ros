#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cstring>
#include <tf/tf.h>
#include <ros/ros.h>
#include <string>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>
#include "libraries/utils.h"
#include <mutex>

ros::Publisher ee_vel_pub ;

struct RobotState{
	Eigen::VectorXd q;
	Eigen::Matrix4d T;
	Eigen::MatrixXd J;

	RobotState() : q(Eigen::VectorXd::Zero(7)), T(Eigen::Matrix4d::Identity()), J(Eigen::MatrixXd::Zero(6,7)) { }
};

struct SetpointState{
	Eigen::Vector3d ball_pos;

	SetpointState() : ball_pos(Eigen::Vector3d(0.79,0.14,1.02)) { }
};

void joints_cb(const sensor_msgs::JointState& );
Eigen::Matrix4d fkbas(Eigen::VectorXd );
void control_update(const ros::TimerEvent&);
double lp_filt(double, double, double);
Eigen::Vector3d sq_err(Eigen::Vector3d );
double sq_err(double );
double sign(double );

RobotState ROBOT;
SetpointState ROBOT_CMD;
double c_dt;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_ee_track_node");		
  ros::NodeHandle n_h;
  double frequency = 50;
  c_dt = (1.0/frequency);
  ros::Rate rate(frequency);

  ros::Subscriber pose_sub = n_h.subscribe("/j2s7s300/joint_states", 1, &joints_cb);  
  ee_vel_pub = n_h.advertise<geometry_msgs::Twist>("/j2s7s300/end_effector_vel_cmds", 1);
    
  ros::Timer ctrl_timer = n_h.createTimer(ros::Duration(1.0/frequency), &control_update);  

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  
}

void joints_cb(const sensor_msgs::JointState& msg){
	for(size_t i = 0 ; i < 7 ; i++){
		ROBOT.q(i) = msg.position[i] ;
	}
}

Eigen::Matrix4d fkbas(Eigen::VectorXd x){
	double q1, q2, q3, q4, q5, q6, q7;
	q1 = x(0);	q3 = x(2);	q5 = x(4);
	q2 = x(1);	q4 = x(3);	q6 = x(5);	q7 = x(6);
	
	Eigen::Matrix4d Abase2tip ;
	Abase2tip(0,0) = cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) - cos(q1)*cos(q2)*sin(q3)))) + sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) - cos(q1)*cos(q2)*sin(q3)));
	Abase2tip(0,1) = sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) - cos(q1)*cos(q2)*sin(q3)))) - cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q3)*sin(q1) - cos(q1)*cos(q2)*sin(q3)));
	Abase2tip(0,2) = sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) - cos(q1)*cos(q2)*sin(q3))) - cos(q6)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2));
	Abase2tip(0,3) = sin(q1)/625.0 + (41.0*cos(q1)*sin(q2))/100.0 - (57.0*cos(q3)*sin(q1))/5000.0 - (873.0*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) - cos(q1)*cos(q4)*sin(q2)))/4000.0 + (873.0*sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)) + cos(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q3)*sin(q1) - cos(q1)*cos(q2)*sin(q3))))/4000.0 + (6221.0*sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q2)*cos(q3)))/20000.0 + (57.0*cos(q1)*cos(q2)*sin(q3))/5000.0 - (6221.0*cos(q1)*cos(q4)*sin(q2))/20000.0;
	Abase2tip(1,0) = cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) + cos(q2)*sin(q1)*sin(q3)))) + sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) + cos(q2)*sin(q1)*sin(q3)));
	Abase2tip(1,1) = sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)) + cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) + cos(q2)*sin(q1)*sin(q3)))) - cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) + cos(q5)*(cos(q1)*cos(q3) + cos(q2)*sin(q1)*sin(q3)));
	Abase2tip(1,2) = sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) + cos(q2)*sin(q1)*sin(q3))) - cos(q6)*(sin(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2));
	Abase2tip(1,3) = cos(q1)/625.0 - (57.0*cos(q1)*cos(q3))/5000.0 - (41.0*sin(q1)*sin(q2))/100.0 - (873.0*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q1)*sin(q2)))/4000.0 + (873.0*sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - sin(q1)*sin(q2)*sin(q4)) - sin(q5)*(cos(q1)*cos(q3) + cos(q2)*sin(q1)*sin(q3))))/4000.0 + (6221.0*sin(q4)*(cos(q1)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/20000.0 - (57.0*cos(q2)*sin(q1)*sin(q3))/5000.0 + (6221.0*cos(q4)*sin(q1)*sin(q2))/20000.0;
	Abase2tip(2,0) = - sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)));
	Abase2tip(2,1) = cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + cos(q5)*sin(q2)*sin(q3)) - sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)));
	Abase2tip(2,2) = - sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4));
	Abase2tip(2,3) = (6221.0*cos(q2)*cos(q4))/20000.0 - (41.0*cos(q2))/100.0 + (57.0*sin(q2)*sin(q3))/5000.0 - (873.0*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)))/4000.0 - (873.0*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/4000.0 + (6221.0*cos(q3)*sin(q2)*sin(q4))/20000.0 + 551.0/2000.0 + 0.28;
	Abase2tip(3,0) = 0.0;
	Abase2tip(3,1) = 0.0;
	Abase2tip(3,2) = 0.0;
	Abase2tip(3,3) = 1.0;
	return Abase2tip ;
}


void control_update(const ros::TimerEvent& e){
	ROBOT.T = fkbas(ROBOT.q); 
	Eigen::Vector3d ee_w_pos_error = ROBOT.T.block(0,3,3,1) - ROBOT_CMD.ball_pos;
	std::cout << ee_w_pos_error.transpose() << std::endl;
	Eigen::Vector3d lin_vels = -0.15*sq_err(ee_w_pos_error);
	geometry_msgs::Twist out_msg;
	out_msg.linear.x = lin_vels(0);
	out_msg.linear.y = lin_vels(1);
	out_msg.linear.z = lin_vels(2);
	//TODO: Add orientation control as well
	ee_vel_pub.publish(out_msg);
}

double lp_filt(double x1, double x2, double a){
	return (x1*a + x2*(1-a));
}

double sign(double x){
	if(x>0.0){
		return 1.0;
	}else{
		if(x<0.0){
			return -1.0;
		}else{
			return 0.0;
		}
	}
}

double sq_err(double x){
	return (sign(x) * sqrt(abs(x)));	
}

Eigen::Vector3d sq_err(Eigen::Vector3d x){
	Eigen::Vector3d result;
	for(int i = 0 ; i < 3 ; i++){
		result(i) = sq_err(x(i));
	}
	return result;
}
















