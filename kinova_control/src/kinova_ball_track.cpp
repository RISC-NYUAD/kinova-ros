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

ros::Publisher ee_vel_pub, reset_pub ;

struct RobotState{
	Eigen::VectorXd q;
	Eigen::Matrix4d T;
	Eigen::MatrixXd J;

	RobotState() : q(Eigen::VectorXd::Zero(7)), T(Eigen::Matrix4d::Identity()), J(Eigen::MatrixXd::Zero(6,7)) { }
};

struct SetpointState{
	Eigen::Vector3d ball_pos;
	Eigen::Vector3d xyn;
	bool invalid_pixels;
	double proximity;
	Eigen::Vector3d lin_vel;
	Eigen::Vector3d ang_vel;
	
	SetpointState() : ball_pos(Eigen::Vector3d(0.79,0.14,1.02)), xyn(Eigen::Vector3d(0,0,1)), invalid_pixels(true), proximity(0.0),
					  lin_vel(Eigen::Vector3d::Zero()), ang_vel(Eigen::Vector3d::Zero())  { }
};

void joints_cb(const sensor_msgs::JointState& );
Eigen::Matrix4d fkbas(Eigen::VectorXd );
void control_update(const ros::TimerEvent&);
void control_update_opt(const ros::TimerEvent&);
void do_position_ctrl(void);
void do_pixel_tracking(void);
void pixels_cb(const geometry_msgs::Twist& );
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
  ros::Subscriber pixels_sub = n_h.subscribe("/j2s7s300/ball_data", 1, &pixels_cb);       
//  ros::Subscriber pixels_sub = n_h.subscribe("/j2s7s300/ball_data", 1, &ranger_cb);       
  ee_vel_pub = n_h.advertise<geometry_msgs::Twist>("/j2s7s300/end_effector_vel_cmds", 1);
  reset_pub = n_h.advertise<std_msgs::Bool>("/j2s7s300/reset", 1);  
    
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

void pixels_cb(const geometry_msgs::Twist& msg){
	if(msg.linear.z < -0.9){
		if(!ROBOT_CMD.invalid_pixels){
			ROBOT_CMD.invalid_pixels = true;
/*			std_msgs::Bool res;
			res.data = true;
			reset_pub.publish(res);
*/
		}
		return;
	}else{
		ROBOT_CMD.invalid_pixels = false;
	}
	ROBOT_CMD.xyn = Eigen::Vector3d(msg.linear.x, msg.linear.y, 1.0);
	ROBOT_CMD.proximity = msg.linear.z ;	
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
	//do_position_ctrl();	

	do_pixel_tracking();

}

void do_pixel_tracking(void){
	if(ROBOT_CMD.invalid_pixels){
		geometry_msgs::Twist out_msg;	
		out_msg.linear.x = 0;
		out_msg.linear.y = 0;
		out_msg.linear.z = 0;
		out_msg.angular.x = 0;
		out_msg.angular.y = 0;
		out_msg.angular.z = 0;
		ee_vel_pub.publish(out_msg);
		return;
	}
	double x_dot_des = -13.0*sq_err(ROBOT_CMD.xyn(0));
	double y_dot_des = -13.0*sq_err(ROBOT_CMD.xyn(1));
	double weight = (1.0/(ROBOT_CMD.proximity+0.001));
	weight = (weight > 1.0) ? 1.0 : weight;
	weight = (weight < 0.55) ? 0.55 : weight;
	Eigen::Vector2d xy_dot_des = Eigen::Vector2d(x_dot_des, y_dot_des) ;

	double Z_hat = 0.75/(ROBOT_CMD.proximity + 0.15);
	if(Z_hat > 3.0){
		Z_hat = 3.0;
	}
/* FULL MAT
	Eigen::MatrixXd t_mat(2,6);
	t_mat(0,0) = -1.0/Z_hat;
	t_mat(0,1) = 0.0;
	t_mat(0,2) = ROBOT_CMD.xyn(0)/Z_hat;
	t_mat(0,3) = ROBOT_CMD.xyn(0) * ROBOT_CMD.xyn(1) ;
	t_mat(0,4) = -(1.0 + pow(ROBOT_CMD.xyn(0),2));
	t_mat(0,5) = ROBOT_CMD.xyn(1);
	t_mat(1,0) = 0.0;
	t_mat(1,1) = -1.0/Z_hat;
	t_mat(1,2) = ROBOT_CMD.xyn(1)/Z_hat;
	t_mat(1,3) = (1.0 + pow(ROBOT_CMD.xyn(1),2));
	t_mat(1,4) = -ROBOT_CMD.xyn(0)*ROBOT_CMD.xyn(1);
	t_mat(1,5) = -ROBOT_CMD.xyn(0);
*/	
	Eigen::MatrixXd t_mat(2,5);
	t_mat(0,0) = -1.0/Z_hat;
	t_mat(0,1) = 0.0;
	t_mat(0,2) = ROBOT_CMD.xyn(0) * ROBOT_CMD.xyn(1) ;
	t_mat(0,3) = -(1.0 + pow(ROBOT_CMD.xyn(0),2));
	t_mat(0,4) = ROBOT_CMD.xyn(1);
	t_mat(1,0) = 0.0;
	t_mat(1,1) = -1.0/Z_hat;
	t_mat(1,2) = (1.0 + pow(ROBOT_CMD.xyn(1),2));
	t_mat(1,3) = -ROBOT_CMD.xyn(0)*ROBOT_CMD.xyn(1);
	t_mat(1,4) = -ROBOT_CMD.xyn(0);

	double Vz = -0.03*(ROBOT_CMD.proximity - 33.0);
	double gain = 1.0/(10.0*(abs(ROBOT_CMD.xyn(0)) + abs(ROBOT_CMD.xyn(1))));
	gain = (gain>1.0) ? 1.0 : gain;
	Vz *= gain;
/*	if(abs(ROBOT_CMD.xyn(0))>0.26 || abs(ROBOT_CMD.xyn(1)>0.26)){
		Vz = 0.0;
	}
*/	Eigen::Vector2d correction_vz;
	correction_vz(0) = ROBOT_CMD.xyn(0)*Vz/Z_hat;
	correction_vz(1) = ROBOT_CMD.xyn(1)*Vz/Z_hat;

	Eigen::Vector2d lhs = xy_dot_des - correction_vz ;
	Eigen::MatrixXd M(5,2);
	M = Utils::pinv(t_mat, 0.001);
	Eigen::VectorXd camera_vels = M * lhs ;
	
	Eigen::Vector3d lin_vel = 0.05 * ROBOT.T.block(0,0,3,3) * Eigen::Vector3d(camera_vels(0), camera_vels(1), Vz);
	Eigen::Vector3d ang_vel = 0.05 * ROBOT.T.block(0,0,3,3) * Eigen::Vector3d(camera_vels(2), camera_vels(3), camera_vels(4));

	//ROBOT_CMD.lin_vel = 0.95*ROBOT_CMD.lin_vel + 0.05*lin_vel;
	//ROBOT_CMD.ang_vel = 0.95*ROBOT_CMD.ang_vel + 0.05*ang_vel;
	ROBOT_CMD.ang_vel = ang_vel;
	ROBOT_CMD.lin_vel = lin_vel;
	
	geometry_msgs::Twist out_msg;	
	out_msg.linear.x = ROBOT_CMD.lin_vel(0);
	out_msg.linear.y = ROBOT_CMD.lin_vel(1);
	out_msg.linear.z = ROBOT_CMD.lin_vel(2);
	out_msg.angular.x = ROBOT_CMD.ang_vel(0);
	out_msg.angular.y = ROBOT_CMD.ang_vel(1);
	out_msg.angular.z = ROBOT_CMD.ang_vel(2);
	ee_vel_pub.publish(out_msg);
}

void do_position_ctrl(void){
	Eigen::Vector3d ee_w_pos_error = ROBOT.T.block(0,3,3,1) - ROBOT_CMD.ball_pos;
	Eigen::Vector3d lin_vels = -0.05*sq_err(ee_w_pos_error);
	geometry_msgs::Twist out_msg;

	Eigen::Vector3d z = ROBOT.T.block(0,2,3,1);
	Eigen::Vector3d P_ = -ee_w_pos_error;
	double P_norm = P_.norm() + 0.001;
	Eigen::Vector3d unit_P = (1.0/P_norm) * P_ ;	

	double error_func = 1.0 - z.transpose() * unit_P ;
	
	double eta = 0.3;
	Eigen::Vector3d omega = Utils::cross(unit_P, -eta*z);
	if(P_norm < 0.15){
		omega *= P_norm;
		lin_vels *= 5.0*P_norm;
	}
	if(error_func < 0.2){
		out_msg.linear.x = lin_vels(0);
		out_msg.linear.y = lin_vels(1);
		out_msg.linear.z = lin_vels(2);
	}

	//Eigen::Vector3d omega = ROBOT.T.block(0,0,3,3) * omega_ ;
	out_msg.angular.x = omega(0);
	out_msg.angular.y = omega(1);
	out_msg.angular.z = omega(2);	

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

void control_update_opt(const ros::TimerEvent& e){ //KINDA OVERKILL, DEPRECATED
	ROBOT.T = fkbas(ROBOT.q); 
	Eigen::Vector3d ee_w_pos_error = ROBOT.T.block(0,3,3,1) - ROBOT_CMD.ball_pos;
	Eigen::Vector3d lin_vels = -0.15*sq_err(ee_w_pos_error);
	geometry_msgs::Twist out_msg;

/*
	out_msg.linear.x = lin_vels(0);
	out_msg.linear.y = lin_vels(1);
	out_msg.linear.z = lin_vels(2);
*/
	Eigen::Vector3d P_ = -ee_w_pos_error;
	double P_norm = P_.norm() + 0.001;
	Eigen::Vector3d unit_P = (1.0/P_norm) * P_ ;	
//	unit_P << 0.0, 1.0, 0.0 ;	
	
	Eigen::Vector3d nz = ROBOT.T.block(0,2,3,1);
	double error = 1.0 - nz.transpose() * unit_P ;
	std::cout << "===============" << std::endl;
	std::cout << unit_P.transpose() << std::endl;
	std::cout << nz.transpose() << std::endl;
	std::cout << "===============" << std::endl;
	Eigen::Matrix3d R_S = Utils::S(nz) ; 
	Eigen::Vector3d A_column = (unit_P.transpose() * R_S).transpose() ;
	
	double lambda = 1.5;
	Eigen::Matrix3d Hm =  2.0 * ( A_column * (A_column.transpose()) + lambda * Eigen::Matrix3d::Identity() );
	double b_hat = 0.9 * sq_err(error) ;
	Eigen::Vector3d fm = 2.0 * b_hat * A_column ;
	
	Eigen::MatrixXd Am(6,3);
	Am.block(0,0,3,3) = Eigen::Matrix3d::Identity();
	Am.block(3,0,3,3) = -Eigen::Matrix3d::Identity();
	Eigen::VectorXd bm(6);
	bm << -0.1, -0.1, -0.1, -0.1, -0.1, -0.1 ;
	Eigen::VectorXd omega = Utils::solveQP(Hm, fm, Am, bm);
	if(!(omega.size()>0)){
		ROS_ERROR("UNSOLVABLE");
		omega = Eigen::VectorXd::Zero(3) ;
	}
	out_msg.angular.x = omega(0);
	out_msg.angular.y = omega(1);
	out_msg.angular.z = omega(2);


	ee_vel_pub.publish(out_msg);
}














