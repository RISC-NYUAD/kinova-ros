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
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>
#include "libraries/utils.h"
#include <mutex>

ros::Publisher joint_cmd_pub, mobile_base_pub ;
trajectory_msgs::JointTrajectory q_msg;

struct RobotState{
	Eigen::VectorXd q;
	Eigen::Matrix4d T;
	Eigen::MatrixXd J;
	double initing_dt;
	Eigen::Matrix4d T_base_world;
	Eigen::Vector3d base_xyw;

	RobotState() : q(Eigen::VectorXd::Zero(7)),
					T(Eigen::Matrix4d::Identity()), J(Eigen::MatrixXd::Zero(6,10)), initing_dt(0.0), T_base_world(Eigen::Matrix4d::Identity()), base_xyw(Eigen::Vector3d::Zero()) { }
};

struct SetpointState{
	Eigen::VectorXd q_des;
	Eigen::VectorXd qdot_des;
	bool mode; //0 for joint vels, 1 for ee vels
	Eigen::VectorXd ee_vel;

	SetpointState() : q_des(Eigen::VectorXd::Zero(7)), qdot_des(Eigen::VectorXd::Zero(7)), mode(0), ee_vel(Eigen::VectorXd::Zero(6)) { }
};


Eigen::Matrix3d Rz(double );
void joints_cb(const sensor_msgs::JointState& );
void ee_vel_cb(const geometry_msgs::Twist& );
void base_pose_cb(const geometry_msgs::Pose& );
void joint_vel_cb(const std_msgs::Float32MultiArray& );
void fk_(void);
void run_ee_vel_ctrl(void);
Eigen::Matrix4d fkbas(Eigen::VectorXd , Eigen::Vector3d );
void control_update(const ros::TimerEvent&);
void reset_cb(const std_msgs::Bool& );
void init_params(); //sets the correct initial joint angle setpoints
double lp_filt(double, double, double);
Eigen::Matrix3d TFtoEigen(tf::Matrix3x3 );

RobotState ROBOT;
SetpointState ROBOT_CMD;
double c_dt;

Eigen::VectorXd Q_U(7), Q_L(7);
std::mutex mtx;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_ctrl_node");		
  ros::NodeHandle n_h;
  double frequency = 200;
  c_dt = (1.0/frequency);
  ros::Rate rate(frequency);
  
  init_params();
	
  Q_U << 6.2, 5.4, 6.2, 5.7, 5.5, 5.1, 5.5 ;
  Q_L << -6.2, 0.84, -6.2, 0.55, -5.5, 1.16, -5.5 ;

  ros::Subscriber pose_sub = n_h.subscribe("/j2s7s300/joint_states", 1, &joints_cb);  
  ros::Subscriber base_pose_sub = n_h.subscribe("/j2s7s300/mobile_base/pose", 1, &base_pose_cb);    
  ros::Subscriber ee_vel_cmd_sub = n_h.subscribe("/j2s7s300/end_effector_vel_cmds", 1, &ee_vel_cb);
  ros::Subscriber joint_vel_cmd_sub = n_h.subscribe("/j2s7s300/joint_vel_cmds", 1, &joint_vel_cb);
  ros::Subscriber reset_sub = n_h.subscribe("/j2s7s300/reset", 1, &reset_cb);  
  
  joint_cmd_pub = n_h.advertise<trajectory_msgs::JointTrajectory>("/j2s7s300/effort_joint_trajectory_controller/command", 1);
/*  finger_cmd_pub = n_h.advertise<trajectory_msgs::JointTrajectory>("/j2s7s300/effort_finger_trajectory_controller/command", 1);  
  finger_tip_1_cmd_pub = n_h.advertise<std_msgs::Float64>("/j2s7s300/finger_tip_1_position_controller/command", 1);  
  finger_tip_2_cmd_pub = n_h.advertise<std_msgs::Float64>("/j2s7s300/finger_tip_2_position_controller/command", 1);      
  finger_tip_3_cmd_pub = n_h.advertise<std_msgs::Float64>("/j2s7s300/finger_tip_3_position_controller/command", 1);  
*/
  mobile_base_pub = n_h.advertise<geometry_msgs::Twist>("/j2s7s300/mobile_base/cmd_vel", 1);

  ros::Timer ctrl_timer = n_h.createTimer(ros::Duration(1.0/frequency), &control_update);  
  

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  
}

void base_pose_cb(const geometry_msgs::Pose& msg){
	ROBOT.T_base_world.block(0,3,3,1) = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z-0.14);
	tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
	tf::Matrix3x3 R(q);
	double roll, pitch, yaw;
	R.getRPY(roll, pitch, yaw);
	ROBOT.T_base_world.block(0,0,3,3) = TFtoEigen(R);	
	ROBOT.base_xyw = Eigen::Vector3d(msg.position.x, msg.position.y, yaw);
}

Eigen::Matrix3d TFtoEigen(tf::Matrix3x3 mat){
  Eigen::Matrix3d R_des;
  for(int i = 0 ; i < 3 ; i++){
  	for(int j = 0 ; j < 3 ; j++){
  		R_des(i,j) = mat[i][j];
  	}
  }
  return R_des;
}

void joints_cb(const sensor_msgs::JointState& msg){
	mtx.lock();
	for(size_t i = 0 ; i < 7 ; i++){
		ROBOT.q(i) = msg.position[i] ;
	}
/*	for(size_t i = 7 ; i < 10 ; i++){
		ROBOT.finger_1(i-7) = msg.position[i];
	}
	for(size_t i = 10 ; i < 13 ; i++){
		ROBOT.finger_2(i-10) = msg.position[i];
	} 
*/	mtx.unlock();
}

Eigen::Matrix4d fkbas(Eigen::VectorXd x, Eigen::Vector3d base_xyw ){
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
	Abase2tip(2,3) = (6221.0*cos(q2)*cos(q4))/20000.0 - (41.0*cos(q2))/100.0 + (57.0*sin(q2)*sin(q3))/5000.0 - (873.0*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - sin(q2)*sin(q3)*sin(q5)))/4000.0 - (873.0*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/4000.0 + (6221.0*cos(q3)*sin(q2)*sin(q4))/20000.0 + 551.0/2000.0 + 0.29;
	Abase2tip(3,0) = 0.0;
	Abase2tip(3,1) = 0.0;
	Abase2tip(3,2) = 0.0;
	Abase2tip(3,3) = 1.0;

	Eigen::Matrix4d T_base_world = Eigen::Matrix4d::Identity();
	T_base_world.block(0,0,3,3) = Rz(base_xyw(2));
	T_base_world.block(0,3,3,1) = Eigen::Vector3d(base_xyw(0), base_xyw(1), 0.0);

	Eigen::Matrix4d Final = T_base_world * Abase2tip ;
	return Final ;
}

Eigen::Matrix3d Rz(double a){
  Eigen::Matrix4d R_ = Utils::rotz(a);
  return R_.block(0,0,3,3);
}

void fk_(void){
	Eigen::Matrix4d current_T = fkbas(ROBOT.q, ROBOT.base_xyw);
	Eigen::MatrixXd J_v(3,10), J_w(3,10);
	J_v.block(0,0,2,2) = Eigen::Matrix2d::Identity();
	J_w.block(0,0,2,2) = Eigen::Matrix2d::Zero();
	

	double epsilon = 0.01;

	Eigen::Vector3d base_p, base_n;
	base_p = ROBOT.base_xyw;
	base_n = ROBOT.base_xyw;
	base_p(2) = base_p(2) + epsilon;
	base_n(2) = base_n(2) - epsilon;
	Eigen::Matrix4d deltaT = (0.5/epsilon)*(fkbas(ROBOT.q,base_p)-fkbas(ROBOT.q,base_n));
	J_v.col(2) = deltaT.block(0,3,3,1);	
	Eigen::Matrix3d S_w = deltaT.block(0,0,3,3) * ((current_T.block(0,0,3,3)).transpose()) ;
	Eigen::Vector3d w = Eigen::Vector3d(S_w(2,1), S_w(0,2), S_w(1,0));
	J_w.col(2) = w ;

	Eigen::VectorXd x_p(7), x_n(7);
	for(int i = 0 ; i < 7 ; i++){
		x_p = ROBOT.q;
		x_n = ROBOT.q;
		x_p(i) = x_p(i) + epsilon;
		x_n(i) = x_n(i) - epsilon;
		Eigen::Matrix4d deltaT = (0.5/epsilon)*(fkbas(x_p,ROBOT.base_xyw)-fkbas(x_n,ROBOT.base_xyw));
		J_v.col(i+3) = deltaT.block(0,3,3,1) ;
		Eigen::Matrix3d S_w = deltaT.block(0,0,3,3) * ((current_T.block(0,0,3,3)).transpose()) ;
		Eigen::Vector3d w = Eigen::Vector3d(S_w(2,1), S_w(0,2), S_w(1,0));
		J_w.col(i+3) = w ;
	}
	Eigen::MatrixXd current_J(6,10);
	current_J.block(0,0,3,10) = J_v;
	current_J.block(3,0,3,10) = J_w;
	ROBOT.T = current_T;
	ROBOT.J = current_J; //saved them globally, no need to call fk again in this run		
	
}

void ee_vel_cb(const geometry_msgs::Twist& msg){
	Eigen::VectorXd vels(6);
	vels << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z ;
	ROBOT_CMD.ee_vel = vels;
	ROBOT_CMD.mode = 1;
}

void run_ee_vel_ctrl(void){
	mtx.lock();
	fk_(); //updates internal variables for EE_transform and EE_Jacobians

	Eigen::MatrixXd pseudo = Utils::pinv(ROBOT.J, 0.001) ;
	Eigen::VectorXd q_dots(10);
	q_dots = pseudo * ROBOT_CMD.ee_vel ;

	Eigen::Matrix3d R_wb = Rz(ROBOT.base_xyw(2)).transpose();
	Eigen::Vector3d v_base = R_wb * Eigen::Vector3d(q_dots(0), q_dots(1), 0.0);
	geometry_msgs::Twist base_msg;
	base_msg.linear.x = v_base(0);
	base_msg.linear.y = v_base(1);
	base_msg.angular.z = q_dots(2);
	mobile_base_pub.publish(base_msg);
	for(int i = 0 ; i < 7 ; i++){
		ROBOT_CMD.qdot_des(i) = q_dots(i+3) ;
	}
	mtx.unlock();
}

void joint_vel_cb(const std_msgs::Float32MultiArray& msg){
	for(size_t i = 0 ; i < 7 ; i++){
		ROBOT_CMD.qdot_des(i) = msg.data[i];
	}
	ROBOT_CMD.mode = 0;
}

void control_update(const ros::TimerEvent& e){

	ROBOT.initing_dt += c_dt;
	if(ROBOT.initing_dt < 10.0){
		if(ROBOT.initing_dt > 5.0){
			ROBOT_CMD.q_des(0) = -0.133;
			ROBOT_CMD.q_des(1) = 3.14;
			ROBOT_CMD.q_des(2) = 0.0;
			ROBOT_CMD.q_des(3) = 4.0;
			ROBOT_CMD.q_des(4) = 0.147;
			ROBOT_CMD.q_des(5) = 2.0;
			ROBOT_CMD.q_des(6) = -1.5708;
		}
	}

	if(ROBOT_CMD.mode == 1){		
	  run_ee_vel_ctrl(); //updates ROBOT_CMD.qdot_des
	}
	
	for(size_t i = 0 ; i < 7 ; i++){	
		ROBOT_CMD.q_des(i) += ROBOT_CMD.qdot_des(i) * c_dt ;
		ROBOT_CMD.q_des(i) = (ROBOT_CMD.q_des(i)>Q_U(i)) ? Q_U(i) : ROBOT_CMD.q_des(i) ;
		ROBOT_CMD.q_des(i) = (ROBOT_CMD.q_des(i)<Q_L(i)) ? Q_L(i) : ROBOT_CMD.q_des(i) ;
	}					
//	ROS_WARN("STATE IS %d, CMD IS %.1f", ROBOT_CMD.fingers_closed, fingers);

	trajectory_msgs::JointTrajectoryPoint q_temp;
	q_temp.time_from_start.sec = 1;
	q_temp.time_from_start.nsec = 0;	
	q_temp.positions.clear();
	for(size_t i = 0 ; i < 7 ; i++){
		q_temp.positions.push_back(ROBOT_CMD.q_des(i));
	}
	q_msg.points.clear();
	q_msg.points.push_back(q_temp);
	joint_cmd_pub.publish(q_msg);
}

void reset_cb(const std_msgs::Bool& msg){
	ROBOT_CMD.qdot_des = Eigen::VectorXd::Zero(7);
	ROBOT_CMD.ee_vel = Eigen::VectorXd::Zero(6);
	ROBOT_CMD.mode = 0;
	init_params();
}

void init_params(void){
	std::string prefix = "j2s7s300_joint_";
	q_msg.joint_names.clear();
	for(size_t i = 0 ; i < 7 ; i++){
		q_msg.joint_names.push_back(prefix + std::to_string(i+1));
	}

	ROBOT_CMD.q_des(0) = 0.0;
	ROBOT_CMD.q_des(1) = M_PI + 0.533;
	ROBOT_CMD.q_des(2) = 0.0;
	ROBOT_CMD.q_des(3) = M_PI + 0.8796;
	ROBOT_CMD.q_des(4) = M_PI;
	ROBOT_CMD.q_des(5) = M_PI;
	ROBOT_CMD.q_des(6) = -M_PI;
	
	ROBOT.initing_dt = 0.0;

}

double lp_filt(double x1, double x2, double a){
	return (x1*a + x2*(1-a));
}













