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

ros::Publisher joint_cmd_pub, finger_cmd_pub, finger_tip_1_cmd_pub, finger_tip_2_cmd_pub, finger_tip_3_cmd_pub ;
trajectory_msgs::JointTrajectory q_msg, finger_msg;

struct RobotState{
	Eigen::VectorXd q;
	Eigen::Vector3d finger_1;
	Eigen::Vector3d finger_2;
	double ranger;
	double ranger_activation_t;
	int r_t;

	RobotState() : q(Eigen::VectorXd::Zero(7)), finger_1(Eigen::Vector3d::Zero()), finger_2(Eigen::Vector3d::Zero()), ranger(10.0), ranger_activation_t(0.0), r_t(0) { }
};

struct SetpointState{
	Eigen::VectorXd q_des;
	Eigen::VectorXd qdot_des;
	bool fingers_closed;

	SetpointState() : q_des(Eigen::VectorXd::Zero(7)), qdot_des(Eigen::VectorXd::Zero(7)), fingers_closed(false) { }
};



void joints_cb(const sensor_msgs::JointState& );
void ee_vel_cb(const geometry_msgs::Twist& );
void joint_vel_cb(const std_msgs::Float32MultiArray& );
void control_update(const ros::TimerEvent&);
void ranger_cb(const sensor_msgs::Range& );
void reset_cb(const std_msgs::Bool& );
void init_params(); //sets the correct initial joint angle setpoints
double lp_filt(double, double, double);

RobotState ROBOT;
SetpointState ROBOT_CMD;
double c_dt;
bool FORCE_GRIPPER_OPEN = false;
double gripper_forced_t = 0.0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinova_ctrl_node");		
  ros::NodeHandle n_h;
  double frequency = 200;
  c_dt = (1.0/frequency);
  ros::Rate rate(frequency);
  
  init_params();
	

  ros::Subscriber pose_sub = n_h.subscribe("/j2s7s300/joint_states", 1, &joints_cb);  
  ros::Subscriber ee_vel_cmd_sub = n_h.subscribe("/j2s7s300/end_effector_vel_cmds", 1, &ee_vel_cb);
  ros::Subscriber joint_vel_cmd_sub = n_h.subscribe("/j2s7s300/joint_vel_cmds", 1, &joint_vel_cb);
  ros::Subscriber reset_sub = n_h.subscribe("/j2s7s300/reset", 1, &reset_cb);  
  
  joint_cmd_pub = n_h.advertise<trajectory_msgs::JointTrajectory>("/j2s7s300/effort_joint_trajectory_controller/command", 1);
  finger_cmd_pub = n_h.advertise<trajectory_msgs::JointTrajectory>("/j2s7s300/effort_finger_trajectory_controller/command", 1);  
  finger_tip_1_cmd_pub = n_h.advertise<std_msgs::Float64>("/j2s7s300/finger_tip_1_position_controller/command", 1);  
  finger_tip_2_cmd_pub = n_h.advertise<std_msgs::Float64>("/j2s7s300/finger_tip_2_position_controller/command", 1);      
  finger_tip_3_cmd_pub = n_h.advertise<std_msgs::Float64>("/j2s7s300/finger_tip_3_position_controller/command", 1);  

  ros::Subscriber ranger_sub = n_h.subscribe("/zranger", 1, &ranger_cb);
  
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
	for(size_t i = 7 ; i < 10 ; i++){
		ROBOT.finger_1(i-7) = msg.position[i];
	}
	for(size_t i = 10 ; i < 13 ; i++){
		ROBOT.finger_2(i-10) = msg.position[i];
	} 
}

void ee_vel_cb(const geometry_msgs::Twist& msg){
	//TODO: From ee twist, compute joint velocities, update ROBOT_CMD.qdot_des
}

void joint_vel_cb(const std_msgs::Float32MultiArray& msg){
	for(size_t i = 0 ; i < 7 ; i++){
		ROBOT_CMD.qdot_des(i) = msg.data[i];
	}
}

void ranger_cb(const sensor_msgs::Range& msg){

	if(FORCE_GRIPPER_OPEN){
		double dt = ros::Time::now().toSec() - gripper_forced_t ;
		if(dt > 5.0){
			FORCE_GRIPPER_OPEN = false;
		}else{
			ROBOT_CMD.fingers_closed = false;
			return;
		}	
	}

	ROBOT.ranger = msg.range;	
	if(ROBOT.ranger < 0.045){
		ROBOT.r_t++;
		if(!ROBOT_CMD.fingers_closed){
			if(ROBOT.r_t > 55){
				ROBOT_CMD.fingers_closed = true;
				ROBOT.ranger_activation_t = ros::Time::now().toSec();
			}
		}

	}else{
		if(ROBOT.ranger > 0.084){
			ROBOT.r_t = 0;
			if((ros::Time::now().toSec() - ROBOT.ranger_activation_t) > 5.0){ 
				ROBOT_CMD.fingers_closed = false;
			}
		}
	}
//	ROS_WARN("GOT RANGER MSG: %.2f",msg.range);
}

void control_update(const ros::TimerEvent& e){
	
	for(size_t i = 0 ; i < 7 ; i++){	
		ROBOT_CMD.q_des(i) += ROBOT_CMD.qdot_des(i) * c_dt ;
	}
	double fingers = 0.0;
	if(ROBOT_CMD.fingers_closed){
		fingers = 1.0;
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

	trajectory_msgs::JointTrajectoryPoint f_temp;
	f_temp.time_from_start.sec = 1;
	f_temp.time_from_start.nsec = 0;	
	f_temp.positions.clear();
	for(size_t i = 0 ; i < 3 ; i++){
		f_temp.positions.push_back(fingers);
	}
	finger_msg.points.clear();
	finger_msg.points.push_back(f_temp);
	finger_cmd_pub.publish(finger_msg);

	std_msgs::Float64 tips;
	tips.data = 0.4*fingers;
	finger_tip_1_cmd_pub.publish(tips);
	finger_tip_2_cmd_pub.publish(tips);
	finger_tip_3_cmd_pub.publish(tips);		
}

void reset_cb(const std_msgs::Bool& msg){
	ROBOT.ranger = 10.0;
	init_params();
	FORCE_GRIPPER_OPEN = true;
	gripper_forced_t = ros::Time::now().toSec();
}

void init_params(void){
	std::string prefix = "j2s7s300_joint_";
	std::string finger_prefix = "j2s7s300_joint_finger_";
	q_msg.joint_names.clear();
	finger_msg.joint_names.clear();
	for(size_t i = 0 ; i < 7 ; i++){
		q_msg.joint_names.push_back(prefix + std::to_string(i+1));
	}
	for(size_t i = 0 ; i < 3 ; i++){
		finger_msg.joint_names.push_back(finger_prefix + std::to_string(i+1));
	}

	ROBOT_CMD.q_des(0) = -0.133;
	ROBOT_CMD.q_des(1) = 3.14;
	ROBOT_CMD.q_des(2) = 0.0;
	ROBOT_CMD.q_des(3) = 4.0;
	ROBOT_CMD.q_des(4) = 0.147;
	ROBOT_CMD.q_des(5) = 2.0;
	ROBOT_CMD.q_des(6) = -1.5708;
}


double lp_filt(double x1, double x2, double a){
	return (x1*a + x2*(1-a));
}

















