#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "libraries/utils.h"

double deg2rad = M_PI/180.0 ;

struct controller_vars{

  Eigen::VectorXd u;
  Eigen::Vector4d r;
  Eigen::MatrixXd Jr;
  Eigen::Matrix4d T;
  Eigen::MatrixXd J;
  
  controller_vars() : u(Eigen::VectorXd::Zero(6)),
  					  r(Eigen::Vector4d::Zero()),
  					  Jr(Eigen::MatrixXd::Zero(4,6)),
  					  T(Eigen::Matrix4d::Identity()),
  					  J(Eigen::MatrixXd::Zero(6,6)) { }
};
controller_vars CTRL;

struct params_type{

  double Y_I;
  //double half_width_wall;
  double Z_I;
  double lB;
  double l1;
  double l2;
  Eigen::MatrixXd K; //6dofs thus 6 gains
  //double height_uav; 
  //double radius_uav;
  //double radius_arm;
  double dt; //might be unused eventually
  double alpha;
  Eigen::Vector3d init_pos;
  double yaw_init;  
  Eigen::Vector2d q_init;
  double sim_time;
  Eigen::VectorXd v_max; //6 limits
  double q1_lim;
  double q2_lim;
  double wall_inclination;
  
  params_type() : Y_I(2.0), Z_I(1.0), lB(0.13), l1(0.27), l2(0.87),
  				  dt(0.01), alpha(0.5), init_pos(Eigen::Vector3d(0.0,1.1,1.5)),
  				  yaw_init(M_PI/4.0), q_init(Eigen::Vector2d(0.0,M_PI/2.0)),
  				  sim_time(50.0),
  				  q1_lim(105.0*deg2rad), q2_lim(105.0*deg2rad), wall_inclination(30.0*deg2rad),
  				  K(Eigen::MatrixXd::Identity(6,6)), v_max(Eigen::VectorXd::Ones(6)) { }
};
params_type params;

double omega_func(double );
Eigen::Matrix4d fkbas(Eigen::VectorXd );
void fk(Eigen::VectorXd );
void controller(Eigen::VectorXd );
double distfun(Eigen::VectorXd , Eigen::VectorXd* );
double alignfun(Eigen::VectorXd , Eigen::VectorXd* );
double sign(double );
double ffunc(Eigen::VectorXd , Eigen::VectorXd* );
Eigen::VectorXd generate_solutions(Eigen::VectorXd );

Eigen::Matrix4d Tg; //target wall homogeneous matrix

int main(int argc, char** argv){

	params.K = 0.4*Eigen::MatrixXd::Identity(6,6);
	params.K(3,3) = 0.8;
	params.v_max << 0.1, 0.1, 0.1, M_PI/18.0, M_PI/18.0, M_PI/18.0 ; 
	Tg << 1.0,0.0,0.0,0.0,0.0,0.0,-1.0, params.Y_I,0.0,1.0,0.0,params.Z_I,0.0,0.0,0.0,1.0 ;
	Tg *= Utils::rotx(params.wall_inclination);

	Eigen::VectorXd state(8); //don't want state to be a global variable
	state.block(0,0,3,1) = params.init_pos;
	state.block(3,0,3,1) << 0.0,0.0,params.yaw_init;
	state.block(6,0,2,1) = params.q_init; //set initial state
	std::cout << state.transpose() << std::endl;	//print for test

	Eigen::VectorXd u_f	= generate_solutions(state);
	std::cout << u_f << std::endl;
	return 0;
}

double omega_func(double s){
	return ((1.6/M_PI)*atan(2.0*sqrt(s)));
}

Eigen::Matrix4d fkbas(Eigen::VectorXd x){
  Eigen::Matrix4d composite_rot = Utils::rotz(x(5)) * Utils::roty(x(4)) * Utils::rotx(x(3)) ;
  Eigen::Matrix3d Q_0 = composite_rot.block(0,0,3,3) ;
  Eigen::Matrix3d Q_1 = Q_0 * (Utils::rotx(x(6)).block(0,0,3,3)) ;
  Eigen::Matrix3d Q_2 = Q_1 * (Utils::rotx(x(7)).block(0,0,3,3)) ;
  
  Eigen::Matrix3d Q_FK;
  Q_FK.col(0) = Q_2.col(1);
  Q_FK.col(1) = Q_2.col(0);
  Q_FK.col(2) = -Q_2.col(2);

  Eigen::Vector3d p2 = x.block(0,0,3,1) - params.lB * Q_0.col(2);
  Eigen::Vector3d p3 = p2 - params.l1 * Q_1.col(2) ;
  Eigen::Vector3d p4 = p3 - params.l2 * Q_2.col(2) ;

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block(0,0,3,3) = Q_FK;
  T.block(0,3,3,1) = p4;
  return T;
}

void fk(Eigen::VectorXd x){

  Eigen::Matrix4d current_T = fkbas(x);
  Eigen::MatrixXd J_v(3,6), J_w(3,6);
  J_v.block(0,0,3,3) = Eigen::Matrix3d::Identity();
  J_w.block(0,0,3,3) = Eigen::Matrix3d::Zero();  
  
  Eigen::VectorXd x_p(8), x_n(8);

  double epsilon = 0.01;
  for(int i = 6 ; i < 9 ; i++){
  	x_p = x;
  	x_n = x;
  	x_p(i-1) = x_p(i-1) + epsilon;
  	x_n(i-1) = x_n(i-1) - epsilon;
  	Eigen::Matrix4d deltaT = (0.5/epsilon)*(fkbas(x_p)-fkbas(x_n));
	J_v.col(i-3) = deltaT.block(0,3,3,1) ;
	Eigen::Matrix3d S_w = deltaT.block(0,0,3,3) * ((current_T.block(0,0,3,3)).transpose()) ;
	Eigen::Vector3d w = Eigen::Vector3d(S_w(2,1), S_w(0,2), S_w(1,0));
	J_w.col(i-3) = w ;
  }
  
  Eigen::MatrixXd current_J(6,6);
  current_J.block(0,0,3,6) = J_v;
  current_J.block(3,0,3,6) = J_w;
  CTRL.T = current_T;
  CTRL.J = current_J; //saved them globally, no need to call fk again in this run

}

void controller(Eigen::VectorXd x){
	
	fk(x); //Updated CTRL.T, CTRL.J
	Eigen::Matrix3d QTg_inv = Tg.block(0,0,3,3).transpose();
	Eigen::Vector3d ptg = Tg.block(0,3,3,1);
	
	Eigen::Vector3d vx = QTg_inv * CTRL.T.block(0,0,3,1);
	Eigen::Vector3d vy = QTg_inv * CTRL.T.block(0,1,3,1);
	Eigen::Vector3d vz = QTg_inv * CTRL.T.block(0,2,3,1);
	Eigen::Vector3d p = QTg_inv * (CTRL.T.block(0,3,3,1)-ptg);
	Eigen::Vector3d vxd = Eigen::Vector3d(1.0,0.0,0.0);
	Eigen::Vector3d vyd = Eigen::Vector3d(0.0,1.0,0.0);
	Eigen::Vector3d vzd = Eigen::Vector3d(0.0,0.0,-1.0);
	Eigen::Vector3d pd = Eigen::Vector3d::Zero();
	
	Eigen::Vector4d r_;
	r_.block(0,0,3,1) = p - pd ;
	r_(3) = 1.0 - vz.transpose() * vzd ;
	
	Eigen::MatrixXd J_v(3,6), J_w(3,6);
	J_v = QTg_inv * CTRL.J.block(0,0,3,6);
	J_w = QTg_inv * CTRL.J.block(3,0,3,6);

	Eigen::MatrixXd Jr_(4,6);
	Jr_.block(0,0,3,6) = J_v;
	Eigen::Vector3d temp = Utils::cross(vzd, vz) ;
	Jr_.block(3,0,1,6) = (temp.transpose()) * J_w ;
	
	Eigen::VectorXd u_(6);
	Eigen::MatrixXd pseudo = Utils::pinv(Jr_, 0.01) ;
	u_ = -params.K * pseudo * r_ ; 
	
/*	*u = u_;
	*r = r_;
	*Jr = Jr_;
*/
	CTRL.u = u_;
	std::cout << u_.transpose() << std::endl;
	CTRL.r = r_;
//	std::cout << r_.transpose() << std::endl;
	CTRL.Jr = Jr_;
//	std::cout << Jr_ << std::endl;
	return; 	
}

double ffunc(Eigen::VectorXd x, Eigen::VectorXd* Jf){
	Eigen::VectorXd JD(6), JA(6);
	double D = distfun(x, &JD);
	double A = alignfun(x, &JA);

	double OmegaP = (omega_func(A+0.01)-omega_func(A))/0.01 ;
	//*F = D - omega_func(A);
	*Jf = JD - OmegaP * JA ;
	return (D - omega_func(A));

}

double distfun(Eigen::VectorXd x, Eigen::VectorXd* Jd){
	Eigen::Vector3d zwall = -Tg.block(0,2,3,1);
	Eigen::Vector3d pwall = Eigen::Vector3d(0.0, params.Y_I, params.Z_I);
	double D = zwall.dot(pwall - CTRL.T.block(0,3,3,1));
	Eigen::VectorXd Jd_ = -(zwall.transpose())*(CTRL.J.block(0,0,3,6));
	*Jd = Jd_;
//	std::cout << "D: " << D << std::endl;
//	std::cout << "JD: " << Jd_ << std::endl;
	return D;
}

double alignfun(Eigen::VectorXd x, Eigen::VectorXd* Ja){
	double R6 = sqrt(abs(CTRL.r(3)));
	double epsilon = 0.001;
	Eigen::VectorXd JA1 = (0.5/(epsilon+R6)) * CTRL.Jr.block(3,0,1,6).transpose();
	
	double h = 1.5;
	double A2 = 2*((1/h)*pow(abs(CTRL.r(0)),h)+(1/h)*pow(abs(CTRL.r(1)),h));
	Eigen::VectorXd JA2 = 2*(pow(abs(CTRL.r(0)),h-1)*sign(CTRL.r(0))*CTRL.Jr.block(0,0,1,6).transpose() + pow(abs(CTRL.r(1)),h-1)*sign(CTRL.r(1))*CTRL.Jr.block(1,0,1,6).transpose());
	
	*Ja =  JA1 + JA2;
//	std::cout << "A: " << R6+A2 << std::endl;
//	std::cout << "JA: " << JA1+JA2 << std::endl;
	
	return (R6 + A2);
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

Eigen::VectorXd generate_solutions(Eigen::VectorXd x){
	controller(x); //updates u,r,Jr also calls fk which updates T,J
	Eigen::VectorXd JF(6);

	double F = ffunc(x, &JF);
//	std::cout << "F: " << F << std::endl;
//	std::cout << "JF: " << JF << std::endl;


	Eigen::MatrixXd Hm = Eigen::MatrixXd::Identity(6,6);
	Eigen::VectorXd fm = -CTRL.u ;

	Eigen::MatrixXd Am(17,6);
	Am.row(0) = JF;
	Am.block(1,0,6,6) = Eigen::MatrixXd::Identity(6,6);
	Am.block(7,0,6,6) = -Eigen::MatrixXd::Identity(6,6);
	
	Eigen::MatrixXd A_helper(2,6);
	A_helper.block(0,0,2,4) = Eigen::MatrixXd::Zero(2,4);
	A_helper.block(0,4,2,2) = Eigen::Matrix2d::Identity();
	Am.block(13,0,2,6) = A_helper;
	A_helper.block(0,4,2,2) = -Eigen::Matrix2d::Identity();
	Am.block(15,0,2,6) = A_helper; //these operations could also be done at init just once
	
//	std::cout << Am << std::endl;
	
	Eigen::VectorXd bm(17);
	bm(0) = -params.alpha * F;
	bm.block(1,0,6,1) = -params.v_max ;
	bm.block(7,0,6,1) = -params.v_max ;
	bm(13) = -params.alpha * (params.q1_lim+x(6));
	bm(14) = -params.alpha * (params.q2_lim+x(7));
	bm(15) = -params.alpha * (params.q1_lim-x(6));
	bm(16) = -params.alpha * (params.q2_lim-x(7));

//	std::cout << bm.transpose() << std::endl;
	
	Eigen::VectorXd u_f = Utils::solveQP(Hm, fm, Am, bm);
	return u_f;

}


















