#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "Eigen/Geometry"
#include "../print_helper.hpp"
#include "rokae/utility.h"
#include <atomic>
#include <ctime>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace rokae;
using namespace RtSupportedFields;

Eigen::MatrixXd freeMotionFunc(Eigen::MatrixXd A);	// 根据RCM空间得到自由运动空间
void swapAndPrintRule(Eigen::MatrixXd matrix, int col1, int col2);	// 交换列
void printMatrix(Eigen::MatrixXd X,int cols,int rows);

int FlagPrinting=0; 		// 判断是否打印
int FlagTime=999; 			// 即1000次
double alpha = 15.0;		// RCM运动速度系数
double belta = 50.0;		// RCM运动位置系数
static bool init = true;	// 是否运动到初始位置
Eigen::Matrix<double, 3, 1> RCM_POINT={1.255,-0.040,0.34};	// 单位：m,远心点坐标，相对于基坐标系
std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};	// 初始位置

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_joint_admittance_controller");
	ros::NodeHandle n;
    ros::Publisher joint_admittance_pub = n.advertise<sensor_msgs::JointState>("/copp/JointState", 1000);
    ros::Rate loop_rate(1);

	std::string rokae_ip = "192.168.0.160";
	std::string local_ip = "192.168.0.100";
	std::error_code ec;
	xMateErProRobot robot(rokae_ip, "192.168.0.100");
	robot.setRtNetworkTolerance(50, ec);
	robot.setOperateMode(OperateMode::automatic, ec);
	robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
	robot.setPowerState(true, ec);
	auto rtCon = robot.getRtMotionController().lock();
	auto model = robot.model();
	robot.stopReceiveRobotState();
	robot.startReceiveRobotState(std::chrono::milliseconds(1), { jointPos_m, jointVel_m,jointAcc_c, tcpPose_m, tauFiltered_m, tauExt_inBase });
	// rtCon->setFilterLimit(true, 10.0);
	// rtCon->setFilterFrequency(10.0, 10.0, 50.0, ec);
	std::array<double, 7> q_m{}, dq_m{}, tor_m{}, ddq_c{};
	std::array<double, 16> pos_m{};
	if(FlagPrinting==0) print(std::cout, "********************1*****************");
	Eigen::MatrixXd A_old(2, 7),Z_old(5, 7);	// 等待差分使用，计算变化率
	A_old.setZero();
	Z_old.setZero();
	// 定义质量矩阵、自由空间阻尼矩阵
	Eigen::MatrixXd inertiaMat(7, 7), definiteMat(5, 5), freeMotion_v(5,1);	// 自由运动的速度,与Df参数相关
	inertiaMat.setZero(), definiteMat.setZero(), freeMotion_v.setZero();
	inertiaMat.topLeftCorner(6, 6) << 0.1*Eigen::MatrixXd::Identity(6, 6);
	inertiaMat.bottomRightCorner(1, 1) << 0.05*Eigen::MatrixXd::Identity(1, 1);
	// 清除缓存数据
    while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));
    // 从当前位置MoveJ运动到拖拽位姿
	robot.getStateData(RtSupportedFields::jointPos_m, q_m);
    rtCon->MoveJ(0.5, q_m, q_drag_xm7p);
	rtCon->startMove(RtControllerMode::jointPosition);
	// 计时开始
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	if(FlagPrinting==0) print(std::cout, "********************2*****************");


	// 回调函数 关节位置控制
	std::function<JointPosition(void)> callback = [&, rtCon] {
		if(FlagPrinting==0) print(std::cout, "********************进入了*****************");

		static double time = 0;
		time += 0.001;
		std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
      	std::chrono::duration<double> duration = now - start;
      	double time_diff = duration.count();
		if(FlagPrinting==0) std::cout<<"时间: "<<time_diff<<std::endl;
      	start = now;
		if (init) {
			init = false;
		};
		
		definiteMat << (5.0 + 0 * exp(-16 * 0))*Eigen::MatrixXd::Identity(5, 5);
		std::array<double, 6> tcp_tor{};
		robot.updateRobotState(std::chrono::milliseconds(1));
		robot.getStateData(jointPos_m, q_m);
		robot.getStateData(tcpPose_m, pos_m);
		robot.getStateData(tauFiltered_m, tor_m);
		robot.getStateData(jointVel_m, dq_m);
		// robot.getStateData(tauExt_inBase, tcp_tor);

		// if(FlagPrinting==0) print(std::cout, "********************5*****************");
		
		Eigen::MatrixXd B_c(3, 2), p_r(3, 1), p_tem(3, 1), x_c(2, 1), p_skew(3, 3);
		B_c << pos_m[0], pos_m[1], pos_m[4], pos_m[5], pos_m[8], pos_m[9];
		p_r << pos_m[3], pos_m[7], pos_m[11];
		if(FlagPrinting==0) std::cout<<"rokae位置："<<p_r(0,0)<<"***"<<p_r(1,0)<<"***"<<p_r(2,0)<<std::endl;

		p_tem = p_r - RCM_POINT ;
		p_skew << 0, -p_tem(2,0), p_tem(1,0), p_tem(2,0), 0, -p_tem(0,0), -p_tem(1,0), p_tem(0,0), 0;
		x_c = B_c.transpose()*p_tem;
		// if(FlagPrinting==0) print(std::cout, "********************6*****************");
		Eigen::MatrixXd jacobian_c(2, 6);
		jacobian_c.topLeftCorner(2, 3) << B_c.transpose()*Eigen::MatrixXd::Identity(3, 3);
		jacobian_c.bottomRightCorner(2, 3) << B_c.transpose()*p_skew;
		std::array<double, 42> jacobian_array = model.jacobian(q_m);
		Eigen::Map<Eigen::Matrix<double, 7, 6>> jacobian_q(jacobian_array.data());
		Eigen::MatrixXd A_new(2, 7), Z_new(5, 7), tempMat_A(2, 2), tempMat_Z(5, 5);
		Eigen::MatrixXd A_weight_pinverse(7, 2), Z_weight_pinverse(7, 5);
		A_new = jacobian_c * jacobian_q.transpose();
		Z_new = freeMotionFunc(A_new);
		tempMat_A = A_new * inertiaMat.inverse() * A_new.transpose();
		A_weight_pinverse = inertiaMat.inverse() * A_new.transpose() * tempMat_A.inverse();	// 加权右伪逆矩阵
		tempMat_Z = Z_new * inertiaMat * Z_new.transpose();
		Z_weight_pinverse = inertiaMat * Z_new.transpose() * tempMat_Z.inverse();	// 加权右伪逆矩阵
		// if(FlagPrinting==0) print(std::cout, "********************8*****************");
		
		// Eigen::Map<Eigen::Matrix<double, 6, 1>> tcp_tor_test(tcp_tor.data());
		Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_now(dq_m.data());
		Eigen::Map<Eigen::Matrix<double, 7, 1>> q_now(q_m.data());

		std::array<double, 7> torque_inner{},torqur_outer{};
		std::array<double, 3> carte_torque{},carte_force{};
		robot.getEndTorque(FrameType::world,torque_inner,torqur_outer,carte_torque,carte_force,ec);
		Eigen::Map<Eigen::Matrix<double, 7, 1>> tor(torqur_outer.data());
		tor=-1.0*tor;

		sensor_msgs::JointState copp_joint_state;
		copp_joint_state.header.stamp = ros::Time().now();
		std::vector<double> jntPos_vec(q_m.begin(), q_m.end());
		std::vector<double> jntVel_vec(dq_m.begin(), dq_m.end());
		std::vector<double> jntTau_vec(torqur_outer.begin(), torqur_outer.end());
		copp_joint_state.position = jntPos_vec;
		copp_joint_state.velocity = jntVel_vec;
		copp_joint_state.effort = jntTau_vec;
		joint_admittance_pub.publish(copp_joint_state);

		// tor=tqr-tqr_full+tqr_fric;
		// if(FlagPrinting==0) std::cout<<"关节传感器：";
		// if(FlagPrinting==0) printMatrix(tqr,7,1);
		if(FlagPrinting==0) std::cout<<"力矩："<<tor(0,0)<<"***"<<tor(1,0)<<"***"<<tor(2,0)<<"***"<<tor(3,0)<<"***"<<tor(4,0)<<"***"<<tor(5,0)<<"***"<<tor(6,0)<<std::endl;
		// if(FlagPrinting==0) print(std::cout, "********************8.0*****************");
		Eigen::MatrixXd q_v_medium_1(7, 7), q_v_medium_2(7, 7), q_v_medium_3(7, 7), q_v_medium_4(7, 7), q_v_sum(7, 7);
		q_v_medium_1 = inertiaMat.inverse() * Z_weight_pinverse * definiteMat * Z_new * inertiaMat;
		q_v_medium_2 = 2 * alpha * A_weight_pinverse * A_new;
		Eigen::MatrixXd A_dt(2, 7), Z_dt(5, 7);
		A_dt = (A_new - A_old) * 1.0 / time_diff;
		Z_dt = (Z_new * inertiaMat - Z_old * inertiaMat) * 1.0 / time_diff;
		q_v_medium_3 = A_weight_pinverse * A_dt;
		q_v_medium_4 = inertiaMat.inverse() * Z_weight_pinverse * Z_dt;
		q_v_sum = q_v_medium_1 + q_v_medium_2 + q_v_medium_3 + q_v_medium_4;
		Eigen::MatrixXd q_no_sum(7, 1),q_no_sum_1(7, 1),q_no_sum_2(7, 1);
		q_no_sum_1 = inertiaMat.inverse() *(Z_weight_pinverse * Z_new*tor);
		q_no_sum_2 = belta * belta * A_weight_pinverse * x_c;
		q_no_sum = q_no_sum_1 - q_no_sum_2;
		Eigen::MatrixXd q_a(7,1),delta_angle(7, 1);
		q_a = q_no_sum - q_v_sum * dq_now;
		delta_angle =(dq_now + q_a * time_diff)* time_diff;
		if(FlagPrinting==0) printMatrix(delta_angle,7,1);
		JointPosition cmd(7);
		for(int j = 0;j < 7;j++){
			cmd.joints[j] = q_now(j,0) + delta_angle(j,0);
		};

		A_old = A_new;
		Z_old = Z_new;
		FlagPrinting = FlagPrinting + 1;
		if(FlagPrinting==FlagTime) FlagPrinting = 0;
		if (time > 20) {
			cmd.setFinished(); // 60秒后结束
		};

        // ros::spinOnce();
        // loop_rate.sleep();

		return cmd;
	};

	
	rtCon->setControlLoop(callback,0,true);
    // 阻塞loop
    rtCon->startLoop(true);
    print(std::cout, "控制结束");

	return 0;
}

Eigen::MatrixXd freeMotionFunc(Eigen::MatrixXd A)
{
	Eigen::MatrixXd matrix(2,7);
	matrix = A; 
	// 尝试交换列使得子矩阵满秩为2
	bool found = false;
	int ex_col1 = 0, ex_col2 = 0;
	for (int col1 = 0; col1 < 6; col1++) {
		for (int col2 = col1 + 1; col2 < 7; col2++) {
			if (abs(matrix(0,col1)*matrix(1,col2)-matrix(0,col2)*matrix(1,col1))<1e-3)
			{
				found = true;
				swapAndPrintRule(matrix, 0, col1);
				swapAndPrintRule(matrix, 1, col2);
				ex_col1 = col1, ex_col2 = col2;
				break;
			};
		};
		if (found) break;
	};
	Eigen::MatrixXd Z_exchange(5, 7), A_m_ex(2, 2), A_r_ex(2, 5);
	A_m_ex <<	matrix(0,0), matrix(0,1),
				matrix(1,0), matrix(1,1);
	A_r_ex <<	matrix(0,2), matrix(0,3), matrix(0,4), matrix(0,5), matrix(0,6),
				matrix(1,2), matrix(1,3), matrix(1,4), matrix(1,5), matrix(1,6);
	A_m_ex = A_m_ex.inverse();
	A_m_ex.transposeInPlace(),A_r_ex.transposeInPlace();
	Z_exchange.topLeftCorner(5, 2) << -1 * A_r_ex * A_m_ex;
	Z_exchange.bottomRightCorner(5, 5) << Eigen::MatrixXd::Identity(5, 5);
	swapAndPrintRule(Z_exchange, 0, ex_col1);
	swapAndPrintRule(Z_exchange, 1, ex_col2);
	// printMatrix(Z_exchange,5,7);
	return Z_exchange;
}

// 交换列并打印变换法则
void swapAndPrintRule(Eigen::MatrixXd matrix, int col1, int col2) {
	for (int i = 0; i < 2; ++i) {
		std::swap(matrix(i,col1), matrix(i,col2));
	};
};


void printMatrix(Eigen::MatrixXd X,int cols,int rows)
{
	for(int i=0;i<cols;i++){
		for(int j=0;j<rows;j++)
		std::cout<<X(i,j)<<"***";
		if(rows!=1)std::cout<<std::endl;
	};
	if(rows==1)std::cout<<std::endl;
};

/*
		jacobian_c.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
		jacobian_c.bottomRightCorner(3, 3) = p_skew;
		for(int i=0;i<3;i++){
			for(int j=0;j<6;j++)
			std::cout<<jacobian_c(i,j)<<"**";
			std::cout<<std::endl;
		};


		// for(int j = 0;j < 7;j++)
		// {
		// 	// tor(j,0)=(tor_medium(j,0)-(gravity(j,0))-friction(j,0)); //-double(inertia(j,0))-double(coriolis(j,0))
		// 	tor(j,0)=tor_medium(j,0)-tor_full(j,0);
		// 	// if(j==1) tor(j,0)= tor(j,0)-8.0;
		// }

		printMatrix(Z_exchange,5,7);
		Eigen::MatrixXd test(2,5);
		test = A*Z_exchange.transpose();
		printMatrix(test,2,5);


		// if(FlagPrinting==0) std::cout<<"力矩："<<tcp_tor_test(0,0)<<"***"<<tcp_tor_test(1,0)<<"***"<<tcp_tor_test(2,0)<<"***"<<tcp_tor_test(3,0)<<"***"<<tcp_tor_test(4,0)<<"***"<<tcp_tor_test(5,0)<<std::endl;
		
			
		// Eigen::MatrixXd tor(7,1);
		// tor=jacobian_q*tcp_tor_test;
		
		// robot.getStateData(jointAcc_c, ddq_c);
		// std::array<double, 7> tor_full{},tor_inertia{},tor_coriolis{},tor_friction{},tor_gravity{};
		// model.getTorqueWithFriction(q_m,dq_m,ddq_c,tor_full,tor_inertia,tor_coriolis,tor_friction,tor_gravity);
		// if(FlagPrinting==0) std::cout<<"总力矩："<<tor_full[0]<<"***"<<tor_full[1]<<"***"<<tor_full[2]<<"***"<<tor_full[3]<<"***"<<tor_full[4]<<"***"<<tor_full[5]<<"***"<<tor_full[6]<<std::endl;
		// if(FlagPrinting==0) std::cout<<"惯性力："<<tor_inertia[0]<<"***"<<tor_inertia[1]<<"***"<<tor_inertia[2]<<"***"<<tor_inertia[3]<<"***"<<tor_inertia[4]<<"***"<<tor_inertia[5]<<"***"<<tor_inertia[6]<<std::endl;
		// if(FlagPrinting==0) std::cout<<"科氏力："<<tor_coriolis[0]<<"***"<<tor_coriolis[1]<<"***"<<tor_coriolis[2]<<"***"<<tor_coriolis[3]<<"***"<<tor_coriolis[4]<<"***"<<tor_coriolis[5]<<"***"<<tor_coriolis[6]<<std::endl;
		// if(FlagPrinting==0) std::cout<<"摩擦力："<<tor_friction[0]<<"***"<<tor_friction[1]<<"***"<<tor_friction[2]<<"***"<<tor_friction[3]<<"***"<<tor_friction[4]<<"***"<<tor_friction[5]<<"***"<<tor_friction[6]<<std::endl;
		// if(FlagPrinting==0) std::cout<<"重力矩："<<tor_gravity[0]<<"***"<<tor_gravity[1]<<"***"<<tor_gravity[2]<<"***"<<tor_gravity[3]<<"***"<<tor_gravity[4]<<"***"<<tor_gravity[5]<<"***"<<tor_gravity[6]<<std::endl;
		
		// Eigen::Map<Eigen::Matrix<double, 7, 1>> tqr(tor_m.data());
		// Eigen::Map<Eigen::Matrix<double, 7, 1>> tqr_fric(tor_friction.data());
		// Eigen::Map<Eigen::Matrix<double, 7, 1>> tqr_full(tor_full.data());

		Eigen::Map<Eigen::Matrix<double, 7, 1>> tor_medium(tor_m.data());
		if(FlagPrinting==0) std::cout<<"力矩："<<tor_medium(0,0)<<"***"<<tor_medium(1,0)<<"***"<<tor_medium(2,0)<<"***"<<tor_medium(3,0)<<"***"<<tor_medium(4,0)<<"***"<<tor_medium(5,0)<<"***"<<tor_medium(6,0)<<std::endl;
		// if(FlagPrinting==0) std::cout<<"力矩："<<tor_full(0,0)<<"***"<<tor_full(1,0)<<"***"<<tor_full(2,0)<<"***"<<tor_full(3,0)<<"***"<<tor_full(4,0)<<"***"<<tor_full(5,0)<<"***"<<tor_full(6,0)<<std::endl;
		

		// std::array<double, 7> tor_full_array = model.getTorque(q_m, dq_m, ddq_c, TorqueType::full);
		// Eigen::Map<const Eigen::Matrix<double, 7, 1>> tor_full(tor_full_array.data());

		std::array<double, 7> acc_m{};
		for(int j = 0;j < 7;j++)
		{
			acc_m[j]=double(dq_m[j]-dq_old[j])/time_diff;
		}
		dq_old=dq_m;
		std::array<double, 7> gravity_array = model.getTorque(q_m, dq_m, ddq_c, TorqueType::gravity);
    	std::array<double, 7> friction_array = model.getTorque(q_m, dq_m, ddq_c, TorqueType::friction);
		std::array<double, 7> inertia_array = model.getTorque(q_m, dq_m, acc_m, TorqueType::inertia);
		std::array<double, 7> coriolis_array = model.getTorque(q_m, dq_m, acc_m, TorqueType::coriolis);
		
		Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
    	Eigen::Map<const Eigen::Matrix<double, 7, 1>> friction(friction_array.data());
		Eigen::Map<const Eigen::Matrix<double, 7, 1>> inertia(gravity_array.data());
		Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(gravity_array.data());

		if(FlagPrinting==0) std::cout<<"力矩："<<gravity(0,0)<<"***"<<gravity(1,0)<<"***"<<gravity(2,0)<<"***"<<gravity(3,0)<<"***"<<gravity(4,0)<<"***"<<gravity(5,0)<<"***"<<gravity(6,0)<<std::endl;
		if(FlagPrinting==0) std::cout<<"力矩："<<friction(0,0)<<"***"<<friction(1,0)<<"***"<<friction(2,0)<<"***"<<friction(3,0)<<"***"<<friction(4,0)<<"***"<<friction(5,0)<<"***"<<friction(6,0)<<std::endl;
		if(FlagPrinting==0) std::cout<<"力矩："<<inertia(0,0)<<"***"<<inertia(1,0)<<"***"<<inertia(2,0)<<"***"<<inertia(3,0)<<"***"<<inertia(4,0)<<"***"<<inertia(5,0)<<"***"<<inertia(6,0)<<std::endl;
		if(FlagPrinting==0) std::cout<<"力矩："<<coriolis(0,0)<<"***"<<coriolis(1,0)<<"***"<<coriolis(2,0)<<"***"<<coriolis(3,0)<<"***"<<coriolis(4,0)<<"***"<<coriolis(5,0)<<"***"<<coriolis(6,0)<<std::endl;



		Eigen::MatrixXd q_no_sum(7, 1),q_no_sum_1(7, 1),q_no_sum_2(7, 1);
		q_no_sum_1 = (Z_weight_pinverse*Z_new*tor );//
		q_no_sum_2 = belta*belta*inertiaMat*A_weight_pinverse*x_c;
		inertiaMat.inverse()*
		if(FlagPrinting==0) print(std::cout, "********************8.1*****************");
		Eigen::MatrixXd q_v_medium_1(7, 7), q_v_medium_2(7, 7), q_v_medium_3(7, 7), q_v_medium_4(7, 7), q_v_sum_m(7, 7) ,q_v_sum(7, 7);
		q_v_medium_1 = Z_weight_pinverse * definiteMat * Z_new * inertiaMat;
		q_v_medium_2 = 2 * alpha * inertiaMat * A_weight_pinverse * A_new;
		if(FlagPrinting==0) print(std::cout, "********************8.2*****************");
		Eigen::MatrixXd A_dt(2, 7), Z_dt(5, 7);
		A_dt = (A_new - A_old) * 1.0 / time_diff;
		Z_dt = (Z_new - Z_old) * 1.0 / time_diff;
		if(FlagPrinting==0) print(std::cout, "********************8.3*****************");
		q_v_medium_3 = inertiaMat * A_weight_pinverse * A_dt;
		q_v_medium_4 = Z_weight_pinverse * Z_dt;
		if(FlagPrinting==0) print(std::cout, "********************8.4*****************");
		q_v_sum_m = q_v_medium_1 + q_v_medium_2 + q_v_medium_3 + q_v_medium_4;
		if(FlagPrinting==0) print(std::cout, "********************8.5*****************");
		q_v_sum = inertiaMat.inverse()*(q_v_sum_m);
		if(FlagPrinting==0) print(std::cout, "********************9*****************");
		JointPosition cmd(7);
		Eigen::MatrixXd delta_angle(7, 1), inverse_medium(7, 7);
		inverse_medium = (q_v_sum - Eigen::MatrixXd::Identity(7, 7) / time_diff);
		delta_angle = inverse_medium.inverse()*(time_diff*q_no_sum + dq_now);
		if(FlagPrinting==0) print(std::cout, "********************10*****************");

*/