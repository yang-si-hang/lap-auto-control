#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "Eigen/Geometry"
#include "../print_helper.hpp"
#include "rokae/utility.h"
#include <atomic>
#include <ctime>

using namespace rokae;
using namespace RtSupportedFields;



Eigen::MatrixXd freeMotionFunc(Eigen::MatrixXd A);
void swapAndPrintRule(Eigen::MatrixXd matrix, int col1, int col2);

//系数
double alpha = 15.0;
double belta = 15.0;


int main() {
	//远心点坐标（mm），相对于基坐标系
	Eigen::MatrixXd RCM_POINT(3, 1);
	RCM_POINT <<0.63,-0.05,0.45;
	std::string ip = "192.168.0.160";
	std::error_code ec;
	xMateErProRobot robot(ip, "192.168.0.100");
	robot.setRtNetworkTolerance(50, ec);
	robot.setOperateMode(OperateMode::automatic, ec);
	robot.setMotionControlMode(MotionControlMode::RtCommand, ec);
	robot.setPowerState(true, ec);
	// Toolset defaultToolset;
	// robot.setToolset(defaultToolset, ec);
	// robot.setDefaultSpeed(1000, ec);
	// robot.setDefaultZone(1, ec);
	auto rtCon = robot.getRtMotionController().lock();
	auto model = robot.model();
	
	robot.stopReceiveRobotState();
	robot.startReceiveRobotState(std::chrono::milliseconds(2), { jointPos_m, jointVel_m,jointAcc_c, tcpPose_m , tau_m });
	rtCon->setFilterLimit(true, 10.0);
	rtCon->setFilterFrequency(10.0, 10.0, 10.0, ec);
	
	std::array<double, 7> q_m{}, dq_m{}, ddq_c{}, tor_m{};
	std::array<double, 16> pos_m{};
	robot.getStateData(jointPos_m, q_m);
	robot.getStateData(tcpPose_m, pos_m);
	// Eigen::MatrixXd dq_old(7,1);

	print(std::cout, "********************1*****************");

	Eigen::MatrixXd A_old(2, 7);
	Eigen::MatrixXd Z_old(5, 7);
	A_old.setZero();
	Z_old.setZero();
	double time = 0;
	int flag=0;
	std::array<double, 42> jacobian_array = model.jacobian(q_m);
	Eigen::Map<const Eigen::Matrix<double, 7, 6>> jacobian_q(jacobian_array.data());

	Eigen::MatrixXd inertiaMat(7, 7), definiteMat(5, 5);
	inertiaMat.setZero();
	inertiaMat.topLeftCorner(6, 6) << 0.5*Eigen::MatrixXd::Identity(6, 6);
	inertiaMat.bottomRightCorner(1, 1) << 0.1*Eigen::MatrixXd::Identity(1, 1);
	definiteMat.setZero();
	static bool init = true;
	Eigen::MatrixXd freeMotion_v(5,1);	//自由运动的速度
	// 清除缓存数据
    while(robot.updateRobotState(std::chrono::steady_clock::duration::zero()));

	print(std::cout, "********************3*****************");

	std::array<double, 7> jntPos{};
    robot.getStateData(RtSupportedFields::jointPos_m, jntPos);
    std::array<double,7> q_drag_xm7p = {0, M_PI/6, 0, M_PI/3, 0, M_PI/2, 0};
	print(std::cout, "********************2*****************");
    // 从当前位置MoveJ运动到拖拽位姿
    rtCon->MoveJ(0.5, jntPos, q_drag_xm7p);
	rtCon->startMove(RtControllerMode::torque);

	// robot.getStateData(jointVel_m, dq_old);
	// robot.getStateData(tau_m, tor_m);
	// tor_offset<<tor_m[0],tor_m[1],tor_m[2],tor_m[3],tor_m[4],tor_m[5],tor_m[6];
	// dq_old<<dq_m[0],dq_m[1],dq_m[2],dq_m[3],dq_m[4],dq_m[5],dq_m[6];
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

	std::function<JointPosition(void)> callback = [&, rtCon] {
	
		if(flag==0) print(std::cout, "********************进入了*****************");
		time += 0.001;
		if(flag==999) flag=0;
		std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
      	std::chrono::duration<double> duration = now - start;
      	double time_diff = duration.count();
      	start = now;
		if (init) {
			freeMotion_v <<  0,0,0,0,0 ;
			init = false;
			if(flag==0) print(std::cout, "********************4*****************");
		};

		definiteMat << (5.0 + 5 * exp(-16 * 0))*Eigen::MatrixXd::Identity(5, 5);
		robot.updateRobotState(std::chrono::milliseconds(1));
		robot.getStateData(jointPos_m, q_m);
		robot.getStateData(jointVel_m, dq_m);
		robot.getStateData(tau_m, tor_m);
		// std::cout<< tor_m[0] <<"***"<< tor_m[1]<< std::endl;
		robot.getStateData(tcpPose_m, pos_m);
		if(flag==0) print(std::cout, "********************5*****************");
		
		Eigen::MatrixXd B_c(3, 2), p_r(3, 1), p_tem(3, 1), x_c(2, 1), p_skew(3, 3);
		B_c <<	pos_m[0], pos_m[1],
				pos_m[4], pos_m[5],
				pos_m[8], pos_m[9];
		p_r << pos_m[3], pos_m[7], pos_m[11];
		
		if(flag==0) std::cout<<"rokae位置："<<p_r(0,0)<<"***"<<p_r(1,0)<<"***"<<p_r(2,0)<<std::endl;
		p_tem = p_r - RCM_POINT ;
		p_skew << 0, -p_tem(2,0), p_tem(1,0), p_tem(2,0), 0, -p_tem(0,0), -p_tem(2,0), p_tem(0,0), 0;
		x_c = B_c.transpose()*p_tem;
		if(flag==0) print(std::cout, "********************6*****************");
		Eigen::MatrixXd jacobian_c(2, 6);
		jacobian_c.topLeftCorner(2, 3) << B_c.transpose()*Eigen::MatrixXd::Identity(3, 3);
		jacobian_c.bottomRightCorner(2, 3) << B_c.transpose()*p_skew;
		if(flag==0) print(std::cout, "********************7*****************");
		std::array<double, 42> jacobian_array = model.jacobian(q_m);
		Eigen::Map<Eigen::Matrix<double, 7, 6>> jacobian_q(jacobian_array.data());
		if(flag==0) print(std::cout, "********************7.1*****************");
		Eigen::MatrixXd A_new(2, 7), Z_new(5, 7), tempMat_A(2, 2), tempMat_Z(5, 5);
		Eigen::MatrixXd A_weight_pinverse(7, 2), Z_weight_pinverse(7, 5);
		A_new = jacobian_c * jacobian_q.transpose();
		if(flag==0) print(std::cout, "******************7.2*****************");
		Z_new = freeMotionFunc(A_new);
		if(flag==0) print(std::cout, "******************7.3*****************");
		tempMat_A = A_new * inertiaMat.inverse()*A_new.transpose();
		A_weight_pinverse = inertiaMat.inverse()*A_new.transpose()*tempMat_A.inverse();
		tempMat_Z = Z_new * inertiaMat*Z_new.transpose();
		Z_weight_pinverse = inertiaMat * Z_new.transpose()*tempMat_Z.inverse();
		if(flag==0) print(std::cout, "********************8*****************");
		
		robot.getStateData(jointAcc_c, ddq_c);

		// std::array<double, 7> acc_m{};
		// for(int j = 0;j < 7;j++)
		// {
		// 	acc_m[j]=double(dq_m[j]-dq_old[j])/time_diff;
		// }
		// dq_old=dq_m;
		// std::array<double, 7> gravity_array = model.getTorque(q_m, dq_m, ddq_c, TorqueType::gravity);
    	// std::array<double, 7> friction_array = model.getTorque(q_m, dq_m, ddq_c, TorqueType::friction);
		// std::array<double, 7> inertia_array = model.getTorque(q_m, dq_m, acc_m, TorqueType::inertia);
		// std::array<double, 7> coriolis_array = model.getTorque(q_m, dq_m, acc_m, TorqueType::coriolis);
		
		// Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
    	// Eigen::Map<const Eigen::Matrix<double, 7, 1>> friction(friction_array.data());
		// Eigen::Map<const Eigen::Matrix<double, 7, 1>> inertia(gravity_array.data());
		// Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(gravity_array.data());
		std::array<double, 7> tor_full_array = model.getTorque(q_m, dq_m, ddq_c, TorqueType::full);
		Eigen::Map<const Eigen::Matrix<double, 7, 1>> tor_full(tor_full_array.data());
		Eigen::Map<Eigen::Matrix<double, 7, 1>> tor_medium(tor_m.data());
		if(flag==0) std::cout<<"力矩："<<tor_medium(0,0)<<"***"<<tor_medium(1,0)<<"***"<<tor_medium(2,0)<<"***"<<tor_medium(3,0)<<"***"<<tor_medium(4,0)<<"***"<<tor_medium(5,0)<<"***"<<tor_medium(6,0)<<std::endl;
		if(flag==0) std::cout<<"力矩："<<tor_full(0,0)<<"***"<<tor_full(1,0)<<"***"<<tor_full(2,0)<<"***"<<tor_full(3,0)<<"***"<<tor_full(4,0)<<"***"<<tor_full(5,0)<<"***"<<tor_full(6,0)<<std::endl;
		// if(flag==0) std::cout<<"力矩："<<gravity(0,0)<<"***"<<gravity(1,0)<<"***"<<gravity(2,0)<<"***"<<gravity(3,0)<<"***"<<gravity(4,0)<<"***"<<gravity(5,0)<<"***"<<gravity(6,0)<<std::endl;
		// if(flag==0) std::cout<<"力矩："<<friction(0,0)<<"***"<<friction(1,0)<<"***"<<friction(2,0)<<"***"<<friction(3,0)<<"***"<<friction(4,0)<<"***"<<friction(5,0)<<"***"<<friction(6,0)<<std::endl;
		// if(flag==0) std::cout<<"力矩："<<inertia(0,0)<<"***"<<inertia(1,0)<<"***"<<inertia(2,0)<<"***"<<inertia(3,0)<<"***"<<inertia(4,0)<<"***"<<inertia(5,0)<<"***"<<inertia(6,0)<<std::endl;
		// if(flag==0) std::cout<<"力矩："<<coriolis(0,0)<<"***"<<coriolis(1,0)<<"***"<<coriolis(2,0)<<"***"<<coriolis(3,0)<<"***"<<coriolis(4,0)<<"***"<<coriolis(5,0)<<"***"<<coriolis(6,0)<<std::endl;

		Eigen::MatrixXd tor(7,1);
		for(int j = 0;j < 7;j++)
		{
			// tor(j,0)=(tor_medium(j,0)-(gravity(j,0))-friction(j,0)); //-double(inertia(j,0))-double(coriolis(j,0))
			tor(j,0)=tor_medium(j,0)-tor_full(j,0);
			// if(j==1) tor(j,0)= tor(j,0)-8.0;
		}
		if(flag==0) std::cout<<"力矩："<<tor(0,0)<<"***"<<tor(1,0)<<"***"<<tor(2,0)<<"***"<<tor(3,0)<<"***"<<tor(4,0)<<"***"<<tor(5,0)<<"***"<<tor(6,0)<<std::endl;
		Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_now(dq_m.data());
		Eigen::MatrixXd q_no_sum(7, 7);
		q_no_sum = inertiaMat.inverse()*(Z_weight_pinverse*Z_new*tor - belta * belta*inertiaMat*A_weight_pinverse*x_c);
		if(flag==0) print(std::cout, "********************8.1*****************");
		Eigen::MatrixXd q_v_medium_1(7, 7), q_v_medium_2(7, 7), q_v_medium_3(7, 7), q_v_medium_4(7, 7), q_v_sum_m(7, 7) ,q_v_sum(7, 7);
		q_v_medium_1 = Z_weight_pinverse * definiteMat * Z_new * inertiaMat;
		q_v_medium_2 = 2 * alpha * inertiaMat * A_weight_pinverse * A_new;
		if(flag==0) print(std::cout, "********************8.2*****************");
		Eigen::MatrixXd A_dt(2, 7), Z_dt(5, 7);
		A_dt = (A_new - A_old) * 1.0 / time_diff;
		Z_dt = (Z_new - Z_old) * 1.0 / time_diff;
		if(flag==0) print(std::cout, "********************8.3*****************");
		q_v_medium_3 = inertiaMat * A_weight_pinverse * A_dt;
		q_v_medium_4 = Z_weight_pinverse * Z_dt;
		if(flag==0) print(std::cout, "********************8.4*****************");
		q_v_sum_m = q_v_medium_1 + q_v_medium_2 + q_v_medium_3 + q_v_medium_4;
		if(flag==0) print(std::cout, "********************8.5*****************");
		q_v_sum = inertiaMat.inverse()*(q_v_sum_m);
		if(flag==0) print(std::cout, "********************9*****************");
		JointPosition cmd(7);
		Eigen::MatrixXd delta_angle(7, 1), inverse_medium(7, 7);
		inverse_medium = (q_v_sum - Eigen::MatrixXd::Identity(7, 7) / time_diff);
		delta_angle = inverse_medium.inverse()*(time_diff*q_no_sum + dq_now);
		if(flag==0) print(std::cout, "********************10*****************");
		// std::array<double, 7> q_goal{};
		// std::array<double, 7> dq_max{},dq_max_start{},dq_max_end{};
		for(int j = 0;j < 7;j++){
			if(flag==0) std::cout<<q_m[j]<<"*****"<<delta_angle(j,0)<<"***"<<j<<"***"<<time_diff<<std::endl;
			cmd.joints[j] = double(q_m[j]) + double(delta_angle(j,0));

		};
		A_old = A_new;
		Z_old = Z_new;
		flag=flag+1;
		if(flag==0) print(std::cout, "********************11*****************");
		if (time > 20) {
			cmd.setFinished(); // 60秒后结束
		}
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
	matrix<< A(0,0),A(0,1),A(0,2),A(0,3),A(0,4),A(0,5),A(0,6),
			A(1,0),A(1,1),A(1,2),A(1,3),A(1,4),A(1,5),A(1,6);
	// 尝试交换列使得子矩阵满秩为2
	bool found = false;
	int ex_col1 = 0;
	int ex_col2 = 0;
	for (int col1 = 0; col1 < 6; col1++) {
		for (int col2 = col1 + 1; col2 < 7; col2++) {
			if (abs(matrix(0,col1)*matrix(1,col2)-matrix(0,col2)*matrix(1,col1))<1e-3)
			{
				found = true;
				// std::cout << col1 << "***" << col2 << endl;
				swapAndPrintRule(matrix, 0, col1);
				swapAndPrintRule(matrix, 1, col2);
				ex_col1 = col1;
				ex_col2 = col2;
				break;
			}
		}
		if (found) break;
	};
	// print(std::cout, "********************7.2.1*****************");
	// 定义Z'矩阵
	Eigen::MatrixXd Z_exchange(5, 7), A_m_ex(2, 2), A_r_ex(2, 5);
	A_m_ex <<	matrix(0,0), matrix(0,1),
				matrix(1,0), matrix(1,1);
	A_r_ex <<	matrix(0,2), matrix(0,3), matrix(0,4), matrix(0,5), matrix(0,6),
				matrix(1,2), matrix(1,3), matrix(1,4), matrix(1,5), matrix(1,6);
	A_m_ex = A_m_ex.inverse();
	A_m_ex.transposeInPlace();
	A_r_ex.transposeInPlace();
	// print(std::cout, "********************7.2.4*****************");
	Z_exchange.topLeftCorner(5, 2) << -1 * A_r_ex * A_m_ex;
	Z_exchange.bottomRightCorner(5, 5) << Eigen::MatrixXd::Identity(5, 5);
	// print(std::cout, "********************7.2.5*****************");
	swapAndPrintRule(Z_exchange, 0, ex_col1);
	swapAndPrintRule(Z_exchange, 1, ex_col2);

	return Z_exchange;
}

// 交换列并打印变换法则
void swapAndPrintRule(Eigen::MatrixXd matrix, int col1, int col2) {
	//std::cout << "Swap columns " << col1 << " and " << col2 << endl;
	for (int i = 0; i < 2; ++i) {
		std::swap(matrix(i,col1), matrix(i,col2));
	}
};