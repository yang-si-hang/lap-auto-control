/**
 * @file sdk_example.cpp
 * @brief SDK各接口使用示例
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"

using namespace rokae;
std::ostream &os = std::cout;

int main() {
  try {
    // *** 1. 连接机器人 ***
    std::string ip = "192.168.0.160";
    std::error_code ec;
    xMateErProRobot robot(ip); // 此处连接的是xMatePro机型

    // 其它机型
//    xMateRobot robot(ip); // 连接xMate6轴机型
//    StandardRobot robot(ip); // 连接工业6轴机型
//    PCB4Robot robot(ip); // 连接PCB4轴机型
//    PCB3Robot robot(ip); // 连接PCB3轴机型

    // *** 2. 查询信息 ***
    auto robotinfo = robot.robotInfo(ec);
    print(os, "控制器版本号:", robotinfo.version, "机型:", robotinfo.type);
    print(os, "xCore-SDK版本:", robot.sdkVersion());

    // *** 3. 获取机器人当前位姿，轴角度，基坐标系等信息 ***
    auto joint_pos = robot.jointPos(ec); // 轴角度 [rad]
    auto joint_vel = robot.jointVel(ec); // 轴速度 [rad/s]
    auto joint_torque = robot.jointTorque(ec); // 轴力矩 [Nm]
    auto tcp_xyzabc = robot.posture(CoordinateType::endInRef, ec);
    auto flan_cart = robot.cartPosture(CoordinateType::flangeInBase, ec);
    auto base_frame = robot.baseFrame(ec); // 基坐标系
    print(os, "末端相对外部参考坐标系位姿", tcp_xyzabc);
    print(os, "法兰相对基坐标系 -", flan_cart);

    // *** 4. 计算逆解 & 正解 ***
    auto model = robot.model();
    auto ik = model.calcIk(tcp_xyzabc, ec);
    auto fk_ret = model.calcFk(ik, ec);

    // *** 5. 查询IO/寄存器 ***
    print(os, "DO1_0当前信号值为:", robot.getDO(1,0,ec));
    robot.setSimulationMode(true, ec); // 只有在打开输入仿真模式下才可以设置DI
    robot.setDI(0, 2, true, ec);
    print(os, "DI0_2当前信号值:", robot.getDI(0, 2, ec));
    robot.setSimulationMode(false, ec); // 关闭仿真模式

    // 读取单个寄存器，类型为float
    // 假设"register0"是个寄存器数组, 长度是10
    float val_f;
    std::vector<float> val_af;
    // 读第1个，即状态监控里的register0[1], 读取结果赋值给val_f
    robot.readRegister("register0", 0, val_f, ec);
    // 读第10个，即状态监控里的register0[10], 读取结果赋值给val_f
    robot.readRegister("register0", 9, val_f, ec);
    // 读整个数组，赋值给val_af, val_af的长度也变为10。此时index参数是多少都无所谓
    robot.readRegister("register0", 9, val_af, ec);

    // 读取int类型寄存器数组
    std::vector<int> val_ai;
    robot.readRegister("register1", 1, val_ai, ec);
    // 写入bool/bit类型寄存器
    robot.writeRegister("register2", 0, true, ec);

    // *** 6. 断开连接再重连 ***
    robot.disconnectFromRobot(ec);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    robot.connectToRobot(ec);

    // *** 7. 打开，关闭拖动 ***
    robot.setOperateMode(rokae::OperateMode::manual, ec);
    robot.setPowerState(false, ec); // 打开拖动之前，需要机械臂处于手动模式下电状态
    // 笛卡尔空间，自由拖动
    robot.enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
    print(os, "打开拖动", ec, "按回车继续");
    std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
    while(getchar() != '\n');
    robot.disableDrag(ec);
    std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式

    // *** 8. Jog机器人 ***
    robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot.setOperateMode(rokae::OperateMode::manual, ec); // 手动模式下jog
    print(os, "准备Jog机器人, 需手动模式上电, 请在10秒内按住使能开关");
    std::this_thread::sleep_for(std::chrono::seconds(10));

    print(os, "-- 开始Jog机器人-- \n世界坐标系下, 沿Z+方向运动50mm, 速率50%，等待机器人停止运动后按回车继续");
    robot.startJog(JogOpt::world, 0.5, 50, 2, true, ec);
    while(getchar() != '\n');
    print(os, "轴空间，6轴负向连续转动，速率5%，按回车停止Jog");
    robot.startJog(JogOpt::jointSpace, 0.05, 5000, 5, false, ec);
    while(getchar() != '\n'); // 按回车停止
    robot.stop(ec); // jog结束必须调用stop()停止

    // *** 9. 打开和关闭碰撞检测 ***
    // 设置各轴灵敏度，范围0.01 ~ 2.0，相当于RobotAssist上设置的1% ~ 200%
    // 触发行为：安全停止；回退距离0.01m
    robot.enableCollisionDetection({1.0, 1.0, 0.01, 2.0, 1.0, 1.0, 1.0}, StopLevel::stop1, 0.01, ec);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // 关闭碰撞检测
    robot.disableCollisionDetection(ec);


    // *** 10. 结束控制，断开连接 ***
    robot.setPowerState(false, ec); // 机器人下电
    robot.disconnectFromRobot(ec);

  } catch (const rokae::Exception &e) {
    std::cout << e.what();
  }

  return 0;
}