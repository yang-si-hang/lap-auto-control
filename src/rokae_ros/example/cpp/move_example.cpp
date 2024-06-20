/**
 * @file move_example.cpp.
 * @brief 非实时运动指令. 根据机型和坐标系的不同, 各示例中的点位不一定可达, 仅供接口使用方法的参考
 *
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#include <iostream>
#include <thread>
#include <cmath>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "print_helper.hpp"

using namespace std;
using namespace rokae;
std::ostream &os = std::cout;

/**
 * @brief 打印运动执行信息
 */
void printInfo(const rokae::EventInfo &info) {
  using namespace rokae::EventInfoKey::MoveExecution;
  print(std::cout, "ID:", std::any_cast<std::string>(info.at(ID)), "Index:", std::any_cast<int>(info.at(WaypointIndex)),
        "Reach target: ", std::any_cast<bool>(info.at(ReachTarget)), std::any_cast<error_code>(info.at(Error)));
}

/**
 * @brief 等待运动结束 - 通过查询路径ID及路点序号是否已完成的方式
 */
void waitForFinish(BaseRobot &robot, const std::string &traj_id, int index){
  using namespace rokae::EventInfoKey::MoveExecution;
  error_code ec;
  while(true) {
    auto info = robot.queryEventInfo(Event::moveExecution, ec);
    if(std::any_cast<std::string>(info.at(ID)) == traj_id &&
       std::any_cast<int>(info.at(WaypointIndex)) == index) {
      if(std::any_cast<bool>(info.at(ReachTarget))) {
        print(std::cout, "路径", traj_id, ":", index, "已完成");
      }
      if(auto _ec = std::any_cast<error_code>(info.at(Error))) {
        print(std::cout, "路径", traj_id, ":", index, "错误:", _ec.message());
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}

/**
 * @brief 等待运动结束 - 通过查询机械臂是否处于运动中的方式
 */
void waitRobot(BaseRobot &robot, bool &running) {
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    error_code ec;
    auto st = robot.operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown){
      running = false;
    }
  }
}

/**
 * @brief 事件处理 - 模拟发生碰撞后等待5秒上电并继续运行
 */
void recoverFromCollision(BaseRobot &robot, const rokae::EventInfo &info) {
  using namespace rokae::EventInfoKey;
  bool isCollided = std::any_cast<bool>(info.at(Safety::Collided));
  print(std::cout, "Collided:", isCollided);
  if(isCollided) {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    error_code ec;
    robot.setPowerState(true, ec);
    robot.moveStart(ec);
    print(std::cout, "Recovered from collision");
  }
}

/**
 * @brief 示例 - 笛卡尔点位设置偏移 & 运动中暂停与继续; 点位适用机型xMateEr7 Pro
 */
void cartesianPointWithOffset(BaseRobot &robot) {
  error_code ec;

  std::array<double, 6> pos = { -0.571, -0.15, 0.5568, -M_PI, 0.573, -M_PI};
  std::array<double,6> offset_z = {0, 0, 0.2, 0, 0, 0};

  MoveLCommand moveL1(pos, 500, 5), moveL2(pos, 800, 0);
  // 相对工件坐标系Z+偏移0.2m
  moveL2.offset = { CartesianPosition::Offset::offs, offset_z};

  MoveJCommand moveJ1(pos, 200, 0), moveJ2(pos, 1000, 80);
  // 相对工具坐标系Z+偏移0.2m
  moveJ2.offset = {CartesianPosition::Offset::relTool, offset_z};

  // 执行这4个点位
  robot.executeCommand({moveL1, moveL2}, ec);
  robot.executeCommand({moveJ1, moveJ2}, ec);

  std::thread input([&]{
    int c{};
    print(os, "[p]暂停 [c]继续 [q]退出");
    while(c != 'q') {
      c = getchar();
      switch(c) {
        case 'p':
          robot.stop(ec);
          print(std::cerr, ec); break;
        case 'c':
          robot.moveStart(ec);
          print(std::cerr, ec); break;
        default: break;
      }
    }
  });
  input.join();
  robot.moveReset(ec);
}

/**
 * @brief 示例 - 七轴冗余运动 & 发生碰撞检测后恢复运动, 点位适用机型xMateER3 Pro
 */
void redundantMove(xMateErProRobot &robot) {
  error_code ec;
  std::string id;

  // 本段示例使用默认工具工件, 速度v500, 转弯区fine
  Toolset defaultToolset;
  robot.setToolset(defaultToolset, ec);
  robot.setDefaultSpeed(500, ec);
  robot.setDefaultZone(0, ec);

  // 可选: 设置碰撞检测事件回调函数
  robot.setEventWatcher(Event::safety, [&](const EventInfo &info){
    recoverFromCollision(robot, info);
  }, ec);


  MoveAbsJCommand moveAbsj({0, M_PI/6, 0, M_PI/3, 0, M_PI_2, 0});
  // ** 1) 变臂角运动 **
  MoveLCommand moveL1({0.562, 0, 0.432, M_PI, 0, -M_PI});
  moveL1.target.elbow = 1.45;
  robot.moveAppend({moveAbsj}, id, ec);
  robot.moveAppend({moveL1}, id, ec);
  moveL1.target.elbow = -1.51;
  robot.moveAppend({moveL1}, id, ec);
  robot.moveStart(ec);
  // 最后一次moveAppend()发送一条指令，故index = 0
  waitForFinish(robot, id, 0);

  // ** 2) 60°臂角圆弧 **
  CartesianPosition circle_p1({0.472, 0, 0.342, M_PI, 0, -M_PI}),
  circle_p2({0.602, 0, 0.342, M_PI, 0, -M_PI}),
  circle_a1({0.537, 0.065, 0.342, M_PI, 0, -M_PI}),
  circle_a2({0.537, -0.065, 0.342, M_PI, 0, -M_PI});
  // 臂角都是60°
  circle_p1.elbow = M_PI/3;
  circle_p2.elbow = M_PI/3;
  circle_a1.elbow = M_PI/3;
  circle_a2.elbow = M_PI/3;

  MoveLCommand moveL2(circle_p1);
  robot.moveAppend({moveL2}, id, ec);
  MoveCCommand moveC1(circle_p2, circle_a1), moveC2(circle_p1, circle_a2);
  std::vector<MoveCCommand> movec_cmds = {moveC1, moveC2};
  robot.moveAppend(movec_cmds, id, ec);
  robot.moveStart(ec);
  // 最后一次moveAppend()发送2条指令，故需要等待第二个点完成后返回，index为第二个点的下标
  waitForFinish(robot, id, (int)movec_cmds.size() - 1);
}

/**
 * @brief 示例 - 使用工具工件坐标系
 */
void moveInToolsetCoordinate(BaseRobot &robot) {
  error_code ec;
  std::string id;
  // 默认的工具工件 tool0, wobj0
  auto currentToolset = robot.setToolset("tool0", "wobj0", ec);
  print(os, "当前工具工件组", currentToolset);

  MoveAbsJCommand moveAbs({0, M_PI/6, -M_PI_2, 0, -M_PI/3, 0});
  robot.moveAppend({moveAbs}, id, ec);
  MoveLCommand movel1({0.563, 0, 0.432, M_PI, 0, M_PI}, 1000, 100);
  MoveLCommand movel2({0.33467, -0.095, 0.51, M_PI, 0, M_PI}, 1000, 100);
  robot.moveAppend({movel1, movel2}, id, ec);
  robot.moveStart(ec);
  bool moving = true;
  waitRobot(robot, moving);

#if 0
  // 举例：执行完movel1和movel2, 需要切换到工具工件, 再执行后面的运动指令
  // 设置工具工件方式1: 直接设定
  Toolset toolset1;
  toolset1.load.mass = 2; // 负载2kg
  toolset1.ref = {{0.1, 0.1, 0}, {0, 0, 0}}; // 外部参考坐标系，X+0.1m, Y+0.1m
  toolset1.end = {{ 0, 0, 0.01}, {0, M_PI/6, 0}}; // 末端坐标，Z+0.01m, Ry+30°
  robot.setToolset(toolset1, ec);

  // 设置工具工件方式2: 使用已创建的工具工件tool1, wobj1
  robot.setToolset("tool1", "wobj1", ec);
  MoveLCommand movel3({0.5, 0, 0.4, M_PI, 0, M_PI}, 1000, 100);
  robot.moveAppend({movel3}, id, ec);
  robot.moveStart(ec);
#endif
}

/**
 * @brief 示例 - 运动中调整运动速率
 */
void adjustSpeed(BaseRobot &robot) {
  error_code ec;
  std::string id;
  double scale = 0.5;
  robot.adjustSpeedOnline(scale, ec); // 设置起始速度比例为50%

  // 示例用: 在cmd1和cmd2两个点位之间运动
  rokae::MoveAbsJCommand cmd1({0, 0, 0, 0, 0, 0}), cmd2({1.5, 1.5,1.5,1.5,1.5,1.5});
  robot.moveAppend({cmd1, cmd2,cmd1,cmd2,cmd1,cmd2,cmd1,cmd2}, id, ec);
  robot.moveStart(ec);
  bool running = true;

  // 读取键盘输入
  std::thread readInput([&]{
    while(running) {
      auto ch = std::getchar();
      if(ch == 'a') {
        if(scale < 0.1) { print(std::cerr, "已达到1%"); continue; }
        scale -= 1e-1;
      } else if(ch == 'd'){
        if(scale > 1) { print(std::cerr, "已达到100%"); continue; }
        scale += 1e-1;
      } else { continue; }
      robot.adjustSpeedOnline(scale, ec);
      print(os, "调整为", scale);
    }
  });
  print(os, "机器人开始运动, 请按[a]减小速度 [d]增大速度, 步长为10%");

  // 等待运动结束
  waitRobot(robot, running);
  readInput.join();
}

/**
 * @brief 示例 - 设置轴配置数据(confData)处理多逆解问题, 点位适用机型xMateCR7
 */
void multiplePosture(xMateRobot &robot) {
  error_code ec;
  std::string id;

  // 本段示例使用默认工具工件
  Toolset defaultToolset;
  robot.setToolset(defaultToolset, ec);

  MoveJCommand moveJ({0.2434, -0.314, 0.591, 1.5456, 0.314, 2.173});
  // 同样的末端位姿，confData不同，轴角度也不同
  moveJ.target.confData =  {-67, 100, 88, -79, 90, -120, 0, 0};
  robot.moveAppend({moveJ}, id, ec);
  moveJ.target.confData =  {-76, 8, -133, -106, 103, 108, 0, 0};
  robot.moveAppend({moveJ}, id, ec);
  moveJ.target.confData =  {-70, 8, -88, 90, -105, -25, 0, 0};
  robot.moveAppend({moveJ}, id, ec);
  robot.moveStart(ec);
  waitForFinish(robot, id, 0);
}

int main() {
  try {
    using namespace rokae;

    // *** 1. 连接机器人 ***
    std::string ip = "192.168.0.160";
    std::error_code ec;
    rokae::xMateErProRobot robot(ip); // ****   xMate 7-axis

    // *** 2. 切换到自动模式并上电 ***
    robot.setOperateMode(OperateMode::automatic, ec);
    robot.setPowerState(true, ec);

    // *** 3. 设置默认运动速度和转弯区 ***
    robot.setMotionControlMode(MotionControlMode::NrtCommand, ec);
    robot.moveReset(ec);
    robot.setDefaultZone(50, ec); // 可选：设置默认转弯区
    robot.setDefaultSpeed(200, ec); // 可选：设置默认速度

    // 可选：设置运动指令执行完成和错误信息回调
    robot.setEventWatcher(Event::moveExecution, printInfo, ec);

    // *** 4. 运动示例程序 ***
//    adjustSpeed(robot);
//    redundantMove(robot);
//    moveInToolsetCoordinate(robot);
//    multiplePosture(robot);
//    squareMove(robot);

    robot.setPowerState(false, ec);
    robot.disconnectFromRobot(ec);
  } catch (const std::exception &e) {
    print(std::cerr, e.what());
  }
  return 0;
}