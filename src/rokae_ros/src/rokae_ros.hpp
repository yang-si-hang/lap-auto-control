#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <cmath>
#include <thread>
#include "rokae/robot.h"
#include "print_helper.hpp"
#include <std_msgs/String.h>


using namespace rokae;

enum ros_control_mode 
{
    cp,//CartesianPosition
    cv,//CartesianVelocity
    cf,//CartesianForce
    jp,//JointPosition
    jv,//JointVelocity
    jf, //JointForce
    drag_start, //用于珞石自带的手动拖动
    drag_stop,
    dragging
};

class ROKAE_ROS
{
public:
    ROKAE_ROS(const std::string &m_ip, const std::string &local_ip);
    ~ROKAE_ROS();

    // 发布所有消息的函数
    void send_state();
    // 启动机器人
    void start_rokae();
    // 控制启动
    void start_control();

    double start;

private:
    double time_change_mode;
    ros_control_mode control_mode = ros_control_mode::cp;
    bool init_flag = false;
    bool init_flag_jp=false;
    // 机器人相关参数
    std::string ip_;       // ip
    std::string local_ip_; // 本机ip
    std::error_code ec_;   // 错误码
    rokae::xMateErProRobot *robot_ptr_;
    std::shared_ptr<rokae::RtMotionControlCobot<(unsigned short)7U>> rtCon_;
    std::array<double, 7> jntPos{};
    std::array<double, 7> jntVel{};
    std::array<double, 7> jntTau{};
    std::array<double, 16> tcpPose{};
    std::array<double, 6> tcpVel{};
    std::array<double, 6> tcpAcc{};

    Eigen::Vector3d tcpPosition0;
    Eigen::Matrix3d tcpRotation0;

    Eigen::Vector3d tcpPositionSum;
    Eigen::Matrix3d tcpRotationSum;


    std::array<double, 16> tcpPoseSave{};

    // ros相关的参数
    ros::Publisher js_pub_; // 关节状态发布
    ros::Publisher cp_pub_; // 笛卡尔位置发布`
    ros::Publisher cv_pub_; // 笛卡尔速度发布
    ros::Publisher ca_pub_; // 笛卡尔加速度发布
    ros::Publisher cf_pub_; // 笛卡尔力发布

    ros::Subscriber jp_sub_; // 关节位置接收
    ros::Subscriber jv_sub_; // 关节速度接收
    ros::Subscriber jf_sub_; // 关节力矩接收
    ros::Subscriber cp_sub_; // 笛卡尔速度接收
    ros::Subscriber cv_sub_; // 笛卡尔速度接收
    ros::Subscriber cf_sub_; // 笛卡尔力和力矩接收
    ros::Subscriber drag_sub; // 珞石自带拖动模式接收
    // 订阅消息的回调函数
    void jp_cb(const sensor_msgs::JointState::ConstPtr &msg);
    void jv_cb(const sensor_msgs::JointState::ConstPtr &msg);
    void jf_cb(const sensor_msgs::JointState::ConstPtr &msg);
    void cp_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void cv_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void cf_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void drag_cb(const std_msgs::String::ConstPtr &msg);


    // 关节控制消息
    std::array<double, 7> jp_command;
    std::array<double, 7> jv_command;
    std::array<double, 7> jf_command;
    std::array<double, 16> cp_command;
    std::array<double, 6> cv_command; // 格式是x,y,z,wx,wy,wz

    // 机器人控制相关
    CartesianPosition cmd_cartesian;

    // 方便计算，提取16D的array中的旋转矩阵,将旋旋转矩阵和位置合成为16D
    Eigen::Matrix3d
    array2rotation(const std::array<double, 16> &pose);
    std::array<double, 16> pose2array(Eigen::Matrix3d &rotation, Eigen::Vector3d &position);

    // 机器人控制回调函数
    JointPosition move_velocity_callback();      // 机器人关节空间位置回调
    JointPosition move_position_callback();      // 机器人关节空间速度回调
    CartesianPosition move_cartesian_callback(); // 机器人笛卡尔空间回调
    CartesianPosition move_twist_callback();     // 机器人笛卡尔空间速度回调
};

ROKAE_ROS::ROKAE_ROS(const std::string &m_ip, const std::string &local_ip)
{
    // 初始化机器人
    local_ip_ = local_ip;
    ip_ = m_ip;
    robot_ptr_ = new rokae::xMateErProRobot(ip_, local_ip_);

    // 初始化节点
    ros::NodeHandle nh;

    // 关节角发布(位置和速度)
    js_pub_ = nh.advertise<sensor_msgs::JointState>("rokae/state/JointState", 1);
    cp_pub_ = nh.advertise<geometry_msgs::PoseStamped>("rokae/state/CartesianPose", 1);
    cv_pub_ = nh.advertise<geometry_msgs::TwistStamped>("rokae/state/Twist", 1);
    ca_pub_ = nh.advertise<geometry_msgs::TwistStamped>("rokae/state/acc", 1);
    cf_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("rokae/state/Wrench", 1);

    // 创建订阅者
    jp_sub_ = nh.subscribe("rokae/command/JointPosition", 1, &ROKAE_ROS::jp_cb, this);
    jv_sub_ = nh.subscribe("rokae/command/JointVelocity", 1, &ROKAE_ROS::jv_cb, this);
    jf_sub_ = nh.subscribe("rokae/command/JointTorque", 1, &ROKAE_ROS::jf_cb, this);
    cp_sub_ = nh.subscribe("rokae/command/CartesianPose", 1, &ROKAE_ROS::cp_cb, this);
    cv_sub_ = nh.subscribe("rokae/command/Twist", 1, &ROKAE_ROS::cv_cb, this);
    cf_sub_ = nh.subscribe("rokae/command/Wrench", 1, &ROKAE_ROS::cf_cb, this);
    drag_sub = nh.subscribe("rokae/command/drag", 1, &ROKAE_ROS::drag_cb, this);

    cv_command={0,0,0,0,0,0};

    tcpPosition0<<0,0,0;

    tcpPositionSum<<0,0,0;
    tcpRotationSum=Eigen::Matrix3d::Identity();

}

ROKAE_ROS::~ROKAE_ROS()
{
    try
    {
        robot_ptr_->setPowerState(false, ec_);
        robot_ptr_->disconnectFromRobot(ec_);
        std::cout << "---------------------rokae power off-------------------" << std::endl;
    }
    catch (const std::exception &e)
    {
        print(std::cerr, e.what());
    }
}

Eigen::Matrix3d
ROKAE_ROS::array2rotation(const std::array<double, 16> &pose)
{
    Eigen::Matrix3d result;
    result << pose[0], pose[1], pose[2],
        pose[4], pose[5], pose[6],
        pose[8], pose[9], pose[10];
    return result;
}

std::array<double, 16> ROKAE_ROS::pose2array(Eigen::Matrix3d &rotation, Eigen::Vector3d &position)
{
    std::array<double, 16> result = {rotation(0, 0), rotation(0, 1), rotation(0, 2), position(0),
                                     rotation(1, 0), rotation(1, 1), rotation(1, 2), position(1),
                                     rotation(2, 0), rotation(2, 1), rotation(2, 2), position(2),
                                     0, 0, 0, 1};
    return result;
}

// 发送状态
void ROKAE_ROS::send_state()
{
    sensor_msgs::JointState js_msg;
    geometry_msgs::PoseStamped tp_msg;
    geometry_msgs::TwistStamped tv_msg;
    geometry_msgs::TwistStamped tcpAcc_msg;

    rtCon_->updateRobotState();

    rtCon_->getStateData(RtSupportedFields::jointPos_m, jntPos);
    rtCon_->getStateData(RtSupportedFields::jointVel_m, jntVel);
    rtCon_->getStateData(RtSupportedFields::tau_m, jntTau);
    rtCon_->getStateData(RtSupportedFields::tcpPose_m, tcpPose);
    rtCon_->getStateData(RtSupportedFields::tcpVel_c,tcpVel);
    rtCon_->getStateData(RtSupportedFields::tcpAcc_c,tcpAcc);

    std::vector<double> jntPos_vec(jntPos.begin(), jntPos.end());
    std::vector<double> jntVel_vec(jntVel.begin(), jntVel.end());
    std::vector<double> jntTau_vec(jntTau.begin(), jntTau.end());
    Eigen::Matrix3d tcp_R = array2rotation(tcpPose);
    Eigen::Quaterniond quaternion(tcp_R);

    if(tcpPosition0.isZero()){
        tcpPosition0<<tcpPose[3],tcpPose[7],tcpPose[11];
        tcpRotation0=tcp_R;
    }

    // 发布关节状态
    js_msg.header.stamp = ros::Time().now();
    js_msg.position = jntPos_vec;
    js_msg.velocity = jntVel_vec;
    js_msg.effort = jntTau_vec;
    js_pub_.publish(js_msg);

    //发送末端速度
    float angular_rate = 1000;
    tv_msg.header.stamp=ros::Time().now();
    tv_msg.twist.linear.x=tcpVel[0];
    tv_msg.twist.linear.y=tcpVel[1];
    tv_msg.twist.linear.z=tcpVel[2];
    tv_msg.twist.angular.x=tcpVel[3] * angular_rate;
    tv_msg.twist.angular.y=tcpVel[4] * angular_rate;
    tv_msg.twist.angular.z=tcpVel[5] * angular_rate;
    cv_pub_.publish(tv_msg);

    //发送末端加速度
    float angular_acc_rate = 1000;
    tcpAcc_msg.header.stamp=ros::Time().now();
    tcpAcc_msg.twist.linear.x=tcpAcc[0];
    tcpAcc_msg.twist.linear.y=tcpAcc[1];
    tcpAcc_msg.twist.linear.z=tcpAcc[2];
    tcpAcc_msg.twist.angular.x=tcpAcc[3] * angular_acc_rate;
    tcpAcc_msg.twist.angular.y=tcpAcc[4] * angular_acc_rate;
    tcpAcc_msg.twist.angular.z=tcpAcc[5] * angular_acc_rate;
    ca_pub_.publish(tcpAcc_msg);


    // 发布末端位置和姿态
    tp_msg.header.stamp = ros::Time().now();
    tp_msg.pose.position.x = tcpPose[3];
    tp_msg.pose.position.y = tcpPose[7];
    tp_msg.pose.position.z = tcpPose[11]; // x,y,z三个方向的位置
    tp_msg.pose.orientation.x = quaternion.x();
    tp_msg.pose.orientation.y = quaternion.y();
    tp_msg.pose.orientation.z = quaternion.z();
    tp_msg.pose.orientation.w = quaternion.w();
    // ROS_INFO("x: %f, y: %f",tp_msg.pose.position.x,tp_msg.pose.position.y);
    cp_pub_.publish(tp_msg);
}

void ROKAE_ROS::start_rokae()
{
    try
    {
        robot_ptr_->setOperateMode(rokae::OperateMode::automatic, ec_);
        robot_ptr_->setMotionControlMode(MotionControlMode::RtCommand, ec_);
        robot_ptr_->setPowerState(true, ec_);
        robot_ptr_->setRtNetworkTolerance(30,ec_);

        rtCon_ = robot_ptr_->getRtMotionController().lock();

        

        //  设置获取机器人状态的类型
        rtCon_->startReceiveRobotState({RtSupportedFields::jointPos_m, RtSupportedFields::jointVel_m,
                                        RtSupportedFields::tau_m, RtSupportedFields::tcpPose_m,RtSupportedFields::tcpVel_c});
        rtCon_->setFilterLimit(true, 10.0);
        rtCon_->setFilterFrequency(10.0, 10.0, 10.0, ec_);

        std::cout
            << "---------------rokae init finish------------" << std::endl;
        init_flag = true;
        init_flag_jp=true;
    }
    catch (const std::exception &e)
    {
        print(std::cerr, e.what());
    }
}

////////////////////////////////////////////////////////////////////////////////////
// ROS回调
void ROKAE_ROS::jp_cb(const sensor_msgs::JointState::ConstPtr &msg)
{
    control_mode = ros_control_mode::jp;
    std::vector<double> m_msg = msg->position;
    // std::copy(m_msg.begin(), m_msg.end(), jp_command.begin());
    for (int i=0;i<7;++i){
        jp_command[i]=m_msg[i];
    }
    
    // std::cout<<"msg->position: "<< msg->position <<"\n"
    //         <<"m_msg: "<<m_msg<<std::endl;
    // std::cout<<jp_command<<std::endl;
}

void ROKAE_ROS::jv_cb(const sensor_msgs::JointState::ConstPtr &msg)
{
    control_mode = ros_control_mode::jv;
    std::vector<double> m_msg = msg->velocity;
    std::copy(m_msg.begin(), m_msg.end(), jv_command.begin());
}

void ROKAE_ROS::jf_cb(const sensor_msgs::JointState::ConstPtr &msg)
{
    control_mode = ros_control_mode::jf;
    std::vector<double> m_msg = msg->effort;
    std::copy(m_msg.begin(), m_msg.end(), jf_command.begin());
}

void ROKAE_ROS::cp_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    control_mode = ros_control_mode::cp;
    Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaternion quaternion(msg->pose.orientation.w,
                                 msg->pose.orientation.x,
                                 msg->pose.orientation.y,
                                 msg->pose.orientation.z);
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();
    cp_command = pose2array(rotationMatrix, position);
}

void ROKAE_ROS::cv_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    control_mode = ros_control_mode::cv;
    cv_command = {msg->twist.linear.x,
                  msg->twist.linear.y,
                  msg->twist.linear.z,
                  msg->twist.angular.x,
                  msg->twist.angular.y,
                  msg->twist.angular.z};
}

void ROKAE_ROS::cf_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    control_mode = ros_control_mode::cf;

}

void ROKAE_ROS::drag_cb(const std_msgs::String::ConstPtr &msg)
{
    std::cout<<"drag_cd receive: "<<msg->data.c_str()<<std::endl;
    if(msg->data == "drag_start")
    {
        control_mode = ros_control_mode::drag_start;
        std::cout<<"control_mode: "<<control_mode<<std::endl;
    }
    else if(msg->data == "drag_stop")
    {
        control_mode = ros_control_mode::drag_stop;
        std::cout<<"control_mode: "<<control_mode<<std::endl;
    }
    
}

////////////////////////////////////////////////////////////////////////////////////
// 机器人回调
JointPosition ROKAE_ROS::move_position_callback()
{
    // std::cout<<"===================== jp_callback ====================="<<std::endl;
    // 读取并发送当前的机器人状态
    send_state();
    ros::spinOnce();

    double time_step = 0.001;
    if (init_flag_jp == true)
    {
        std::cout<<"--------------------------------------flag-----------------------------------"<<std::endl;
        jp_command = jntPos;
        init_flag_jp = false;
    }

    JointPosition cmd = {jp_command[0], jp_command[1], jp_command[2], jp_command[3],
                         jp_command[4], jp_command[5], jp_command[6]};
    // std::cout<<cmd.joints<<std::endl;
    if (control_mode != ros_control_mode::jp)
    {
        std::cout<<"change mode jp -> "<< control_mode <<std::endl;
        cmd.setFinished();
    }
    if (!ros::ok())
    {
        cmd.setFinished(); // ros停止时跳出
    }
    return cmd;
}

JointPosition ROKAE_ROS::move_velocity_callback()
{
    // 读取并发送当前的机器人状态
    // std::cout<<"===================== jv_callback ====================="<<std::endl;
    send_state();
    ros::spinOnce();
    double time_step = 0.001;
    JointPosition cmd = {jntPos[0] + jv_command[0] * time_step,
                         jntPos[1] + jv_command[1] * time_step,
                         jntPos[2] + jv_command[2] * time_step,
                         jntPos[3] + jv_command[3] * time_step,
                         jntPos[4] + jv_command[4] * time_step,
                         jntPos[5] + jv_command[5] * time_step,
                         jntPos[6] + jv_command[6] * time_step};
    if (control_mode != ros_control_mode::jv)
    {
        std::cout<<"change mode jv -> "<< control_mode<<std::endl;
        cmd.setFinished();
    }
    if (!ros::ok())
    {
        cmd.setFinished(); // ros停止时跳出
    }
    return cmd;
}

CartesianPosition ROKAE_ROS::move_cartesian_callback()
{
    // std::cout<<"===================== cp_callback ====================="<<std::endl;

    CartesianPosition cmd;


    double now = ros::Time::now().toSec();

    // std::cout << now - start << std::endl;
    start = now;

    send_state();
    ros::spinOnce();
    // std::cout << "cp:" << cp_command << std::endl;
    if (init_flag == true)
    {
        std::cout<<"--------------------------------------flag-----------------------------------"<<std::endl;
        cp_command = tcpPose;
        init_flag = false;
    }

    // 赋值
    cmd.pos = cp_command;

    // std::cout << cmd.pos << std::endl;
    if (control_mode != ros_control_mode::cp)
    {
        std::cout<<"change mode cp -> "<< control_mode<<std::endl;
        cmd.setFinished();
        time_change_mode = ros::Time::now().toSec();
    }

    if (!ros::ok())
    {
        cmd.setFinished(); // ros停止时跳出
    }
    return cmd;
}

CartesianPosition ROKAE_ROS::move_twist_callback()
{
    // std::cout<<"===================== cv_callback ====================="<<std::endl;
    send_state();
    ros::spinOnce();
    double time_step = 0.001;
    CartesianPosition cmd;


    // 计算累计位置变化
    Eigen::Vector3d delta_position={cv_command[0]* time_step,cv_command[1]* time_step,cv_command[2]* time_step};
    tcpPositionSum=tcpPositionSum+delta_position;


    // 计算当前时刻的姿态
    Eigen::Vector3d rotationVector(cv_command[3] * time_step, cv_command[4] * time_step, cv_command[5] * time_step);
    Eigen::AngleAxisd rotation(rotationVector.norm(), rotationVector.normalized());
    tcpRotationSum=rotation.toRotationMatrix()*tcpRotationSum;

    Eigen::Matrix3d rotation_now=tcpRotationSum*tcpRotation0;
    Eigen::Vector3d position_now=tcpPositionSum+tcpPosition0;

    std::array<double, 16> cmd_pose=pose2array(rotation_now,position_now);

    // 赋值
    cmd.pos=cmd_pose;

    if (control_mode != ros_control_mode::cv)
    {
        std::cout<<"change mode cv -> "<< control_mode<<std::endl;
        cmd.setFinished();
    }
    if (!ros::ok())
    {
        cmd.setFinished(); // ros停止时跳出
    }
    return cmd;
}

// 开始运动
void ROKAE_ROS::start_control()
{
    // 开始指定控制模式   下面的几个控制模式只能选择一个(没有写力控制，以后可以补充)
    send_state();


    // 笛卡尔位置控制
    control_mode = ros_control_mode::cp;
    rtCon_->startMove(RtControllerMode::cartesianPosition);
    std::function<CartesianPosition()> callback = std::bind(&ROKAE_ROS::move_cartesian_callback, this);

    

    // 笛卡尔速度控制
    // control_mode = ros_control_mode::cv;
    // rtCon_->startMove(RtControllerMode::cartesianPosition);
    // std::function<CartesianPosition()> callback = std::bind(&ROKAE_ROS::move_twist_callback, this);

    
    // // 关节位置控制
    // control_mode = ros_control_mode::jp;
    // rtCon_->startMove(RtControllerMode::jointPosition);
    // std::function<JointPosition()> callback = std::bind(&ROKAE_ROS::move_position_callback, this);

    // // 关节速度控制
    // control_mode = ros_control_mode::jv;
    // rtCon_->startMove(RtControllerMode::jointPosition);
    // std::function<JointPosition()> callback = std::bind(&ROKAE_ROS::move_velocity_callback, this);


    // 设置回调函数
    rtCon_->setControlLoop(callback);
    // 阻塞loop
    rtCon_->startLoop(true);


    while (ros::ok())
    {
        if(control_mode == ros_control_mode::drag_start)
        {
            std::cout<<"循环进入 drag_start 分支"<<std::endl;
            rtCon_->stopLoop();
            rtCon_->stopMove();
            robot_ptr_->setOperateMode(rokae::OperateMode::manual, ec_);// 打开拖动之前，需要机械臂处于手动模式下电状态
            robot_ptr_->setPowerState(false, ec_);// 打开拖动之前，需要机械臂处于手动模式下电状态
            robot_ptr_->enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec_);// 笛卡尔空间，自由拖动
            std::cout<<"mode change time""打开拖动"<<ec_<<std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待切换控制模式
            control_mode = ros_control_mode::dragging;
            std::cout<<"已开启拖动模式..."<<std::endl;
        }
        else if(control_mode == ros_control_mode::drag_stop)
        {
            std::cout<<"循环进入 drag_stop 分支"<<std::endl;
            rtCon_->stopLoop();
            rtCon_->stopMove();
            robot_ptr_-> disableDrag(ec_);
            std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待切换控制模式
            control_mode == ros_control_mode::jp;
        }
        else if(control_mode == ros_control_mode::dragging)
        {
            send_state();
            // std::cout<<"dragging"<<std::endl;
            continue;
        }
        else
        {
            rtCon_->stopLoop();
            rtCon_->stopMove();
        }
        

        if(control_mode == ros_control_mode::cp)
        {
            rtCon_->startMove(RtControllerMode::cartesianPosition);
            std::function<CartesianPosition()> callback = std::bind(&ROKAE_ROS::move_cartesian_callback, this);
            rtCon_->setControlLoop(callback);

        }
        else if (control_mode == ros_control_mode::cv)
        {
            rtCon_->startMove(RtControllerMode::cartesianPosition);

            rtCon_->updateRobotState();
            rtCon_->getStateData(RtSupportedFields::tcpPose_m, tcpPose);
            Eigen::Matrix3d tcp_R = array2rotation(tcpPose);
            tcpPosition0<<tcpPose[3],tcpPose[7],tcpPose[11];
            tcpRotation0=tcp_R;

            std::function<CartesianPosition()> callback = std::bind(&ROKAE_ROS::move_twist_callback, this);
            rtCon_->setControlLoop(callback);

        }
        else if (control_mode == ros_control_mode::jp)
        {

            rtCon_->startMove(RtControllerMode::jointPosition);
            std::function<JointPosition()> callback = std::bind(&ROKAE_ROS::move_position_callback, this);
            rtCon_->setControlLoop(callback);

        }
        else if (control_mode == ros_control_mode::jv)
        {
            rtCon_->startMove(RtControllerMode::jointPosition);
            std::function<JointPosition()> callback = std::bind(&ROKAE_ROS::move_velocity_callback, this);
            rtCon_->setControlLoop(callback);
        }

        // 设置回调函数
        std::cout<<"mode change time"<< ros::Time::now().toSec() - time_change_mode <<std::endl;
        // 阻塞loop
        if(control_mode != ros_control_mode::dragging )
            rtCon_->startLoop(true);

    }

    std::cout << "控制结束" << std::endl;
}
