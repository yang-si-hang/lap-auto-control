//
// Created by chy on 2024/3/27.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Joy.h>
#include "VirtuoseAPI.h"

float set_force[6];

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_haption_node_2");
    // ros::init(argc, argv, "my_haption_node_3");
    ros::NodeHandle nh;
    ros::Rate loop_rate(125);
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/haption2/state/pose", 1);
    ros::Publisher twist_publisher = nh.advertise<geometry_msgs::TwistStamped>("/haption2/state/twist", 1);
    ros::Publisher button_publisher = nh.advertise<sensor_msgs::Joy>("/haption2/state/buttons", 1);
    ros::Publisher exbutton_publisher = nh.advertise<sensor_msgs::Joy>("/haption2/state/exbuttons", 1);
    // ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/haption3/state/pose", 1);
    // ros::Publisher twist_publisher = nh.advertise<geometry_msgs::TwistStamped>("/haption3/state/twist", 1);
    // ros::Publisher button_publisher = nh.advertise<sensor_msgs::Joy>("/haption3/state/buttons", 1);
    // ros::Publisher exbutton_publisher = nh.advertise<sensor_msgs::Joy>("/haption3/state/exbuttons", 1);

    // lzw 20240423
    ros::Publisher wrench_publisher = nh.advertise<geometry_msgs::WrenchStamped>("/haption2/state/wrench", 1);
    // ros::Publisher wrench_publisher = nh.advertise<geometry_msgs::WrenchStamped>("/haption3/state/wrench", 1);
    float fWrench[6];

    VirtContext VC;
    // std::string ip_address = "192.168.100.10#53210"; // 这个地方的端口号：当机械臂是120型号时为5000，是166型号和190型号时改为53210
    std::string ip_address = "192.168.100.10#5000"; // 这个地方的端口号：当机械臂是120型号时为5000，是166型号和190型号时改为53210
    VC = virtOpen(ip_address.c_str());
    if (VC == nullptr)
    {
        fprintf(stderr, "Open  faild: %s\n",
                virtGetErrorMessage(virtGetErrorCode(nullptr)));
    }
    else
    {
        printf("Calling virtOpen with address <%s>\n", ip_address.c_str());
    }

    float identity[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    virtSetForceFactor(VC, 1.0f);
    virtSetSpeedFactor(VC, 1.0f);
    virtEnableForceFeedback(VC, true);
    virtSetTimeStep(VC, 0.001);
    virtSetBaseFrame(VC, identity);
    virtSetObservationFrame(VC, identity);
    virtSetCommandType(VC, COMMAND_TYPE_VIRTMECH);
    virtSetIndexingMode(VC, INDEXING_ALL_FORCE_FEEDBACK_INHIBITION);
    virtSetPowerOn(VC, 1);
    virtVmSetDefaultToCartesianPosition(VC); // 锁住所有关节
    virtVmSetBaseFrameToCurrentFrame(VC);

    float positions[7], speed[6];
    int button_power[1], button_red[1], button_yellow[1], button2[1], button3[1], button4[1], button5[1];
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped twist;
    geometry_msgs::WrenchStamped wrench;
    sensor_msgs::Joy joy;
    std::vector<int> buttons_vector(6);

    bool button_flag = false;
    bool button_flag_saved = false;
    bool is_changed = false;

    while (ros::ok())
    {

        // std::cout << "speed: " << speed[0] << " " << speed[1] << " " << speed[2] << std::endl;

        virtGetButton(VC, 1, button_red);

        button_flag_saved = button_flag;

        if (button_red[0] && !button_flag)
        {
            button_flag = true;
        }
        else if (!button_red[0] && button_flag)
        {
            button_flag = false;
        }

        if (button_flag_saved != button_flag)
        {
            is_changed = true;
        }
        else
        {
            is_changed = false;
        }

        // 切换状态
        if (button_flag && is_changed)
        {
            virtSetPowerOn(VC, 0);

            virtSetCommandType(VC, COMMAND_TYPE_IMPEDANCE);
            virtSetIndexingMode(VC, INDEXING_ALL_FORCE_FEEDBACK_INHIBITION);
            virtSetPowerOn(VC, 1);
            std::cout << "switch1" << std::endl;
        }
        else if (!button_flag && is_changed)
        {
            virtSetPowerOn(VC, 0);
            virtSetCommandType(VC, COMMAND_TYPE_VIRTMECH);
            virtSetIndexingMode(VC, INDEXING_ALL_FORCE_FEEDBACK_INHIBITION);
            virtSetPowerOn(VC, 1);
            virtVmSetDefaultToCartesianPosition(VC); // 锁住所有关节
            virtVmSetBaseFrameToCurrentFrame(VC);

            std::cout << "switch2" << std::endl;
        }

        // 发布所有状态
        virtGetPosition(VC, positions);
        virtGetSpeed(VC, speed);
        virtGetButton(VC, 6, button_power);
        // virtGetButton(VC, 1, button_red);
        virtGetButton(VC, 2, button_yellow);
        virtGetForce(VC, fWrench);

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = positions[0];
        pose.pose.position.y = positions[1];
        pose.pose.position.z = positions[2];
        pose.pose.orientation.x = positions[3];
        pose.pose.orientation.y = positions[4];
        pose.pose.orientation.z = positions[5];
        pose.pose.orientation.w = positions[6];

        twist.header.stamp = ros::Time::now();
        twist.header.frame_id = "world";
        twist.twist.linear.x = speed[0];
        twist.twist.linear.y = speed[1];
        twist.twist.linear.z = speed[2];
        twist.twist.angular.x = speed[3];
        twist.twist.angular.y = speed[4];
        twist.twist.angular.z = speed[5];

        wrench.header.stamp = ros::Time::now();
        wrench.header.frame_id = "world";
        wrench.wrench.force.x = fWrench[0];
        wrench.wrench.force.y = fWrench[1];
        wrench.wrench.force.z = fWrench[2];
        wrench.wrench.torque.x = fWrench[3];
        wrench.wrench.torque.y = fWrench[4];
        wrench.wrench.torque.z = fWrench[5];

        joy.header.stamp = ros::Time::now();
        buttons_vector[0] = button_power[0];
        buttons_vector[1] = button_red[0];
        buttons_vector[2] = button_yellow[0];

        joy.buttons = buttons_vector;

        pose_publisher.publish(pose);
        twist_publisher.publish(twist);
        button_publisher.publish(joy);
        wrench_publisher.publish(wrench);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
