//
// Created by chy on 2024/3/27.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Joy.h>
#include "VirtuoseAPI.h"

float set_pose[7];

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // std::cout << "flag" << std::endl;
    set_pose[0] = msg->pose.position.x;
    set_pose[1] = msg->pose.position.y;
    set_pose[2] = msg->pose.position.z;
    set_pose[3] = msg->pose.orientation.x;
    set_pose[4] = msg->pose.orientation.y;
    set_pose[5] = msg->pose.orientation.z;
    set_pose[6] = msg->pose.orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "haption_pose_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(125);
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/haption/state/pose", 1);
    ros::Publisher twist_publisher = nh.advertise<geometry_msgs::TwistStamped>("/haption/state/twist", 1);
    ros::Publisher button_publisher = nh.advertise<sensor_msgs::Joy>("/haption/state/buttons", 1);
    ros::Publisher wrench_publisher = nh.advertise<geometry_msgs::WrenchStamped>("/haption/state/wrench", 1);
    ros::Publisher exbutton_publisher = nh.advertise<sensor_msgs::Joy>("/haption/state/exbuttons", 1);

    ros::Subscriber pose_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("/haption/command/pose", 1, pose_callback);

    VirtContext VC;
    std::string ip_address = "192.168.100.10#53210";//这个地方的端口号：当机械臂是120型号时为5000，是166型号和190型号时改为53210
    VC = virtOpen(ip_address.c_str());
    if (VC == nullptr)
    {
        fprintf(stderr, "Open faild: %s\n",
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
    // virtSetIndexingMode(VC, INDEXING_ALL);
    virtSetPowerOn(VC, 1);

    float positions[7], speed[6], force[6];
    int button_power[1], button_red[1], button_yellow[1], button2[1], button3[1], button4[1], button5[1];
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped twist;
    geometry_msgs::WrenchStamped wrench;
    sensor_msgs::Joy joy;
    std::vector<int> buttons_vector(6);
    virtVmActivate(VC);
    // virtVmSetRobotMode(VC, 1);
    int result = virtVmSetDefaultToCartesianPosition(VC); // 锁住所有关节
    virtVmSetBaseFrameToCurrentFrame(VC);

    virtGetPosition(VC, positions);
    printf("result : %d", result);

    bool is_init = false;
    bool is_changed = false;
    int deadtMan[1];

    while (ros::ok())
    {
        // virtGetPosition(VC, positions);
        if (!is_init && positions[0] != 0.0)
        {
            for (int i = 0; i < 7; ++i)
            {
                set_pose[i] = positions[i];
                std::cout << set_pose[i] << " ";
            }
            std::cout << std::endl;
            is_init = true;
        }

        // virtSetPosition(VC, set_pose);

        virtGetSpeed(VC, speed);
        virtGetForce(VC, force);
        virtGetButton(VC, 6, button_power);
        virtGetButton(VC, 1, button_red);
        virtGetButton(VC, 2, button_yellow);
        virtGetDeadMan(VC, deadtMan);

        // std::cout << "deadMan: " << deadtMan[0] << std::endl;

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

        joy.header.stamp = ros::Time::now();
        buttons_vector[0] = button_power[0];
        buttons_vector[1] = button_red[0];
        buttons_vector[2] = button_yellow[0];

        joy.buttons = buttons_vector;

        wrench.header.stamp = ros::Time::now();
        wrench.wrench.force.x = force[0];
        wrench.wrench.force.y = force[1];
        wrench.wrench.force.z = force[2];
        wrench.wrench.torque.x = force[3];
        wrench.wrench.torque.y = force[4];
        wrench.wrench.torque.z = force[5];

        pose_publisher.publish(pose);
        twist_publisher.publish(twist);
        button_publisher.publish(joy);
        wrench_publisher.publish(wrench);
        // std::cout << "pose.x: " << set_pose[0] << std::endl;

        // virtGetPosition(VC, positions);
        // virtSetPosition(VC, positions);
        // virtGetSpeed(VC, speed);
        // virtSetSpeed(VC, speed);

        if (button_red[0] == 1 && !is_changed)
        {
            // virtVmDeactivate(VC);

            if (virtVmSetDefaultToTransparentMode(VC))
            {
                fprintf(stderr, "Error calling virtSetFree: %s\n", virtGetErrorMessage(virtVmSetDefaultToTransparentMode(VC)));
            }
            // virtVmActivate(VC);
            is_changed = true;
            ROS_INFO("flag1");
        }
        else if (button_red[0] == 0 && is_changed)
        {
            virtVmDeactivate(VC);
            virtVmSetDefaultToCartesianPosition(VC);
            virtVmSetBaseFrameToCurrentFrame(VC);
            virtVmActivate(VC);
            is_changed = false;
            ROS_INFO("flag2");
        }
        else
        {
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    virtVmDeactivate(VC);
    virtVmSetDefaultToTransparentMode(VC);
    return 0;
}
