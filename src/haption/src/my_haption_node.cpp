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

void wrench_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    // std::cout << "flag" << std::endl;
    set_force[0] = msg->wrench.force.x;
    set_force[1] = msg->wrench.force.y;
    set_force[2] = msg->wrench.force.z;
    set_force[3] = msg->wrench.torque.x;
    set_force[4] = msg->wrench.torque.y;
    set_force[5] = msg->wrench.torque.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_haption_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(125);
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/haption/state/pose", 1);
    ros::Publisher twist_publisher = nh.advertise<geometry_msgs::TwistStamped>("/haption/state/twist", 1);
    ros::Publisher button_publisher = nh.advertise<sensor_msgs::Joy>("/haption/state/buttons", 1);
    ros::Publisher exbutton_publisher = nh.advertise<sensor_msgs::Joy>("/haption/state/exbuttons", 1);
    ros::Subscriber wrench_subscriber = nh.subscribe<geometry_msgs::WrenchStamped>("/haption/command/wrench", 1, wrench_callback);

    VirtContext VC;
    // std::string ip_address = "192.168.100.10#5000";
    std::string ip_address = "192.168.100.10#53210";
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
    virtSetCommandType(VC, COMMAND_TYPE_IMPEDANCE);
    // virtSetIndexingMode(VC, INDEXING_ALL_FORCE_FEEDBACK_INHIBITION);
    virtSetPowerOn(VC, 1);
    virtSetPowerOn(VC, 1);

    float positions[7], speed[6];
    int button_power[1], button_red[1], button_yellow[1], button2[1], button3[1], button4[1], button5[1];
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped twist;
    geometry_msgs::WrenchStamped wrench;
    sensor_msgs::Joy joy;
    std::vector<int> buttons_vector(6);

    while (ros::ok())
    {
        virtGetPosition(VC, positions);
        virtGetSpeed(VC, speed);
        virtGetButton(VC, 6, button_power);
        virtGetButton(VC, 1, button_red);
        virtGetButton(VC, 2, button_yellow);

        // std::cout << "speed: " << speed[0] << " " << speed[1] << " " << speed[2] << std::endl;

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

        pose_publisher.publish(pose);
        twist_publisher.publish(twist);
        button_publisher.publish(joy);
        // std::cout << set_force[0] << " " << set_force[1] << " " << set_force[2] << std::endl;
        if (virtSetForce(VC, set_force))
        {
            fprintf(stderr, "Error calling virtSetForce: %s\n", virtGetErrorMessage(virtGetErrorCode(VC)));
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
