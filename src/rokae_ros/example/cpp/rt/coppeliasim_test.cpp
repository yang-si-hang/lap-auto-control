#include "/home/irobotcare/rokae/rokae_ws_wyh/src/rokae_ros/src/rokae_ros.hpp"
#include <std_msgs/Float64.h>
//#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "copp_sim_joint_state");
    ros::NodeHandle n;
    ros::Publisher copp_pub = n.advertise<std_msgs::Float64>("/copp/JointState", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    int flag = 1;
    while(ros::ok())
    {
        std_msgs::Float64 copp_f64;
        copp_f64.data = count * M_PI * 0.01;
        copp_pub.publish(copp_f64);

        ros::spinOnce();

        loop_rate.sleep();///ros::Rate对象按照前面的设置进行休眠延时。

        count = count + 1 * flag;
        if(count >= 90) flag = -1;
        if(count <= -90) flag = 1;
    }

    return 0;

}
