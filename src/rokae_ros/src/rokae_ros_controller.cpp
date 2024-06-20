#include "rokae_ros.hpp"
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "rokae_ros_controller");
    std::string r_ip = "192.168.0.160";
    std::string local_ip = "192.168.0.100";

    ROKAE_ROS rokae_ros(r_ip, local_ip);

    rokae_ros.start_rokae();

    rokae_ros.start_control();

    return 0;
}