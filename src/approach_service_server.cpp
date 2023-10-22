#include "rclcpp/node.hpp"
#include <attach_self/srv/go_to_Loading.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Approach_Server : public rclcpp::Node{
    



};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Approach_Server>());
    rclcpp::shutdown();
    return 0;
}