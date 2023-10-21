#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

class Attach_self : public rclcpp::Node{

    public:
        Attach_self(int &argc, char **argv): Node("attach_self"){
            obstacle = std::stof(argv[2]);
            degree = std::stoi(argv[4]);

            sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10 , 
            std::bind(&Attach_self::scan_callback, this, std::placeholders::_1));

            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            std::bind(&Attach_self::odom_callback, this, std::placeholders::_1));

            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);

        }

    private:
        float obstacle;
        int degree;
        geometry_msgs::msg::Twist vel;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

        
        }

        void odom_callback(const nav_msgs::msg::Odometry msg){
            
        
        }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Attach_self>(argc, argv));
    rclcpp::shutdown();
    return 0;
}