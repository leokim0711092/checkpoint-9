#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>
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
            

            this->declare_parameter("obstacle", 0.0);
            this->declare_parameter<int>("degrees", 0);

            getting_params();

            sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10 , 
            std::bind(&Attach_self::scan_callback, this, std::placeholders::_1));

            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
            std::bind(&Attach_self::odom_callback, this, std::placeholders::_1));

            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);

        }

    private:
        float obstacle;
        int degrees;
        geometry_msgs::msg::Twist vel;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        bool turn;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "obstacle: %f", obstacle);
            RCLCPP_INFO(this->get_logger(), "degrees: %i", degrees );
            if(msg->ranges[359] > obstacle){
                vel.linear.x = 0.2;
            }else if (msg->ranges[359] < obstacle){
                vel.linear.x = 0;
                turn = true;
            }
        }

        void odom_callback(const nav_msgs::msg::Odometry msg){
            float cur_degree = euler_degree_transform(msg)/M_PI*360;
            RCLCPP_INFO(this->get_logger(), "cur_degrees: %f", cur_degree );
            if (degrees - cur_degree > 1 && turn ) {
                vel.angular.z = (degrees - cur_degree)*M_PI/360*0.5 ;
            }else{
                vel.angular.z = 0;
            }
        }

        void getting_params(){
            obstacle = this->get_parameter("obstacle").get_parameter_value().get<float>();
            degrees = this->get_parameter("degrees").as_int();
        }

        float euler_degree_transform(const nav_msgs::msg::Odometry t_st){
                float x = t_st.pose.pose.orientation.x;
                float y = t_st.pose.pose.orientation.y;
                float z = t_st.pose.pose.orientation.z;
                float w = t_st.pose.pose.orientation.w; 

            return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
        }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Attach_self>(argc, argv));
    rclcpp::shutdown();
    return 0;
}