#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <attach_shelf/srv/go_to_loading.hpp>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Approach_Server : public rclcpp::Node{
    
    public:
        Approach_Server(): Node("approach_server"){
            
            sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10 , 
            std::bind(&Approach_Server::scan_callback, this, std::placeholders::_1));
            srv_ = create_service<attach_shelf::srv::GoToLoading>("/approach_shelf",std::bind(&Approach_Server::attach_callback, this, std::placeholders::_1, std::placeholders::_2));
            
        }
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
        sensor_msgs::msg::LaserScan::SharedPtr scan_msg;
        rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr srv_;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            scan_msg = msg;
            RCLCPP_INFO(this->get_logger(), "Intensity size:%zu" ,msg->intensities.size());

            for(size_t i=0; i<msg->intensities.size();i++)
            if(msg->intensities[i]>= 8000)
            RCLCPP_INFO(this->get_logger(), "Scan intensity:%zu, %f",i, msg->intensities[i]);

        }

        void attach_callback(const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> req, 
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Response> res){
            if(req->attach_to_shelf == true){
                
            }
        }

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Approach_Server>());
    rclcpp::shutdown();
    return 0;
}