#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <attach_shelf/srv/go_to_loading.hpp>
#include <cmath>
#include <cstddef>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <utility>
#include <vector>
#include <algorithm>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class Approach_Server : public rclcpp::Node{
    
    public:
        Approach_Server(): Node("approach_server"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_){
            
            sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10 , 
            std::bind(&Approach_Server::scan_callback, this, std::placeholders::_1));
            srv_ = create_service<attach_shelf::srv::GoToLoading>("/approach_shelf",std::bind(&Approach_Server::attach_callback, this, std::placeholders::_1, std::placeholders::_2));
            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
        }
    private:
        
        geometry_msgs::msg::Twist vel;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
        rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr srv_;
        // tf2_ros::TransformBroadcaster broadcaster_{this}; //this will disappear in short time
        tf2_ros::StaticTransformBroadcaster static_broadcaster_{this};
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        float x1, x2, y1, y2;
        size_t accept_idx_size=0;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            
            // RCLCPP_INFO(this->get_logger(), "Intensity size:%zu" ,msg->intensities.size());
            // for(size_t i=0; i<msg->intensities.size();i++)
            // if(msg->intensities[i]>= 8000)
            // RCLCPP_INFO(this->get_logger(), "Scan intensity:%zu, %f",i, msg->intensities[i]);

            int minimum_idx_difference = 30;
            std::vector<std::pair<float, int>> intensity_cont;

            for(size_t i=0; i<msg->intensities.size();i++) 
            intensity_cont.emplace_back(msg->intensities[i],i);

            std::sort(intensity_cont.begin(), intensity_cont.end(),
                  [](const std::pair<float, int>& a, const std::pair<float, int>& b)-> 
                  bool {
                    return a.first > b.first;
                  });
            std::vector<int> accept_idx;
            for(const auto & pair : intensity_cont){
                

                bool close = false;
                for(const int acp : accept_idx){
                    if(std::abs(acp - pair.second) < minimum_idx_difference){
                        close = true;
                        break;
                    }
                }

                if(!close) accept_idx.push_back(pair.second);
                if(accept_idx.size() == 2) break;
            }
            accept_idx_size = accept_idx.size();

            int idx_1 = accept_idx[0];
            int idx_2 = accept_idx[1];
            
            float angle1 = msg->angle_min + idx_1*msg->angle_increment;
            float angle2 = msg->angle_min + idx_2*msg->angle_increment;

            x1 = msg->ranges[idx_1]* cos(angle1);
            y1 = msg->ranges[idx_1]* sin(angle1);

            x2 = msg->ranges[idx_2]* cos(angle2);
            y2 = msg->ranges[idx_2]* sin(angle2);

            // RCLCPP_INFO(this->get_logger(), "Intensity Index 1:%i",idx_1);
            // RCLCPP_INFO(this->get_logger(), "Intensity Index 2:%i",idx_2);
            // RCLCPP_INFO(this->get_logger(), "Intensity  1:%f",intensity_cont[0].first);
            // RCLCPP_INFO(this->get_logger(), "Intensity  2:%f",intensity_cont[1].first);
        }


        void attach_callback(const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> req, 
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Response> res){

            if(req->attach_to_shelf == true && accept_idx_size == 2){
                // broadcaster_.sendTransform(broadcast_transform( (x1+x2)/2 , (y1+y2)/2 )); //this will disappear in short time
                static_broadcaster_.sendTransform(broadcast_transform( (x1+x2)/2 , (y1+y2)/2 ));
                move_towards_cart_frame();
                RCLCPP_INFO(this->get_logger(), "Approach call");

                RCLCPP_INFO(this->get_logger(), "Accept_idz = %zu",accept_idx_size);

                res->complete = true;
            }else if(req->attach_to_shelf == false && accept_idx_size == 2){
                // broadcaster_.sendTransform(broadcast_transform( (x1+x2)/2 , (y1+y2)/2 )); //this will disappear in short time
                static_broadcaster_.sendTransform(broadcast_transform( (x1+x2)/2 , (y1+y2)/2 ));
                RCLCPP_INFO(this->get_logger(), "Approach not call but publish frame");
               
                RCLCPP_INFO(this->get_logger(), "Accept_idz = %zu",accept_idx_size);

                res->complete = false;
            }else {
                RCLCPP_INFO(this->get_logger(), "Approach did not meet two legs");

                if (res->complete) {
                    RCLCPP_INFO(this->get_logger(), "final_approach: true");
                }else if (!res->complete) {
                    RCLCPP_INFO(this->get_logger(), "final_approach: false");
                } else{
                    RCLCPP_INFO(this->get_logger(), "final_approach: none");
                }
                RCLCPP_INFO(this->get_logger(), "Accept_idz = %zu",accept_idx_size);

                // RCLCPP_INFO(this->get_logger(), "Intensity  1:%f",intensity_cont[0].first);
                // RCLCPP_INFO(this->get_logger(), "Intensity  2:%f",intensity_cont[1].first);
                res->complete = false;
            }
        }

        geometry_msgs::msg::TransformStamped broadcast_transform(float x,float y){
                
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->now();
                t.header.frame_id = "robot_front_laser_base_link";
                t.child_frame_id = "cart_frame";
                t.transform.translation.x = x;
                t.transform.translation.y = y;
                t.transform.translation.z = 0.0;  // Assuming it's 2D plane
                t.transform.rotation.x = 0.0;
                t.transform.rotation.y = 0.0;
                t.transform.rotation.z = 0.0;
                t.transform.rotation.w = 1.0;

                return t;
        }

         void move_towards_cart_frame(){
            // Get the transform from base_link to cart_frame
            geometry_msgs::msg::TransformStamped transform;
            try
            {
                transform = tf_buffer_.lookupTransform("robot_front_laser_base_link", "cart_frame", rclcpp::Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                return;
            }

            // Compute the direction and magnitude based on the transform's translation
            float distance_to_point = std::sqrt(
                transform.transform.translation.x * transform.transform.translation.x +
                transform.transform.translation.y * transform.transform.translation.y);

            float desired_distance = distance_to_point;  // Approach by 30cm
            vel.linear.x = desired_distance; // move in x-direction of robot's frame
            RCLCPP_INFO(this->get_logger(), "vel.linear = %f",vel.linear.x);

            pub_->publish(vel);
            rclcpp::Rate l(1);
            l.sleep();
            vel.linear.x = 0;
            pub_->publish(vel);
    }


};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Approach_Server>());
    rclcpp::shutdown();
    return 0;
}