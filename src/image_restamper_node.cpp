#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <string>

class ImageRestamperNode : public rclcpp::Node
{
    public:
    ImageRestamperNode() : Node("image_restamper_node")
    {
        sub_topic_ = this->declare_parameter<std::string>("sub_topic", "/image_raw");
        pub_topic_ = this->declare_parameter<std::string>("pub_topic", "/image_restamped");

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(sub_topic_, 10, std::bind);
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(pub_topic_, 10);
    }

    ~ImageRestamperNode(){

    }

    private:
    std::string sub_topic_, pub_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;

    void Restamp(sensor_msgs::msg::Image::SharedPtr msg){
        msg->header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
        try{
        img_pub_->publish(*msg);
        }
        catch (...){
            RCLCPP_INFO(this->get_logger(), "Imagem restampada não publicada");
        }
    }
}