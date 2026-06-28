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

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(sub_topic_, 10,
             std::bind(&ImageRestamperNode::Restamp, this, std::placeholders::_1));
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(pub_topic_, 10);
        RCLCPP_INFO(this->get_logger(), "Nó image_restamper_node iniciado com inscrição em %s e publicando em %s", sub_topic_, pub_topic_);
    }

    ~ImageRestamperNode(){

    }

    private:
    std::string sub_topic_, pub_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    bool primeira_leitura = true;
    uint32_t offset_clocks_ns;


    void Restamp(sensor_msgs::msg::Image::SharedPtr msg){
        rclcpp::Time tempo_steady = rclcpp::Clock(RCL_STEADY_TIME).now();
        if (primeira_leitura){
            uint32_t t0_imagem = msg->header.stamp.nanosec;
            offset_clocks_ns = tempo_steady.nanoseconds() - t0_imagem;
            primeira_leitura = false;
        }
        uint32_t tempo_mensagem_ns = msg->header.stamp.nanosec;
        msg->header.stamp = rclcpp::Time(tempo_mensagem_ns + offset_clocks_ns);
        try{
        img_pub_->publish(*msg);
        }
        catch (...){
            RCLCPP_INFO(this->get_logger(), "Imagem restampada não publicada");
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageRestamperNode>());
    rclcpp::shutdown();
    return 0;
}