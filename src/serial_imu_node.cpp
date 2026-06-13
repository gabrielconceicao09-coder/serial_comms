#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serial/serial.h"

#include <string>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;

class SerialImuNode : public rclcpp::Node
{
    public:
    SerialImuNode() : Node("serial_imu_node") 
    {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_ = this->declare_parameter<int>("baudrate", 921600);

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());

        try
        {
            serial_.setPort(port_);
            serial_.setBaudrate(baudrate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);

            serial_.open();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Erro abrindo port: %s", e.what());
            rclcpp::shutdown();
            return;
        };

        if (!serial_.isOpen())
        {
            RCLCPP_ERROR(this->get_logger(), "Comunicação serial não abriu");
            rclcpp::shutdown();
            return;
        };

        RCLCPP_INFO(this->get_logger(), "Comunicação serial aberta em: %s", port_.c_str());       
        
        

        timer_ = this->create_wall_timer(1ms, std::bind(&SerialImuNode::ReadPub_callback, this));
        RCLCPP_INFO(this->get_logger(), "Timer criado com callback");        
    }

    ~SerialImuNode()
    {

    }

    private:
    std::string port_;
    int baudrate_;

    serial::Serial serial_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void ReadPub_callback()
    {
        RCLCPP_INFO(this->get_logger(), "ReadPub_callback() chamado");
        
        if (!serial_.available())
        {
            RCLCPP_WARN(this->get_logger(), "Serial não disponível");
            return; //Retorna pra q seja tentado novamente
        }
        
        std::string line = serial_.readline(65536, "\n");

        if (line.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Problema na leitura ou leitura vazia");
            return;
        }

        std::stringstream ss(line);

        std::vector<double> valores;
        std::string item;
        
        try
        {
        while (std::getline(ss, item, ','))
        {
            valores.push_back(std::stod(item));
        }
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Erro na conversão de valores para double: %s", item.c_str());
            return;
        }

        if (valores.size() != 9)
        {
            RCLCPP_INFO(this->get_logger(), "Leitura de número errado de valores");
            return; 
        }
        
        sensor_msgs::msg::Imu msg;
        
        msg.header.frame_id = "imu_link";

        msg.linear_acceleration.x = valores[0];
        msg.linear_acceleration.y = valores[1];
        msg.linear_acceleration.z = valores[2];
        msg.linear_acceleration_covariance[0] = -1;
        msg.angular_velocity.x = valores[3];
        msg.angular_velocity.y = valores[4];
        msg.angular_velocity.z = valores[5];
        msg.angular_velocity_covariance[0] = -1;
        msg.orientation.x = valores[6];
        msg.orientation.y = valores[7];
        msg.orientation.z = valores[8]; //compõe a mensagem
        msg.orientation_covariance[0] = -1;


        msg.header.stamp = this->now() + rclcpp::Duration(0, 10000000);
        try {
            imu_pub_->publish(msg); //publica as medições do MPU
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Mensagem não publicada");
        }
            
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialImuNode>());
    rclcpp::shutdown();
    return 0;
}
