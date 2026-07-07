#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "serial/serial.h"

#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>

//Recebe dados dos encoders obtidos da ESP32. Publica os dados compostos em mensagens nos formatos ROS2
//Inscreve o tópico de comandos de velocidade e envia os comandos para a ESP32

using namespace std::chrono_literals;

class SerialImuNode : public rclcpp::Node
{
    public:
    SerialMotorsNode() : Node("serial_motors_node") 
    {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_ = this->declare_parameter<int>("baudrate", 921600);

        variancia_encoders_ = this->declare_parameter<double>("variancia_encoders", 1.0); //TODO: Melhor estimativa para a covariância
        raio_roda_esq_ = this->declare_parameter<double>("raio_roda_esq", 0.125); //em metros
        raio_roda_dir_ = this->declare_parameter<double>("raio_roda_dir", 0.125); //em metros 

        encoders_topic_ = this->declare_parameter<std::string>("encoders_topic", "encoders/twist_raw");
        commands_topic_ = this->declare_parameter<std::string>("commands_topic", "nav2/commands");

        axis_frame_ = this->declare_parameter<std::string>("axis_frame", "base_link"); //Frame do eixo. Como normalmente é a referência, base_link (centro do eixo na base do robô)
        
        encoders_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(encoders_topic_, rclcpp::SensorDataQoS());
        commands_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(commands_topic_, 10, std::bind(&SerialMotorsNode::CommandSub_callback, this, _1)); //TODO: Checar como são os comandos do Nav2

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
        
        
        timer_ = this->create_wall_timer(4ms, std::bind(&SerialMotorsNode::ReadPub_callback, this));
        RCLCPP_INFO(this->get_logger(), "Timer criado com callback");        
    }

    ~SerialImuNode()
    {

    }

    private:
    //Parâmetros:
    std::string port_, raw_imu_topic_, raw_mag_topic_, gps_topic_, sonar_topic_;
    int baudrate_;
    double variancia_encoders_;
    double raio_roda_esq_, raio_roda_dir_;

    std::string axis_frame_;

    serial::Serial serial_;

    //Publishers:
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr encoders_pub_;

    //Subscriptions:
    rclcpp::Subscription<geometry_msg::msg::Twist>::SharedPtr commands_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    //Variáveis para ajuste temporal das mensagens
    bool primeira_leitura = true;
    int64_t offset_clocks_ns;
    double offset_alpha_ = 0.01;
    uint32_t ultima_seq_encoders = 0;


    void ReadPub_callback()
    {
        RCLCPP_INFO(this->get_logger(), "ReadPub_callback() chamado");

        //Leitura da porta serial:
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

        if (valores.size() != ) 
        {
            RCLCPP_INFO(this->get_logger(), "Leitura de número errado de valores. Linha: %s, n de valores: %i", line.c_str(), valores.size());
            return; 
        }
        //----------------------------------------------------------------------------
        
        //Tempo atual do sistema:
        rclcpp::Time tempo_ros = rclcpp::Clock(RCL_SYSTEM_TIME).now();
            
        //Composição mensagem Twist dos encoders:
        uint32_t sequencia_encoders = (uint32_t) valores[0];
        if (sequencia_encoders>ultima_seq_encoders){
            ultima_seq_encoders = sequencia_encoders;

            //Composição do tempo de aquisição no relógio do sistema:
            if (primeira_leitura){
                offset_clocks_ns = tempo_ros.nanoseconds() - micros_esp_imu*1000LL;
                primeira_leitura = false;
            }
            else{
                //Ajuste contínuo do offset:
                offset_clocks_ns = (int64_t) offset_clocks_ns*(1-offset_alpha_) + (tempo_ros.nanoseconds() - micros_esp_imu*1000LL)*offset_alpha_;
            }
                
            auto encodersMsg = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();

            encoderMsg->header.frame_id = axis_frame_;

            double rpm_esq = valores[2];
            double rpm_dir = valores[3];
            //TODO: calcular velocidade angular e velocidade linear do robô

            encoderMsg->twist.twist.angular.x = 0.0;
            encoderMsg->twist.twist.angular.y = 0.0;
            encoderMsg->twist.twist.angular.z = rpm_esq; //ERRADO
            encoderMsg->twist.twist.linear.x = rpm_esq; //ERRADO
            encoderMsg->twist.twist.linear.y = rpm_dir; //ERRADO
            encoderMsg->twist.twist.linear.z = 0.0;
            encoderMsg->twist.covariance[0] = variancia_encoders_; //TODO: Checar/melhorar variância dos encoders (acho que deve ser alta) 
            encoderMsg->twist.covariance[1] = 0.0; encoderMsg->twist.covariance[2] = 0.0; encoderMsg->twist.covariance[3] = 0.0; 
            encoderMsg->twist.covariance[4] = 0.0; encoderMsg->twist.covariance[5] = 0.0; encoderMsg->twist.covariance[6] = 0.0; 
            encoderMsg->twist.covariance[7] = variancia_encoders_; 
            encoderMsg->twist.covariance[8] = 0.0; encoderMsg->twist.covariance[9] = 0.0; encoderMsg->twist.covariance[10] = 0.0;
            encoderMsg->twist.covariance[11] = 0.0; encoderMsg->twist.covariance[12] = 0.0; encoderMsg->twist.covariance[13] = 0.0;
            encoderMsg->twist.covariance[14] = variancia_encoders_;
            encoderMsg->twist.covariance[15] = 0.0; encoderMsg->twist.covariance[16] = 0.0; encoderMsg->twist.covariance[17] = 0.0;
            encoderMsg->twist.covariance[18] = 0.0; encoderMsg->twist.covariance[19] = 0.0; encoderMsg->twist.covariance[20] = 0.0;
            encoderMsg->twist.covariance[21] = variancia_encoders_;
            encoderMsg->twist.covariance[22] = 0.0; encoderMsg->twist.covariance[23] = 0.0; encoderMsg->twist.covariance[24] = 0.0;
            encoderMsg->twist.covariance[25] = 0.0; encoderMsg->twist.covariance[26] = 0.0; encoderMsg->twist.covariance[27] = 0.0;
            encoderMsg->twist.covariance[28] = variancia_encoders_;
            encoderMsg->twist.covariance[29] = 0.0; encoderMsg->twist.covariance[30] = 0.0; encoderMsg->twist.covariance[31] = 0.0;
            encoderMsg->twist.covariance[32] = 0.0; encoderMsg->twist.covariance[33] = 0.0; encoderMsg->twist.covariance[34] = 0.0;
            encoderMsg->twist.covariance[35] = variancia_encoders_;

            int64_t micros_esp_encoders = (int64_t) valores[1];
            int64_t timestamp_encoders_ns = micros_esp_encoders*1000LL + offset_clocks_ns;
            encodersMsg->header.stamp = rclcpp::Time(timestamp_encoders_ns);

            try{
                encoder_pub_->publish(*encoderMsg);
            } catch (...) RCLCPP_WARN(this->get_logger(), "Mensagem encoders não publicada");
        }
    
    }
    
    void CommandSub_callback(geometry_msgs::msg::Twist msg)
    {
        float vel_linearx = msg.linear.x;
        float vel_lineary = msg.linear.y;
        float vel_linearz = msg.linear.z;

        float vel_angx = msg.angular.x;
        float vel_angy = msg.angular.y;
        float vel_angz = msg.angular.z;

        //TODO: Cálculo das velocidades necessárias;
        float rpm_ref_esq = 100.0;
        float rpm_ref_dir = 100.0;

        const std::string rpm_ref_esq_s = std::to_string(rpm_ref_esq);
        const std::string rpm_ref_dir_s = std::to_string(rpm_ref_dir);
        const std::string marcador_esq = "E";
        const std::string marcador_dir = "D";
        const std::string linebreak = "\n";

        size_t bytes_escritos = serial_.write(&marcador_esq);
        bytes_escritos += serial_.write(&rpm_ref_esq_s);
        bytes_escritos += serial_.write(&linebreak);
        bytes_escritos += serial_.write(&marcador_dir);
        bytes_escritos += serial_.write(&rpm_ref_dir_s);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialMotorsNode>());
    rclcpp::shutdown();
    return 0;
}
