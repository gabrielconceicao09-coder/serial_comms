#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "serial/serial.h"

#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>

//Struct de alcance máx. e mín., posição e ângulo relativo dos sonares para cálculo da PointCloud2:
struct ConfigSonar{
    float x; //metros
    float y;
    float angulo_rel; //rad
    float max_alc; //metros
    float min_alc;
};

using namespace std::chrono_literals;

class SerialImuNode : public rclcpp::Node
{
    public:
    SerialImuNode() : Node("serial_imu_node") 
    {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_ = this->declare_parameter<int>("baudrate", 921600);

        imu_topic_ = this->declare_parameter<std::string>("imu_topic", "imu");
        imu_freq_ideal_ = this->declare_parameter<int>("imu_freq_ideal", 100);
        //gps_topic_ = this->declare_parameter<std::string>("gps_topic", "fix");
        //sonar_topic_ = this->declare_parameter<std::string>("sonar_topic", "sonar/pcl");

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
        //gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic_, rclcpp::SensorDataQoS());
        //sonar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(sonar_topic_, rclcpp::SensorDataQoS());

        /*//Structs para cada um dos sonares
        sonar1_config_ = {1.0, 1.0, 0.0, 2.0, 1.0};//this->declare_parameter<ConfigSonar>("sonar1_config", {1.0, 1.0, 0.0, 2.0, 1.0});
        sonar2_config_ = {1.0, 1.0, 0.0, 2.0, 1.0};//this->declare_parameter<ConfigSonar>("sonar2_config", {1.0, 1.0, 0.0, 2.0, 1.0});
        sonar3_config_ = {1.0, 1.0, 0.0, 2.0, 1.0};//this->declare_parameter<ConfigSonar>("sonar3_config", {1.0, 1.0, 0.0, 2.0, 1.0});
        sonar4_config_ = {1.0, 1.0, 0.0, 2.0, 1.0};//this->declare_parameter<ConfigSonar>("sonar4_config", {1.0, 1.0, 0.0, 2.0, 1.0});
        sonar5_config_ = {1.0, 1.0, 0.0, 2.0, 1.0};//this->declare_parameter<ConfigSonar>("sonar5_config", {1.0, 1.0, 0.0, 2.0, 1.0});

        sonares_.push_back(sonar1_config_);
        sonares_.push_back(sonar2_config_);
        sonares_.push_back(sonar3_config_);
        sonares_.push_back(sonar4_config_);
        sonares_.push_back(sonar5_config_);
        */
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
    //Parâmetros:
    std::string port_, imu_topic_;//, gps_topic_, sonar_topic_;
    int baudrate_, imu_freq_ideal_;
    //ConfigSonar sonar1_config_, sonar2_config_, sonar3_config_, sonar4_config_, sonar5_config_;
    //std::vector<ConfigSonar> sonares_;

    serial::Serial serial_;

    //Publishers:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    //rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    //Variáveis para ajuste temporal das mensagens
    bool primeira_leitura = true;
    int64_t offset_clocks_imu_ns;
    uint32_t ultimo_micros_esp_imu, ultima_seq_imu;

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
            RCLCPP_WARN(this->get_logger(), "Erro na conversão de valores para double (IMU): %s", item.c_str());
            return;
        }

        if (valores.size() != 9)
        {
            RCLCPP_INFO(this->get_logger(), "Leitura de número errado de valores. Linha: %s", line.c_str());
            return; 
        }
        //----------------------------------------------------------------------------
        
        //Tempo atual do sistema:
        rclcpp::Time tempo_sistema = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        //Composição da mensagem Imu:
        auto imuMsg = std::make_shared<sensor_msgs::msg::Imu>();
        imuMsg->header.frame_id = "imu_link";

        uint32_t sequencia = (uint32_t) valores[0];
        uint32_t micros_esp_imu = (uint32_t) valores[1];

        imuMsg->linear_acceleration.x = valores[2];
        imuMsg->linear_acceleration.y = valores[3];
        imuMsg->linear_acceleration.z = valores[4];
        imuMsg->linear_acceleration_covariance[0] = -1;
        imuMsg->angular_velocity.x = valores[5];
        imuMsg->angular_velocity.y = valores[6];
        imuMsg->angular_velocity.z = valores[7];
        imuMsg->angular_velocity_covariance[0] = -1;
        imuMsg->orientation.x = valores[8];
        imuMsg->orientation.y = valores[9];
        imuMsg->orientation.z = valores[10]; //compõe a mensagem
        imuMsg->orientation_covariance[0] = -1;
        
        //Composição do tempo de aquisição no relógio do sistema:
        if (primeira_leitura){
            offset_clocks_imu_ns = tempo_sistema.nanoseconds() - (int64_t)micros_esp_imu*1000LL;
            primeira_leitura = false;
        }

        int64_t offset_clocks_instantaneo_imu_ns = tempo_sistema.nanoseconds() - (int64_t)micros_esp_imu*1000LL;
        //Ajuste do offset para evitar drift em longos tempos de operação:
        offset_clocks_imu_ns += (offset_clocks_instantaneo_imu_ns-offset_clocks_imu_ns)/1000;

        imuMsg->header.stamp = rclcpp::Time((int64_t)micros_esp_imu*1000LL + offset_clocks_imu_ns);

        try {
            imu_pub_->publish(*imuMsg); //publica as medições do MPU
        }
        catch (...)
        {
            RCLCPP_WARN(this->get_logger(), "Mensagem IMU não publicada");
        }
        //-----------------------------------------------------------------
        
        /*//Composição da mensagem NavSatFix (GPS):
        auto gpsMsg = std::make_shared<sensor_msgs::msg::NavSatFix>();
        gpsMsg->header.stamp = tempo_atual;
        gpsMsg->header.frame_id = "gps_link";

        //Status do gps:
        auto gpsStatusMsg = std::make_shared<sensor_msgs::msg::NavSatStatus>();
        gpsStatusMsg->status = 0;//STATUS_FIX //Falta obter do gps e adicionar aqui o status real das mensagens de gps. (se referir a https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatStatus.html)
        gpsStatusMsg->service = 0;//SERVICE_GPS; //Define o serviço que o gps tá usando
        gpsMsg->status = gpsStatusMsg;

        gpsMsg->latitude = 1.0; //graus
        gpsMsg->longitude = 1.0; //graus
        gpsMsg->altitude = 1.0; //metro
        gpsMsg->position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Covariância das medidas do gps
        gpsMsg->position_covariance_type = 0;//COVARIANCE_TYPE_UNKNOWN;

        try {gps_pub_->publish(*gpsMsg);}
        catch (...) {RCLCPP_WARN(this->get_logger(), "Mensagem GPS não publicada");}
        //-----------------------------------------------------------------
        /*
        //Composição da mensagem PointCloud2 (Sonar):
        auto sonarPc2Msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        sonarPc2Msg->header.stamp = tempo_atual;
        sonarPc2Msg->header.frame_id = "base_link";

        //Modificador para alocação de memória e organização da mensagem PointCloud2
        sensor_msgs::PointCloud2Modifier modificador(*sonarPc2Msg); 
        modificador.setPointCloud2FieldsByString(1, "xyz");
        modificador.resize(sonares_.size());
        
        //Iteradores para preencher cada campo da PointCloud2:
        sensor_msgs::PointCloud2Iterator<float> iter_x(*sonarPc2Msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*sonarPc2Msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*sonarPc2Msg, "z");
        
        //Calcula a posição x e y do ponto a adicionar na PointCloud2 para as medidas de cada sonar
        for (size_t i; i < sonares_.size(); i++){
            float dist = 1.0 //recebe distância de cada sonar de acordo com índice i
            auto& config = sonares_[i];
            if (dist >= config.min_alc && dist <= config.max_alc) {
                *iter_x = config.x + dist*std::cos(config.angulo_rel);
                *iter_y = config.y + dist*std::sin(config.angulo_rel);
                *iter_z = 0.0;
            } else {
                *iter_x = std::numeric_limits::quiet_NaN();
                *iter_y = std::numeric_limits::quiet_NaN();
                *iter_z = std::numeric_limits::quiet_NaN();
            }

            iter_x ++; iter_y ++; iter_z ++;
        }
        try {sonar_pub_->publish(*sonarPc2Msg);}
        catch (...) {RCLCPP_WARN(this->get_logger(), "Mensagem sonares não publicada");}
        //--------------------------------------------------------------------------------
        */
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialImuNode>());
    rclcpp::shutdown();
    return 0;
}
