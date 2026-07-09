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

//Recebe dados da IMU, do GPS e dos sonares, obtidos da ESP32. Publica os dados compostos em mensagens nos formatos ROS2

using namespace std::chrono_literals;

class SerialSensorsNode : public rclcpp::Node
{
    public:
    SerialSensorsNode() : Node("serial_sensors_node") 
    {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        baudrate_ = this->declare_parameter<int>("baudrate", 921600);

        variancia_acl_ = this->declare_parameter<double>("variancia_acl", 200*std::pow(0.0016541342577078063, 2.0));
        variancia_gir_ = this->declare_parameter<double>("variancia_gir", 200*std::pow(0.00011503473194962085,2));
        variancia_mag_ = this->declare_parameter<double>("variancia_mag", 0.1);
        variancia_gps_long_ = this->declare_parameter<double>("variancia_gps_long", 1.0);
        variancia_gps_lat_ = this->declare_parameter<double>("variancia_gps_lat", 1.0);
        variancia_gps_alt_ = this->declare_parameter<double>("variancia_gps_alt", 1.0);
        variancia_sonar_ = this->declare_parameter<double>("variancia_sonares", 1.0);
        fov_sonar_ = this->declare_parameter<float>("fov_sonar", 0.2618); //Rad. 0.2618 rad = 15 graus 
        min_range_sonar_ = this->declare_parameter<float>("min_range_sonar", 0.005); //m
        max_range_sonar_ = this->declare_parameter<float>("max_range_sonar", 2.0); //m
        //offset_kalibr_s_ = this->declare_parameter<double>("offset_kalibr_s", -0.022073607573335652);
        //offset_kalibr = (int64_t) offset_kalibr_s_*1e9;


        raw_imu_topic_ = this->declare_parameter<std::string>("raw_imu_topic", "imu/data_raw");
        raw_mag_topic_ = this->declare_parameter<std::string>("raw_mag_topic", "imu/mag");
        gps_topic_ = this->declare_parameter<std::string>("gps_topic", "gps/fix_raw");
        sonar_topic_ = this->declare_parameter<std::string>("sonar_topic", "sonar/");
        
        imu_frame_ = this->declare_parameter<std::string>("imu_frame", "imu_link");
        gps_frame_ = this->declare_parameter<std::string>("gps_frame", "gps_link");
        sonar1_frame_ = this->declare_parameter<std::string>("sonar1_frame", "sonar1_link");
        sonar2_frame_ = this->declare_parameter<std::string>("sonar2_frame", "sonar2_link");
        sonar3_frame_ = this->declare_parameter<std::string>("sonar3_frame", "sonar3_link");
        sonar4_frame_ = this->declare_parameter<std::string>("sonar4_frame", "sonar4_link");
        sonar5_frame_ = this->declare_parameter<std::string>("sonar5_frame", "sonar5_link");

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(raw_imu_topic_, rclcpp::SensorDataQoS());
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(raw_mag_topic_, rclcpp::SensorDataQoS());
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic_, rclcpp::SensorDataQoS());
        sonar_pub_ = this->create_publisher<sensor_msgs::msg::Range>(sonar_topic_, rclcpp::SensorDataQoS());
        
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
        
        

        timer_ = this->create_wall_timer(4ms, std::bind(&SerialSensorsNode::ReadPub_callback, this));
        RCLCPP_INFO(this->get_logger(), "Timer criado com callback");        
    }

    ~SerialSensorsNode()
    {

    }

    private:
    //Parâmetros:
    std::string port_, raw_imu_topic_, raw_mag_topic_, gps_topic_, sonar_topic_;
    int baudrate_;
    double variancia_acl_, variancia_gir_, variancia_mag_;
    double variancia_gps_long_, variancia_gps_lat_, variancia_gps_alt_;
    double variancia_sonar_;
    float fov_sonar_, min_range_sonar_, max_range_sonar_;
    
    std::string imu_frame_, gps_frame_;
    std::string sonar1_frame_, sonar2_frame_, sonar3_frame_, sonar4_frame_, sonar5_frame_;


    serial::Serial serial_;

    //Publishers:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    //Variáveis para ajuste temporal das mensagens
    bool primeira_leitura = true;
    int64_t offset_clocks_ns;
    double offset_alpha_ = 0.01;
    //int64_t ultimo_timestamp_imu_ns = 0;
    //double offset_kalibr_s_;
    //int64_t offset_kalibr;
    //uint32_t ultimo_micros_esp_imu;
    uint32_t ultima_seq_imu = 0;
    uint32_t ultima_seq_gps = 0;
    uint32_t ultima_seq_sonar = 0;


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

        if (valores.size() != 25) //TODO: CORRIGIR PARA NÚMERO CERTO DE VALORES
        {
            RCLCPP_INFO(this->get_logger(), "Leitura de número errado de valores. Linha: %s, n de valores: %li", line.c_str(), valores.size());
            return; 
        }
        //----------------------------------------------------------------------------
        
        //Tempo atual do sistema:
        rclcpp::Time tempo_ros = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        //Composição das mensagens de dados não filtrados do MPU:

        uint32_t sequencia_imu = (uint32_t) valores[0];
        if (sequencia_imu>ultima_seq_imu){
            auto rawImuMsg = std::make_shared<sensor_msgs::msg::Imu>();
            rawImuMsg->header.frame_id = "imu_link";

            int64_t micros_esp_imu = (int64_t) valores[1];
            ultima_seq_imu = sequencia_imu;

            rawImuMsg->linear_acceleration.x = valores[2];
            rawImuMsg->linear_acceleration.y = valores[3];
            rawImuMsg->linear_acceleration.z = valores[4];
            rawImuMsg->linear_acceleration_covariance[0] = variancia_acl_;
            rawImuMsg->linear_acceleration_covariance[1] = 0.0;
            rawImuMsg->linear_acceleration_covariance[2] = 0.0;
            rawImuMsg->linear_acceleration_covariance[3] = 0.0;
            rawImuMsg->linear_acceleration_covariance[4] = variancia_acl_;
            rawImuMsg->linear_acceleration_covariance[5] = 0.0;
            rawImuMsg->linear_acceleration_covariance[6] = 0.0;
            rawImuMsg->linear_acceleration_covariance[7] = 0.0;
            rawImuMsg->linear_acceleration_covariance[8] = variancia_acl_;
            rawImuMsg->angular_velocity.x = valores[5];
            rawImuMsg->angular_velocity.y = valores[6];
            rawImuMsg->angular_velocity.z = valores[7];
            rawImuMsg->angular_velocity_covariance[0] = variancia_gir_;
            rawImuMsg->angular_velocity_covariance[1] = 0.0;
            rawImuMsg->angular_velocity_covariance[2] = 0.0;
            rawImuMsg->angular_velocity_covariance[3] = 0.0;
            rawImuMsg->angular_velocity_covariance[4] = variancia_gir_;
            rawImuMsg->angular_velocity_covariance[5] = 0.0;
            rawImuMsg->angular_velocity_covariance[6] = 0.0;
            rawImuMsg->angular_velocity_covariance[7] = 0.0;
            rawImuMsg->angular_velocity_covariance[8] = variancia_gir_;

            auto rawMagMsg = std::make_shared<sensor_msgs::msg::MagneticField>();
            rawMagMsg->header.frame_id = "imu_link";
            
            rawMagMsg->magnetic_field.x = valores[8];
            rawMagMsg->magnetic_field.y = valores[9];
            rawMagMsg->magnetic_field.z = valores[10];
            rawMagMsg->magnetic_field_covariance[0] = variancia_mag_;
            rawMagMsg->magnetic_field_covariance[1] = 0.0;
            rawMagMsg->magnetic_field_covariance[2] = 0.0;
            rawMagMsg->magnetic_field_covariance[3] = 0.0;
            rawMagMsg->magnetic_field_covariance[4] = variancia_mag_;
            rawMagMsg->magnetic_field_covariance[5] = 0.0;
            rawMagMsg->magnetic_field_covariance[6] = 0.0;
            rawMagMsg->magnetic_field_covariance[7] = 0.0;
            rawMagMsg->magnetic_field_covariance[8] = variancia_mag_;
            
            //Composição do tempo de aquisição no relógio do sistema:
            if (primeira_leitura){
                offset_clocks_ns = tempo_ros.nanoseconds() - micros_esp_imu*1000LL;
                primeira_leitura = false;
            }
            else{
                //Ajuste contínuo do offset:
                offset_clocks_ns = (int64_t) offset_clocks_ns*(1-offset_alpha_) + (tempo_ros.nanoseconds() - micros_esp_imu*1000LL)*offset_alpha_;
            }

            int64_t timestamp_imu_ns = micros_esp_imu*1000LL + offset_clocks_ns; //+ offset_kalibr;

            //RCLCPP_INFO(this->get_logger(), "Dt: %li", timestamp_imu_ns-ultimo_timestamp_imu_ns);

            /*//Tenta garantir monotonicidade dos timestamps:
            if (timestamp_imu_ns <= ultimo_timestamp_imu_ns){
                RCLCPP_WARN(this->get_logger(), "Timestamps invertidos, somando 1 ns para tentar garantir monotonicidade. dt: %li", timestamp_imu_ns-ultimo_timestamp_imu_ns);
                timestamp_imu_ns = ultimo_timestamp_imu_ns + 1;
            }
            ultimo_timestamp_imu_ns = timestamp_imu_ns;
            */
            

            rawImuMsg->header.stamp = rclcpp::Time(timestamp_imu_ns);
            rawMagMsg->header.stamp = rclcpp::Time(timestamp_imu_ns);

            try {
                imu_pub_->publish(*rawImuMsg);
                mag_pub_->publish(*rawMagMsg);
            }
            catch (...)
            {
                RCLCPP_WARN(this->get_logger(), "Mensagens IMU não publicadas");
            }
        }
        //-----------------------------------------------------------------
        
        //Composição da mensagem NavSatFix (GPS):
        uint32_t sequencia_gps = valores[11];

        if (sequencia_gps>ultima_seq_gps){
            ultima_seq_gps = sequencia_gps;

            auto gpsMsg = std::make_shared<sensor_msgs::msg::NavSatFix>();
            gpsMsg->header.frame_id = gps_frame_;

            int fix = (int) valores[17];
            gpsMsg->status.status = fix ? 0 : -1; //STATUS_FIX: 0 se tem posição não _augmented_ e -1 se não tem posição
            gpsMsg->status.service = 1;//SERVICE_GPS: Define o serviço que o gps tá usando. 1: GPS

            if (gpsMsg->status.status>=0){ //Checa se o GPS reportou que conseguiu localizar onde está
                gpsMsg->latitude = valores[13]; //graus
                gpsMsg->longitude = valores[14]; //graus
                gpsMsg->altitude = valores[15]; //metro
                gpsMsg->position_covariance = {2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0}; //Covariância das medidas do gps
                gpsMsg->position_covariance_type = 1; //COVARIANCE_TYPE_APPROXIMATED;

                int64_t micros_esp_gps = (int64_t) valores[12]; //TODO: Quando tiver parsing pronto, colocar índice correto.
                int64_t timestamp_gps_ns = micros_esp_gps*1000LL + offset_clocks_ns;
                gpsMsg->header.stamp = rclcpp::Time(timestamp_gps_ns);

                try {gps_pub_->publish(*gpsMsg);}
                catch (...) {RCLCPP_WARN(this->get_logger(), "Mensagem GPS não publicada");}
            }
            else RCLCPP_INFO(this->get_logger(), "Mensagem GPS: STATUS_NO_FIX, mensagem não publicada");
        }

        //-----------------------------------------------------------------
        
        //Composição das mensagens sonares (Range):
        uint32_t sequencia_sonar = valores[18];
        if (sequencia_sonar>ultima_seq_sonar){
            ultima_seq_sonar = sequencia_sonar;

            auto sonarMsg1 = std::make_shared<sensor_msgs::msg::Range>();
            auto sonarMsg2 = std::make_shared<sensor_msgs::msg::Range>();
            auto sonarMsg3 = std::make_shared<sensor_msgs::msg::Range>();
            auto sonarMsg4 = std::make_shared<sensor_msgs::msg::Range>();
            auto sonarMsg5 = std::make_shared<sensor_msgs::msg::Range>();

            sonarMsg1->radiation_type = 0; //0: ULTRASOUND
            sonarMsg2->radiation_type = 0; 
            sonarMsg3->radiation_type = 0; 
            sonarMsg4->radiation_type = 0; 
            sonarMsg5->radiation_type = 0; 

            sonarMsg1->field_of_view = fov_sonar_;
            sonarMsg2->field_of_view = fov_sonar_;
            sonarMsg3->field_of_view = fov_sonar_;
            sonarMsg4->field_of_view = fov_sonar_;
            sonarMsg5->field_of_view = fov_sonar_;

            sonarMsg1->min_range = min_range_sonar_;
            sonarMsg2->min_range = min_range_sonar_;
            sonarMsg3->min_range = min_range_sonar_;
            sonarMsg4->min_range = min_range_sonar_;
            sonarMsg5->min_range = min_range_sonar_;

            sonarMsg1->max_range = max_range_sonar_;
            sonarMsg2->max_range = max_range_sonar_;
            sonarMsg3->max_range = max_range_sonar_;
            sonarMsg4->max_range = max_range_sonar_;
            sonarMsg5->max_range = max_range_sonar_;

            float range1 = (float) valores[20];
            if (range1<min_range_sonar_){
                range1 = -std::numeric_limits<float>::infinity();
            } else if (range1>max_range_sonar_){
                range1 = std::numeric_limits<float>::infinity();
            } else {
                sonarMsg1->range = range1;
            }

            float range2 = (float) valores[21];
            if (range2<min_range_sonar_){
                range2 = -std::numeric_limits<float>::infinity();
            } else if (range1>max_range_sonar_){
                range2 = std::numeric_limits<float>::infinity();
            } else {
                sonarMsg2->range = range2;
            }

            float range3 = (float) valores[22];
            if (range3<min_range_sonar_){
                range3 = -std::numeric_limits<float>::infinity();
            } else if (range1>max_range_sonar_){
                range3 = std::numeric_limits<float>::infinity();
            } else {
                sonarMsg3->range = range3;
            }

            float range4 = (float) valores[23];
            if (range4<min_range_sonar_){
                range4 = -std::numeric_limits<float>::infinity();
            } else if (range1>max_range_sonar_){
                range4 = std::numeric_limits<float>::infinity();
            } else {
                sonarMsg4->range = range4;
            }

            float range5 = (float) valores[24];
            if (range5<min_range_sonar_){
                range5 = -std::numeric_limits<float>::infinity();
            } else if (range5>max_range_sonar_){
                range5 = std::numeric_limits<float>::infinity();
            } else {
                sonarMsg5->range = range5;
            }

            int64_t micros_esp_sonares = (int64_t) valores[19];
            int64_t timestamp_sonares_ns = micros_esp_sonares*1000LL + offset_clocks_ns;

            sonarMsg1->header.stamp = rclcpp::Time(timestamp_sonares_ns);
            sonarMsg2->header.stamp = rclcpp::Time(timestamp_sonares_ns);
            sonarMsg3->header.stamp = rclcpp::Time(timestamp_sonares_ns);
            sonarMsg4->header.stamp = rclcpp::Time(timestamp_sonares_ns);
            sonarMsg5->header.stamp = rclcpp::Time(timestamp_sonares_ns);
            
            sonarMsg1->header.frame_id = sonar1_frame_;
            sonarMsg2->header.frame_id = sonar2_frame_;
            sonarMsg3->header.frame_id = sonar3_frame_;
            sonarMsg4->header.frame_id = sonar4_frame_;
            sonarMsg5->header.frame_id = sonar5_frame_;

            try {
                sonar_pub_->publish(*sonarMsg1);
                sonar_pub_->publish(*sonarMsg2);
                sonar_pub_->publish(*sonarMsg3);
                sonar_pub_->publish(*sonarMsg4);
                sonar_pub_->publish(*sonarMsg5);
            }
            catch (...) {RCLCPP_WARN(this->get_logger(), "Mensagens sonares não publicadas");}
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialSensorsNode>());
    rclcpp::shutdown();
    return 0;
}
