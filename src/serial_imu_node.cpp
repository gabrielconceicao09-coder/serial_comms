#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
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
        variancia_acl_ = this->declare_parameter<double>("variancia_acl", 200*std::pow(0.0016541342577078063, 2.0));
        variancia_gir_ = this->declare_parameter<double>("variancia_gir", 200*std::pow(0.00011503473194962085,2));
        variancia_mag_ = this->declare_parameter<double>("variancia_mag", 0.1);
        variancia_gps_long_ = this->declare_parameter<double>("variancia_gps_long", 1.0);
        variancia_gps_lat_ = this->declare_parameter<double>("variancia_gps_lat", 1.0);
        variancia_gps_alt_ = this->declare_parameter<double>("variancia_gps_alt", 1.0);
        raio_roda_esq_ = this->declare_parameter<double>("raio_roda_esq", 0.125); //em metros
        raio_roda_dir_ = this->declare_parameter<double>("raio_roda_dir", 0.125); //em metros 
        //offset_kalibr_s_ = this->declare_parameter<double>("offset_kalibr_s", -0.022073607573335652);
        //offset_kalibr = (int64_t) offset_kalibr_s_*1e9;


        raw_imu_topic_ = this->declare_parameter<std::string>("raw_imu_topic", "imu/data_raw");
        raw_mag_topic_ = this->declare_parameter<std::string>("raw_mag_topic", "imu/mag");
        gps_topic_ = this->declare_parameter<std::string>("gps_topic", "gps/fix_raw");
        sonar_topic_ = this->declare_parameter<std::string>("sonar_topic", "sonar/pcl");
        encoders_topic_ = this->declare_parameter<std::string>("encoders_topic", "encoders/twist_raw");

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(raw_imu_topic_, rclcpp::SensorDataQoS());
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(raw_mag_topic_, rclcpp::SensorDataQoS());
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic_, rclcpp::SensorDataQoS());
        sonar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(sonar_topic_, rclcpp::SensorDataQoS());
        encoders_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(encoders_topic_, rclcpp::SensorDataQoS());

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
        
        

        timer_ = this->create_wall_timer(4ms, std::bind(&SerialImuNode::ReadPub_callback, this));
        RCLCPP_INFO(this->get_logger(), "Timer criado com callback");        
    }

    ~SerialImuNode()
    {

    }

    private:
    //Parâmetros:
    std::string port_, raw_imu_topic_, raw_mag_topic_, gps_topic_, sonar_topic_;
    int baudrate_;
    double variancia_acl_, variancia_gir_, variancia_mag_;
    double variancia_gps_long_, variancia_gps_lat_, variancia_gps_alt_;
    double raio_roda_esq_, raio_roda_dir_;
    //ConfigSonar sonar1_config_, sonar2_config_, sonar3_config_, sonar4_config_, sonar5_config_;
    //std::vector<ConfigSonar> sonares_;

    serial::Serial serial_;

    //Publishers:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr encoders_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    //Variáveis para ajuste temporal das mensagens
    bool primeira_leitura = true;
    int64_t offset_clocks_ns;
    double offset_alpha_ = 0.01;
    //int64_t ultimo_timestamp_imu_ns = 0;
    //double offset_kalibr_s_;
    //int64_t offset_kalibr;
    //uint32_t ultimo_micros_esp_imu, ultima_seq_imu;


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

        if (valores.size() != 11) //TODO: CORRIGIR PARA NÚMERO CERTO DE VALORES
        {
            RCLCPP_INFO(this->get_logger(), "Leitura de número errado de valores. Linha: %s, n de valores: %i", line.c_str(), valores.size());
            return; 
        }
        //----------------------------------------------------------------------------
        
        //Tempo atual do sistema:
        rclcpp::Time tempo_ros = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        //Composição das mensagens de dados não filtrados do MPU:
        auto rawImuMsg = std::make_shared<sensor_msgs::msg::Imu>();
        rawImuMsg->header.frame_id = "imu_link";

        uint32_t sequencia = (uint32_t) valores[0];
        int64_t micros_esp_imu = (int64_t) valores[1];

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
        //-----------------------------------------------------------------
        
        //Composição da mensagem NavSatFix (GPS):
        auto gpsMsg = std::make_shared<sensor_msgs::msg::NavSatFix>();
        gpsMsg->header.frame_id = "gps_link";

        gpsMsg->status.status = 0;//STATUS_FIX //Falta obter do gps e adicionar aqui o status real das mensagens de gps. (se referir a https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatStatus.html)
        gpsMsg->status.service = 0;//SERVICE_GPS; //Define o serviço que o gps tá usando
        if (gpsMsg->status.status>=0){ //Checa se o GPS reportou que conseguiu localizar onde está
            gpsMsg->latitude = 1.0; //graus
            gpsMsg->longitude = 1.0; //graus
            gpsMsg->altitude = 1.0; //metro
            gpsMsg->position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //Covariância das medidas do gps
            gpsMsg->position_covariance_type = 1; //COVARIANCE_TYPE_APPROXIMATED;

            int64_t micros_esp_gps = (int64_t) valore[] //TODO: Quando tiver parsing pronto, colocar índice correto.
            int64_t timestamp_gps_ns = micros_esp_gps*1000LL + offset_clocks_ns;
            gpsMsg->header.stamp = rclcpp::Time(timestamp_gps_ns);

            try {gps_pub_->publish(*gpsMsg);}
            catch (...) {RCLCPP_WARN(this->get_logger(), "Mensagem GPS não publicada");}
        }
        else RCLCPP_INFO(this->get_logger(), "Mensagem GPS: STATUS_NO_FIX, mensagem não publicada");
        //-----------------------------------------------------------------
        /*
        //Composição da mensagem PointCloud2 (Sonar):
        auto sonarPc2Msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

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

        int64_t micros_esp_sonares = (int64_t) valores[] //TODO: Quando tiver parsing pronto, colocar índice correto.
        int64_t timestamp_sonares_ns = micros_esp_sonares*1000LL + offset_clocks_ns;
        sonarPc2Msg->header.stamp = rclcpp::Time(timestamp_sonares_ns);

        try {sonar_pub_->publish(*sonarPc2Msg);}
        catch (...) {RCLCPP_WARN(this->get_logger(), "Mensagem sonares não publicada");}
        //--------------------------------------------------------------------------------
        */
        /*
        //Composição mensagem Twist dos encoders:

        auto encodersMsg = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();

        encoderMsg->header.frame_id = "base_link";

        encoderMsg->twist.twist.angular.x =;//TODO: inserir os valores lidos
        encoderMsg->twist.twist.angular.y =;
        encoderMsg->twist.twist.angular.z =;
        encoderMsg->twist.twist.linear.x =;
        encoderMsg->twist.twist.linear.y =;
        encoderMsg->twist.twist.linear.z =;
        encoderMsg->twist.covariance[0] =; //TODO: Variância dos encoders (acho que deve ser alta) 
        encoderMsg->twist.covariance[1] =; encoderMsg->twist.covariance[2] =; encoderMsg->twist.covariance[3] =; 
        encoderMsg->twist.covariance[4] =; encoderMsg->twist.covariance[5] =; encoderMsg->twist.covariance[6] =; 
        encoderMsg->twist.covariance[7] =; 
        encoderMsg->twist.covariance[8] =; encoderMsg->twist.covariance[9] =; encoderMsg->twist.covariance[10] =;
        encoderMsg->twist.covariance[11] =; encoderMsg->twist.covariance[12] =; encoderMsg->twist.covariance[13] =;
        encoderMsg->twist.covariance[14] =;
        encoderMsg->twist.covariance[15] =; encoderMsg->twist.covariance[16] =; encoderMsg->twist.covariance[17] =;
        encoderMsg->twist.covariance[18] =; encoderMsg->twist.covariance[19] =; encoderMsg->twist.covariance[20] =;
        encoderMsg->twist.covariance[21] =;
        encoderMsg->twist.covariance[22] =; encoderMsg->twist.covariance[23] =; encoderMsg->twist.covariance[24] =;
        encoderMsg->twist.covariance[25] =; encoderMsg->twist.covariance[26] =; encoderMsg->twist.covariance[27] =;
        encoderMsg->twist.covariance[28] =;
        encoderMsg->twist.covariance[29] =; encoderMsg->twist.covariance[30] =; encoderMsg->twist.covariance[31] =;
        encoderMsg->twist.covariance[32] =; encoderMsg->twist.covariance[33] =; encoderMsg->twist.covariance[34] =;
        encoderMsg->twist.covariance[35] =;

        int64_t micros_esp_encoders = (int64_t) valores[] //TODO: Quando tiver parsing pronto, colocar índice correto.
        int64_t timestamp_encoders_ns = micros_esp_encoders*1000LL + offset_clocks_ns;
        encodersMsg->header.stamp = rclcpp::Time(timestamp_encoders_ns);

        try{
            encoder_pub_->publish(*encoderMsg);
        } catch (...) RCLCPP_WARN(this->get_logger(), "Mensagem encoders não publicada");
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
