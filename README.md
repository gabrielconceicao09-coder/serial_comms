Pacote de interface serial para ROS2. 
Separado em 2 nós: 
serial_sensors_node, que recebe dados da IMU, GPS e Sonares, separados por vírgula na seguinte ordem:
sequência_imu, timestamp_imu, aceleração_x, aceleração_y, aceleração_z, vel_angular_x, vel_angular_y, vel_angular_z,
magnetômetro_x, magnetômetro_y, magnetômetro_z,
sequência_sonares, timestamp_sonares, sonar_1, sonar_2, sonar_3, sonar_4, sonar_5,
sequência_gps, timestamp_gps, latitude, longitude, altitude, número de satélites, tem fix

serial_motors_node, que recebe dados dos encoders:
sequência_esq, timestamp_esq, rpm_esq, sequência_dir, timestamp_dir, rpm_dir
e envia comandos de velocidade recebidos por mensagens do ros2.

Ambos nós compõem e publicam mensagens do ros2 com base nos dados recebidos e parâmetros configuráveis de variância e frame de referência.
o timestamp das mensagens é calculado com base no timestamp recebido por serial e no tempo atual do sistema, adequando o tempo marcado na leitura ao relógio do sistema
com um offset calculado na primeira leitura que é atualizado gradativamente com os offsets calculados em iterações seguintes através de um filtro passa-baixas digital.
