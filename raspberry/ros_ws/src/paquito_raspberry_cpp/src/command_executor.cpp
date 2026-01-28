// Autor: MAXO237
// Autor: blackzafiro
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <algorithm>

// --- LIBRERÍAS DE LINUX NECESARIAS ---
#include <unistd.h>       // Para write(), close()
#include <fcntl.h>        // Para open(), O_RDWR
#include <sys/ioctl.h>    // Para ioctl()
#include <linux/i2c-dev.h>// Para I2C_SLAVE
#include <cstdint>        // Para tipos int16_t, uint8_t
#include <string>         // Para std::string

// Definición de constantes
const double MAX_RAD_PER_SEC = 3.1416; 
const int MAX_PWM_VALUE = 255;

// Comandos básicos

enum Command {
  STOP =             0b00000000,
  BRAKE =            0b11001100,
  ACCELERATE =       0b00110011,
  FORWARD =          0b00001111,
  NE =               0b00001010, //right turn
  RIGHT =            0b01101001, //SE already exists
  SEAST =            0b10100000, //right back
  BACKWARD =         0b11110000,
  SW =               0b01000001, //left back
  LEFT =             0b10010110,
  NW =               0b00000101, //left turn
  CLOCKWISE =        0b01011010, //clockwise
  COUNTCLOCKWISE =   0b10100101, //countclockwise
  SPEAK =            0b00010001,
  MOVE_CAMERA =      0b00100010, //spin camera servo (1 param)
  SET_WHEELS_SPEED = 0b11111111, //manually set speeds per wheel (4 params)
};


class CommandExecutor : public rclcpp::Node
{
public:
    CommandExecutor() : Node("pwm_converter")
    {
        // -------------- INICIALIZAR I2C --------------------
        const char *busName = "/dev/i2c-1";
        
        // Abre el archivo del bus I2C
        file_i2c = open(busName, O_RDWR);
        if (file_i2c < 0) {
            RCLCPP_ERROR(this->get_logger(), "ERROR FATAL: No se pudo abrir el bus I2C %s", busName);
            // Nota: En un caso real, aquí deberíamos detener el nodo o lanzar excepción
        }

        // Configura la dirección del esclavo (0x08)
        if (ioctl(file_i2c, I2C_SLAVE, 0x08) < 0) {
            RCLCPP_ERROR(this->get_logger(), "ERROR FATAL: No se pudo conectar con el Arduino en 0x08");
        }

        // 1. Suscriptor
        _pwm_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_velocities", 10,
            std::bind(&CommandExecutor::pwm_callback, this, std::placeholders::_1));

        // 2. Suscriptor a comandos de cadena
        _command_subscription = this->create_subscription<std_msgs::msg::String>(
            "command_for_paquito", 10,
            std::bind(&CommandExecutor::string_command_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "PWM Converter I2C iniciado correctamente.");
    }

    // Destructor para cerrar el archivo I2C limpiamente
    ~CommandExecutor() {
        if (file_i2c >= 0) {
            close(file_i2c);
        }
    }

private:
    // Callback para un comando simple por cadena
    void string_command_callback(const std_msgs::msg::String & msg) const
    {
        std::string command = msg.data;
        RCLCPP_INFO(this->get_logger(), "Comando por cadena recibido: '%s'", command.c_str());

        // Comandos de 1 byte
        uint8_t buffer[1];
        if (command == "stop") {
            buffer[0] = STOP;
        }
        else if (command == "speak") {
            buffer[0] = SPEAK;
        }
        write(file_i2c, buffer, 1);
    }

    // Función de envío corregida (9 bytes)
    void enviar_4_pwm(int16_t p0, int16_t p1, int16_t p2, int16_t p3)
    {
        if (file_i2c < 0) return; // Seguridad si no se abrió el bus

        uint8_t buffer[9];
        buffer[0] = 0xFF;   // Identificador del comando para asignar pwms

        // Descomposición manual en bytes (Little Endian)
        // Motor 1 (FL)
        buffer[1] = p0 & 0xFF;         
        buffer[2] = (p0 >> 8) & 0xFF;  
        
        // Motor 2 (RL)
        buffer[3] = p1 & 0xFF;
        buffer[4] = (p1 >> 8) & 0xFF;

        // Motor 3 (FR)
        buffer[5] = p2 & 0xFF;
        buffer[6] = (p2 >> 8) & 0xFF;

        // Motor 4 (RR)
        buffer[7] = p3 & 0xFF;
        buffer[8] = (p3 >> 8) & 0xFF;

        // Enviamos los 9 bytes
        write(file_i2c, buffer, 9);
    }

    // Callback
    void pwm_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "Tamaño incorrecto de arreglo: %zu", msg->data.size());
            return;
        }

        int16_t pwm_values[4];

        for (size_t i = 0; i < 4; ++i) {
            double rad_per_sec = msg->data[i];

            // Limitamos magnitud, conservamos signo
            double val_clamped = std::clamp(rad_per_sec, -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);

            // Conversión lineal
            int pwm_calc = static_cast<int>( (val_clamped / MAX_RAD_PER_SEC) * MAX_PWM_VALUE );

            // Casting a int16_t
            pwm_values[i] = static_cast<int16_t>(pwm_calc);

        }

        // Debug en consola (opcional, puedes comentarlo si llena mucho la pantalla)
        // RCLCPP_INFO(this->get_logger(), "S: %d %d %d %d", pwm_values[0], pwm_values[1], pwm_values[2], pwm_values[3]);

        enviar_4_pwm(pwm_values[0], pwm_values[1], pwm_values[2], pwm_values[3]);
    }

    int file_i2c;  
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _pwm_subscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _command_subscription;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandExecutor>());
    rclcpp::shutdown();
    return 0;
}
