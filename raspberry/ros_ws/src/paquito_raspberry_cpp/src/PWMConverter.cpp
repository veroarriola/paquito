// Autor: MAXO237
// Autor: blackzafiro
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>
#include <algorithm>

// --- LIBRERÍAS DE LINUX NECESARIAS ---
#include <unistd.h>       // Para write(), close()
#include <fcntl.h>        // Para open(), O_RDWR
#include <sys/ioctl.h>    // Para ioctl()
#include <linux/i2c-dev.h>// Para I2C_SLAVE
#include <cstdint>        // Para tipos int16_t, uint8_t

// Definición de constantes
const double MAX_RAD_PER_SEC = 3.1416; 
const int MAX_PWM_VALUE = 255;

class PWMConverter : public rclcpp::Node
{
public:
    PWMConverter() : Node("pwm_converter")
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
        _vel_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_velocities", 10,
            std::bind(&PWMConverter::velocity_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "PWM Converter I2C iniciado correctamente.");
    }

    // Destructor para cerrar el archivo I2C limpiamente
    ~PWMConverter() {
        if (file_i2c >= 0) {
            close(file_i2c);
        }
    }

private:

    // Función de envío corregida (8 bytes)
    void enviar_4_pwm(int16_t p0, int16_t p1, int16_t p2, int16_t p3) {
        if (file_i2c < 0) return; // Seguridad si no se abrió el bus

        uint8_t buffer[8];

        // Descomposición manual en bytes (Little Endian)
        // Motor 1 (FL)
        buffer[0] = p0 & 0xFF;         
        buffer[1] = (p0 >> 8) & 0xFF;  
        
        // Motor 2 (RL)
        buffer[2] = p1 & 0xFF;
        buffer[3] = (p1 >> 8) & 0xFF;

        // Motor 3 (FR)
        buffer[4] = p2 & 0xFF;
        buffer[5] = (p2 >> 8) & 0xFF;

        // Motor 4 (RR)
        buffer[6] = p3 & 0xFF;
        buffer[7] = (p3 >> 8) & 0xFF;

        // Enviamos los 8 bytes
        write(file_i2c, buffer, 8);
    }

    // Callback
    void velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _vel_subscription;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PWMConverter>());
    rclcpp::shutdown();
    return 0;
}
