#include <cmath>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>  
#include "paquito.hpp"  

using std::placeholders::_1;
using namespace std::chrono_literals;

class CarVelocitySubscriber : public rclcpp::Node
{
public:
    CarVelocitySubscriber(rclcpp::NodeOptions options=rclcpp::NodeOptions()):Node("wheel_velocity_publisher", options)
    {
       // Se suscribe a comandos para asignar la velocidad al robot (suscripción a cmd_vel)
        _vel_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CarVelocitySubscriber::set_velocity, this, _1));

        // Publicador de velocidades de ruedas
        _wheel_vel_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "wheel_velocities", 10);

        RCLCPP_INFO(this->get_logger(), "Wheel velocity publisher started");
        _timer = this->create_wall_timer(33ms, std::bind(&CarVelocitySubscriber::publish, this));
    }

    // asigna las velocidades
    void set_velocity(const geometry_msgs::msg::Twist);
    // luego las publica
    void publish();

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _vel_subscription;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _wheel_vel_publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    const double WHEEL_R = 0.0375;

    // Velocidades angulares
    double w_fl = 0.;
    double w_rl = 0.;
    double w_fr = 0.;
    double w_rr = 0.;
};

// Cinemática inversa.  Convierte el comando de movimiento (cmd_vel) en velocidades de ruedas.
void CarVelocitySubscriber::set_velocity(const geometry_msgs::msg::Twist vel)
{
    double vx = vel.linear.x;
    double vy = vel.linear.y;
    double wz = vel.angular.z;
    // WHEEL_X: Distancia en x entre las llantas (definido en paquito.hpp)
    // WHEEL_Y: Distancia en y entre las llantas (definido en paquito.hpp)
    double c = WHEEL_X + WHEEL_Y;

    w_fl = (vx - vy - c * wz) / WHEEL_R;
    w_rl = (vx + vy - c * wz) / WHEEL_R;
    w_fr = (vx + vy + c * wz) / WHEEL_R;
    w_rr = (vx - vy + c * wz) / WHEEL_R;
}

    // Publica las cuatro velocidades en el siguiente orden:
    //  front_left_wheel
    //  rear_left_wheel
    //  front_right_wheel
    //  rear_right_wheel
void CarVelocitySubscriber::publish()
{
    std_msgs::msg::Float64MultiArray vel_array;
    vel_array.data = {w_fl, w_rl, w_fr, w_rr};  
    _wheel_vel_publisher->publish(vel_array);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarVelocitySubscriber>());
    rclcpp::shutdown();
    return 0;
}

// NOTAS:
// En el código original (move_node.cpp): 
// 	- set_velocity() -> aplica cinemática inversa (a partir de una velocidad lineal y angular calcula la velocidad de cada rueda)
// 	- publish() -> aplica cinemática directa (movimiento del robot en el mundo a partir de las velocidades de cada rueda)
// 	- publish_wheel_transform() -> publica el marco de referencia de cada rueda respecto al chasis 
