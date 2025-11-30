#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>

class MotorDriverNode : public rclcpp::Node
{
public:
    MotorDriverNode() : Node("motor_driver_node")
    {
        RCLCPP_INFO(this->get_logger(), "Motor driver node started");

        // Open the serial port
        serial_port_ = open("/dev/ttyAMA1", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        } else {
            struct termios tty;
            if (tcgetattr(serial_port_, &tty) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            }

            cfsetospeed(&tty, B115200);
            cfsetispeed(&tty, B115200);
            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tty.c_cflag &= ~CRTSCTS;
            tty.c_cflag |= CREAD | CLOCAL;
            tty.c_lflag &= ~(ECHO | ICANON);

            if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
            }
        }

        // Subscribe to Nav2 velocity commands
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&MotorDriverNode::cmd_vel_callback, this, std::placeholders::_1));
    }

    ~MotorDriverNode()
    {
        if (serial_port_ >= 0) {
            close(serial_port_);
        }
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;
        double wheel_base = 0.26;  // meters

        double left_speed  = linear_x - (wheel_base / 2.0) * angular_z;
        double right_speed = linear_x + (wheel_base / 2.0) * angular_z;

        std::string cmd_left  = "L" + std::string(1, (left_speed >= 0) ? 'F' : 'B') + std::to_string(std::fabs(left_speed));
        std::string cmd_right = "R" + std::string(1, (right_speed >= 0) ? 'F' : 'B') + std::to_string(std::fabs(right_speed));

        std::string payload = "CMD " + cmd_left + " " + cmd_right;

        uint8_t checksum = 0;
        for (char c : payload) {
            checksum ^= static_cast<uint8_t>(c);
        }

        std::string cmd = payload + " *" + std::to_string(checksum) + "\n";

        if (serial_port_ >= 0) {
            int bytes_sent = write(serial_port_, cmd.c_str(), cmd.length());
            if (bytes_sent == (int)cmd.length()) {
                RCLCPP_INFO(this->get_logger(), "Sent command: %s", cmd.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Partial write: %d/%zu bytes", bytes_sent, cmd.length());
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    int serial_port_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
