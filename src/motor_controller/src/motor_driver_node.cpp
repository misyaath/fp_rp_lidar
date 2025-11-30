#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

class MotorDriverNode : public rclcpp::Node
{
public:
    MotorDriverNode() : Node("motor_driver_node")
    {
        RCLCPP_INFO(this->get_logger(), "Motor driver node started");

        // Open the serial port
        serial_port_ = open("/dev/ttyAMA1", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %d from opening serial port", serial_port_);
        } else {

            struct termios tty;
            if (tcgetattr(serial_port_, &tty) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            }

            tcgetattr(serial_port_, &tty);
            tty.c_lflag &= ~(ECHO | ICANON);  // Disable echo and canonical mode
            tcsetattr(serial_port_, TCSANOW, &tty);


            cfsetospeed(&tty, B115200);
            cfsetispeed(&tty, B115200);
            tty.c_cflag &= ~PARENB;        // No parity
            tty.c_cflag &= ~CSTOPB;        // 1 stop bit
            tty.c_cflag &= ~CSIZE;         // Clear data size bits
            tty.c_cflag |= CS8;            // 8 data bits
            tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
            tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

            if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
            }
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorDriverNode::cmd_vel_callback, this, std::placeholders::_1));
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
        double wheel_base = 0.1702;  // meters

        // Compute wheel velocities (m/s)
        double left_speed  = linear_x - (wheel_base / 2.0) * angular_z;
        double right_speed = linear_x + (wheel_base / 2.0) * angular_z;

        // Format serial commands with direction and speed
        std::string cmd_left  = "L" + std::string(1, (left_speed >= 0) ? 'F' : 'B') + std::to_string(fabs(left_speed)) + " ";
        std::string cmd_right = "R" + std::string(1, (right_speed >= 0) ? 'F' : 'B') + std::to_string(fabs(right_speed)) + " ";

        std::string payload = "CMD " + cmd_left + " " + cmd_right;

        uint8_t checksum = 0;
        for (char c : payload) {
            checksum ^= static_cast<uint8_t>(c);
        }

        std::string cmd = payload + " *" + std::to_string(checksum) + "\n";

        if (serial_port_ >= 0) {
            RCLCPP_INFO(this->get_logger(), "Sending speeds: %s", cmd.c_str());

            int bytes_sent_left = write(serial_port_, cmd.c_str(), cmd.length());
            if (bytes_sent_left == cmd.length()) {
                RCLCPP_INFO(this->get_logger(), "Left speed sent: %d bytes", bytes_sent_left);
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