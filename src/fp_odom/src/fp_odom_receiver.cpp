


#include <cstdio>
#include <regex>
#include <string>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"


class OdomReceiverNode : public rclcpp::Node {
public:
    OdomReceiverNode() : Node("odom_receiver_node") {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        serial_port_ = open("/dev/ttyAMA1", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_port_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        }

        struct termios tty;
        tcgetattr(serial_port_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        cfmakeraw(&tty);
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;
        tcsetattr(serial_port_, TCSANOW, &tty);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&OdomReceiverNode::read_serial, this));
    }

private:
    void read_serial() {
        char buffer[128];

        int bytes_read = read(serial_port_, buffer, sizeof(buffer));
//        RCLCPP_INFO(this->get_logger(), "Bytes read: %d", bytes_read);
        if (bytes_read < 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial read failed: errno=%d", errno);
            return;
        }

        if (bytes_read == 0) {
            RCLCPP_WARN(this->get_logger(), "No data received this cycle");
            return;
        }

        serial_buffer_.append(buffer, bytes_read);
        size_t new_line_pos;

        while((new_line_pos = serial_buffer_.find('\n')) != std::string::npos) {
          std::string line = serial_buffer_.substr(0, new_line_pos);
          serial_buffer_.erase(0, new_line_pos + 1);



          if(line.find("ODOM") != std::string::npos) {
//              RCLCPP_INFO(this->get_logger(), "Raw serial data: '%s'", line.c_str());
              parse_and_publish(line);
          }
        }

    }

    void parse_and_publish(const std::string& line) {
        std::regex pattern(R"(x:(\S+)\s+y:(\S+)\s+th:(\S+)\s+vx:(\S+)\s+wz:(\S+))");
        std::smatch match;
        if (std::regex_search(line, match, pattern) && match.size() == 6) {
            double x;
            double y;
            double theta;
            double vx;
            double wz;
           try {
                x = std::stod(match[1]);
                y = std::stod(match[2]);
                theta = std::stod(match[3]);
                vx = std::stod(match[4]);
                wz = std::stod(match[5]);
           } catch (const std::exception& e) {
               RCLCPP_WARN(this->get_logger(), "Failed to parse float: %s", e.what());
               return;
           }


            tf2::Quaternion q;
            q.setRPY(0, 0, theta);
            auto now = this->get_clock()->now();
            auto odom = nav_msgs::msg::Odometry();
            odom.header.stamp = now;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_footprint";

            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.orientation = tf2::toMsg(q);

            odom.twist.twist.linear.x = vx;
            odom.twist.twist.angular.z = wz;
            odom_pub_->publish(odom);

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id = "base_footprint";
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    int serial_port_;
    std::string serial_buffer_;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomReceiverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

