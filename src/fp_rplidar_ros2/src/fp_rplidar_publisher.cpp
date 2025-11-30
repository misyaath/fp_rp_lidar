#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <vector>
#include <string>


#include "sl_lidar.h"
#include "sl_lidar_driver.h"


class RplidarPublisher: public rclcpp::Node
{
  public:
    RplidarPublisher():Node("rplidar_publisher"),
                         is_lidar_connected_(false),
                         lidar_driver_(nullptr),
                         channel(nullptr)
    {
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("serial_baudrate", 460800);
        this->declare_parameter("frame_id", "laser_frame");

        serial_port_ = this->get_parameter("serial_port").as_string();
        serial_baudrate_ = this->get_parameter("serial_baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

       publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        if (!connectLidar()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to RPLIDAR. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RplidarPublisher::publishScanData, this));
    }
    ~RplidarPublisher()
    {
        if (lidar_driver_) {
            lidar_driver_->stop();
            delete lidar_driver_;
            lidar_driver_ = nullptr;
        }

        if(channel) {
          channel->close();
          channel = nullptr;
        }
    }

    private:

        bool cleanupResourcesAndReturnFalse()
        {
            if (lidar_driver_) {
                delete lidar_driver_;
                lidar_driver_ = nullptr;
            }
            if (channel) {
                channel->close();
                channel = nullptr;
            }
            is_lidar_connected_ = false;
            return false;
        }

      bool connectLidar(){
          auto driverResult = sl::createLidarDriver();

          if (!driverResult) {
              RCLCPP_ERROR(this->get_logger(), "Failed to create a lidar driver instance.");
              return false;
          }

          lidar_driver_ = *driverResult;

          auto channelResult = sl::createSerialPortChannel(serial_port_, serial_baudrate_);


          if (!channelResult) {
              RCLCPP_ERROR(this->get_logger(), "Failed to connect to the lidar on port %s", serial_port_.c_str());
             return cleanupResourcesAndReturnFalse();
          }
          channel = *channelResult;
          auto res  = lidar_driver_->connect(channel);

          sl_lidar_response_device_info_t deviceInfo;
          res = lidar_driver_->getDeviceInfo(deviceInfo);
          if (SL_IS_FAIL(res)) {
              RCLCPP_ERROR(this->get_logger(), "Failed to get lidar device info.");
              return cleanupResourcesAndReturnFalse();
          }

          printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
            deviceInfo.model,
            deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
            deviceInfo.hardware_version);

         sl_lidar_response_device_health_t  healthinfo;
          res = lidar_driver_->getHealth(healthinfo);
          if (SL_IS_FAIL(res)) {
              RCLCPP_ERROR(this->get_logger(), "Failed to get lidar health info.");
              return cleanupResourcesAndReturnFalse();
          }

          printf("Lidar health status : ");
          switch (healthinfo.status)
          {
              case SL_LIDAR_STATUS_OK:
                  printf("OK.");
              break;
              case SL_LIDAR_STATUS_WARNING:
                  printf("Warning.");
              break;
              case SL_LIDAR_STATUS_ERROR:
                  printf("Error.");
              break;
          }
          printf(" (errorcode: %d)\n", healthinfo.error_code);


          lidar_driver_->setMotorSpeed();
          std::vector<sl::LidarScanMode> scanModes;
          lidar_driver_->getAllSupportedScanModes(scanModes);
          lidar_driver_->startScanExpress(false, scanModes[0].id);
          is_lidar_connected_ = true;
          RCLCPP_INFO(this->get_logger(), "Successfully connected to RPLIDAR on port %s", serial_port_.c_str());
          return true;
      }

      void publishScanData(){
          if (!is_lidar_connected_) {
              return;
          }

          sl_lidar_response_measurement_node_hq_t nodes[8192];
          size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);

          sl_result res = lidar_driver_->grabScanDataHq(nodes, nodeCount);

          if (SL_IS_FAIL(res)) {
              RCLCPP_WARN(this->get_logger(), "Failed to grab scan data.");
              return;
          }

          auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

          scan_msg->header.stamp = this->now();
          scan_msg->header.frame_id = frame_id_;

          // Lidar parameters (these are typical for an RPLIDAR A2M8)
          // You may need to adjust these based on your specific lidar model.
          // The values here represent a 360-degree scan with a 10 Hz frequency.
          scan_msg->angle_min = -M_PI; // Start angle in radians
          scan_msg->angle_max = M_PI;  // End angle in radians
          scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / nodeCount;
          scan_msg->time_increment = 0.1 / nodeCount; // Time between measurements
          scan_msg->scan_time = 0.1; // Time for a full scan (10 Hz = 0.1 seconds)
          scan_msg->range_min = 0.15; // Minimum range in meters
          scan_msg->range_max = 12.0;  // Maximum range in meters

          // Resize the ranges and intensities vectors
          scan_msg->ranges.resize(nodeCount);
          scan_msg->intensities.resize(nodeCount);

          for (size_t i = 0; i < nodeCount; ++i) {
              // Check if the data point is valid
              if (nodes[i].dist_mm_q2 != 0) {
                  // Convert distance from mm with a 1/4 precision to meters
                  scan_msg->ranges[i] = nodes[i].dist_mm_q2 / 4.0f / 1000.0f;
              } else {
                  // Set invalid ranges to infinity
                  scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
              }
              // Populate intensity data if available
              scan_msg->intensities[i] = nodes[i].quality;
          }

          // Publish the LaserScan message
          publisher_->publish(std::move(scan_msg));
      }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sl::ILidarDriver* lidar_driver_;
    bool is_lidar_connected_;
    sl::IChannel* channel;


    std::string serial_port_;
    int serial_baudrate_;
    std::string frame_id_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an instance of the RplidarPublisher node and run it
    rclcpp::spin(std::make_shared<RplidarPublisher>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}