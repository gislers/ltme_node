#ifndef LTME_NODE_H
#define LTME_NODE_H

#include "ltme_interfaces/srv/query_serial.hpp"
#include "ltme_interfaces/srv/query_firmware_version.hpp"
#include "ltme_interfaces/srv/query_hardware_version.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "ldcp/device.h"

#include <mutex>
#include <atomic>

class LidarDriver : public rclcpp::Node
{
public:
  const static std::string DEFAULT_ENFORCED_TRANSPORT_MODE;
  const static std::string DEFAULT_FRAME_ID;
  const static bool DEFAULT_INVERT_FRAME;
  const static int DEFAULT_SCAN_FREQUENCY;
  const static double ANGLE_MIN_LIMIT;
  const static double ANGLE_MAX_LIMIT;
  const static double DEFAULT_ANGLE_EXCLUDED_MIN;
  const static double DEFAULT_ANGLE_EXCLUDED_MAX;
  const static double RANGE_MIN_LIMIT;
  const static double RANGE_MAX_LIMIT_NO_INIT;
  const static double RANGE_MAX_LIMIT_02A;
  const static double RANGE_MAX_LIMIT_R1;
  const static double RANGE_MAX_LIMIT_R2;
  const static double RANGE_MAX_LIMIT_I1;
  const static double RANGE_MAX_LIMIT_I2;
  const static int DEFAULT_AVERAGE_FACTOR;
  const static int DEFAULT_SHADOW_FILTER_STRENGTH;
  const static int DEFAULT_RECEIVER_SENSITIVITY_BOOST;

public:
  LidarDriver();
  void run();

private:
  void getParameters();
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;

  // Services
  void querySerialService(const ltme_interfaces::srv::QuerySerial::Request::SharedPtr request,
    const ltme_interfaces::srv::QuerySerial::Response::SharedPtr response);
  void queryFirmwareVersion(const ltme_interfaces::srv::QueryFirmwareVersion::Request::SharedPtr request,
    const ltme_interfaces::srv::QueryFirmwareVersion::Response::SharedPtr response);
  void queryHardwareVersion(const ltme_interfaces::srv::QueryHardwareVersion::Request::SharedPtr request,
    const ltme_interfaces::srv::QueryHardwareVersion::Response::SharedPtr response);
  void requestHibernationService(const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);
  void requestWakeUpService(const std_srvs::srv::Trigger::Request::SharedPtr request,
    const std_srvs::srv::Trigger::Response::SharedPtr response);
  void quitDriverService(const std_srvs::srv::Empty::Request::SharedPtr request,
    const std_srvs::srv::Empty::Response::SharedPtr response);

  rclcpp::Service<ltme_interfaces::srv::QuerySerial>::SharedPtr query_serial_service_;
  rclcpp::Service<ltme_interfaces::srv::QueryFirmwareVersion>::SharedPtr query_firmware_version_service_;
  rclcpp::Service<ltme_interfaces::srv::QueryHardwareVersion>::SharedPtr query_hardware_version_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_hibernation_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr request_wakeup_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr quit_driver_service_;

  std::string device_model_;
  std::string device_address_;
  std::string enforced_transport_mode_;
  std::string frame_id_;
  bool invert_frame_;
  int scan_frequency_override_;
  double angle_min_;
  double angle_max_;
  double angle_excluded_min_;
  double angle_excluded_max_;
  double range_min_;
  double range_max_;
  int average_factor_;
  int shadow_filter_strength_;
  int receiver_sensitivity_boost_;

  std::unique_ptr<ldcp_sdk::Device> device_;
  std::mutex mutex_;

  std::atomic_bool hibernation_requested_;
  std::atomic_bool quit_driver_;
};

#endif
