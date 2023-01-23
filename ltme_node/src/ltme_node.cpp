#include "ltme_node/ltme_node.h"
#include <arpa/inet.h>
#include "sensor_msgs/msg/laser_scan.hpp"

const std::string LidarDriver::DEFAULT_ENFORCED_TRANSPORT_MODE = "none";
const std::string LidarDriver::DEFAULT_FRAME_ID = "laser";
const bool LidarDriver::DEFAULT_INVERT_FRAME = false;
const int LidarDriver::DEFAULT_SCAN_FREQUENCY = 15;
const double LidarDriver::ANGLE_MIN_LIMIT = -3.142;
const double LidarDriver::ANGLE_MAX_LIMIT = 3.142;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MIN = -3.142;
const double LidarDriver::DEFAULT_ANGLE_EXCLUDED_MAX = -3.142;
const double LidarDriver::RANGE_MIN_LIMIT = 0.05;
const double LidarDriver::RANGE_MAX_LIMIT_NO_INIT = -1;
const double LidarDriver::RANGE_MAX_LIMIT_02A = 30;
const double LidarDriver::RANGE_MAX_LIMIT_R1 = 30;
const double LidarDriver::RANGE_MAX_LIMIT_R2 = 30;
const double LidarDriver::RANGE_MAX_LIMIT_I1 = 100;
const double LidarDriver::RANGE_MAX_LIMIT_I2 = 70;
const int LidarDriver::DEFAULT_AVERAGE_FACTOR = 1;
const int LidarDriver::DEFAULT_SHADOW_FILTER_STRENGTH = 50;
const int LidarDriver::DEFAULT_RECEIVER_SENSITIVITY_BOOST = 0;

LidarDriver::LidarDriver()
  : Node("ltme_node")
  , hibernation_requested_(false)
  , quit_driver_(false)
{
  declare_parameter("device_model", std::string());
  declare_parameter("device_address", std::string());
  declare_parameter("enforced_transport_mode", DEFAULT_ENFORCED_TRANSPORT_MODE);
  declare_parameter("frame_id", DEFAULT_FRAME_ID);
  declare_parameter("invert_frame", DEFAULT_INVERT_FRAME);
  declare_parameter("scan_frequency_override", 0);
  declare_parameter("angle_min", ANGLE_MIN_LIMIT);
  declare_parameter("angle_max", ANGLE_MAX_LIMIT);
  declare_parameter("angle_excluded_min", DEFAULT_ANGLE_EXCLUDED_MIN);
  declare_parameter("angle_excluded_max", DEFAULT_ANGLE_EXCLUDED_MAX);
  declare_parameter("range_min", RANGE_MIN_LIMIT);
  declare_parameter("range_max", RANGE_MAX_LIMIT_NO_INIT);
  declare_parameter("average_factor", DEFAULT_AVERAGE_FACTOR);
  declare_parameter("shadow_filter_strength", DEFAULT_SHADOW_FILTER_STRENGTH);
  declare_parameter("receiver_sensitivity_boost", DEFAULT_RECEIVER_SENSITIVITY_BOOST);
}

void LidarDriver::getParameters() 
{
  device_model_ = get_parameter("device_model").as_string();
  if (device_model_.empty()) {
    RCLCPP_ERROR(get_logger(), "Missing required parameter \"device_model\"");
    exit(-1);
  }
  else if (device_model_ != "LTME-02A" &&
      device_model_ != "LT-R1" && device_model_ != "LT-R2" &&
      device_model_ != "LT-I1" && device_model_ != "LT-I2") {
    RCLCPP_ERROR(get_logger(), "Unsupported device model %s", device_model_.c_str());
    exit(-1);
  }

  if (get_parameter("range_max").as_double() == RANGE_MAX_LIMIT_NO_INIT) {  // no user parameter
    if (device_model_ == "LTME-02A") {
      range_max_ = RANGE_MAX_LIMIT_02A;
    } else if (device_model_ == "LT-R1") {
      range_max_ = RANGE_MAX_LIMIT_R1;
    } else if (device_model_ == "LT-R2") {
      range_max_ = RANGE_MAX_LIMIT_R2;
    } else if (device_model_ == "LT-I1") {
      range_max_ = RANGE_MAX_LIMIT_I1;
    } else if (device_model_ == "LT-I2") {
      range_max_ = RANGE_MAX_LIMIT_I2;
    }
    RCLCPP_INFO(get_logger(), "No range_max set, setting to %f", range_max_);
    set_parameter(rclcpp::Parameter("range_max", range_max_));
  }

  device_address_ = get_parameter("device_address").as_string();
  if (device_address_.empty()) {
    RCLCPP_ERROR(get_logger(), "Missing required parameter \"device_address\"");
    exit(-1);
  }

  enforced_transport_mode_ = get_parameter("enforced_transport_mode").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  invert_frame_ = get_parameter("invert_frame").as_bool();
  scan_frequency_override_ = get_parameter("scan_frequency_override").as_int();
  angle_min_ = get_parameter("angle_min").as_double();
  angle_max_ = get_parameter("angle_max").as_double();
  angle_excluded_min_ = get_parameter("angle_excluded_min").as_double();
  angle_excluded_max_ = get_parameter("angle_excluded_max").as_double();
  range_min_ = get_parameter("range_min").as_double();
  range_max_ = get_parameter("range_max").as_double();
  average_factor_ = get_parameter("average_factor").as_int();
  shadow_filter_strength_ = get_parameter("shadow_filter_strength").as_int();
  receiver_sensitivity_boost_ = get_parameter("receiver_sensitivity_boost").as_int();

  if (!(enforced_transport_mode_ == "none" || enforced_transport_mode_ == "normal" || enforced_transport_mode_ == "oob")) {
    RCLCPP_ERROR(get_logger(), "Transport mode \"%s\" not supported", enforced_transport_mode_.c_str());
    exit(-1);
  }
  if (scan_frequency_override_ != 0 &&
    (scan_frequency_override_ < 10 || scan_frequency_override_ > 30 || scan_frequency_override_ % 5 != 0)) {
    RCLCPP_ERROR(get_logger(), "Scan frequency %d not supported", scan_frequency_override_);
    exit(-1);
  }
  if (!(angle_min_ < angle_max_)) {
    RCLCPP_ERROR(get_logger(), "angle_min (%f) can't be larger than or equal to angle_max (%f)", angle_min_, angle_max_);
    exit(-1);
  }
  if (angle_min_ < ANGLE_MIN_LIMIT) {
    RCLCPP_ERROR(get_logger(), "angle_min is set to %f while its minimum allowed value is %f", angle_min_, ANGLE_MIN_LIMIT);
    exit(-1);
  }
  if (angle_max_ > ANGLE_MAX_LIMIT) {
    RCLCPP_ERROR(get_logger(), "angle_max is set to %f while its maximum allowed value is %f", angle_max_, ANGLE_MAX_LIMIT);
    exit(-1);
  }
  if (!(range_min_ < range_max_)) {
    RCLCPP_ERROR(get_logger(), "range_min (%f) can't be larger than or equal to range_max (%f)", range_min_, range_max_);
    exit(-1);
  }
  if (range_min_ < RANGE_MIN_LIMIT) {
    RCLCPP_ERROR(get_logger(), "range_min is set to %f while its minimum allowed value is %f", range_min_, RANGE_MIN_LIMIT);
    exit(-1);
  }
  if (average_factor_ <= 0 || average_factor_ > 8) {
    RCLCPP_ERROR(get_logger(), "average_factor is set to %d while its valid value is between 1 and 8", average_factor_);
    exit(-1);
  }
  if (shadow_filter_strength_ < 0 || average_factor_ > 100) {
    RCLCPP_ERROR(get_logger(), "shadow_filter_strength is set to %d while its valid value is between 0 and 100 (inclusive)", shadow_filter_strength_);
    exit(-1);
  }
  if (receiver_sensitivity_boost_ < -20 || receiver_sensitivity_boost_ > 10) {
    RCLCPP_ERROR(get_logger(), "receiver_sensitivity_boost is set to %d while the valid range is between -20 and 10 (inclusive)", receiver_sensitivity_boost_);
    exit(-1);
  }
}

void LidarDriver::run()
{
  std::unique_lock<std::mutex> lock(mutex_);
  getParameters();
  laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 16);

  query_serial_service_ = this->create_service<ltme_interfaces::srv::QuerySerial>(
    "query_serial", std::bind(&LidarDriver::querySerialService, this, std::placeholders::_1, std::placeholders::_2));
  query_firmware_version_service_ = this->create_service<ltme_interfaces::srv::QueryFirmwareVersion>(
    "query_firmware_version", std::bind(&LidarDriver::queryFirmwareVersion, this, std::placeholders::_1, std::placeholders::_2));
  query_hardware_version_service_ = this->create_service<ltme_interfaces::srv::QueryHardwareVersion>(
    "query_hardware_version", std::bind(&LidarDriver::queryHardwareVersion, this, std::placeholders::_1, std::placeholders::_2));
  request_hibernation_service_ = this->create_service<std_srvs::srv::Trigger>(
    "request_hibernation", std::bind(&LidarDriver::requestHibernationService, this, std::placeholders::_1, std::placeholders::_2));
  request_wakeup_service_ = this->create_service<std_srvs::srv::Trigger>(
    "request_wake_up", std::bind(&LidarDriver::requestWakeUpService, this, std::placeholders::_1, std::placeholders::_2));
  quit_driver_service_ = this->create_service<std_srvs::srv::Empty>(
    "quit_driver", std::bind(&LidarDriver::quitDriverService, this, std::placeholders::_1, std::placeholders::_2));
  
  std::string address_str = device_address_;
  std::string port_str = "2105";

  size_t position = device_address_.find(':');
  if (position != std::string::npos) {
    address_str = device_address_.substr(0, position);
    port_str = device_address_.substr(position + 1);
  }

  in_addr_t address = htonl(INADDR_NONE);
  in_port_t port = 0;
  try {
    address = inet_addr(address_str.c_str());
    if (address == htonl(INADDR_NONE))
      throw std::exception();
    port = htons(std::stoi(port_str));
  }
  catch (...) {
    RCLCPP_ERROR(get_logger(), "Invalid device address: %s", device_address_.c_str());
    exit(-1);
  }

  ldcp_sdk::NetworkLocation location(address, port);
  device_ = std::unique_ptr<ldcp_sdk::Device>(new ldcp_sdk::Device(location));

  rclcpp::Rate loop_rate(0.3);
  while (rclcpp::ok() && !quit_driver_.load()) {
    rclcpp::spin_some(shared_from_this());
    if (device_->open() == ldcp_sdk::no_error) {
      hibernation_requested_ = false;

      lock.unlock();

      RCLCPP_INFO(get_logger(), "Device opened");

      bool reboot_required = false;
      if (device_model_ == "LTME-02A" && enforced_transport_mode_ != "none") {
        std::string firmware_version;
        if (device_->queryFirmwareVersion(firmware_version) == ldcp_sdk::no_error) {
          RCLCPP_INFO(get_logger(), "Detected firmware version: %s", firmware_version.c_str());
          if (firmware_version < "0201")
            RCLCPP_WARN(get_logger(), "Firmware version %s supports normal transport mode only, "
              "\"enforced_transport_mode\" parameter will be ignored", firmware_version.c_str());
          else {
            bool oob_enabled = false;
            if (device_->isOobEnabled(oob_enabled) == ldcp_sdk::no_error) {
              if ((enforced_transport_mode_ == "normal" && oob_enabled) ||
                  (enforced_transport_mode_ == "oob" && !oob_enabled)) {
                RCLCPP_INFO(get_logger(), "Transport mode will be switched to \"%s\"", oob_enabled ? "normal" : "oob");
                device_->setOobEnabled(!oob_enabled);
                device_->persistSettings();
                reboot_required = true;
              }
            }
            else
              RCLCPP_WARN(get_logger(), "Unable to query device for its current transport mode, "
                "\"enforced_transport_mode\" parameter will be ignored");
          }
        }
        else
          RCLCPP_WARN(get_logger(), "Unable to query device for firmware version, \"enforced_transport_mode\" parameter will be ignored");
      }

      if (!reboot_required) {
        int scan_frequency = DEFAULT_SCAN_FREQUENCY;
        if (scan_frequency_override_ != 0)
          scan_frequency = scan_frequency_override_;
        else {
          if (device_->getScanFrequency(scan_frequency) != ldcp_sdk::no_error)
            RCLCPP_WARN(get_logger(), "Unable to query device for scan frequency and will use %d as the frequency value", scan_frequency);
        }

        if (shadow_filter_strength_ != DEFAULT_SHADOW_FILTER_STRENGTH) {
          if (device_->setShadowFilterStrength(shadow_filter_strength_) == ldcp_sdk::no_error)
            RCLCPP_INFO(get_logger(), "Shadow filter strength set to %d", shadow_filter_strength_);
          else
            RCLCPP_WARN(get_logger(), "Unable to set shadow filter strength");
        }

        if (receiver_sensitivity_boost_ != DEFAULT_RECEIVER_SENSITIVITY_BOOST) {
          if (device_->setReceiverSensitivityBoost(receiver_sensitivity_boost_) == ldcp_sdk::no_error) {
            RCLCPP_INFO(get_logger(), "Receiver sensitivity boost %d applied", receiver_sensitivity_boost_);
            int current_receiver_sensitivity = 0;
            if (device_->getReceiverSensitivityValue(current_receiver_sensitivity) == ldcp_sdk::no_error)
              RCLCPP_INFO(get_logger(), "Current receiver sensitivity: %d", current_receiver_sensitivity);
          }
        }

        device_->startMeasurement();
        device_->startStreaming();

        auto readScanBlock = [&](ldcp_sdk::ScanBlock& scan_block) {
          if (device_->readScanBlock(scan_block) != ldcp_sdk::no_error)
            throw std::exception();
        };

        ldcp_sdk::ScanBlock scan_block;

        bool device_ready = false;
        for (int i = 0; i < 5 && !device_ready; i++) {
          rclcpp::spin_some(shared_from_this());
          try {
            readScanBlock(scan_block);
            device_ready = true;
          }
          catch (...) {
            RCLCPP_INFO(get_logger(), "Waiting for device to become ready...");
          }
        }

        if (device_ready) {
          double fov_angle_min = 0, fov_angle_max = 0;
          switch (scan_block.angular_fov) {
            case ldcp_sdk::ANGULAR_FOV_270DEG:
              fov_angle_min = -M_PI * 3 / 4;
              fov_angle_max = M_PI * 3 / 4;
              break;
            case ldcp_sdk::ANGULAR_FOV_360DEG:
              fov_angle_min = -M_PI;
              fov_angle_max = M_PI;
              break;
            default:
              RCLCPP_ERROR(get_logger(), "Unsupported FoV flag %d", scan_block.angular_fov);
              exit(-1);
          }
          angle_min_ = (angle_min_ > fov_angle_min) ? angle_min_ : fov_angle_min;
          angle_max_ = (angle_max_ < fov_angle_max) ? angle_max_ : fov_angle_max;

          int beam_count = scan_block.block_count * scan_block.block_length * 360 /
            ((scan_block.angular_fov == ldcp_sdk::ANGULAR_FOV_270DEG) ? 270 : 360);
          int beam_index_min = std::ceil(angle_min_ * beam_count / (2 * M_PI));
          int beam_index_max = std::floor(angle_max_ * beam_count / (2 * M_PI));
          int beam_index_excluded_min = std::ceil(angle_excluded_min_ * beam_count / (2 * M_PI));
          int beam_index_excluded_max = std::floor(angle_excluded_max_ * beam_count / (2 * M_PI));

          sensor_msgs::msg::LaserScan laser_scan;
          laser_scan.header.frame_id = frame_id_;
          laser_scan.angle_min = (!invert_frame_) ? angle_min_ : angle_max_;
          laser_scan.angle_max = (!invert_frame_) ? angle_max_ : angle_min_;
          laser_scan.angle_increment = ((!invert_frame_) ? 1 : -1) *
              2 * M_PI / beam_count * average_factor_;
          laser_scan.time_increment = 1.0 / scan_frequency / beam_count * average_factor_;
          laser_scan.scan_time = 1.0 / scan_frequency;
          laser_scan.range_min = range_min_;
          laser_scan.range_max = range_max_;

          auto updateLaserScan = [&](const ldcp_sdk::ScanBlock& scan_block) {
            int block_size = scan_block.layers[0].ranges.size();
            for (int i = 0; i < block_size; i++) {
              int beam_index = (scan_block.block_index - scan_block.block_count / 2) * block_size + i;
              if (beam_index < beam_index_min || beam_index > beam_index_max)
                continue;
              if (beam_index >= beam_index_excluded_min && beam_index <= beam_index_excluded_max)
                continue;
              if (scan_block.layers[0].ranges[i] != 0) {
                laser_scan.ranges[beam_index - beam_index_min] = scan_block.layers[0].ranges[i] * 0.002;
                laser_scan.intensities[beam_index - beam_index_min] = scan_block.layers[0].intensities[i];
              }
              else {
                laser_scan.ranges[beam_index - beam_index_min] = std::numeric_limits<float>::infinity();
                laser_scan.intensities[beam_index - beam_index_min] = 0;
              }
            }
          };

          while (rclcpp::ok() && !quit_driver_.load()) {
            rclcpp::spin_some(shared_from_this());

            laser_scan.ranges.resize(beam_index_max - beam_index_min + 1);
            laser_scan.intensities.resize(beam_index_max - beam_index_min + 1);

            std::fill(laser_scan.ranges.begin(), laser_scan.ranges.end(), 0.0);
            std::fill(laser_scan.intensities.begin(), laser_scan.intensities.end(), 0.0);

            try {
              do {
                readScanBlock(scan_block);
              } while (scan_block.block_index != 0);

              laser_scan.header.stamp = get_clock()->now();

              while (scan_block.block_index != scan_block.block_count - 1) {
                updateLaserScan(scan_block);
                readScanBlock(scan_block);
              }
              updateLaserScan(scan_block);

              if (average_factor_ != 1) {
                int final_size = laser_scan.ranges.size() / average_factor_;
                for (int i = 0; i < final_size; i++) {
                  double ranges_total = 0, intensities_total = 0;
                  int count = 0;
                  for (int j = 0; j < average_factor_; j++) {
                    int index = i * average_factor_ + j;
                    if (laser_scan.ranges[index] != 0) {
                      ranges_total += laser_scan.ranges[index];
                      intensities_total += laser_scan.intensities[index];
                      count++;
                    }
                  }

                  if (count > 0) {
                    laser_scan.ranges[i] = ranges_total / count;
                    laser_scan.intensities[i] = (int)(intensities_total / count);
                  }
                  else {
                    laser_scan.ranges[i] = 0;
                    laser_scan.intensities[i] = 0;
                  }
                }

                laser_scan.ranges.resize(final_size);
                laser_scan.intensities.resize(final_size);
              }

              laser_scan_publisher_->publish(laser_scan);

              if (hibernation_requested_.load()) {
                device_->stopMeasurement();
                RCLCPP_INFO(get_logger(), "Device brought into hibernation");
                rclcpp::Rate loop_rate(10);
                while (hibernation_requested_.load()) {
                  rclcpp::spin_some(shared_from_this());
                  loop_rate.sleep();
                }
                device_->startMeasurement();
                RCLCPP_INFO(get_logger(), "Woken up from hibernation");
              }
            }
            catch (const std::exception&) {
              RCLCPP_WARN(get_logger(), "Error reading data from device");
              break;
            }
          }
        }
        else
          RCLCPP_INFO(get_logger(), "Device is not ready. Will close connection and retry");

        device_->stopStreaming();
      }
      else
        device_->reboot();

      lock.lock();
      device_->close();

      if (!reboot_required)
        RCLCPP_INFO(get_logger(), "Device closed");
      else
        RCLCPP_INFO(get_logger(), "Device rebooted");
    }
    else {
      auto& clk = *this->get_clock();
      RCLCPP_INFO_THROTTLE(get_logger(), clk, 5000, "Waiting for device... [%s]", device_address_.c_str());
      loop_rate.sleep();
    }
  }
}

void LidarDriver::querySerialService(const ltme_interfaces::srv::QuerySerial::Request::SharedPtr /*request*/,
    const ltme_interfaces::srv::QuerySerial::Response::SharedPtr response)
{
  std::string serial {};
  bool success = false;

  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock() && device_->querySerial(serial) == ldcp_sdk::no_error) {
    success = true;
  }
  response->success = success;
  response->serial = serial;
}

void LidarDriver::queryFirmwareVersion(const ltme_interfaces::srv::QueryFirmwareVersion::Request::SharedPtr /*request*/,
    const ltme_interfaces::srv::QueryFirmwareVersion::Response::SharedPtr response)
{
  std::string firmware_version {};
  bool success = false;

  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock() && device_->queryFirmwareVersion(firmware_version) == ldcp_sdk::no_error) {
    success = true;
  }
  response->success = success;
  response->firmware_version = firmware_version;
}

void LidarDriver::queryHardwareVersion(const ltme_interfaces::srv::QueryHardwareVersion::Request::SharedPtr /*request*/,
    const ltme_interfaces::srv::QueryHardwareVersion::Response::SharedPtr response)
{
  std::string hardware_version {};
  bool success = false;

  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock() && device_->queryHardwareVersion(hardware_version) == ldcp_sdk::no_error) {
    success = true;
  }
  response->success = success;
  response->hardware_version = hardware_version;
}

void LidarDriver::requestHibernationService(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  std::string error_message {};
  bool success = false;

  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    hibernation_requested_ = true;
    success = true;
  } else {
    error_message = "Could not obtain lock";
  }
  response->success = success;
  response->message = error_message;
}

void LidarDriver::requestWakeUpService(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  std::string error_messsage {};
  bool success = false;

  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    hibernation_requested_ = false;
    success = true;
  } else {
    error_messsage = "Could not obtain lock";
  }
  response->success = success;
  response->message = error_messsage;
}

void LidarDriver::quitDriverService(const std_srvs::srv::Empty::Request::SharedPtr /*request*/,
    const std_srvs::srv::Empty::Response::SharedPtr /*response*/)
{
  quit_driver_ = true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarDriver>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
