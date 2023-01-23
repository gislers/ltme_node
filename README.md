ROS 2 driver for LitraTech's latest generation of mechanical 2D LiDARs running LDCP (**L**iDAR **D**ata and **C**ontrol **P**rotocol). Supported models are:
* **LTME-02A**
* R Series: **LT-R1, LT-R2**
* I Series: **LT-I1, LT-I2**

# Build and Install

## Dependencies

* C++17 capable compiler: **[Required]** This targets ROS Humble which is the current LTS ROS2 distribution.
* OpenSSL: *[Optional]* The firmware updater will not be built if OpenSSL development files are missing; other parts of the package are not affected.

## Build the Package

Clone or extract package source to your ROS 2 workspace's `src` directory, then build the workspace:

```
cd ~/ltme_ws/src
git clone https://github.com/gislers/ltme_node.git
cd .. && colcon build
```

# Nodes

## ltme_node

Reads and publishes measurement data (ranges & intensities) from connected device. It also exposes several services for other nodes to query information about the device and control its operation mode.

### Published Topics

`scan` ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)): Laser scan measurements from connected device. The topic name can be changed with `<remap>` tag.

### Services

`~query_serial` (`ltme_interfaces/QuerySerial`): Returns connected device's serial number as a string.

*ltme_interfaces/QuerySerial*
```
---
bool success
string serial
```

`~query_firmware_version` (`ltme_interfaces/QueryFirmwareVersion`): Returns connected device's firmware version as a string.

*ltme_interfaces/QueryFirmwareVersion*
```
---
bool success
string firmware_version
```

`~query_hardware_version` (`ltme_interfaces/QueryHardwareVersion`): Returns connected device's hardware version as a string.

*ltme_interfaces/QueryHardwareVersion*
```
---
bool success
string hardware_version
```

`~request_hibernation` ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)): Put device into standby mode. During standby the device will turn off its motor and laser to prevent wearing and save power; no data will be published until it's brought out of standby.

`~request_wake_up` ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html)): Exit standby mode and resume normal operation.

`~quit_driver` ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)): Close the lidar driver and release resources.

### Parameters

`~device_model` (string) **[Required]**: Model name of the target device. Supported models are:
* `LTME-02A`
* R Series: `LT-R1`, `LT-R2`
* I Series: `LT-I1`, `LT-I2`

`~device_address` (string) **[Required]**: IP address of the target device. For supported models, factory default address is `192.168.10.160`.

`~frame_id` (string, default: "laser") *[Optional]*: Frame ID of published `LaserScan` messages.

`~invert_frame` (bool, default: "false") *[Optional]*: If this option is enabled, published `LaserScan` messages will have their X and Z axes inverted. When the device is mounted upside down, you can use this option to revert the inversion caused by mounting, and make it looks like the scans are from a device installed in a regular way.

`~angle_min` and `~angle_max` (float, default: -2.356 and 2.356 for models with 270-degree FoV; -3.142 and 3.142 for models with 360-degree FoV) *[Optional]*: Start and end angle of published laser scans (in radians). Default values for these parameters make the entire angular FoV available without clipping.

`~angle_excluded_min` and `~angle_excluded_max` (float, default: -3.142 and -3.142) *[Optional]*: Range of angle (in radians) for which data should be excluded from published laser scans. Default values for these parameters make the entire angular FoV available without clipping.

`~range_min` and `~range_max` (float, default: 0.05 and model-specific default max) *[Optional]*: Minimum and maximum range values that are considered valid for published laser scans. Range values out of these bounds should be ignored.

## ltme_interfaces

Holds message and service definitions used in the `ltme_node`. 