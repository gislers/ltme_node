from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ltme_node = Node(
        package="ltme_node",
        executable="ltme_node",
        name="ltme_node",
        output="screen",
        parameters=[
            {"device_model": "LTME-02A"},
            # IP address of LTME-02A
            {"device_address": "192.168.10.160"},

            # Enforce specific transport mode used by the device to stream measurement data.
            # If set to "normal" or "oob", device settings will be changed to selected mode in case of a mismatch. 
            #  Available options are: 
            #  * none: don't enforce any mode and use device's current configuration 
            #  * normal: enforce normal (in-band) mode, i.e. TCP connection for command interaction is reused for data streaming 
            #  * oob: enforce out-of-band mode, where data are streamed through a dedicated UDP channel 
            {"enforced_transport_mode": "none"},

            # Frame ID used by the published LaserScan messages
            {"frame_id": "laser"},

            #  If this option is enabled, published LaserScan messages will have their X and Z axes inverted. 
            #  This is mostly useful when the device is mounted upside down, as it effectively undos the inversion created by the mounting, 
            #  and makes it look like the scans are from a device installed in a normal, upward direction. 
            {"invert_frame": False},

            # LTME-02 can be configured with different scan frequencies, ranging from 10 Hz to 30 Hz with 5 Hz increments.
            # The driver automatically queries device for its frequency upon connection and setup LaserScan parameters accordingly. 
            # If for some reason this doesn't work for you (e.g., a device with outdated firmware), 
            # this parameter can be used to override automatic detection and manually specify a correct frequency value. 
            {"scan_frequency_override": 15},

            #  Start and end angle of published scans 
            #  As LTME-02 has an FOV of 270 degrees, the minimum allowed value for angle_min is -2.356 (about -3 * pi / 4), and the maximum allowed value for angle_max is 2.356 (about 3 * pi / 4) 
            #  <param name="angle_min" value="-1.571"/> 
            {"angle_max", 1.571},

            #  Range of angle for which data should be excluded from published scans 
            #  Leave these two parameters commented out if a full 270 degree FOV is desired 
            {"angle_excluded_min": -0.785},
            {"angle_excluded_max": 0.785},

            #  Minimum and maximum range value of published scans 
            #  Defaults to 0.05 and 30 respectively if not specified 
            {"range_min": 0.05},
            {"range_max": 30},

            #  Number of neighboring measurements to be averaged
            #  Averaging reduces jitter but angular resolution will also decrease by the same factor
            {"average_factor": 2},

            #  Adjust how data post-processing stage will filter scan artifacts caused by veiling effect 
            #  Valid range is between 0 and 100 inclusive, larger value leads to more aggressive filtering 
            {"shadow_filter_strength": 50},


            #  Adjust sensitivity level of the rangefinder subsystem 
            #  Negative value corresponds to decreased sensitivity while positive value means enhanced sensitivity 
            #  Only integers between -20 and 10 (inclusive) are allowed 
            {"receiver_sensitivity_boost": 0}
        ]
    )

    ld.add_action(ltme_node)
    return ld
