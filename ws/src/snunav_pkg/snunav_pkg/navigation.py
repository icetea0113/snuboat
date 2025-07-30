# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from snumsg_pkg.msg import MissionCode, Sensor
import numpy as np

# MissionDirector message information
# uint32 aabbcccc
# aa is auto/manual & init/run/pause/stand by/done
# bb is sensor mode (00: Qualisys, 10: SLAM, 20: GPS-RTK, 30: Marker)
# cc is maneuver mode

STATUS_QUALISYS_OFF = "00"
STATUS_QUALISYS_ON = "01"
STATUS_QUALISYS_STANDBY = "02"
STATUS_SLAM_OFF = "10"
STATUS_SLAM_ON = "11"
STATUS_SLAM_STANDBY = "12"
STATUS_GPS_RTK_OFF = "20"
STATUS_GPS_RTK_ON = "21"
STATUS_GPS_RTK_STANDBY = "22"
STATUS_MARKER_OFF = "30"
STATUS_MARKER_ON = "31"
STATUS_MARKER_STANDBY = "32"
STATUS_ERROR = "99"

class Navigation(Node):

    def __init__(self):
        super().__init__('navigation')
        
        # Publishers
        self.sensor_state_publisher_ = self.create_publisher(Sensor, 'sensor', 10)

        # Subscribers
        self.mission_subscriber_ = self.create_subscription(
            MissionCode,
            'mission_code',
            self.mission_callback,
            10)
        
        # Sensor Subscribers (assuming Float32MultiArray: [status, pose(6), vel(6)])
        self.qualisys_subscriber_ = self.create_subscription(Float32MultiArray, 'qualisys_data', self.qualisys_callback, 10)
        self.slam_subscriber_ = self.create_subscription(Float32MultiArray, 'slam_data', self.slam_callback, 10)
        self.gps_rtk_subscriber_ = self.create_subscription(Float32MultiArray, 'gps_rtk_data', self.gps_rtk_callback, 10)
        self.marker_subscriber_ = self.create_subscription(Float32MultiArray, 'marker_data', self.marker_callback, 10)

        # Timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # State variables
        self.mission_code = 0
        self.active_sensor_mode = -1 # -1: None, 0x10: Qualisys, 0x20: SLAM, 0x30: GPS-RTK, 0x40: Marker
        
        self.pose = np.zeros(6, dtype=np.float32)
        self.vel = np.zeros(6, dtype=np.float32)
        self.status = STATUS_ERROR

        # Sensor specific data
        self.status_qualisys = STATUS_QUALISYS_OFF
        self.pose_qualisys = np.zeros(6, dtype=np.float32)
        self.vel_qualisys = np.zeros(6, dtype=np.float32)
        
        self.status_slam = STATUS_SLAM_OFF
        self.pose_slam = np.zeros(6, dtype=np.float32)
        self.vel_slam = np.zeros(6, dtype=np.float32)

        self.status_gps_rtk = STATUS_GPS_RTK_OFF
        self.pose_gps_rtk = np.zeros(6, dtype=np.float32)
        self.vel_gps_rtk = np.zeros(6, dtype=np.float32)

        self.status_marker = STATUS_MARKER_OFF
        self.pose_marker = np.zeros(6, dtype=np.float32)
        self.vel_marker = np.zeros(6, dtype=np.float32)

    def mission_callback(self, msg):
        # motor_mode, sensor_mode, maneuver_mode, sub_maneuver_mode, subsub_maneuver_mode, status
        self.mission_code = int(msg.mission_code, 16)
        self.active_sensor_mode = (self.mission_code & 0x0F0000) >> 16
        self.get_logger().info(f'Received Mission Code: {hex(self.mission_code)}, Sensor Mode: {hex(self.active_sensor_mode)}')

    ## -- TODO : Implement actual sensor data processing logic
    # For now, we will just simulate the data reception and processing
    def qualisys_callback(self, msg):
        if len(msg.data) == 13:
            self.status_qualisys = int(msg.data[0])
            self.pose_qualisys = np.array(msg.data[1:7], dtype=np.float32)
            self.vel_qualisys = np.array(msg.data[7:13], dtype=np.float32)

    def slam_callback(self, msg):
        if len(msg.data) == 13:
            self.status_slam = int(msg.data[0])
            self.pose_slam = np.array(msg.data[1:7], dtype=np.float32)
            self.vel_slam = np.array(msg.data[7:13], dtype=np.float32)

    def gps_rtk_callback(self, msg):
        if len(msg.data) == 13:
            self.status_gps_rtk = int(msg.data[0])
            self.pose_gps_rtk = np.array(msg.data[1:7], dtype=np.float32)
            self.vel_gps_rtk = np.array(msg.data[7:13], dtype=np.float32)

    def marker_callback(self, msg):
        if len(msg.data) == 13:
            self.status_marker = int(msg.data[0])
            self.pose_marker = np.array(msg.data[1:7], dtype=np.float32)
            self.vel_marker = np.array(msg.data[7:13], dtype=np.float32)

    ##############################################################################
    
    def timer_callback(self):
        # Combine sensor type prefix and sensor state suffix to create the final status
        status_prefix = ""
        status_suffix = ""

        if self.active_sensor_mode == 0x0: # Qualisys
            self.pose = self.pose_qualisys
            self.vel = self.vel_qualisys
            status_suffix = str(self.status_qualisys)
        elif self.active_sensor_mode == 0x1: # SLAM
            self.pose = self.pose_slam
            self.vel = self.vel_slam
            status_suffix = str(self.status_slam)
        elif self.active_sensor_mode == 0x2: # GPS-RTK
            self.pose = self.pose_gps_rtk
            self.vel = self.vel_gps_rtk
            status_suffix = str(self.status_gps_rtk)
        elif self.active_sensor_mode == 0x3: # Marker
            self.pose = self.pose_marker
            self.vel = self.vel_marker
            status_suffix = str(self.status_marker)
        
        status_prefix = str(self.active_sensor_mode >> 4)
        if status_prefix:
            self.status = f"{status_prefix}{status_suffix}"
        else:
            self.status = STATUS_ERROR

        # Publish the current sensor state
        msg = Sensor()
        msg.tick = self.get_clock().now().to_msg()
        msg.status = self.status
        msg.pose = self.pose.tolist()
        msg.vel = self.vel.tolist()
        
        self.sensor_state_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()