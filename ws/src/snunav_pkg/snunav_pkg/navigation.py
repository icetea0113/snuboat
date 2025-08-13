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
from nav_msgs.msg import Odometry
from snumsg_pkg.msg import MissionCode, Sensor
import numpy as np
from collections import deque

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
STATUS_SILS_OFF = "40"
STATUS_SILS_ON = "41"
STATUS_SILS_STANDBY = "42"
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
        self.slam_subscriber_ = self.create_subscription(Odometry, '/kiss/odometry', self.slam_callback, 10)
        self.gps_rtk_subscriber_ = self.create_subscription(Float32MultiArray, 'gps_rtk_data', self.gps_rtk_callback, 10)
        self.marker_subscriber_ = self.create_subscription(Float32MultiArray, 'marker_data', self.marker_callback, 10)
        self.sils_subscriber_ = self.create_subscription(Float32MultiArray, 'sils_navigation_data', self.sils_callback, 10)

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
        self.pose_qualisys_queue = np.zeros((5, 6), dtype=np.float32)
        self.vel_qualisys_queue = np.zeros((5, 6), dtype=np.float32)
        self.pose_qualisys_filt = np.zeros(6, dtype=np.float32)
        self.vel_qualisys_filt = np.zeros(6, dtype=np.float32)

        self.status_slam = STATUS_SLAM_OFF
        self.pose_slam = np.zeros(6, dtype=np.float32)
        self.vel_slam = np.zeros(6, dtype=np.float32)

        self.status_gps_rtk = STATUS_GPS_RTK_OFF
        self.pose_gps_rtk = np.zeros(6, dtype=np.float32)
        self.vel_gps_rtk = np.zeros(6, dtype=np.float32)

        self.status_marker = STATUS_MARKER_OFF
        self.pose_marker = np.zeros(6, dtype=np.float32)
        self.vel_marker = np.zeros(6, dtype=np.float32)
        
        self.status_sils = STATUS_SILS_OFF
        self.pose_sils = np.zeros(6, dtype=np.float32)
        self.vel_sils = np.zeros(6, dtype=np.float32)
        
        self.x_data_queue = None


    def mission_callback(self, msg):
        # motor_mode, sensor_mode, maneuver_mode, sub_maneuver_mode, subsub_maneuver_mode, status
        self.mission_code = int(msg.mission_code, 16)
        self.active_sensor_mode = (self.mission_code & 0x00F0000) >> 16
        # self.get_logger().info(f'Received Mission Code: {hex(self.mission_code)}, Sensor Mode: {hex(self.active_sensor_mode)}')

    ## -- TODO : Implement actual sensor data processing logic
    # For now, we will just simulate the data reception and processing
    def qualisys_callback(self, msg):
        if len(msg.data) == 13:
            self.status_qualisys = int(msg.data[0])
            self.pose_qualisys = np.array(msg.data[1:7], dtype=np.float32)
            self.vel_qualisys = np.array(msg.data[7:13], dtype=np.float32)

            # unit translation
            # self.pose_qualisys[:4] *= 0.001
            # self.vel_qualisys[:4] *= 0.001 
            self.pose_qualisys[3:] *= np.pi/180
            self.vel_qualisys[3:] *= np.pi/180 

            # filter
            # self.pose_qualisys_queue = np.roll(self.pose_qualisys_queue, -1, axis=0)
            # self.pose_qualisys_queue[-1] = self.pose_qualisys
            # self.vel_qualisys_queue = np.roll(self.vel_qualisys_queue, -1, axis=0)
            # self.vel_qualisys_queue[-1] = self.vel_qualisys

            # self.pose_qualisys_filt = np.mean(self.pose_qualisys_queue, axis=0)
            # self.vel_qualisys_filt = np.mean(self.vel_qualisys_queue, axis=0)
            
            # Median filter
            if self.x_data_queue is None:
                self.pos_maxlen_param = 20
                self.pos_avglen_param = 4
                self.x_data_queue = deque(maxlen = self.pos_maxlen_param)
                self.y_data_queue = deque(maxlen = self.pos_maxlen_param)
                self.z_data_queue = deque(maxlen = self.pos_maxlen_param)
                self.roll_data_queue = deque(maxlen = self.pos_maxlen_param)
                self.pitch_data_queue = deque(maxlen = self.pos_maxlen_param)
                self.yaw_data_queue = deque(maxlen = self.pos_maxlen_param)

                self.pos_data_queues = [self.x_data_queue, self.y_data_queue, self.z_data_queue,
                                        self.roll_data_queue, self.pitch_data_queue, self.yaw_data_queue]

                self.vel_maxlen_param = 20
                self.vel_avglen_param = 4
                self.u_data_queue = deque(maxlen = self.vel_maxlen_param)
                self.v_data_queue = deque(maxlen = self.vel_maxlen_param)
                self.w_data_queue = deque(maxlen = self.vel_maxlen_param)
                self.p_data_queue = deque(maxlen = self.vel_maxlen_param)
                self.q_data_queue = deque(maxlen = self.vel_maxlen_param)
                self.r_data_queue = deque(maxlen = self.vel_maxlen_param)

                self.vel_data_queues = [self.u_data_queue, self.v_data_queue, self.w_data_queue,
                                        self.p_data_queue, self.q_data_queue, self.r_data_queue]

            if len(self.x_data_queue) >= self.pos_maxlen_param:
                for idx in range(len(self.pos_data_queues)):
                    self.pos_data_queues[idx].popleft()
                
                for idx in range(len(self.vel_data_queues)):
                    self.vel_data_queues[idx].popleft()
        
            if len(self.x_data_queue) < self.pos_maxlen_param:
                for idx, data in enumerate(self.pose_qualisys):
                    self.pos_data_queues[idx].append(data)
                self.pose_qualisys_filt = [
                    median_filter(self.x_data_queue, self.pos_avglen_param),
                    median_filter(self.y_data_queue, self.pos_avglen_param),
                    median_filter(self.z_data_queue, self.pos_avglen_param),
                    median_filter(self.roll_data_queue, self.pos_avglen_param),
                    median_filter(self.pitch_data_queue, self.pos_avglen_param),
                    self.pose_qualisys[5]
                ]
                # self.get_logger().info(f"Pose data added to queue: {data}")

            if len(self.u_data_queue) < self.vel_maxlen_param:
                for idx, data in enumerate(self.vel_qualisys):
                    self.vel_data_queues[idx].append(data)
                                
                self.vel_qualisys_filt = [
                    median_filter(self.u_data_queue, self.vel_avglen_param),
                    median_filter(self.v_data_queue, self.vel_avglen_param),
                    median_filter(self.w_data_queue, self.vel_avglen_param),
                    median_filter(self.p_data_queue, self.vel_avglen_param),
                    median_filter(self.q_data_queue, self.vel_avglen_param),
                    # median_filter(self.r_data_queue, self.vel_avglen_param)
                    self.vel_qualisys[5]
                ]
                # self.get_logger().info(f"Velocity data added to queue: {data}")
                

        self.timer_callback()
        
    def slam_callback(self, msg):
        self.status_slam = STATUS_SLAM_ON
        
        pose = msg.pose.pose
        x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        
        self.pose_slam = np.array([pose.position.x, pose.position.y, pose.position.z, 
                                roll, pitch, yaw], dtype=np.float32)
        
        twist = msg.twist.twist
        self.vel_slam = np.array([twist.linear.x, twist.linear.y, twist.linear.z, 
                                twist.angular.x, twist.angular.y, twist.angular.z], dtype=np.float32)

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
            
    def sils_callback(self, msg):
        self.status_sils = STATUS_SILS_ON
        if len(msg.data) == 13:
            self.status_sils = int(msg.data[0])
            self.pose_sils = np.array(msg.data[1:7], dtype=np.float32)
            self.vel_sils = np.array(msg.data[7:13], dtype=np.float32)
            # self.get_logger().info(f'SILS Status: {self.status_sils}, Pose: {self.pose_sils}, Velocity: {self.vel_sils}')
            
            self.timer_callback()  # Call timer callback to update sensor state immediately

    ##############################################################################
    
    def timer_callback(self):
        # Combine sensor type prefix and sensor state suffix to create the final status
        status_prefix = ""
        status_suffix = ""

        if self.active_sensor_mode == 0x0: # Qualisys
            # self.pose = self.pose_qualisys
            # self.vel = self.vel_qualisys
            if self.x_data_queue is None:
                self.pose = self.pose_qualisys
                self.vel = self.vel_qualisys
                # self.get_logger().info("not filter!!!")  
            elif len(self.x_data_queue) < self.pos_maxlen_param:
                self.pose = self.pose_qualisys
                self.vel = self.vel_qualisys
                # self.get_logger().info("not filter!!!")  
            else:
                self.pose = np.array(self.pose_qualisys_filt)
                self.vel = np.array(self.vel_qualisys_filt)
            status_suffix = str(self.status_qualisys)
        elif self.active_sensor_mode == 0x1: # SLAM
            self.pose = np.array(self.pose_slam)
            self.vel = np.array(self.vel_slam)
            status_suffix = str(self.status_slam)
        elif self.active_sensor_mode == 0x2: # GPS-RTK
            self.pose = np.array(self.pose_gps_rtk)
            self.vel = np.array(self.vel_gps_rtk)
            status_suffix = str(self.status_gps_rtk)
        elif self.active_sensor_mode == 0x3: # Marker
            self.pose = np.array(self.pose_marker)
            self.vel = np.array(self.vel_marker)
            status_suffix = str(self.status_marker)
        elif self.active_sensor_mode == 0x4: # SILS
            self.pose = np.array(self.pose_sils)
            self.vel = np.array(self.vel_sils)
            status_suffix = str(self.status_sils)
        # self.get_logger().info(f'SILS Pose at timer: {self.pose}, Velocity: {self.vel}')
        
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

def median_filter(dataset, filter_size=6):
    sorted_data = np.sort(dataset)
    median_data_set = sorted_data[len(sorted_data) // 2 - filter_size // 2:len(sorted_data) // 2 + filter_size // 2]
    return np.mean(median_data_set)

def main(args=None):
    rclpy.init(args=args)
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()