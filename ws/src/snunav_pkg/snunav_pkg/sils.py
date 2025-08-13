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

from std_msgs.msg import String, Float32MultiArray, Int32
from snumsg_pkg.msg import MissionCode, Sensor, Control
import numpy as np
from snunav_pkg.utils.ship_dyn import ShipDyn

from std_msgs.msg import String


class SILS(Node):

    def __init__(self):
        super().__init__('sils')
        self.pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, np.deg2rad(45)])  # [x, y, z, roll, pitch, yaw]
        self.vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [vx, vy, vz, vroll, vpitch, vyaw]
        self.port_rps_cmd = 0.0
        self.port_steer_cmd = 0.0
        self.stbd_rps_cmd = 0.0
        self.stbd_steer_cmd = 0.0
        self.port_rps_fb = 0.0
        self.port_steer_fb = 0.0
        self.stbd_rps_fb = 0.0
        self.stbd_steer_fb = 0.0
        self.ship_dyn = ShipDyn()
        
        # Subscribers
        self.ctrl_cmd = self.create_subscription(
            Control,
            'ctrl_cmd_sils',
            self.sils_callback,
            10)
        
        # Publishers
        self.navigation_sils_pub = self.create_publisher(Float32MultiArray, 'sils_navigation_data', 10)
        self.motor_fb_sils_pub = self.create_publisher(Control, 'sils_motor_fb_data', 10)

    def sils_callback(self, msg):
        if len(msg.ctrl) == 4:
            self.port_rps_cmd = float(msg.ctrl[0])
            self.stbd_rps_cmd = float(msg.ctrl[1])
            self.port_steer_cmd = float(msg.ctrl[2])
            self.stbd_steer_cmd = float(msg.ctrl[3])
            # self.get_logger().info(f'SILS control command received: '
            #                        f'Port RPS: {self.port_rps_cmd}, Stbd RPS: {self.stbd_rps_cmd}, '
            #                        f'Port Steer: {self.port_steer_cmd}, Stbd Steer: {self.stbd_steer_cmd}')
        else:
            self.get_logger().error('Invalid SILS control command received. Expected 4 values.')
            
        # mapping member variables to msg
        pos_old = np.array([self.pos[0], self.pos[1], self.pos[5]])
        vel_old = np.array([self.vel[0], self.vel[1], self.vel[5]])
        ctrl_input = np.array([self.port_steer_cmd, self.stbd_steer_cmd, self.port_rps_cmd, self.stbd_rps_cmd])
        ctrl_old = np.array([self.port_steer_fb, self.stbd_steer_fb, self.port_rps_fb, self.stbd_rps_fb])
        wind_state = np.array([0.0, 0.0]) # TODO: Wind state should be provided or calculated
        
        # Boat Dynamics Simulation
        pos_new, vel_new, ctrl_new = self.ship_dyn.update_ship_state(pos_old, vel_old, ctrl_input, ctrl_old, wind_state, 0.1)
        
        # mapping dynamics output to msg
        self.pos[0] = pos_new[0]
        self.pos[1] = pos_new[1]
        self.pos[5] = pos_new[2]
        self.vel[0] = vel_new[0]
        self.vel[1] = vel_new[1]
        self.vel[5] = vel_new[2]
        self.port_steer_fb = ctrl_new[0]
        self.stbd_steer_fb = ctrl_new[1]
        self.port_rps_fb = ctrl_new[2]
        self.stbd_rps_fb = ctrl_new[3]
        
        #Publish Navigation Data
        nav_msg = Float32MultiArray()
        nav_msg.data = [float(41),  # Status: SILS ON
            self.pos[0], self.pos[1], self.pos[2],
            self.pos[3], self.pos[4], self.pos[5],
            self.vel[0], self.vel[1], self.vel[2],
            self.vel[3], self.vel[4], self.vel[5]]
        self.navigation_sils_pub.publish(nav_msg)
        # self.get_logger().info(f'Published SILS navigation data: {nav_msg.data}')
        
        #Publish Motor Feedback Data
        motor_fb_msg = Control()
        motor_fb_msg.ctrl = [self.port_rps_fb, self.stbd_rps_fb, self.port_steer_fb, self.stbd_steer_fb]
        motor_fb_msg.tick = self.get_clock().now().to_msg()
        self.motor_fb_sils_pub.publish(motor_fb_msg)
        self.get_logger().info(f'Published SILS motor feedback data: {motor_fb_msg.ctrl}')

def main(args=None):
    rclpy.init(args=args)

    sils = SILS()

    rclpy.spin(sils)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sils.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
