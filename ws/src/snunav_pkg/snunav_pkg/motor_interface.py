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
import socket
import threading
import time
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray


class MotorInterface(Node):

    def __init__(self):
        super().__init__('motor_interface')
        self.DEST  = ("192.168.1.100", 7777)
        self.BIND  = ("192.168.1.2", 7777)  # 수신 포트

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.BIND)
        
        self.ctrl_cmd_boat_sub = self.create_subscription(
            Float32MultiArray,
            'ctrl_cmd_boat',
            self.motor_interface_callback,
            10)
        
        self.ctrl_fb_boat_pub = self.create_publisher(
            Float32MultiArray,
            'ctrl_fb_boat',
            10)        

    def motor_interface_callback(self, msg):
        # receive feedback from boat
        
        # make ctrl cmd string
        pass
        
def main(args=None):
    rclpy.init(args=args)

    motor_interface = MotorInterface()

    rclpy.spin(motor_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()