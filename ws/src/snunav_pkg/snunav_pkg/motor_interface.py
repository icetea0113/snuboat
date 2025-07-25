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

from std_msgs.msg import String, Float32MultiArray


class MotorInterface(Node):

    def __init__(self):
        super().__init__('motor_interface')

        self.state_publisher_ = self.create_publisher(
            Float32MultiArray,
            'ctrl',
            10)
        
        timer_period = 0.1  # seconds
        
        self.subscriber_ = self.create_subscription(
            Float32MultiArray,
            'ctrl_cmd',
            self.listener_callback,
            10)
        self.motor_state_subscriber_ = self.create_subscription(
            Float32MultiArray,
            'motor_state',
            self.motor_state_callback,
            10)

        self.cmd_rps_p, self.cmd_del_p = 0.0, 0.0 
        self.cmd_rps_s, self.cmd_del_s = 0.0, 0.0
        
        self.rps_p, self.del_p = 0.0, 0.0
        self.rps_s, self.del_s = 0.0, 0.0

    def listener_callback(self, msg):
        self.cmd_rps_p, self.cmd_del_p, self.cmd_rps_s, self.cmd_del_s = msg.data
        self.get_logger().info('Received command: '
                                'cmd_rps_p: %.2f, cmd_del_p: %.2f,'
                                'cmd_rps_s: %.2f, cmd_del_s: %.2f' %
                                (self.cmd_rps_p, self.cmd_del_p,
                                self.cmd_rps_s, self.cmd_del_s))

    def motor_state_callback(self, msg):
        self.rps_p, self.del_p, self.rps_s, self.del_s = msg.data
        
        # Publish the received motor state
        state_msg = Float32MultiArray()
        state_msg.data = [self.rps_p, self.del_p, self.rps_s, self.del_s]
        self.state_publisher_.publish(state_msg)
        
        self.get_logger().info('Actual state: '
                                'rps_p: %.2f, del_p: %.2f,'
                                'rps_s: %.2f, del_s: %.2f' %
                                (self.rps_p, self.del_p,
                                self.rps_s, self.del_s))
        
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