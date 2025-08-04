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

import os 
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from snumsg_pkg.msg import MissionCode  # 변경: std_msgs/String → MissionCode

class MissionDirector(Node):

    def __init__(self):
        super().__init__('mission_director')

        # 1) 파라미터 선언 (런치 파일에서 넘어오지 않을 경우를 대비한 기본값)
        self.declare_parameter('frequency', 10.0)  # [Hz]
        self.declare_parameter('sils_mode', '0')
        self.declare_parameter('maneuver_mode', '0')
        self.declare_parameter('sub_maneuver_mode', '0')
        self.declare_parameter('subsub_maneuver_mode', '0')
        self.declare_parameter('sensor_mode', '0')
        self.declare_parameter('motor_mode', '0')
        self.declare_parameter('status', '0')

        # 2) 퍼블리셔: MissionCode 타입
        self.publisher_ = self.create_publisher(
            MissionCode,
            'mission_code',  # 토픽 이름
            10
        )
        
        self.update_params()
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

        # self.status_subscriber_ = self.create_subscription(Int32, 'ctrl_status', self. status_callback, 10)

    def update_params(self):
        """현재 파라미터 서버에 설정된 값을 읽어와서 속성에 저장합니다."""
        # get_parameter().value 가 int 등이 올 수 있으므로 str()로 변환
        self.frequency         = float(self.get_parameter('frequency').value)
        self.sils_mode         = str(self.get_parameter('sils_mode').value)
        self.maneuver_mode     = str(self.get_parameter('maneuver_mode').value)
        self.sub_maneuver_mode = str(self.get_parameter('sub_maneuver_mode').value)
        self.subsub_maneuver_mode = str(self.get_parameter('subsub_maneuver_mode').value)
        self.sensor_mode       = str(self.get_parameter('sensor_mode').value)
        self.motor_mode        = str(self.get_parameter('motor_mode').value)
        self.status            = str(self.get_parameter('status').value)
        
        if self.sils_mode == '1':
            self.sensor_mode = '4'  # SILS 모드에서는 sensor_mode = 4

    def timer_callback(self):
        # 런타임 중에 파라미터가 바뀔 수도 있으므로, 매번 콜백마다 갱신
        # self.update_params()
        # <!> yaml 파일에서 읽어오는 것은 처음 실행할 때 뿐이고

        # MissionCode 메시지 생성 및 발행
        msg = MissionCode()
        msg.tick = self.get_clock().now().to_msg()
        msg.mission_code = (
            self.sils_mode +
            self.motor_mode +
            self.sensor_mode +
            self.maneuver_mode +
            self.sub_maneuver_mode +
            self.subsub_maneuver_mode +
            self.status
        )
        # 메시지 발행 / 000000 6자리 / 각 모드가 1자리씩
        # sils_mode, motor_mode, sensor_mode, maneuver_mode, sub_maneuver_mode, subsub_maneuver_mode, status

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing mission_code: "{msg.mission_code}"')

    def status_callback(self, msg):
        self.status = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = MissionDirector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
