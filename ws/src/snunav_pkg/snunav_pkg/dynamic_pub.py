#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DynamicPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_float32multiarray_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/ctrl_cmd_boat', 10)
        self.get_logger().info(
            "값을 입력하면 [rps P, rps S, del P, del S] 형태로 발행됩니다.\n"
            "예: 11.8 11.8 0.0 0.0\n"
            "종료하려면 Ctrl+C"
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.latest_values = [0.0, 0.0, 0.0, 0.0]

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.latest_values
        self.publisher_.publish(msg)

    def run_input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("값 입력: ").strip()
                if not user_input:
                    continue
                parts = user_input.split()
                if len(parts) != 4:
                    self.get_logger().warn("4개의 실수를 입력하세요: rps P, rps S, del P, del S")
                    continue
                self.latest_values = [float(x) for x in parts]
                self.get_logger().info(f"새 값 발행: {self.latest_values}")
            except ValueError:
                self.get_logger().error("숫자만 입력하세요.")
            except KeyboardInterrupt:
                break

def main(args=None):
    rclpy.init(args=args)
    node = DynamicPublisher()

    try:
        node.run_input_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()