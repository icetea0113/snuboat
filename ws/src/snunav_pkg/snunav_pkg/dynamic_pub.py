#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import os

# Define utils directory and input file path
UTILS_DIR = os.path.join(os.path.dirname(__file__), 'utils')
INPUT_FILE = os.path.join(UTILS_DIR, 'motor_input.txt')

class DynamicPublisher(Node):
    def __init__(self, publish_rate=10.0):
        # Ensure utils directory exists
        os.makedirs(UTILS_DIR, exist_ok=True)
        if not os.path.exists(INPUT_FILE):
            with open(INPUT_FILE, 'w') as f:
                f.write("0 0 0 0\n")
        super().__init__('rqt')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/ctrl_cmd_boat', 10)
        self.get_logger().info(
            "[rps P, rps S, del P, del S]\n"
            "Example: 11.8 11.8 0.0 0.0\n"
            "Press Ctrl+C to exit"
        )
        self.latest_values = [0.0, 0.0, 0.0, 0.0]
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.running = True

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = self.latest_values
        self.get_logger().info(f"Publishing: {self.latest_values}")
        self.publisher_.publish(msg)

    def input_thread_loop(self):
        while self.running and rclpy.ok():
            try:
                with open(INPUT_FILE, 'r') as f:
                    lines = f.readlines()
                if lines:
                    line = lines[-1].strip()
                else:
                    line = ""
                parts = line.split()
                if len(parts) != 4:
                    values = [0.0, 0.0, 0.0, 0.0]
                else:
                    try:
                        values = [float(x) for x in parts]
                    except ValueError:
                        values = [0.0, 0.0, 0.0, 0.0]
                self.latest_values = values
                msg = Float32MultiArray()
                msg.data = self.latest_values
                self.publisher_.publish(msg)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = DynamicPublisher()

    t = threading.Thread(target=node.input_thread_loop, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        t.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()