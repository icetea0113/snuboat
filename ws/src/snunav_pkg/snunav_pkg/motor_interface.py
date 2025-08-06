"""
ROS 2 MotorInterface node
  • 구독:  /ctrl_cmd_boat  (Float32MultiArray, [L_rps, R_rps, L_deg, R_deg])
  • 발행:  /ctrl_fb_boat   (Float32MultiArray, [mode, L_rps_fb, R_rps_fb,
                                             L_deg_fb, R_deg_fb, err_code])
  • 전송:  UDP $CTRL 프레임
  • 수신:  UDP $FB   프레임
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket
import threading
import time

class MotorInterface(Node):
    def __init__(self):
        super().__init__('motor_interface')

        # UDP 설정 -----------------------------------------------------------
        self.DEST = ('192.168.1.100', 7777)   # MCU IP, port
        self.BIND = ('192.168.1.2', 7777)   # 수신(IP,port) – PC(Nvidia Jetson) 주소
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.BIND)
        self.sock.settimeout(0.05)            # non-blocking recv
        
        # ROS I/O ------------------------------------------------------------
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, 'ctrl_cmd_boat',
            self.motor_interface_callback, 10)

        self.fb_pub = self.create_publisher(
            Float32MultiArray, 'ctrl_fb_boat', 10)

        # 피드백 수신 스레드 --------------------------------------------------
        self.recv_thread = threading.Thread(
            target=self._udp_feedback_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info('MotorInterface node started')

    # ──────────────────────────────────────────────────────────────────
    #  ROS → UDP : CTRL 명령 전송
    # ──────────────────────────────────────────────────────────────────
    def motor_interface_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(
                'ctrl_cmd_boat expects 4 elements [L_rps, R_rps, L_deg, R_deg]')
            return

        l_rps, r_rps, l_deg, r_deg = msg.data
        frame = (f"$CTRL,{l_rps},{r_rps},{l_deg},{r_deg}\r\n")
        try:
            self.sock.sendto(frame.encode('ascii'), self.DEST)
            self.get_logger().info(f'Sent: {frame.strip()}')
        except OSError as e:
            self.get_logger().error(f'UDP send error: {e}')

    # ──────────────────────────────────────────────────────────────────
    #  UDP → ROS : FB 피드백 수신
    # ──────────────────────────────────────────────────────────────────
    def _udp_feedback_loop(self):
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            self.get_logger().error(f'received data: {data}')

            try:
                line = data.decode('ascii').strip()
            except UnicodeDecodeError:
                continue

            if not line.startswith('$FB'):
                continue

            parts = line.split(',')
            if len(parts) < 7:
                continue  # malformed

            try:
                # ['$ FB', n, a, b, c, d, e]
                fb = Float32MultiArray()
                fb.data = [  # mode (int지만 float로 전달)
                           float(parts[2]),  # L_rps
                           float(parts[3]),  # R_rps
                           float(parts[4]),  # L_deg
                           float(parts[5])]  # R_deg
                           
                # 스레드-세이프 게시
                rclpy.executors.call_soon_threadsafe(
                    self.get_executor(), self.fb_pub.publish, fb)
            except ValueError:
                continue  # number conversion fail

    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        super().destroy_node()
        try:
            self.sock.close()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()