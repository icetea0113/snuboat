"""
ROS 2 MotorInterface node
  • 구독:  /ctrl_cmd_boat  (Float32MultiArray, [L_rps, R_rps, L_deg, R_deg])
  • 발행:  /ctrl_fb_boat   (Float32MultiArray, [mode, L_rps_fb, R_rps_fb,
                                             L_deg_fb, R_deg_fb, err_code])
  • 전송:  UDP $CTRL 프레임
  • 전송:  UDP $SET  프레임
  • 수신:  UDP $FB   프레임
"""
import rclpy
from rclpy.node import Node
import numpy as np
import random
from std_msgs.msg import Float32MultiArray
from snumsg_pkg.msg import Control  # 변경: Float32MultiArray → Control
import socket
import threading
import time

class MotorInterface(Node):
    def __init__(self):
        super().__init__('motor_interface')

        self.del_rate = self.declare_parameter('common_params.del_rate', 0.1).get_parameter_value().double_value
        self.get_logger().info(f'Declared del_rate: {self.del_rate} deg/s')
        # UDP 설정 -----------------------------------------------------------
        self.DEST = ('127.0.0.1', 7777)   # MCU IP, port
        self.BIND = ('127.0.0.1', 7777)   # 수신(IP,port) – PC(Nvidia Jetson) 주소
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.BIND)
        self.sock.settimeout(0.05)            # non-blocking recv
        
        frame = (f"$SET,{self.del_rate}\r\n")
        self.ctrl_cmd = [0.0, 0.0, 0.0, 0.0]  # 초기값 설정
        self.ctrl_fb = [0.0, 0.0, 0.0, 0.0]  # 피드백 초기값 설정
        try:
            self.sock.sendto(frame.encode('ascii'), self.DEST)
            self.get_logger().info(f'Sent: {frame.strip()}')
        except OSError as e:
            self.get_logger().error(f'UDP send error: {e}')
        
        # ROS I/O ------------------------------------------------------------
        self.cmd_sub = self.create_subscription(
            Control, 'ctrl_cmd_boat',
            self.motor_interface_callback, 10)

        self.fb_pub = self.create_publisher(
            Control, 'ctrl_fb_boat', 10)

        # 피드백 수신 스레드 --------------------------------------------------
        self.recv_thread = threading.Thread(
            target=self._udp_feedback_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info('MotorInterface node started')

    # ──────────────────────────────────────────────────────────────────
    #  ROS → UDP : CTRL 명령 전송
    # ──────────────────────────────────────────────────────────────────
    def motor_interface_callback(self, msg: Control):
        if not msg.ctrl or len(msg.ctrl) != 4:
            self.get_logger().warn(
                'ctrl_cmd_boat expects 4 elements [L_rps, R_rps, L_deg, R_deg]')
            return
        
        self.ctrl_cmd = msg.ctrl  # 저장 (추후 사용 가능)
        self.motor_worker_cmd()
        l_rps, r_rps, l_deg, r_deg = self.ctrl_cmd
        # frame = (f"$CTRL,{l_rps},{r_rps},{l_deg},{r_deg}\r\n")
        frame = (f"$FB,1,{l_rps},{r_rps},{l_deg},{r_deg},0\r\n")
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
                fb = Control()
                fb.tick = self.get_clock().now().to_msg()
                fb.ctrl = [ 
                           float(parts[2]),  # L_rps
                           float(parts[3]),  # R_rps
                           float(parts[4]),  # L_deg
                           float(parts[5])]  # R_deg
                
                fb.ctrl = self.motor_worker_fb(fb)
                self.get_logger().info(f'Feedback received: {fb.ctrl}')
                self.fb_pub.publish(fb)

            except ValueError:
                continue  # number conversion fail

    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        super().destroy_node()
        try:
            self.sock.close()
        except Exception:
            pass
             
        
        
    def motor_worker_cmd(self):
        if(self.ctrl_cmd[0] > 0.1):
            self.ctrl_cmd[0] = 0.8563 * self.ctrl_cmd[0] + 5.8419 + random.random()/1000
        elif(self.ctrl_cmd[0] < -0.1):
            self.ctrl_cmd[0] = -(0.8563 * abs(self.ctrl_cmd[0]) + 5.8419) + random.random()/1000
        
        if(self.ctrl_cmd[1] > 0.1):
            self.ctrl_cmd[1] = 0.8563 * self.ctrl_cmd[1] + 5.8419 + random.random()/1000
        elif(self.ctrl_cmd[1] < -0.1):
            self.ctrl_cmd[1] = -(0.8563 * abs(self.ctrl_cmd[1]) + 5.8419) + random.random()/1000
    def motor_worker_fb(self, fb):
        # update rps
        if(fb.ctrl[0] > 0.1):
            self.ctrl_fb[0] = 1.1677 * fb.ctrl[0] - 6.8189
        elif(fb.ctrl[0] < -0.1):
            self.ctrl_fb[0] = -(1.1677 * abs(fb.ctrl[0]) - 6.8189)

        if(fb.ctrl[1] > 0.1):
            self.ctrl_fb[1] = 1.1677 * fb.ctrl[1] - 6.8189
        elif(fb.ctrl[1] < -0.1):
            self.ctrl_fb[1] = -(1.1677 * abs(fb.ctrl[1]) - 6.8189)

        # update steering
        if abs(fb.ctrl[2]-self.ctrl_fb[2]) < self.del_rate*0.1:
            self.ctrl_fb[2] = fb.ctrl[2]
        else:
            self.ctrl_fb[2] = np.sign(fb.ctrl[2]-self.ctrl_fb[2])*self.del_rate*0.1 + self.ctrl_fb[2]+ random.random()/1000
        if abs(fb.ctrl[3]-self.ctrl_fb[3]) < self.del_rate*0.1:
            self.ctrl_fb[3] = fb.ctrl[3]
        else:
            self.ctrl_fb[3] = np.sign(fb.ctrl[3]-self.ctrl_fb[3])*self.del_rate*0.1 + self.ctrl_fb[3]+ random.random()/1000
        self.ctrl_fb[2] = np.clip(self.ctrl_fb[2] , -30.0, 30.0)
        self.ctrl_fb[3] = np.clip(self.ctrl_fb[3] , -30.0, 30.0)
        return self.ctrl_fb
        

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