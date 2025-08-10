import math
import socket
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

@dataclass
class CurmcMsg:
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    yaw_deg: Optional[float] = None
    u: Optional[float] = None
    status: Optional[str] = None

def nmea_checksum_ok(sentence: str) -> bool:
    try:
        star = sentence.rindex('*')
        body = sentence[1:star]
        given = sentence[star + 1:].strip()
        csum = 0
        for ch in body:
            csum ^= ord(ch)
        return f"{csum:02X}" == given.upper()
    except Exception:
        return True

def _to_float(s: str) -> Optional[float]:
    s = s.strip()
    if s == "" or s.upper() == "NULL":
        return None
    try:
        return float(s)
    except ValueError:
        return None

def parse_curmc(line: str) -> Optional[CurmcMsg]:
    """
    0: $CURMC
    1: GMT (hhmmss.ss)  
    2: Status (A/V)
    3: X coordinate  [m]
    4: Y coordinate  [m]
    5: Z coordinate  [m]
    6: Roll angle    [deg]
    7: Pitch angle   [deg]
    8: Ship speed    [m/s]
    9: Significant wave height  
    10: Water temperature 
    11: Course over ground [deg]
    12: UTC Date (ddmmyy) 
    13: Magnetic variation
    14: Checksum (after '*')
    """
    line = line.strip()
    if not line.startswith('$CURMC'):
        print(f"Invalid line format: {line}")
        return None
    if '*' in line and not nmea_checksum_ok(line):
        print(f"Invalid checksum in line: {line}")
        return None

    core = line.split('*')[0]
    parts = [p.strip() for p in core.split(',')]
    if len(parts) < 12:
        return None

    return CurmcMsg(
        status=parts[2] if len(parts) > 2 else None,
        x=_to_float(parts[3]),
        y=_to_float(parts[4]),
        z=_to_float(parts[5]),
        roll_deg=_to_float(parts[6]),
        pitch_deg=_to_float(parts[7]),
        u=_to_float(parts[8]),
        yaw_deg=_to_float(parts[11])
    )

def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

class UdpCurmcNode(Node):
    def __init__(self):
        super().__init__('udp_curmc_node')

        self.port = self.declare_parameter('port', 56001).get_parameter_value().integer_value
        self.bind_address = self.declare_parameter('bind_address', '0.0.0.0').get_parameter_value().string_value
        self.use_raw_speed = self.declare_parameter('publish_raw_speed', True).get_parameter_value().bool_value

        self.pub = self.create_publisher(Float32MultiArray, 'qualisys_data', 10)

        self.prev_t = None
        self.prev_x = None
        self.prev_y = None
        self.prev_psi = None
        self.prev_roll = None
        self.prev_pitch = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.bind_address, self.port))
        self.sock.settimeout(1.0)

        self.alive = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

        self.get_logger().info(f"Listening UDP {self.bind_address}:{self.port}")

    def _recv_loop(self):
        while self.alive:
            try:
                data, _ = self.sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break

            for sentence in data.decode(errors='ignore').strip().splitlines():
                msg = parse_curmc(sentence)
                self.get_logger().debug(f"Received: {sentence}")
                if msg:
                    self._handle_curmc(msg)

    def _handle_curmc(self, m: CurmcMsg):
        now = self.get_clock().now().nanoseconds * 1e-9
        psi = wrap_to_pi(math.radians(m.yaw_deg)) if m.yaw_deg is not None else None
        roll_rad = math.radians(m.roll_deg) if m.roll_deg is not None else None
        pitch_rad = math.radians(m.pitch_deg) if m.pitch_deg is not None else None

        # u,v 계산
        u_calc, v_calc = None, None
        if (self.prev_t is not None
            and m.x is not None and m.y is not None
            and self.prev_x is not None and self.prev_y is not None
            and psi is not None):
            dt = now - self.prev_t
            if dt > 1e-3:
                vx = (m.x - self.prev_x) / dt
                vy = (m.y - self.prev_y) / dt
                u_calc = math.cos(psi) * vx + math.sin(psi) * vy
                v_calc = -math.sin(psi) * vx + math.cos(psi) * vy
        u_out = m.u if (self.use_raw_speed and m.u is not None) else (u_calc if u_calc is not None else float('nan'))
        v_out = v_calc if v_calc is not None else float('nan')
        z_out = m.z if m.z is not None else float('nan')
        r_out, p_out, q_out = float('nan'), float('nan'), float('nan')
        if psi is not None and self.prev_psi is not None and self.prev_t is not None:
            dt = now - self.prev_t
            if dt > 1e-3:
                r_out = wrap_to_pi(psi - self.prev_psi) / dt
        if roll_rad is not None and self.prev_roll is not None and self.prev_t is not None:
            dt = now - self.prev_t
            if dt > 1e-3:
                p_out = (roll_rad - self.prev_roll) / dt
        if pitch_rad is not None and self.prev_pitch is not None and self.prev_t is not None:
            dt = now - self.prev_t
            if dt > 1e-3:
                q_out = (pitch_rad - self.prev_pitch) / dt

        arr = Float32MultiArray()
        arr.data = [
            1.0 if m.status == 'A' else (0.0 if m.status == 'V' else float('nan')),
            m.x if m.x is not None else float('nan'),
            m.y if m.y is not None else float('nan'),
            m.z if m.z is not None else float('nan'),
            m.roll_deg if m.roll_deg is not None else float('nan'),
            m.pitch_deg if m.pitch_deg is not None else float('nan'),
            psi if psi is not None else float('nan'),
            u_out,
            v_out,
            z_out,
            p_out,
            q_out,
            r_out,
        ]
        self.pub.publish(arr)
        self.get_logger().info(f"Published: {arr.data}")
        # 상태 업데이트
        self.prev_t = now
        if m.x is not None: self.prev_x = m.x
        if m.y is not None: self.prev_y = m.y
        if psi is not None: self.prev_psi = psi
        if roll_rad is not None: self.prev_roll = roll_rad
        if pitch_rad is not None: self.prev_pitch = pitch_rad

def main(args=None):
    rclpy.init(args=args)
    node = UdpCurmcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.alive = False
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()
