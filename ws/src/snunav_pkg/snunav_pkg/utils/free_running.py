import rclpy
from rclpy.node import Node

class FreeRunning(Node):
    def __init__(self):
        super().__init__('free_running_node')
        # common_params에서 파라미터들을 불러오기
        self.del_rate = self.declare_parameter('common_params.del_rate', 0.1).get_parameter_value().double_value
        self.rps_rate = self.declare_parameter('common_params.rps_rate', 0.1).get_parameter_value().double_value
        self.rps_max = self.declare_parameter('common_params.rps_max', 2.0).get_parameter_value().double_value
        self.del_max = self.declare_parameter('common_params.del_max', 30.0).get_parameter_value().double_value
        self.deadzone_start = self.declare_parameter('common_params.deadzone_start', 1).get_parameter_value().integer_value
        self.deadzone_end = self.declare_parameter('common_params.deadzone_end', 1).get_parameter_value().integer_value

        # 각 함수별 파라미터 로드 상태 추적
        self._speed_mapping_loaded = False
        self._turning_loaded = False
        self._zigzag_loaded = False
        self._pivot_turn_loaded = False
        self._crabbing_loaded = False
        self._pull_out_loaded = False
        self._spiral_loaded = False

        # 각 함수별 파라미터 저장 변수들
        self._speed_mapping_params = {}
        self._turning_params = {}
        self._zigzag_params = {}
        self._pivot_turn_params = {}
        self._crabbing_params = {}
        self._pull_out_params = {}
        self._spiral_params = {}

        self.get_logger().info(f'FreeRunning initialized with common_params:')
        
        self.get_logger().info(f'  del_rate: {self.del_rate}')
        self.get_logger().info(f'  rps_rate: {self.rps_rate}')
        self.get_logger().info(f'  rps_max: {self.rps_max}')
        self.get_logger().info(f'  del_max: {self.del_max}')
        self.get_logger().info(f'  deadzone_start: {self.deadzone_start}')
        self.get_logger().info(f'  deadzone_end: {self.deadzone_end}')

    def speed_mapping(self, rps_cmd, vel):
        # 한 번만 파라미터 로드
        if not self._speed_mapping_loaded:
            self._speed_mapping_params = {
                'len_timeseries': self.declare_parameter('free_running_mode.speed_mapping_mode.len_timeseries', 100).get_parameter_value().integer_value,
                'rps_cmd': self.declare_parameter('free_running_mode.speed_mapping_mode.rps_cmd', 0.0).get_parameter_value().double_value,
                'del_cmd': self.declare_parameter('free_running_mode.speed_mapping_mode.del_cmd', 0.0).get_parameter_value().double_value,
                'conv_u_tol': self.declare_parameter('free_running_mode.speed_mapping_mode.conv_u_tol', 0.05).get_parameter_value().double_value
            }
            self._speed_mapping_loaded = True
        
        # rps_max, rps_rate 등의 파라미터를 사용한 로직 구현
        limited_rps = min(abs(rps_cmd), self.rps_max) * (1 if rps_cmd >= 0 else -1)
        return limited_rps

    def turning(self, ctrl):
        # 한 번만 파라미터 로드
        if not self._turning_loaded:
            self._turning_params = {
                'rps_cmd': self.declare_parameter('free_running_mode.turning_mode.rps_cmd', 0.0).get_parameter_value().double_value,
                'del_cmd': self.declare_parameter('free_running_mode.turning_mode.del_cmd', 0.0).get_parameter_value().double_value
            }
            self._turning_loaded = True
        
        # del_max 파라미터를 사용한 조향 로직 구현
        limited_ctrl = max(-self.del_max, min(self.del_max, ctrl))
        return limited_ctrl

    def zigzag(self, target_psi, pos, ctrl):
        # 한 번만 파라미터 로드
        if not self._zigzag_loaded:
            self._zigzag_params = {
                'rps_cmd': self.declare_parameter('free_running_mode.zigzag_mode.rps_cmd', 0.0).get_parameter_value().double_value,
                'target_psi': self.declare_parameter('free_running_mode.zigzag_mode.target_psi', 0.0).get_parameter_value().double_value,
                'initial_psi_direction': self.declare_parameter('free_running_mode.zigzag_mode.initial_psi_direction', 1).get_parameter_value().integer_value
            }
            self._zigzag_loaded = True
        
        # del_rate를 사용한 zigzag 로직 구현
        pass

    def pivot_turn(self, rpsP_cmd, rpsS_cmd):
        # 한 번만 파라미터 로드
        if not self._pivot_turn_loaded:
            self._pivot_turn_params = {
                'rpsP_cmd': self.declare_parameter('free_running_mode.pivot_turn_mode.rpsP_cmd', 0.0).get_parameter_value().double_value,
                'rpsS_cmd': self.declare_parameter('free_running_mode.pivot_turn_mode.rpsS_cmd', 0.0).get_parameter_value().double_value
            }
            self._pivot_turn_loaded = True
        
        # deadzone_start, deadzone_end를 고려한 pivot turn 로직
        pass

    def crabbing(self):
        # 한 번만 파라미터 로드
        if not self._crabbing_loaded:
            self._crabbing_params = {
                'rpsP_cmd': self.declare_parameter('free_running_mode.crabbing_mode.rpsP_cmd', 0.0).get_parameter_value().double_value,
                'rpsS_cmd': self.declare_parameter('free_running_mode.crabbing_mode.rpsS_cmd', 0.0).get_parameter_value().double_value,
                'delP_cmd': self.declare_parameter('free_running_mode.crabbing_mode.delP_cmd', 0.0).get_parameter_value().double_value,
                'delS_cmd': self.declare_parameter('free_running_mode.crabbing_mode.delS_cmd', 0.0).get_parameter_value().double_value
            }
            self._crabbing_loaded = True
        
        # Implement the crabbing logic here
        pass

    def pull_out(self):
        # 한 번만 파라미터 로드
        if not self._pull_out_loaded:
            self._pull_out_params = {
                'rps_cmd': self.declare_parameter('free_running_mode.pull_out_mode.rps_cmd', 0.0).get_parameter_value().double_value,
                'del_cmd': self.declare_parameter('free_running_mode.pull_out_mode.del_cmd', 0.0).get_parameter_value().double_value,
                'turning_duration': self.declare_parameter('free_running_mode.pull_out_mode.turning_duration', 0.0).get_parameter_value().double_value
            }
            self._pull_out_loaded = True
        
        # Implement the pull out logic here
        pass

    def spiral(self):
        # 한 번만 파라미터 로드
        if not self._spiral_loaded:
            self._spiral_params = {
                'rps_cmd': self.declare_parameter('free_running_mode.spiral_mode.rps_cmd', 0.0).get_parameter_value().double_value,
                'del_cmd': self.declare_parameter('free_running_mode.spiral_mode.del_cmd', [20.0, 10.0, 5.0]).get_parameter_value().double_array_value,
                'turning_duration': self.declare_parameter('free_running_mode.spiral_mode.turning_duration', 0.0).get_parameter_value().double_value
            }
            self._spiral_loaded = True
        
        # Implement the spiral logic here
        pass

def main(args=None):
    rclpy.init(args=args)
    free_running = FreeRunning()
    
    try:
        rclpy.spin(free_running)
    except KeyboardInterrupt:
        pass
    finally:
        free_running.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()