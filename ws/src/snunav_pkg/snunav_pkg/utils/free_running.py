import rclpy
from rclpy.node import Node
import numpy as np

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
        self.frequency = self.declare_parameter('common_params.frequency', 10.0).get_parameter_value().double_value
        self.dt = 1/self.frequency

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
        self._zigzag_direction = 0
        self._pivot_turn_params = {}
        self._crabbing_params = {}
        self._pull_out_params = {}
        self._pull_out_start = False
        self._pull_out_duration = 0.0
        self._pull_out_end = False
        self._spiral_params = {}
        self._spiral_target_del_idx = 0
        self._spiral_start = False
        self._spiral_duration = 0.0
        self._spiral_end = False

        self.get_logger().info(f'FreeRunning initialized with common_params:')
        
        self.get_logger().info(f'  del_rate: {self.del_rate}')
        self.get_logger().info(f'  rps_rate: {self.rps_rate}')
        self.get_logger().info(f'  rps_max: {self.rps_max}')
        self.get_logger().info(f'  del_max: {self.del_max}')
        self.get_logger().info(f'  deadzone_start: {self.deadzone_start}')
        self.get_logger().info(f'  deadzone_end: {self.deadzone_end}')

    def speed_mapping(self, ctrl, vel):
        # 한 번만 파라미터 로드
        if not self._speed_mapping_loaded:
            self._speed_mapping_params = {
                'len_timeseries': self.declare_parameter('free_running_mode.speed_mapping_mode.len_timeseries', 100).get_parameter_value().integer_value,
                'target_rps': self.declare_parameter('free_running_mode.speed_mapping_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': self.declare_parameter('free_running_mode.speed_mapping_mode.target_del', 0.0).get_parameter_value().double_value,
                'conv_u_tol': self.declare_parameter('free_running_mode.speed_mapping_mode.conv_u_tol', 0.05).get_parameter_value().double_value
            }
            self._speed_mapping_loaded = True
            
        rpsP, rpsS = ctrl[0], ctrl[1]
        
        target_rps = self._speed_mapping_params['target_rps']

        if abs(target_rps-rpsP) < self.rps_rate * self.dt:
            rpsP_cmd = target_rps
        else:
            rpsP_cmd = rpsP + np.sign(target_rps-rpsP)*self.rps_rate*self.dt
            
        rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)
            
        if abs(target_rps-rpsS) < self.rps_rate * self.dt:
            rpsS_cmd = target_rps
        else:
            rpsS_cmd = rpsS + np.sign(target_rps-rpsS)*self.rps_rate*self.dt
            
        rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)    
        
        ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, 0.0, 0.0])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd

    def turning(self, ctrl):
        # 한 번만 파라미터 로드
        if not self._turning_loaded:
            self._turning_params = {
                'target_rps': self.declare_parameter('free_running_mode.turning_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': self.declare_parameter('free_running_mode.turning_mode.target_del', 0.0).get_parameter_value().double_value
            }
            self._turning_loaded = True

        rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]

        target_rps = self._turning_params['target_rps']

        if abs(target_rps-rpsP) < self.rps_rate * self.dt:
            rpsP_cmd = target_rps
        else:
            rpsP_cmd = rpsP + np.sign(target_rps-rpsP)*self.rps_rate*self.dt
            
        rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)
            
        if abs(target_rps-rpsS) < self.rps_rate * self.dt:
            rpsS_cmd = target_rps
        else:
            rpsS_cmd = rpsS + np.sign(target_rps-rpsS)*self.rps_rate*self.dt
            
        rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max) 
        
        ##
        
        target_del = self._turning_params['target_del']

        if abs(target_del-delP) < self.del_rate * self.dt:
            delP_cmd = target_del
        else:
            delP_cmd = delP + np.sign(target_del-delP)*self.del_rate*self.dt

        delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)

        if abs(target_del-delS) < self.del_rate * self.dt:
            delS_cmd = target_del
        else:
            delS_cmd = delS + np.sign(target_del-delS)*self.del_rate*self.dt

        delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)

        ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd

    def zigzag(self, pos, ctrl):
        # 한 번만 파라미터 로드
        if not self._zigzag_loaded:
            self._zigzag_params = {
                'target_rps': self.declare_parameter('free_running_mode.zigzag_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_psi': self.declare_parameter('free_running_mode.zigzag_mode.target_psi', 0.0).get_parameter_value().double_value,
                'initial_psi_direction': self.declare_parameter('free_running_mode.zigzag_mode.initial_psi_direction', 1).get_parameter_value().integer_value
            }
            self._zigzag_loaded = True
        
        rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]
        psi = np.rad2deg(pos[2])  # pos[2]는 yaw 또는 heading을 나타낸다고 가정
        target_rps = self._zigzag_params['target_rps']
        target_psi = self._zigzag_params['target_psi']
        initial_psi_direction = self._zigzag_params['initial_psi_direction']
        
        if self._zigzag_direction == 0:
            self._zigzag_direction = initial_psi_direction

        if abs(psi) - abs(target_psi) >= 0 and psi * target_psi > 0:
            self._zigzag_direction *= -1  # 방향 전환
            direction_message = "left" if self._zigzag_direction < 0 else "right"
            self.get_logger().info(f'Zigzag direction changed to {direction_message}. Current psi: {psi}, Target psi: {target_psi}')
        
        if abs(target_rps-rpsP) < self.rps_rate * self.dt:
            rpsP_cmd = target_rps
        else:
            rpsP_cmd = rpsP + np.sign(target_rps-rpsP)*self.rps_rate*self.dt

        rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

        if abs(target_rps-rpsS) < self.rps_rate * self.dt:
            rpsS_cmd = target_rps
        else:
            rpsS_cmd = rpsS + np.sign(target_rps-rpsS)*self.rps_rate*self.dt

        rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)
        
        target_del = self.del_max * self._zigzag_direction
        if abs(target_del-delP) < self.del_rate * self.dt:
            delP_cmd = target_del
        else:
            delP_cmd = delP + np.sign(target_del-delP)*self.del_rate*self.dt
        delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)
        if abs(target_del-delS) < self.del_rate * self.dt:
            delS_cmd = target_del
        else:
            delS_cmd = delS + np.sign(target_del-delS)*self.del_rate*self.dt
        delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)
                
        ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
        # del_rate를 사용한 zigzag 로직 구현
        return ctrl_cmd

    def pivot_turn(self, ctrl):
        # 한 번만 파라미터 로드
        if not self._pivot_turn_loaded:
            self._pivot_turn_params = {
                'target_rpsP': self.declare_parameter('free_running_mode.pivot_turn_mode.target_rpsP', 0.0).get_parameter_value().double_value,
                'target_rpsS': self.declare_parameter('free_running_mode.pivot_turn_mode.target_rpsS', 0.0).get_parameter_value().double_value
            }
            self._pivot_turn_loaded = True

        rpsP, rpsS = ctrl[0], ctrl[1]
        
        target_rpsP = self._pivot_turn_params['target_rpsP']
        target_rpsS = self._pivot_turn_params['target_rpsS']

        if abs(target_rpsP-rpsP) < self.rps_rate * self.dt:
            rpsP_cmd = target_rpsP
        else:
            rpsP_cmd = rpsP + np.sign(target_rpsP-rpsP)*self.rps_rate*self.dt

        rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

        if abs(target_rpsS-rpsS) < self.rps_rate * self.dt:
            rpsS_cmd = target_rpsS
        else:
            rpsS_cmd = rpsS + np.sign(target_rpsS-rpsS)*self.rps_rate*self.dt

        rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)

        # deadzone_start, deadzone_end를 고려한 pivot turn 로직
        ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, 0.0, 0.0])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd

    def crabbing(self, ctrl):
        # 한 번만 파라미터 로드
        if not self._crabbing_loaded:
            self._crabbing_params = {
                'target_rpsP': self.declare_parameter('free_running_mode.crabbing_mode.target_rpsP', 0.0).get_parameter_value().double_value,
                'target_rpsS': self.declare_parameter('free_running_mode.crabbing_mode.target_rpsS', 0.0).get_parameter_value().double_value,
                'target_delP': self.declare_parameter('free_running_mode.crabbing_mode.target_delP', 0.0).get_parameter_value().double_value,
                'target_delS': self.declare_parameter('free_running_mode.crabbing_mode.target_delS', 0.0).get_parameter_value().double_value
            }
            self._crabbing_loaded = True
        rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]
        
        target_rpsP = self._crabbing_params['target_rpsP']
        target_rpsS = self._crabbing_params['target_rpsS']
        target_delP = self._crabbing_params['target_delP']
        target_delS = self._crabbing_params['target_delS']
        
        if abs(target_rpsP-rpsP) < self.rps_rate * self.dt:
            rpsP_cmd = target_rpsP
        else:
            rpsP_cmd = rpsP + np.sign(target_rpsP-rpsP)*self.rps_rate*self.dt

        rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

        if abs(target_rpsS-rpsS) < self.rps_rate * self.dt:
            rpsS_cmd = target_rpsS
        else:
            rpsS_cmd = rpsS + np.sign(target_rpsS-rpsS)*self.rps_rate*self.dt

        rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)

        ##

        if abs(target_delP-delP) < self.del_rate * self.dt:
            delP_cmd = target_delP
        else:
            delP_cmd = delP + np.sign(target_delP-delP)*self.del_rate*self.dt
        delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)

        if abs(target_delS-delS) < self.del_rate * self.dt:
            delS_cmd = target_delS
        else:
            delS_cmd = delS + np.sign(target_delS-delS)*self.del_rate*self.dt
        delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)

        ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
        
        # Implement the crabbing logic here
        return ctrl_cmd

    def pull_out(self, tick, ctrl):
        # 한 번만 파라미터 로드
        if not self._pull_out_loaded:
            self._pull_out_params = {
                'target_rps': self.declare_parameter('free_running_mode.pull_out_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': self.declare_parameter('free_running_mode.pull_out_mode.target_del', 0.0).get_parameter_value().double_value,
                'turning_duration': self.declare_parameter('free_running_mode.pull_out_mode.turning_duration', 0.0).get_parameter_value().double_value
            }
            self._pull_out_loaded = True
        
        rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]

        target_rps = self._pull_out_params['target_rps']
        target_del = self._pull_out_params['target_del']
        if tick.sec >= self._pull_out_duration:
            self.get_logger().info('Pull out completed.')
            self._pull_out_end = True
            target_del = 0.0
        if not self._pull_out_start:
            self._pull_out_start = True
            self._pull_out_duration = tick.sec + self._pull_out_params['turning_duration']
        
        if abs(target_rps-rpsP) < self.rps_rate * self.dt:
            rpsP_cmd = target_rps
        else:
            rpsP_cmd = rpsP + np.sign(target_rps-rpsP)*self.rps_rate*self.dt

        rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

        if abs(target_rps-rpsS) < self.rps_rate * self.dt:
            rpsS_cmd = target_rps
        else:
            rpsS_cmd = rpsS + np.sign(target_rps-rpsS)*self.rps_rate*self.dt

        rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)
        
        if abs(target_del-delP) < self.del_rate * self.dt:
            delP_cmd = target_del
        else:
            delP_cmd = delP + np.sign(target_del-delP)*self.del_rate*self.dt
        delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)
        if abs(target_del-delS) < self.del_rate * self.dt:
            delS_cmd = target_del
        else:
            delS_cmd = delS + np.sign(target_del-delS)*self.del_rate*self.dt
        delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)
                
        ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
        # Implement the pull out logic here
        return ctrl_cmd

    def spiral(self, tick, ctrl):
        # 한 번만 파라미터 로드
        if not self._spiral_loaded:
            self._spiral_params = {
                'target_rps': self.declare_parameter('free_running_mode.spiral_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': self.declare_parameter('free_running_mode.spiral_mode.target_del', [20.0, 10.0, 5.0]).get_parameter_value().double_array_value,
                'turning_duration': self.declare_parameter('free_running_mode.spiral_mode.turning_duration', 0.0).get_parameter_value().double_value
            }
            self._spiral_loaded = True
            
        rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]

        target_rps = self._turning_params['target_rps']
        
        target_del_list = self._spiral_params['target_del']
        target_rps = self._pull_out_params['target_rps']
        
        if self._spiral_target_del_idx == len(target_del_list):
            self.get_logger().info('Spiral completed.')
            self._spiral_end = True
            target_del = 0.0
            target_rps = 0.0
            
        while self._spiral_target_del_idx < len(target_del_list):
            target_del = target_del_list[self._spiral_target_del_idx]
            if tick.sec >= self._spiral_duration:
                self.get_logger().info(f'Spiral angle {target_del} completed.')
                self._spiral_target_del_idx += 1
                self._spiral_start = False
            if not self._spiral_start:
                self._spiral_start = True
                self._spiral_duration = tick.sec + self._spiral_params['turning_duration']
        
        if abs(target_rps-rpsP) < self.rps_rate * self.dt:
            rpsP_cmd = target_rps
        else:
            rpsP_cmd = rpsP + np.sign(target_rps-rpsP)*self.rps_rate*self.dt

        rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

        if abs(target_rps-rpsS) < self.rps_rate * self.dt:
            rpsS_cmd = target_rps
        else:
            rpsS_cmd = rpsS + np.sign(target_rps-rpsS)*self.rps_rate*self.dt

        rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)
        
        if abs(target_del-delP) < self.del_rate * self.dt:
            delP_cmd = target_del
        else:
            delP_cmd = delP + np.sign(target_del-delP)*self.del_rate*self.dt
        delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)
        if abs(target_del-delS) < self.del_rate * self.dt:
            delS_cmd = target_del
        else:
            delS_cmd = delS + np.sign(target_del-delS)*self.del_rate*self.dt
        delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)
        
        ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
            # Implement the spiral logic here
        # Implement the spiral logic here
        return ctrl_cmd

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