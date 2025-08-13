import rclpy, math
from rclpy.node import Node
import numpy as np
from scipy.interpolate import interp1d

def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

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
        self.U_tol = self.declare_parameter('common_params.U_tol', 0.05).get_parameter_value().double_value
        self.U_queue_len = self.declare_parameter('common_params.U_queue_len', 10).get_parameter_value().integer_value
        self.target_rps_set = self.declare_parameter('common_params.target_rps_set', [10.0, 20.0, 30.0, 40.0, 50.0]).get_parameter_value().double_array_value
        self.target_U_set = self.declare_parameter('common_params.target_U_set', [0.2, 0.4, 0.5, 0.6, 0.7]).get_parameter_value().double_array_value
        self._interp_rps_U = interp1d(self.target_rps_set, self.target_U_set, bounds_error=False, fill_value='extrapolate', kind='linear')

        # 각 함수별 파라미터 로드 상태 추적
        self._speed_mapping_loaded = False
        self._turning_loaded = False
        self._zigzag_loaded = False
        self._pivot_turn_loaded = False
        self._crabbing_loaded = False
        self._pull_out_loaded = False
        self._spiral_loaded = False
        self._random_bangbang_loaded = False
        self._random_3321_loaded = False
        self._random_3211_loaded = False
        self._random_211_loaded = False

        self._steady_state_time = -1
        self._steady_state_status = False

        self._max_rps_time = -1
        self._max_rps_time_status = False

        self.U_queue = np.array([], dtype=np.float64)  # Control command queue for averaging

        # 각 함수별 파라미터 저장 변수들
        self._speed_mapping_params = {}
        self._speed_mapping_start = False
        self._speed_mapping_duration = 0.0
        self._speed_mapping_end = False
        
        self._turning_params = {}
        self._turning_start = False
        self._turning_duration = 0.0
        self._turning_end = False
        
        self._zigzag_params = {}
        self._zigzag_direction = 0
        self._zigzag_start = False
        self._zigzag_duration = 0.0
        self._zigzag_end = False
        
        self._pivot_turn_params = {}
        self._pivot_turn_start = False
        self._pivot_turn_duration = 0.0
        self._pivot_turn_end = False

        self._crabbing_params = {}
        self._crabbing_start = False
        self._crabbing_duration = 0.0
        self._crabbing_end = False
        
        self._pull_out_params = {}
        self._pull_out_start = False
        self._pull_out_duration = 0.0
        self._pull_out_end = False
        
        self._spiral_params = {}
        self._spiral_target_del_idx = 0
        self._spiral_start = False
        self._spiral_duration = 0.0
        self._spiral_end = False

        self._random_bangbang_params = {}
        self._random_bangbang_duration = 0.0
        self._random_bangbang_start = False
        self._random_bangbang_end = False
        self._random_bangbang_repeat_count = 0

        self._random_3321_params = {}
        self._random_3321_duration = 0.0
        self._random_3321_start = False
        self._random_3321_end = False
        self._random_3321_repeat_count = 0

        self._random_3211_params = {} 
        self._random_3211_duration = 0.0
        self._random_3211_start = False
        self._random_3211_end = False
        self._random_3211_repeat_count = 0

        self._random_211_params = {} 
        self._random_211_duration = 0.0
        self._random_211_start = False
        self._random_211_end = False
        self._random_211_repeat_count = 0

        self.get_logger().info(f'FreeRunning initialized with common_params:')
        
        self.get_logger().info(f'  del_rate: {self.del_rate}')
        self.get_logger().info(f'  rps_rate: {self.rps_rate}')
        self.get_logger().info(f'  rps_max: {self.rps_max}')
        self.get_logger().info(f'  del_max: {self.del_max}')
        self.get_logger().info(f'  deadzone_start: {self.deadzone_start}')
        self.get_logger().info(f'  deadzone_end: {self.deadzone_end}')

    def to_max_rps(self, tick, vel, duration, target_rps):
        if self._max_rps_time == -1:
            self._max_rps_time = tick + duration
        if tick < self._max_rps_time:
            # rpsP_cmd = target_rps  
            # rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max) 
            # rpsS_cmd = target_rps  
            # rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)    
            # ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, 0.0, 0.0])
            ctrl_cmd = np.array([target_rps, target_rps, 0.0, 0.0])
        return ctrl_cmd

    def to_steady_state(self, tick, target_rps, vel):
        # U_mean = np.mean(self.U_queue) if len(self.U_queue) > 0 else 0.0
        U_mean = self.U_queue[-1]
        target_U = self._interp_rps_U(target_rps)
        # target_U = 0.0181*target_rps - 0.0452 # LC3
        self.get_logger().info(f'To steady state: tick={tick}, U_mean={U_mean}, target_U={target_U}, target_rps={target_rps}')
        # if abs(U_mean - target_U) < self.U_tol:
        if U_mean > target_U - self.U_tol:
            self._steady_state_status = True
            self.get_logger().info(f'##### Steady state achieved at tick {tick} with U_mean: {U_mean}, target_U: {target_U} #####')
        rpsP_cmd = target_rps
        rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)
        rpsS_cmd = target_rps
        rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)
        # delP_cmd = np.round(-200.0 * vel[2], 0) # for LC3 Fn 0.1
        # delP_cmd = np.round(-50.0 * vel[2], 0) # for LC3 Fn 0.15
        # delP_cmd = np.round(-200.0 * vel[2], 0) # for LC1 Fn 0.1
        delP_cmd = np.round(-50.0 * vel[2], 0) # for LC1 Fn 0.15
        delS_cmd = delP_cmd
        ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd
    
    def speed_mapping(self, tick, pos, vel, ctrl):
        # 한 번만 파라미터 로드
        if not self._speed_mapping_loaded:
            self._speed_mapping_params = {
                'len_timeseries': self.declare_parameter('free_running_mode.speed_mapping_mode.len_timeseries', 100).get_parameter_value().integer_value,
                'target_rps': self.declare_parameter('free_running_mode.speed_mapping_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': self.declare_parameter('free_running_mode.speed_mapping_mode.target_del', 0.0).get_parameter_value().double_value,
                'conv_u_tol': self.declare_parameter('free_running_mode.speed_mapping_mode.conv_u_tol', 0.05).get_parameter_value().double_value,
                'duration': self.declare_parameter('free_running_mode.speed_mapping_mode.duration', 0.0).get_parameter_value().double_value
            }
            self._speed_mapping_loaded = True

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U

        if not self._speed_mapping_start:
            self._speed_mapping_start = True
            self._speed_mapping_duration = tick + self._speed_mapping_params['duration']
        if tick >= self._speed_mapping_duration:
            self.get_logger().info('Speed mapping completed.')
            self._speed_mapping_end = True
            target_rps = 0.0
        
        rpsP, rpsS = ctrl[0], ctrl[1]
        
        target_rps = self._speed_mapping_params['target_rps']

        rpsP_cmd = target_rps
            
        rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)
            
        rpsS_cmd = target_rps
            
        rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)    
        
        ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, 0.0, 0.0])  # [rpsP, rpsS, delP, delS]
        
        if self._max_rps_time == -1 or self._max_rps_time > tick:
            ctrl_cmd = self.to_max_rps(tick, vel, 4.0, 25)
            self.get_logger().info('Transitioning to max RPS before turning.')
        return ctrl_cmd, self._speed_mapping_end

    def turning(self, tick, pos, vel, ctrl):
        # 한 번만 파라미터 로드
        if not self._turning_loaded:
            self._turning_params = {
                'target_rps': self.declare_parameter('free_running_mode.turning_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': self.declare_parameter('free_running_mode.turning_mode.target_del', 0.0).get_parameter_value().double_value,
                'duration': self.declare_parameter('free_running_mode.turning_mode.duration', 0.0).get_parameter_value().double_value
            }
            self._turning_loaded = True

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U

        # if self._steady_state_time == -1 or self._steady_state_time > tick:
        #     ctrl_cmd = self.to_steady_state(tick, self._turning_params['target_rps'])
        #     self.get_logger().info('Transitioning to steady state before turning.')
        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._turning_params['target_rps'], vel)
            if self._max_rps_time == -1 or self._max_rps_time > tick:
                ctrl_cmd = self.to_max_rps(tick, vel, 3.7, 35)
                self.get_logger().info('Transitioning to max RPS before turning.')
        else:
            if not self._turning_start:
                self._turning_start = True
                self._turning_duration = tick + self._turning_params['duration']
                self.get_logger().info(f'Turning started with duration: {self._turning_params["duration"]} seconds')
            if tick >= self._turning_duration:
                self.get_logger().info('Turning completed.')
                self._turning_end = True
                target_rps = 0.0
                target_del = 0.0
                
            rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]
            target_rps = self._turning_params['target_rps']

            rpsP_cmd = target_rps       
            rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

            rpsS_cmd = target_rps          
            rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max) 
            
            ##
            
            target_del = self._turning_params['target_del']

            delP_cmd = target_del
            delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)

            delS_cmd = target_del
            delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)

            ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]

            self.get_logger().info(f'Turning control command: {ctrl_cmd}')
        return ctrl_cmd, self._turning_end

    def zigzag(self, tick, pos, vel, ctrl):
        # 한 번만 파라미터 로드
        if not self._zigzag_loaded:
            self._zigzag_params = {
                'target_rps': self.declare_parameter('free_running_mode.zigzag_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_psi': self.declare_parameter('free_running_mode.zigzag_mode.target_psi', 0.0).get_parameter_value().double_value,
                'initial_psi_direction': self.declare_parameter('free_running_mode.zigzag_mode.initial_psi_direction', 1).get_parameter_value().integer_value,
                'duration': self.declare_parameter('free_running_mode.zigzag_mode.duration', 0.0).get_parameter_value().double_value
            }
            self.target_del = None
            self._zigzag_loaded = True

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U
        
        # if self._steady_state_time == -1 or self._steady_state_time > tick:
        #     ctrl_cmd = self.to_steady_state(tick, self._zigzag_params['target_rps'])
        #     self.get_logger().info('Transitioning to steady state before zigzag.')
        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._zigzag_params['target_rps'], vel)
            if self._max_rps_time == -1 or self._max_rps_time > tick:
                ctrl_cmd = self.to_max_rps(tick, vel, 3.7, 35)
                self.get_logger().info('Transitioning to max RPS before zigzag.')
        else:
            if self.target_del is None:
                self.target_del = self._zigzag_params['target_psi'] * self._zigzag_params['initial_psi_direction']
                self.get_logger().info(f'target_del: {self.target_del}, dir: {self._zigzag_params["initial_psi_direction"]}') 
            rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]
            psi = (pos[2])  # pos[2]는 yaw 또는 heading을 나타낸다고 가정
            target_rps = self._zigzag_params['target_rps']
            
            initial_psi_direction = self._zigzag_params['initial_psi_direction']
            
            if not self._zigzag_start:
                self._zigzag_start = True
                self._zigzag_direction = initial_psi_direction
                self._zigzag_duration = tick + self._zigzag_params['duration']
                self._initial_zigzag_psi = psi
            if tick >= self._zigzag_duration:
                self.get_logger().info('Zigzag completed.')
                self._zigzag_end = True
                target_rps = 0.0
                self.target_del = 0.0
                
            if self._zigzag_direction == 0:
                self._zigzag_direction = initial_psi_direction
        
            target_psi = wrap_to_pi(self._initial_zigzag_psi + np.deg2rad(self._zigzag_params['target_psi']) * self._zigzag_direction)
            if wrap_to_pi(abs(psi - self._initial_zigzag_psi) - abs(target_psi - self._initial_zigzag_psi)) >= 0 and (psi - self._initial_zigzag_psi) * (target_psi - self._initial_zigzag_psi) > 0:
                self._zigzag_direction *= -1  # 방향 전환
                self.target_del *= -1
                direction_message = "right" if self._zigzag_direction < 0 else "left"
                self.get_logger().info(f'Zigzag direction changed to {direction_message}. Current psi: {np.rad2deg(psi)}, Target psi: {np.rad2deg(target_psi)}')

            rpsP_cmd = target_rps
            rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

            rpsS_cmd = target_rps
            rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)
            
            delP_cmd = self.target_del
            delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)
            
            delS_cmd = self.target_del
            delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)
            self.get_logger().info(f'now_psi: {np.rad2deg(psi)},target_del: {(self.target_del)}, target_psi: {np.rad2deg(target_psi)} ')
            ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
            # del_rate를 사용한 zigzag 로직 구현
        return ctrl_cmd, self._zigzag_end

    def pivot_turn(self, tick, pos, vel, ctrl):
        # 한 번만 파라미터 로드
        if not self._pivot_turn_loaded:
            self._pivot_turn_params = {
                'target_rpsP': self.declare_parameter('free_running_mode.pivot_turn_mode.target_rpsP', 0.0).get_parameter_value().double_value,
                'target_rpsS': self.declare_parameter('free_running_mode.pivot_turn_mode.target_rpsS', 0.0).get_parameter_value().double_value,
                'duration': self.declare_parameter('free_running_mode.pivot_turn_mode.duration', 0.0).get_parameter_value().double_value
            }
            self._pivot_turn_loaded = True

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U

        # if self._steady_state_time == -1 or self._steady_state_time > tick:
        #     ctrl_cmd = self.to_steady_state(tick, self._pivot_turn_params['target_rps'])
        #     self.get_logger().info('Transitioning to steady state before pivot turn.')
        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._pivot_turn_params['target_rpsP'], vel)
        else:
            if not self._pivot_turn_start:
                self._pivot_turn_start = True
                self._pivot_turn_duration = tick + self._pivot_turn_params['duration']
            if tick >= self._pivot_turn_duration:
                self.get_logger().info('Pivot turn completed.')
                self._pivot_turn_end = True
                rpsP_cmd = 0.0
                rpsS_cmd = 0.0
                delP_cmd = 0.0
                delS_cmd = 0.0
                
            rpsP, rpsS = ctrl[0], ctrl[1]
            
            target_rpsP = self._pivot_turn_params['target_rpsP']
            target_rpsS = self._pivot_turn_params['target_rpsS']

            rpsP_cmd = target_rpsP
            rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

            rpsS_cmd = target_rpsS
            rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)

            # deadzone_start, deadzone_end를 고려한 pivot turn 로직
            ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, 0.0, 0.0])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd, self._pivot_turn_end

    def crabbing(self, tick, pos, vel, ctrl):
        # 한 번만 파라미터 로드
        if not self._crabbing_loaded:
            self._crabbing_params = {
                'target_rpsP': self.declare_parameter('free_running_mode.crabbing_mode.target_rpsP', 0.0).get_parameter_value().double_value,
                'target_rpsS': self.declare_parameter('free_running_mode.crabbing_mode.target_rpsS', 0.0).get_parameter_value().double_value,
                'target_delP': self.declare_parameter('free_running_mode.crabbing_mode.target_delP', 0.0).get_parameter_value().double_value,
                'target_delS': self.declare_parameter('free_running_mode.crabbing_mode.target_delS', 0.0).get_parameter_value().double_value,
                'duration': self.declare_parameter('free_running_mode.crabbing_mode.duration', 0.0).get_parameter_value().double_value
            }
            self._crabbing_loaded = True

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U

        # if self._steady_state_time == -1 or self._steady_state_time > tick:
        #     ctrl_cmd = self.to_steady_state(tick, self._crabbing_params['target_rps'])
        #     self.get_logger().info('Transitioning to steady state before crabbing.')
        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._crabbing_params['target_rpsP'], vel)
        else:
            if not self._crabbing_start:
                self._crabbing_start = True
                self._crabbing_duration = tick + self._crabbing_params['duration']
            if tick >= self._crabbing_duration:
                self.get_logger().info('Crabbing completed.')
                self._crabbing_end = True
                target_delP = 0.0
                target_delS = 0.0
                target_rpsP = 0.0
                target_rpsS = 0.0
                
            rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]
            
            target_rpsP = self._crabbing_params['target_rpsP']
            target_rpsS = self._crabbing_params['target_rpsS']
            target_delP = self._crabbing_params['target_delP']
            target_delS = self._crabbing_params['target_delS']

            rpsP_cmd = target_rpsP
            rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)


            rpsS_cmd = target_rpsS
            rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)

            delP_cmd = target_delP
            delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)

            delS_cmd = target_delS
            delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)

            ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
            
            # Implement the crabbing logic here
        return ctrl_cmd, self._crabbing_end

    def pull_out(self, tick, pos, vel, ctrl):
        # 한 번만 파라미터 로드
        if not self._pull_out_loaded:
            self._pull_out_params = {
                'target_rps': self.declare_parameter('free_running_mode.pull_out_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': self.declare_parameter('free_running_mode.pull_out_mode.target_del', 0.0).get_parameter_value().double_value,
                'duration_turning': self.declare_parameter('free_running_mode.pull_out_mode.duration_turning', 0.0).get_parameter_value().double_value,
                'duration_neutral': self.declare_parameter('free_running_mode.pull_out_mode.duration_neutral', 0.0).get_parameter_value().double_value
            }
            self._pull_out_loaded = True
            self._pull_out_duration_turning = tick + self._pull_out_params['duration_turning']

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U
            
        # if self._steady_state_time == -1 or self._steady_state_time > tick:
        #     ctrl_cmd = self.to_steady_state(tick, self._pull_out_params['target_rps'])
        #     self.get_logger().info('Transitioning to steady state before pull out.')
        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._pull_out_params['target_rps'], vel)
        else:
            if not self._pull_out_start:
                self._pull_out_start = True
                self._pull_out_duration_turning = tick + self._pull_out_params['duration_turning']
                self._pull_out_duration_neutral = self._pull_out_duration_turning + self._pull_out_params['duration_neutral']
            
            rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]

            target_rps = self._pull_out_params['target_rps']
            target_del = self._pull_out_params['target_del']
            if tick >= self._pull_out_duration_turning:
                self.get_logger().info('Pull out completed.')
                target_del = 0.0
            if tick >= self._pull_out_duration_neutral:
                self.get_logger().info('Pull out neutral phase started.')
                self._pull_out_end = True
                target_del = 0.0
                target_rps = 0.0
            
            rpsP_cmd = target_rps
            rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

            rpsS_cmd = target_rps
            rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)

            delP_cmd = target_del
            delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)

            delS_cmd = target_del
            delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)
                    
            ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
            # Implement the pull out logic here
        return ctrl_cmd, self._pull_out_end

    def spiral(self, tick, pos, vel, ctrl):
        # 한 번만 파라미터 로드
        if not self._spiral_loaded:
            self._spiral_params = {
                'target_rps': self.declare_parameter('free_running_mode.spiral_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': self.declare_parameter('free_running_mode.spiral_mode.target_del', [20.0, 10.0, 5.0]).get_parameter_value().double_array_value,
                'duration': self.declare_parameter('free_running_mode.spiral_mode.duration', 0.0).get_parameter_value().double_value
            }
            self._spiral_duration = tick + self._spiral_params['duration']
            self._spiral_loaded = True

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U
        
        # if self._steady_state_time == -1 or self._steady_state_time > tick:
        #     ctrl_cmd = self.to_steady_state(tick, self._spiral_params['target_rps'])
        #     self.get_logger().info('Transitioning to steady state before spiral.')
        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._spiral_params['target_rps'], vel)
        else:
            rpsP, rpsS, delP, delS = ctrl[0], ctrl[1], ctrl[2], ctrl[3]
            
            target_del_list = self._spiral_params['target_del']
            target_rps = self._spiral_params['target_rps']
            
            if self._spiral_target_del_idx == len(target_del_list):
                self.get_logger().info('Spiral completed.')
                self._spiral_end = True
                target_del = 0.0
                target_rps = 0.0
                
            if self._spiral_target_del_idx < len(target_del_list):
                target_del = target_del_list[self._spiral_target_del_idx]
                if tick >= self._spiral_duration:
                    self.get_logger().info(f'Spiral angle {target_del} completed.')
                    self._spiral_target_del_idx += 1
                    self._spiral_start = False
                if not self._spiral_start:
                    self._spiral_start = True
                    self._spiral_duration = tick + self._spiral_params['duration']
                    self.get_logger().info(f'Spiral started with target delta: {target_del}, duration: {self._spiral_params["duration"]} seconds')
            
            rpsP_cmd = target_rps
            rpsP_cmd = np.clip(rpsP_cmd, -self.rps_max, self.rps_max)

            rpsS_cmd = target_rps
            rpsS_cmd = np.clip(rpsS_cmd, -self.rps_max, self.rps_max)

            delP_cmd = target_del
            delP_cmd = np.clip(delP_cmd, -self.del_max, self.del_max)

            delS_cmd = target_del
            delS_cmd = np.clip(delS_cmd, -self.del_max, self.del_max)

            ctrl_cmd = np.array([rpsP_cmd, rpsS_cmd, delP_cmd, delS_cmd])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd, self._spiral_end

    def random_bangbang(self, tick, pos, vel, ctrl):
        # 1) 파라미터 로드
        if not self._random_bangbang_loaded:
            p = self.declare_parameter
            self._random_bangbang_params = {
                'target_rps': p('free_running_mode.bangbang_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': p('free_running_mode.bangbang_mode.target_del', 0.0).get_parameter_value().double_value,
                'initial_direction': p('free_running_mode.bangbang_mode.initial_psi_direction', 1).get_parameter_value().integer_value,
                'duration': p('free_running_mode.bangbang_mode.duration', 0.0).get_parameter_value().double_value,
                'repeat': p('free_running_mode.bangbang_mode.repeat', 1).get_parameter_value().integer_value
            }
            self._random_bangbang_direction = self._random_bangbang_params['initial_direction']
            self._random_bangbang_loaded = True
            self.get_logger().info(f'Random bangbang parameters loaded: {self._random_bangbang_params}')

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U

        if self._random_bangbang_end:
            # return np.zeros(4), True
            return np.array([self._random_bangbang_params['target_rps'], self._random_bangbang_params['target_rps'], 0.0, 0.0]), True

        # if self._steady_state_time == -1 or tick < self._steady_state_time:
        #     ctrl_cmd = self.to_steady_state(tick, vel,
        #                      self._random_bangbang_params['target_rps'])
        #     self.get_logger().info(
        #         'Transitioning to steady state before random_bangbang.'
        #     )
        #     return ctrl_cmd, False

        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._random_bangbang_params['target_rps'], vel)
            if self._max_rps_time == -1 or self._max_rps_time > tick:
                # ctrl_cmd = self.to_max_rps(tick, vel, 3.7, 35)
                ctrl_cmd = self.to_max_rps(tick, vel, 4.0, 44.0)
                self.get_logger().info('Transitioning to max RPS before random_bangbang.')
        else:
            # 2) 초기화 (첫 사이클 시작)
            if not self._random_bangbang_start:
                self._random_bangbang_start = True
                self._random_bangbang_end = False
                self._random_bangbang_repeat_count = 0
                self._random_bangbang_duration = tick + self._random_bangbang_params['duration']
                self.get_logger().info(f'Random bangbang started: duration={self._random_bangbang_params["duration"]}, repeat={self._random_bangbang_params["repeat"]}, direction={self._random_bangbang_direction}')

            # 3) 사이클 완료 여부 확인
            if tick >= self._random_bangbang_duration:
                self._random_bangbang_repeat_count += 1
                self.get_logger().info(f'Random bangbang cycle {self._random_bangbang_repeat_count} completed')
                if self._random_bangbang_repeat_count < self._random_bangbang_params['repeat']:
                    # 방향 전환
                    self._random_bangbang_direction *= -1
                    self._random_bangbang_duration = tick + self._random_bangbang_params['duration']
                    self.get_logger().info(f'Random bangbang direction changed to {self._random_bangbang_direction}')
                else:
                    self._random_bangbang_end = True
                    self._random_bangbang_start = False
                    self.get_logger().info('Random bangbang completed all repeats')
                    # return np.zeros(4), True
                    return np.array([self._random_bangbang_params['target_rps'], self._random_bangbang_params['target_rps'], 0.0, 0.0]), False

            # 4) 제어 명령 생성
            target_rps = self._random_bangbang_params['target_rps']
            target_del = self._random_bangbang_params['target_del'] * self._random_bangbang_direction
            rps_cmd = np.clip(target_rps, -self.rps_max, self.rps_max)
            del_cmd = np.clip(target_del, -self.del_max, self.del_max)
            ctrl_cmd = np.array([rps_cmd, rps_cmd, del_cmd, del_cmd])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd, False

    def random_3321(self, tick, pos, vel, ctrl):
        # 1) 파라미터 로드
        if not self._random_3321_loaded:
            p = self.declare_parameter
            self._random_3321_params = {
                'target_rps': p('free_running_mode.random_mode.3321_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': p('free_running_mode.random_mode.3321_mode.target_del', 0.0).get_parameter_value().double_value,
                'initial_direction': p('free_running_mode.random_mode.3321_mode.initial_psi_direction', 1).get_parameter_value().integer_value,
                'duration': p('free_running_mode.random_mode.3321_mode.duration', 0.0).get_parameter_value().double_value,
                'repeat': p('free_running_mode.random_mode.3321_mode.repeat', 1).get_parameter_value().integer_value
            }
            self._random_3321_direction = self._random_3321_params['initial_direction']
            self._random_3321_loaded = True
            self.get_logger().info(f'Random 3321 parameters loaded: {self._random_3321_params}')

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U

        # if self._steady_state_time == -1 or tick < self._steady_state_time:
        #     ctrl_cmd = self.to_steady_state(tick, vel,
        #                      self._random_3321_params['target_rps'])
        #     self.get_logger().info(
        #         'Transitioning to steady state before random_3321.'
        #     )
        #     return ctrl_cmd, False

        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._random_3321_params['target_rps'], vel)

        if self._random_3321_end:
            return np.array([self._random_3321_params['target_rps'], self._random_3321_params['target_rps'], 0.0, 0.0]), True

        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._random_bangbang_params['target_rps'], vel)
            if self._max_rps_time == -1 or self._max_rps_time > tick:
                ctrl_cmd = self.to_max_rps(tick, vel, 3.7, 35)
                # ctrl_cmd = self.to_max_rps(tick, vel, 4.0, 44.0)
                self.get_logger().info('Transitioning to max RPS before random_3321.')
        # 2) 초기화 (첫 사이클 시작)
        else:
            if not self._random_3321_start:
                self._random_3321_start = True
                self._random_3321_end = False
                self._random_3321_repeat_count = 0
                # 첫 3T duration
                self._random_3321_duration = tick + self._random_3321_params['duration'] * 3
            self.get_logger().info(f'Random 3321 started: repeat={self._random_3321_params["repeat"]}, direction={self._random_3321_direction}')

            # 3) 사이클 완료 여부 확인
            if tick >= self._random_3321_duration:
                self._random_3321_repeat_count += 1
                self.get_logger().info(f'Random 3321 cycle {self._random_3321_repeat_count} completed')
                if self._random_3321_repeat_count < self._random_3321_params['repeat']:
                    # 다음 기간 설정(3T→2T→1T)
                    mult = {1: 3, 2: 2}.get(self._random_3321_repeat_count, 1)
                    self._random_3321_duration = tick + self._random_3321_params['duration'] * mult
                    # 방향 전환
                    self._random_3321_direction *= -1
                    self.get_logger().info(f'Random 3321 direction changed to {self._random_3321_direction}, next duration multiplier={mult}')
                else:
                    self._random_3321_end = True
                    self._random_3321_start = False
                    self.get_logger().info('Random 3321 completed all repeats')
                    return np.zeros(4), True

            # 4) 제어 명령 생성
            target_rps = self._random_3321_params['target_rps']
            target_del = self._random_3321_params['target_del'] * self._random_3321_direction
            rps_cmd = np.clip(target_rps, -self.rps_max, self.rps_max)
            del_cmd = np.clip(target_del, -self.del_max, self.del_max)
            ctrl_cmd = np.array([rps_cmd, rps_cmd, del_cmd, del_cmd])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd, False

    def random_3211(self, tick, pos, vel, ctrl):
        # 1) 파라미터 로드
        if not self._random_3211_loaded:
            p = self.declare_parameter
            self._random_3211_params = {
                'target_rps': p('free_running_mode.random_mode.3211_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': p('free_running_mode.random_mode.3211_mode.target_del', 0.0).get_parameter_value().double_value,
                'initial_direction': p('free_running_mode.random_mode.3211_mode.initial_psi_direction', 1).get_parameter_value().integer_value,
                'duration': p('free_running_mode.random_mode.3211_mode.duration', 0.0).get_parameter_value().double_value,
                'repeat': p('free_running_mode.random_mode.3211_mode.repeat', 1).get_parameter_value().integer_value
            }
            self._random_3211_direction = self._random_3211_params['initial_direction']
            self._random_3211_loaded = True
            self.get_logger().info(f'Random 3211 parameters loaded: {self._random_3211_params}')

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U

        # if self._steady_state_status == False:
        #     ctrl_cmd = self.to_steady_state(tick, self._random_3211_params['target_rps'], vel)

        # if self._steady_state_time == -1 or tick < self._steady_state_time:
        #     ctrl_cmd = self.to_steady_state(tick, vel,
        #                      self._random_3211_params['target_rps'])
        #     self.get_logger().info(
        #         'Transitioning to steady state before random_3211.'
        #     )
        #     return ctrl_cmd, False

        if self._random_3211_end:
            return np.array([self._random_3211_params['target_rps'], self._random_3211_params['target_rps'], 0.0, 0.0]), True

        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._random_3211_params['target_rps'], vel)
            if self._max_rps_time == -1 or self._max_rps_time > tick:
                ctrl_cmd = self.to_max_rps(tick, vel, 3.7, 35)
                # ctrl_cmd = self.to_max_rps(tick, vel, 4.0, 44.0)
                self.get_logger().info('Transitioning to max RPS before random : 3211 test.')
        else:
            # 2) 초기화 (첫 사이클 시작)
            if not self._random_3211_start:
                self._random_3211_start = True
                self._random_3211_end = False
                self._random_3211_repeat_count = 0
                # 첫 3T duration
                self._random_3211_duration = tick + self._random_3211_params['duration'] * 3
                self.get_logger().info(f'Random 3211 started: repeat={self._random_3211_params["repeat"]}, direction={self._random_3211_direction}')

            # 3) 사이클 완료 여부 확인
            if tick >= self._random_3211_duration:
                self._random_3211_repeat_count += 1
                self.get_logger().info(f'Random 3211 cycle {self._random_3211_repeat_count} completed')
                if self._random_3211_repeat_count < self._random_3211_params['repeat']:
                    # 다음 기간 설정(3T→2T→1T)
                    mult = {1: 2, 2: 1}.get(self._random_3211_repeat_count, 1)
                    self._random_3211_duration = tick + self._random_3211_params['duration'] * mult
                    # 방향 전환
                    self._random_3211_direction *= -1
                    self.get_logger().info(f'Random 3211 direction changed to {self._random_3211_direction}, next duration multiplier={mult}')
                else:
                    self._random_3211_end = True
                    self._random_3211_start = False
                    self.get_logger().info('Random 3211 completed all repeats')
                    return np.zeros(4), True

            # 4) 제어 명령 생성
            target_rps = self._random_3211_params['target_rps']
            target_del = self._random_3211_params['target_del'] * self._random_3211_direction
            rps_cmd = np.clip(target_rps, -self.rps_max, self.rps_max)
            del_cmd = np.clip(target_del, -self.del_max, self.del_max)
            ctrl_cmd = np.array([rps_cmd, rps_cmd, del_cmd, del_cmd])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd, False
    
    def random_211(self, tick, pos, vel, ctrl):
        # 1) 파라미터 로드
        if not self._random_211_loaded:
            p = self.declare_parameter
            self._random_211_params = {
                'target_rps': p('free_running_mode.random_mode.211_mode.target_rps', 0.0).get_parameter_value().double_value,
                'target_del': p('free_running_mode.random_mode.211_mode.target_del', 0.0).get_parameter_value().double_value,
                'initial_direction': p('free_running_mode.random_mode.211_mode.initial_psi_direction', 1).get_parameter_value().integer_value,
                'duration': p('free_running_mode.random_mode.211_mode.duration', 0.0).get_parameter_value().double_value,
                'repeat': p('free_running_mode.random_mode.211_mode.repeat', 1).get_parameter_value().integer_value
            }
            self._random_211_direction = self._random_211_params['initial_direction']
            self._random_211_loaded = True
            self.get_logger().info(f'Random 211 parameters loaded: {self._random_211_params}')

        # update U_queue
        U = np.sqrt(vel[0]**2 + vel[1]**2)

        if len(self.U_queue) < self.U_queue_len:
            self.U_queue = np.append(self.U_queue, U)
        else:
            self.U_queue = np.roll(self.U_queue, -1)
            self.U_queue[-1] = U

        # if self._steady_state_status == False:
        #     ctrl_cmd = self.to_steady_state(tick, self._random_3211_params['target_rps'], vel)

        # if self._steady_state_time == -1 or tick < self._steady_state_time:
        #     ctrl_cmd = self.to_steady_state(tick, vel,
        #                      self._random_3211_params['target_rps'])
        #     self.get_logger().info(
        #         'Transitioning to steady state before random_3211.'
        #     )
        #     return ctrl_cmd, False

        if self._random_211_end:
            return np.array([self._random_211_params['target_rps'], self._random_211_params['target_rps'], 0.0, 0.0]), True

        if self._steady_state_status == False:
            ctrl_cmd = self.to_steady_state(tick, self._random_211_params['target_rps'], vel)
            if self._max_rps_time == -1 or self._max_rps_time > tick:
                # ctrl_cmd = self.to_max_rps(tick, vel, 3.7, 35)
                ctrl_cmd = self.to_max_rps(tick, vel, 4.0, 43.0)
                self.get_logger().info('Transitioning to max RPS before random : 211 test.')
        else:
            # 2) 초기화 (첫 사이클 시작)
            if not self._random_211_start:
                self._random_211_start = True
                self._random_211_end = False
                self._random_211_repeat_count = 0
                # 첫 3T duration
                self._random_211_duration = tick + self._random_211_params['duration'] * 2
                self.get_logger().info(f'Random 211 started: repeat={self._random_211_params["repeat"]}, direction={self._random_211_direction}')

            # 3) 사이클 완료 여부 확인
            if tick >= self._random_211_duration:
                self._random_211_repeat_count += 1
                self.get_logger().info(f'Random 211 cycle {self._random_211_repeat_count} completed')
                if self._random_211_repeat_count < self._random_211_params['repeat']:
                    # 다음 기간 설정(2T→1T→1T)
                    mult = {1:1}.get(self._random_211_repeat_count, 1)
                    self._random_211_duration = tick + self._random_211_params['duration'] * mult
                    # 방향 전환
                    self._random_211_direction *= -1
                    self.get_logger().info(f'Random 211 direction changed to {self._random_211_direction}, next duration multiplier={mult}')
                else:
                    self._random_211_end = True
                    # self._random_211_end = False
                    self._random_211_start = False
                    self.get_logger().info('Random 211 completed all repeats')
                    # return np.zeros(4), True
                    return np.array([self._random_211_params['target_rps'], self._random_211_params['target_rps'], 0.0, 0.0]), False

            # 4) 제어 명령 생성
            target_rps = self._random_211_params['target_rps']
            target_del = self._random_211_params['target_del'] * self._random_211_direction
            rps_cmd = np.clip(target_rps, -self.rps_max, self.rps_max)
            del_cmd = np.clip(target_del, -self.del_max, self.del_max)
            ctrl_cmd = np.array([rps_cmd, rps_cmd, del_cmd, del_cmd])  # [rpsP, rpsS, delP, delS]
        return ctrl_cmd, False

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