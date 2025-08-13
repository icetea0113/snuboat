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

from std_msgs.msg import String, Float32MultiArray, Int32
from snumsg_pkg.msg import MissionCode, Sensor, Control
import numpy as np
from snunav_pkg.utils.free_running import FreeRunning
import time
# from docking import heuristicDocking_Entering, DRLDocking_Entering, DRL_Approach, DRL_Turning, DRL_Entering, DRL_Pushing, heuristicDocking_Approaching, heuristicDocking_Turning, heuristicDocking_Entering, heuristicDocking_Pushing
# from DP import DP_controlMethod1, DP_controlMethod2
# from PP import PP_algorithm1, PP_algorithm2

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.pos = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.ctrl_cmd = np.array([0.0, 0.0, 0.0, 0.0])  # [rpsP, delP, rpsS, delS]
        self.ctrl = np.array([0.0, 0.0, 0.0, 0.0])
        self.status = 0                                 # 0 = init, 1 = run, 2 = pause, 3 = done
        self.mission_code = 0x00000000
        self.tick = 0
        self.__missionCodeParser = self.create_subscription(
            MissionCode,
            'mission_code',
            self.mission_callback,
            10)
        self.__silscontrol_subscriber = self.create_subscription(
            Control,
            'sils_motor_fb_data',
            self.sils_callback,
            10)
        self.__sensor_subscriber = self.create_subscription(
            Sensor,
            'sensor',
            self.sensor_callback,
            10)
        self.__ctrlcmdboatPublisher = self.create_publisher(
            Control,
            'ctrl_cmd_boat',
            10)
        self.__ctrlcmdsilsPublisher = self.create_publisher(
            Control,
            'ctrl_cmd_sils',
            10)
        self.status_publisher = self.create_publisher(
            Int32,
            'ctrl_status',
            10)
        
        self.free_running = FreeRunning()
    
    def sils_callback(self, msg):
        if len(msg.ctrl) == 4:
            self.ctrl[0] = float(msg.ctrl[0])
            self.ctrl[1] = float(msg.ctrl[1])
            self.ctrl[2] = float(msg.ctrl[2])
            self.ctrl[3] = float(msg.ctrl[3])
            # self.get_logger().info(f'SILS control feedback received: '
            #                           f'Port RPS: {self.ctrl[0]}, Stbd RPS: {self.ctrl[1]}, '
            #                           f'Port Steer: {self.ctrl[2]}, Stbd Steer: {self.ctrl[3]}')
        else:
            self.get_logger().error('Invalid SILS control command received. Expected 4 values.')
            
    def mission_callback(self, msg):
        self.tick = self.get_time_seconds(msg.tick)
        mission_code = int(msg.mission_code, 16)
        self.mission_code = (mission_code & 0x000FFF0)
        self.is_sils = (mission_code & 0xF000000) >> 24
        # self.get_logger().info('SILS mode: %s' % self.is_sils)
        self.controllerMode()
        # self.get_logger().info('Received maneuver code: "%s"' % hex(self.mission_code))

    def sensor_callback(self, msg):
            self.pos[0] = float(msg.pose[0])
            self.pos[1] = float(msg.pose[1])
            self.pos[2] = float(msg.pose[5])
            self.vel[0] = float(msg.vel[0])
            self.vel[1] = float(msg.vel[1])
            self.vel[2] = float(msg.vel[5])
            # self.get_logger().info(f'Sensor data received: '
            #                           f'Position: {self.pos}, Velocity: {self.vel}')
    def controllerMode(self):
        maneuver_mode = (self.mission_code & 0x00F000) >> 12
        if maneuver_mode == 0x1:
            self.freeRunningController()
            # self.ctrl_cmd = np.array([-15.0, -15.0, 30.0, 30.0])  # Example control command
        elif maneuver_mode == 0x2:
            self.dockingController()
        elif maneuver_mode == 0x3:
            self.DPController()
        elif maneuver_mode == 0x4:
            self.PPController()
        else:
            raise ValueError("Unknown maneuver mode: {}".format(hex(maneuver_mode)))

        status_msg = Int32()
        status_msg.data = self.status
        self.status_publisher.publish(status_msg)
        # self.get_logger().info(f'Published status command: {self.status}')

        ctrl_msg = Control()
        ctrl_msg.ctrl = self.ctrl_cmd.tolist()
        ctrl_msg.tick = self.get_clock().now().to_msg()
        
        if self.is_sils == 1:
            self.__ctrlcmdsilsPublisher.publish(ctrl_msg)
            # self.get_logger().info(f'Published control command to SILS: {self.ctrl_cmd}')
        else:
            self.__ctrlcmdboatPublisher.publish(ctrl_msg)
            # self.get_logger().info(f'Published control command to boat: {self.ctrl_cmd}')
        # self.__ctrlcmdboatPublisher.publish(ctrl_msg)
        # self.get_logger().info(f'Published control command to boat: {self.ctrl_cmd}')

    def freeRunningController(self):
        # sils_mode, motor_mode, sensor_mode, maneuver_mode, sub_maneuver_mode, subsub_maneuver_mode, status
        ctrl_cmd = np.zeros(4)  # [rpsP, rpsS, delP, delS]
        submaneuver_mode    = (self.mission_code & 0x0F00) >> 8
        subsub_maneuver_mode = (self.mission_code & 0x00F0) >> 4

        # print("submaneuver_mode: {}, subsub_maneuver_mode: {}".format(hex(submaneuver_mode), 
        # hex(subsub_maneuver_mode)))

        state = 0
        if submaneuver_mode == 0x0:
            ctrl_cmd, state = self.free_running.speed_mapping(self.tick, self.pos, self.vel, self.ctrl)
        elif submaneuver_mode == 0x1:
            ctrl_cmd, state = self.free_running.turning(self.tick, self.pos, self.vel, self.ctrl)
        elif submaneuver_mode == 0x2:
            # self.get_logger().info(f'pos: {self.pos}, vel: {self.vel}, ctrl: {self.ctrl}')
            ctrl_cmd, state = self.free_running.zigzag(self.tick, self.pos, self.vel, self.ctrl)
        elif submaneuver_mode == 0x3:
            ctrl_cmd, state = self.free_running.pivot_turn(self.tick, self.pos, self.vel, self.ctrl)
        elif submaneuver_mode == 0x4:
            ctrl_cmd, state = self.free_running.crabbing(self.tick, self.pos, self.vel, self.ctrl)
        elif submaneuver_mode == 0x5:
            ctrl_cmd, state = self.free_running.pull_out(self.tick, self.pos, self.vel, self.ctrl)
        elif submaneuver_mode == 0x6:
            # self.get_logger().info(f'pos: {self.pos}, vel: {self.vel}, ctrl: {self.ctrl}')
            ctrl_cmd, state = self.free_running.spiral(self.tick, self.pos, self.vel, self.ctrl)
        elif submaneuver_mode == 0x7:
            # self.get_logger().info(f'pos: {self.pos}, vel: {self.vel}, ctrl: {self.ctrl}')
            ctrl_cmd, state = self.free_running.random_bangbang(self.tick, self.pos, self.vel, self.ctrl)
        elif submaneuver_mode == 0x8:
            if subsub_maneuver_mode == 0x0:
                # self.get_logger().info(f'pos: {self.pos}, vel: {self.vel}, ctrl: {self.ctrl}')
                ctrl_cmd, state = self.free_running.random_3321(self.tick, self.pos, self.vel, self.ctrl)
            elif subsub_maneuver_mode == 0x1:
                # self.get_logger().info(f'pos: {self.pos}, vel: {self.vel}, ctrl: {self.ctrl}')
                ctrl_cmd, state = self.free_running.random_3211(self.tick, self.pos, self.vel, self.ctrl)
            elif subsub_maneuver_mode == 0x2:
                # self.get_logger().info(f'pos: {self.pos}, vel: {self.vel}, ctrl: {self.ctrl}')
                ctrl_cmd, state = self.free_running.random_211(self.tick, self.pos, self.vel, self.ctrl)
            else:
                self.get_logger().error(f'Unknown subsub maneuver mode: {subsub_maneuver_mode}')
                raise ValueError("Unknown subsub maneuver mode: {}".format(hex(subsub_maneuver_mode)))
        else:
            self.status = 2  # pause
            raise ValueError("Unknown submaneuver mode: {}".format(hex(submaneuver_mode)))

        # print(f'Controller state: {state}, tick: {self.tick}, ctrl_cmd: {ctrl_cmd}')

        if self.status == 0:  # init
            self.status = 1
        if state == 1:
            self.status = 3
            if not (submaneuver_mode == 0x8 or submaneuver_mode == 0x7):
                ctrl_cmd = np.zeros(4)  # Stop the controller
            # ctrl_cmd = np.zeros(4)
            
        self.ctrl_cmd = ctrl_cmd

    def dockingController(self):
        submaneuver_mode = (self.mission_code & 0x0000FF0) >> 4
        if submaneuver_mode == 0x00:
            pass
            # self.heuristicDocking_Entering()
        elif submaneuver_mode == 0x10:
            pass
            # self.DRLDocking_Entering()
        elif submaneuver_mode == 0x20:
            stage = submaneuver_mode & 0x0F
            if stage == 0x01:
                self.DRL_Approach()
            elif stage == 0x02:
                self.DRL_Turning()
            elif stage == 0x03:
                self.DRL_Entering()
            elif stage == 0x04:
                self.DRL_Pushing()
            else:
                raise ValueError("Unknown DRL docking stage: {}".format(hex(stage)))
        elif submaneuver_mode == 0x30:
            stage = submaneuver_mode & 0x0F
            if stage == 0x01:
                self.heuristicDocking_Approaching()
            elif stage == 0x02:
                self.heuristicDocking_Turning()
            elif stage == 0x03:
                self.heuristicDocking_Entering()
            elif stage == 0x04:
                self.heuristicDocking_Pushing()
        else:
            raise ValueError("Unknown docking submaneuver mode: {}".format(hex(submaneuver_mode)))
    
    def DPController(self):
        submaneuver_mode = (self.mission_code & 0x0000FF0) >> 4
        if submaneuver_mode == 0x00:
            self.DP_controlMethod1()
        elif submaneuver_mode == 0x10:
            self.DP_controlMethod2()
        else:
            raise ValueError("Unknown DP submaneuver mode: {}".format(hex(submaneuver_mode)))
    
    def PPController(self):
        submaneuver_mode = (self.mission_code & 0x0000FF0) >> 4
        if submaneuver_mode == 0x00:
            self.PP_algorithm1()
        elif submaneuver_mode == 0x10:
            self.PP_algorithm2()
        else:
            raise ValueError("Unknown PP submaneuver mode: {}".format(hex(submaneuver_mode)))
    
    def get_time_seconds(self, time_msg):
            return time_msg.sec + time_msg.nanosec * 1e-9

    def time_diff_seconds(self, time1, time2):
        return self.get_time_seconds(time1) - self.get_time_seconds(time2)
    
def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
