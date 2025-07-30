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

from std_msgs.msg import String
from snumsg_pkg.msg import MissionCode, Sensor
import numpy as np
from utils.free_running import FreeRunning
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
        self.__missionCodeParser = self.create_subscription(
            MissionCode,
            'mission_code',
            self.mission_callback,
            10)

    def mission_callback(self, msg):
        mission_code = int(msg.mission_code, 16)
        self.mission_code = (mission_code & 0x0000FFFF)
        self.controllerMode()
        self.get_logger().info('Received maneuver code: "%s"' % hex(self.mission_code))

    def controllerMode(self):
        maneuver_mode = (self.mission_code & 0x0000F000) >> 12
        if maneuver_mode == 0x0:
            self.freeRunningController()
        elif maneuver_mode == 0x1:
            self.dockingController()
        elif maneuver_mode == 0x2:
            self.DPController()
        elif maneuver_mode == 0x3:
            self.PPController()
        else:
            raise ValueError("Unknown maneuver mode: {}".format(hex(maneuver_mode)))
    
    def freeRunningController(self):
        submaneuver_mode = (self.mission_code & 0x00000FF0) >> 4
        if submaneuver_mode == 0x00:
            target_rps = 0.0
            FreeRunning.speed_mapping(target_rps, self.vel)
        elif submaneuver_mode == 0x10:
            control = FreeRunning.turning(self.pos, self.ctrl)
        elif submaneuver_mode == 0x20:
            control = FreeRunning.zigzag(self.pos, self.ctrl)
        elif submaneuver_mode == 0x30:
            control = FreeRunning.pivot_turn(self.pos, self.ctrl)
        elif submaneuver_mode == 0x40:
            control = FreeRunning.crabbing(self.pos, self.ctrl)
        elif submaneuver_mode == 0x50:
            control = FreeRunning.pull_out(self.pos, self.ctrl)
        elif submaneuver_mode == 0x60:
            control = FreeRunning.spiral(self.pos, self.ctrl)
        else:
            raise ValueError("Unknown submaneuver mode: {}".format(hex(submaneuver_mode)))

    def dockingController(self):
        submaneuver_mode = (self.mission_code & 0x00000FF0) >> 4
        if submaneuver_mode == 0x00:
            self.heuristicDocking_Entering()
        elif submaneuver_mode == 0x10:
            self.DRLDocking_Entering()
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
        submaneuver_mode = (self.mission_code & 0x00000FF0) >> 4
        if submaneuver_mode == 0x00:
            self.DP_controlMethod1()
        elif submaneuver_mode == 0x10:
            self.DP_controlMethod2()
        else:
            raise ValueError("Unknown DP submaneuver mode: {}".format(hex(submaneuver_mode)))
    
    def PPController(self):
        submaneuver_mode = (self.mission_code & 0x00000FF0) >> 4
        if submaneuver_mode == 0x00:
            self.PP_algorithm1()
        elif submaneuver_mode == 0x10:
            self.PP_algorithm2()
        else:
            raise ValueError("Unknown PP submaneuver mode: {}".format(hex(submaneuver_mode)))
    

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
