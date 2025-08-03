This is ROS 2 Humble Workspace folder.

# 1. 워크스페이스 루트로 이동
cd ~/snu_boat_ws

# 2. 의존성 업데이트 및 특정 패키지(my_pkg)만 빌드
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select snumsg_pkg snunav_pkg --symlink-install

# 3. 빌드 결과 적용 at the path of ws(둘 중 하나만 하여도 됨)
source ./install/setup.bash
source ./install/local_setup.bash

source ~/.bashrc

# 4. roslaunch
ros2 launch snunav_pkg sils.launch.py

# If you want to run SILS mode, please change field named sils_mode 0 to 1 in params.yaml