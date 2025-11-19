

sudo apt update
sudo apt install ros-humble-desktop

sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros

sudo apt install \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-navigation2
  
sudo apt install ros-humble-tf2-tools

sudo apt install python3-pyqt5 python3-pyqt5.qtquick



실행 명령어
ros2 launch tb3_security_system full_system.launch.py

가제보 안열릴때
sudo apt install --reinstall ros-humble-gazebo-ros-pkgs
pkill -f gzserver
pkill -f gzclient
source ~/turtlebot3_ws/install/setup.bash

그래도 가제보 안열릴 때
rm -rf ~/.gazebo/model_cache/*
LAUNCH_FILE=~/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/launch/turtlebot3_world.launch.py
cp $LAUNCH_FILE ${LAUNCH_FILE}.bak
sudo apt install --reinstall ros-humble-gazebo-ros-pkgs
pkill -f gzserver
pkill -f gzclient
source ~/turtlebot3_ws/install/setup.bash

그래도 가제보 안열릴때 2
ps -aux | grep gazebo 로 2번째 pid 숫자 확인
kill -9 숫자
rm -rf ~/.gazebo/model_cache/*
LAUNCH_FILE=~/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/launch/turtlebot3_world.launch.py
cp $LAUNCH_FILE ${LAUNCH_FILE}.bak
sudo apt install --reinstall ros-humble-gazebo-ros-pkgs
pkill -f gzserver
pkill -f gzclient
source ~/turtlebot3_ws/install/setup.bash


시뮬레이션 2로봇
source ~/ros2_ws/install/setup.bash
ros2 launch tb3_security_system sim_two_robots.launch.py

실제 2로봇
각각
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_1

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py namespace:=tb3_2

원격
ros2 launch tb3_security_system real_two_robots.launch.py


시뮬레이션 2 로봇 안될 때
- Gazebo 완전히 종료
  pkill -9 gazebo
  pkill -9 gzserver
  pkill -9 gzclient
- 혹시 남아 있는 ROS 노드도 종료
  pkill -9 python3
- 현재 DDS 통신 초기화
  sudo fuser -k 11345/tcp
  sudo fuser -k 11346/tcp



