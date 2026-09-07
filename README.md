# cpp-2d-navigation-stack

ROS 2 기반의 2D SLAM 및 자율주행 스택입니다. ICP 기반 위치 추정, A* 전역 경로 계획, 로컬 경로 추종을 하나의 저장소에서 함께 관리합니다.

## 패키지

| 패키지 | 역할 |
| --- | --- |
| `cpp_2d_slam` | LiDAR/odometry/IMU를 이용한 2D SLAM, `/map`, `/slam_pose`, `/scan_deskewed` 발행 |
| `global_planner` | Occupancy Grid와 목표 지점을 이용한 A* 전역 경로 계획, `/plan` 발행 |
| `local_controller` | 전역 경로 추종, 로컬 장애물 회피, `/cmd_vel` 발행 |

`local_controller/launch/navigation.launch.py`가 세 패키지를 함께 실행합니다.

## 빌드

ROS 2 Humble 환경의 워크스페이스 `src` 아래에서 저장소를 clone한 뒤 빌드합니다.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Inkiiee/cpp-2d-slam.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cpp_2d_slam global_planner local_controller
source install/setup.bash
```

`cpp_2d_slam` 빌드에는 Qt 6가 필요합니다. g2o가 설치되어 있으면 pose graph 최적화에 사용하고, 없으면 내장 최적화기를 사용합니다.

## 실행

```bash
ros2 launch local_controller navigation.launch.py
```

주요 실행 옵션의 예시는 다음과 같습니다.

```bash
ros2 launch local_controller navigation.launch.py \
  max_linear_velocity:=0.1 \
  max_angular_velocity:=0.4 \
  robot_radius:=0.18 \
  safety_margin:=0.03
```

상세한 알고리즘 설명은 [`cpp_2d_slam/README.md`](cpp_2d_slam/README.md), 로컬 제어기 설정은 [`local_controller/README.md`](local_controller/README.md)를 참고하세요.

## 저장소 구조

```text
cpp-2d-navigation-stack/
├── cpp_2d_slam/
├── global_planner/
└── local_controller/
```
