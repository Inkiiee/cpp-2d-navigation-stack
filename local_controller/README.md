# local_controller

`global_planner`가 발행하는 `/plan`을 `/slam_pose`로 로봇 좌표계에
변환하고, `cpp_2d_slam`이 발행하는 `/scan_deskewed` 기반 로컬 코스트맵으로
충돌을 피하며 추종하는 ROS 2 컨트롤러입니다.

Global `/map`이 아직 없어 `/plan`이 나오지 않는 동안에는 `/goal_pose`를
직접 로봇 좌표로 변환해 `GOAL_SEEKING` 모드로 이동합니다. 이후 정상
`/plan`이 들어오면 자동으로 `TRACKING` 모드로 전환합니다.

## 좌표계 설계

- `/plan`, `/slam_pose`: `map` 좌표계
- `/scan_deskewed`, `/local_costmap`, 예측 궤적: `base_link` 좌표계
- scan을 map 좌표계로 누적하지 않으므로 동적 장애물 잔상이 남지 않습니다.
- `cpp_2d_slam`의 odom 이력으로 각 beam을 scan 종료 시점에 맞춘 PointCloud2를
  사용합니다.

## 입출력

- `/plan` (`nav_msgs/msg/Path`)
- `/goal_pose` (`geometry_msgs/msg/PoseStamped`)
- `/slam_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- `/scan_deskewed` (`sensor_msgs/msg/PointCloud2`)
- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/local_costmap` (`nav_msgs/msg/OccupancyGrid`)

## 제어 과정

1. `cpp_2d_slam`이 raw scan을 deskew해 `/scan_deskewed`로 발행합니다.
2. 매 point cloud마다 로봇 중심 rolling costmap을 새로 만듭니다.
3. scan endpoint를 `robot_radius + safety_margin`만큼 inflation합니다.
4. global path의 lookahead point를 `/slam_pose`로 로봇 좌표에 변환합니다.
5. Pure Pursuit로 기본 속도를 계산합니다.
6. 충돌 시 선속도와 각속도를 함께 샘플링합니다.
7. 전체 global path 거리와 trajectory 전체의 최소 장애물 여유를 평가합니다.
8. `TRACKING`, `GOAL_SEEKING`, `AVOIDING`, `BLOCKED` 상태로 동작합니다.
9. direct path가 연속 여러 번 안전할 때만 원래 추종 모드로 복귀합니다.
10. pose 또는 point cloud가 timeout되면 항상 정지합니다.

## Map 없는 경우의 fallback

```text
/plan 있음
  -> TRACKING -> AVOIDING/BLOCKED

/plan 없음 + /goal_pose 있음
  -> GOAL_SEEKING -> AVOIDING/BLOCKED

pose, goal, point cloud 중 필요한 입력 없음
  -> STOP
```

`GOAL_SEEKING`에서는 현재 pose와 goal을 잇는 직선을 임시 reference로
사용합니다. 주변 장애물은 피할 수 있지만 U자 장애물이나 막다른 길의
전역적인 우회 경로는 map과 Global Planner가 준비된 뒤에 해결됩니다.

새 `/goal_pose`가 들어오면 끝점이 새 goal과 맞지 않는 과거 `/plan`은
폐기합니다. Planner가 같은 goal의 경로를 발행하면 다시 `TRACKING`으로
전환합니다.

현재 회피는 완전한 Nav2 DWB/DWA 구현이 아니라, 교육용으로 단순화한
DWA-like 선속도/각속도 샘플링입니다.

## 빌드 및 실행

```bash
cd /mnt/c/Users/USER/Desktop/test_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select local_controller
source install/setup.bash

ros2 run local_controller standalone_local_controller --ros-args \
  --params-file src/local_controller/config/local_controller.yaml
```

세 패키지를 한 번에 실행하려면 다음 통합 launch를 사용합니다.

```bash
cd /mnt/c/Users/USER/Desktop/test_ros
source /opt/ros/humble/setup.bash
colcon build --packages-select cpp_2d_slam global_planner local_controller
source install/setup.bash

ros2 launch local_controller navigation.launch.py
```

최대 선속도와 각속도는 launch 인자로 바로 조절할 수 있습니다.

```bash
ros2 launch local_controller navigation.launch.py \
  max_linear_velocity:=0.1 \
  max_angular_velocity:=0.4
```

- `max_linear_velocity`: 전진 최대 속도, 단위 `m/s`
- `max_angular_velocity`: 회전 최대 속도, 단위 `rad/s`

장애물 인식 범위와 로봇 크기도 launch에서 조절할 수 있습니다.

```bash
ros2 launch local_controller navigation.launch.py \
  obstacle_max_range:=1.2 \
  robot_radius:=0.18 \
  safety_margin:=0.03 \
  costmap_width:=3.0 \
  costmap_height:=3.0
```

- `obstacle_max_range`: 로봇 중심에서 장애물로 사용할 최대 scan 거리
- `robot_radius`: 원형으로 가정한 실제 로봇 반경
- `safety_margin`: 로봇 반경 바깥에 추가할 안전 여유
- `costmap_width`, `costmap_height`: 로봇 중심 Local Costmap 전체 크기

실제 장애물 inflation 반경은 다음과 같습니다.

```text
inflation_radius = robot_radius + safety_margin
```

시뮬레이션의 `/clock`을 사용할 때는 다음처럼 실행합니다.

```bash
ros2 launch local_controller navigation.launch.py use_sim_time:=true
```

RViz에서 `/local_costmap`을 `Map` display로 추가하면 로봇 주변 inflated
장애물을 확인할 수 있습니다.

점군 발행 여부는 다음 명령으로 확인할 수 있습니다.

```bash
ros2 topic hz /scan_deskewed
ros2 topic echo /scan_deskewed --once
```

처음 실기 테스트에서는 `max_linear_velocity`를 `0.05` 정도로 낮추고,
바퀴를 띄우거나 즉시 정지할 수 있는 환경에서 `/cmd_vel`을 확인하세요.
