# global_planner

ROS 2 `nav_msgs/msg/OccupancyGrid` 위에서 A* 탐색을 수행해 전역 경로를 생성하는 독립형 global planner입니다. `cpp_2d_slam`의 현재 위치와 map을 받아 RViz 등에서 지정한 목표까지의 `/plan`을 발행합니다.

## 동작 과정

1. `/map`을 받으면 점유 값과 `robot_radius`를 이용해 장애물 inflation grid를 만듭니다.
2. `/slam_pose`를 A* 시작 위치로 사용합니다.
3. `/goal_pose`를 받으면 시작점과 목표점을 map cell로 변환합니다.
4. 시작점 또는 목표점이 inflation 영역 안에 있으면 `max_snap_distance` 범위에서 가장 가까운 free cell을 찾습니다.
5. A*로 경로를 탐색하고 각 cell 중심을 `nav_msgs/msg/Path`로 변환해 `/plan`에 발행합니다.

기본값에서는 8방향 이동을 사용하며, 대각선으로 장애물 모서리를 통과하는 corner cutting은 허용하지 않습니다. 주행 가능한 점유 cell도 값이 높을수록 traversal cost가 조금 증가합니다.

## 입출력 토픽

| 구분 | 기본 토픽 | 메시지 형식 | 설명 |
| --- | --- | --- | --- |
| 입력 | `/map` | `nav_msgs/msg/OccupancyGrid` | 전역 점유 격자 지도 |
| 입력 | `/slam_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 현재 로봇 위치 |
| 입력 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 목표 위치와 방향 |
| 출력 | `/plan` | `nav_msgs/msg/Path` | A*로 생성한 전역 경로 |

## 파라미터

| 이름 | 형식 | 기본값 | 설명 |
| --- | --- | --- | --- |
| `map_topic` | string | `/map` | map 입력 토픽 |
| `pose_topic` | string | `/slam_pose` | 현재 pose 입력 토픽 |
| `goal_topic` | string | `/goal_pose` | goal 입력 토픽 |
| `plan_topic` | string | `/plan` | 전역 경로 출력 토픽 |
| `obstacle_threshold` | int | `50` | 이 값 이상의 점유 cell을 장애물로 처리하며, 범위는 0~100으로 제한 |
| `allow_unknown` | bool | `true` | 점유 값 `-1`인 unknown cell의 통과 허용 여부 |
| `robot_radius` | double | `0.25` | 장애물 inflation 반경, 단위 m |
| `use_diagonal` | bool | `true` | 8방향 이동 사용 여부. `false`이면 4방향 이동 |
| `max_snap_distance` | double | `0.5` | 시작점·목표점을 가까운 free cell로 보정할 최대 거리, 단위 m |
| `map_transient_local` | bool | `false` | transient-local QoS로 latched map을 받을지 여부 |

`obstacle_threshold`, `robot_radius`, `max_snap_distance`는 노드 시작 시 유효 범위로 보정됩니다.

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select global_planner
source install/setup.bash
```

## 단독 실행

```bash
ros2 run global_planner standalone_global_planner
```

파라미터를 직접 지정하는 예시는 다음과 같습니다.

```bash
ros2 run global_planner standalone_global_planner --ros-args \
  -p robot_radius:=0.18 \
  -p obstacle_threshold:=50 \
  -p allow_unknown:=false \
  -p use_diagonal:=true \
  -p max_snap_distance:=0.5 \
  -p map_transient_local:=true
```

RViz의 `2D Goal Pose`로 `/goal_pose`를 발행하고, `Path` display에 `/plan`을 지정하면 결과를 확인할 수 있습니다.

## 전체 내비게이션 실행

`cpp_2d_slam`, `global_planner`, `local_controller`를 함께 실행하려면 통합 launch를 사용합니다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cpp_2d_slam global_planner local_controller
source install/setup.bash
ros2 launch local_controller navigation.launch.py
```

통합 launch에서는 `cpp_2d_slam`이 먼저 발행한 map도 받을 수 있도록 `map_transient_local=true`가 적용됩니다.

## 좌표계와 재계획 조건

- `/map`, `/slam_pose`, `/goal_pose`는 같은 좌표계에 있어야 합니다.
- 이 planner는 pose 또는 goal에 TF 변환을 적용하지 않습니다. frame이 map frame과 다르면 경고만 출력합니다.
- 새 `/goal_pose`를 받으면 map과 pose가 준비된 경우 즉시 계획합니다.
- 새 `/map`을 받으면 inflation grid를 다시 만들고, pose와 goal이 있으면 다시 계획합니다.
- `/slam_pose` 갱신만으로는 경로를 다시 계획하지 않습니다.
- map 밖의 시작점·목표점, snap할 수 없는 장애물 내부 위치, 도달 불가능한 목표에 대해서는 경로를 발행하지 않습니다.

## 경로 출력

경로의 각 pose는 map cell 중심에 배치됩니다. 각 pose의 방향은 다음 pose를 향하고, 마지막 pose는 요청한 goal 방향을 유지합니다.
