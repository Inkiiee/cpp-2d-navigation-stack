"""test_icp, global planner, local controller를 함께 실행하는 bringup launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 시뮬레이터를 사용할 때만 true로 덮어쓸 수 있도록 공통 인자로 둔다.
    use_sim_time = LaunchConfiguration("use_sim_time")
    max_linear_velocity = LaunchConfiguration("max_linear_velocity")
    max_angular_velocity = LaunchConfiguration("max_angular_velocity")
    obstacle_max_range = LaunchConfiguration("obstacle_max_range")
    robot_radius = LaunchConfiguration("robot_radius")
    safety_margin = LaunchConfiguration("safety_margin")
    costmap_width = LaunchConfiguration("costmap_width")
    costmap_height = LaunchConfiguration("costmap_height")

    # 설치된 local_controller 패키지의 기본 YAML을 찾아 자동으로 적용한다.
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("local_controller"),
            "config",
            "local_controller.yaml",
        ]
    )

    # ICP SLAM:
    # /slam_pose, /map, map->odom TF, /scan_deskewed를 발행한다.
    test_icp_node = Node(
        package="test_icp",
        executable="test_icp",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    # Global Planner:
    # /map + /slam_pose + /goal_pose를 받아 /plan을 발행한다.
    # test_icp의 transient_local /map을 시작 순서와 관계없이 받도록 설정한다.
    global_planner_node = Node(
        package="global_planner",
        executable="standalone_global_planner",
        name="standalone_global_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_topic": "/map",
                "pose_topic": "/slam_pose",
                "goal_topic": "/goal_pose",
                "plan_topic": "/plan",
                "map_transient_local": True,
            }
        ],
    )

    # Local Controller:
    # /plan이 있으면 TRACKING, 아직 map/plan이 없으면 /goal_pose를 이용한
    # GOAL_SEEKING으로 동작하고 최종 /cmd_vel을 발행한다.
    local_controller_node = Node(
        package="local_controller",
        executable="standalone_local_controller",
        name="standalone_local_controller",
        output="screen",
        parameters=[
            controller_config,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                # YAML 기본값보다 뒤에 적용되어 launch 인자가 최종 속도가 된다.
                "max_linear_velocity": ParameterValue(
                    max_linear_velocity, value_type=float
                ),
                "max_angular_velocity": ParameterValue(
                    max_angular_velocity, value_type=float
                ),
                "obstacle_max_range": ParameterValue(
                    obstacle_max_range, value_type=float
                ),
                "robot_radius": ParameterValue(robot_radius, value_type=float),
                "safety_margin": ParameterValue(safety_margin, value_type=float),
                "costmap_width": ParameterValue(costmap_width, value_type=float),
                "costmap_height": ParameterValue(
                    costmap_height, value_type=float
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock from /clock",
            ),
            DeclareLaunchArgument(
                "max_linear_velocity",
                default_value="0.2",
                description="Maximum forward velocity in m/s",
            ),
            DeclareLaunchArgument(
                "max_angular_velocity",
                default_value="0.8",
                description="Maximum yaw velocity in rad/s",
            ),
            DeclareLaunchArgument(
                "obstacle_max_range",
                default_value="2.0",
                description="Maximum scan obstacle range in meters",
            ),
            DeclareLaunchArgument(
                "robot_radius",
                default_value="0.25",
                description="Circular robot footprint radius in meters",
            ),
            DeclareLaunchArgument(
                "safety_margin",
                default_value="0.05",
                description="Extra obstacle inflation margin in meters",
            ),
            DeclareLaunchArgument(
                "costmap_width",
                default_value="4.0",
                description="Robot-centered costmap width in meters",
            ),
            DeclareLaunchArgument(
                "costmap_height",
                default_value="4.0",
                description="Robot-centered costmap height in meters",
            ),
            test_icp_node,
            global_planner_node,
            local_controller_node,
        ]
    )
