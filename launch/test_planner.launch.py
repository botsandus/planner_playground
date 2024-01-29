from os import getenv, path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Get the launch directory
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    lifecycle_nodes = ["smoother_server", "planner_server"]

    load_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server_amcl",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"topic_name": "map_amcl"},
                    {"yaml_filename": get_package_share_directory("planner_playground")
                                      + "/config/map.yaml"},
                ],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server_amcl",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {
                        "autostart": True
                    },
                    {"node_names": ["map_server_amcl"]},
                ],
                output="screen",
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=False,
                respawn_delay=2.0,
                parameters=[
                    get_package_share_directory("planner_playground")
                    + "/config/smoother_server.yaml"
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=False,
                respawn_delay=2.0,
                parameters=[
                    [
                        get_package_share_directory("planner_playground")
                        + "/config/planner_server.yaml"
                    ],
                    [
                        get_package_share_directory("planner_playground")
                        + "/config/global_costmap.yaml"
                    ],
                    [
                        get_package_share_directory("planner_playground")
                        + "/config/global_costmap_test.yaml"
                    ],
                    [
                        get_package_share_directory("planner_playground")
                        + "/config/costmaps_plugins.yaml"
                    ],
                    {"GridBased.debug_visualizations": True},
                ],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("/map", "/map_amcl")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{"autostart": True}, {"node_names": lifecycle_nodes}],
            ),
            Node(
                package="planner_playground",
                executable="test_planner_with_rviz_node",
                name="test_planner_with_rviz_node",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    [get_package_share_directory("planner_playground"), "/config/", "nav.rviz"],
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(load_nodes)
    return ld
