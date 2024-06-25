from os import getenv, path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    load_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),
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
                    {"bond_timeout": 0.0},
                    {
                        "autostart": True
                    },
                    {"node_names": ["map_server_amcl", "amcl"]},
                ],
                output="screen",
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    get_package_share_directory("localisation_on_map")
                    + "/config/amcl.yaml",
                    {"set_initial_pose": True},
                    {"bond_heartbeat_period": 0.0},
                    {"initial_pose.x": 458.56},
                    {"initial_pose.y": 52.47},
                    {"initial_pose.z": 0.0},
                    {"initial_pose.yaw": -1.57},
                ],
                remappings=[
                    ("map", "map_amcl"),
                    ("scan", "/lidars/bpearl_merged_scan"),
                ],
                output="screen",
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
