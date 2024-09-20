import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def launch_setup(context, *args, **kwargs):
    log_level = LaunchConfiguration("log_level").perform(context)
    config_file = LaunchConfiguration("bim_file")
    #move_group_node_name = LaunchConfiguration("move_group_node_name").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    rviz_file = LaunchConfiguration("rviz_file").perform(context)
    gui_config = LaunchConfiguration("gui_config").perform(context)

    path = get_package_share_path("user_interface")

    rviz_config = path / "rviz/user_interface.rviz"

    rosbim_manager = Node(
        package="rosbim_manager",
        executable="rosbim_manager.py",
        output="screen",
        parameters=[{"bim_file": config_file}],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    export_geometry_spawner = Node(
        package="rosbim_manager",
        executable="spawner.py",
        output="screen",
        arguments=["--name", "rosbim_export_geometry_plugin"],
        emulate_tty=True,
    )

    with open(
        f"{get_package_share_directory('concert_main_gui')}/config/rviz_files/{rviz_file}_config.rviz"
    ) as f:
        rviz_file_content = f.read()
    print("Move Group Namespace" in rviz_file_content)

    main_gui_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="concert_main_gui",
                executable="concert_gui",
                output="both",
                parameters=[
                    {"rviz_file": rviz_file}
                ],
            )
        ],
    )

    return [rosbim_manager, export_geometry_spawner, main_gui_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            DeclareLaunchArgument(
                "bim_file",
                default_value="/concert_application_ws/src/rosbim/rosbim_example_models/models/Crane_Hall_V10.ifc",
                description="Absolute path of the bim file",
            ),
            DeclareLaunchArgument(
                "rviz_file",
                default_value="drilling",
                description="Rviz GUI to load.",
            ),
            DeclareLaunchArgument(
                "gui_config",
                default_value="gui_config.yaml",
                description="Rviz GUI to load.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
