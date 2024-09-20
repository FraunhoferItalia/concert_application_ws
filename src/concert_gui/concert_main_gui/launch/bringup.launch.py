import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace
from fhi_ros2.launch import get_remote_moveit_config


def launch_setup(context, *args, **kwargs):
    move_group_node_name = LaunchConfiguration("move_group_node_name").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    rviz_file = LaunchConfiguration("rviz_file")

    # TODO: add Generic remapping file
    main_gui_node = Node(
        package="concert_main_gui",
        executable="concert_gui",
        output="both",
        parameters=[{"rviz_file": rviz_file}, get_remote_moveit_config(move_group_node_name, namespace)],
    )

    transport_mission_generator = Node(
        package="concert_mission_generator",
        executable="transport_mission_generator.py",
        name="transport_mission_generator_node",
    )


    return [
        PushRosNamespace(namespace),
        main_gui_node,
        transport_mission_generator,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "move_group_node_name",
                default_value="move_group",
                description="Name of the 'move_group' node from which parameter should be retrieved",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="/steamdeck_ws/src/steamdeck/default.rviz",
                description="Absolute path to the rviz config",
            ),
            DeclareLaunchArgument(
                "rviz_file",
                default_value="transport",
                description="Rviz GUI to load.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
