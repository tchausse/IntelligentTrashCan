"""
Simulate the trash can robot in Gazebo Harmonic.

Starts Gazebo (with or without its GUI), spawns the robot, starts its
controllers and the fill-level monitor, and bridges the simulated sensors to
ROS. Drive it by publishing geometry_msgs/TwistStamped on /cmd_vel.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def gazebo(context):
    """
    Start the Gazebo server, plus the GUI when asked for.

    gz runs without a shell in between, so that shutdown signals reach it and
    no server is left running after the launch ends. The server renders the
    sensors offscreen, so it needs no display.
    """
    world = LaunchConfiguration('world').perform(context)
    # gz_ros2_control's Gazebo system plugin lives next to the ROS libraries.
    plugin_path = os.pathsep.join(
        p for p in (os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'),
                    os.environ.get('LD_LIBRARY_PATH')) if p)
    server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '2', world],
        additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': plugin_path},
        name='gz_server',
        output='screen',
    )
    actions = [
        server,
        RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[Shutdown()])),
    ]
    if LaunchConfiguration('gui').perform(context).lower() == 'true':
        gui = ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '2'], name='gz_gui', output='screen')
        # Closing the GUI window ends the simulation.
        actions += [
            gui,
            RegisterEventHandler(OnProcessExit(target_action=gui, on_exit=[Shutdown()])),
        ]
    return actions


def generate_launch_description():
    bringup = FindPackageShare('trashcan_bringup')
    description = FindPackageShare('trashcan_description')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([description, 'urdf', 'trashcan.urdf.xacro'])]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'trashcan',
            '-z', '0.01',
        ],
        output='screen',
    )

    # gz_ros2_control starts the controller manager once the robot is spawned.
    def spawner(controller, *extra_args):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '120',
                *extra_args,
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

    controllers = [
        spawner('joint_state_broadcaster'),
        # Expose the drive on the conventional top-level topics.
        spawner('diff_drive_controller', '--controller-ros-args',
                '-r /diff_drive_controller/cmd_vel:=/cmd_vel'
                ' -r /diff_drive_controller/odom:=/odom'),
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([bringup, 'config', 'gz_bridge.yaml']),
            'use_sim_time': True,
        }],
        output='screen',
    )

    fill_level = Node(
        package='trashcan_sensors',
        executable='fill_level_node',
        parameters=[
            PathJoinSubstitution([bringup, 'config', 'fill_level.yaml']),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'trashcan.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([bringup, 'worlds', 'minimal_room.sdf']),
            description='Gazebo world file to load.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo GUI; false runs the simulation headless.'),
        DeclareLaunchArgument(
            'rviz', default_value='false', description='Start RViz.'),
        OpaqueFunction(function=gazebo),
        robot_state_publisher,
        bridge,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=controllers)),
        fill_level,
        rviz,
    ])
