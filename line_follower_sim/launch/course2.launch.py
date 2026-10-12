"""
Gazebo course with a blue line + TurtleBot3 waffle_pi (the TurtleBot3 model with a camera).

Sets the environment the sim needs on this setup, so no exports are required:
  TURTLEBOT3_MODEL=waffle_pi, GAZEBO_IP=127.0.0.1 (works offline),
  QT_QPA_PLATFORM=xcb (Gazebo GUI and camera on Wayland desktops),
  GAZEBO_MODEL_PATH += this package's models.

Usage: ros2 launch line_follower_sim course2.launch.py [gui:=false] [x_pose:=0.0] [y_pose:=0.0]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower_sim')
    tb3_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    world = os.path.join(pkg_share, 'worlds', 'turtlebot3_course2.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Open the Gazebo window'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),

        # Must run before the TurtleBot3 launch files are included: they read
        # TURTLEBOT3_MODEL from the environment.
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_share, 'models')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzclient.launch.py')),
            condition=IfCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch, 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_launch, 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
    ])
