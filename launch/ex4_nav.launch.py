# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim
# Adapted by: Tom Howard
# Adapted from: https://github.com/ROBOTIS-GIT/turtlebot3/blob/jazzy/turtlebot3_navigation2/launch/navigation2.launch.py
# Adaptation Date: November 2025
# Description: Customized launch configuration for AMR31001 Lab2 navigation exercise.

# TODO: 
# * Include SLAM with no Rviz
# * Customise rviz nav config to turn off TF and Robot Description (or fix the assoicated errors)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=PathJoinSubstitution([
            FindPackageShare('amr31001_lab2'),
            'maps',
            'my_map.yaml'
        ])
    )
    
    param_dir = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([
            FindPackageShare('turtlebot3_navigation2'),
            'param',
            'waffle.yaml'
        ])
    )

    rviz_config_dir = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription( 
            PythonLaunchDescriptionSource( 
                PathJoinSubstitution([ 
                    FindPackageShare("tuos_tb3_tools"), 
                    "launch", 
                    "slam.launch.py" 
                ])
            ),
            launch_arguments={ 
                'environment': 'real'
            }.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'), 
                    'launch',
                    'bringup_launch.py'
                ]),
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])