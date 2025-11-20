from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
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
                    FindPackageShare("nav2_map_server"), 
                    "launch", 
                    "map_saver_server.launch.py" 
                ])
            )
        ),
        Node(
            package='amr31001_lab2',
            executable='ex4.py',
            name='ex4_map_saver',
            output='screen'
        )
    ])

