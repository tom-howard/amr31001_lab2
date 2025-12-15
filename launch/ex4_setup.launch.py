from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, LogInfo, 
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    this_pkg_share = get_package_share_directory("amr31001_lab2")
   
    img_capture_node = Node(
        package='amr31001_lab2',
        executable='ex4.py',
        name='image_capture_node',
        output='screen'
    )

    spawn_image_processing = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' run tuos_examples image_colours.py ',
            this_pkg_share,
            '/images/cam_img.jpg '
        ]],
        shell=True
    )
    
    return LaunchDescription([
        RegisterEventHandler(
            OnProcessExit(
                target_action=img_capture_node,
                on_exit=[
                    LogInfo('launch image processing...'),
                    TimerAction(
                        period=2.0,
                        actions=[spawn_image_processing],
                    )
                ]
            )
        ),
    ])