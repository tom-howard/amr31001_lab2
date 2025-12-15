from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
 
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
            get_package_share_directory("amr31001_lab2"),
            '/images/cam_img.jpg '
        ]],
        shell=True
    )
    
    return LaunchDescription([
        img_capture_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=img_capture_node,
                on_exit=[
                    LogInfo(msg='launching image_colours.py...'),
                    spawn_image_processing
                ]
            )
        ),
    ])