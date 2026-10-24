import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam = get_package_share_directory('slam_toolbox')
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=[
            EmitEvent(event=Shutdown(reason='ROS process exited; restart container'))
        ])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': os.environ.get('USE_SIM_TIME', 'false'),
                'slam_params_file': '/config/slam.yaml',
            }.items(),
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/arduino_bridge.py'],
            output='screen',
        ),
    ])
