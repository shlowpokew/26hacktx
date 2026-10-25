import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam = get_package_share_directory('slam_toolbox')
    lidar_frame = os.environ.get('LIDAR_FRAME', 'laser')
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
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD19',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD19',
                'topic_name': 'scan/raw',
                'frame_id': lidar_frame,
                'port_name': os.environ.get('LIDAR_SERIAL_PORT', '/dev/lidar'),
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0,
            }],
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/lidar_scan_filter.py'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/encoder_odometry.py'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/human_tracker.py'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/rover_controller.py'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/button_io.py'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/mission_manager.py'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/breadcrumb_manager.py'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/backtrack_navigator.py'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                os.environ.get('LIDAR_X_M', '0.0'),
                os.environ.get('LIDAR_Y_M', '0.0'),
                os.environ.get('LIDAR_Z_M', '0.0'),
                '0', '0', os.environ.get('LIDAR_YAW_RAD', '0.0'),
                'base_link', lidar_frame,
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['python3', '/opt/robot/arduino_bridge.py'],
            output='screen',
        ),
    ])
