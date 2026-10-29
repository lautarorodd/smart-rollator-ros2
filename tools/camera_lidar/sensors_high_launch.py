"""Startet LiDAR und Kamera der OBEREN Ebene im Namensraum /high.

  ros2 launch ~/camera_lidar/sensors_high_launch.py lidar_port:=/dev/serial/by-path/... \
      cam_device:=/dev/v4l/by-path/...-video-index0 lidar_front_deg:=0 lidar_z:=0.72 \
      cam_x:=-0.10 cam_y:=0 cam_z:=0.18

Themen: /high/scan, /high/image_raw. Alle Positionen in Metern.
lidar_z: Hoehe der oberen LiDAR-Ebene MINUS Hoehe der unteren (0,28 m), also relativ zum unteren LiDAR.
cam_*: Lage der oberen Kamera relativ zum OBEREN LiDAR (hinter dem LiDAR = negativ).
cam_enable:=false startet nur den LiDAR (z. B. fuer den Hoehentest ohne obere Kamera).
Hinweis: Das Launch-File des Herstellers (sllidar_s2_launch.py) ist fuer die S2-Familie gedacht.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

DEG = ' * 3.14159265358979 / 180.0'


def generate_launch_description():
    lidar_launch = os.path.join(get_package_share_directory('sllidar_ros2'),
                                'launch', 'sllidar_s2_launch.py')
    defaults = {
        'lidar_port': '/dev/ttyUSB1', 'cam_device': '/dev/video2',
        'lidar_front_deg': '0.0', 'lidar_z': '0.72',
        'cam_x': '0.0', 'cam_y': '0.0', 'cam_z': '0.18', 'cam_pitch_deg': '0.0',
        'cam_framerate': '30.0', 'cam_enable': 'true',
    }
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]

    lidar = GroupAction([
        PushRosNamespace('high'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            launch_arguments={'serial_port': LaunchConfiguration('lidar_port'),
                              'frame_id': 'laser_high'}.items()),
    ])

    camera = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam', namespace='high',
        condition=IfCondition(LaunchConfiguration('cam_enable')),
        parameters=[{
            'video_device': LaunchConfiguration('cam_device'),
            'image_width': 640, 'image_height': 360,
            'framerate': PythonExpression([LaunchConfiguration('cam_framerate')]),
            'pixel_format': 'yuyv2rgb', 'camera_name': 'cam_high',
            'frame_id': 'camera_high_link',
        }])

    focus = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=['v4l2-ctl', '-d', LaunchConfiguration('cam_device'),
             '-c', 'focus_automatic_continuous=0', '-c', 'focus_absolute=0',
             '-c', 'exposure_dynamic_framerate=0'], output='screen',
            condition=IfCondition(LaunchConfiguration('cam_enable')))])

    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('lidar_z'),
                   '--yaw', PythonExpression(['-1.0 * ', LaunchConfiguration('lidar_front_deg'), DEG]),
                   '--frame-id', 'base_link', '--child-frame-id', 'laser_high'])

    tf_cam = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', LaunchConfiguration('cam_x'), '--y', LaunchConfiguration('cam_y'),
                   '--z', PythonExpression([LaunchConfiguration('lidar_z'), ' + ', LaunchConfiguration('cam_z')]),
                   '--pitch', PythonExpression([LaunchConfiguration('cam_pitch_deg'), DEG]),
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_high_link'])

    return LaunchDescription(args + [lidar, camera, focus, tf_laser, tf_cam])
