"""Startet LiDAR und Kamera gemeinsam inkl. statischer Transformationen.

  ros2 launch ~/camera_lidar/sensors_launch.py lidar_front_deg:=90 \
      cam_x:=-0.19 cam_y:=0.0 cam_z:=0.18 cam_yaw_deg:=0.0 cam_pitch_deg:=0.0

base_link: x = Gehrichtung, y = links, z = oben (Ursprung am LiDAR).
lidar_front_deg: Sensorwinkel der Gehrichtung des LiDAR (bei Bedarf wie in den Tests).
cam_*: Lage der Kamera relativ zum LiDAR in Metern (cam_x NEGATIV = hinter dem LiDAR).
cam_pitch_deg: Neigung der Kamera, positiv = nach unten.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DEG = ' * 3.14159265358979 / 180.0'


def generate_launch_description():
    lidar_launch = os.path.join(get_package_share_directory('sllidar_ros2'),
                                'launch', 'sllidar_s2_launch.py')
    defaults = {
        'lidar_port': '/dev/ttyUSB0', 'cam_device': '/dev/video0',
        'lidar_front_deg': '0.0', 'cam_x': '0.0', 'cam_y': '0.0', 'cam_z': '0.0',
        'cam_yaw_deg': '0.0', 'cam_pitch_deg': '0.0',
    }
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
        launch_arguments={'serial_port': LaunchConfiguration('lidar_port')}.items())

    camera = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('cam_device'),
            'image_width': 640, 'image_height': 360, 'framerate': 30.0,
            'pixel_format': 'yuyv2rgb', 'camera_name': 'c920',
            'frame_id': 'camera_link',
        }])

    # Fokus festsetzen (erst nach dem Start der Kamera; Fehler sind unkritisch)
    focus = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=['v4l2-ctl', '-d', LaunchConfiguration('cam_device'),
             '-c', 'focus_automatic_continuous=0', '-c', 'focus_absolute=0',
             '-c', 'exposure_dynamic_framerate=0'], output='screen')])

    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', PythonExpression(['-1.0 * ', LaunchConfiguration('lidar_front_deg'), DEG]),
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'])

    tf_cam = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', LaunchConfiguration('cam_x'), '--y', LaunchConfiguration('cam_y'),
                   '--z', LaunchConfiguration('cam_z'),
                   '--yaw', PythonExpression([LaunchConfiguration('cam_yaw_deg'), DEG]),
                   '--pitch', PythonExpression([LaunchConfiguration('cam_pitch_deg'), DEG]),
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'])

    return LaunchDescription(args + [lidar, camera, focus, tf_laser, tf_cam])
