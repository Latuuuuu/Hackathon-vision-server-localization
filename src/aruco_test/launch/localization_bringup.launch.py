"""Everything needed for localization: camera + extrinsic calibration + robot localization.

This is what `docker compose up -d` starts. Each part can be turned off, e.g.

    ros2 launch aruco_test localization_bringup.launch.py camera:=false
    ros2 launch aruco_test localization_bringup.launch.py live.compact:=true live.period:=0.1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Launch argument -> which included launch file it is forwarded to ('' = not forwarded)
FORWARDED = {
    'field_file': 'calib',
    'live.compact': 'calib',
    'live.period': 'calib',
    'calib.on_startup': 'calib',
    'calib.apply': 'calib',
    'robot.id': 'localization',
    'robot.marker_size': 'localization',
    'target_height': 'localization',
    'final_pose_topic': 'localization',
    'final_pose_yaw_offset_deg': 'localization',
    'debug.img': 'localization',
}

ARGUMENTS = [
    DeclareLaunchArgument('camera', default_value='true', description='Start the RealSense driver'),
    DeclareLaunchArgument('calib', default_value='true', description='Start field_calib_node (publishes map -> camera_link)'),
    DeclareLaunchArgument('localization', default_value='true', description='Start pnp_duck_node (publishes /pose/global)'),
] + [
    DeclareLaunchArgument(name, default_value='', description=f'Forwarded to the {target} launch file')
    for name, target in FORWARDED.items()
]


def include(package, launch_file, arguments):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=arguments)


def enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def launch_setup(context):
    # Only pass on the arguments that were actually given, so the included defaults stay in charge
    forwarded = {target: [] for target in ('calib', 'localization')}
    for name, target in FORWARDED.items():
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            forwarded[target].append((name, value))

    actions = []
    if enabled(context, 'camera'):
        actions.append(include('realsense2_camera', 'rs_launch.py', []))
    if enabled(context, 'calib'):
        actions.append(include('field_calib', 'field_calib.launch.py', forwarded['calib']))
    if enabled(context, 'localization'):
        actions.append(include('aruco_test', 'pnp_duck.launch.py', forwarded['localization']))
    return actions


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
