
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
	# Basic arguments
    DeclareLaunchArgument(
        "RGB_topic",
        default_value="/camera/camera/color/image_raw",
        description="RGB image topic subscribed by pnp_duck_node",
    ),
    DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera/color/camera_info",
        description="Camera info topic matching RGB_topic",
    ),
    DeclareLaunchArgument(
        "pose_topic",
        default_value="/duck/pose/pnp",
        description="Output pose topic",
    ),
	DeclareLaunchArgument(
        "world_frame",
        default_value="map",
        description="World frame",
    ),
	DeclareLaunchArgument(
        "camera_frame",
        default_value="camera_color_optical_frame",
        description="Camera frame",
    ),

	# Target arguments
	DeclareLaunchArgument(
        "robot.id",
        default_value="1",
        description="Robot marker ID (DICT_APRILTAG_16h5)",
    ),
	DeclareLaunchArgument(
        "robot.marker_size",
        default_value="0.1",
        description="Robot marker edge length in meters (black border, not including white margin)",
    ),

	# Pose filter arguments
	DeclareLaunchArgument(
        "pose_filter.enable",
        default_value="false",
        description="Enable pose filter",
    ),
	DeclareLaunchArgument(
        "pose_filter.alpha",
        default_value="0.1",
        description="Pose filter alpha",
    ),
	DeclareLaunchArgument(
        "pose_filter.max_jump_m",
        default_value="0.15",
        description="Pose filter max jump in meters",
    ),

	# Debug arguments
	DeclareLaunchArgument(
        "debug.enable",
        default_value="true",
        description="Enable debug mode",
    ),
    DeclareLaunchArgument(
        "debug.img",
        default_value="true",
        description="Enable image show for debugging",
    ),
]


def generate_launch_description():

	pnp_duck_node = Node(
		package='aruco_test',
		executable='pnp_duck_node',
		name='pnp_duck_node',
		output='screen',
        parameters=[
            {'RGB_topic': LaunchConfiguration("RGB_topic")},
            {'camera_info_topic': LaunchConfiguration("camera_info_topic")},
            {'pose_topic': LaunchConfiguration("pose_topic")},
            {'world_frame': LaunchConfiguration("world_frame")},
            {'camera_frame': LaunchConfiguration("camera_frame")},
            {'robot.id': LaunchConfiguration("robot.id")},
            {'robot.marker_size': LaunchConfiguration("robot.marker_size")},
            {'pose_filter.enable': LaunchConfiguration("pose_filter.enable")},
            {'pose_filter.alpha': LaunchConfiguration("pose_filter.alpha")},
            {'pose_filter.max_jump_m': LaunchConfiguration("pose_filter.max_jump_m")},
            {'debug.enable': LaunchConfiguration("debug.enable")},
            {'debug.img': LaunchConfiguration("debug.img")},
        ]
	)

	ld = LaunchDescription(ARGUMENTS)

	ld.add_action(pnp_duck_node)

	return ld
