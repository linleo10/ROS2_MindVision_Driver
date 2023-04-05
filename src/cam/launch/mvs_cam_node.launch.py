import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	share_path = get_package_share_directory('cam')
	cam_config = os.path.join(share_path, 'config/cam.yaml')

	return LaunchDescription([
			DeclareLaunchArgument(
				name='params_file', 
				default_value=cam_config
			),
			Node(
				name="cam",
				package = "cam",
				executable = "cam",
				parameters = [LaunchConfiguration('params_file')]
			)
	])