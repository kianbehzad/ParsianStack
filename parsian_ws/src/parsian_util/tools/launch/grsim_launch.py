from ament_index_python.packages import get_package_share_directory
from  launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
	param_path = os.path.join(
            get_package_share_directory('parsian_util'),
            'params',
            'config_params.yaml'
            )
	print(param_path)
	return LaunchDescription([
        Node(
            package='parsian_protobuf_wrapper',
            node_executable='grsim',
            node_name='grsim_node',
            parameters=[param_path]

        )
        ])