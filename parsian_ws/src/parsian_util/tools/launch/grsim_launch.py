from ament_index_python.packages import get_package_share_directory
from  launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    param_path = os.path.join(get_package_share_directory('parsian_util'), 'params', 'config_params.yaml')
    
    grsim_node = Node(package='parsian_protobuf_wrapper', node_executable='grsim', node_name='grsim_node', parameters=[param_path])
    vision_node = Node(package='parsian_protobuf_wrapper', node_executable='vision', node_name='vision_node', parameters=[param_path])
    worldmodel_node = Node(package='parsian_world_model', node_executable='worldmodel', node_name='worldmodel_node', parameters=[param_path])
    
    return LaunchDescription([grsim_node, vision_node, worldmodel_node])
