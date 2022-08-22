import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # pkg_share = FindPackageShare('mavrover').find('mavrover')

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('mavrover'),
        'config',
        'mavrover_param.yaml'
        )
        
    mavros_node=Node(
        package = 'mavros',
        # name = 'my_mavros',
        executable = 'mavros_node',
        parameters = [config],
        output='screen'
    )
    mavrover_main_node=Node(
        namespace='/mavrover',
        package = 'mavrover',
        executable = 'main_mavros',
        output='screen'

    )
    ld.add_action(mavros_node)
    ld.add_action(mavrover_main_node)

    return ld