import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # pkg_share = FindPackageShare('mavrover').find('mavrover')
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='simulation_pkg').find('simulation_pkg')

    world_file_name = 'rover_ardupilot.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    pkg_dir = get_package_share_directory('simulation_pkg')

    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    

    world = os.path.join(pkg_dir, 'worlds', world_file_name)
    # launch_file_dir = os.path.join(pkg_dir, 'launch')
    
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world],
            output='log'
            )

    # config = os.path.join(
    #     get_package_share_directory('mavrover'),
    #     'config',
    #     'mavrover_param.yaml'
    #     )
        
    teleop_twist_keyboard_node=Node(
        package = 'teleop_twist_keyboard',
        executable = 'teleop_twist_keyboard',
        output='screen',
        prefix = 'xterm -e',
        name='teleop'
    )
    mavrover_main_pymavlink_node=Node(
        namespace='/mavrover',
        package = 'mavrover',
        executable = 'main_pymavlink',
        arguments = ['--ros-args', '--log-level', 'INFO']
    )


    # gazebo2topics_node=Node(
    #     namespace='/mavrover',
    #     package = 'mavrover',
    #     executable = 'gazebo2topics'
    # )
    
    ld.add_action(teleop_twist_keyboard_node)
    ld.add_action(mavrover_main_pymavlink_node)

    # ld.add_action(gazebo2topics_node)

    ld.add_action(gazebo)

    return ld