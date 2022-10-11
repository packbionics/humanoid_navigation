import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    footstep_planner_dir = get_package_share_directory('footstep_planner')
    map_src = os.path.join(footstep_planner_dir, os.path.join('maps', 'sample.yaml'))

    default_rviz2_config_path = os.path.join(footstep_planner_dir, os.path.join('config', 'rviz2_footstep_planning.rviz'))

    rviz2_config_file = LaunchConfiguration('rviz2_config')

    rviz2_config_arg = DeclareLaunchArgument(   'rviz2_config', 
                                                default_value=default_rviz2_config_path, 
                                                description='loads rviz2 with appropriate settings for footstep planning')

    # execute rqt
    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui'
    )

    # execute rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz2_config_file]
    )

    # execute map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_src}]
    )

    # executes lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        parameters=[{'node_names': ['map_server']}]
    )

    # execute footstep planner
    footstep_planner_launch =   IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(
                                        os.path.join(footstep_planner_dir, os.path.join('launch', 'footstep_planner.launch.py'))
                                    )
                                )

    ld = LaunchDescription([rviz2_config_arg,
                            rqt,
                            rviz2,
                            map_server,
                            lifecycle_manager,
                            footstep_planner_launch])

    return ld