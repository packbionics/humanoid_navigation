import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    footstep_planner = Node(
        package='footstep_planner',
        executable='footstep_planner_node',
        output='screen',
        parameters=['/home/anthony/dev-ws/src/humanoid_navigation/footstep_planner/config/footsteps_asimo.yaml']
    ) 

    return LaunchDescription([footstep_planner])