from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Executes footstep planner
    footstep_planner = Node(
        package='footstep_planner',
        executable='footstep_planner_node',
        output='screen'
    ) 

    return LaunchDescription([footstep_planner])