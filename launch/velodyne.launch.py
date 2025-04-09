from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare the launch argument
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )

    limoncello_node = Node(
        package='limoncello',
        namespace='',
        executable='limoncello',
        name='limoncello',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('limoncello'), 'config', 'velodyne.yaml']),
            {'use_sim_time': True}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', PathJoinSubstitution([FindPackageShare('limoncello'), 'config', 'rviz', 'cat.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        limoncello_node,
        rviz_node
    ])
