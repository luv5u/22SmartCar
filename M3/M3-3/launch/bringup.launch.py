"""M3-3 full stack: M3-2 simulation + AMCL + Ackermann Nav2 + RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    for path in [root / 'launch/localization.launch.py',
                 root.parent / 'M3-2/sim.launch.py',
                 root / 'behavior_trees/navigate_to_pose.xml',
                 root / 'behavior_trees/navigate_through_poses.xml']:
        if not path.is_file():
            raise FileNotFoundError(f'Missing required file: {path}')
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': str(root / 'behavior_trees/navigate_to_pose.xml'),
            'default_nav_through_poses_bt_xml': str(root / 'behavior_trees/navigate_through_poses.xml'),
        },
        convert_types=True,
    )
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(root.parent / 'M3-2/maps/map.yaml')),
        # Existing, tested M3-2 simulation uses Gazebo /clock throughout.
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true']),
        DeclareLaunchArgument('params_file', default_value=str(root / 'config/nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(root / 'launch/localization.launch.py')),
                launch_arguments={'map': LaunchConfiguration('map')}.items(),
            ),
        ]),
        # navigation_launch only: localization above already owns map_server/AMCL.
        GroupAction(scoped=True, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2 / 'launch/navigation_launch.py')),
                launch_arguments={
                    'params_file': params,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'autostart': 'true',
                    'use_composition': 'False',
                }.items(),
            ),
        ]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', str(root / 'navigation.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
