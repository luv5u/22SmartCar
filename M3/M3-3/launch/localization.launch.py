"""Stage 1: existing M3-2 simulation + saved map + AMCL (no motion)."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    previous = root.parent / 'M3-2'
    sim = previous / 'sim.launch.py'
    if not sim.is_file():
        raise FileNotFoundError(f'Extract into M3/M3-3; missing {sim}')
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(previous / 'maps/map.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2 / 'launch/localization_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': str(root / 'config/amcl_params.yaml'),
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
    ])
