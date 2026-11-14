"""M3-2 scene, reusing the installed M3-1 car and controllers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    car_share = Path(get_package_share_directory("smart_car_description"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))
    world = Path(__file__).resolve().parent / "worlds" / "scene.world"
    if not world.is_file():
        raise FileNotFoundError(f"Missing world: {world}")
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ",
                 str(car_share / "urdf/smart_car.urdf.xacro")]),
        value_type=str,
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / "launch/gazebo.launch.py")),
        launch_arguments={"world": str(world)}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[{
            "robot_description": robot_description, "use_sim_time": True,
        }],
    )
    # Start in the south corridor, facing +x, away from all walls.
    # Supplied URDF: wheel bottom is 0.06 m below base_footprint.
    # Spawn 5 mm above contact to reduce landing disturbance; verify yaw before driving.
    spawn_car = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-topic", "robot_description", "-entity", "smart_car",
                   "-x", "0", "-y", "-4", "-z", "0.065", "-Y", "0",
                   "-timeout", "120.0"],
    )
    controller_nodes = [
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=[name, "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "120"])
        for name in ["joint_state_broadcaster", "ackermann_steering_controller"]
    ]
    return LaunchDescription([gazebo, state_publisher, spawn_car, *controller_nodes])
