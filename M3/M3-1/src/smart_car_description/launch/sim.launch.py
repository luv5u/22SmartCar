from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 按包名找到构建安装后的资源目录。
    car_share = Path(get_package_share_directory("smart_car_description"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))

    # 将 Xacro 展开，作为机器人的结构描述。
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            str(car_share / "urdf/smart_car.urdf.xacro"),
        ]),
        value_type=str,
    )

    # 启动仿真世界与图形窗口。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch/gazebo.launch.py")
        ),
        launch_arguments={
            "world": str(car_share / "worlds/square.world"),
        }.items(),
    )

    # 发布机器人描述，并根据关节状态更新 TF。
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    # 从 robot_description 话题读取模型，生成小车。
    spawn_car = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "smart_car",
            "-z", "0.15",
            "-timeout", "120.0",
        ],
    )

    # 等待模型中的控制器管理器就绪，再加载控制器。
    controller_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                name,
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "120",
            ],
        )
        for name in [
            "joint_state_broadcaster",
            "ackermann_steering_controller",
        ]
    ]

    return LaunchDescription([
        gazebo,
        state_publisher,
        spawn_car,
        *controller_nodes,
    ])