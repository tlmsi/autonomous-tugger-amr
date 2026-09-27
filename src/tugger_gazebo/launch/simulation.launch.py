import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    description_pkg = get_package_share_directory(
        'tugger_description'
    )

    gazebo_pkg = get_package_share_directory(
        'tugger_gazebo'
    )

    ros_gz_sim_pkg = get_package_share_directory(
        'ros_gz_sim'
    )

    xacro_file = os.path.join(
        description_pkg,
        'urdf',
        'tugger.urdf.xacro'
    )

    world_file = os.path.join(
        gazebo_pkg,
        'worlds',
        'tugger_test.sdf'
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file
        ]),
        value_type=str
    )

    # Gazebo resolves package:// URIs as model:// URIs.
    # Add the parent of the package share directory so Gazebo can find:
    # model://tugger_description/meshes/...
    gazebo_resource_path = os.path.dirname(description_pkg)

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            gazebo_resource_path,
            os.pathsep,
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_pkg,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'autonomous_tugger',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_robot
    ])
