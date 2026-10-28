from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Declarar argumento para la ruta del URDF
    urdf_arg = DeclareLaunchArgument(
        'urdf_path',
        description='Ruta absoluta al archivo URDF'
    )

    # Obtener el argumento
    urdf_path = LaunchConfiguration('urdf_path')

    # Cargar el contenido del URDF
    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str
    )

    pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    joint = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            '/home/ros2curso/curso/lab3/demo_simulation_1/rviz/config.rviz'
        ],
        output='screen'
    )

    return LaunchDescription([
        urdf_arg,
        pub,
        joint,
        rviz
    ])
