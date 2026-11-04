from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_share = get_package_share_directory('gazebo_introduction')

    # Declarar argumentos
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='diff_robot.urdf',
        description='Nombre del archivo URDF dentro de models/'
    )

    gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Lanzar joint_state_publisher_gui'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar el reloj de la simulacion (/clock)'
    )

    # Obtener los argumentos
    model = LaunchConfiguration('model')
    use_gui = LaunchConfiguration('use_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Ruta completa al URDF seleccionado
    urdf_path = PathJoinSubstitution([pkg_share, 'models', model])

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
            'robot_description': robot_description,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool)
        }]
    )

    joint = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui)
    )

    rviz = ExecuteProcess(
        cmd=[
            'rviz2',
            '--ros-args',
            '-p',
            ['use_sim_time:=', use_sim_time]
        ],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        sim_time_arg,
        pub,
        joint,
        rviz
    ])
