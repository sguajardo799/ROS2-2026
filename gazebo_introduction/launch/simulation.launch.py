import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('gazebo_introduction')
    gz_share = get_package_share_directory('ros_gz_sim')

    # Declarar argumento para seleccionar el modelo
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='diff_robot.urdf',
        description='Nombre del archivo URDF dentro de models/'
    )

    # Declarar argumento para seleccionar el mundo
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Mundo de Gazebo: nombre de un mundo instalado o ruta absoluta a un .sdf'
    )

    # Visualizacion: robot_state_publisher + rviz (sin GUI, Gazebo publica /joint_states)
    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'visualization.launch.py')
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'use_gui': 'false',
            'use_sim_time': 'true'
        }.items()
    )

    # Gazebo con el mundo seleccionado (-r: inicia la simulacion corriendo)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items()
    )

    # Spawnear el robot desde el topico /robot_description
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diff_robot',
            '-z', '0.1'
        ]
    )

    # Puente entre topicos de Gazebo y ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ]
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        visualization,
        gazebo,
        spawn,
        bridge
    ])
