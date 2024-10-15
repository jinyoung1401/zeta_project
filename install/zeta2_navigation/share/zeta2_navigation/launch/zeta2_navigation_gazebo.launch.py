import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 디렉토리 정의
    zeta2_nav2_dir = os.path.join(get_package_share_directory('zeta2_navigation'))
    turtlebot3_gazebo_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'))

    # Launch 파일에서 사용할 인자 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  # 시뮬레이션 시간 사용
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            zeta2_nav2_dir,
            'maps',
            'office',
            'mapping.yaml'))  # 맵 파일 위치

    param_file_name = 'zeta_nomal' + '.yaml'  # 파라미터 파일 설정
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('zeta2_navigation'),
            'params',
            param_file_name))

    # Gazebo 월드 파일 경로 설정 (TurtleBot3 사용)
    world_file = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')

    # RViz 설정 파일 경로
    rviz_config_dir = os.path.join(
        zeta2_nav2_dir,
        'rviz',
        'nav2.rviz')

    return LaunchDescription([
        # 맵과 파라미터 파일을 인자로 받음
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Gazebo를 불러오는 Launch 파일 포함
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_gazebo_dir, '/launch', '/turtlebot3_world.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # RViz 실행을 위한 노드 설정
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        
        # 로봇 상태를 확인하기 위한 토픽 (예: /cmd_vel)
        Node(
            package='turtlebot3_teleop',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen'),
    ])
