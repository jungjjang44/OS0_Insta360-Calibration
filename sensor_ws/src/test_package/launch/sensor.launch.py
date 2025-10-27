from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node  # ★ 추가


def generate_launch_description():
    # 1) 포함할 Ouster launch 파일 이름 (기본: driver.launch.py)
    ouster_launch_file = LaunchConfiguration('ouster_launch_file')
    declare_ouster_launch_file = DeclareLaunchArgument(
        'ouster_launch_file',
        # default_value='driver.launch.py', # original file
        default_value='os0.launch.py', # 수정된 파일
        description='ouster_ros/launch 안의 포함할 launch 파일 이름'
    )

    # 2) 원본 launch가 쓰는 인자들
    params_file   = LaunchConfiguration('params_file')
    ouster_ns     = LaunchConfiguration('ouster_ns')
    os_driver_name = LaunchConfiguration('os_driver_name')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=str(
            Path(get_package_share_directory('ouster_ros')) / 'config' / 'driver_params.yaml'
        ),
        description='Ouster 드라이버 파라미터 파일 경로'
    )
    declare_ouster_ns = DeclareLaunchArgument(
        'ouster_ns', default_value='ouster', description='네임스페이스'
    )
    declare_os_driver_name = DeclareLaunchArgument(
        'os_driver_name', default_value='os_driver', description='드라이버 노드 이름'
    )

    # 3) 포함할 launch 경로를 Substitution으로 생성
    ouster_share = get_package_share_directory('ouster_ros')
    launch_path = PathJoinSubstitution([
        TextSubstitution(text=ouster_share),
        'launch',
        ouster_launch_file
    ])

    include_ouster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'params_file': params_file,
            'ouster_ns': ouster_ns,
            'os_driver_name': os_driver_name,
        }.items()
    )

    # 4) ★ Insta360 Publisher Node 추가 ★
    insta360_node = Node(
        package='test_package',                   # 패키지명
        executable='image_publisher',             # setup.py의 entry_points에서 정의한 이름
        name='insta360_publisher',                # 노드 이름
        output='screen',
        parameters=[{
            'width': 1920,
            'height': 1080,
            'fps': 30,
            'fourcc': 'MJPG',
            'crop.mode': 'bottom_half',
            'publish_compressed': False
        }]
    )

    return LaunchDescription([
        declare_ouster_launch_file,
        declare_params_file,
        declare_ouster_ns,
        declare_os_driver_name,
        include_ouster,
        insta360_node,    # ← 여기서 두 개 동시에 실행됨
    ])
