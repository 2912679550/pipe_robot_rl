import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # todo =================== 1. 基础路径配置 ===================
    pkg_share = get_package_share_directory("pipe_robot_ros2")
    model_path = os.path.join(pkg_share, 'models', 'pipe_robot', 'pipe_robot_model.sdf')
    urdf_path = os.path.join(pkg_share, 'models', 'pipe_robot', 'pipe_robot.urdf')
    empty_world_path = "/usr/share/gazebo-11/worlds/empty.world"
    
    # 读取URDF内容（作为robot_description参数，Gazebo和controller_manager都需要）
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
        
    # todo ================== 2. 启动Gazebo（指定空世界绝对路径） ===================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': empty_world_path,
            'verbose': 'true'
        }.items()
    )
    
    # todo ================== 4. 加载Gazebo模型  ===================
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',  # Foxy中是spawn_entity.py（和Humble一致）
        arguments=[
            '-file', model_path,
            '-entity', 'pipe_robot',
            '-x', '1.0', '-y', '0', '-z', '5.0',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )
    
    
    
    return LaunchDescription([
        gazebo_launch,
        TimerAction(
            period=2.0,
            actions=[spawn_model],
        ),
    ])

