# 先导入必要的包
import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ===== 基础路径配置 =====
    pkg_share = get_package_share_directory("test_ctrl_ros2")
    model_path = os.path.join(pkg_share, 'models', 'test2', 'model_pro.sdf')
    urdf_path = os.path.join(pkg_share, 'models', 'test2', 'mini_model.urdf')
    empty_world_path = "/usr/share/gazebo-11/worlds/empty.world"

    # 读取URDF内容（作为robot_description参数，Gazebo和controller_manager都需要）
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    # todo ===== 1. 启动Gazebo（指定空世界绝对路径） =====
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': empty_world_path,
            'verbose': 'true'
        }.items()
    )

    # todo ===== 2. 启动机器人状态发布器 =====
    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # todo ===== 3. 加载Gazebo模型  =====
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',  # Foxy中是spawn_entity.py（和Humble一致）
        arguments=[
            '-file', model_path,
            '-entity', 'test2',
            '-x', '0', '-y', '0', '-z', '0.1',
            '-R', '0', '-P', '0', '-Y', '0'
        ],
        output='screen'
    )

    # todo ===== 4. spawner控制器到gazebo中 =====
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    load_force1_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['force1_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # todo ===== 5. 启动顺序（关键：先启动gazebo→导入模型→延迟启动spawner，确保插件连接） =====
    return LaunchDescription([
        gazebo_launch,
        state_publisher_node,
        spawn_model,
        # 等待延迟启动
        TimerAction(period=2.0, actions=[load_joint_state_broadcaster]),
        TimerAction(period=3.0, actions=[load_force1_controller])
    ])