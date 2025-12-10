import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # todo 1 . 定义模型路径和世界文件路径
    model_path = os.path.join('/home/vulcan/Acdimic/urdf_ws/src/pipe_robot/test_ctrl_ros2/models/test2/model_pro.sdf')
    empty_world_path = "/usr/share/gazebo-11/worlds/empty.world"
    urdf_path = "/home/vulcan/Acdimic/urdf_ws/src/pipe_robot/test_ctrl_ros2/models/test2/mini_model.urdf"

    
    #todo  2 . 包含 gazebo 启动（不显式指定 world，使用默认）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
    )

    # todo 3. 加载SDF模型到Gazebo
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

    # 启动 ros2_control 的 controller manager 节点（传入 controllers yaml 和 robot_description）
    #todo 4. 启动控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_path, 'use_sim_time': True}],
        output='screen',
        namespace='/test2'
    )
    
    #todo  5. CLI: 先 configure 再 start（Foxy 的 --set-state 可选 configure/start）
    load_configure = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'configure', '/test2/force1_velocity_controller'],
        output='screen'
    )
    load_start = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', '/test2/force1_velocity_controller'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_model,
        controller_manager,
        load_configure,
        load_start
    ])