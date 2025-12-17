import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # todo 1 . 使用 package share 目录构建模型、URDF 和 controllers 路径
    pkg_share = get_package_share_directory('test_ctrl_ros2')
    model_path = os.path.join(pkg_share, 'models', 'test2', 'model_pro.sdf')
    # empty_world_path = os.path.join('/usr', 'share', 'gazebo-11', 'worlds', 'empty.world')
    urdf_path = os.path.join(pkg_share, 'models', 'test2', 'mini_model.urdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    
    #todo  2 . 包含 gazebo 启动（不显式指定 world，使用默认）
    # IncludeLaunchDescription 用于包含其他 launch 文件
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
    # 读取 URDF 文本（必须把 URDF 的文本传给 robot_description）
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
        
    # todo 3.5 测试： 状态发布器
    # state_publisher_node = Node(
    #     package= 'robot_state_publisher',
    #     executable= 'robot_state_publisher',
    #     parameters= [{'robot_description': robot_description}],
    #     output= 'screen',
    #     namespace= '/test2'
    # )
        
    #todo 4. 启动控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml, {'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
        namespace='/test2'
    )
    
    #todo  5. CLI: 先 configure 再 start（Foxy 的 --set-state 可选 configure/start）
    load_configure = ExecuteProcess(
        cmd=[   'ros2', 'control', 
                'load_controller', 
                '--set-state', 'configure', 
                'force1_velocity_controller'
            ],
        output='screen'
    )
    load_start = ExecuteProcess(
        cmd=[   'ros2', 'control', 
                'load_controller', 
                '--set-state', 'start', 
                'force1_velocity_controller'
            ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_model,
        # 等待spawn结束后再启动controller
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_model,
                on_exit=[   
                            controller_manager , 
                            load_configure
                        ],
            )
        ),
        # 事件触发动作， 等待配置完成后再启动执行
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_configure,
                on_exit=[load_start],
            )
        ),
        # load_start ,
    ])