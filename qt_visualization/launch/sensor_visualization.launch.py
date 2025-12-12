from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import GroupAction, TimerAction

def generate_launch_description():
    # Create a component container with IPC enabled for better performance
    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',  # 使用单线程容器减少线程切换开销
        name='sensor_container',
        namespace='',
        output='screen',
        parameters=[
            # 启用进程内通信（关键优化！）
            {'use_intra_process_comms': True},
            # 启用共享内存传输
            {'use_shared_memory': True}
        ],
        composable_node_descriptions=[
            ComposableNode(
                package='fake_sensor_driver',
                plugin='fake_sensor_driver::FakeSensorNode',
                name='fake_sensor_node',
                parameters=[
                    {'frequency': 20.0},
                    {'mode': 'random'},
                    {'qos_depth': 10},  # 增加队列深度以减少消息丢失
                    {'qos_reliability': 'best_effort'}
                ]
            ),
            ComposableNode(
                package='data_processing',
                plugin='data_processing::DataProcessingNode',
                name='data_processing_node',
                parameters=[
                    {'alpha': 0.05},  # 调整滤波参数
                    {'processing_frequency': 50.0}  # 提高处理频率
                ]
            )
        ]
    )
    
    # Run Qt visualization as a separate executable
    qt_visualization = Node(
        package='qt_visualization',
        executable='qt_visualization_node',
        name='qt_visualization_node',
        output='screen',
        parameters=[
            {'use_intra_process_comms': False}  # 可视化节点不启用IPC，因为它在不同进程
        ]
    )
    
    # Register event handler: when visualization node exits, emit Shutdown event
    viz_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=qt_visualization,
            on_exit=[
                EmitEvent(event=Shutdown(reason="Visualization window closed"))
            ]
        )
    )
    
    # Control startup order
    group_actions = GroupAction([
        container,
        TimerAction(
            period=0.5,
            actions=[qt_visualization]
        )
    ])
    
    return LaunchDescription([
        group_actions,
        viz_exit_handler
    ])