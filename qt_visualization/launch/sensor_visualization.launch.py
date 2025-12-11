from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    # Define the nodes
    fake_sensor_node = Node(
        package='fake_sensor_driver',
        executable='fake_sensor_node',
        name='fake_sensor_node',
        output='screen'
    )
    
    data_processing_node = Node(
        package='data_processing',
        executable='data_processing_node',
        name='data_processing_node',
        output='screen'
    )
    
    qt_visualization_node = Node(
        package='qt_visualization',
        executable='qt_visualization_node',
        name='qt_visualization_node',
        output='screen'
    )
    
    # Register event handler: when qt_visualization_node exits, emit Shutdown event
    event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=qt_visualization_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason="Qt Visualization Window closed"))
            ]
        )
    )
    
    return LaunchDescription([
        fake_sensor_node,
        data_processing_node,
        qt_visualization_node,
        event_handler
    ])