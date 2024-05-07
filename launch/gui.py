from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='robotem_rovne_gui',
      executable='robotem_rovne_gui_node',
      name='robotem_rovne_gui',
      namespace='t07',
      output='screen',
      emulate_tty=True,
      parameters=[
      ]
    )
  ])
