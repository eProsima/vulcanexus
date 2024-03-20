from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  sensor_nodes = []

  for id in range(1, 11):

    sensor_node = Node(
      package="demo_keys_cpp",
      executable="multiple_topic_sensor",
      parameters=[
                {"id": id}
            ])

    sensor_nodes.append(sensor_node)

  return LaunchDescription(sensor_nodes)
