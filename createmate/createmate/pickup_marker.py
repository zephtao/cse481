import rclpy

from createmate_interfaces.srv import PickupDrawTool
from createmate_interfaces.action import Sleepy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class PickupMarker(Node):
  'ros code to pickup a marker and return to paper'
  def __init__(self):
    super().__init__('marker_pickup')
    self.srv = self.create_service(PickupDrawTool, 'pickup_draw_tool', self.pickup_marker_cb)
    callback_group2 = ReentrantCallbackGroup()
    self.sleepy_action_client = ActionClient(self, Sleepy, 'sleepy_act', callback_group=callback_group2)


  def pickup_marker_cb(self, request, response):
    # wait on server
    goal_msg = Sleepy.Goal()
    goal_msg.sleep_secs = 5
    self.sleepy_action_client.wait_for_server()
    self.get_logger().info("Waiting for sleeper")
    response = self.sleepy_action_client.send_goal(goal_msg)
    self.get_logger().info(f"sleeper done!: {response}")
    response.success = True
    return response

def main():
  rclpy.init()
  executor = MultiThreadedExecutor()
  
  marker_pickup_node = PickupMarker()
  executor.add_node(marker_pickup_node)
  executor.spin()

  rclpy.shutdown()


if __name__ == '__main__':
  main()