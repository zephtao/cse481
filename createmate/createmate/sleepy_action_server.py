import time
import rclypy

from createmate_interfaces.action import Sleepy
from rclpy.action import ActionServer
from rclpy.node import Node


class SleepyActionServer(Node):
  'test action server to see if calling an action that blocks for some time will mess up services for some reason'
  def __init__(self):
    super().__init__('sleepy_action_node')
    self.sleepy_action = ActionServer(self, Sleepy, 'sleepy_act', self.sleep_cb)


  def sleep_cb(self, goal_handle):
    # wait on server
    self.get_logger().info('so sleepy...')
    sleep_time = goal_handle.request.sleep_secs
    time.sleep(sleep_time)
    goal_handle.succeed()
    result = Sleepy.Result()
    result.success = True
    self.get_logger().info('waking up!')
    return result

def main():
  rclpy.init()
  
  sleepy_node = SleepyActionServer()

  rclpy.spin(sleepy_node)

  rclpy.shutdown()


if __name__ == '__main__':
    main()