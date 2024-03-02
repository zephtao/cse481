import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import hello_helpers.hello_misc as hm

'''
  Coordinate between interface drawing requests and setup components
'''

class Coordinator(Node):
  def __init__(self):
    super().__init__('coordinator')
    # startup robot
    
    # home robot
    self.home_sub = create_subscription(,'/stretch/ishomed')
    if not homed:
      self.home_the_robot_service = self.create_client(Trigger, '/home_the_robot', callback_group=reentrant_cb)
      while not self.home_the_robot_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting on '/home_the_robot' service...")
        self.get_logger().info('Node ' + self.node_name + ' connected to /home_the_robot service.')
      
      home_req = Trigger.Request()
      home_the_robot_service.call(home_req)
    
    # await a 


def main():
    rclpy.init()
    node = Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
