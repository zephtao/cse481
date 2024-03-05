import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs import Bool

import hello_helpers.hello_misc as hm

'''
  Coordinate between interface drawing requests and setup components
'''
'''
#0: The startup! Make sure the robot is homed + set up
#1: user sends request for shape, location, color
#2: is the robot holding the correct marker?
#3: if yes, reset to the starting drawing pose
#4: if no, navigate to the table and pick up the correct marker + flip to the drawing enabled position
#5: Once everything is set up, pass the information onto the drawing node (size + circle)

'''
class CoreState(Enum):
  HOME = 1,
  SETUP = 2, 
  DRAW_READY = 3

class Coordinator(Node):
  '''
    Central coordinator in charge of handling communications with the javascript application 
    and relay instructions to position the robot according to the user request, request shapes/sizes
    from the drawing node, and let it know when drawing is ready
  '''
  def __init__(self):
    super().__init__('coordinator')
    # startup robot
    self.state = CoreState.STARTUP
    # home robot
    self.home_sub = self.create_subscription(Bool, '/stretch/is_homed', self.robot_home_check, 1)
    self.is_homed = False

    # setup communication with user to figure out when to allow them to #TODO write srv file for response
    self.publisher = self.create_service(DrawShape, '/user_shape_req', self.handle_draw_req, 1)

    self.draw_ready_pub = self.create_publisher(Bool, '/createmate/draw_ready')

    #timer
    timer_period = 1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)


  def handle_draw_req(self, draw_req):
    if self.state == CoreState.DRAW_READY:  
    else:
      return 
    
  def robot_home_check(self, home_msg):
    '''
    Homes the robot if it is not already homed
    home_msg: /std_msgs/Bool
    '''
    self.get_logger().info(f'Received homing topic message: {home_msg}')
    if not home_msg && self.state == CoreState.STARTUP: #state check for redundancy
      self.get_logger().info('Robot not yet homed, contacting home_the_robot server to initiate homing sequence')

      # CONNECT TO SERVICE
      self.home_the_robot_service = self.create_client(Trigger, '/home_the_robot', callback_group=reentrant_cb)

      while not self.home_the_robot_service.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Waiting on '/home_the_robot' service...")

      self.get_logger().info('Node ' + self.node_name + ' connected to /home_the_robot service.')

      # REQUEST HOMING
      trigger_req = Trigger.Request()
      home_future = self.home_the_robot_service.call_async(trigger_req) # services return future that complete when request does
      rclpy.spin_until_future_complete(self, home_future)
      # TODO: this will assume the robot homing always goes smoothly
      self.get_logger().info('Robot finished the homing sequence!')

      #move onto next phase
      self.state = CoreState.SETUP 

    # destroy subscription so we don't go into the method unnecessarily
    self.home_sub.destroy()

  def timer_callback(self):
    self.draw_ready_pub.publish(self.state == CoreState.DRAW_READY) #TODO double check this isn't diff from Bool object

  def run_controller(self):
    # first, await homing message and finish homing sequence
    while self.state = CoreState.STARTUP:
      rclpy.spin_once(node)
    
    
    
  

      
    


def main():
  rclpy.init()
  node = Coordinator()
  node.run_controller()

if __name__ == '__main__':
    main()
