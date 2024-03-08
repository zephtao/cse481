import rclpy
import createmate_interfaces.action
from createmate_interfaces.msg import DrawShapes, Shape, ShapesProgression
from createmate_interfaces.srv import Navigate, PickupDrawTool
from enum import Enum
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

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
  STARTUP = 1,
  ACCEPTING_DRAW_REQS = 2,
  DRAWING = 3

class Tool:
  NONE=''
  TOOL1 = 'tool1'
  TOOL2 = 'tool2'

class CoordinatorActionServer(Node):
  '''
    Central coordinator in charge of handling communications with the javascript application 
    and relay instructions to position the robot according to the user request, request shapes/sizes
    from the drawing node, and let it know when drawing is ready
  '''
  def __init__(self):
    super().__init__('coordinator')
    self.get_logger().info('starting up coordinator node')
    # startup robot
    self.state = CoreState.ACCEPTING_DRAW_REQS

    self.tool_in_grip = Tool.TOOL1 #TODO: assumes starting off with marker in hand rn!

    # create a separate reentrant callback group so blocking within the shape handling (bc of waiting for actions), 
    # does not block other callbacks
    self.ui_exec_callback_group = ReentrantCallbackGroup()

    # home robot to start
    # borrow use of callback group since we are calling a service inside its callback
    self.home_sub = self.create_subscription(Bool, '/is_homed', self.robot_home_check, 1, callback_group=self.ui_exec_callback_group)
    # REMAINING FIELDS INITIALIZED IN RUN_CONTROLLER
    
  def robot_home_check(self, home_msg):
    '''
    Receives whether robot is already homed through /is_homed topic. 
    Homes the robot if it is not already homed
    home_msg: /std_msgs/Bool
    '''
    self.get_logger().info(f'Received robot homed topic message: {home_msg}')
    if not home_msg and self.state == CoreState.STARTUP: #state check for redundancy
      self.get_logger().info('Robot not yet homed, contacting home_the_robot server to initiate homing sequence')

      # CONNECT TO HOMING SERVICE
      self.home_the_robot_service = self.create_client(Trigger, '/home_the_robot')

      while not self.home_the_robot_service.wait_for_service(timeout_sec=2.0):
        self.get_logger().info("Waiting on '/home_the_robot' service...")

      self.get_logger().info('Node ' + self.node_name + ' connected to /home_the_robot service.')

      # REQUEST HOMING
      trigger_req = Trigger.Request()
      home_future = self.home_the_robot_service.call(trigger_req)
      # # TODO: rn this assumes the robot homing always goes smoothly
      self.get_logger().info('Robot finished the homing sequence!')
    else:
      self.get_logger().info('robot is already homed!')
    
    # move onto next phase
    self.state = CoreState.ACCEPTING_DRAW_REQS
    
    # destroy subscription so we don't deal with homing anymore
    self.home_sub.destroy()

  def handle_ui_draw_reqs(self, draw_reqs):
    '''
      goal callback function to accept/reject goal request
    '''
    if self.state == CoreState.ACCEPTING_DRAW_REQS:
      self.get_logger().info('Accepting the following draw request from UI: {draw_req}')
      self.state = CoreState.DRAWING
      return GoalResponse.ACCEPT
    else:
      self.get_logger().info('Not currently accepting drawing requests. Rejecting the following UI request: {draw_req}')
      return GoalResponse.REJECT
  
  def get_correct_tool(self, req_tool):
    if req_tool != self.tool_in_grip:
      # navigate to the marker table
      navSrvReq = Navigate.Request()
      navSrvReq.target_map_pose = 'face_marker_table'
      res = self.nav_client.call(navSrvReq)
      self.get_logger().info('result of createmate navigation service: {res}')

      # pickup the marker
      pickupToolReq = PickupDrawTool.Request()
      pickupToolReq.markerid = req_tool
      res = self.pickup_tool_client.call(pickupToolReq)
      self.get_logger().info('result of createmate pickup tool service: {res}')
      self.tool_in_grip = req_tool

      # navigate to the canvas
      navSrvReq = Navigate.Request()
      navSrvReq.target_map_pose = 'face_canvas'
      res = self.nav_client.call(navSrvReq)
      self.get_logger().info('result of createmate navigation service: {res}')

  def execute_user_draw_shapes(self, goal_handle):
    '''
      Orchestrate user drawing request execution
    '''
    # extract requested shape goals
    ds_goals = goal_handle.request.shape_goals.shapes #list of shape messages
    self.get_logger().info('executing goal: {ds_goals}')
    
    # init feedback
    shapes_feedback = DrawShapes.Feedback()
    
    # keep track of whether all shapes succeeded
    shape_failed = False

    # loop through requested shapes
    for i in range(len(ds_goals)): #DrawShapes message array of shapes
      shapes_feedback.shape_num = i
      shapes_feedback.status = "setup"
      goal_handle.publish_feedback(shapes_feedback)
      #1: check correct drawing implement being held
      get_correct_tool(shape_req.tool)

      #2 send drawing request 
      shapes_feedback.status = "init drawing"
      goal_handle.publish_feedback(shapes_feedback)
      shape_goal_msg = ds_goals[i]
      shape_result = self.robo_shape_action_client.send_goal(shape_goal_msg) #drawing node uses the Shape messages in the DrawShapes messages
      if shape_result == "COMPLETE":
        shapes_feedback.status = "complete" #TODO: add a failure check if drawing node sends failure
      else:
        shapes_feedback.status = "failed"
        shape_failed = True
      goal_handle.publish_feedback(shapes_feedback)

    goal_handle.succeed()
    result = DrawShapes.Result()

    # reply with results (false success if ANY shapes failed)
    result.total_success = not shape_failed
    self.state = CoreState.ACCEPTING_DRAW_REQS
    return result

  def run_controller(self):
    '''
      Homes the robot and then sets up needed servers and clients using the default executor
    '''
    # first, await homing message and finish homing sequence
    self.get_logger().info('get homing message')
    while self.state == CoreState.STARTUP:
      rclpy.spin_once(self)

    self.get_logger().info('setting up user interface action server')

    # start up the action server to receive drawing requests from UI
    self.user_draw_action_server = ActionServer(
      self,
      createmate_interfaces.action.DrawShapes,
      'user_draw_shapes',
      execute_callback = self.execute_user_draw_shapes,
      goal_callback = self.handle_ui_draw_reqs,
      callback_group = self.ui_exec_callback_group
    )

    # action client to req marker switches
    self.get_logger().info('setting up service clients')

    # pickup marker service
    self.pickup_tool_client = self.create_client(PickupDrawTool, 'pickup_draw_tool')
    while not self.pickup_tool_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

    # navigation to preset map pose service
    self.nav_client = self.create_client(Navigate, 'nav2_preset_map_pose')
    while not self.nav_client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

    # TODO: create an action client to communicate w/ the drawing node
    # self.robo_shape_action_client = ActionClient(self, StretchDrawShape, 'stretch_draw_shape')
    self.get_logger().info('done setting up clients')

def main():
  rclpy.init()
  coor_node = CoordinatorActionServer()

  # first, will use default executor to home if necessary and set up servers/clients
  coor_node.run_controller()

  # switch to a multithreaded executor to handle callbacks dealing with ui requests and other functions
  executor = MultiThreadedExecutor()
  executor.add_node(coor_node)
  try:
    coor_node.get_logger().info('beginning coordinator servers and client handling...')
    executor.spin()
  except KeyboardInterrupt:
    coor_node.get_logger().info('Keyboard interrupt... shutting down')
  
  coor_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()
