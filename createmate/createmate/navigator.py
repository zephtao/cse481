import os
import json
import rclpy
import time

from createmate_interfaces.srv import Navigate
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, PoseStamped
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

class Navigator(Node):
  def __init__(self):
    super().__init__('draw_nav')
    # callbackgroup for action server responses (avoid deadlock w/ the node's service callback)
    self.ac_cbs = ReentrantCallbackGroup()

    # subscribe to the initial pose topic
    self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.define_init_pose, 10, callback_group=self.ac_cbs)
    self.init_pose = Pose()
    self.initial_pose_set = False

    # create action client for nav2 stack
    self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.ac_cbs)
    while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

    # load poses
    self.pose_filepath = '/home/hello-robot/cse481/team2/save_poses.json'
    if os.path.isfile(self.pose_filepath):
      with open(self.pose_filepath, 'r') as pose_file:
        self.poses = json.load(pose_file)
    else:
      self.get_logger().info('file not found')
  
  # def extract_req_pose(self, request):
  #   self.get_logger().info('extracting navigation position data')
  #   #load the pose goal
  #   req_pose = self.poses[request.target_map_pose]
  #   req_vector = req_pose['vector']
  #   req_quaternion = req_pose['quaternion']
  #   self.get_logger.info('creating message')
  #   goal = PoseStamped()
  #   goal.header.frame_id = req_pose.frame_id
  #   goal.pose.position = Point(x=float(req_vector.x), y=float(req_vector.y), z=float(req_vector.z))
  #   goal.pose.orientation = Quaternion(x=float(req_quaternion.x), y=float(req_quaternion.y), z=float(req_quaternion.z), w=float(req_quaternion.w))
  #   self.get_logger.info('preparing final message')
  #   #final goal
  #   goal_msg = NavigateToPose.Goal()
  #   goal_msg.pose = goal
  #   return goal_msg

  def nav2_preset_map_pose(self, request, response):
    '''
      Callback for the node's service will navigate to chosen map location
    '''
    self.get_logger().info(f'received a request for {request.target_map_pose}')
    # check the pose is in the file
    if request.target_map_pose not in self.poses.keys():
      self.get_logger().info(f'The requested pose, {request.target_map_pose}, was not found in the saved pose file')
      response.success = False
      return response
    else: 
      req_pose = self.poses[request.target_map_pose]
      self.get_logger().info(f'found the position: {req_pose}')

    # extract goal info + prepare message
    #goal_msg = self.extract_req_pose(request)
    self.get_logger().info('extracting navigation position data')
    #load the pose goal
    self.get_logger().info('here')
    req_vector = req_pose['vector']
    req_quaternion = req_pose['quaternion']
    self.get_logger().info('creating message')
    goal = PoseStamped()
    goal.header.frame_id = req_pose['frame_id']
    goal.pose.position = Point(x=float(req_vector['x']), y=float(req_vector['y']), z=float(req_vector['z']))
    goal.pose.orientation = Quaternion(x=float(req_quaternion['x']), y=float(req_quaternion['y']), z=float(req_quaternion['z']), w=float(req_quaternion['z']))
    self.get_logger().info('preparing final message')
    #final goal
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = goal

    self.get_logger().info(f'preparing to send message! {goal_msg}')

    self.get_logger().info(f'Navigating to goal: {goal_msg}')
    nav_result = self.nav_to_pose_client.send_goal(goal_msg)

    #TODO: not actually sure what this will return
    self.get_logger().info(f'navigation complete?... result: {nav_result}')

    # return 
    response.success = True
    return response


  def define_init_pose(self, pose_with_covar):
    self.init_pose = pose_with_covar.pose.pose # define the initial pose
    self.initial_pose_set = True

    # destroy sub so we don't redefine by accident

  def wait_for_init_pose(self):
    while not self.initial_pose_set:
      rclpy.spin_once(self, timeout_sec=0.5)
    self.get_logger().info(f'received the initial start pose! Will now start nav2_preset_map_pose service...')
     # create service
    self.srv = self.create_service(Navigate, 'nav2_preset_map_pose', self.nav2_preset_map_pose, callback_group=self.ac_cbs)
    self.get_logger().info('service created')

def main():
  rclpy.init()
  draw_nav = Navigator()

  # first, wait for initial pose to be set
  print('Send an initial pose estimate through the /setInitialPose topic (can be done on rviz)')
  draw_nav.wait_for_init_pose()
  draw_nav.destroy_subscription(draw_nav.init_pose_sub)

  # switch to a multithreaded executor to handle callbacks dealing
  # with coordinator navigation requests
  executor = MultiThreadedExecutor()
  executor.add_node(draw_nav)
  try:
    executor.spin()
  except KeyboardInterrupt:
    draw_nav.get_logger().info('Keyboard interrupt... shutting down')
    
  draw_nav.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()