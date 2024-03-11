import json
import os
import rclpy
import time

import ikpy.chain
from ikpy.utils.geometry import rpy_matrix
from createmate_interfaces.srv import GoalPosition
from createmate_interfaces.action import Sleepy
from enum import Enum
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_geometry_msgs import PointStamped, Point
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class PickupSeq(Enum):
  READY=1,
  BASE=2,
  LIFT=3,
  EXTEND=4,
  GRASP=5,
  PICKUP=6,
  DRAWSTART=7,
  DRAWSTARTBASE=8

'''
I'M SORRY THIS A MISLEADING NAME. IT ALSO DOES PRESET POSES
'''

class PickupMarker(Node):
  'ros code to pickup a marker'
  def __init__(self):
    super().__init__('marker_pickup')
    # callbackgroup for action server responses (avoid deadlock w/ the node's service callback)
    ac_cbs = ReentrantCallbackGroup()
    self.srv = self.create_service(GoalPosition, 'move_to_preset', self.pickup_marker_cb, callback_group=ac_cbs)

    # buffer asynchronously fills with transformations
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    # robot body vars
    self.urdf_path = '/home/hello-robot/cse481/team2/src/createmate/createmate/stretch.urdf'
    self.chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path) # joints chained together

    # subscribe to current joint states and store
    self.joint_states = JointState()
    self.join_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1, callback_group=ac_cbs)

    # create action client to move joints with stretch core driver
    self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory', callback_group=ac_cbs)
    server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
    if not server_reached:
        self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')

    # joint names whose positions are included in the message to the action server:
    self.joint_names = ['translate_mobile_base', 'joint_lift', 'wrist_extension','gripper_aperture', 'joint_wrist_pitch']

    # read all the aruco -> grasp center transforms recorded
    # load poses
    self.pose_filepath = '/home/hello-robot/cse481/team2/save_poses.json'
    if os.path.isfile(self.pose_filepath):
      with open(self.pose_filepath, 'r') as pose_file:
        self.poses = json.load(pose_file)
        self.get_logger().info('pose file read')
    else:
      self.get_logger().info('file not found')

    # state of node
    self.state = PickupSeq.READY

  def next_phase(self):
    if self.state == PickupSeq.READY:
      self.state = PickupSeq.BASE
    elif self.state == PickupSeq.BASE:
      self.state = PickupSeq.LIFT
    elif self.state == PickupSeq.LIFT:
      self.state = PickupSeq.EXTEND
    elif self.state == PickupSeq.EXTEND:
      self.state = PickupSeq.GRASP
    elif  self.state == PickupSeq.GRASP:
      self.state = PickupSeq.PICKUP
    elif self.state == PickupSeq.PICKUP:
      self.state = PickupSeq.READY

  # CALLBACKS FOR EXTERNAL NODE MESSAGES
  def joint_states_callback(self, joint_states):
    '''
      callback to update current joint states from service
    '''
    self.joint_states = joint_states
    # save the current relevant joint states
    self.curr_trajectory_positions = self.get_q_init()

  def transform_to_base_frame(self, point_stamped):
    '''
        Converts point (in aruco/target_object1 frame) to base_link frame
        point_stamped
    '''
    try:
      # obtain point in terms of base_link
      transformed_point = self.tf_buffer.transform(point_stamped, 'base_link')
      self.get_logger().info(f'{point_stamped.point} ({point_stamped.header.frame_id} frame) -> {transformed_point} (base_link frame)')
      res = [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z]
      return res
    except TransformException as ex:
      self.get_logger().info(f'Could not transform point from {point_stamped.header.frame_id} to base_link: {ex}')
      return None

  def get_q_init(self):
    '''
      returns current configuration of joints
    '''
    def bound_range(name, value):
      names = [l.name for l in self.chain.links]
      index = names.index(name)
      bounds = self.chain.links[index].bounds
      return min(max(value, bounds[0]), bounds[1])

    q_base = 0.0
    q_lift = bound_range('joint_lift', self.joint_states.position[1])
    q_arml = bound_range('joint_arm_l0', self.joint_states.position[5])
    q_yaw = bound_range('joint_wrist_yaw', self.joint_states.position[8])
    q_pitch = bound_range('joint_wrist_pitch', self.joint_states.position[9])
    q_roll = bound_range('joint_wrist_roll', self.joint_states.position[10])

    return [0.0, q_base, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]

  def get_q_soln(self, target_point):
    '''
    Returns the new chain positions for the end effector to reach the target point
    target_point: shape (3,)
    q_init: shape (15, ) for this urdf
    '''
    self.get_logger().info(f'the current joint positions are: {self.curr_trajectory_positions}')
    q_soln = self.chain.inverse_kinematics(target_point, initial_position=self.curr_trajectory_positions)
    return q_soln

  def move_to_configuration(self, q, base_done):
    '''
        accepts chain link positions in terms of base_link and move robot to that configuration
        q: chain link positions
        base_done: boolean indicating whether the base was determined to be aligned (just for marker pickup)
        NOTE: if in BASE state, will not do anything. The outer caller will then move onto the next phase
    '''
    # if the  base if aligned, don't move anything, allow caller to move onto next phase
    if self.state == PickupSeq.BASE and base_done:
      return True 
    # ['translate_mobile_base', 'joint_lift', 'wrist_extension','gripper_aperture', 'joint_wrist_pitch']
    # q =  [0.0, q_base, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]
    #set up trajectory goal
    trajectory_goal = FollowJointTrajectory.Goal()
    trajectory_goal.trajectory.header.frame_id = 'base_link'
    trajectory_goal.trajectory.joint_names = self.joint_names

    # create trajectory point
    goal_point = JointTrajectoryPoint()
    duration_goal = Duration(seconds=10).to_msg()
    goal_point.time_from_start = duration_goal

    # move just the marker to the desired starting position
    if self.state == PickupSeq.DRAWSTART:
      trajectory_goal.trajectory.joint_names = ['wrist_extension', 'joint_lift']
      goal_point.positions = [4*q[5], q[3]]
    # move 
    elif self.state == PickupSeq.DRAWSTARTBASE:
      trajectory_goal.trajectory.joint_names = ['translate_mobile_base']
      goal_point.positions= [q[1]]
    else: # this will go through the pickup marker sequence
      # default pose to send is curr pose, with gripper straight out
      goal_point.positions = [0.0, self.curr_trajectory_positions[3], 4* self.curr_trajectory_positions[5], 0.0, 0.0]

      # if grasping or picking up, close gripper
      if self.state.value >= PickupSeq.GRASP.value :
        self.get_logger().info('gripper will be closed...')
        goal_point.positions[3] =  -0.12
      else: # otherwise, keep gripper open
        self.get_logger().info('gripper will be open...')
        goal_point.positions[3] = 0.08

      # adjust pose w. appropriate q goal to move one limb at a time
      if self.state == PickupSeq.BASE: # only moving base at the beginning
        self.get_logger().info('only moving base....')
        goal_point.positions[0] = q[1]
      elif self.state == PickupSeq.LIFT:
        self.get_logger().info('lifting arm...')
        goal_point.positions[1] = q[3]
      elif self.state == PickupSeq.EXTEND:
        self.get_logger().info('extending arm...')
        goal_point.positions[2] = 4*q[5]
      elif self.state == PickupSeq.PICKUP:
        self.get_logger().info('picking up the marker')
        goal_point.positions[1] = 1 # HARDCODED LIFT

    trajectory_goal.trajectory.points = [goal_point]
    self.get_logger().info(f'trajectory goal sent:{trajectory_goal}')
    # send joint trajectory
    trajectory_res = self.trajectory_client.send_goal(trajectory_goal)
    # TODO: maybe figure out what to do if this fails lol... and what that return looks like
    self.get_logger().info(f'trajectory goal result: {trajectory_res}')
    #TODO: if it fails, return false
    return True

  def pickup_marker(self, marker_name):
    self.state = PickupSeq.BASE
    target_done = False
    base_done = False

    # get recorded pose
    marker_grasp_info = self.poses['grab_tool']
    position = marker_grasp_info['vector']
    rec_pt = PointStamped(point=Point(x=position['x'], y=position['y'], z=position['z']))
    rec_pt.header.frame_id = marker_name 

    # loop through these steps until grasping marker and picked up
    while not target_done:  
      self.get_logger().info('Retrieving current joint positions...')
      # only recalculate ik when dealing with base movement, the last calculated
      # q_soln when base is aligned will be used for arm movements (lift, extend)
      if not base_done:
        tf_found = False
        base_done = False
        while not tf_found:
          self.get_logger().info(f'Attempting tf calculation...')
          target = self.transform_to_base_frame(rec_pt)
          if target is not None:
            tf_found = True
          else:
            time.sleep(0.5)
        self.get_logger().info(f'target point for wrist center: {target}')
        self.get_logger().info('Solving Inverse Kinematics for goal point')
        q = self.get_q_soln(target) # returns only base changed if the threshold is not met
        if abs(q[1]) < 0.1:
          self.get_logger().info('the base is aligned!')
          base_done = True
        else: 
          self.get_logger().info(f'the base is not aligned yet, still requires movement of: {q[1]}')

      success = self.move_to_configuration(q, base_done)
      time.sleep(4) # sleep just for safety

      if success and self.state.value < PickupSeq.PICKUP.value and base_done:
        # move onto the next pickup phase
        self.next_phase()
        self.get_logger().info('moved onto next phase')
      elif not success:
        # trajectory did not succeed, send same request
        self.get_logger().info('did not succeed, will try again')
      elif base_done:
        # we reached the end of the pickup sequence!
        target_done == True
      else:
        self.get_logger().info('now that movement has ended, will check base alignment again')

    # move to default marker holding pose
    self.move_to_default_pose('stow_marker')
    self.get_logger().info('target goal reached!')
  
  def move_to_default_pose(self, pose_name):
    goal_pose = self.poses.get(pose_name)
    joints = goal_pose.get('joints') #dictionary 

    trajectory_goal = FollowJointTrajectory.Goal()
    trajectory_goal.trajectory.header.frame_id = goal_pose.get('frame_id')
    trajectory_goal.trajectory.joint_names = joints.keys()

    # create trajectory point
    goal_point = JointTrajectoryPoint()
    goal_point.positions = list(joints.values())
    duration_goal = Duration(seconds=20).to_msg()
    goal_point.time_from_start = duration_goal
    trajectory_goal.trajectory.points = [goal_point]
    self.get_logger().info(f'trajectory goal sent:{trajectory_goal}')
    # send joint trajectory
    trajectory_res = self.trajectory_client.send_goal(trajectory_goal)
    # TODO: maybe figure out what to do if this fails lol... and what that return looks like
    self.get_logger().info(f'trajectory goal result: {trajectory_res}')
    #TODO: if it fails, return false
    return True
  
  def to_marker_start(self, request):
    '''
    request of type GoalPosition.Request
    this will move the marker to the requested start location for a shape
    '''
    self.state = PickupSeq.DRAWSTART
    canvas_pt = PointStamped(point=Point(x=request.x, y=request.y, z=request.z))
    canvas_pt.header.frame_id = request.markerid
    target = self.transform_to_base_frame(canvas_pt)
    self.get_logger().info(f'target point for wrist center: {target}')
    self.get_logger
    self.get_logger().info('Solving Inverse Kinematics for goal point')
    q = self.get_q_soln(target)

    # move arm lift/extend
    self.get_logger().info(f'the solution is: {q}, will now attempt to move to the configuration for manipulate {self.state.name} phase')
    self.move_to_configuration(q)
    self.get_logger().info('will now move base forward to contact paper')
    self.state = PickupSeq.DRAWSTARTBASE
    self.move_to_configuration(q)

  def pickup_marker_cb(self, request, response):
    # move to desired poses
    if request.pose_name == 'grab_tool':
      self.get_logger().info('grabbing tool!')
      self.pickup_marker(request.markerid)
    elif request.pose_name == 'setup_draw':
      self.get_logger().info('setting up to draw at requested shape location!')
      self.to_marker_start(request)
    else:
      self.get_logger().info('moving to default pose!')
      self.move_to_default_pose(request.pose_name)
    response.success = True #TODO, maybe if no progress after multiple loops, then send fail
    return response

def main():
  rclpy.init()
  executor = MultiThreadedExecutor()

  marker_pickup_node = PickupMarker()
  executor.add_node(marker_pickup_node)
  try:
    executor.spin()
  except KeyboardInterrupt:
    marker_pickup_node.get_logger().info('Keyboard interrupt... shutting down')
  marker_pickup_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
