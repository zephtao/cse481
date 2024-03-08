import rclpy
import time

import ikpy.chain
from ikpy.utils.geometry import rpy_matrix
from createmate_interfaces.srv import PickupDrawTool
from createmate_interfaces.action import Sleepy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
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
  PICKUP=6


class PickupMarker(Node):
  'ros code to pickup a marker'
  def __init__(self):
    super().__init__('marker_pickup')
    self.srv = self.create_service(PickupDrawTool, 'pickup_draw_tool', self.pickup_marker_cb)

    # callbackgroup for action server responses (avoid deadlock w/ the node's service callback)
    ac_cbs = ReentrantCallbackGroup()

    # buffer asynchronously fills with transformations
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    # robot body vars
    self.urdf_path = '/home/hello-robot/cse481/zephyr_ws/src/createmate/createmate/stretch.urdf'
    self.chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path) # joints chained together

    # subscribe to current joint states and store
    self.joint_states = JointState()
    self.join_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1, callback_group=ac_cbs)

    # create action client to move joints with stretch core driver
    self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory', callback_group=ac_cbs)
    server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
    if not server_reached:
        self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
        sys.exit()

    # joint names whose positions are included in the message to the action server:
    self.joint_names = ['translate_mobile_base', 'joint_lift', 'wrist_extension','gripper_aperture', 'joint_wrist_pitch']
    
    # read all the aruco -> grasp center transforms recorded
    # load poses
    self.pose_filepath = '/home/hello-robot/cse481/zephyr_ws/save_poses.json'
    if os.path.isfile(self.pose_filepath):
      with open(self.pose_filepath, 'r') as pose_file:
        self.poses = json.load(pose_file)
        self.get_logger().info('pose file read')
    else:
      self.get_logger().info('file not found')

    # state of node
    self.state = PickupSeq.READY
    
    #TODO: uncomment for testing w.o actually initiating marker pickup!
    #self.sleepy_action_client = ActionClient(self, Sleepy, 'sleepy_act', callback_group=ac_cbs)

  # CALLBACKS FOR EXTERNAL NODE MESSAGES
  def joint_states_callback(self, joint_states):
    '''
      callback to update current joint states from service
    '''
    self.joint_states = joint_states

  def transform_to_base_frame(self, frame_id):
    '''
        Converts point (in aruco/target_object1 frame) to base_link frame
        record_tf: Transform from aruco_marker to grasp_center
    '''
    marker_grasp_info = self.poses['pickup_new_marker']
    position = marker_grasp_info.vector
    rec_pt = PointStamped(point=Point(x=position.x, y=position.y, z=position.z))
    rec_pt.header.frame_id = frame_id

    try:
      # obtain point in terms of base_link
      transformed_point = self.tf_buffer.transform(rec_pt, 'base_link')
      self.get_logger().info(f'{rec_pt.point} ({rec_pt.header.frame_id} frame) -> {transformed_point} (base_link frame)')
      res = [transformed_point.point.x, transformed_point.point.y, transformed_point.point.z]
      return res
    except TransformException as ex:
      self.get_logger().info(f'Could not transform point from {rec_pt.header.frame_id} to base_link: {ex}')
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

  def get_q_soln(self, target_point, q_init):
    '''
    Returns the new chain positions for the end effector to reach the target point
    target_point: shape (3,)
    q_init: shape (15, ) for this urdf
    '''
    q_soln = self.chain.inverse_kinematics(target_point, initial_position=q_init)

    # if on base phase, check whether the base is already aligned or needs to be moved
    # base alignment will be be at a certain threshold #TODO: needs to be tested
    if self.state == PickupSeq.BASE && abs(q_soln[1]) < 0.01 :
      self.get_logger().info(f'The base translation solution is {q_sol[1]}, so the base is aligned! moving onto arm lift phase')
      self.state == PickupSeq.LIFT
    elif self.state == PickupSeq.BASE
      self.get_logger().info(f'the base is not yet aligned... only returning base translation manipulation goal')
      q_init[1] = q_soln[1]
      q_soln = q_init
    return q_soln

  def move_to_configuration(self, q):
    ''' 
        accepts chain link positions in terms of base_link and move robot to that configuration
        q: chain link positions
        base: boolean indicating whether only the base is being moved
        gripper_open: bolean indicating whether the gripper should be open 
            for this step (true if base_only)
    '''
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
    # always have wrist pitch as straight out
    goal_point.positions[4] == 0.0
    # if grasping or picking up, close gripper
    if self.state.value >= PickupSeq.GRASP.value :
      self.get_logger().info('gripper will be closed...')
      goal_point.positions[3] =  0.08
    else: # otherwise, keep gripper open
      self.get_logger().info('gripper will be open...')
      goal_point.positions[3] = -0.12

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
    return true


  def pickup_marker_cb(self, request, response):
    self.state = PickupSeq.BASE
    target_done = False

    # loop through these steps until grasping marker and picked up 
    while not target_done:
      # only recalculate ik when dealing with base movement, the last calculated 
      # q_soln when base is aligned will be used for arm movements (lift, extend)
      if self.state == PickupSeq.BASE:
        tf_found = false
        while not tf_found:
          self.get_logger().info(f'Attempting tf calculation...')
          target = self.transform_to_base_frame(request.markerid)
          if target is not None:
            tf_found = True
          else:
            time.sleep(0.5)
        self.get_logger().info(f'target point for wrist center: {target}')
        # solve
        self.get_logger().info('Retrieving current joint positions...')
        q_init = self.get_q_init() # shape (15,)
        self.get_logger().info(f'Current joint positions: {q_init}')
        self.get_logger().info('Solving Inverse Kinematics for goal point')
        q = self.get_q_soln(target, q_init)
      
      # move robot base
      self.get_logger().info(f'the solution is: {q}, will now attempt to move to the configuration for manipulate {self.state.name} phase')
      success = self.move_to_configuration(q)
      if success && self.state.value < PickupSeq.PICKUP.value:
        # move onto the next pickup phase
        self.state == PickupSeq(self.state + 1)
        self.get_logger().info('moved onto next phase')
      elif not success:
        self.get_logger().info('did not succeed, will try again')
      else:
        target_done == True
    self.get_logger().info('target goal reached!')
    response.success = True #TODO, maybe if no progress after multiple loops, then send fail
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