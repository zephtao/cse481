import json
import os
import rclpy

from createmate_interfaces.msg import RecordPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from tf2_ros import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PoseListener(Node):

  def __init__(self):
    super().__init__('pose_listener')

    # to listen to aruco-robot transforms
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    #listen for keyboard requests to save poses
    self.pose_subscriber = self.create_subscription(RecordPose, 'pose_record_req', self.record_pose, 10)

    # subscriber for current location and pose rel to map
    self.map_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.save_map_pose, 1)
    self.map_pose = None

    # subscribe to current joint states and store
    self.joint_states = JointState()
    self.join_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)

    # check if pose file already exists and load previous data
    # RENAME TO WITHOUT INTERMEDIATE TO PROTECT FROM OVERWRITING
    self.pose_filepath = '/home/hello-robot/cse481/team2/save_poses_intermediate.json'
    if os.path.isfile(self.pose_filepath):
      with open(self.pose_filepath, 'r') as pose_file:
        self.poses = json.load(pose_file)
    else:
      self.poses = {}

  def joint_states_callback(self, joint_states):
    '''
      callback to update current joint states from service
    '''
    self.joint_states = joint_states

  def save_map_pose(self, msg):
    '''
      save pose of robot on map
      msg: Geometry_msgs/msg/PoseWithCovarianceStamped
    '''
    self.map_pose = PoseStamped()
    self.map_pose.header = Header()
    self.map_pose.header.frame_id = msg.header.frame_id
    # posewithcovariancestamped -> posewithcovariance -> pose
    self.map_pose.pose = msg.pose.pose
      
  def record_pose(self, rec_msg):
    '''
      callback dealing with keyboard request to save pose
      rec_msg: createmate_interfaces/msg/RecordPose
    '''
    if rec_msg.command == 'record_pose':
      self.get_logger().info(f'Received request to record pose in {rec_msg.frame} frame')

      if rec_msg.frame == 'map': # position on map
        if self.map_pose is not None:
          position = self.map_pose.pose.position
          orientation = self.map_pose.pose.orientation
          vector = {'x': position.x, 'y': position.y, 'z': position.z}
          quaternion = {'x': orientation.x, 'y': orientation.y, 'z': orientation.z, 'w': orientation.w}
          pose_entry =  {'type': 'location', 'frame_id': self.map_pose.header.frame_id, 'child_frame_id': '', 'vector': vector, 'quaternion': quaternion}
        else:
          self.get_logger().info('could not localize robot on map')
          return
      elif rec_msg.frame == 'base_link':
        joints = {self.joint_states.name[1]:self.joint_states.position[1], #joint_lift
                      'wrist_extension':self.joint_states.position[2] * 4, # extend
                      self.joint_states.name[6]:self.joint_states.position[6], # head pan
                      self.joint_states.name[7]:self.joint_states.position[7], # joint_head_tilt
                      self.joint_states.name[8]:self.joint_states.position[8], #joint_wrist_yaw
                      self.joint_states.name[9]:self.joint_states.position[9], #joint_wrist_pitch
                      self.joint_states.name[10]:self.joint_states.position[10], # joint_wrist_roll
                      }
        pose_entry = {'type': 'position', 'frame_id': 'base_link', 'child_frame_id': '', 'joints': joints}
      else : # an aruco frame
        tf = self.get_gripper_aruco_tf(rec_msg.frame)
        translation = tf.transform.translation
        rotation = tf.transform.rotation
        tf_vector = {'x': translation.x, 'y': translation.y, 'z': translation.z}
        tf_quaternion = {'x': rotation.x, 'y': rotation.y, 'z': rotation.z, 'w': rotation.w}
        pose_entry = {'type': 'transform', 'frame_id': rec_msg.frame, 'child_frame_id': 'link_grasp_center', 'vector': tf_vector, 'quaternion': tf_quaternion}

      # add to pose dict
      self.poses[rec_msg.name] = pose_entry

    elif rec_msg.command == 'quit':
      self.get_logger().info(f'Got quit message... writing all requested transforms to {self.pose_filepath}')

      # write all poses to filepath
      json_obj = json.dumps(self.poses, indent=2)
      with open(self.pose_filepath, 'w') as pose_file:
        pose_file.write(json_obj)
      
      # shutdown
      self.get_logger().info('shutting down')
      self.destroy_node()
      rclpy.shutdown()

  def get_gripper_aruco_tf(self, target_frame):
    '''
      Retrieves tf from
    '''
    from_frame_rel = 'link_grasp_center'
    try:
      now = Time()
      trans = self.tf_buffer.lookup_transform(
        target_frame,
        from_frame_rel,
        now)
      return trans
    except TransformException as ex:
      self.get_logger().info(f'Could not transform {from_frame_rel} to {target_frame}: {ex}')

def main():
  rclpy.init()
  node = PoseListener()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

if __name__ == '__main__':
    main()