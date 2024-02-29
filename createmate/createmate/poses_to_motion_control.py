import ikpy.chain
from ikpy.utils.geometry import rpy_matrix
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from ros2_numpy import numpify
from geometry_msgs.msg import Transform
from tf2_ros import TransformException
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import PointStamped, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from enum import Enum
import re  # regular expression
import sys

'''
  Keep track of node state. Serves as the node's state
  machine. The node can only become unblocked when a trajectory completes.
  The base will move to align with the marker first. Then the arm/gripper
  will continue
'''
class State(Enum):
    BLOCK = 1,      # awaiting trajectory completion
    BASE_READY = 2, # ready to move the base
    ARM_READY = 3   # ready to move the arm/gripper

class ReplayMotions(Node):
    def __init__(self):
        super().__init__('motion_replayer')

        # buffer asynchronously fills with transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # robot body vars
        self.urdf_path = '/home/hello-robot/cse481/zephyr_ws/src/createmate/createmate/stretch.urdf'
        self.chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path) # joints chained together

        # subscribe to current joint states and store
        self.joint_states = JointState()
        self.join_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
            # /stretch/joint_wrist_state
            #     name:
            # - wrist_extension
            # - joint_lift
            # - joint_arm_l3
            # - joint_arm_l2
            # - joint_arm_l1
            # - joint_arm_l0
            # - joint_head_pan
            # - joint_head_tilt
            # - joint_wrist_yaw
            # - joint_wrist_pitch
            # - joint_wrist_roll
            # - joint_gripper_finger_left
            # - joint_gripper_finger_right
            # position:
            # - 0.10000606469070243
            # - 0.6008078095092801
            # - 0.025001516172675608
            # - 0.025001516172675608
            # - 0.025001516172675608
            # - 0.025001516172675608
            # - -0.09582797005421338
            # - 0.02767317904990556
            # - 0.003834951969714103
            # - -0.015339807878856412
            # - -0.0015339807878856412
            # - 9.358721673485041e-05
            # - 9.358721673485041e-05

        # create action client to move joints with stretch core driver
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        # joint names whose positions are included in the message to the action server:
        self.joint_names = ['translate_mobile_base', 'joint_lift', 'wrist_extension','joint_gripper_finger_left']
        
        # read all the aruco -> grasp center transforms recorded
        self.file_transforms = self.read_pose_file()
        self.get_logger().info('pose file read')
        self.curr_file_tf = 0

        # timer for node to call motion playback
        time_period = 1.0
        self.timer = self.create_timer(time_period, self.playback_motions)
        self.state = State.BASE_READY

    # CALLBACKS FOR EXTERNAL NODE MESSAGES
    def joint_states_callback(self, joint_states):
        '''
          callback to update current joint states from service
        '''
        self.joint_states = joint_states

    def trajectory_server_response(self, future):
        '''
          response from trajectory server (not complete yet, but accepted or denied)
        '''
        goal_resp = future.result()
        self.get_traj_server_res = goal_resp.get_result_async()
        # add another callback awaiting final trajectory result
        # TODO: add check for whether the trajectory was accepted
        self.get_traj_server_res.add_done_callback(self.traj_res_callback)

    def traj_res_callback(self, future):
      '''
        completion of trajectory request
      '''
      res = future.result().result
      self.get_logger().info('trajectory complete')
      self.state = State.ARM_READY

    def transform_to_base_frame(self, recorded_tf):
        '''
            Converts point (in aruco/target_object1 frame) to base_link frame
            record_tf: Transform from aruco_marker to grasp_center
        '''

        rec_pt = PointStamped()
        rec_pt.header.frame_id = 'target_object1'
        rec_pt.point.x =  recorded_tf.translation.x
        rec_pt.point.y =  recorded_tf.translation.y
        rec_pt.point.z =  recorded_tf.translation.z

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
        # target_point = self.read_motion_recording()
        # target_orientation = rpy_matrix(0.0, 0.0, -np.pi/2)
        # pretarget_orientation = rpy_matrix(0.0, 0.0, 0.0)
        # q_mid = self.chain.inverse_kinematics(target_point, pretarget_orientation, orientation_mode='all', initial_position=q_init)
        # q_soln = self.chain.inverse_kinematics(target_point, target_orientation, orientation_mode='all', initial_position=q_mid)
        # self.get_logger().info(f'the chain links: {self.chain.links}')
        q_soln = self.chain.inverse_kinematics(target_point, initial_position=q_init)

        if self.state == State.BASE_READY: # only move the base, keep all other joints at their current position
          q_init[1] = q_soln[1]
          q_soln = q_init

        return q_soln

    # from inverse_kinematics notebook
    def move_to_configuration(self, q, step, gripper_open=True):
        '''
            accepts chain link positions in terms of base_link and move robot to that configuration
            q: chain link positions
            base: boolean indicating whether only the base is being moved
            gripper_open: bolean indicating whether the gripper should be open 
                for this step (true if base_only)
        '''
        #set up trajectory goal
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        trajectory_goal.trajectory.joint_names = self.joint_names

        # create trajectory point
        goal_point = JointTrajectoryPoint()
        duration_goal = Duration(seconds=(2*step)).to_msg()
        goal_point.time_from_start = duration_goal

        if gripper_open:
            goal_point.positions = [0, q[3], q[4] + q[5] + q[6] + q[7] + q[8], 0.16]
        else: 
             goal_point.positions = [0, q[3], q[4] + q[5] + q[6] + q[7] + q[8], 0.22]

        if self.state == State.BASE_READY: # only moving base at the beginning
            goal_point.positions[0] = q[1]

        trajectory_goal.trajectory.points = [goal_point]
        self.get_logger().info(f'trajectory goal to send:{trajectory_goal}')
        # block until joint trajectory done
        self.state = State.BLOCK
        # send joint trajectory
        self._get_result_future = self.trajectory_client.send_goal_async(trajectory_goal)
        self._get_result_future.add_done_callback(self.trajectory_server_response) # block calculating new poses until motion is complete


    def parse_transform_string(self, transform_string):
        # Extracting translation values
        start_index = transform_string.find("x=")
        end_index = transform_string.find(", y=")
        x = float(transform_string[start_index + 2:end_index])

        start_index = transform_string.find("y=")
        end_index = transform_string.find(", z=")
        y = float(transform_string[start_index + 2:end_index])

        start_index = transform_string.find("z=")
        end_index = transform_string.find(")")
        z = float(transform_string[start_index + 2:end_index])

        # Creating and returning the Transform object
        transform = Transform()
        transform.translation=Vector3(x=x, y=y, z=z)
        return transform

    def read_pose_file(self):
        '''
            Reads all transforms recorded in the poses
            file and returns them in a list
        '''
        # Define a regular expression pattern to match the Transform field
        pattern = r'transform=(geometry_msgs.msg.Transform\([^)]+\))'

        # Read the contents of the file
        with open('poses.txt', 'r') as file:
            file_contents = file.read()

        # Find all occurrences of the Transform field in the file contents
        transform_matches = re.findall(pattern, file_contents)

        # Extract and print the Transform fields for manual verification
        # + convert to Transform Dtype
        transforms = []
        for match in transform_matches:
            self.get_logger().info(match)
            transforms.append(self.parse_transform_string(match))
        
        return transforms

    def playback_motions(self):
        '''
            main node entry function to playback 
            motions read from a pose file
        '''
        # only run this code if not awaiting for
        if self.state is not State.BLOCK:
            self.get_logger().info(f'Attempting tf #{self.curr_file_tf}')
            file_tf =  self.file_transforms[self.curr_file_tf]
            target = self.transform_to_base_frame(file_tf)

            # only continue is file point transformed to base_link successfully
            if target is not None:
                self.get_logger().info('Retrieving current joint positions...')
                q_init = self.get_q_init() # shape (15,)
                self.get_logger().info(f'Current joint positions: {q_init}')
                self.get_logger().info('Solving Inverse Kinematics for goal point')

                q = self.get_q_soln(target, q_init)

                self.get_logger().info(f'the solution is: {q}, will now attempt to move to the configuration')

                if (self.curr_file_tf == len(self.file_transforms) - 2):  
                    self.move_to_configuration(q, self.curr_file_tf, gripper_open=False) #close gripper for second to last step
                else:
                    self.move_to_configuration(q, self.curr_file_tf)

                if self.curr_file_tf == len(self.file_transforms) -1:
                    self.get_logger.info("done replaying all poses")
                    rclpy.shutdown()
                else :
                    self.curr_file_tf += 1

def main():
    rclpy.init()
    node = ReplayMotions()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
