'''
This file is for drawing shapes (circle, triangle, rectangle) with the trajectory client so that
the stretch driver can also run simultaneously with this script.
'''

import sys
import time
import numpy as np
import stretch_body.robot
import stretch_body.trajectories
stretch_body.trajectories.WAYPOINT_ISCLOSE_ATOL = 0.1
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from createmate_interfaces.srv import DrawShape
from createmate_interfaces.msg import Shape
from enum import Enum
from sensor_msgs.msg import JointState
import ikpy.chain
from ikpy.utils.geometry import rpy_matrix
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ros2_numpy import numpify
from geometry_msgs.msg import Transform
from tf2_ros import TransformException
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import PointStamped, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState
import re  # regular expression


class DrawService(Node):
    def __init__(self):
        super().__init__('draw_service')

        # create a separate reentrant callback group so blocking within the shape handling (bc of waiting for actions), 
        # does not block other callbacks
        self.exec_cb_group = ReentrantCallbackGroup()

        self.srv = self.create_service(DrawShape, 'draw_shape', self.draw_shape_callback, callback_group = self.exec_cb_group)

        # shape msg request
        #self.shape = DrawShape.shape
        #self.start_location = DrawShape.start_location 

        # buffer asynchronously fills with transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # robot body vars
        self.urdf_path = '/home/hello-robot/cse481/team2/src/createmate/createmate/stretch.urdf'
        self.chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path) # joints chained together

        # create action client to move joints with stretch core driver
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        # joint names whose positions are included in the message to the action server:
        self.joint_names = ['wrist_extension', 'joint_lift']
        self.join_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)

    # CALLBACKS FOR EXTERNAL NODE MESSAGES
    def joint_states_callback(self, joint_states):
        '''
            callback to update current joint states from service
        '''
        self.joint_states = joint_states
        self.arm_pos = self.joint_states.position[0]
        self.lift_pos = self.joint_states.position[1]

    def draw_circle_trajectory(self, n, diameter=0.1): #0.1
        # get the arm and lift positions (given center of circle)
        # TODO: for steph: replace self.joint_states with start_location from zephyr
        # arm_pos = self.joint_states.position[0] + (diameter / 2)
        # lift_pos = self.joint_states.position[1]
    
        # sample n points along circle
        t = np.linspace(0, 2*np.pi, n, endpoint=True)
        x = (diameter / 2) * np.cos(t) + self.arm_pos #+ (diameter / 2)
        y = (diameter / 2) * np.sin(t) + self.lift_pos
        circle_mat = np.c_[x, y]

        time_dt = 25 / n

        points = []
        for i, pt in enumerate(circle_mat):
            # calculate time stamp
            pt_t = i * time_dt

            # create trajectory point
            point = JointTrajectoryPoint()
            point.positions = [pt[0], pt[1]]
            duration_point = Duration(seconds=pt_t)
            point.time_from_start = duration_point.to_msg()
            
            # add the waypoints to the trajectory
            points.append(point)
        
        self.get_logger().info('sending trajectory')
        prelim_traj = FollowJointTrajectory.Goal()
        #set up trajectory goal
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        trajectory_goal.trajectory.joint_names = self.joint_names
        trajectory_goal.trajectory.points = points
        self.get_logger().info(f'trajectory goal to send:{trajectory_goal}')

        # send joint trajectory
        self._get_result_future = self.trajectory_client.send_goal(trajectory_goal)

    def draw_triangle_position(self, side_len, height):

        # get the arm and lift positions (given center of triangle)
        # TODO: not needed to do this here if calling node deals with moving the arm to the correct pos
        #arm_pos = self.joint_states.position[0] #- (side_len / 2)
        #lift_pos = self.joint_states.position[1] #- (height / 2)

        # Define the triangle's corner points relative to the initial arm and lift positions
        triangle_corners = np.array([
            [self.arm_pos + side_len, self.lift_pos], #start point
            [self.arm_pos + side_len / 2, self.lift_pos + side_len], # middle top
            [self.arm_pos, self.lift_pos] # back to orig corner
        ])

        # Following example from above
        for x, y in triangle_corners:
            # create trajectory point
            point = JointTrajectoryPoint()
            point.positions = [x, y]
            duration_point = Duration(seconds=0)
            point.time_from_start = duration_point.to_msg()

            #set up trajectory goal
            trajectory_goal = FollowJointTrajectory.Goal()
            trajectory_goal.trajectory.header.frame_id = 'base_link'
            trajectory_goal.trajectory.joint_names = self.joint_names
            trajectory_goal.trajectory.points = [point]
            self.get_logger().info(f'trajectory goal to send:{trajectory_goal}')

            # send joint trajectory
            self._get_result_future = self.trajectory_client.send_goal(trajectory_goal)

            time.sleep(2.5)

    def draw_square_trajectory(self, side_len):
        # get the arm and lift positions (given center of square)
        # arm_pos = self.joint_states.position[0] + (side_len / 2)
        # lift_pos = self.joint_states.position[1] + (side_len / 2)

        time_dt = 15

        waypoints = [                                              
            (self.arm_pos, self.lift_pos + side_len),  # Move up 
            (self.arm_pos + side_len, self.lift_pos + side_len),   # Move right
            (self.arm_pos + side_len, self.lift_pos),       # Move down 
            (self.arm_pos, self.lift_pos)                   # Return left back to start (⬜️)
        ]

        points = []

        for i, pt in enumerate(waypoints):
            pt_t = i * time_dt

            # create trajectory point
            point = JointTrajectoryPoint()
            point.positions = [pt[0], pt[1]]
            duration_point = Duration(seconds=pt_t)
            point.time_from_start = duration_point.to_msg()
            
            # add the waypoints to the trajectory
            points.append(point)

         #set up trajectory goal
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.header.frame_id = 'base_link'
        trajectory_goal.trajectory.joint_names = self.joint_names
        trajectory_goal.trajectory.points = points
        self.get_logger().info(f'trajectory goal to send:{trajectory_goal}')

        # send joint trajectory
        self._get_result_future = self.trajectory_client.send_goal(trajectory_goal)   

    # func for shape messages
    def draw_shape_callback(self, request, response):
        if request.shape.shape== 'c':
            self.get_logger().info('received circle request')
            self.draw_circle_trajectory(50) 
        elif request.shape.shape == 't':
            self.get_logger().info('received triangle request')
            self.draw_triangle_position(0.15, 0.15)
        else:
            self.get_logger().info('received square request')
            self.draw_square_trajectory(0.15)
        response.shape_drawn = True
        return response

def main():
  rclpy.init()
  executor = MultiThreadedExecutor()

  draw_node = DrawService()
  executor.add_node(draw_node)

  try:
    executor.spin()
  except KeyboardInterrupt:
    draw_node.get_logger().info('Keyboard interrupt... shutting down')
  
  draw_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
