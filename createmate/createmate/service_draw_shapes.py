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
# TODO: add import for Shape message
from enum import Enum
from sensor_msgs.msg import JointState

class DrawService(Node):
    def __init__(self):
        super().__init__('draw_service')
        self.srv = self.create_service(DrawShape, 'draw_shape', self.draw_shape_callback)

        # shape msg request
        self.shape = DrawShape.shape
        self.start_location = DrawShape.start_location

        # create action client to move joints with stretch core driver
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        # joint names whose positions are included in the message to the action server:
        self.joint_names = ['joint_lift', 'wrist_extension']

        # TODO: do we need this? from poses_to_motion: timer for node to call motion playback
        time_period = 1.0
        self.timer = self.create_timer(time_period, self.playback_motions)

    # func for shape messages
    def draw_shape_callback(self):
        if self.shape == 'c':
            draw_circle_trajectory(50)
        elif self.shape == 't':
            draw_triangle_position_mode(0.15)
        else:
            draw_square_trajectory_mode(0.15)

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
        self.get_traj_server_res = goal_resp.get_result_async() # TODO: maybe not async
        # add another callback awaiting final trajectory result
        # TODO: add check for whether the trajectory was accepted
        self.get_traj_server_res.add_done_callback(self.traj_res_callback)

    def traj_res_callback(self, future):
        '''
        completion of trajectory request
        '''
        res = future.result().result
        self.get_logger().info(f'trajectory complete, result: {res}')