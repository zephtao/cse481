'''
This file is for drawing shapes (circle, triangle, rectangle) with the trajectory client so that
the stretch driver can also run simultaneously with this script.

use the ActionClient to use the ROS2 Strech libraries
Create an ActionClient for the trajectory - server already exists
(this should just add it to a list of waypoints for a shape)

figure out what joint states are sent as messages
'joint_lift', 'joint_arm_l0', 
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
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from enum import Enum
from sensor_msgs.msg import JointState

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

def __init__(self):
     # create action client to move joints with stretch core driver
    self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
    server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
    if not server_reached:
        self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
        sys.exit()

    # joint names whose positions are included in the message to the action server:
    self.joint_names = ['translate_mobile_base', 'joint_lift', 'wrist_extension','gripper_aperture', 'joint_wrist_pitch']

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
    self.get_logger().info(f'trajectory complete, result: {res}')

def draw_circle_trajectory(n, diameter_m=0.5): #0.2
    t = np.linspace(0, 2*np.pi, n, endpoint=True)
    x = (diameter_m / 2) * np.cos(t) + arm_init
    y = (diameter_m / 2) * np.sin(t) + lift_init
    circle_mat = np.c_[x, y]
    time_dt = 25 / n
    for i in range(n):
        pt = circle_mat[i]
        pt_t = i * time_dt
        self.trajectory_client.send_goal(t_s=pt_t, x_m=pt[0])
        r.lift.trajectory.add(t_s=pt_t, x_m=pt[1])
    r.follow_trajectory()
    time.sleep(n * time_dt + 1.5)

def draw_triangle_trajectory_mode(side_length_m, time_dt=1.5, globalv_m=None, globala_m=None):
    # Calculate height of equilateral triangle
    triangle_height = (side_length_m ** 2 - (side_length_m / 2) ** 2) ** 0.5

    # Define the triangle's corner points relative to the initial arm and lift positions
    triangle_corners = np.array([
        [arm_init, lift_init],                                          # Starting corner (bottom left)
        [arm_init + side_length_m, lift_init],                          # Move right (straight line now)
        [arm_init + side_length_m / 2, lift_init + triangle_height],    # Move to top corner (> shape)
        [arm_init, lift_init]                                           # Return to start (triangle shape)
    ])
    
    # Calculate the time to reach each corner
    # (assume equal time intervals between corners)
    corner_times = np.arange(len(triangle_corners)) * time_dt
    
    # Clear any old trajectory data
    # Not sure if needed but will leave it here
    r.arm.trajectory.clear()
    r.lift.trajectory.clear()
    
    # Following example from above 😅
    for i, (corner, t_s) in enumerate(zip(triangle_corners, corner_times)):
        r.arm.trajectory.add(t_s=t_s, x_m=corner[0], v_m=globalv_m, a_m=globala_m)
        r.lift.trajectory.add(t_s=t_s, x_m=corner[1], v_m=globalv_m, a_m=globala_m)
    
    r.follow_trajectory()
    time.sleep(len(triangle_corners) * time_dt + 0.5)

def draw_square_trajectory_mode(n, side_length_m, time_dt=1.5, globalv_m=None, globala_m=None):
    waypoints = [
        (arm_init, lift_init),                                  # Start at bottom left
        (arm_init + side_length_m, lift_init),                  # Move right (_)
        (arm_init + side_length_m, lift_init + side_length_m),  # Move up (_|)
        (arm_init, lift_init + side_length_m),                  # Move left (])
        (arm_init, lift_init)                                   # Return to start (⬜️)
    ]

    # ???
    r.arm.trajectory.clear()
    r.lift.trajectory.clear()

    for i, (x, y) in enumerate(waypoints):
        pt_t = i * time_dt
        r.arm.trajectory.add(t_s=pt_t, x_m=x, v_m=globalv_m, a_m=globala_m)
        r.lift.trajectory.add(t_s=pt_t, x_m=y, v_m=globalv_m, a_m=globala_m)
    r.follow_trajectory()
    time.sleep(n * time_dt + 0.5)

