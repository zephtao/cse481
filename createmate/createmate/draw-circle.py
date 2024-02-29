# Modified from https://gist.github.com/hello-binit/8c46cfb773a8a0255b4703a09808c26e
import sys
import time
import numpy as np
import stretch_body.robot
import stretch_body.trajectories
stretch_body.trajectories.WAYPOINT_ISCLOSE_ATOL = 0.1


r = stretch_body.robot.Robot()
r.arm.motor.disable_guarded_mode()
r.lift.motor.disable_guarded_mode()
r.startup()
if not r.is_calibrated():
    print('Home the robot')
    sys.exit(1)
arm_init = 0.1
lift_init = 1.0


def setup_robot():
    r.arm.move_to(arm_init)
    r.lift.move_to(lift_init)
    r.push_command()
    r.end_of_arm.move_to('wrist_yaw', 1.5707)
    r.end_of_arm.move_to('stretch_gripper', 100)
    time.sleep(5)
    input('Press enter to close the gripper')
    r.end_of_arm.move_to('stretch_gripper', -100)
    r.base.left_wheel.gains['enable_vel_watchdog'] = 0
    r.base.left_wheel._dirty_gains = 1
    r.base.right_wheel.gains['enable_vel_watchdog'] = 0
    r.base.right_wheel._dirty_gains = 1
    r.push_command()
    time.sleep(3)
    input('Press enter to command base towards whiteboard at 1cm/sec. NOTE: base is blind, you should be ready to press enter again to stop the base')
    r.base.set_velocity(0.01, 0.0)
    r.push_command()
    input('Press enter to stop the base')
    r.base.enable_freewheel_mode()
    r.push_command()

def draw_circle_position_mode(n, diameter_m=0.2):
    t = np.linspace(0, 2*np.pi, n, endpoint=True)
    x = (diameter_m / 2) * np.cos(t) + arm_init
    y = (diameter_m / 2) * np.sin(t) + lift_init
    circle_mat = np.c_[x, y]
    for pt in circle_mat:
        r.arm.move_to(pt[0])
        r.lift.move_to(pt[1])
        r.push_command()
        time.sleep(1.5)

# def draw_circle_trajectory_mode(n, diameter_m=0.2, time_dt=1.5, globalv_m=None, globala_m=None):
#     t = np.linspace(0, 2*np.pi, n, endpoint=True)
#     x = (diameter_m / 2) * np.cos(t) + arm_init
#     y = (diameter_m / 2) * np.sin(t) + lift_init
#     circle_mat = np.c_[x, y]
#     for i in range(n):
#         pt = circle_mat[i]
#         pt_t = i * time_dt
#         r.arm.trajectory.add(t_s=pt_t, x_m=pt[0], v_m=globalv_m, a_m=globala_m)
#         r.lift.trajectory.add(t_s=pt_t, x_m=pt[1], v_m=globalv_m, a_m=globala_m)
#     r.follow_trajectory()
#     time.sleep(n * time_dt + 0.5)
        
def draw_circle_trajectory(n, diameter_m=0.2):
    t = np.linspace(0, 2*np.pi, n, endpoint=True)
    x = (diameter_m / 2) * np.cos(t) + arm_init
    y = (diameter_m / 2) * np.sin(t) + lift_init
    circle_mat = np.c_[x, y]
    time_dt = 25 / n
    for i in range(n):
        pt = circle_mat[i]
        pt_t = i * time_dt
        r.arm.trajectory.add(t_s=pt_t, x_m=pt[0])
        r.lift.trajectory.add(t_s=pt_t, x_m=pt[1])
    r.follow_trajectory()
    time.sleep(n * time_dt + 1.5)



# NEW CODE DOWN HERE!!!!

# Run draw_triangle_trajectory_mode(0.5, time_dt=1.0) 
# to draw a triangle with 0.5m sides (each side taking 1 second to traverse)
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

def draw_triangle_position_mode(side_length_m):
    # Calculate height of equilateral triangle
    triangle_height = (side_length_m ** 2 - (side_length_m / 2) ** 2) ** 0.5

    # Get (x1, y1) (x2, y2) (x3, y3) to represent a triangle
    # Define the triangle's corner points relative to the initial arm and lift positions
    triangle_corners = np.array([
        [arm_init + side_length_m, lift_init],                          # Move right (straight line now)
        [arm_init + side_length_m / 2, lift_init + triangle_height],    # Move to top corner (> shape)
        [arm_init, lift_init]                                           # Return to start (triangle shape)
    ])
    
    # Following example from above 😅
    for x, y in triangle_corners:
        r.arm.move_to(x)
        r.push_command()
        r.lift.move_to(y)
        r.push_command()
        time.sleep(1.5)

# Run draw_square_trajectory_mode(0.5, time_dt=1.0) 
# to draw a square with 0.5m sides (each side taking 1 second to traverse)
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


if __name__ == "__main__":
    yn = input('setup robot? (y/[n]): ')
    if yn == 'y':
        setup_robot()
    # draw_circle_position_mode(10, 1.5)
    draw_circle_trajectory(50)
    # draw_triangle_trajectory_mode(0.5)
    # draw_triangle_position_mode(2)
    # draw_square_trajectory_mode(0.5, 0.5, time_dt=1.0)
