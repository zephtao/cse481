# ros code usage guide

To run a node:
1. Add your node to the setup.py file
  - alter the entry_points, 'console_scripts' entry
    script_name=createmate.[python script name]:main

How ROS Nodes run:
- use rclpy .spin, .spin_once, or .spin_until_future_complete

- .spin: does not terminate, work will be done through callbacks
- .spin_once: will check all unhandled callbacks and then you can do some non-callback work
      - don't do too much between spin_once calls
      could look like:
        while(condition):
          spin_once
          method_call_to_do_some_work()

- .spin_until_future_complete: not sure someone fill this in when you figure it out. Will stop
  spinning the node when the "future completes". Not currently fully sure of how to define said future.

Callbacks:
- all asynchronous, however I think they are guaranteed to run uninterrupted until completion
- I BELIEVE the executor will just queue up the callbacks
- Strategy I've been using so far: setting a boolean/state variable that I check in a callback to see if I actually 
    want to run the callback code. For example, I didn't want to request a new joint movement until the previous one
    was done, so I set a flag to keep track of this.

URDF
The URDF we are using contains the following chain links:
    Kinematic chain: name=chain links=['Base link', 'joint_base_translation', 'joint_mast',
    'joint_lift', 'joint_arm_l4', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0', 
    'joint_wrist_yaw', 'joint_wrist_yaw_bottom', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_straight_gripper',
    'joint_grasp_center'] active_links=[ True  True  True  True  True  True  True  True  True  True  True  True True  True  True]

# stretch core
Sending arm navigation requests:
 - Use follow joint trajectory to send position requests to the stretch driver action server: https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/follow_joint_trajectory/
 - look at poses_to_motion_control.py for example
 - to figure out what joint_names you can pass to this, look in this code: https://github.com/hello-robot/stretch_ros2/blob/humble/stretch_core/stretch_core/command_groups.py
 ^ I could not find documentation or a better strategy for figuring this out... lmk if you do. The code looks at commanded_joint_names, so you can ctrl-f that to figure out what it looks for in there.
  - you can also play around w/ the robot and send the action server messages via CLI to see what works

viewing current joint states:
- subscribe node to the /stretch/joint_states topic.
- on CLI: view the contents of the topic with ros2 topic echo /stretch/joint_states

The order of joints in this topic is:
- wrist_extension
- joint_lift
- joint_arm_l3
- joint_arm_l2
- joint_arm_l1
- joint_arm_l0
- joint_head_pan
- joint_head_tilt
- joint_wrist_yaw
- joint_wrist_pitch
- joint_wrist_roll
- joint_gripper_finger_left
- joint_gripper_finger_right (Note: only use finger left or right in the JointTrajectory requests. either will result in the movement of both)
