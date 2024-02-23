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

- .spin_until_future_complete: not sure someone fill this in when you figure it out

Callbacks:
- all asynchronous, however I think they are guaranteed to run uninterrupted until completion
- I BELIEVE the executor will just queue up the callbacks
- Strategy I've been using so far: setting a boolean/state variable that I check in a callback to see if I actually 
    want to run the callback code

Sending arm navigation requests:
 - Use follow joint trajectory to send position requests to the stretch driver action server: https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/follow_joint_trajectory/
 - 

