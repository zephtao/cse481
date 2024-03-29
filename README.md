# cse481

When we want to use our workspace on the robot, run this command first:\
`source install/setup.bash`

Every time you run code that moves the joints, you'll have to run this command:\
`stretch_robot_home.py`\
If you get an error saying "cannot home while run_stopped", you need to push the button on the side top of the robot until it beeps to unlock the run_stop.

To launch the driver:\
`ros2 launch stretch_core stretch_driver.launch.py mode:=position`\
`mode` can be set to `navigation`, `position`, or `trajectory`\
Set it to `position` for keyboard teleoperating the robot.\
The driver should be running when you want to programatically control the robot through ROS.\
The driver may prevent other things from running, like homing the robot.

To start the web socket on the robot, use this command:\
`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

Look in [JOINT_LIMITS](https://github.com/hello-robot/stretch_web_teleop/blob/bc7985cc9838c9bc3d40b631e328acb1a62e855a/src/shared/util.tsx#L64) for the complete list of joint_names and the min/max values allowed. You can add them to joint_names in the moveLift function in App.js.

To see all packages:\
`ros2 pkg list`

Order of running commands when using web interface to control robot:
1. `stretch_robot_home.py`
2. `ros2 launch stretch_core stretch_driver.launch.py`
3. `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
