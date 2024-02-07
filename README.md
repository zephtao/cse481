# cse481

Every time you run code that moves the joints, you'll have to run this command:\
`stretch_robot_home.py`

To launch the driver:\
`ros2 launch stretch_core stretch_driver.launch.py`

To start the web socket on the robot, use this command:\
`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

Look in [JOINT_LIMITS](https://github.com/hello-robot/stretch_web_teleop/blob/bc7985cc9838c9bc3d40b631e328acb1a62e855a/src/shared/util.tsx#L64) for the complete list of joint_names and the min/max values allowed. You can add them to joint_names in the moveLift function in App.js.