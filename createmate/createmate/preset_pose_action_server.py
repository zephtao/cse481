import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

'''
    Read from preset pose file and send stretch robot to pose
'''
class SendToPresetPose(Node):
    