import rclpy
from rclpy.node import Node

class PickupMarker(Node):
  'ros code to pickup a marker and return to '
  def __init__(self):
  super().__init__('marker_pickup')
  self.srv = self.create_service(PickupMarker, )
  
