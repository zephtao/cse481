from pynput import keyboard

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Node to listen for appropriate keyboard input and save frame transform accordingly
class KeyboardInput(Node):
    def __init__(self):
        super().__init__('keyboard_input')
        self.publisher = self.create_publisher(String, 'frame_transform_req', 10) # demo_commands topic with queue size of 10

    def publish_message(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)

    def on_keyboard_press(self, key):
        if key == keyboard.Key.esc:
            self.publish_message('quit')
            self.get_logger().info('quit message sent...')
            self.destroy_node()
            rclpy.shutdown()
        if key == keyboard.Key.space:

            self.publish_message('record_pose')
            self.get_logger().info('record_pose message sent...')

    def get_keyboard_input(self):
        with keyboard.Listener(on_press=self.on_keyboard_press) as listener: # onkeyboard press is a callback function
            listener.join()

def main():
    rclpy.init()
    node = KeyboardInput()
    node.get_keyboard_input()

if __name__ == '__main__':
    main()

# geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1708400927, nanosec=403441895), frame_id='link_grasp_center'), child_frame_id='target_object1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.5372364620953266, y=-0.22509781296434073, z=0.24617252052644534), rotation=geometry_msgs.msg.Quaternion(x=0.04452516070929938, y=0.03900741571137317, z=-0.2495200768241972, w=0.9665586701514699)))
# geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1708400940, nanosec=681592773), frame_id='link_grasp_center'), child_frame_id='target_object1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.31434450884893206, y=-0.014661480732279075, z=0.3403597768549247), rotation=geometry_msgs.msg.Quaternion(x=0.7130356974202358, y=0.003661715498180148, z=-0.6911579283474606, w=-0.11775994279250879)))
# geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1708400956, nanosec=90192627), frame_id='link_grasp_center'), child_frame_id='target_object1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.3134130916611446, y=-0.0146675722974699, z=0.34128054052590384), rotation=geometry_msgs.msg.Quaternion(x=0.7096565635922275, y=0.009792230278915477, z=-0.6951402156878204, w=-0.114331773843172)))
# geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1708400993, nanosec=850075684), frame_id='link_grasp_center'), child_frame_id='target_object1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.2865763842636059, y=-0.015695035822294368, z=0.3422455873660746), rotation=geometry_msgs.msg.Quaternion(x=0.7048917279966858, y=0.03206055162495163, z=-0.702538481526339, w=-0.0924091705731414)))
# geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1708401010, nanosec=928606689), frame_id='link_grasp_center'), child_frame_id='target_object1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.2879905802106184, y=-0.019189334169116223, z=0.4410683153758024), rotation=geometry_msgs.msg.Quaternion(x=-0.6974975078027082, y=-0.033772470337488286, z=0.7105814696353129, w=0.08620105490789105)))
