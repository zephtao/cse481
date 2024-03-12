import rclpy

from pynput import keyboard
from rclpy.node import Node
from createmate_interfaces.msg import RecordPose

# Node to listen for appropriate keyboard input and save frame transform accordingly
class KeyboardInput(Node):
    def __init__(self):
        super().__init__('keyboard_input')
        self.publisher = self.create_publisher(RecordPose, 'pose_record_req', 10)
        # self.frame_choices = {1:"map", 2:"target_object", 3:"base_link"}
        self.frame_choices = {1:"map", 2:"holder", 3:"base_link"}

    def on_keyboard_press(self, key):
        if key == keyboard.Key.esc:
            msg = RecordPose()
            msg.command = 'quit'
            self.publisher.publish(msg)
            self.get_logger().info('quit message sent...')
            self.destroy_node()
            rclpy.shutdown()

        elif key == keyboard.Key.space:
            msg = RecordPose()
            msg.command = 'record_pose'
            msg.name = input('type pose name:')
            frame = int(input(f'Choose a frame\n{self.frame_choices}'))

            while frame not in self.frame_choices:
                frame=input('incorrect input, try again: ')
            
            msg.frame = self.frame_choices.get(frame)
            self.publisher.publish(msg)

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
