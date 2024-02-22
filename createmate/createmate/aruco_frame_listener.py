import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):

    def __init__(self):
        super().__init__('stretch_tf_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    

        self.pose_subscriber = self.create_subscription(String, 'frame_transform_req', self.record_transform, 10)
        self.posefile = open('/home/hello-robot/cse481/zephyr_ws/poses.txt', 'w')
        
    def record_transform(self, msg):
        from_frame_rel = 'target_object1'
        target_frame = 'link_grasp_center' # holds all our desired frame
        if msg.data == 'record_pose':
            self.get_logger().info('Received request to record transform')
            try:
                now = Time()
                trans = self.tf_buffer.lookup_transform(
                    target_frame,
                    from_frame_rel,
                    now)
                self.posefile.write(f'{trans}\n')
                self.get_logger().info(f'{trans}')
            except TransformException as ex:
                self.get_logger().info(
                        f'Could not transform {target_frame} to {from_frame_rel}: {ex}')
        elif msg.data == 'quit':
            self.get_logger().info(f'Got quit message... writing all requested transforms to poses.txt file')
            self.posefile.close()
            self.destroy_node()
            rclpy.shutdown()
            
def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()