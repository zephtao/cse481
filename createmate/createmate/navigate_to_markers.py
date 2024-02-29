from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, PoseStamped
from geometry_msgs.msg import Point, Quaternion
from rclpy.time import Time
import csv

# topic for initial pose "/setInitialPose"
#   type /geometry_msgs/msg/PoseWithCovarianceStamped

class Nav2Markers(Node):
    def _init_(self):
      super()._init_('nav2marker')

      # subscribe to the initial pose topic
      self.init_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'setInitialPose', self.define_init_pose, 10)
      self.init_pose = PointStamped()
      self.init_pose.header.frame_id = 'map'
      self.initial_pose_set = False

      # read goal pose
      self.goal = PoseStamped()
      self.goal.header.frame_id = 'map'
      with open('goal_pose.csv', mode='r') as goal_f:
        pose_data = csv.reader(file, delimiter = ',')
      
      #assuming csv data stored in this format
      self.goal.pose.position = Point(x=pose_data[0], y=pose_data[1], z=pose_data[2])
      self.goal.pose.orientation = Quaternion(x=pose_data[3], y=pose_data[4], z=pose_data[5], w=pose_data[6] )

      # action client to navigate to end pose:
      self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
      # self.compute_path_to_pose_client = ActionClient(
      #       self, ComputePathToPose, 'compute_path_to_pose'
      #   )

    def nav_to_goal(self):
      while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")
      goal_msg = NavigateToPose.Goal()
      goal_msg.pose = pose
      goal_msg.b


    def define_init_pose(self, pose_with_covar):
      self.init_pose.header.stamp = self.nav.get_clock().now().to_msg()
      self.init_pose = pose_with_covar.pose # define the initial pose
      self.initial_pose_set = True

    def wait_for_init_pose(self):
      while not self.initial_pose_set:
        rclpy.spin_once(self, timeout_sec=0.5)

      self.get_logger().info(f'received the initial start pose! Will now try to navigate to this goal pose: {self.goal}')
      


def main():
  # initialize node
  rclpy.init()
  navigate_to_markers = Nav2Markers()


  # wait for initial pose to be sent
  self.get_logger().info('Send an initial pose estimate through the /setInitialPose topic (can be done on rviz)')
  navigate_to_markers.wait_for_init_pose()
  navigate_to_markers.nav_to_goal()