# dwa_planner/goal_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)
        # default goal (can be changed)
        self.goal = (1.5, 1.5)
        self.get_logger().info('Goal publisher started; publishing to /goal_pose')

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self.goal[0])
        msg.pose.position.y = float(self.goal[1])
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
